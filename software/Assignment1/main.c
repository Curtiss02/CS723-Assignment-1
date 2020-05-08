#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "global.h"
#include "vga_output.h"

//system includes
#include <altera_avalon_pio_regs.h>
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"




// Defines
#define PARAM_INITIAL_INSTANT_THRES 	50.0
#define PARAM_INITIAL_RATE_THRES 		999.0

#define PARAM_UNSTABLE_DELAY			500 // shed subsequent loads after 500ms
#define PARAM_STABLE_DELAY				500 // re-introduce loads after 500ms

#define PARAM_NUM_LOADS					5 	// number of loads (switches) in use


// Definition of Task Stacks
#define   TASK_STACKSIZE       2048


// Definition of Task Priorities

#define PRIORITY_SWITCHPOLLING 11
#define PRIORITY_LOADCONTROL 12
#define PRIORITY_UPDATEVGA 9
#define PRIORITY_STABILITYMONITOR 13


/* Custom Types */
typedef struct params {
	float instant_threshold;
	float rate_threshold;
} t_params;


// Function Prototypes
void ISR_FreqReceived(void);
void ISR_KeyInputReceived(void);
void ISR_EnableMaintenanceMode(void);

void TaskStabilityMonitor();
void TaskLoadControl();
void TaskSwitchPolling();
void TaskUpdateVGA();

void HelperLoadDisconnect(void);
void HelperLoadReconnect(void);
bool HelperLoadsManaged(void);
void HelperAddFrequencyReceived(float frequency);
void testError(BaseType_t xR, char* msg);

void initDataStructs(void);
void initTasks(void);


// Tick defines
const TickType_t taskStabilityMonitorDelay = 10 / portTICK_PERIOD_MS; // Allows for 100Hz maximum monitoring
const TickType_t taskLoadControl = 50 / portTICK_PERIOD_MS; // Reasonable enough switch delay - reduces processor load
const TickType_t taskSwitchPolling = 100 / portTICK_PERIOD_MS; // 100ms switch polling delay
const TickType_t taskUpdateVGA = 16 / portTICK_PERIOD_MS; // Approx. 60Hz refresh rate


/* Queues */
// Both are mailboxes (queues with max length of 1)
QueueHandle_t qFreqReceived;
QueueHandle_t qLoadControl;


/* Semaphores */
SemaphoreHandle_t xFrequencies;


/* Global vars */

t_params gParams;
bool gLoadShedding;

bool gManagingLoads;
bool gSystemStatus[PARAM_NUM_LOADS];
bool gSystemStatusEntry[PARAM_NUM_LOADS];


float gHistoryROC[FREQ_ARR_SIZE] = {0};
float gHistoryFreq[FREQ_ARR_SIZE] = {0};
int gHistoryStartPos;

TickType_t gStableInitialTick;
TickType_t gUnstableInitialTick;







void ISR_FreqReceived()
{
	#define SAMPLING_FREQ 16000.0
	float hz= SAMPLING_FREQ/(float)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR(qFreqReceived, &hz, pdFALSE );
}

void ISR_KeyInputReceived()
{
	// TODO Set params.instant_threshold and params.rate_threshold
}

void ISR_EnableMaintenanceMode()
{
	  int keyval = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) & 0x4;
	  if(keyval){
		  	gLoadShedding = false;
			gManagingLoads = !gManagingLoads;
	  }
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);


	// TODO Is waiting 100ms for the system status to
	// return to switch state alright??
}



/* Tasks */

// A bit monolithic, but I'll have to think about how I would chunk it down
// It doesn't accomplish too much.
void TaskStabilityMonitor()
{
	float freq;

	while (1)
	{
		xQueueReceive(qFreqReceived, &freq, portMAX_DELAY);
		HelperAddFrequencyReceived(freq);

		if(gManagingLoads)
		{
				// Ensure atomic read by reading to a local var
				//Step back history pointer by 1 to get newest value
				int _historyEndPos;
				if(gHistoryStartPos == 0){
					 _historyEndPos = FREQ_ARR_SIZE -1;
				}
				else{
					 _historyEndPos = gHistoryStartPos - 1;
				}
				float _instant = gHistoryFreq[_historyEndPos];
				float _rate = fabs(gHistoryROC[_historyEndPos]);

/*
				printf("TASK :: TaskStabilityMonitor Received Message\n");
				fflush(stdout);
				printf("     :: index 0: %f, index1: %f\n", _instant, _rate);
				fflush(stdout);*/

				if(_instant < gParams.instant_threshold || _rate > gParams.rate_threshold)
				{
					// If not running already, start unstable timer on unstable behavior
/*
					printf("     :: THRESHOLD TRIGGERED \n");
					fflush(stdout);
*/

					// If moving into load-shedding mode
					// Set up variables and instantly shed first load
					if(!gLoadShedding)
					{
						int i;
						for(i = 0; i < PARAM_NUM_LOADS; i++) {

							gSystemStatusEntry[i] = gSystemStatus[i];
						}

						gLoadShedding = true;
/*						printf("--------------------------------------------------------------------------------------------\n");*/
						HelperLoadDisconnect();
					}

					gStableInitialTick = 0;

					if(gUnstableInitialTick == 0)
						gUnstableInitialTick = xTaskGetTickCount();
				}
				else
				{
					// Otherwise stop unstable timer
					if(gStableInitialTick == 0)
						gStableInitialTick = xTaskGetTickCount();

					gUnstableInitialTick = 0;
				}

				// Act on timers
				if(gUnstableInitialTick) {
					int tickTimer = (xTaskGetTickCount()-gUnstableInitialTick)*portTICK_PERIOD_MS;

					if(tickTimer >= PARAM_UNSTABLE_DELAY) {
/*						printf(">>>>>>>>>>> UNSTABLE :: %i <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", tickTimer);
						fflush(stdout);*/

						// Do subsequent load shedding
						HelperLoadDisconnect();

						bool msg = true;
						xQueueSend(qLoadControl, (void *)&msg, 0);

						gUnstableInitialTick = 0;
					}
				}
				if(gStableInitialTick) {
					int tickTimer = (xTaskGetTickCount()-gStableInitialTick)*portTICK_PERIOD_MS;

					if(tickTimer >= PARAM_STABLE_DELAY) {
/*						printf(">>>>>>>>>>> STABLE :: %i   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", tickTimer);
						fflush(stdout);*/

						// Do re-introduction of load
						HelperLoadReconnect();

						bool msg = true;
						xQueueSend(qLoadControl, (void *)&msg, 0);

						gStableInitialTick = 0;
					}
				}
			
		}

		vTaskDelay(taskStabilityMonitorDelay);
	}
}

void TaskLoadControl()
{
	bool *msg;
	while (1)
	{
		xQueueReceive(qLoadControl, &msg, portMAX_DELAY);

		if((bool)msg == true)
		{
/*			printf("TASK :: TaskLoadControl Received Message\n");
			fflush(stdout);
			printf("     :: index 0: %f, index1: %f\n", gHistoryFreq[gHistoryStartPos], gHistoryROC[gHistoryStartPos]);
			fflush(stdout);*/

			int i;
			int _systemStatus = 0;
			int _systemStatusEntry = 0;

			for(i = 0; i < PARAM_NUM_LOADS; i++){
				_systemStatus |=  ((int)gSystemStatus[i]) << i;
				_systemStatusEntry |= ((int)gSystemStatusEntry[i]) << i;
			}
			


			if(!gLoadShedding){
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, _systemStatus);
			}
			else{
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, _systemStatus);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, _systemStatusEntry & (~(_systemStatus)));
			}

			
			// TODO Output gSystemSwitchStatus to LEDs


		}
		vTaskDelay(taskLoadControl);
	}
}

void TaskSwitchPolling()
{
	while (1) {
		// TODO Functionality
		// Assign load switch status from hardware
		bool _loadSwitchStatus[PARAM_NUM_LOADS];
		int _switchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;


		int i = 0;
		

		for(i = 0; i < PARAM_NUM_LOADS; i++){
			_loadSwitchStatus[i]= (bool) (_switchValue & (1 << i));
		}

		// If currently in load shedding mode

		if(gLoadShedding) {
			for(i = 0; i < PARAM_NUM_LOADS; i++) {
				// Disable load, and ensure it won't be re-enabled by `HelperLoadReconnect()`
				gSystemStatus[i] = (gSystemStatus[i] && _loadSwitchStatus[i]);
				gSystemStatusEntry[i] = (gSystemStatusEntry[i] && _loadSwitchStatus[i]);
			}
		} else {
			for(i = 0; i < PARAM_NUM_LOADS; i++) {
				// Otherwise just update the status of the load
				gSystemStatus[i] = _loadSwitchStatus[i];
			}
		}



		// Run load control task for every switch poll update
		//
		// TODO Discuss whether we want this. Current reasoning is it is probably the
		// cheapest computational solution (rather than looking for changes in switch state)
		bool msg = true;
		xQueueSend(qLoadControl, (void *)&msg, 0);

		vTaskDelay(taskSwitchPolling);
	}
}

void TaskUpdateVGA()
{


	while (1)
	{

		drawSystemUptime(xTaskGetTickCount()/1000);
		drawGraphs(gHistoryFreq, gHistoryROC, gHistoryStartPos);
		vTaskDelay(taskUpdateVGA);

	}
}

//
// Helper functions
//
void HelperLoadDisconnect()
{
	// Disable the lowest priority load
	int i;
	for(i = 0; i < PARAM_NUM_LOADS; i++)
	{
		if(gSystemStatus[i])
		{
			gSystemStatus[i] = false;
			return;
		}
	}
}

void HelperLoadReconnect()
{
	// Enable the highest priority load
	int i;
	for(i = PARAM_NUM_LOADS - 1; i >= 0; i--)
	{
		// If load currently disabled, but was enabled before load shedding
		if(!gSystemStatus[i] && gSystemStatusEntry[i])
		{
			gSystemStatus[i] = true;

			// If all loads have been re-enabled, set flag to false
			if(!HelperLoadsManaged()) {
				gLoadShedding = false;
			}
			return;
		}
	}
}

bool HelperLoadsManaged()
{
	// Return false if vectors gSystemStatus and gSystemStatusEntry
	// are not identical
	int i;
	for(i = 0; i < PARAM_NUM_LOADS; i++)
	{
		if(gSystemStatus[i] != gSystemStatusEntry[i])
			return true;
	}
	return false;
}

void HelperAddFrequencyReceived(float frequency){
	gHistoryFreq[gHistoryStartPos] = frequency;
	float denominator;
		if(gHistoryStartPos==0){
			denominator = (gHistoryFreq[0]+gHistoryFreq[99]);
			if(denominator == 0){
				gHistoryROC[0] = 0;
			}
			else{
				gHistoryROC[0] = (gHistoryFreq[0]-gHistoryFreq[99]) * 2.0 * gHistoryFreq[0] * gHistoryFreq[99] / denominator;
			}

		}
		else{
			denominator = (gHistoryFreq[gHistoryStartPos]+gHistoryFreq[gHistoryStartPos-1]);
			if(denominator == 0){
				gHistoryROC[gHistoryStartPos] = 0;
			}else{
				gHistoryROC[gHistoryStartPos] = (gHistoryFreq[gHistoryStartPos]-gHistoryFreq[gHistoryStartPos-1]) * 2.0 * gHistoryFreq[gHistoryStartPos]* gHistoryFreq[gHistoryStartPos-1] / denominator;
			}

		}
		gHistoryStartPos = (gHistoryStartPos + 1) % FREQ_ARR_SIZE;
}

void testError(BaseType_t xR, char* msg)
{
	if(xR == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	{
		fprintf(stderr, "Could not allocate required memory for %s", msg);
		exit(1);
	}
}

void initKeyboard(){
	  alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	  if(ps2_device == NULL){
	    printf("can't find PS/2 device\n");
	  }

	  alt_up_ps2_clear_fifo (ps2_device) ;

	  alt_irq_register(PS2_IRQ, ps2_device, ISR_KeyInputReceived);
	  // register the PS/2 interrupt
	  IOWR_8DIRECT(PS2_BASE,4,1);
}

void initPushButtonISR(){
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Activate interupt for only BTN_3
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ, NULL , ISR_EnableMaintenanceMode);
}

// This function creates the queues and semaphores
void initDataStructs(void)
{
	gParams.instant_threshold = PARAM_INITIAL_INSTANT_THRES;
	gParams.rate_threshold = PARAM_INITIAL_RATE_THRES;

	gLoadShedding = false;

	// Initialize load switch and load control vectors
	int i;
	for(i=0; i < PARAM_NUM_LOADS; i++) {
		gSystemStatus[i] = false;
		gSystemStatusEntry[i] = false;
	}

	gManagingLoads = false;

	gStableInitialTick = 0;
	gUnstableInitialTick = 0;

	gHistoryStartPos = 0;

	/* Semaphores */
	xFrequencies = xSemaphoreCreateBinary();
	if (xFrequencies == 0)
		fputs("Could not create semaphores xFrequencies", stderr);

	/* Queues */
	// Creating a queue of depth 1 for storing a boolean (update flag)
	qFreqReceived = xQueueCreate(1, sizeof(float));
	if (qFreqReceived == 0)
		fputs("Could not create queue qFreqReceived", stderr);

	qLoadControl = xQueueCreate(1, sizeof(bool));
	if (qLoadControl == 0)
		fputs("Could not create queue qLoadControl", stderr);
}


// This function creates the tasks
void initTasks(void)
{
	BaseType_t xR;


	TaskHandle_t hTaskStabilityMonitor;
	xR = xTaskCreate(TaskStabilityMonitor,"TSM",TASK_STACKSIZE,NULL,PRIORITY_STABILITYMONITOR,&hTaskStabilityMonitor);
	testError(xR, "TSM");

	TaskHandle_t hTaskLoadControl;
	xR = xTaskCreate(TaskLoadControl,"TLC",TASK_STACKSIZE,NULL,PRIORITY_LOADCONTROL,&hTaskLoadControl);
	testError(xR, "TLC");

	TaskHandle_t hTaskSwitchPolling;
	xR = xTaskCreate(TaskSwitchPolling,"TSP",TASK_STACKSIZE,NULL,PRIORITY_SWITCHPOLLING,&hTaskSwitchPolling);
	testError(xR, "TSP");


	TaskHandle_t hTaskUpdateVGA;
	xR = xTaskCreate(TaskUpdateVGA,"TUV",TASK_STACKSIZE,NULL,PRIORITY_UPDATEVGA,&hTaskUpdateVGA);
	testError(xR, "TUV");

}

void initISR(){
	//register frequency analyser ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, ISR_FreqReceived);
	initKeyboard();
	initPushButtonISR();

}

int main(int argc, char **argv)
{


	initVGA();
	initDataStructs();
	initTasks();
	initISR();
	// Starting the scheduler
	vTaskStartScheduler();

	// In case if the scheduler failed to start.
	for(;;);
}

