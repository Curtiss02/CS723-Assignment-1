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
#include <altera_avalon_timer.h>




// Defines


#define TIMESTAMP_FREQ 100000000
#define TIMESTAMP_TO_US_DIV 100

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048


// Definition of Task Priorities

#define PRIORITY_SWITCHPOLLING 11
#define PRIORITY_LOADCONTROL 12
#define PRIORITY_UPDATEVGA 9
#define PRIORITY_STABILITYMONITOR 13
#define PRIORITY_UPDATETHRESHOLDS 10


/* Custom Types */
typedef struct params {
	float instant_threshold;
	float rate_threshold;
} t_params;

//Stored in micro seconds
typedef struct timing_params{
	int max;
	int min;
	int avg;
	int history[5];
	int history_pos;
	long sum;
} timing_info;


// Function Prototypes
void ISRFreqReceived(void* context, alt_u32 id);
void ISRKeyInputReceived(void* context, alt_u32 id);
void ISREnableMaintenanceMode(void* context, alt_u32 id);

void TaskStabilityMonitor();
void TaskLoadControl();
void TaskSwitchPolling();
void TaskUpdateVGA();

void HelperLoadDisconnect(void);
void HelperLoadReconnect(void);
bool HelperLoadsManaged(void);
void HelperAddFrequencyReceived(float frequency);
bool HelperIsDigit(char c);
int HelperCharToInt(char c);
void HelperAddNewTiming(int timing);
void testError(BaseType_t xR, char* msg);

void initDataStructs(void);
void initTasks(void);


// Tick defines
const TickType_t taskStabilityMonitorDelay = 10 / portTICK_PERIOD_MS; // Allows for 100Hz maximum monitoring
const TickType_t taskLoadControl = 10 / portTICK_PERIOD_MS; // Reasonable enough switch delay - reduces processor load
const TickType_t taskSwitchPolling = 100 / portTICK_PERIOD_MS; // 100ms switch polling delay
const TickType_t taskUpdateVGA = 16 / portTICK_PERIOD_MS; // Approx. 60Hz refresh rate
const TickType_t taskUpdateThresholds =  10 / portTICK_PERIOD_MS;


/* Queues */

QueueHandle_t qFreqReceived;
QueueHandle_t qLoadControl;

QueueHandle_t qCharReceived;

/* Semaphores */
SemaphoreHandle_t xSystemStatus;


/* Global vars */
timing_info gTimingInfo;
t_params gParams;
bool gLoadShedding;

bool gManagingLoads;
bool gSystemStatus[PARAM_NUM_LOADS];
bool gSystemStatusEntry[PARAM_NUM_LOADS];


float gHistoryROC[FREQ_ARR_SIZE] = {0};
float gHistoryFreq[FREQ_ARR_SIZE] = {0};
int gHistoryStartPos;



bool gKeyDown;
char gLastKeyPressed;

bool gLoadShedTimer = false;
TickType_t gStableInitialTick;
TickType_t gUnstableInitialTick;







void ISRFreqReceived(void* context, alt_u32 id)
{
	#define SAMPLING_FREQ 16000.0
	float hz= SAMPLING_FREQ/(float)IORD(FREQUENCY_ANALYSER_BASE, 0);
	//Start timestamp timer for checking validity

	//Measure execution time for first load shed, dont care if already load shedding I think
	if(!gLoadShedding && gManagingLoads){
		alt_timestamp_start();
		gLoadShedTimer = true;
	}
	xQueueSendToBackFromISR(qFreqReceived, &hz, pdFALSE );
}

void ISRKeyInputReceived(void* context, alt_u32 id)
{
	  char ascii;
	  int status = 0;
	  unsigned char key = 0;
	  KB_CODE_TYPE decode_mode;
	  status = decode_scancode (context, &decode_mode , &key , &ascii) ;

	  if ( status == 0 ) //success
	  {
	    // print out the result
	    switch ( decode_mode )
	    {

	      case KB_ASCII_MAKE_CODE :
	    	  //Check if key pressed is digit
	        if(HelperIsDigit(ascii)){
	        	if(gKeyDown){
	        		if(ascii == gLastKeyPressed){
	        			gKeyDown = false;
	        		}
	        	}
	        	else{
	        		gKeyDown = true;
	        		gLastKeyPressed = ascii;
	        		xQueueSendToBackFromISR(qCharReceived, &ascii, pdFALSE );


	        	}
	        }


	        break ;
	      default :
	        break ;
	    }

	  }
}

void ISREnableMaintenanceMode(void* context, alt_u32 id)
{
	  int _keyval = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) & 0x4;
	  if(_keyval){
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



				if(_instant < gParams.instant_threshold || _rate > gParams.rate_threshold)
				{
					// If not running already, start unstable timer on unstable behavior


					// If moving into load-shedding mode
					// Set up variables and instantly shed first load
					if(!gLoadShedding)
					{
						int i;
						for(i = 0; i < PARAM_NUM_LOADS; i++) {
							xSemaphoreTake(xSystemStatus, portMAX_DELAY);
							gSystemStatusEntry[i] = gSystemStatus[i];
							xSemaphoreGive(xSystemStatus);
						}

						gLoadShedding = true;

						HelperLoadDisconnect();
						bool msg = true;
						xQueueSend(qLoadControl, (void *)&msg, 0);
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


			int i;
			int _systemStatus = 0;
			int _systemStatusEntry = 0;

			taskENTER_CRITICAL();
			for(i = 0; i < PARAM_NUM_LOADS; i++){
				_systemStatus |=  ((int)gSystemStatus[i]) << i;
				_systemStatusEntry |= ((int)gSystemStatusEntry[i]) << i;
			}
			taskEXIT_CRITICAL();


			if(!gLoadShedding){
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, _systemStatus);
			}
			else{
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, _systemStatus);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, _systemStatusEntry & (~(_systemStatus)));
			}
			
			if(gLoadShedTimer && gLoadShedding){
				int time = (long)((alt_timestamp())/TIMESTAMP_TO_US_DIV);
				HelperAddNewTiming(time);
				gLoadShedTimer = false;
			}



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
		xSemaphoreTake(xSystemStatus, portMAX_DELAY);
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
		xSemaphoreGive(xSystemStatus);


		// Run load control task for every switch poll update
		//
		// TODO Discuss whether we want this. Current reasoning is it is probably the
		// cheapest computational solution (rather than looking for changes in switch state)
		bool msg = true;
		xQueueSend(qLoadControl, (void *)&msg, 0);

		vTaskDelay(taskSwitchPolling);
	}
}

//Task for updating the instant and ROC thresholds in maintenance mode
//Will receive ints from the keybaord ISR
void TaskUpdateThresholds(){
	//All thresholds are number 00.0-99.9
	int _keyCounter = 0;

	char _msg;

	while(1){

		if(gManagingLoads){
			_keyCounter = 0;
		}
		xQueueReceive(qCharReceived, &_msg, portMAX_DELAY);

		if(!gManagingLoads && !gLoadShedding){
			switch(_keyCounter){
				case 0:
					gParams.instant_threshold = HelperCharToInt(_msg) * 10;
					_keyCounter++;
					break;
				case 1:
					gParams.instant_threshold += HelperCharToInt(_msg);
					_keyCounter++;
					break;
				case 2:
					gParams.instant_threshold += (float)HelperCharToInt(_msg) * 0.1;
					_keyCounter++;
					break;
				case 3:
					gParams.rate_threshold = HelperCharToInt(_msg) * 10;
					_keyCounter++;
					break;
				case 4:
					gParams.rate_threshold += HelperCharToInt(_msg);
					_keyCounter++;
					break;
				case 5:
					gParams.rate_threshold += (float)HelperCharToInt(_msg) * 0.1;
					_keyCounter = 0;
					break;
				default:
					break;
			}
		}



		vTaskDelay(taskUpdateVGA);
	}


}

void TaskUpdateVGA()
{


	float _historyFreq[FREQ_ARR_SIZE];
	float _historyROC[FREQ_ARR_SIZE];
	int _historyStartPos;

	while (1)
	{

		//Copy in arrays so that datat does nto change while running
		taskENTER_CRITICAL();
		_historyStartPos = gHistoryStartPos;
		int i;
		for(i = 0; i < FREQ_ARR_SIZE; i++){
			_historyFreq[i] = gHistoryFreq[i];
			_historyROC[i] = gHistoryROC[i];
		}
		taskEXIT_CRITICAL();

		drawSystemUptime(xTaskGetTickCount()/1000);
		drawGraphs(_historyFreq, _historyROC, _historyStartPos);
		drawThresholds(gParams.instant_threshold , gParams.rate_threshold);
		drawTimings(gTimingInfo.avg, gTimingInfo.min, gTimingInfo.max, gTimingInfo.history, gTimingInfo.history_pos);
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
			xSemaphoreTake(xSystemStatus, portMAX_DELAY);
			gSystemStatus[i] = false;
			xSemaphoreGive(xSystemStatus);
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
			xSemaphoreTake(xSystemStatus, portMAX_DELAY);
			gSystemStatus[i] = true;
			xSemaphoreGive(xSystemStatus);

			break;
		}

	}
	// If all loads have been re-enabled, set flag to false
	if(!HelperLoadsManaged()) {
		gLoadShedding = false;
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

bool HelperIsDigit(char c){
	if(c <= '9' && c >= '0'){
		return true;
	}
	else{
		return false;
	}
}
int HelperCharToInt(char c){
	return (c - '0');
}

void HelperAddNewTiming(int timing){
	if(timing > gTimingInfo.max){
		gTimingInfo.max = timing;
	}
	else if(timing < gTimingInfo.min){
		gTimingInfo.min = timing;
	}


	//Moving Average, add sub oldest add newest from sum
	gTimingInfo.sum = gTimingInfo.sum + timing - gTimingInfo.history[gTimingInfo.history_pos];
	gTimingInfo.avg = gTimingInfo.sum / 5;

	gTimingInfo.history[gTimingInfo.history_pos] = timing;
	gTimingInfo.history_pos = (gTimingInfo.history_pos + 1) % 5;
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
	  set_keyboard_rate(ps2_device, 0xFF);
	  alt_irq_register(PS2_IRQ, ps2_device, ISRKeyInputReceived);
	  // register the PS/2 interrupt
	  IOWR_8DIRECT(PS2_BASE,4,1);
}

void initPushButtonISR(){
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Activate interupt for only BTN_3
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ, NULL , ISREnableMaintenanceMode);
}

// This function creates the queues and semaphores
void initDataStructs(void)
{
	gParams.instant_threshold = PARAM_INITIAL_INSTANT_THRES;
	gParams.rate_threshold = PARAM_INITIAL_RATE_THRES;

	//Init the timing info;



	gTimingInfo.avg = 0;
	int i;
	for(i = 0; i < 5; i++){
		gTimingInfo.history[i] = 0;
	}
	gTimingInfo.history_pos = 0;
	gTimingInfo.max = 0;
	gTimingInfo.min = INT_MAX;
	gTimingInfo.sum = 0;


	gLoadShedding = false;

	// Initialize load switch and load control vectors

	for(i=0; i < PARAM_NUM_LOADS; i++) {
		gSystemStatus[i] = false;
		gSystemStatusEntry[i] = false;
	}

	gManagingLoads = false;

	gStableInitialTick = 0;
	gUnstableInitialTick = 0;

	gHistoryStartPos = 0;

	/* Semaphores */
	xSystemStatus = xSemaphoreCreateMutex();
	if (xSystemStatus == 0)
		fputs("Could not create semaphores xFrequencies", stderr);

	/* Queues */
	// Creating a queue of depth 1 for storing a boolean (update flag)
	qFreqReceived = xQueueCreate(10, sizeof(float));
	if (qFreqReceived == 0)
		fputs("Could not create queue qFreqReceived", stderr);

	qLoadControl = xQueueCreate(1, sizeof(bool));
	if (qLoadControl == 0)
		fputs("Could not create queue qLoadControl", stderr);

	qCharReceived = xQueueCreate(6, sizeof(char));

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

	TaskHandle_t hTaskUpdateThresholds;
	xR = xTaskCreate(TaskUpdateThresholds,"TUt",TASK_STACKSIZE,NULL,PRIORITY_UPDATETHRESHOLDS,&hTaskUpdateThresholds);
	testError(xR, "TUT");

}

void initISR(){
	//register frequency analyser ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, ISRFreqReceived);
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

