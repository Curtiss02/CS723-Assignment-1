#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*
 *  Not sure what this was here for. Was not added by meee!
 *  Don't believe it was even being used in the example code.
 */
//#define mainREGION_1_SIZE	7001
//#define mainREGION_2_SIZE	18105
//#define mainREGION_3_SIZE	2807
//
// void initialize_heap(void) {
//	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
//	const HeapRegion_t xHeapRegions[] =
//	{
//		/* Start address with dummy offsets						Size */
//		{ ucHeap + 1,											mainREGION_1_SIZE },
//		{ ucHeap + 15 + mainREGION_1_SIZE,						mainREGION_2_SIZE },
//		{ ucHeap + 19 + mainREGION_1_SIZE + mainREGION_2_SIZE,	mainREGION_3_SIZE },
//		{ NULL, 0 }
//	};
//
//	/* Pass the array into vPortDefineHeapRegions(). */
//	vPortDefineHeapRegions( xHeapRegions );
//}


// Defines
#define PARAM_INITIAL_INSTANT_THRES 	2.0
#define PARAM_INITIAL_RATE_THRES 		1.0

#define PARAM_UNSTABLE_DELAY			500 // shed subsequent loads after 500ms
#define PARAM_STABLE_DELAY				500 // re-introduce loads after 500ms

#define PARAM_NUM_LOADS					5 	// number of loads (switches) in use


/* Custom Types */
typedef struct params {
	float instant_threshold;
	float rate_threshold;
} t_params;


// Function Prototypes
void ISR_FreqRecieved(void);
void ISR_KeyInputRecieved(void);
void ISR_EnableMaintenanceMode(void);

void TaskStabilityMonitor();
void TaskLoadControl();
void TaskSwitchPolling();
void TaskUpdateVGA();

void HelperLoadDisconnect(void);
void HelperLoadReconnect(void);
bool HelperLoadsManaged(void);
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
QueueHandle_t qFreqRecieved;
QueueHandle_t qLoadControl;


/* Semaphores */
SemaphoreHandle_t xFrequencies;


/* Global vars */
float gFrequencies[2];
t_params gParams;
bool gLoadShedding;
bool gLoadSwitchStatus[PARAM_NUM_LOADS];
bool gManagingLoads;
bool gSystemStatus[PARAM_NUM_LOADS];
bool gSystemStatusEntry[PARAM_NUM_LOADS];

TickType_t gStableInitialTick;
TickType_t gUnstableInitialTick;


/* Fake ISRs */
unsigned int ulISR_FreqNumber = 3;
/* Fake interrupt generator */
const TickType_t taskInterruptGenDelay = 20 / portTICK_PERIOD_MS;
void TaskInterruptGen() {
	while (1) {
		vPortGenerateSimulatedInterrupt(ulISR_FreqNumber);

		vTaskDelay(taskInterruptGenDelay);
	}
}
unsigned int ulISR_FreqRecieved( void );
unsigned int ulISR_FreqRecieved( void )
{
	printf("ISR  :: ISR_FreqRecieved EXEC\n");
	fflush(stdout);

	gFrequencies[1] = gFrequencies[0];
	//float rand_val = (float)rand()/(float)(RAND_MAX/5.0);
	float rand_val = 0.9;
	gFrequencies[0] = rand_val;

	bool msg = true;
	xQueueSend(qFreqRecieved, (void *)&msg, 0);

    return 1;
}



/* Placeholder ISRs */
void ISR_FreqRecieved()
{
	// TODO Follow example set out above in `ulISR_FreqRecieved()`
}

void ISR_KeyInputRecieved()
{
	// TODO Set params.instant_threshold and params.rate_threshold
}

void ISR_EnableMaintenanceMode()
{
	gLoadShedding = false;
	gManagingLoads = !gManagingLoads;

	// TODO Is waiting 100ms for the system status to
	// return to switch state alright??
}



/* Tasks */

// A bit monolithic, but I'll have to think about how I would chunk it down
// It doesn't accomplish too much.
void TaskStabilityMonitor()
{
	bool *msg;
	while (1)
	{
		xQueueReceive(qFreqRecieved, &msg, portMAX_DELAY);

		if(gManagingLoads)
		{
			if((bool)msg == true)
			{
				// Ensure atomic read by reading to a local var
				float _gFrequencies[2];
				taskENTER_CRITICAL();
				_gFrequencies[0] = gFrequencies[0];
				_gFrequencies[1] = gFrequencies[1];
				taskEXIT_CRITICAL();

				printf("TASK :: TaskStabilityMonitor Received Message\n");
				printf("     :: index 0: %f, index1: %f\n", _gFrequencies[0], _gFrequencies[1]);
				fflush(stdout);

				float _instant = _gFrequencies[0];
				float _rate = _gFrequencies[0] - _gFrequencies[1];
				if(_instant < gParams.instant_threshold || _rate < gParams.rate_threshold)
				{
					// If not running already, start unstable timer on unstable behavior
					printf("     :: THRESHOLD TRIGGERED \n");

					// If moving into load-shedding mode
					// Set up variables and instantly shed first load
					if(!gLoadShedding)
					{
						for(int i = 0; i < PARAM_NUM_LOADS; i++) {
							gSystemStatus[i] = gLoadSwitchStatus[i];
							gSystemStatusEntry[i] = gLoadSwitchStatus[i];
						}

						gLoadShedding = true;

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
						printf(">>>>>>>>>>> UNSTABLE :: %i <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", tickTimer);

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
						printf(">>>>>>>>>>> STABLE :: %i   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", tickTimer);

						// Do re-introduction of load
						HelperLoadReconnect();

						bool msg = true;
						xQueueSend(qLoadControl, (void *)&msg, 0);

						gStableInitialTick = 0;
					}
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
			printf("TASK :: TaskLoadControl Received Message\n");
			printf("     :: index 0: %f, index1: %f\n", gFrequencies[0], gFrequencies[1]);
			fflush(stdout);

			// TODO Output gSystemSwitchStatus to LEDs

			vTaskDelay(taskLoadControl);
		}
	}
}

void TaskSwitchPolling()
{
	while (1) {
		// TODO Functionality
		// Assign load switch status from hardware

		// Placeholder values
		gLoadSwitchStatus[0] = false;
		gLoadSwitchStatus[1] = true;
		gLoadSwitchStatus[2] = false;
		gLoadSwitchStatus[3] = true;
		gLoadSwitchStatus[4] = true;

		// If currently in load shedding mode
		if(gLoadShedding) {
			for(int i = 0; i < PARAM_NUM_LOADS; i++) {
				// Disable load, and ensure it won't be re-enabled by `HelperLoadReconnect()`
				gSystemStatus[i] = (gSystemStatus[i] && gLoadSwitchStatus[i]);
				gSystemStatusEntry[i] = (gSystemStatusEntry[i] && gLoadSwitchStatus[i]);
			}
		} else {
			for(int i = 0; i < PARAM_NUM_LOADS; i++) {
				// Otherwise just update the status of the load
				gSystemStatus[i] = gLoadSwitchStatus[i];
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
	bool *msg;
	while (1)
	{
		xQueuePeek(qFreqRecieved, &msg, portMAX_DELAY); // Don't empty mailbox

		// TODO This function will need to implement its own history of values

		// TODO Curtiss implement VGA functionality

		vTaskDelay(taskUpdateVGA);
	}
}

//
// Helper functions
//
void HelperLoadDisconnect()
{
	// Disable the lowest priority load

	for(int i = 0; i < PARAM_NUM_LOADS; i++)
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

	for(int i = PARAM_NUM_LOADS - 1; i >= 0; i--)
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

	for(int i = 0; i < PARAM_NUM_LOADS; i++)
	{
		if(gSystemStatus[i] != gSystemStatusEntry[i])
			return true;
	}
	return false;
}

void testError(BaseType_t xR, char* msg)
{
	if(xR == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	{
		fprintf(stderr, "Could not allocate required memory for %s", msg);
		exit(1);
	}
}



// This function creates the queues and semaphores
void initDataStructs(void)
{
	gParams.instant_threshold = PARAM_INITIAL_INSTANT_THRES;
	gParams.rate_threshold = PARAM_INITIAL_RATE_THRES;

	gLoadShedding = false;

	// Initialize load switch and load control vectors
	for(int i=0; i < PARAM_NUM_LOADS; i++) {
		gLoadSwitchStatus[i] = false;
		gSystemStatus[i] = true;
		gSystemStatusEntry[i] = true;
	}

	gManagingLoads = true;

	gStableInitialTick = 0;
	gUnstableInitialTick = 0;

	/* Semaphores */
	xFrequencies = xSemaphoreCreateBinary();
	if (xFrequencies == 0)
		fputs("Could not create semaphores xFrequencies", stderr);

	/* Queues */
	// Creating a queue of depth 1 for storing a boolean (update flag)
	qFreqRecieved = xQueueCreate(1, sizeof(bool));
	if (qFreqRecieved == 0)
		fputs("Could not create queue qFreqRecieved", stderr);

	qLoadControl = xQueueCreate(1, sizeof(bool));
	if (qLoadControl == 0)
		fputs("Could not create queue qLoadControl", stderr);
}


// This function creates the tasks
void initTasks(void)
{
	BaseType_t xR;

	TaskHandle_t hTaskStabilityMonitor;
	xR = xTaskCreate(TaskStabilityMonitor,"TSM",500,NULL,1,&hTaskStabilityMonitor);
	testError(xR, "TSM");

	TaskHandle_t hTaskLoadControl;
	xR = xTaskCreate(TaskLoadControl,"TLC",500,NULL,2,&hTaskLoadControl);
	testError(xR, "TLC");

	TaskHandle_t hTaskSwitchPolling;
	xR = xTaskCreate(TaskSwitchPolling,"TSP",500,NULL,3,&hTaskSwitchPolling);
	testError(xR, "TSP");

	TaskHandle_t hTaskUpdateVGA;
	xR = xTaskCreate(TaskUpdateVGA,"TUV",500,NULL,4,&hTaskUpdateVGA);
	testError(xR, "TUV");

}


int main(int argc, char **argv)
{
	//initialize_heap();

	/* Fake ISR init */
	vPortSetInterruptHandler( ulISR_FreqNumber, ulISR_FreqRecieved );

	/* Fake interrupt generator */
	BaseType_t xR;
	TaskHandle_t hTaskInterruptGen;
	// Creating a sending task
	xR = xTaskCreate(TaskInterruptGen,"TIG",500,NULL,1,&hTaskInterruptGen);
	testError(xR, "TaskInterruptGen");

	initDataStructs();
	initTasks();

	// Starting the scheduler
	vTaskStartScheduler();

	// In case if the scheduler failed to start.
	for(;;);
}

