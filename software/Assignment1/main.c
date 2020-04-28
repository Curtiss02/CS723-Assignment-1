
// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "global.h"


// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <altera_avalon_pio_regs.h>
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts


#include "vga_output.h"

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define DEBUG_PRINT_PRIORITY 10
#define CHECK_SWITCH_PRIORITY 11
#define SET_LED_PRIORITY 12
#define UPDATE_SCREEN_PRIORITY 9
// Definition of Message Queue

//Definition of LCD Constants
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Semaphores

xSemaphoreHandle x;

//Definition of Queues
static QueueHandle_t Q_freq_data;


// globals variables
volatile unsigned int maintenance_mode = 0;

unsigned int switch_val;
unsigned int load_state = 0;
float f_vals[FREQ_ARR_SIZE] = {0};
float df_vals[FREQ_ARR_SIZE] = {0};



void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	float hz= SAMPLING_FREQ/(float)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR( Q_freq_data, &hz, pdFALSE );

}

void maintenance_mode_button_isr(void* context, alt_u32 id){
	  int keyval = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) & 0x4;
	  if(keyval){
		  maintenance_mode = !(maintenance_mode);
	  }
	  // clears the edge capture register
	  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

}
void register_button_isr(){
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Activate interupt for only BTN_3
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ, NULL , maintenance_mode_button_isr);
}

void debug_print_task(void *pvParameters){
	while(1){
		printf("Maintenance Mode: %d\n", maintenance_mode);
		vTaskDelay(1000);
	}
}
void check_switch_task(void *pvParameters){
	while(1){
		//Only care about first 5 bits/loads
		switch_val = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
		//Should be often enough so that user does not notice delay
		vTaskDelay(200);
	}
}
void update_screen_task(void *pvParameters){
	int data_arr_pos = 0;
	unsigned int z = 0;
	while(1){
		//Testing
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, f_vals+data_arr_pos, 0 );


			float denominator;
			if(data_arr_pos==0){
				denominator = (f_vals[0]+f_vals[99]);
				if(denominator == 0){
					df_vals[0] = 0;
				}
				else{
					df_vals[0] = (f_vals[0]-f_vals[99]) * 2.0 * f_vals[0] * f_vals[99] / denominator;
				}

			}
			else{
				denominator = (f_vals[data_arr_pos]+f_vals[data_arr_pos-1]);
				if(denominator == 0){
					df_vals[0] = 0;
				}else{
					df_vals[data_arr_pos] = (f_vals[data_arr_pos]-f_vals[data_arr_pos-1]) * 2.0 * f_vals[data_arr_pos]* f_vals[data_arr_pos-1] / denominator;
				}

			}
			data_arr_pos = (data_arr_pos + 1) % FREQ_ARR_SIZE;
		}
		drawSystemUptime(xTaskGetTickCount()/1000);

		drawGraphs(f_vals, df_vals, data_arr_pos);
		vTaskDelay(16);
	}
}
void set_led_task(void *pvParameters){
	while(1){
		if(maintenance_mode){
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, switch_val);

		}
		else{

		}
		vTaskDelay(100);
	}

}





void initOSDataStructs(){
	//Only care about first 5 bits/loads
	switch_val = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
	Q_freq_data = xQueueCreate(100, sizeof(float));


}
void initCreateTasks(){
	xTaskCreate(debug_print_task, "debug_print_task", TASK_STACKSIZE, NULL, DEBUG_PRINT_PRIORITY, NULL);
	xTaskCreate(check_switch_task, "checK_switch_task", TASK_STACKSIZE, NULL, CHECK_SWITCH_PRIORITY, NULL);
	xTaskCreate(set_led_task, "set_led_task", TASK_STACKSIZE, NULL, SET_LED_PRIORITY, NULL);
	xTaskCreate(update_screen_task, "update_screen_task", TASK_STACKSIZE, NULL, UPDATE_SCREEN_PRIORITY, NULL);
}

void initISRs(){
	register_button_isr();
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
}



int main(int argc, char* argv[], char* envp[])
{
	initVGA();
	drawBackground();
	srand(901237);
	initOSDataStructs();
	initCreateTasks();
	initISRs();

	vTaskStartScheduler();
	for(;;);


	return 0;
}

