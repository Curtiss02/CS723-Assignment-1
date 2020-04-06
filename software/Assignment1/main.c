
// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>


// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <altera_avalon_pio_regs.h>
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts


// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities


// Definition of Message Queue

//Definition of LCD Constants
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Semaphores

xSemaphoreHandle x;


// globals variables
volatile unsigned int maintenance_mode = 0;


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
void initOSDataStructs(){

}
void initCreateTasks(){
	xTaskCreate(debug_print_task, "debug_print_task", TASK_STACKSIZE, NULL, 10, NULL);
}

void initISRs(){
	register_button_isr();
}




int main(int argc, char* argv[], char* envp[])
{
	//initOSDataStructs();
	initCreateTasks();
	initISRs();
	vTaskStartScheduler();
	for(;;);


	return 0;
}
