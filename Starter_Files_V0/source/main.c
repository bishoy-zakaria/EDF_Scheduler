/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

/* ************************** EDF ******************************* */

SemaphoreHandle_t Task_3_semaphore_handler = NULL;

QueueHandle_t UART_Queue_Handler = NULL;

TaskHandle_t Button_1_TASK_Handler = NULL;
TaskHandle_t Button_2_TASK_Handler = NULL;
TaskHandle_t Informing_TASK_Handler = NULL;
TaskHandle_t UART_TASK_Handler = NULL;

char But_1_H [20]= "button one is high";
char But_1_L [20]= "button one is low";

char But_2_H [20]= "button two is high";
char But_2_L [20]= "button two is low";

char Inf [6]= "Ready";

char Buffer[20];

void Button_1_TASK( void * pvParameters )
{
	uint32_t i=0;
	uint16_t b=0;
	TickType_t xLastWakeTime;
 const TickType_t xFrequency = 100;

     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			char* ptr_1 = But_1_H;
			char* ptr_2 = But_1_L;
			
			xSemaphoreTake( Task_3_semaphore_handler , ( TickType_t ) 10 );
			
		   if( GPIO_read(PORT_0 , PIN0) == PIN_IS_HIGH )
			  {
					
					xQueueSend( UART_Queue_Handler , (void*) ptr_1 , ( TickType_t ) 0 );
						
					
				 
			  }
			 else
				 {
					 xQueueSend( UART_Queue_Handler , (void*) ptr_2, ( TickType_t ) 0 );
				 }
				 
				 for(i=0;i<=1000;i++)
				 {
					 for(b=0;i<=60000;i++)
					 i=i;
				 }
				 
			    xSemaphoreGive( Task_3_semaphore_handler);
				 GPIO_write( PORT_0, PIN2, PIN_IS_LOW);
					vTaskDelayUntil( &xLastWakeTime, xFrequency );
					GPIO_write( PORT_0, PIN2, PIN_IS_HIGH);
				 GPIO_write( PORT_0, PIN3, PIN_IS_LOW);
				
				 
    }
}


void Button_2_TASK( void * pvParameters )
{
	uint32_t i=0;
	uint16_t b=0;
	TickType_t xLastWakeTime;
 const TickType_t xFrequency = 100;

     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			char* ptr_2_1 = But_2_H;
			char* ptr_2_2 = But_2_L;
			
			xSemaphoreTake( Task_3_semaphore_handler , ( TickType_t ) 10 );
			
		   if( GPIO_read(PORT_0 , PIN1) == PIN_IS_HIGH )
			  {
					
				   xQueueSend( UART_Queue_Handler , (void*) ptr_2_1, ( TickType_t ) 0 );
				 
			  }
			 else
				 {
					 xQueueSend( UART_Queue_Handler , (void*) ptr_2_2, ( TickType_t ) 0 );
				 }
				 for(i=0;i<=9000;i++)
				 {
					 for(b=0;i<=60000;i++)
					 i=i;
				 }
				 
			 xSemaphoreGive( Task_3_semaphore_handler );
				 
				 GPIO_write( PORT_0, PIN4, PIN_IS_LOW);
		  	vTaskDelayUntil( &xLastWakeTime, xFrequency );	
				 GPIO_write( PORT_0, PIN4, PIN_IS_HIGH);
         GPIO_write( PORT_0, PIN2, PIN_IS_LOW);
				 GPIO_write( PORT_0, PIN3, PIN_IS_LOW);				 
				 
    }
}

void Informing_TASK( void * pvParameters )
{
	uint32_t i=0;
	uint16_t b=0;
	TickType_t xLastWakeTime;
 const TickType_t xFrequency = 50;

     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			char* Inf_ptr = Inf;
			xSemaphoreTake( Task_3_semaphore_handler , ( TickType_t ) 10 );
			
			 xQueueSend( UART_Queue_Handler , (void*)Inf_ptr, ( TickType_t ) 0 );
				 
			for(i=0;i<=5000;i++)
				 {
					 for(b=0;i<=60000;i++)
					 i=i;
				 }
			
			 xSemaphoreGive( Task_3_semaphore_handler);
					GPIO_write( PORT_0, PIN5, PIN_IS_LOW);
						  	vTaskDelayUntil( &xLastWakeTime, xFrequency );
			   GPIO_write( PORT_0, PIN5, PIN_IS_HIGH);
         GPIO_write( PORT_0, PIN4, PIN_IS_LOW);
         GPIO_write( PORT_0, PIN2, PIN_IS_LOW);
				 GPIO_write( PORT_0, PIN3, PIN_IS_LOW);
			   

				
				 
    }
}

void UART_TASK( void * pvParameters )
{
	uint32_t i=0;
	uint16_t b=0;
	TickType_t xLastWakeTime;
 const TickType_t xFrequency = 50;

     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();
	
    for( ;; )
    {
			char* Buf_ptr = Buffer;
			xSemaphoreTake( Task_3_semaphore_handler , ( TickType_t ) 50 );
			
		  xQueueReceive(
                               UART_Queue_Handler,
                               (void*)Buf_ptr,
                               ( TickType_t ) 0
                            );
			
			
	   	vSerialPutString( (const signed char * const) Buf_ptr, 20);
		 for(i=0;i<=4000;i++)
				 {
					 for(b=0;i<=60000;i++)
					 i=i;
				 }
			
			 xSemaphoreGive( Task_3_semaphore_handler );
							GPIO_write( PORT_0, PIN6, PIN_IS_LOW);								 
					vTaskDelayUntil( &xLastWakeTime, xFrequency );
			 	GPIO_write( PORT_0, PIN6, PIN_IS_HIGH);									 
				 GPIO_write( PORT_0, PIN5, PIN_IS_LOW);
         GPIO_write( PORT_0, PIN4, PIN_IS_LOW);
         GPIO_write( PORT_0, PIN2, PIN_IS_LOW);
				 GPIO_write( PORT_0, PIN3, PIN_IS_LOW);
				 
    }
}

void vApplicationTickHook( void )
{
	
			    
			GPIO_write( PORT_0, PIN3, PIN_IS_HIGH);
	GPIO_write( PORT_0, PIN3, PIN_IS_LOW);
	
}


int main( void )
{
	
	prvSetupHardware();
	
Task_3_semaphore_handler = xSemaphoreCreateMutex();
	
  UART_Queue_Handler = xQueueCreate( 20, 20);
	
	
  xTaskPeriodicCreate(    Button_1_TASK,
                            "Button_1 task",
                            100,
                            NULL,
                            2,
                            &Button_1_TASK_Handler,
                          100);	

xTaskPeriodicCreate(    Button_2_TASK,
                            "Button_2 task",
                            100,
                            NULL,
                            1,
                            &Button_2_TASK_Handler,
                         100 );

xTaskPeriodicCreate(    Informing_TASK,
                            "normal task",
                            100,
                            NULL,
                            3,
                            &Informing_TASK_Handler,
                          50);
													
xTaskPeriodicCreate(    UART_TASK,
                            "UART Sending task",
                            100,
                            NULL,
                            4,
                            &UART_TASK_Handler,
                          50);
	vTaskStartScheduler();

	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


