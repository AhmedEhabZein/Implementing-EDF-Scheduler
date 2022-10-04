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
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
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

/* Tasks Periods */
#define B1_PERIOD         50
#define B2_PERIOD         50
#define PERIODIC_TX_PERIOD     100
#define UART_PERIOD            20
#define L1_PERIOD        10
#define L2_PERIOD        100

/* Tasks Handlers */
TaskHandle_t B1Handler      = NULL;
TaskHandle_t B2Handler      = NULL;
TaskHandle_t PeriodicTxHandler = NULL;
TaskHandle_t UartHandler        = NULL;
TaskHandle_t L1Handler     = NULL;
TaskHandle_t L2Handler     = NULL;

/*Queue Handlers*/
QueueHandle_t xQueue1 = NULL;
QueueHandle_t xQueue2 = NULL;
QueueHandle_t xQueue3 = NULL;

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
 
 
 /*Tasks Implemntations*/ 
void B1( void * pvParameters )
{
	pinState_t B1_New_State;
	pinState_t  B1_Old_State = GPIO_read(PORT_0 , PIN0);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char Flag = 0;

	for( ;; )
	{
		
		
		B1_New_State = GPIO_read(PORT_0 , PIN0);
		
		
		if( B1_New_State == PIN_IS_HIGH &&  B1_Old_State == PIN_IS_LOW)
		{
			
			Flag = 1;
		}
		else if (B1_New_State == PIN_IS_LOW &&  B1_Old_State == PIN_IS_HIGH)
		{
			
			
			Flag = 0;
		}
		else
		{
			Flag = 3;
		}
		
		 B1_Old_State = B1_New_State;
		
		
		xQueueOverwrite( xQueue1 , &Flag );

		vTaskDelayUntil( &xLastWakeTime , B1_PERIOD);

	}
}

void B2( void * pvParameters )
{
	pinState_t  B2_Old_State = GPIO_read(PORT_0 , PIN1);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char Flag = 0;
	pinState_t B2_New_State;

	for( ;; )
	{
		
		B2_New_State = GPIO_read(PORT_0 , PIN1);
		
		if( B2_New_State == PIN_IS_HIGH &&  B2_Old_State == PIN_IS_LOW)
		{
			
			Flag = 1;
		}
		else if (B2_New_State == PIN_IS_LOW &&  B2_Old_State == PIN_IS_HIGH)
		{
			
			Flag = 0;
			
		}
		else
		{
			Flag = 3;
		}
		
		 B2_Old_State = B2_New_State;
		
		xQueueOverwrite( xQueue2 , &Flag );
			
		vTaskDelayUntil( &xLastWakeTime , B2_PERIOD);
	}
}


void Periodic_Tx (void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t i = 0;
	char Trans[28];
	strcpy(Trans, "\nHello World!.");

	for( ; ; )
	{
		for( i = 0 ; i < 28 ; i++)
		{
			xQueueSend( xQueue3 , Trans+i ,100);
		}
		
		vTaskDelayUntil( &xLastWakeTime , PERIODIC_TX_PERIOD);
	}
}

void Uart_Receiver (void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	signed char B1;
	signed char B2;
	char Rx[28];
	
	uint8_t i = 0;
	for( ; ; )
	{
		if( xQueueReceive( xQueue1, &B1 , 0) && B1 != 3)
		{
			xSerialPutChar('\n');		
			if(B1==1)
			{
							char String[27]=" Rising edge: B1 \n";
							vSerialPutString((signed char *) String, strlen(String));
			}
			else
			{
							char String[27]=" Falling edge: B1 \n";
							vSerialPutString((signed char *) String, strlen(String));
			}
		}
		else
		{
			
		}
		
		if( xQueueReceive( xQueue2, &B2 , 0) && B2 != 3)
		{
			xSerialPutChar('\n');		
			if(B2==1)
			{
							char String[27]=" Rising edge: B2 \n";
							vSerialPutString((signed char *) String, strlen(String));
			}
			else
			{
							char String[27]=" Falling edge: B2 \n";
							vSerialPutString((signed char *) String, strlen(String));
			}
		}
		else
		{
			
		}
		
		if( uxQueueMessagesWaiting(xQueue3) != 0)
		{
			for( i = 0 ; i < 28 ; i++)
			{
				xQueueReceive( xQueue3, (Rx+i) , 0);
			}
			vSerialPutString( (signed char *) Rx, strlen(Rx));
			xQueueReset(xQueue3);
		}
		
		vTaskDelayUntil( &xLastWakeTime , UART_PERIOD);
	}
}
	




void L1 ( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i = 0;
	uint32_t Period = 12000*5; /* (XTAL / 1000U)*time_in_ms  */
	for( ; ; )
	{
		for( i = 0 ; i <= Period; i++)
		{
		}
		vTaskDelayUntil( &xLastWakeTime , L1_PERIOD);
	}
}

void L2 ( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i = 0;
	uint32_t Period = 12000*12; /* (XTAL / 1000U)*time_in_ms  */
		
	for( ; ; )
	{		
		for( i = 0 ; i <= Period; i++)
		{
			
		}

		vTaskDelayUntil( &xLastWakeTime , L2_PERIOD);

	}
}
 
 
 
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
	xQueue1 = xQueueCreate( 1, sizeof(char) );
	xQueue2 = xQueueCreate( 1, sizeof(char) );
	xQueue3 = xQueueCreate( 28, sizeof(char) );

    /* Create Tasks here */
	xTaskPeriodicCreate(
			B1,                  /* Function that implements the task. */
			"BUTTON 1 MONITOR",                /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&B1Handler,       /* Used to pass out the created task's handle. */
			B1_PERIOD);     /* Period for the task */

	xTaskPeriodicCreate(
			B2,                  /* Function that implements the task. */
			"BUTTON 2 MONITOR",                /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&B2Handler,       /* Used to pass out the created task's handle. */
			B2_PERIOD);     /* Period for the task */

	xTaskPeriodicCreate(
			Periodic_Tx,               /* Function that implements the task. */
			"PERIODIC TRANSMITTER",             /* Text name for the task. */
			100,                                /* Stack size in womain.crds, not bytes. */
			( void * ) 0,                       /* Parameter passed into the task. */
			1,                                  /* Priority at which the task is created. */
			&PeriodicTxHandler,   /* Used to pass out the created task's handle. */
			PERIODIC_TX_PERIOD);  /* Period for the task */

	xTaskPeriodicCreate(
			Uart_Receiver,                      /* Function that implements the task. */
			"UART RECEIVER",                    /* Text name for the task. */
			100,                                /* Stack size in words, not bytes. */
			( void * ) 0,                       /* Parameter passed into the task. */
			1,                                  /* Priority at which the task is created. */
			&UartHandler,          /* Used to pass out the created task's handle. */
			UART_PERIOD);         /* Period for the task */

	xTaskPeriodicCreate(
			L1,                 /* Function that implements the task. */
			"LOAD 1",               /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&L1Handler,      /* Used to pass out the created task's handle. */
			L1_PERIOD);	   /* Period for the task */

	xTaskPeriodicCreate(
			L2,                 /* Function that implements the task. */
			"LOAD 2",               /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&L2Handler,      /* Used to pass out the created task's handle. */
			L2_PERIOD); 	 /* Period for the task */

	
		
	/* Now all the tasks have been started - start the scheduler.
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();
	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook (void)
{
	GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN2,PIN_IS_LOW);

}


/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000; //20 kHhZ
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/