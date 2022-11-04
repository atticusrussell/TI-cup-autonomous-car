/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* LJBeato
* 1/14/2021
*
* Filename: main_timer_template.c
*/
#include <stdio.h>
#include <stdlib.h>

#include "msp.h"
#include "uart.h"
#include "switches.h"
#include "leds.h"
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
extern uint32_t SystemCoreClock;

// these are not used by the timer
BOOLEAN g_sendData = FALSE;
uint16_t line[128];

int colorIndex = 0;
BYTE colors[7] = { RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, WHITE };

BOOLEAN Timer1RunningFlag = FALSE;
BOOLEAN Timer2RunningFlag = FALSE;
BOOLEAN switch1Flash= FALSE;

unsigned long MillisecondCounter = 0;
unsigned long LastMSCount = NULL;
//

//  I/O interrupt pin setup
//
// DIR     SEL0/SEL1    IE    IES     Port Mode
//  0          00        0     0       Input, rising edge trigger
//  0          00        0     1       Input, falling edge trigger, interrupt
//  0          00        1     0       Input, rising edge trigger, interrupt
//  0          00        1     1       Input, falling edge trigger, interrupt
//

void Switch1_Interrupt_Init(void)
{
	// disable interrupts
	DisableInterrupts();
	// initialize the Switch as per previous lab
	Switch1_Init();
	
	//7-0 PxIFG RW 0h Port X interrupt flag
	//0b = No interrupt is pending.
	//1b = Interrupt is pending.
	// clear flag1 (reduce possibility of extra interrupt)	
  	P1->IFG &= ~BIT1; 

	//7-0 PxIE RW 0h Port X interrupt enable
	//0b = Corresponding port interrupt disabled
	//1b = Corresponding port interrupt enabled	
	// arm interrupt on  P1.1	
  	P1->IE |= BIT1;  

	//7-0 PxIES RW Undefined Port X interrupt edge select
  	//0b = PxIFG flag is set with a low-to-high transition.
  	//1b = PxIFG flag is set with a high-to-low transition
	// now set the pin to cause falling edge interrupt event
	// P1.1 is falling edge event
	P1->IES |= BIT1;

	// now set the pin to cause falling edge interrupt event
  	NVIC_IPR8 = (NVIC_IPR8 & 0x00FFFFFF)|0x40000000; // priority 2
	
	// enable Port 1 - interrupt 35 in NVIC	
  	NVIC_ISER1 = 0x00000008;  
	
	// enable interrupts  (// clear the I bit	)
  	EnableInterrupts();              
	
}

void Switch2_Interrupt_Init(void)
{
	// disable interrupts
	DisableInterrupts();
	
	// initialize the Switch as per previous lab
	Switch2_Init();
	
	// now set the pin to cause falling edge interrupt event
	// P1.4 is falling edge event
	P1->IES |= BIT4;

	// clear flag4 (reduce possibility of extra interrupt)
  	P1->IFG &= ~BIT4; 
  
	// arm interrupt on P1.4 
  	P1->IE |= BIT4;     
	
	// now set the pin to cause falling edge interrupt event
  	NVIC_IPR8 = (NVIC_IPR8&0x00FFFFFF)|0x40000000; // priority 2
	
	// enable Port 1 - interrupt 35 in NVIC
  	NVIC_ISER1 = 0x00000008;         
	
	// enable interrupts  (// clear the I bit	)
  	EnableInterrupts();              
	
}

// PORT 1 IRQ Handler
// LJBeato
// Will be triggered if any pin on the port causes interrupt
//
// Derived From: Jonathan Valvano
void PORT1_IRQHandler(void)
{
	float numSeconds = 0.0;
	unsigned long numMs = 0;
	char temp[32];

	// First we check if it came from Switch1 ?
  	if(P1->IFG & BIT1){
		// We check to see if the timer is already running. 
		if (Timer1RunningFlag){
			// If the timer is already running stop it and turn off LED1
			switch1Flash = FALSE;
			Timer1RunningFlag = FALSE;
		} else {
			// we start a timer to toggle the LED1 1 second ON and 1 second OFF
			switch1Flash = TRUE;
			Timer1RunningFlag = TRUE;
		}
		// acknowledge P1.1 is pressed, by setting BIT1 to zero - remember P1.1 is switch 1
		P1->IFG &= ~BIT1; // clear flag, acknowledge
	}
  
	// Now check to see if it came from Switch2 ?
  	if(P1->IFG & BIT4){
		if (Timer2RunningFlag){
			//numSeconds = (MillisecondCounter - LastMSCount) / 1000;
			numMs = MillisecondCounter - LastMSCount;
			// supposed to be in seconds but is for testing
			// sprintf(temp, "ms since prev press: %lu \r\n", numMs);
			// uart0_put(temp);
			numSeconds = numMs * 0.001;
			sprintf(temp, "sec since prev press: %f \r\n", numSeconds);
			uart0_put(temp);
		} else {
			Timer2RunningFlag = TRUE;
		}

		// record current time to use next press to measure elapsed time
		LastMSCount = MillisecondCounter;
		
		LED2_SetColor(colors[colorIndex]);
		//iterate through the colors
		if (colorIndex < 6){
			colorIndex++;
		} else {colorIndex = 0;}


		// acknowledge P1.4 is pressed, by setting BIT4 to zero - remember P1.4 is switch 2
		P1->IFG &= ~BIT4;     // clear flag4, acknowledge
  	}
}

//
// Interrupt Service Routine for Timer32-1
//
//
//
void Timer32_1_ISR(void)
{
	if (switch1Flash){
		if (LED1_State() == FALSE )
		{
			LED1_On();
		}
		else LED1_Off();
	}
}

//
// Interrupt Service Routine
//
//
//
void Timer32_2_ISR(void)
{

		MillisecondCounter++;

}


//
// main
//
//
//
int main(void){
	//initializations
	uart0_init();
	uart0_put("\r\nLab5 Timer demo\r\n");
	// Set the Timer32-1 to 2Hz (0.5 sec between interrupts)
	// ? set SystemCoreClock 1/desiredtime?
	// initialize Timer A32-1;
	Timer32_1_Init(&Timer32_1_ISR, SystemCoreClock/2, T32DIV1); 
        
	// Setup Timer32-2 with a .001 second timeout.
	// So use DEFAULT_CLOCK_SPEED/(1/0.001) = SystemCoreClock/1000
	// initialize Timer A32-2;
	Timer32_2_Init(&Timer32_2_ISR, SystemCoreClock/1000, T32DIV1); 
    
	Switch1_Interrupt_Init();
	Switch2_Interrupt_Init();
	LED1_Init();
	LED2_Init();
	EnableInterrupts();
  while(1)
	{
    WaitForInterrupt();
  }
}
