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
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
#include "ADC14.h"
// The sprintf function seemed to cause a hange in the interrupt service routine.
// I think if we increase the HEAP size, it will work
// change to Heap_Size       EQU     0x00000200 in startup_msp432p401r_uvision.s
 
//#define test1
// #define forLoop // I think the instructions about a for loop in the lab
// manual (5.2.6) seem unecessary - define to test. Mem[0] is already accessed 
// in ADC14.c and is returned by ADC_In();

BOOLEAN Timer1RunningFlag = FALSE;
BOOLEAN Timer2RunningFlag = FALSE;

unsigned long MillisecondCounter = 0;

// Interrupt Service Routine for Timer32-1
void Timer32_1_ISR(void)
{
	char tempStr0[64];
	char tempStr1[64];
	//get analog signal value
	unsigned int adcIn = ADC_In();
	#ifdef test1
		#ifdef forloop
		for (int i = 0; i < ADC14->MEM[0]; i++){
		#endif
			sprintf(tempStr0,"Decimal:       %u \r\n", adcIn);		
			sprintf(tempStr1,"Hexadecimal: 0x%x\r\n", adcIn);
		#ifdef forloop
			}
		#endif
	#else
		// celcius and fahrenheit temperature conversions from TMP36
		unsigned int  cel, fahr, mV; 
		// 2346 derived manually measuring voltage, temp, and ADC output
		// dividing by the precision of ADC (16383)'
		mV = (adcIn * 2346)/16383;
		// celcius conversion
		cel = (mV-500)/10; //((adcIn * 2000) / 16383) - 500;
		// fahrenheit conversion
		fahr = (cel * 1.8) + 32;
		sprintf(tempStr0, "Temp is %u deg. C\r\n", cel);
		sprintf(tempStr1, "Temp is %u deg. F\r\n", fahr);
#endif
	uart0_put(tempStr0); 
	uart0_put(tempStr1);
}

// main
int main(void)
{
	//initializations
	uart0_init();
	uart0_put("\r\nLab5 ADC demo\r\n");

	
	ADC0_InitSWTriggerCh6();
	// initialize Timer A32-1;
	Timer32_1_Init(&Timer32_1_ISR, SystemCoreClock / 2, T32DIV1);
	EnableInterrupts();
	
  while(1){
	  WaitForInterrupt();
	}
}

