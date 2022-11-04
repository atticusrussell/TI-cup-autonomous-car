// ADC14.c
// Runs on MSP432
// ADC input, software trigger, 14-bit conversion,
// 2.5 V static (always on) reference
// Daniel Valvano
// June 11, 2015

// Modified
// LJBeato
// March 2021

#include <stdint.h>
#include "msp.h"
#include "Common.h"
#include "CortexM.h"
#include "ADC14.h"
//
//
// We are going to use Port 4 Pin 7 which is ADC A6
// P4.7 = A6
//
//
void ADC0_InitSWTriggerCh6(void)
{
	// wait for reference to be idle
	// REF_A->CTL0
  	while (REF_A->CTL0 & BIT(16)){}; // keep stuck in while loop when BUSY is high
		
	// set reference voltage to 2.5V
	// 1) configure reference for static 2.5V
	// REF_A->CTL0
  	REF_A->CTL0 = 0x0039;             
		
	// wait for reference voltage to be ready
	// REF_A->CTL0
  	while((REF_A->CTL0&0x1000) == 0){};

	// 2) ADC14ENC = 0 to allow programming
	// ADC14->CTL0
  	ADC14->CTL0 &= ~BIT1; // set bit1 to 0

	// 3) wait for BUSY to be zero		
	// ADC14->CTL0
  	while(ADC14->CTL0&0x00010000){};   
		
	
	// ------------------------------------------------------------------		
	// 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
	// 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
	// 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
	// 25    ADC14ISSH  invert sample-and-hold  0b =  not inverted
	// 24-22 ADC14DIVx  clock divider         000b = /1
	// 21-19 ADC14SSELx clock source select   100b = SMCLK
	// 18-17 ADC14CONSEQx mode select          00b = Single-channel, single-conversion
	// 16    ADC14BUSY  ADC14 busy              0b (read only)
			
	// 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks (use 32 clocks)
			
	// 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks (use 32 clocks)
			
	// 7     ADC14MSC   multiple sample         0b = not multiple
	// 6-5   reserved                          00b (reserved)
	// 4     ADC14ON    ADC14 on                1b = powered up

	// 3-2   reserved                          00b (reserved)
	// 1     ADC14ENC   enable conversion       0b = ADC14 disabled
	// 0     ADC14SC    ADC14 start             0b = No start (yet)
	// ------------------------------------------------------------------
	// 4) single, SMCLK, on, disabled, /1, 32 clocks, SHM	pulse-mode
	// ADC14->CTL0
	// inefficient way:
	// ADC14->CTL0 |= (BIT4 | BIT8 | BIT9 | BIT(12) | BIT(13) | BIT(21) | BIT (26)); // set the high bits
	// ADC14->CTL0 &= ~(BIT(10) | BIT(11) | BIT(14) | BIT(15)); // set the bits low
	// 100 0010 0000 0011 0011 0001 0000 == 0x420 3310
	ADC14->CTL0 = 0x4203310;

	// 20-16 STARTADDx  start addr          00000b = ADC14MEM0
	// 15-6  reserved                  0000000000b (reserved)
	// --
  	// 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
		// --
  	// 3     ADC14DF    data read-back format   0b = Binary unsigned
  	// 2     REFBURST   reference buffer burst  0b = reference on continuously
  	// 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
	//
	// 5) ADC14MEM0, 14-bit, ref on, regular power
	// ADC14->CTL1
	// 0b00000000110000 = 0x30
  	ADC14->CTL1 = 0x30; 
		
	// ADC14->MCTL[0]
	// VREF buffered
	// End of sequence
	// 00110b = If ADC14DIF = 0: A6;
	// // 6) 0 to 2.5V, channel 6
  	ADC14->MCTL[0] = 0x00000186;         
	

	// 15   ADC14WINCTH Window comp threshold   0b = not used
	// 14   ADC14WINC   Comparator enable       0b = Comparator disabled
	// 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
	// 12   reserved                            0b (reserved)
	// 11-8 ADC14VRSEL  V(R+) and V(R-)      0001b = V(R+) = VREF, V(R-) = AVSS
	// 7    ADC14EOS    End of sequence         1b = End of sequence
	// 6-5  reserved                           00b (reserved)
	// 4-0  ADC14INCHx  Input channel        0110b = A6, P4.7

	// 7) no interrupts
	ADC14->IER0 = 0;
	ADC14->IER1 = 0;

	//
	// P4.7 is Analog In A6
	// 8) analog mode on A6, P4.7
	// set pins for ADC A6
	// SEL0, SEL1
  	P4->SEL0 |= BIT7;                  
  	P4->SEL1 |= BIT7;
	
	// 9) enable
	// ADC14->CTL0
	ADC14->CTL0 |= BIT1;
}


// ADC14->IFGR0 bit 0 is set when conversion done
// cleared on read ADC14MEM0
// ADC14->CLRIFGR0 bit 0, write 1 to clear flag
// ADC14->IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14->MEM[0] 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
unsigned int  ADC_In(void)
{
	unsigned int adcIn; 
	
	// 1) wait for BUSY to be zero  ADC14->CTL0
	// ADC14->CTL0
  	while(ADC14->CTL0&0x00010000){}; 
		
	// 2) start single conversion	  
	// ADC14->CTL0
  	ADC14->CTL0 |= BIT0; // start

	// 3) wait for ADC14->IFGR0, ADC14->IFGR0 bit 0 is set when conversion done
	// ADC14->IFGR0
  	while(!(ADC14->IFGR0 & BIT0)){};
		
	// 14 bit sample returned  ADC14->MEM[0]
	// ADC14->MEM[0] 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
	adcIn = ADC14->MEM[0];

	return adcIn;                 // 4) return result 0 to 16383
}
