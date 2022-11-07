// TimerA.c

/*  LJBeato
    2021
    TimerA functionality to drive DC motor
    and Servo Motor
 */


#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"
#include "Uart.h"

// Make these arrays 5 deep, since we are using indexes 1-4 for the pins
static uint32_t DEFAULT_PERIOD_A0[5] = {0,0,0,0,0};
static uint32_t DEFAULT_PERIOD_A2[5] = {0,0,0,0,0};
//***************************PWM_Init*******************************
// PWM output on P2.4, P2.5, P2.6, P2.7
// Inputs:  period of P2.4...P2.7 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)
//          pin number (1,2,3,4)
// Outputs: none
int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin)
{

	// Timer A0.1
	if (pin == 1)
	{
		// configure as timer A0 controlled: SEL0=1 SEL1=0
		P2->SEL0 |= BIT4;
		P2->SEL1 &= ~BIT4;
		// configure direction out DIR = 1
		P2->DIR |= BIT4;
	}
    // Timer A0.2
	else if (pin == 2)
	{ 
		// configure as timer A0 controlled: SEL0=1 SEL1=0
		P2->SEL0 |= BIT5;
		P2->SEL1 &= ~BIT5;
		// configure direction out DIR = 1
		P2->DIR |= BIT5;
	}	
    // Timer A0.3
	else if (pin == 3)
	{
		// configure as timer A0 controlled: SEL0=1 SEL1=0
		P2->SEL0 |= BIT6;
		P2->SEL1 &= ~BIT6;
		// configure direction out DIR = 1
		P2->DIR |= BIT6;
	}	
    // Timer A0.4
	else if (pin == 4)
	{
		// configure as timer A0 controlled: SEL0=1 SEL1=0
		P2->SEL0 |= BIT7;
		P2->SEL1 &= ~BIT7;
		// configure direction out DIR = 1
		P2->DIR |= BIT7;
	}
	else return -2;



	
	// save the period for this timer instance
	// DEFAULT_PERIOD_A0[pin] where pin is the pin number
	DEFAULT_PERIOD_A0[pin] = period;
	// TIMER_A0->CCR[0]
	TIMER_A0->CCR[0] = DEFAULT_PERIOD_A0[pin];

	/*
	TAxCCTL0 to TAxCCTL6 Register Description
	--------------------------------------------------------------------------
	Bit 	Field 		Type 		Reset		 Description
	15-14 	CM			 RW 		0h 			Capture mode
											00b = No capture
											01b = Capture on rising edge
											10b = Capture on falling edge
											11b = Capture on both rising and
											falling edges
	---------------------------------------------------------------------------
	13-12 	CCIS 		RW 			0h 		Capture/compare input select.
	These bits select the TAxCCR0 input signal. See the device-specific data sheet for specific signal connections.
											00b = CCIxA
											01b = CCIxB
											10b = GND
											11b = VCC
	---------------------------------------------------------------------------
	11 		SCS 		RW 			0h 		Synchronize capture source.
	This bit is used to synchronize the capture input signal with the timer clock.
											0b = Asynchronous capture
											1b = Synchronous capture
	---------------------------------------------------------------------------
	10 SCCI RW 0h Synchronized capture/compare input.
	The selected CCI input signal is latched with the EQUx signal and can be read via this bit
	---------------------------------------------------------------------------
	9 Reserved			 R 			0h			 Reserved. Reads as 0.
	---------------------------------------------------------------------------
	8 		CAP 		RW 			0h 				Capture mode
											0b = Compare mode
											1b = Capture mode
	---------------------------------------------------------------------------
	7-5		OUTMOD 		RW 			0h 		Output mode.
	Modes 2, 3, 6, and 7 are not useful for TAxCCR0 because EQUx = EQU0.
											000b = OUT bit value
											001b = Set
											010b = Toggle/reset
											011b = Set/reset
											100b = Toggle
											101b = Reset
											110b = Toggle/set
											111b = Reset/set
	---------------------------------------------------------------------------
	4 		CCIE		RW 			0h 	Capture/compare interrupt enable.
		This bit enables the interrupt request of the corresponding CCIFG flag.
											0b = Interrupt disabled
											1b = Interrupt enabled
	---------------------------------------------------------------------------
	3 		CCI 		R 			0h 		Capture/compare input.
	The selected input signal can be read by this bit.
	---------------------------------------------------------------------------
	2 		OUT 		RW 			0h 		Output.
		For output mode 0, this bit directly controls the state of the output.
											0b = Output low
											1b = Output high
	---------------------------------------------------------------------------
	1 		COV 		RW 			0h 		Capture overflow.
		This bit indicates a capture overflow occurred. COV must be reset with software.
											0b = No capture overflow occurred
											1b = Capture overflow occurred
	---------------------------------------------------------------------------
	0 		CCIFG 		RW 			0h 		Capture/compare interrupt flag
											0b = No interrupt pending
											1b = Interrupt pending
	---------------------------------------------------------------------------
	*/
	// TIMER_A0->CCTL[pin]
	// BIT5 BIT6 BIT7 HIGH
	TIMER_A0->CCTL[pin] |= (BIT5 | BIT6 | BIT7); // RESET/SET

	// set the duty cycle
	uint16_t dutyCycle = (uint16_t) (percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
    //TIMER_A0->CCR[pin]
	TIMER_A0->CCR[pin] = dutyCycle;

	/* TAxCTL Register Description
	--------------------------------------------------------------------------
	Bit		Field 	Type 	Reset 	Description
	--------------------------------------------------------------------------
	15-10 	Reserved RW 	0h 		Reserved
	--------------------------------------------------------------------------
	9-8 	TASSEL 	RW 		0h 		Timer_A clock source select
									00b = TAxCLK
									01b = ACLK
									10b = SMCLK
									11b = INCLK
	--------------------------------------------------------------------------
	7-6 	ID 		RW 		0h 		Input divider. 
	These bits along with the TAIDEX bits select the divider for the input clock.
									00b = /1
									01b = /2
									10b = /4
									11b = /8
	--------------------------------------------------------------------------
	5-4 	MC 		RW 		0h 		Mode control. 
		Setting MCx = 00h when Timer_A is not in use conserves power.
			00b = Stop mode: Timer is halted
			01b = Up mode: Timer counts up to TAxCCR0
			10b = Continuous mode: Timer counts up to 0FFFFh
			11b = Up/down mode: Timer counts up to TAxCCR0 then down to 0000h
	--------------------------------------------------------------------------
	3 		Reserved RW 	0h 		Reserved
	--------------------------------------------------------------------------
	2 		TACLR 	RW 		0h 		Timer_A clear. 
	Setting this bit resets TAxR, the timer clock divider logic, and the
	count direction. The TACLR bit is automatically reset and is always read as zero.
	--------------------------------------------------------------------------
	1 		TAIE 	RW 		0h 		Timer_A interrupt enable. 
		This bit enables the TAIFG interrupt request.
									0b = Interrupt disabled
									1b = Interrupt enabled
	--------------------------------------------------------------------------
	0 		TAIFG 	RW 		0h 		Timer_A interrupt flag
									0b = No interrupt pending
									1b = Interrupt pending
	--------------------------------------------------------------------------
	*/
	// Timer CONTROL register
	// TIMER_A0->CTL
	// BIT9 : Clock source -> SMCLK
	// BIT4 : Up mode
	// BIT2 : Clear Timer_A
	TIMER_A0->CTL |= (BIT9 | BIT4 | BIT2); 
	return 0;
}
//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on pin
// Inputs:  dutycycle, pin
// Outputs: none
// percentDutyCycle is a number between 0 and 1  (ie. 0.5 = 50%)
void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	TIMER_A0->CCR[pin] = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A0[pin]);
}

//***************************PWM_Init*******************************
// PWM output on P5.6
// Inputs:  period of P5.6 is number of counts before output changes state
//          percentDutyCycle (0 -> 1.0)//          duty cycle
//          pin number (1,2,3,4), but always 1
// Outputs: none
int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin)
{

	// NOTE: Timer A2 only exposes 1 PWM pin
	// TimerA2.1
	if (pin == 1)
	{
		// configure as timer A2 controlled: SEL0=1 SEL1=0
		P5->SEL0 |= BIT6;
		P5->SEL1 &= ~BIT6;
		// configure direction out DIR = 1
		P5->DIR |= BIT6;
	}
	else return -2;

	// NOTE: Setup similar to TimerA0 // TODO
	// You will have to use the prescaler (clock divider) to get down to 20ms
	// save the period for this timer instance

	// DEFAULT_PERIOD_A2[pin] where pin is the pin number
	DEFAULT_PERIOD_A2[pin] = period;
	// TIMER_A2->CCR[0]
	TIMER_A2->CCR[0] = DEFAULT_PERIOD_A2[pin];

	/*
	TAxCCTL0 to TAxCCTL6 Register Description
	--------------------------------------------------------------------------
	Bit 	Field 		Type 		Reset		 Description
	15-14 	CM			 RW 		0h 			Capture mode
											00b = No capture
											01b = Capture on rising edge
											10b = Capture on falling edge
											11b = Capture on both rising and
											falling edges
	---------------------------------------------------------------------------
	13-12 	CCIS 		RW 			0h 		Capture/compare input select.
	These bits select the TAxCCR0 input signal. See the device-specific data sheet for specific signal connections.
											00b = CCIxA
											01b = CCIxB
											10b = GND
											11b = VCC
	---------------------------------------------------------------------------
	11 		SCS 		RW 			0h 		Synchronize capture source.
	This bit is used to synchronize the capture input signal with the timer clock.
											0b = Asynchronous capture
											1b = Synchronous capture
	---------------------------------------------------------------------------
	10 SCCI RW 0h Synchronized capture/compare input.
	The selected CCI input signal is latched with the EQUx signal and can be read via this bit
	---------------------------------------------------------------------------
	9 Reserved			 R 			0h			 Reserved. Reads as 0.
	---------------------------------------------------------------------------
	8 		CAP 		RW 			0h 				Capture mode
											0b = Compare mode
											1b = Capture mode
	---------------------------------------------------------------------------
	7-5		OUTMOD 		RW 			0h 		Output mode.
	Modes 2, 3, 6, and 7 are not useful for TAxCCR0 because EQUx = EQU0.
											000b = OUT bit value
											001b = Set
											010b = Toggle/reset
											011b = Set/reset
											100b = Toggle
											101b = Reset
											110b = Toggle/set
											111b = Reset/set
	---------------------------------------------------------------------------
	4 		CCIE		RW 			0h 	Capture/compare interrupt enable.
		This bit enables the interrupt request of the corresponding CCIFG flag.
											0b = Interrupt disabled
											1b = Interrupt enabled
	---------------------------------------------------------------------------
	3 		CCI 		R 			0h 		Capture/compare input.
	The selected input signal can be read by this bit.
	---------------------------------------------------------------------------
	2 		OUT 		RW 			0h 		Output.
		For output mode 0, this bit directly controls the state of the output.
											0b = Output low
											1b = Output high
	---------------------------------------------------------------------------
	1 		COV 		RW 			0h 		Capture overflow.
		This bit indicates a capture overflow occurred. COV must be reset with software.
											0b = No capture overflow occurred
											1b = Capture overflow occurred
	---------------------------------------------------------------------------
	0 		CCIFG 		RW 			0h 		Capture/compare interrupt flag
											0b = No interrupt pending
											1b = Interrupt pending
	---------------------------------------------------------------------------
	*/
	// TIMER_A2->CCTL[pin]
	// BIT5 BIT6 BIT7 HIGH
	TIMER_A2->CCTL[pin] |= (BIT5 | BIT6 | BIT7); // RESET/SET

	// set the duty cycle
	uint16_t dutyCycle = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]);

	// CCR[n] contains the dutyCycle just calculated, where n is the pin number
	// TIMER_A2->CCR[pin]
	TIMER_A2->CCR[pin] = dutyCycle;

	/* TAxCTL Register Description
	--------------------------------------------------------------------------
	Bit		Field 	Type 	Reset 	Description
	--------------------------------------------------------------------------
	15-10 	Reserved RW 	0h 		Reserved
	--------------------------------------------------------------------------
	9-8 	TASSEL 	RW 		0h 		Timer_A clock source select
									00b = TAxCLK
									01b = ACLK
									10b = SMCLK
									11b = INCLK
	--------------------------------------------------------------------------
	7-6 	ID 		RW 		0h 		Input divider.
	These bits along with the TAIDEX bits select the divider for the input clock.
									00b = /1
									01b = /2
									10b = /4
									11b = /8
	--------------------------------------------------------------------------
	5-4 	MC 		RW 		0h 		Mode control.
		Setting MCx = 00h when Timer_A is not in use conserves power.
			00b = Stop mode: Timer is halted
			01b = Up mode: Timer counts up to TAxCCR0
			10b = Continuous mode: Timer counts up to 0FFFFh
			11b = Up/down mode: Timer counts up to TAxCCR0 then down to 0000h
	--------------------------------------------------------------------------
	3 		Reserved RW 	0h 		Reserved
	--------------------------------------------------------------------------
	2 		TACLR 	RW 		0h 		Timer_A clear.
	Setting this bit resets TAxR, the timer clock divider logic, and the
	count direction. The TACLR bit is automatically reset and is always read as zero.
	--------------------------------------------------------------------------
	1 		TAIE 	RW 		0h 		Timer_A interrupt enable.
		This bit enables the TAIFG interrupt request.
									0b = Interrupt disabled
									1b = Interrupt enabled
	--------------------------------------------------------------------------
	0 		TAIFG 	RW 		0h 		Timer_A interrupt flag
									0b = No interrupt pending
									1b = Interrupt pending
	--------------------------------------------------------------------------
	*/
	// Timer CONTROL register
	// TIMER_A2->CTL
	// BIT9 : Clock source -> SMCLK
	// BIT6 BIT7: Input Divider -> /8
	// BIT4 : Up mode
	// BIT2 : Clear Timer_A
	TIMER_A2->CTL |= (BIT9 | BIT7  | BIT6 | BIT4 | BIT2);

	/*
	TAxEX0 Register Description
	--------------------------------------------------------------------------
	Bit 	Field 	Type 	Reset	 Description
	--------------------------------------------------------------------------
	15-3 Reserved 	R 		0h 			Reserved. Reads as 0
	--------------------------------------------------------------------------
	2-0    TAIDEX 	RW 		0h 			Input divider expansion. 
	These bits along with the ID bits select the divider for the input clock.
										000b = Divide by 1
										001b = Divide by 2
										010b = Divide by 3
										011b = Divide by 4
										100b = Divide by 5
										101b = Divide by 6
										110b = Divide by 7
										111b = Divide by 8
	--------------------------------------------------------------------------
	*/
	// divide by 2
	TIMER_A2->EX0 |= BIT0; // NOTE could just say =1

	return 0;
}
//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P5.6
// Inputs:  percentDutyCycle, pin  (should always be 1 for TimerA2.1)
//         
// Outputs: none
// 
void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin)
{
	TIMER_A2->CCR[pin] = (uint16_t)(percentDutyCycle * (double)DEFAULT_PERIOD_A2[pin]);
}
