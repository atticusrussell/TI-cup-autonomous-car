/*
 * Main module for testing the PWM Code for the K64F
 * 
 * Author:  
 * Created:  
 * Modified: Carson Clarke-Magrab <ctc7359@rit.edu> 
 * LJBeato
 * 2021
 */

#include "msp.h"
#include "uart.h"
#include "TimerA.h"

// #define PART1 // comment out for part 2



/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

//void control_speed(int){}
void forward(){
	int i=0;
	int waitNo = 1; //delay in the for loop 

	// 0 to 100% duty cycle in forward direction
	for (i=0; i<100; i++) {
			// INSERT CODE HERE
		TIMER_A0_PWM_DutyCycle(0.01*i, 1); //Forward left motor
		TIMER_A0_PWM_DutyCycle(0.0*i, 4); //Forward right motor
		delay(waitNo);
	}
}

void reverse(){
	int i=0;
	int waitNo = 1; //delay in the for loop
	
	// 0 to 100% duty cycle in reverse direction
	for (i=0; i<100; i++) {
			// INSERT CODE HERE
		TIMER_A0_PWM_DutyCycle(0.0*i, 1); //Reverse right motor
		TIMER_A0_PWM_DutyCycle(0.01*i, 4); //Reverse left motor
			
		delay(waitNo);
	}

}

int main(void) {
	// Initialize UART and PWM
	uart0_init();
	P2 -> SEL0 |= BIT4; //PWM generated on P2.4
	P2 -> SEL1 |= BIT4; //PWM generated on P2.4
	P2 -> SEL0 |= BIT7; //PWM generated on P2.7
	P2 -> SEL1 |= BIT7; //PWM generated on P2.7

	// Print welcome over serial
	uart0_put("Running... \n\r");
	
	// Part 1 - UNCOMMENT THIS
	// Generate 20% duty cycle at 10kHz
	TIMER_A0_PWM_Init(300, 0.2, 4);
	TIMER_A0_PWM_Init(300, 0.0, 1);

#ifdef PART1

	// INSERT CODE HERE
	//Initializing to 300 because 3 mhz / 10 khz = 300 cycles
	
	TIMER_A0_PWM_DutyCycle(0.2, 4);
	TIMER_A2_PWM_DutyCycle(0.2, 1);
	
	
	for(;;) ;  //then loop forever

#else 

	//Part 2 - UNCOMMENT THIS
	for(;;)  //loop forever
	{
		// uint16_t dc = 0;
		// uint16_t freq = 10000; // Frequency = 10 kHz 
		// uint16_t dir = 0;
		// char c = 48;
		int i=0;
		int waitNo = 1; //delay in the for loop 

		// 0 to 100% duty cycle in forward direction
		for (i=0; i<100; i++) {
		    // INSERT CODE HERE
			TIMER_A0_PWM_DutyCycle(0.01*i, 1); //Forward left motor
			TIMER_A0_PWM_DutyCycle(0.0*i, 4); //Forward right motor
			delay(waitNo);
		}
		
		// 100% down to 0% duty cycle in the forward direction
		for (i=100; i>=0; i--) {
		    // INSERT CODE HERE
			TIMER_A0_PWM_DutyCycle(0.01*i, 1); //Forward right motor
			TIMER_A0_PWM_DutyCycle(0.0*i, 4); //Forward left motor
			delay(waitNo);
		}
		
		// 0 to 100% duty cycle in reverse direction
		for (i=0; i<100; i++) {
		    // INSERT CODE HERE
			TIMER_A0_PWM_DutyCycle(0.0*i, 1); //Reverse right motor
			TIMER_A0_PWM_DutyCycle(0.01*i, 4); //Reverse left motor
			  
			delay(waitNo);
		}
		
		// 100% down to 0% duty cycle in the reverse direction
		for (i=100; i>=0; i--) {
		    // INSERT CODE HERE
			TIMER_A0_PWM_DutyCycle(0.0*i, 1); //Reverse right motor
			TIMER_A0_PWM_DutyCycle(0.01*i, 4); //Reverse left motor
			delay(waitNo);
		}

	}
#endif
	return 0;
}



