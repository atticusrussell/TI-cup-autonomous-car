/**
 * @file DCMotors.c
 * @authors Erica Coles, Atticus Russell
 * @brief file to interface motors with IDE msp432P4111 car project
 * @note trying to make it modular. Customized for RPi Motor Driver board.
 * @version 0.1
 * @date November 2022
 * 
 * 
 */

#include "msp.h"
#include "TimerA.h"


/**
 * @brief initialize left and right DC motor to be PWM controlled
 * 
 */
void DC_motors_init(void){
	// configure port multiplexors to SEL0=1, SEL1=0
	// configure port multiplexor P2.4 so that it is TA0.CCI2A/TA0.1
	P2 -> SEL0 |= BIT4; 
	P2 -> SEL1 &= ~BIT4; 
	// configure port multiplexer P2.7 so that it is TA0.CCI4A/TA0.4
	P2 -> SEL0 |= BIT7; 
	P2 -> SEL1 &= ~BIT7;
}


int main(void) {


	
	// Part 1 - UNCOMMENT THIS
	// Generate 20% duty cycle at 10kHz
	// initialize the // TODO left? motor at 20%?
	TIMER_A0_PWM_Init(300, 0.2, 4);
	// initialize the // TODO right? motor at 0% DC. 
	TIMER_A0_PWM_Init(300, 0.0, 1);


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
	return 0;
}

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


/**
 * @brief main function for testing the motors
 * 
 * @return int 
 */
int main(void){
	dc_motors_init();


}

