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
 * @brief define how the RPi motor driver shield maps to the MSP432P4111
 * @note TA0.1 -> P2.4 ->	M1A 
 * @note TA0.2 -> P2.5 ->	M1B 
 * 
 * @note TA0.3 -> P2.6 ->	M2A 
 * @note TA0.4 -> P2.7 ->	M2B 
 * 
 * @note P3.6 ->	M1EN
 * @note P3.7 ->	M2EN
 */


/**
 * @brief initialize left and right DC motor to be PWM controlled
 * 
 */
void DC_motors_init(void){
	
	/* INITIALIZE ALL OF THE PINS*/
	// initialize motor enable pins as GPIO (SEL0=0, SEL1=0)
	// P3.6 ->	M1EN (GPIO)
	P3 -> SEL0 &= ~BIT6;
	P3 -> SEL1 &= ~BIT6;
	// P3.7 ->	M2EN (GPIO)
	P3 -> SEL0 &= ~BIT7;
	P3 -> SEL1 &= ~BIT7;
	// set the enable pins as outputs
	P3	-> DIR |= BIT6; 
	P3	-> DIR |= BIT7;
	// set the enable pins to high drive strength
	P3	-> DS |= BIT6;
	P3	-> DS |= BIT7;


	// configure motor port multiplexers to SEL0=1, SEL1=0
	// configure M1A port multiplexor P2.4 so that it is controlled by TA0.1
	P2 -> SEL0 |= BIT4; 
	P2 -> SEL1 &= ~BIT4; 
	// configure M1B port multiplexer P2.5 so that it is controlled by TA0.2
	P2 -> SEL0 |= BIT5; 
	P2 -> SEL1 &= ~BIT5; 
	// configure M2A port multiplexor P2.6 so that it is controlled by TA0.3
	P2 -> SEL0 |= BIT6; 
	P2 -> SEL1 &= ~BIT6; 
	// configure M2B port multiplexer P2.7 so that it is controlled by TA0.4
	P2 -> SEL0 |= BIT7; 
	P2 -> SEL1 &= ~BIT7; 

	/* INITIALIZE ALL OF THE TIMER driven PWM outputs */
	// initialize PWM using Timer A0
	// Initialize at 10kHz if the frequency is 3000000
	// Generate 0% duty cycle at 10kHz
	float init_DC = 0.0;
	// initialize the // TODO left? motor at 20%?
	TIMER_A0_PWM_Init(300, 0.2, 4);
	// initialize the // TODO right? motor at 0% DC. 
	TIMER_A0_PWM_Init(300, 0.0, 1);
}




int main(void) {


	
	


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

