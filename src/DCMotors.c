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
 * @brief initialize left and right DC motor to be PWM controlled at 10 kHz.
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


	/* INITIALIZE ALL OF THE TIMER driven PWM outputs on Timer A0 */
	// desired motor frequency in Hz
	int motor_freq_Hz = 10000; // initialized to 10 kHz like in Lab 6
	// calculate the period for the current frequency
	int motor_period = HIGH_CLOCK_SPEED/motor_freq_Hz;
	// initialize to zero percent duty cycle
	float initial_DC = 0.0; 
	// initialize each of the motor outputs
	TIMER_A0_PWM_Init(motor_period, initial_DC, 1);
	TIMER_A0_PWM_Init(motor_period, initial_DC, 2);
	TIMER_A0_PWM_Init(motor_period, initial_DC, 3);
	TIMER_A0_PWM_Init(motor_period, initial_DC, 4);
}


/**
 * @brief enable motors 1 and 2
 * 
 */
void DC_motors_enable(void){
	P3->OUT |= BIT6;	// Enable Motor 1
	P3->OUT |= BIT7;	// Enable Motor 2
}


/**
 * @brief disable motors 1 and 2
 * 
 */
void DC_motors_disable(void){
	P3->OUT &= ~BIT6;	// Disable Motor 1
	P3->OUT &= ~BIT7;	// Disable Motor 2
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

