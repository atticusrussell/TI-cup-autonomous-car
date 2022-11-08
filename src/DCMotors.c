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

#define MAX_SPEED 	(60)

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
 * @brief stop all of the DC motors by setting their duty cycles to zero
 * 
 */
void stop_DC_motors(void){
	TIMER_A0_PWM_DutyCycle(0.0,1);
	TIMER_A0_PWM_DutyCycle(0.0,2);
	TIMER_A0_PWM_DutyCycle(0.0,3);
	TIMER_A0_PWM_DutyCycle(0.0,4);
}


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


/**
 * @brief takes in a speed value and makes sure its not higher than the max
 * 
 * @param speed
 * @return double  
 */
double bound_speed(double speed){
	if (speed > MAX_SPEED){
		return MAX_SPEED;
	} else{
		return speed;
	}
}


/**
 * @brief moves the left motor as determined by the speed and direction params
 * 
 * @param speed a number from 1 to 100 that selects motor speed
 * @param direction 0 for forward, 1 for reverse
 */
void left_motor_move(double speed, int direction){
	speed = bound_speed(speed);
	int highPin; //pin to have non-zero PWM value on
	int lowPin; //pin to have zero PWM duty cycle on
	if (direction == 0){
		// if direction is zero we go forward
		lowPin = 2;
		highPin = 1;
	}else{
		// we go backward
		lowPin = 1;
		highPin = 2;
	}
	// set the PWM stuff
	TIMER_A0_PWM_DutyCycle(0.01*speed, highPin); //Forward left motor
	TIMER_A0_PWM_DutyCycle(0.0, lowPin); //Forward right motor
}


/**
 * @brief moves the right motor as determined by the speed and direction params
 * 
 * @param speed a number from 1 to 100 that selects motor speed
 * @param direction 0 for forward, 1 for reverse
 */
void right_motor_move(double speed, int direction){
	speed = bound_speed(speed);
	int highPin; //pin to have non-zero PWM value on
	int lowPin; //pin to have zero PWM duty cycle on
	if (direction == 0){
		// if direction is zero we go forward
		lowPin = 4;
		highPin = 3;
	}else{
		// we go backward
		lowPin = 3;
		highPin = 4;
	}
	// set the PWM stuff
	TIMER_A0_PWM_DutyCycle(0.01*speed, highPin); //Forward left motor
	TIMER_A0_PWM_DutyCycle(0.0, lowPin); //Forward right motor
}


/**
 * @brief set both motors to move in same direction and speed
 * 
 * @param speed a value between 1 and 100 (or the max speed) to set the motor
 * @param direction 0 for forward, 1 for reverse
 */
void motors_move(double speed, int direction){
	right_motor_move(speed,direction);
	left_motor_move(speed,direction);
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
 * @brief main function to test the implementation of motor code
 * @note should be commented out or removed in the actual code that gets run
 * 
 * @return int - have to for main function. never actually returned. 
 */
int main(void){
	DC_motors_init();
	DC_motors_enable();

	// for(;;)  //loop forever
	// {

//	int i=0;
//	int waitNo = 1; //delay in the for loop in ms

	motors_move(40.0,0);
	delay(100); // wait
	stop_DC_motors();

	// // 0 to 100% duty cycle in forward direction
	// for (i=0; i<MAX_SPEED; i+=5) {
	// 		// INSERT CODE HERE
		
	// 	delay(waitNo);
	// }
		
		// // 100% down to 0% duty cycle in the forward direction
		// for (i=100; i>=0; i--) {
		//     // INSERT CODE HERE
		// 	TIMER_A0_PWM_DutyCycle(0.01*i, 1); //Forward right motor
		// 	TIMER_A0_PWM_DutyCycle(0.0*i, 4); //Forward left motor
		// 	delay(waitNo);
		// }
		
		// // 0 to 100% duty cycle in reverse direction
		// for (i=0; i<100; i++) {
		//     // INSERT CODE HERE
		// 	TIMER_A0_PWM_DutyCycle(0.0*i, 1); //Reverse right motor
		// 	TIMER_A0_PWM_DutyCycle(0.01*i, 4); //Reverse left motor
			  
		// 	delay(waitNo);
		// }
		
		// // 100% down to 0% duty cycle in the reverse direction
		// for (i=100; i>=0; i--) {
		//     // INSERT CODE HERE
		// 	TIMER_A0_PWM_DutyCycle(0.0*i, 1); //Reverse right motor
		// 	TIMER_A0_PWM_DutyCycle(0.01*i, 4); //Reverse left motor
		// 	delay(waitNo);
		// }

	// }
	return 0;	// NOTE unreachable



}

