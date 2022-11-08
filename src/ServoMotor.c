/**
 * @file ServoMotor.c
 * @authors Erica Coles, Atticus Russell
 * @brief file to interface with servo steering IDE msp432 car project
 * @note trying to make it modular and re-usable. Customized for MG996r servo.
 * @version 0.1
 * @date 11-04-2022
 * 
 * 
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "msp.h"
#include "TimerA.h"
#include "ServoMotor.h"


//int servoPeriod;
double oneDegDC = 0.5/SERVO_MAX_ANGLE_DEG;


/**
 * @brief waits for a delay (in milliseconds)
 * 
 * @param del - The delay in milliseconds
 * @note not accurate in milliseconds with 3 MHz clk. maybe with 48?
 */
void delay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}
	


 
/**
 * notes from slides about NXP car servo
 * microcontrollers tell the controller boards inside servos where to
 *		turn by inputting pulse signals.
 *			• These signals are called pulse-proportional modulation.
 *			– The width of the pulse is what drives the controller board.
 *			– For example, typically a 1 - 2m pulse out of a 20ms period  
 *				is used in a continuous fashion
 * 	A 1.5ms pulse says do not turn at all
 *   	A 1.0ms pulse says to turn 90o	counter-clockwise
 *  	A 2.0ms pulse says to turn 90o in the clockwise direction
 */


/**
 * @brief initialize the servo to be controlled using Timer A2
 * 
 */
void servo_init(void){
	uint16_t timerPeriod = SERVO_CLK/SERVO_PWM_FREQ_HZ;
	double servoStraightDC = 1.5/SERVO_PWM_PERIOD_MS;
	
	// start with servo straight -> initialize to 1.5 ms
	TIMER_A2_PWM_Init(timerPeriod, servoStraightDC, 1); 
}


/**
 * @brief Set the duty cycle of the servo
 * 
 * @param servoPulse the duration of the pulse to set. From 1 to 2 ms. 
 */
void set_servo_pulse(double servoPulse){
	double servoDC = servoPulse/SERVO_PWM_PERIOD_MS;
	TIMER_A2_PWM_DutyCycle(servoDC, 1);
}


/**
 * @brief takes the sign of an int
 * 
 * @param x 
 * @return int: -1 if negative, 1 if positive
 */
int sign(int x) {
    return (x > 0) - (x < 0);
}


/**
 * @brief Set the angle of the steering in degrees
 * 
 * @param steeringAngle an angle in degrees to set the steering. -left, +right
 * @note using int for now - FUTURE convert to float if more precision needed
 */
void set_steering_deg(int16_t steeringAngle){
	int direction = sign(steeringAngle); 
	// make sure that it is within the correct bounds
	if (abs(steeringAngle) > SERVO_MAX_ANGLE_DEG){
		if(direction == -1){
			steeringAngle = -SERVO_MAX_ANGLE_DEG;
		} else{
			steeringAngle = SERVO_MAX_ANGLE_DEG;
		}
	}
	/* A 1.5ms pulse says do not turn at all
	* 	A 1.0ms pulse says to turn 90o	counter-clockwise
	* 	A 2.0ms pulse says to turn 90o in the clockwise direction	 */
	// FUTURE maybe don't do a floating point multiply in the future
	// 1.5 + ( +-1)*(value from 60 to 60)*(0.5/60)
	double servoPulse= 1.5 +  (steeringAngle * oneDegDC);
	set_servo_pulse(servoPulse);
}
	

int main(void){
	servo_init();
	while(1){
		set_steering_deg(-60);
		delay(100);
		set_steering_deg(60);
		delay(100);
		set_steering_deg(0);
		delay(100);
	}
}
