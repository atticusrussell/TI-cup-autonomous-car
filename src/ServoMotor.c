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



// // which way to turn but encoded as ints
// enum turnDir{
// 	Left = -1,
// 	Right = 1
// } direction;


//int servoPeriod;
double oneDegDC = 0.5/MAX_ANGLE_DEG;



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
	


	/* start puzzling comments from prior lab */
	//initialize cycles for period
	// uint32_t clkFrq = 3000000; // 3 MHz
	// uint16_t desiredServoFrq = 50; // 50 Hz
	// uint16_t period = clkFrq / desiredServoFrq;
	/* end puzzling comments from prior lab */
	// TODO  i think that servo is pin 5.6 on the board
	/**
	 * notes from slides about NXP car servo
	 * microcontrollers tell the controller boards inside servos where to
	 *		turn by inputting pulse signals.
		*			• These signals are called pulse-proportional modulation.
		*			– The width of the pulse is what drives the controller board.
		*			– For example, typically a 1 - 2m pulse out of a 20ms period  
		*				is used in a continuous fashion
		* 	A 1.5ms pulse says do not turn at all
		* 	A 1.0ms pulse says to turn 90o	counter-clockwise
		* 	A 2.0ms pulse says to turn 90o in the clockwise direction
		*/
void servo_init(void){
	int servoPeriod = SERVO_CLK/PWM_FREQ_HZ;
	double servoStraightDC = 1.5/servoPeriod;
	
	// TODO start with servo straight -> initialize to 1.5 ms
	TIMER_A2_PWM_Init(servoPeriod, servoStraightDC, 1); 
}


/**
 * @brief Set the duty cycle of the servo
 * 
 * @param servoDC the new duty cycle to set the servo to. is a num 0 to 1 
 */
void set_servo_DC(double servoDC){
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
	if (abs(steeringAngle) > MAX_ANGLE_DEG){
		if(direction == -1){
			steeringAngle = -MAX_ANGLE_DEG;
		} else{
			steeringAngle = MAX_ANGLE_DEG;
		}
	}
	/* A 1.5ms pulse says do not turn at all
	* 	A 1.0ms pulse says to turn 90o	counter-clockwise
	* 	A 2.0ms pulse says to turn 90o in the clockwise direction	 */
	// FUTURE maybe don't do a floating point multiply in the future
	double servoDC= 1 +  (direction * steeringAngle * oneDegDC);

	set_servo_DC(servoDC);

}
	

int main(void){
	servo_init();
	while(1){
		for(int i=-60;i<61;i+=15){
			set_steering_deg(i);
			delay(5);
		}
	}
}
