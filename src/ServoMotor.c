/**
 * @file ServoMotor.c
 * @authors Erica Coles, Atticus Russell
 * @brief file to interface with servo steering IDE msp432 car project
 * @note trying to make it modular and re-usable
 * @version 0.1
 * @date 11-04-2022
 * 
 * 
 */


#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"

// adjustable parameters
uint16_t timerPeriod;
uint16_t delayMs; // delay constant
uint16_t servoPeriod; //20 cycles	




// TODO create a header file


// TODO create adjust func and initialization func instead of main 
	
void servo_init(void){
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
	timerPeriod = 60000;
	servoPeriod = 20; //20 cycles
	
	// TODO start with servo straight -> initialize to 1.5 Ms
	// start at 1.0 ms pulse - should be 90 ccw
	TIMER_A2_PWM_Init(timerPeriod, 1.5/servoPeriod, 1); 
}



void set_servo()
	{

		//pulse of 1.x ms should xx the servo motor
		TIMER_A2_PWM_DutyCycle(2.0 / servoPeriod, 1);
		
}
