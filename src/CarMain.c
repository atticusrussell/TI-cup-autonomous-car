/**
 * @file		CarMain.c
 * @author 	Atticus Russell
 * @author 	Erica Coles
 * @details Main file to control CMPE-460 (IDE) car project on MSP432 
 * @date 	Fall 2022 semester RIT
*/
#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "uart.h"
#include "Common.h"

// constant params to tweak as needed
// maximum motor duty cycle
#define MOTOR_MAX_SPEED (30)
// normal forward motor duty cycle
#define NORMAL_SPEED 	(10)
// slower motor duty cycle for turns
#define TURN_SPEED 		(5)
// turn increments (tuning how hard we turn) - unitless rn
#define TURN_INCREMENT (3)


// FUTURE  see if we want to run processor at faster 48 MHz clk for 
//		responsiveness
// FUTURE implement error handling


/* define the variables here that define state of car and we're gonna modify */
// number from 0 to 100 that will be converted into duty cycle for motor
int motorSpeed; 

// the direction of the motor -> pin to write to i think
enum motorDir{
	FWD = 1,
	REV = 0
};

// angle of wheels in degrees with 0 = straight, + right - left. used for servo
signed int turnAngle;

// if edge has been detected near car (or maybe just at all - tbd )
BOOLEAN edgeNear;
// edge direction: -1 left, 1 is right, 0 straight ahead (we'll turn left 0)
signed int edgeDir;
// if we think we're on the track
BOOLEAN onTrack;

/* currently unused but things to implement */
// FUTURE distance to the edge if we know it/ can find it
// int edgeDist;

// FUTURE centralize delay function

/* initialize state variables to starting values */
motorDir = FWD;
motorSpeed = NORMAL_SPEED;
turnAngle = 0;
edgeNear = 0;
onTrack = 1;


// [x] implement logic conditionals
// [x] call update functions
// [ ] create abstract turning functions in this file
// [ ] refine existing servo code with callable functions
// [ ] test with hardware the servo function
// [ ] create abstract DC motor speed function in this file
// [ ] implement motor speed and dir funcs in existing file
// [ ] test in hardware the motor function
// [ ] test camera functionality/ look at the MATLAB plot of the track lab5 code
// [ ] create function to use the camera to see if on the track
// [ ] create specific carpet detection function in some file (milestone 1)
// [ ] create function to detect edge with camera
// [ ] create function to report edge direction with camera 
// NOTE maybe direction not totally necessary - could possibly pick a direction at random and just see if edge gets closer or further.



/**
 * @brief Call servo file to set direction of wheels
 * 
 * @param angle est. angle of wheels in degrees. 0 = straight, + right -left.
 */
void set_steering(signed int angle){
	// TODO call the servo functions in some smart way
}


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


// NOTE we want to abstract away the hardware details in main
int main(void){
	// TODO call initialization function for servo

	// TODO call initialization function for motor

	// TODO call initialization function for camera

	// infinite loop to contain logic
	while(1){
		// if on the track
		if(onTrack){
			if(edgeNear){
				// go to slower speed
				// NOTE we may want to do this incrementally too - tbd
				motorSpeed = TURN_SPEED;

				/* turn wheels opposite dir to found edge */
				// hopefully this will update fast enough to turn incrementally
				if(edgeDir <= 0){ // if edge to the left or straight ahead
					// adjustments to turn right
					turnAngle += TURN_INCREMENT;
				} else{
					// turn left
					turnAngle -= TURN_INCREMENT;
				}
			} else{ 
				/* no edge detected or near */
				turnAngle = 0; //set steering straight
				motorSpeed = NORMAL_SPEED;
			}

		} else {
			/* not on track */
			motorSpeed = 0; // stop the car
			// FUTURE in future reverse back onto track maybe
		}

		/* call the code -> real world update functions */
		set_motor_speed(motorSpeed);
		set_motor_dir(motorDir);
		set_steering(turnAngle);

		/* update functions map from real world -> code */
		edgeNear = get_edge_near();
		if (edgeNear){
			edgeDir = get_edge_dir();
		}
		onTrack = get_on_track();

	}
	
	return 0;  // never reached due to infinite while loop
}