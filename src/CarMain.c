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

// constant params to tweak as needed
// maximum motor duty cycle
#define MOTOR_MAX_SPEED (30)
// normal forward motor duty cycle
#define NORMAL_SPEED 	(10)
// slower motor duty cycle for turns
#define TURN_SPEED 		(5)


// TODO in future see if we want to run processor at faster 48 MHz clk for 
//		responsiveness

// NOTE we want to abstract away the hardware details in main


// define the variables here that define state of car and we're gonna modify  

// number from 0 to 100 that will be converted into duty cycle for motor
int motorSpeed; 

// the direction of the motor -> pin to write to i think
enum motorDir{
	FWD = 1,
	REV = 0
};

// angle of wheels in degrees with 0 = straight, + right - left. used for servo
signed int turnAngle;


// initialize state variables to starting values
motorDir = FWD;
motorDC = NORMAL_SPEED;
turnAngle = 0;


int main(void){

	// TODO implement camera functionality
	// [ ] use the camera to see if on the track

	// set motor direction to always forward until otherwise needed
	set_motor_dir(fwd);

	// infinite loop to contain other functions
	while(1){
		// if on the track
		if(on_track()){

			if(edge_detected){
				set_motor_speed(TURN_DC);



				
			}
			
			

			// if detect edge
				
		
		// else if edge detected 

				// change motor speed to be slower
			
				// call servo function to turn wheels opposite of edge found


		} else { //if not on track

			set_motor_speed(0);
		}

	}
	
	






			


}
