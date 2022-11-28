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
#include "ServoMotor.h"
#include "DCMotors.h"
#include "CortexM.h"
#include "Camera.h"
#include	"LEDs.h"
#include "ControlPins.h"
#include "PID.h"

#define USE_PID_STEERING

// turn increments (tuning how hard we turn) - unitless rn
#define TURN_INCREMENT 	(3)


/* define the variables here that define state of car and we're gonna modify */


/* camera variables*/
extern uint16_t line[128];			// current array of raw camera data
uint16_t smoothLine[128];	// camera data filtered by smoothing
uint8_t	trackCenterIndex;	// camera index of the center of the track
uint16_t trackCenterValue;	// value of the camera at the center of the track
BOOLEAN 	carOnTrack;			// true if car is on the track, otherwise false
extern BOOLEAN	g_sendData;			// if the camera is ready to send new data


/* motor variables */
// number from 0 to 100 that will be converted into duty cycle for motor
int motorSpeed; 

// the direction of the motor -> pin to write to i think
enum motorDir{
	FWD = 0,
	REV = 1
};


/* steering variables*/
// angle of wheels in degrees with 0 = straight, + right - left. used for servo
int8_t steeringAngle; // will only go from -60 to 60


///* initialize state variables to starting values */
//motorDir = FWD;
//motorSpeed = NORMAL_SPEED;
//turnAngle = 0;
//edgeNear = 0;


/* Milestone 1 and 2 - carpet detection and stop, bidirectional oval */
// [x] implement logic conditionals
// [x] call update functions
// [x] refine existing servo code with callable functions
// [x] test with hardware the servo function
// [x] create abstract DC motor speed function in this file
// [x] implement motor speed and dir funcs in existing file
// [x] make DC motor header file
// [x] test in hardware the motor function
// [x] test camera functionality/ look at the MATLAB plot of the track lab5 code
// [x] create function to use the camera to see if on the track
// [x] figure out what direction the center of the track is
// [x] turn the wheels towards the center of the track
// [x] get signoff for MS1 = carpet detection and biderectional oval

/* Milestone 2 - Figure 8 intersection*/
// [x] test figure 8 
// [x] get signoff for figure 8  - demo to TA. 

/* Milestone 3 -  PID Control */
// [x] outline PID control implementation for steering
// [x] fill in rough pid control framework
// [ ] test PID control
// [ ] tune PID control (look into automated tuning)
// [ ] make PID control lap track proficiently
// [ ] obtain signoff for PID - TA demo

// RACE 
// [ ] Create modes with different levels of aggression/ motor DC
// [ ] use switches to select different track mode
// [ ] use LED to display track mode  
// [ ] Test car in different track configurations
// [ ] test different modes and optimize

// optional
// [ ] implement differential drive for sharper turning
// [ ] use the visual mass to determine corners vs straights and set drive speed
// [ ] create states for approaching corner, in corner, leaving corner,
// [ ] convert to using improved CortexM file made during DAC EC by AJR
// [ ] implement PID control of the drive motors

// very optional
// FUTURE distance to the edge if we know it/ can find it
// FUTURE centralize delay function
// if edge has been detected near car (or maybe just at all - tbd )
//BOOLEAN edgeNear;
// edge direction: -1 left, 1 is right, 0 straight ahead (we'll turn left 0)
//signed int edgeDir;






	/**
	 * @brief steers the servo to the center of the track
	 * 
	 * @param trackCenter a number from 1 to 128 that indicates the track center
	 */
	void steer_to_center(uint8_t trackCenter){
		int8_t steerAng;	// ang deg center of car to center of track
		// if less than 64 will be negative and left, otherwise pos and right
		steerAng = 64 - trackCenter;
		// 64 is close enough to 60. 
		// if it is greater than 60 or less than -60 bounding func will catch
		set_steering_deg(steerAng*TURN_INCREMENT);
	}



// NOTE we want to abstract away the hardware details in main
int main(void){
	// FUTURE make into its own function
	/* Initialize each component of the car*/
	DisableInterrupts();
	servo_init();
	DC_motors_init();
	INIT_Camera();
	// enable the LEDs for signaling state info
	LED1_Init();
	LED2_Init();
	// enable interrupts so the camera updates
	EnableInterrupts();

	#ifdef USE_PID_STEERING
	// FUTURE move this out of main() if possible - maybe init func?
	// define PID vars if applicable
	pid_tune_t steerPIDTune;
	// TODO tune these values - 0.5, 0.0, 0.0 is a place to start from slides
	steerPIDTune.kp = 0.005; // [ ] tuned kp
	steerPIDTune.ki = 0.025; // [ ] tuned ki
	steerPIDTune.kd = 0.0045; // [ ] OPTIONAL tuned kd

	pid_hist_t steerPIDHist;
	// initialize all of the past vars to zero as shown in slides
	steerPIDHist.error_n1 = 0;
	steerPIDHist.error_n2 = 0;
	steerPIDHist.setPoint_n1 = 0;
	#endif

	// infinite loop to contain logic
	while(1){


		// light up the LED when the camera is sending data
		if(g_sendData== TRUE){
			LED1_On();
		} else{
			LED1_Off();
		}


		smooth_line(line,smoothLine);
		trackCenterIndex = get_track_center(smoothLine);
		trackCenterValue = smoothLine[trackCenterIndex];
		carOnTrack = get_on_track(trackCenterValue);

		// turn the sevo towards the center of the track
		#ifndef USE_PID_STEERING
		// just use regular steering method
		steer_to_center(trackCenterIndex);
		#else 
		// use PID steering
		steer_to_center(SteeringPID(steerPIDTune, &steerPIDHist, steerPIDHist.setPoint_n1 ,trackCenterIndex));

		#endif

		if(carOnTrack){
			DC_motors_enable();
			// make the LED green if on the track
			LED2_SetColor(GREEN);
			motors_move(NORMAL_SPEED, 0);
			
		} else{
			//we are off the track
			LED2_SetColor(RED);
			stop_DC_motors();
		}
	}
	
	// return 0;  // never reached due to infinite while loop
}


// //  delay function if needed - shouldn't use this  -  use CortexM mod (DAC)
///**
// * @brief waits for a delay (in milliseconds)
// * 
// * @param del - The delay in milliseconds
// * @note not accurate in milliseconds with 3 MHz clk. maybe with 48?
// */
//void delay(int del){
//	volatile int i;
//	for (i=0; i<del*50000; i++){
//		;// Do nothing
//	}
//}

/* initial concept / pseudocode to use as guidance/notes

	if(onTrack){
		if(edgeNear){
			// go to slower speed
			// NOTE we may want to do this incrementally too - tbd
			motorSpeed = TURN_SPEED;

			// turn wheels opposite dir to found edge 
			// hopefully this will update fast enough to turn incrementally
			if(edgeDir <= 0){ // if edge to the left or straight ahead
				// adjustments to turn right
				turnAngle += TURN_INCREMENT;
			} else{
				// turn left
				turnAngle -= TURN_INCREMENT;
			}
		} else{ 
			// no edge detected or near 
			turnAngle = 0; //set steering straight
			motorSpeed = NORMAL_SPEED;
		}

	} else {
		// not on track 
		motorSpeed = 0; // stop the car
		// FUTURE in future reverse back onto track maybe
	}

	// call the code -> real world update functions 
	set_motor_speed(motorSpeed);
	set_motor_dir(motorDir);
	set_steering_deg(turnAngle);

	// update functions map from real world -> code 
	edgeNear = get_edge_near();
	if (edgeNear){
		edgeDir = get_edge_dir();
	}
	onTrack = get_on_track();
 */
