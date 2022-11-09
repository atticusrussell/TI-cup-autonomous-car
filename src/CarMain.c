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

#define 	BASIC_TESTING

// turn increments (tuning how hard we turn) - unitless rn
#define TURN_INCREMENT 	(3)


// FUTURE implement error handling
/* define the variables here that define state of car and we're gonna modify */


/* camera variables*/
extern uint16_t line[128];			// current array of raw camera data
uint16_t smoothLine[128];	// camera data filtered by smoothing
uint8_t	trackCenterIndex;	// camera index of the center of the track
uint16_t trackCenterValue;	// value of the camera at the center of the track
BOOLEAN 	carOnTrack;			// true if car is on the track, otherwise false
extern BOOLEAN	g_sendData;			// if the camera is ready to send new data


// number from 0 to 100 that will be converted into duty cycle for motor
int motorSpeed; 

// the direction of the motor -> pin to write to i think
enum motorDir{
	FWD = 0,
	REV = 1
};

// angle of wheels in degrees with 0 = straight, + right - left. used for servo
int16_t turnAngle;

// if edge has been detected near car (or maybe just at all - tbd )
BOOLEAN edgeNear;
// edge direction: -1 left, 1 is right, 0 straight ahead (we'll turn left 0)
signed int edgeDir;


/* currently unused but things to implement */
// FUTURE distance to the edge if we know it/ can find it
// int edgeDist;

// FUTURE centralize delay function

///* initialize state variables to starting values */
//motorDir = FWD;
//motorSpeed = NORMAL_SPEED;
//turnAngle = 0;
//edgeNear = 0;
//onTrack = 1;


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
// [ ] create specific carpet detection function in some file (milestone 1)
// [ ] create function to detect edge with camera
// [ ] create function to report edge direction with camera 
// [x] figure out what direction the center of the track is
// [ ] turn the wheels towards the center of the track

// FUTURE use switches and OLED to selelct different track mode and display
// FUTURE use interrupts when doing track edge detection for optimization



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



// NOTE we want to abstract away the hardware details in main
int main(void){
	// FUTURE make into its own function
	/* Initialize each component of the car*/
	DisableInterrupts();
	// call initialization function for servo
	servo_init();
	// call initialization function for motor
	DC_motors_init();
	// call initialization function for camera
	INIT_Camera();
	// enable the LEDs for signaling state info
	LED1_Init();
	LED2_Init();
	// enable interrupts so the camera updates
	EnableInterrupts();

	// infinite loop to contain logic
	while(1){

		#ifdef BASIC_TESTING
		// TODO write some basic test functionality here
		// TODO write this such that the servo points the wheels towards the center of the track

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

		if(carOnTrack){
			// make the LED green if on the track
			LED2_SetColor(GREEN);
			// TODO turn the sevo towards the center of the track
			
		} else{
			//we are off the track
			LED2_SetColor(RED);
		}





		#else // run the actual code





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
		set_steering_deg(turnAngle);

		/* update functions map from real world -> code */
		edgeNear = get_edge_near();
		if (edgeNear){
			edgeDir = get_edge_dir();
		}
		onTrack = get_on_track();

		#endif

	}
	
	// return 0;  // never reached due to infinite while loop
}
