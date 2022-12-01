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
#include "switches.h"

/* commentable defines to control functionality */
// #define USE_PID_STEERING
// #define DISABLE_DRIVE_MOTORS // NOTE COMMENT THIS OUT TO RUN for real

#define MODE_SWITCHING
#define CAR_ARMING
// #define ON_TRACK_LEDS

#define RACECAR_STATE_MACHINE
#define RSM_LEDS

/* define constants that are important parameters to tune */
// turn increments (tuning how hard we turn) unitless scalar
#define TURN_INCREMENT 	(3)






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
// [x] test PID control
// [x] tune PID control (look into automated tuning)
// [x] make PID control lap track proficiently
// [x] obtain signoff for PID - TA demo
// NOTE the tuning params are janky as hell rn but got the demo!!


// RACE Prep
// [x] abandon PID steering 
// [x] arm and disarm motors with a button
// [x] convert to using improved CortexM file made during DAC EC by AJR
// [x] move initialization to seperate function
// [x] move some global vars inside of main() for C best practices
// [ ] Reduce (tune) minimal visual center of mass so that 2 wheels off track
// [ ] fix issue with stopping while still on track
// [ ] Create modes with different levels of aggression/ motor DC and min VCM
// [ ] use switches to select different track mode
// [ ] use LED to display track mode  
// [ ] Test car in different track configurations
// [ ] test different modes and optimize

// optional/ideas
// [ ] use a scalar multiplier for the different aggression modes
// [ ] implement differential drive for sharper turning
// [ ] use the visual mass to determine corners vs straights and set drive speed
// [ ] determine which edge closer when almost off track - spin that wheel faster to get back on 
// [ ] create states for approaching corner, in corner, leaving corner,
// [ ] implement PID control of the drive motors
// [ ] convert to follow C best practices from IDE handbook
// NOTE 0.45 highest DC for motors that won't blow them up

// very optional
// FUTURE distance to the edge if we know it/ can find it
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


/**
 * @brief Initialize each component of the car
 * 
 */
void initCarParts(void){
	DisableInterrupts();
	servo_init();
	#ifndef DISABLE_DRIVE_MOTORS
	DC_motors_init();
	#endif
	INIT_Camera();
	// enable the LEDs for signaling state info
	LED1_Init();
	LED2_Init();
	// enable the switches for modes
	Switch1_Init();
	Switch2_Init();
	// enable interrupts so the camera updates
	EnableInterrupts();
	#ifdef USE_PID_STEERING
	steering_pid_init();
	#endif
}



// NOTE we want to abstract away the hardware details in main
int main(void){
	/* OG Variables*/
	/* camera variables*/
	extern uint16_t line[128];			// current array of raw camera data
	uint16_t smoothLine[128];	// camera data filtered by smoothing
	uint8_t	trackCenterIndex;	// camera index of the center of the track
	//uint16_t trackCenterValue;	// value of the camera at the center of the track
	BOOLEAN 	carOnTrack;			// true if car is on the track, otherwise false
	extern BOOLEAN	g_sendData;			// if the camera is ready to send new data


	/* motor variables */
	// number from 0 to 100 that will be converted into duty cycle for motor
	// int motorSpeed; 

	// the direction of the motor -> pin to write to i think
	enum motorDir{
		FWD = 0,
		REV = 1
	};

	/* steering variables*/
	// angle of wheels in degrees with 0 = straight, + right - left. used for servo
	// int8_t steeringAngle; // will only go from -60 to 60
	/* end OG variables*/

	#ifdef CAR_ARMING
	BOOLEAN carArmed = FALSE;
	#endif

	#ifdef RACECAR_STATE_MACHINE
	// this is the part of the track the car is on 
	enum jeremyClarkson{
		straight = 0,
		normal = 1,
		approachingTurn = 2,
		turning = 3,
		trackEdge = 4,
		richardHammond = 5  // off the track
	} jeremyClarkson;
	#endif

	#ifdef MODE_SWITCHING
	enum speedSetting{
		reckless = 0,
		balanced = 1,
		conservative = 2
	} speedSetting;

	typedef struct carSettings{
		int normalSpeed;
		int maxSpeed;	// the absolute max speed the car will do on straights
		int vcmThreshold; // what the car counts as the track edge
	} carSettingsT;

	/* create each of the modes*/
	// TODO set unique high speed and VCMs for each mode
	int sharedVCM = TUNING_ON_TRACK_VCM;
	carSettingsT recklessMode = {HIGH_SPEED,MAX_SPEED,sharedVCM};
	carSettingsT balancedMode = {NORMAL_SPEED,MAX_SPEED,sharedVCM};
	carSettingsT conservativeMode = {LOW_SPEED, MAX_SPEED, CONSERVATIVE_VCM};


	// var that stores the state of the car
	typedef struct carState{
		// int steeringAngle;
		// int motorSpeed;
		enum speedSetting attackMode;
		enum jeremyClarkson trackPosition;
		int16_t magnitudeVCM; // value of the camera at the center of the track
	} carStateT;



	carStateT thisCarState;
	thisCarState.attackMode = reckless; //choose the starting mode
	BOOLEAN modeLEDUpdated = FALSE; //allows the initial mode to be displayed
	thisCarState.trackPosition = normal; 
	carSettingsT thisCarSettings = recklessMode;
	#endif

	#ifdef USE_PID_STEERING
	// define PID vars if applicable
	float trackCenterDeg = 0.0;
	float PIDRes;
	// intermediate vars to provide visibility into PID actions
	float angleScale = 121.0/128.0;
	int16_t roughCenter;
	int16_t scaledCenter;
	int16_t iPIDRes;
	#endif

	initCarParts();
	// infinite loop to contain logic
	while(1){
		#ifdef CAR_ARMING
		if (Switch2_Pressed()){
			// wait a bit to avoid push being registered twice
			Clock_Delay_n_ms(1000,HIGH_CLOCK_SPEED);	
			/*  if car is going from disarmed to armed, do a countdown to go using the LEDs and then resume indicating the current state*/
			if (!carArmed){
				BYTE prevColor = LED2_GetColor();
				LED2_SetColor(RED);
				Clock_Delay_n_ms(500,HIGH_CLOCK_SPEED); // half a second delay
				LED2_SetColor(YELLOW);
				Clock_Delay_n_ms(500,HIGH_CLOCK_SPEED); // half a second delay
				LED2_SetColor(GREEN);
				Clock_Delay_n_ms(500,HIGH_CLOCK_SPEED); // half a second delay
				LED2_SetColor(prevColor);
				/* red LED on when car is armed*/
				LED1_On();
			} else{
				LED1_Off();
			}
			carArmed = ~carArmed; //toggle state of the car being armed
		}
		#endif

		#ifdef MODE_SWITCHING
		if(Switch1_Pressed()){
			// wait a bit to avoid push being registered twice
			Clock_Delay_n_ms(300,HIGH_CLOCK_SPEED);	
			if(thisCarState.attackMode == reckless){
				thisCarState.attackMode = balanced; // go to next mode
			} else if(thisCarState.attackMode == balanced){
				thisCarState.attackMode = conservative; 
			} else{
				// wrap around to starting mode 
				thisCarState.attackMode = reckless;
			}
			modeLEDUpdated = FALSE; //need to update mode LED
		}

		// updates LED2 color based on mode selected
		if (!modeLEDUpdated){
			switch (thisCarState.attackMode){
				case reckless:
					LED2_SetColor(MAGENTA);
					thisCarSettings = recklessMode;
					break;
				case balanced:
					LED2_SetColor(WHITE);
					thisCarSettings = balancedMode;
					break;
				case conservative:
					LED2_SetColor(BLUE);
					thisCarSettings = conservativeMode;
					break;
				default:
					LED2_SetColor(RED);
					thisCarSettings = balancedMode;
					break;
			}
			modeLEDUpdated = TRUE;
		}
		#endif

		#ifdef CAR_ARMING
		if(carArmed){
		#endif
			// do processing when the camera is sending data
			if(g_sendData== TRUE){
				smooth_line(line,smoothLine);
				trackCenterIndex = get_track_center(smoothLine);
				thisCarState.magnitudeVCM = smoothLine[trackCenterIndex];
				carOnTrack = get_on_track(thisCarState.magnitudeVCM, thisCarSettings.vcmThreshold);
			} 

			#ifdef RACECAR_STATE_MACHINE
			// TODO maybe just end up scaling turn angle and speed based on magVCM
			if(thisCarState.magnitudeVCM > THRESHOLD_STRAIGHT){
				thisCarState.trackPosition = straight;
			} else if(thisCarState.magnitudeVCM > THRESHOLD_NORMAL){
				thisCarState.trackPosition = normal;
			} else if(thisCarState.magnitudeVCM > THRESHOLD_APPROACH){
				thisCarState.trackPosition = approachingTurn;
			} else if(thisCarState.magnitudeVCM > THRESHOLD_TURNING){
				thisCarState.trackPosition = turning;
			} else if (thisCarState.magnitudeVCM > THRESHOLD_EDGE){
				thisCarState.trackPosition = trackEdge;
			} else{
				thisCarState.trackPosition = richardHammond;
			}

			#ifdef RSM_LEDS
			BYTE ledColor;
			switch (thisCarState.trackPosition)
			{
			case straight:
				ledColor = RED;
				break;
			case normal:
				ledColor = YELLOW;
				break;
			case approachingTurn:
				ledColor = GREEN;
				break;
			case turning:
				ledColor = CYAN;
				break;
			case trackEdge:
				ledColor = BLUE;
				break;
			case richardHammond:
				ledColor = MAGENTA;
				break;
			
			default:
				break;
			}
			LED2_SetColor(ledColor);
			#endif
			#endif

			// turn the servo towards the center of the track
			#ifndef USE_PID_STEERING
			// just use regular steering method
			steer_to_center(trackCenterIndex);
			#else 
			// use PID steering
			// // lets tell PID that positive and negative exist
			// // convert from value between 0 and 127 to -60 to 60
			roughCenter = -(trackCenterIndex - 62); // should be 64 but tuned lol
			scaledCenter = roughCenter * 4;
			// bound steering 
			if (scaledCenter < -60){ scaledCenter = -60;}
			if (scaledCenter > 60){ scaledCenter = 60;}
			PIDRes = SteeringPID(scaledCenter);
			iPIDRes = (int16_t) PIDRes;	
			// steer to calculated point
			set_steering_deg(iPIDRes);
			#endif

			if(carOnTrack){
				#ifdef ON_TRACK_LEDS
				LED2_SetColor(GREEN);
				#endif
				#ifndef DISABLE_DRIVE_MOTORS
				DC_motors_enable();
				motors_move(thisCarSettings.normalSpeed, FWD);
				#endif
				
			} else{
				//we are off the track
				#ifdef ON_TRACK_LEDS
				LED2_SetColor(RED);
				#endif
				#ifndef DISABLE_DRIVE_MOTORS
				stop_DC_motors();
				#endif
			}
		#ifdef CAR_ARMING
		} else{
			stop_DC_motors();
			set_steering_deg(0); // set servos to straight ahead
		}
		#endif
	}
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

///* initialize state variables to starting values */
//motorDir = FWD;
//motorSpeed = NORMAL_SPEED;
//turnAngle = 0;
//edgeNear = 0;

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
