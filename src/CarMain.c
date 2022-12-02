/**
 * @file		CarMain.c
 * @author 	Atticus Russell
 * @author 	Erica Coles
 * @details Main file to control CMPE-460 (IDE) car project on MSP432 
 * @date 	Fall 2022 semester RIT
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
// #define DISABLE_DRIVE_MOTORS // NOTE COMMENT THIS OUT TO RUN for real // TODO eliminate this because of the car arming feature 

#define MODE_SWITCHING
#define CAR_ARMING
// #define ON_TRACK_LEDS

#define RACECAR_STATE_MACHINE
#define RSM_LEDS
// #define RSM_EVASIVE_MAN

/* define constants that are important parameters to tune */
// turn increments (tuning how hard we turn) unitless scalar
#define TURN_INCREMENT 	(3)

// #define DIFFERENTIAL_STEERING

#define IW_ANGLE_MULTIPLY (0.3)
#define OW_ANGLE_MULTIPLY (0.1)

#define SPEED_SCALE_VCM_DIVISOR (7000)





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
// [x] Reduce (tune) minimal visual center of mass so that 2 wheels off track
// [x] fix issue with stopping while still on track
// [x] Create modes with different levels of aggression/ motor DC and min VCM
// [x] create states for approaching corner, in corner, leaving corner,
// [x] use the visual mass to determine corners vs straights and set drive speed
// [x] use switches to select different track mode
// [x] use LED to display track mode
// [x] swap the RSM_SPEED to a boolean and use it only for reckless mode
// [x] when in edge state turn the servo to its max 
// [ ] implement differential drive for sharper turning - esp. near edge

// [ ] test evasive steering
// [ ] test differential steering
// [ ] tune differential steering
// [ ] test different modes and optimize

// optional/ideas
// [ ] use a scalar multiplier for the different aggression modes
// [ ] determine which edge closer when almost off track - spin that wheel faster to get back on 
// [ ] convert to follow C best practices from IDE handbook
// NOTE i heard "0.45 highest DC for motors that won't blow them up"
//	even if that's not true it may be the highest sensible setting

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
 * @return 
 */
int8_t steer_to_center(uint8_t trackCenter){
	int8_t steerAng;	// ang deg center of car to center of track
	// if less than 64 will be negative and left, otherwise pos and right
	steerAng = 64 - trackCenter;
	// 64 is close enough to 60. 
	// if it is greater than 60 or less than -60 bounding func will catch
	return set_steering_deg(steerAng*TURN_INCREMENT);
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


/**
 * @brief Checks to see if arming button pressed and arm/disarm accordingly. Sets LEDs also
 * 
 * @note if unarmed->armed does a countdown with red->YLW->grn and arms
 * @note LED1 is off if armed, and on when not
 * @param carArmed whether car is already armed
 * @return BOOLEAN the new armed/disarmed state
 */
BOOLEAN checkArmingButton(BOOLEAN carArmed){
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
	return carArmed;
}


int main(void){
	/* camera variables*/
	extern uint16_t line[128];			// current array of raw camera data
	uint16_t smoothLine[128];	// camera data filtered by smoothing
	uint8_t	trackCenterIndex;	// camera index of the center of the track
	//uint16_t trackCenterValue;	// value of the camera at center of track
	BOOLEAN 	carOnTrack;			// true if car is on the track, otherwise false
	extern BOOLEAN	g_sendData;			// if the camera is ready to send new data


	// the direction of the motor -> pin to write to i think
	enum motorDir{
		FWD = 0,
		REV = 1
	};

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
	};

	BYTE stateLEDColor;
	#endif

	#ifdef MODE_SWITCHING
	enum speedSetting{
		reckless = 0,
		balanced = 1,
		conservative = 2
	};

	struct carSettingsStruct{
		int normalSpeed;
		//TODO remove this probably or switch it for a scalar to * speed by
		int maxSpeed;	// the absolute max speed the car will do on straights 
		int vcmThreshold; // what the car counts as the track edge
		BOOLEAN useSpeedScale; // whether to use statespeed
		BOOLEAN useDiffSteering;
	};

	/* create each of the modes*/
	struct carSettingsStruct recklessMode = {RECKLESS_SPEED,MAX_SPEED,TUNING_ON_TRACK_VCM,TRUE,TRUE};
	struct carSettingsStruct balancedMode = {NORMAL_SPEED,MAX_SPEED, NORMAL_VCM,TRUE,TRUE};
	struct carSettingsStruct conservativeMode = {CONSERVATIVE_SPEED, MAX_SPEED, CONSERVATIVE_VCM, FALSE, FALSE};

	// var that stores the state of the car
	struct carStateStruct{
		/* angle of wheels in deg with 0 = straight, + right - left.for servo */
		int8_t steeringAngle; // will only go from -60 to 60
		/* number from 0 to 100 to be converted into duty cycle for motor*/
		// int motorSpeed;
		enum speedSetting attackMode;
		enum jeremyClarkson trackPosition;
		int setSpeed; //the speed that the car is currently set to
		int16_t magnitudeVCM; // value of the camera at the center of the track
	};

	/* instantiate carState and decide how car should default*/
	struct carStateStruct carState;
	carState.attackMode = balanced; //choose the starting mode
	BOOLEAN modeLEDUpdated = FALSE; //allows the initial mode to be displayed
	carState.trackPosition = normal;
	carState.steeringAngle = 0; // start straight ahead
	/* instantiate carSettings and choose the default*/
	struct carSettingsStruct carSettings = recklessMode;
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

	/* differential steering*/
	// TODO make struct diffsteering
	int innerWheelSpeed;
	int outerWheelSpeed;
	int baseSpeed;
	int absAngle;


	// BOOLEAN useSpeedScale = TRUE;
	double speedFactor = 1.0;

	initCarParts();
	// infinite loop to contain logic
	while(1){
		#ifdef CAR_ARMING
		carArmed = checkArmingButton(carArmed);
		#endif

		#ifdef MODE_SWITCHING
		if(Switch1_Pressed()){
			// wait a bit to avoid push being registered twice
			Clock_Delay_n_ms(300,HIGH_CLOCK_SPEED);	
			if(carState.attackMode == reckless){
				carState.attackMode = balanced; // go to next mode
			} else if(carState.attackMode == balanced){
				carState.attackMode = conservative; 
			} else{
				// wrap around to starting mode 
				carState.attackMode = reckless;
			}
			modeLEDUpdated = FALSE; //need to update mode LED
		}

		// updates LED2 color based on mode selected
		if (!modeLEDUpdated){
			switch (carState.attackMode){
				case reckless:
					LED2_SetColor(MAGENTA);
					carSettings = recklessMode;
					break;
				case balanced:
					LED2_SetColor(WHITE);
					carSettings = balancedMode;
					break;
				case conservative:
					LED2_SetColor(BLUE);
					carSettings = conservativeMode;
					break;
				default:
					LED2_SetColor(RED);
					carSettings = balancedMode;
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
				carState.magnitudeVCM = smoothLine[trackCenterIndex];
				carOnTrack = get_on_track(carState.magnitudeVCM, carSettings.vcmThreshold);
			} 

			#ifdef RACECAR_STATE_MACHINE
			// TODO maybe just end up scaling turn angle and speed based on magVCM
			if(carState.magnitudeVCM > THRESHOLD_STRAIGHT){
				carState.trackPosition = straight;
			} else if(carState.magnitudeVCM > THRESHOLD_NORMAL){
				carState.trackPosition = normal;
			} else if(carState.magnitudeVCM > THRESHOLD_APPROACH){
				carState.trackPosition = approachingTurn;
			} else if(carState.magnitudeVCM > THRESHOLD_TURNING){
				carState.trackPosition = turning;
			} else if (carState.magnitudeVCM > THRESHOLD_EDGE){
				carState.trackPosition = trackEdge;
			} else{
			// TODO add richardHammond as part of RSM / usse it for "off track"
				carState.trackPosition = richardHammond;
			}

			switch (carState.trackPosition){
				case straight:
					stateLEDColor = RED;
					// carState.setSpeed = STRAIGHT_SPEED;
					break;
				case normal:
					stateLEDColor = YELLOW;
					// carState.setSpeed = NORMAL_SPEED;
					break;
				case approachingTurn:
					stateLEDColor = GREEN;
					// carState.setSpeed = APPROACH_SPEED;
					break;
				case turning:
					stateLEDColor = CYAN;
					// carState.setSpeed = TURNING_SPEED;
					break;
				case trackEdge:
					stateLEDColor = BLUE;
					// carState.setSpeed = EDGE_SPEED;
					break;
				case richardHammond:
					stateLEDColor = MAGENTA;
					// carState.setSpeed = HAMMOND_SPEED;
					break;
				
				default:
					break;
			}




			#ifdef RSM_LEDS
			LED2_SetColor(stateLEDColor);
			#endif // ifdef RSM_LEDS
			#endif // ifdef RACECAR_STATE_MACHINE

			#ifdef RSM_EVASIVE_MAN
			/* turn all the way towards the center of the track in current dir if near the track edge */
			if(carState.trackPosition == trackEdge){
				carState.steeringAngle = set_steering_deg(sign(carState.steeringAngle)*SERVO_MAX_ANGLE_DEG);
			} else{
			#endif // RSM_EVASIVE_MAN
				// turn the servo towards the center of the track
				#ifndef USE_PID_STEERING
				// just use regular steering method
				carState.steeringAngle = steer_to_center(trackCenterIndex);
				#else 
				// use PID steering
				// // lets tell PID that positive and negative exist
				// // convert from value between 0 and 127 to -60 to 60
				roughCenter = -(trackCenterIndex - 62); // should be 64 but tuned
				scaledCenter = roughCenter * 4;
				// bound steering 
				if (scaledCenter < -60){ scaledCenter = -60;}
				if (scaledCenter > 60){ scaledCenter = 60;}
				PIDRes = SteeringPID(scaledCenter);
				iPIDRes = (int16_t) PIDRes;	
				// steer to calculated point
				carState.steeringAngle =  set_steering_deg(iPIDRes);
				#endif
			#ifdef RSM_EVASIVE_MAN
			}
			#endif // RSM_EVASICE_MAN

			if(carSettings.useSpeedScale){
			// scale the speed by how straight it is 
			speedFactor = carState.magnitudeVCM / SPEED_SCALE_VCM_DIVISOR;
			carState.setSpeed = carSettings.normalSpeed * speedFactor;
			}

			if(carOnTrack){
				#ifdef ON_TRACK_LEDS //TODO swap to #ifndef RACECAR_STATE_MACHINE and then implement this identical functionality in RSM too
				LED2_SetColor(GREEN);
				#endif

				#ifndef DISABLE_DRIVE_MOTORS
				DC_motors_enable();
				if(carSettings.useDiffSteering){
					absAngle = abs(carState.steeringAngle);

					if(carSettings.useSpeedScale){
						baseSpeed = carState.setSpeed;
					} else{
						baseSpeed = carSettings.normalSpeed;
					}

					innerWheelSpeed = baseSpeed - absAngle*IW_ANGLE_MULTIPLY;
					outerWheelSpeed = baseSpeed + absAngle*OW_ANGLE_MULTIPLY;

					if (sign(carState.steeringAngle)<0){
						//left turn
						left_motor_move(innerWheelSpeed, FWD);
						right_motor_move(outerWheelSpeed, FWD);
					} else{
						// right turn or straight
						left_motor_move(outerWheelSpeed, FWD);
						right_motor_move(innerWheelSpeed, FWD);
					}
				} else{
					if(carSettings.useSpeedScale){

						motors_move(carState.setSpeed, FWD);
					} else{
						motors_move(carSettings.normalSpeed, FWD);
					}
				}
				#endif //ifndef DISABLE_DRIVE_MOTORS
				
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
