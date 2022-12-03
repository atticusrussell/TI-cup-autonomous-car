/**
 * @file		CarMain.c
 * @author 	Atticus Russell
 * @author 	Erica Coles
 * @details Main file to control CMPE-460 (IDE) car project on MSP432 
 * @date 	Fall 2022 semester RIT
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // abs() function
#include "msp.h"
#include "uart.h"
#include "Common.h"
#include "ServoMotor.h"
#include "DCMotors.h"
#include "CortexM.h"
#include "Camera.h"
#include	"LEDs.h"
#include "ControlPins.h" // camera
#include "PID.h"
#include "switches.h" // mode switching
#include "Timer32.h" // one-shot timer for off-track

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
#define OG_TURN_SCALAR 	(3) // for the old camera angle
#define STUPID_TURN_SCALAR (2.4)
#define TUNED_TURN_SCALAR (2.4) // for the new camera angle to make less twitchy

// #define DIFFERENTIAL_STEERING

#define IW_ANGLE_MULTIPLY (0.3)
#define OW_ANGLE_MULTIPLY (0.05)

#define STUPID_IW_ANGLE_MULTIPLY (0.33)
#define STUPID_OW_ANGLE_MULTIPLY (0.055)

#define SPEED_SCALE_VCM_DIVISOR (4000) //used to be 7000

// #define MIN_TURN_SPEED 25
// #define MIN_SPEED 27

// #define NO_BRAKES

/* PID stuff */




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
// [x] implement differential drive for sharper turning - esp. near edge
// [x] make servo steering less twitchy with adjusted camera
// [x] test differential steering
// [x] tune differential steering

// [x] I NEED PID - TUNE IT because I need to use the error history
// [ ] test and tune PID for reckless mode


// [ ] create "data lost" state below a certain VCM where you stop updating values and just move the car
// [ ] when go "off track" keep going with historical servo angle
// [x] configure Timer32_2 to run in one-shot mode
// [x] figure out how to use generated interrupt to stop the car delayed 
// [ ] implement it well 
// NOTE even without onTrack detection turned on it still stops - param adjust



// [ ] create error history
// [ ] tie "off track" to RSM 
// [ ] with adjusted camera keep history of center and detect off track based on that

// [ ] test different modes and optimize

// optional/ideas
// [ ] use a scalar multiplier for the different aggression modes
// [ ] determine which edge closer when almost off track - spin that wheel faster to get back on 
// [ ] convert to follow C best practices from IDE handbook
// NOTE i heard "0.45 highest DC for motors that won't blow them up"
//	even if that's not true it may be the highest sensible setting


/* global variables and justifications*/
// needs to be global because accessed by IRQ function
	BOOLEAN headlessModeActive = FALSE;

// needs to be global to be accessed in the differential steering function
enum motorDir{ // the direction of the motor 
	FWD = 0,
	REV = 1
};

// // needs to be global because accessed by many functions and
// double steerPIDErrHistArray[PID_STEERING_ERROR_ENTRIES];




/**
 * @brief steers the servo to the center of the track
 * 
 * @param trackCenterIndex number from 1 to 128 that indicates the track center
 * @return 
 */
int8_t steer_to_center(uint8_t trackCenterIndex, double steerScalar, BOOLEAN usePIDServo){
	/* ang deg center of car to center of track. if less than 64 will be negative and left, otherwise pos and right */
	// 64 is close enough to 60. 
	int8_t unscaledSteerAng = 64 - trackCenterIndex;
	// run through trusty proportion
	double scaledUnboundedSteeringAng = unscaledSteerAng*steerScalar;
	// if it is greater than 60 or less than -60 bounding func will catch
	double scaledSteeringAng = fbound_steering_angle(scaledUnboundedSteeringAng);
	//NOTE - ^ this is a change - if non-PID steering becomes bad undo it 

	if(usePIDServo){
	double straightAheadDeg = 0.0;
		// use PID steering
		// TODO implement PID call here
		// NOTE maybe just use steerScalar as kp
		// TODO consider only returning the non-PID val if things don't work
		// intermediate var for visibility
		double PIDRes = SteeringPID(straightAheadDeg ,scaledSteeringAng);
		// steer to calculated point - cast to int
		return set_steering_deg((int16_t) PIDRes);
		
	} else{
		// if not using PID - just set steering to angle calculated
		return set_steering_deg(scaledSteeringAng);
	}
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


/**
 * @brief called by the Timer32 IRQ after it expires - stops the car
 * 
 */
void endHeadlessMode(void){
	headlessModeActive = FALSE;
}


/**
 * @brief implements motor differential thrust to assist with steering
 * 
 * @param steeringAngle the angle that the servos are already set to steer to
 * @param baseSpeed 		the baseline speed to modify per motor
 */
void runDiffThrust(int8_t steeringAngle, double baseSpeed, double innerWheelAngleMultiple, double outerWheelAngleMultiple){
	double innerWheelSpeed;
	double outerWheelSpeed;
	int absAngle = abs(steeringAngle);
	
	innerWheelSpeed = baseSpeed - absAngle*innerWheelAngleMultiple;
	outerWheelSpeed = baseSpeed + absAngle*outerWheelAngleMultiple;
	#ifdef MIN_SPEED
	if(baseSpeed<MIN_SPEED){
		outerWheelSpeed = MIN_SPEED;
	}
	#endif

	if (sign(steeringAngle)<0){
		//left turn
		left_motor_move(innerWheelSpeed, FWD);
		right_motor_move(outerWheelSpeed, FWD);
	} else{
		// right turn or straight
		left_motor_move(outerWheelSpeed, FWD);
		right_motor_move(innerWheelSpeed, FWD);
	}
}


int main(void){
	/* camera variables*/
	extern uint16_t line[128];			// current array of raw camera data
	uint16_t smoothLine[128];	// camera data filtered by smoothing
	uint8_t	trackCenterIndex;	// camera index of the center of the track
	BOOLEAN 	carOnTrack;			// true if car is on the track, otherwise false
	extern BOOLEAN	g_sendData;			// if the camera is ready to send new data


	
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
		int vcmThreshold; // what the car counts as the track edge
		BOOLEAN useSpeedScale; // whether to use statespeed
		BOOLEAN useDiffThrust;
		double steeringScalar;
		BOOLEAN enableHeadless;
		BOOLEAN usePIDServo;
		double innerWheelAngleMultiple;
		double outerWheelAngleMultiple;
	};

	/* create each of the modes*/
	struct carSettingsStruct recklessMode = {CONSERVATIVE_SPEED,NORMAL_VCM,TRUE,TRUE, OG_TURN_SCALAR, FALSE, FALSE, IW_ANGLE_MULTIPLY, OW_ANGLE_MULTIPLY};
	struct carSettingsStruct balancedMode = {CONSERVATIVE_SPEED, NORMAL_VCM,FALSE,TRUE, OG_TURN_SCALAR, FALSE, FALSE, IW_ANGLE_MULTIPLY,OW_ANGLE_MULTIPLY};
	struct carSettingsStruct conservativeMode = {CONSERVATIVE_SPEED, CONSERVATIVE_VCM, FALSE, FALSE, OG_TURN_SCALAR, FALSE, FALSE, IW_ANGLE_MULTIPLY, OW_ANGLE_MULTIPLY};

	// var that stores the state of the car
	struct carStateStruct{
		/* angle of wheels in deg with 0 = straight, + right - left.for servo */
		int8_t steeringAngle; // will only go from -60 to 60
		enum speedSetting attackMode;
		enum jeremyClarkson trackPosition;
		/* number from 0 to 100 to be converted into duty cycle for motor*/
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
	#endif // MODE_SWITCHING


	/* differential thrust*/
	double desiredSpeed;

	/* speed scale (initialize to harmless value of 1)*/
	double speedFactor = 1.0; 


	/* vars to handle losing VCM but not being all the way off the track*/
	// make struct lol
	// make instance of carStateStruct that will contain last state
	struct carStateStruct  lastCarState;
	int msHeadlessChicken = 100 ; // amount of time it will continue with last data before cutting off
	
	BOOLEAN onTrackBuffer;
	

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

		// TODO make into a function
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
				onTrackBuffer = get_on_track(carState.magnitudeVCM, carSettings.vcmThreshold);
			} 

			#ifdef RACECAR_STATE_MACHINE
			// TODO make into a function
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

			// TODO make into a function
			switch (carState.trackPosition){
				case straight:
					stateLEDColor = RED;
					break;
				case normal:
					stateLEDColor = YELLOW;
					break;
				case approachingTurn:
					stateLEDColor = GREEN;
					break;
				case turning:
					stateLEDColor = CYAN;
					break;
				case trackEdge:
					stateLEDColor = BLUE;
					break;
				case richardHammond:
					stateLEDColor = MAGENTA;
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
				carState.steeringAngle = steer_to_center(trackCenterIndex, carSettings.steeringScalar, carSettings.usePIDServo);
			#ifdef RSM_EVASIVE_MAN
			}
			#endif // RSM_EVASICE_MAN

			if(carSettings.useSpeedScale){
				// scale the speed by how straight it is 
				speedFactor = carState.magnitudeVCM / SPEED_SCALE_VCM_DIVISOR;
				carState.setSpeed = carSettings.normalSpeed * speedFactor;
				// create a min speed during turns
				#ifdef MIN_TURN_SPEED
				if ((carState.attackMode >= trackEdge) && carState.setSpeed < MIN_TURN_SPEED){
					carState.setSpeed = MIN_TURN_SPEED;
				}
				#endif
			}


			// NOTE WORK IN PROGRESS
			/* figure out headless and carOnTrack */
			if(carSettings.enableHeadless){
				if(headlessModeActive){
				// do headless stuff
				carState = lastCarState;

				} else{
					// not headless mode
					if(onTrackBuffer){
						// if not headless mode and on track
						// only update carOnTrack if headless mode not active
						carOnTrack = onTrackBuffer;
						
						// update lastcarstate
						lastCarState = carState;
						/* FUTURE if lastCarState tests pointed wrong - array of them and access a prev. */
					}else{
						/* not already in headless mode but "off track" */
						// start the timer to run with current settings for a time
						Timer32_2_Init(*endHeadlessMode, msHeadlessChicken, T32DIV1);
						headlessModeActive = TRUE;
					}
				}
			} else{
				carOnTrack = onTrackBuffer;
			}
		
			// NOTE actual calls to movement here
			#ifndef NO_BRAKES
			if(carOnTrack){  
			#endif // ifndef NO_BRAKES
				DC_motors_enable();
				// determine baseline speed
				if(carSettings.useSpeedScale){
					desiredSpeed = carState.setSpeed;
				} else{
					desiredSpeed = carSettings.normalSpeed;
				}

				// Differential steering
				if(carSettings.useDiffThrust){
					runDiffThrust(carState.steeringAngle, desiredSpeed,carSettings.innerWheelAngleMultiple, carSettings.outerWheelAngleMultiple);
				} else{
				// regular drive both motors same speed
					motors_move(desiredSpeed, FWD);
				}
			#ifndef NO_BRAKES
			} else{
				//we are off the track
				stop_DC_motors();
			}
			#endif // ifndef NO_BRAKES
		#ifdef CAR_ARMING
		} else{
			stop_DC_motors();
			set_steering_deg(0); // set servos to straight ahead
		}
		#endif // CAR_ARMING
	}
}
