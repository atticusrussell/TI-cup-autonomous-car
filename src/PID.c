/**
 * @file PID.c
 * @author Atticus Russell 
 * @brief PID control for IDE MSP432 car
 * @version 0.1
 * @date November 2022
 * @class CMPE-460 Interface and Digital Electronics
 * 
 * 
 */

#include	<stdint.h>
#include "PID.h"
#include "ServoMotor.h"



/* global because I'm struggling to do this tastefully*/
// TODO maybe setpoint array??
float setPoint;
// float setPoint_n1; // this is the same as "actual"
float error, error_n1, error_n2;
// gets rid of implicit type conversion warning
float kpSteer = KP_STEER; 
float kiSteer = KI_STEER; 
float kdSteer = KD_STEER; 


double steerErrHistArray[PID_STEERING_ERROR_ENTRIES];



void steering_pid_init(void){
	for (int i=0; i < PID_STEERING_ERROR_ENTRIES; i++){
		steerErrHistArray[i] = 0.0;
	}
	setPoint = 0;
// 	setPoint_n1 = 0;
}


float SteeringPID(float intended, float actual){
	// grab errors from array
	error_n1 = steerErrHistArray[1];
	error_n2 = steerErrHistArray[2];

	error = intended - actual;
	setPoint = actual +
		kpSteer * (error) +
		kiSteer * (error + error_n1)*0.5f +
		kdSteer * (error - 2.0f * error_n1 + error_n2);

	// update history
	for (int i=1; i < PID_STEERING_ERROR_ENTRIES; i++){
		// shift the values to the right
		steerErrHistArray[i+1] = steerErrHistArray[i];
	}
	steerErrHistArray[0] = error;

	// setPoint_n1 = setPoint;
	return setPoint;
}
