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



/* global shit because I'm struggling to do this tastefully*/
float setPoint;
float setPoint_n1;
float error, error_n1, error_n2;
// gets rid of implicit type conversion warning
float kpSteer = KP_STEER; 
float kiSteer = KI_STEER; 
float kdSteer = KD_STEER; 



void steering_pid_init(void){
	setPoint = 0;
	setPoint_n1 = 0;
	error = 0.0;
	error_n1 = 0.0;
	error_n2 = 0.0;
}


float SteeringPID(float intended){
	error = intended - setPoint_n1;
	setPoint = setPoint_n1 +
		kpSteer * (error - error_n1) +
		kiSteer * (error + error_n1)*0.5f +
		kdSteer * (error - 2.0f * error_n1 + error_n2);

	// update history
	error_n2 = error_n1;
	error_n1 = error;
	setPoint_n1 = setPoint;

	return setPoint;
}
