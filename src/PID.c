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


float ImplementPID(pid_nums_t pidN, float current, float intended){
	float setPoint, error;
	error = intended - current;

	// informed by lecture 21 - control -  slide 35
	setPoint = current +
		pidN.kp * (error - pidN.error_n1) +
		pidN.ki * (error + pidN.error_n1)*0.5f *
		pidN.kd * (error - 2.0f*pidN.error_n1 + pidN.error_n2);

	// update struct
	pidN.error_n2 = pidN.error_n1;
	pidN.error_n1 = error;
	pidN.setPoint_n1 = setPoint; 
	
	return setPoint;
}

