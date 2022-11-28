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
	float new_val;
	pidN.error = intended - current;

	// informed by lecture 21 - control -  slide 35
	new_val = current +
		pidN.kp * (pidN.error - pidN.error_n1) +
		pidN.ki * (pidN.error + pidN.error_n1)/2.0 *
		pidN.kd * (pidN.error - 2.0*pidN.error_n1 + pidN.error_n2);

	// update struct
	pidN.error_n2 = pidN.error_n1;
	pidN.error_n1 = pidN.error;
	pidN.val_n1 = new_val; 
	
	return new_val;
}

