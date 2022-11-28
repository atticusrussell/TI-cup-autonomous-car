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

/**
 * @brief reusable implementation of PID algorithm
 * 
 * @param pidN_p 	pointer to PID struct
 * @param current  current value of the output
 * @param intended desired value of the output
 * @return float new setpoint
 */
float ReusePID(pid_nums_t* pidN_p, float current, float intended){
	float setPoint, error;
	error = intended - current;

	// informed by lecture 21 - control -  slide 35
	setPoint = current +
		pidN_p->kp * (error - pidN_p->error_n1) +
		pidN_p->ki * (error + pidN_p->error_n1)*0.5f *
		pidN_p->kd * (error - 2.0f*pidN_p->error_n1 + pidN_p->error_n2);

	// update struct
	pidN_p->error_n2 = pidN_p->error_n1;
	pidN_p->error_n1 = error;
	pidN_p->setPoint_n1 = setPoint; 
	
	return setPoint;
}


/**
 * @brief servo steering specific implementation of PID
 * 
 * @param pidN_p pointer to the PID struct for the servo
 * @param current current value of the servo pointing
 * @param intended desired value of the servo pointing
 * @return float new setpoint 
 */
float SteeringPID(pid_nums_t* pidN_p, float current, float intended){
	float setPoint = ReusePID(pidN_p, current, intended);
	// prevent integral overshoot/ runaway
	setPoint = bound_steering_angle(setPoint);
	return setPoint;
}
