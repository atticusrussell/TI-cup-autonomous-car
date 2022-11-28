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
 * @param pidHist_p 	pointer to PID struct
 * @param current  current value of the output
 * @param intended desired value of the output
 * @return float new setpoint
 */
float ReusePID(pid_tune_t pidTune, pid_hist_t* pidHist_p, float current, float intended){
	
	float setPoint, error;
	error = intended - current;

	// informed by lecture 21 - control -  slide 35
	setPoint = current +
		pidTune.kp * (error - pidHist_p->error_n1) +
		pidTune.ki * (error + pidHist_p->error_n1)*0.5f +
		pidTune.kd * (error - 2.0f*pidHist_p->error_n1 + pidHist_p->error_n2);

	// update struct
	pidHist_p->error_n2 = pidHist_p->error_n1;
	pidHist_p->error_n1 = error;
	pidHist_p->setPoint_n1 = setPoint; 
	
	return setPoint;
}


/**
 * @brief servo steering specific implementation of PID
 * 
 * @param pidHist_p pointer to the PID struct for the servo
 * @param current current value of the servo pointing
 * @param intended desired value of the servo pointing
 * @return float new setpoint 
 */
float SteeringPID(pid_tune_t pidTune, pid_hist_t* pidHist_p, float current, float intended){
	float setPoint = ReusePID(pidTune, pidHist_p, current, intended);
	// prevent integral overshoot/ runaway through bounding
	// TODO try not bounding this and see what happens
	setPoint = fbound_steering_angle(setPoint);
	pidHist_p->setPoint_n1 = setPoint;
	return setPoint;
}
