/**
 * @file PID.h
 * @author Atticus Russell
 * @brief header file for IDE MSP432 car PID control
 * @version 0.1
 * 
 * 
 */
 
#ifndef _PID_HEADER_FILE_
#define _PID_HEADER_FILE_

typedef struct pid_nums{
	float kp;
	float ki;
	float kd;
	float error_n1;		// last error
	float error_n2; 		// two errors ago
	float setPoint_n1;	// last setpoint 
} pid_nums_t;


/* Function prototypes */
float ImplementPID(pid_nums_t pidN, float current, float intended);

 #endif
