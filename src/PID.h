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

#define KP_STEER (0.99)
#define KI_STEER (0.3)
#define KD_STEER (0.0)

// typedef struct pid_tune{
// 	float kp;
// 	float ki;
// 	float kd;
// } pid_tune_t;

// typedef struct pid_hist{
// 	float error_n1;		// last error
// 	float error_n2; 		// two errors ago
// 	float setPoint_n1;	// last setpoint 
// } pid_hist_t;


// /* Function prototypes */
// float ReusePID(pid_tune_t pidTune, pid_hist_t* pidHist_p, float current, float intended);
void	steering_pid_init(void);
float SteeringPID(float intended);

 #endif
