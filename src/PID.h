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

//TODO tune me
//  try next 0.05, 0.025, 0.045
#define KP_STEER (0.05) // 0.05
#define KI_STEER (0.025) // 0.025
#define KD_STEER (0.0)

#define PID_STEERING_ERROR_ENTRIES	(3) //TODO tune me


// /* Function prototypes */
// float ReusePID(pid_tune_t pidTune, pid_hist_t* pidHist_p, float current, float intended);
void	steering_pid_init(void);
float SteeringPID(float intended, float actual);

 #endif
