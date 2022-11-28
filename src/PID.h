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
	float error;
	float error_n1;		// last error
	float error_n2; 	// two errors ago
	float val_n1;		// last value
} pid_nums_t;


 /* Function prototypes */ 

// void INIT_Camera(void);
// BOOLEAN get_on_track(uint16_t maxCamVal);
// uint8_t get_track_center(uint16_t* smoothData);
// void smooth_line(uint16_t* rawData, uint16_t* smoothData);

 #endif
