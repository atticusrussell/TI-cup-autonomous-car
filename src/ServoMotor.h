/**
 * @file ServoMotor.c
 * @authors Erica Coles, Atticus Russell
 * @brief header file to help interface with servo steering IDE msp432 car
 * @version 0.1
 * 
 * 
 */
 
#include "Common.h"

#ifndef _SERVOMOTOR_HEADER_FILE_
#define _SERVOMOTOR_HEADER_FILE_

/* define constants for this  servo */
// the MG996r servo has a total travel of 120 degrees - 60 in each direction
#define MAX_ANGLE_DEG 	(60)
// the MG996r servo operates with a 50 Hz / 20 ms PWM period
#define PWM_FREQ_HZ 		(50)
#define PWM_PERIOD_MS	(20)


// the clock that this servo will be run at and to divide to find period
 #define SERVO_CLK 		(30000000)

 /* Function prototypes */
 int sign(int x);
 void servo_init(void);
 void set_servo_DC(double servoDC);
 void set_servo_deg(int16_t steeringAngle);

 
 #endif
