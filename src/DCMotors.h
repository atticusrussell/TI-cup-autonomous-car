/**
 * @file DCMotors.h
 * @authors Erica Coles, Atticus Russell
 * @brief DC Motor header file for IDE msp432 car project
 * @version 0.1
 * 
 * 
 */

#include <stdint.h>
#include "Common.h"

#ifndef	_DC_MOTOR_HEADER_FILE_
#define	_DC_MOTOR_HEADER_FILE_

// constant params // TODO tweak as needed
// maximum motor duty cycle to use during straights
#define MAX_SPEED 		(40) //NOTE untested 
// normal forward motor duty cycle
#define STUPID_SPEED			(48)

#define RECKLESS_SPEED		(46)
#define NORMAL_SPEED 		(40) // DONT CHANGE  40 THIS WORKS VCM 3000 
// 27 too slow to get up the lil hill
#define CONSERVATIVE_SPEED 		(35)	// works - barely gets up hill but does

// function prototypes
void stop_DC_motors(void);
void DC_motors_init(void);
void DC_motors_enable(void);
void DC_motors_disable(void);
double bound_speed(double speed);
void left_motor_move(double speed, int direction);
void right_motor_move(double speed, int direction);
void motors_move(double speed, int direction);


#endif
