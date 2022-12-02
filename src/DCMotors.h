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
#define MAX_SPEED 		(65) //NOTE untested 
// normal forward motor duty cycle

#define STRAIGHT_SPEED		(45)
#define NORMAL_SPEED 		(35) // DONT CHANGE  THIS WORKS VCM 3000 
#define APPROACH_SPEED		(35)	// NOTE too fast to get around corners ATM
#define TURNING_SPEED		(30)	// NOTE untested
#define EDGE_SPEED			(27)
#define HAMMOND_SPEED		(25)


#define RECKLESS_SPEED		(40)

// 27 too slow to get up the lil hill
#define CONSERVATIVE_SPEED 		(30)	// works - barely gets up hill but does

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
