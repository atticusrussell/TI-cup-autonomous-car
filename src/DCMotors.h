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

// constant params to tweak as needed
// maximum motor duty cycle
#define MAX_SPEED 		(65.0)
// normal forward motor duty cycle
#define NORMAL_SPEED 	(35)
// slower motor duty cycle for turns
#define TURN_SPEED 		(5)

// TODO function prototypes
void stop_DC_motors(void);
void DC_motors_init(void);
void DC_motors_enable(void);
void DC_motors_disable(void);
double bound_speed(double speed);
void left_motor_move(double speed, int direction);
void right_motor_move(double speed, int direction);
void motors_move(double speed, int direction);


#endif
