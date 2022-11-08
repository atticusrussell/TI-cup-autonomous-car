/**
 * @file CarDefines.h
 * @authors Erica Coles, Atticus Russell
 * @brief header file for IDE msp432 car project
 * @version 0.1
 * 
 * 
 */

#include <stdint.h>
#include "Common.h"

#ifndef	_CAR_HEADER_FILE_
#define	_CAR_HEADER_FILE_

// constant params to tweak as needed
// maximum motor duty cycle
#define MAX_SPEED 		(65.0)
// normal forward motor duty cycle
#define NORMAL_SPEED 	(35)
// slower motor duty cycle for turns
#define TURN_SPEED 		(5)
// turn increments (tuning how hard we turn) - unitless rn
#define TURN_INCREMENT 	(3)

#endif
