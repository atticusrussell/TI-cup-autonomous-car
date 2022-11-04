/*
 * Author:		Atticus Russell
 * Date: 		10/03/2022
 * File:        switches.h
 * Purpose:     header file for switch routines on MSP432r Launchpad
 * Exercise:	Lab05
 * Class:		CMPE460
 * School:		RIT
 *
 * Notes:		
 *
 */

#include "Common.h"

#ifndef _SWITCH_HEADER_FILE_
#define _SWITCH_HEADER_FILE_

/* Function prototypes */
void Switch1_Init(void);
void Switch2_Init(void);
BOOLEAN Switch1_Pressed(void);
BOOLEAN Switch2_Pressed(void);

#endif
