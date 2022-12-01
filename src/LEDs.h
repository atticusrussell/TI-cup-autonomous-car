/*
 * Author:		Atticus Russell
 * Date: 		10/03/2022
 * File:        LEDs.h
 * Purpose:     header file for LED routines on MSP432r Launchpad
 * Exercise:	Lab05
 * Class:		CMPE460
 * School:		RIT
 *
 * Notes:		
 *
 */

#include "Common.h"

#ifndef _LED_HEADER_FILE_
#define _LED_HEADER_FILE_

#define RED 		0b001
#define BLUE 	 	0b100
#define GREEN 	 	0b010
#define CYAN 	 	0b110
#define MAGENTA	0b101
#define YELLOW 	0b011
#define WHITE 	 	0b111

/* Function prototypes */
void LED1_Init(void);
void LED2_Init(void);
BOOLEAN LED1_State(void);
//BOOLEAN LED2_State(void);
void LED1_On(void);
void LED1_Off(void);
BYTE LED2_GetColor(void);
void LED2_SetColor(BYTE);

#endif
