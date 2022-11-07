/*
 * Author:		Erica Coles
 * Date: 		10/12/2022
 * File:        TimerA.h
 * Purpose:     header file for TimerA 
 * Exercise:	Lab6
 *
 * Notes:		
 *
 */
 
#include <stdint.h>
#include "Common.h"


#ifndef _TIMERA_HEADER_FILE_
#define _TIMERA_HEADER_FILE_
 
 /* Function prototypes */
 int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);
 void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);
 int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);
 void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);
 
 #endif
