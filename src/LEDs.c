/*
 * File:        LEDs.c
 * Purpose:     Provide routines for LEDs on msp432r
 *
 * Notes:		
 * Author: Atticus Russell
 * Date: 10/3/2022
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "msp.h"
#include "leds.h"	// file with the function prototypes
#include "Common.h" // from Lab1 code

// ajr NOTE: red LED P2.0, green LED P2.1, blue LED P2.2
void LED1_Init(void)
{
	// configure PortPin for LED1 as port I/O
	P1->SEL0 &= ~BIT0; // select Port1 Bit0 for use as port (simple) I/O
	P1->SEL1 &= ~BIT0; // select Port1 Bit1 for use as port (simple) I/O

	// make built-in LED1 high drive strength
	P1->DS |= BIT0;

	// make built-in LED1 out
	P1->DIR |= BIT0; // set Bit0 as output

	// turn off LED
	P1->OUT &= ~BIT0; // Set P1.0 to '0'
}

void LED2_Init(void)
{
	// configure PortPin for LED2 as port I/O
	P2->SEL0 &= ~7; // select Port2 Bit0 for use as port (simple) I/O
	P2->SEL1 &= ~7; // select Port2 Bit1 for use as port (simple) I/O

	// make built-in LED2 LEDs high drive strength
	P2->DS |= 7;

	// make built-in LED2 out
	P2->DIR |= 7; // set Bit0 as output

	// turn off LED
	P2->OUT &= ~7; // Set P2.0 to '0'
}

BOOLEAN LED1_State(void){
	return (P1->OUT & BIT0);
}

// BOOLEAN LED2_State(void)
// {
// 	return (P1->OUT & BIT0);
// }

void LED1_On(void){
	P1->OUT |= 1;
}

void LED1_Off(void)
{
	P1->OUT &= ~1;
}

BYTE LED2_GetColor(void){
	return (P2->OUT & BIT7);
}

void LED2_SetColor(BYTE chosenColor){
	P2->OUT &= 0;
	P2->OUT |= chosenColor;
}
