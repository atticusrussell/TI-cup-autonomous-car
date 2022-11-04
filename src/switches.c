/*
 * File:        switches.c
 * Purpose:     Provide routines for switches on msp432r
 *
 * Notes:
 * Author: Atticus Russell
 * Date: 10/3/2022
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "msp.h"
#include "switches.h"	// file with the function prototypes
#include "Common.h" // from Lab1 code

void Switch1_Init(void)
{
	// configure PortPin for Switch 1 as port (simple) I/O
	P1->SEL0 &= ~BIT1; // configue P1.1 as (simple) I/O
	P1->SEL1 &= ~BIT1;

	// configure switch pins as input
	P1->DIR &= ~BIT1; // set Pin1 as input

	// configure input with pull up resistor for Sw1
	P1->REN |= BIT1; /* P1.1 pull resistor enabled */
	P1->OUT |= BIT1; /* Pull up/down is selected by P1->OUT */
}

void Switch2_Init(void)
{
	// configure PortPin for Switch2 as port (simple) I/O
	P1->SEL0 &= ~BIT4; // configue P1.4 as (simple) I/O
	P1->SEL1 &= ~BIT4;

	// configure switch pins as input
	P1->DIR &= ~BIT4; // set Pin2 as input

	// configure input with pull up resistor for Sw2
	P1->REN |= BIT4; /* P1.4 pull resistor enabled */
	P1->OUT |= BIT4; /* Pull up/down is selected by P1->OUT */
}

//------------Switch_Input------------
// Read and return the status of Switch1
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch1_Pressed(void) // SWAPPED THESE BUT IT WORKS SO IDK
{
	BOOLEAN retVal = FALSE;
	// check if pressed
	if (P1->IN & BIT1) /* see if switch1 is pressed */
		retVal = FALSE;
	else
		retVal = TRUE;

	return (retVal); // return TRUE(pressed) or FALSE(not pressed)
}

//------------Switch_Input------------
// Read and return the status of Switch2
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch2_Pressed(void) // SWAPPED THESE BUT IT WORKS SO IDK
{
	BOOLEAN retVal = FALSE;
	// check if pressed
	if (P1->IN & BIT4) /* see if switch1 is pressed */
		retVal = FALSE;
	else
		retVal = TRUE;
	return (retVal); // return TRUE(pressed) or FALSE(not pressed)
}
