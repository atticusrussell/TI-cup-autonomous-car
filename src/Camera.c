/**
 * @file Camera.c
 * @author Erica Coles
 * @author Atticus Russell 
 * @brief provides functionality to use linescan camera with MSP432 uC for autonomous car on track.
 * @version 0.1
 * @date November 2022
 * @class CMPE-460 Interface and Digital Electronics
 * 
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include "msp.h"
#include "uart.h"
#include "leds.h"
#include "switches.h"
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
#include "ADC14.h"
#include "ControlPins.h"


#define LineThreshold 1000//need to determine this value

int count = 1;
int trace[1][128] = 0; //stored values for raw input
int bintrace[1][128] = 0; //stored values for edge detection
int smoothtrace[1][128] = 0; //stored values for 5-pt averager


///////////////////////////////////////////////////////
//
// NOTE: For the camera, you may want to change the default
//       clock to 48MHz
//
// To do that: Edit system_msp432p401r.c
//             Change:   #define  __SYSTEM_CLOCK    3000000
//             To:       #define  __SYSTEM_CLOCK    48000000 
// ADC will be P4.7 A6
//
// SI Pin will be P5.5 A0
//
// CLK Pin will be P5.4 A1//
//

// line stores the current array of camera data
extern uint16_t line[128]; //deleted "extern"
extern BOOLEAN g_sendData; //deleted "extern"

static char str[100];

// ADC_In() gets the latest value from the ADC
// ADC will be P4.7 A6

// SI Pin will be P5.5 A0

// CLK Pin will be P5.4 A1

// main


/////////////////////////////////////////////////////
//
// simple delay function
//
//
/////////////////////////////////////////////////////
void myDelay(void)
{
	volatile int j = 0;
	for (j = 0; j < 800000; j++)
	{
		;
	}
}
/////////////////////////////////////////////////////
//
// INIT_Camera function
//
//
/////////////////////////////////////////////////////
void INIT_Camera(void)
{
	g_sendData = FALSE;
	ControlPin_SI_Init();
	ControlPin_CLK_Init();
	ADC0_InitSWTriggerCh6();
}

/* [] CameraValues (){
	//CAMERA_AVG => an integer value for how long the averaging length will occur
	//gfpLineAverage => global floating point array of camera center line values
	//fpLinePos => returned from read camera this is the center line position
	//ReadCamera() => is the read camera function call returns a floating point value of fpLinePos

	// this will shift the values up and throw away the oldest value
	// then add a new reading
	for (i=CAMERA_AVG;i>0;i—)
		{
		gfpLineAverage[i]=gfpLineAverage[i-1];
		}
	// if no line was detected the previous camera value will be passed on
	if (fpLinePos=ReadCamera())
		{
		gfpLineAverage[0]= fpLinePos;
		}
} */


/// @brief Detects if the car is on or off the track
/// @param maxCamVal maximum value of camera data 
/// @return True if the car is off track, otherwise false if car is on track
BOOLEAN trackDetection(uint16_t maxCamVal){
	maxCamVal = line[128];
	if (maxCamVal > LineThreshold){
		uart0_put("Car is off track");
		return TRUE; //car is off the track

	}
	uart0_put("Car is on track");
	return FALSE; //car is on track
}

uint8_t centerTrack(uint16_t* smoothtrace, uint16_t* lineData){
	uint8_t rightLineVal = 0;
	uint8_t leftLineVal = 0;
	uint8_t i ;
	uint8_t centerLineVal;

	for(i = 64; i < 127; i++) {
		if(lineData[i] == 0){
			rightLineVal = i;
			break;
		}
	}

	for(i = 64; i >= 0; i--) {
		if(lineData[i] == 0){
			leftLineVal = i;
			break;
		}
	}

	// return the center value of the track
	centerLineVal = leftLineVal + ((rightLineVal - leftLineVal)/2);
	return centerLineVal;
}

int movAvg(uint16_t* lineData, uint16_t* smoothTrace){
	uint16_t mov_avg = 0;
	int j = 0;

	for(j = 0; j < 128; j++){
		mov_avg = (lineData[j+3] + lineData[j+2] + lineData[j+1] + lineData[j] + lineData[j-1])/5;
		smoothTrace[j] = mov_avg;
	}
}

/////////////////////////////////////////////////////
//
// main function
//
//
/////////////////////////////////////////////////////
int main(void)
{
	int i = 0;
	//initializations
	DisableInterrupts();
	uart0_init();
	uart0_put("\r\nLab5 CAMERA demo\r\n");

	
	uart0_put("\r\nINIT LEDs\r\n");
	LED1_Init();
	LED2_Init();
	// remember that we double the desired frequency because we need to account

	uart0_put("\r\nINIT Camera CLK and SI\r\n");
	uart0_put("\r\nINIT ADC\r\n");	
	INIT_Camera();
	
	uart0_put("\r\nINIT Switch 2\r\n");
	Switch2_Init();


	uart0_put("\r\nEnable Interrupts\r\n");
	EnableInterrupts();
	uart0_put("\r\nInterrupts successfully enabled\r\n");

	while(1)
	{

		if (g_sendData == TRUE) 
		{
			LED1_On();
			// send the array over uart
			sprintf(str,"%i\n\r",-1); // start value
			uart0_put(str);
			for (i = 0; i < 128; i++) 
			{
				sprintf(str,"%i\n\r", line[i]);
				uart0_put(str);
			}
			sprintf(str,"%i\n\r",-2); // end value
			uart0_put(str);
			LED1_Off();
			g_sendData = FALSE;
		}
		// do a small delay
		myDelay();
	}
}
