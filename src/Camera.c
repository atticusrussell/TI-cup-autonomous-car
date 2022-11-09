/**
 * @file Camera.c
 * @author Erica Coles
 * @author Atticus Russell 
 * @brief provides functionality to use linescan camera with MSP432 uC for autonomous car on track.
 * @version 0.1
 * @date November 2022
 * @class CMPE-460 Interface and Digital Electronics
 * @note based on camera file developed in IDE Lab05
 * 
 * 
 */

#include	<stdint.h>
#include "Common.h"
#include "ADC14.h"
#include "ControlPins.h"
#include "Camera.h"



/**
 * @note For the camera, you may want to change the default clock to 48MHz
 * 	To do that: Edit system_msp432p4111.c
 * 		Change:   #define  __SYSTEM_CLOCK    3000000
 * 		To:       #define  __SYSTEM_CLOCK    48000000 
 * @note ADC will be P4.7 A6
 * @note SI Pin will be P5.5 A0
 * @note CLK Pin will be P5.4 A1
 * 
 */
extern uint16_t line[128]; 
extern BOOLEAN g_sendData; 
 

/**
 * @brief initializes the car's camera.
 * 
 */
void INIT_Camera(void){
	g_sendData = FALSE;
	ControlPin_SI_Init();
	ControlPin_CLK_Init();
	ADC0_InitSWTriggerCh6();
}


/**
 * @brief evaluate if the car is on the track based on the y value of center
 * 
 * @param centerCamVal value at visual center of camera data 
 * @return True if the car is on track, otherwise false 
 */
BOOLEAN get_on_track(uint16_t maxCamVal){
	if (maxCamVal > TRACK_MIN_VAL){
		return TRUE; //car is on the track
	} else{
		return FALSE; //car is off track
	}
}


/**
 * @brief Get the index of the center of the track by summing the y values
 * @note the white track has a light intensity. We want to know the index of the visual center/avg
 * 
 * @param smoothData the smoothed camera data
 * @return uint8_t the index from 0 to 127 of the visual center of the input
 */
uint8_t get_track_center(uint16_t* smoothData){
	double x_sum = 0.0;
	double y_sum = 0.0;
	
	for(int i = 0; i < 128; i++){
		x_sum += smoothData[i]*i; // is weighted by the multiplication
		y_sum += smoothData[i];
	}

	return (uint8_t) (x_sum/y_sum);
}


/**
 * @brief takes a 5-point moving average of the data to smooth it
 * 
 * @param rawData a 128 wide array of the raw camera data
 * @param smoothData an empty 128 wide array to populate with smoothed data
 * @note dealing weirdly with accepting the output as a param to not do malloc
 * @note based on the MATLAB code for processing the camera data in Lab5
 * 
 */
void smooth_line(uint16_t* rawData, uint16_t* smoothData){
	uint16_t avg;

	for(int j = 0; j < 128; j++){
		avg = (rawData[j+2] + rawData[j+1] + rawData[j] + rawData[-1] + rawData[j-2])/5;
		smoothData[j] = avg;
	}
}
