/**
 * @file Camera.h
 * @author Atticus Russell
 * @brief header file to help interface with camera for IDE car
 * @version 0.1
 * 
 * 
 */
 
#include <stdint.h>
#include "Common.h"

#ifndef _CAMERA_HEADER_FILE_
#define _CAMERA_HEADER_FILE_

// the minimum camera value that is considered to be on the tack
#define TRACK_MIN_VAL	(10000)

 /* Function prototypes */ 
void INIT_Camera(void);
BOOLEAN get_on_track(uint16_t maxCamVal);
uint8_t get_track_center(uint16_t* smoothData);
void smooth_line(uint16_t* rawData, uint16_t* smoothData);

 #endif
