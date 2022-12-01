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

// the minimum camera visual centrer mass that is considered to be on the track
#define OG_ON_TRACK_VCM	(6000) // old one - too high honestly
#define TUNING_ON_TRACK_VCM	(5000)

 /* Function prototypes */ 
void INIT_Camera(void);
BOOLEAN get_on_track(uint16_t maxCamVal, uint16_t onTrackThreshold);
uint8_t get_track_center(uint16_t* smoothData);
void smooth_line(uint16_t* rawData, uint16_t* smoothData);

 #endif
