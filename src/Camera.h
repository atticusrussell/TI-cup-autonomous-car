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

#define CONSERVATIVE_VCM	(5000) // this works nicely - will keep as cons.
#define NORMAL_VCM		(3000)	
#define RECKLESS_VCM	(2400) // TODO test and tune

// track position threshold values for the camera avg magnitude VCM.
// will be considered in the state if avg vcm is greater than the threshold
#define THRESHOLD_STRAIGHT		(12000)
#define THRESHOLD_NORMAL		(10000)
#define THRESHOLD_APPROACH		(7000)
#define THRESHOLD_TURNING		(4000)
#define THRESHOLD_EDGE			(3500)


 /* Function prototypes */ 
void INIT_Camera(void);
BOOLEAN get_on_track(uint16_t maxCamVal, uint16_t onTrackThreshold);
uint8_t get_track_center(uint16_t* smoothData);
void smooth_line(uint16_t* rawData, uint16_t* smoothData);

#endif
