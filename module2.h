#ifndef _MODULE_2_H
#define _MODULE_2_H

#include <stdio.h>
#include <stdlib.h>
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

/// This file has the public definitions for this module
/// Any functions required by external modules are listed here

#define MAX_EL 900
#define MIN_EL -900
#define MAX_AZ 1600
#define MIN_AZ 300



int acquireMeasurements(int minAzimuth, int maxAzimuth, int minElevation, int maxElevation, 
            int resolution, int samples_per_orientation, int sampleFrequency);

void initServo(void);

void testLaser(void);

void setAzimuth(int angle);

void setElevation(int angle);

int getRange(int delayMS, int samples_per_orientation);

void sampleGetIMUdata(int* imuInfo);

void sampleSendData(int range, int servoAzi, int servoEle, 
                int IMU1, int IMU2, int IMU3, int IMU4);
                
void sampleAbort(void);

void delayby1ms(void);

// Recent updates

// calculate delayMS in module2 function
// updated get range function and header
// changed up module2 function
// declared overflow as a global variable for module2.c
// added overflow interrupt
// swapped duty cycle ports for  azimuth and elevation
// changed where azimuth and elevation functions are called
// changed name of module2 function, removed mode flag as required input
// Added boundary value looping back to module2 function

#endif                                                                           
