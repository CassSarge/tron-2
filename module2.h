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



int module2(int azimuth_min, int azimuth_max, int elevation_min, int elevation_max, int 
  resolution, int samples_per_orientation, int sample_frequency, int mode_flag);

void initServo(void);

void testLaser(void);

void setAzimuth(int angle);

void setElevation(int angle);

int sampleGetRange(int sampleFrequency, int samples_per_orientation);

void sampleGetIMUdata(int* imuInfo);

void sampleSendData(int range, int servoAzi, int servoEle, 
                int IMU1, int IMU2, int IMU3, int IMU4);
                
void sampleAbort(void);

#endif                                                                           
