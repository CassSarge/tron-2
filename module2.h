#ifndef _MODULE_2_H
#define _MODULE_2_H

#include <stdio.h>
#include <stdlib.h>
#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

/// This file has the public definitions for this module
/// Any functions required by external modules are listed here

#define MAX_EL_STEPS 5
#define MAX_AZ_STEPS 5
#define ARRAY_VALS 7 // How many objects the polar coordinates array stores
    // 5 meaning (elevation,azimuth,range,IMU el, IMU az)



int module2(int azimuth_min, int azimuth_max, int elevation_min, int elevation_max, int 
  resolution, int samples_per_orientation, int sample_frequency, int mode_flag);

void initServo(void);

void testLaser(void);


#endif                                                                           