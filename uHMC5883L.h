#ifndef __UHMC5883L_H__
#define __UMC5883L_H__

#include <Arduino.h>
#include <HMC5883L.h>
#include "sensor.h"

extern bool compass_found;

void Compass_hmc5883l_Init();
float getHeading();
void getCompass_Data();
void get_one_sample_date_mxyz();
void get_calibration_Data ();

#endif



