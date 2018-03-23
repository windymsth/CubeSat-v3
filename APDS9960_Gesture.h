#ifndef __APDS9960_GESTURE_H__
#define __APDS9960_GESTURE_H__

#include <Arduino.h>
#include <Servo.h>
#include "sensor.h"

void APDS9960_Init();

/**
	@return Gesture_Type:
						DIR_UP,			--- 1
						DIR_DOWN,		--- 2
						DIR_LEFT, 	--- 3
						DIR_RIGHT,	--- 4
						DIR_NEAR,		--- 5
						DIR_FAR,		--- 6
						DIR_NONE		--- 7
 */
volatile char Gesture_meas();

volatile void Gesture_Ctrl_Mode();	//*****called by cubesat.ino

volatile void interruptRoutine();
volatile char handleGesture();
volatile void Gesture_Ctrl_Mode();

#endif



