#ifndef __USERRGB_H__
#define __USERRGB_H__

//	arduino include
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>

//	user include
#include "type.h"
#include "sensor.h"

void WS2812B_Init(void);
void WS2812B_Handle(void);

#endif



