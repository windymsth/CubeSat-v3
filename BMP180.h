#ifndef	__BMP180_H_
#define	__BMP180_H_

#include <Arduino.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include "sensor.h"

#define BMP180ADD 0xEE>>1  

//	I2C address of BMP180  
//	write is (0xEE)     read is (0xEF)     

unsigned long bmp180ReadUP();
unsigned int bmp180ReadUT();
int bmp180ReadDate(unsigned char address);
int bmp180Read(unsigned char address);
long bmp180GetPressure(unsigned long up);
short bmp180GetTemperature(unsigned int ut);
void BMP180start();
double BMP180_Calculate();
void BMP180_Init();
void Bmp180_Handle();

#endif



