#ifndef __CAM_H__
#define __CAM_H__

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "type.h"
#include "sensor.h"

//Camera vary
#define PIC_PKT_LEN    128
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define CAM_SERIAL     Serial1
#define PIC_FMT        PIC_FMT_VGA

extern unsigned long picTotalLen; 

int preCapture(void);
int Capture(void);
int GetData(void);
int sendData(void);
void cam_init(void);

#endif






