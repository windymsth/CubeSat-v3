#ifndef	__SOLAR_PANEL_CTRL_H_
#define	__SOLAR_PANEL_CTRL_H_

#include "sensor.h"

//***** User define ******//
#define SERVO_PIN_R    8  //R 8
#define SERVO_PIN_L    7  //L 7

#define L_SERVOR_DOWN   50  //L
#define L_SERVOR_UP    110  //L
#define R_SERVOR_DOWN  120  //R
#define R_SERVOR_UP     60  //R

extern Servo myservoR, myservoL;

//***** User function ******//
void solar_Servo_Init();
void set_solar_panel_up();
void set_solar_panel_down();
void set_solar_panel_left_up();
void set_solar_panel_right_up();

#endif



