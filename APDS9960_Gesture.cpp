#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "APDS9960_Gesture.h"

// Pins
#define APDS9960_INT    2 // Needs to be an interrupt pin

// Constants

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;

void APDS9960_Init() {

  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));

  // Initialize interrupt service routine
  attachInterrupt(0, interruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }
}

volatile char Gesture_meas() {
  char DIR;
  if ( isr_flag == 1 ) {
    Serial.println(F("in G"));
    DIR = handleGesture();
    isr_flag = 0;
  }
  return DIR;
}

volatile void interruptRoutine() {
  isr_flag = 1;
}

volatile char handleGesture() {
  char DIR_handle;
  if ( apds.isGestureAvailable() ) {
    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("UP");
        DIR_handle = DIR_Hand_UP;
        break;
      case DIR_DOWN:
        Serial.println("DOWN");
        DIR_handle = DIR_Hand_DOWN;
        break;
      case DIR_LEFT:
        Serial.println("LEFT");
        DIR_handle = DIR_Hand_LEFT;
        break;
      case DIR_RIGHT:
        Serial.println("RIGHT");
        DIR_handle = DIR_Hand_RIGHT;
        break;
      case DIR_NEAR:
        Serial.println("NEAR");
        DIR_handle = DIR_Hand_NEAR;
        break;
      case DIR_FAR:
        Serial.println("FAR");
        DIR_handle = DIR_Hand_FAR;
        break;
      default:
        Serial.println("NONE");
        DIR_handle = DIR_Hand_NONE;
    }
  }
  return DIR_handle;
}

volatile void Gesture_Ctrl_Mode()
{
  switch ( Gesture_meas() )
  {
    case	DIR_Hand_UP:
      Serial.println("GCM_up");
      is_opt_busy = true;
      is_listen = false;
      GPSSerial.end();	//*****  modify by WJG
      myservoR.attach(SERVO_PIN_R);	//*****  modify by WJG
      myservoL.attach(SERVO_PIN_L);	//*****  modify by WJG
      RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL);
      set_solar_panel_up();
      is_opt_busy = false;
      break;

    case	DIR_Hand_DOWN:
      Serial.println("GCM_down");
      is_opt_busy = true;
      RF_SERIAL.write(COMM_CLOSE_SOLAR_PANEL);
      set_solar_panel_down();	//*****  modify by WJG
      myservoR.detach();	//*****  modify by WJG
      myservoL.detach();	//*****  modify by WJG
      GPSSerial.begin(9600);	//*****  modify by WJG
      is_listen = true;
      is_opt_busy = false;
      break;

    case	DIR_Hand_LEFT:
      Serial.println("GCM_left");
      is_opt_busy = true;
      is_listen = false;
      GPSSerial.end();	//*****  modify by WJG
      //myservoR.attach(SERVO_PIN_R);	//*****  modify by WJG
      myservoL.attach(SERVO_PIN_L);	//*****  modify by WJG
      RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL);
      set_solar_panel_left_up();
      is_opt_busy = false;
      break;

    case	DIR_Hand_RIGHT:
      Serial.println("GCM_right");
      is_opt_busy = true;
      is_listen = false;
      GPSSerial.end();	//*****  modify by WJG
      myservoR.attach(SERVO_PIN_R);	//*****  modify by WJG
      //myservoL.attach(SERVO_PIN_L);	//*****  modify by WJG
      RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL);
      set_solar_panel_right_up();
      is_opt_busy = false;
      break;

    default: break;
  }//** end switch(DIR)
}



