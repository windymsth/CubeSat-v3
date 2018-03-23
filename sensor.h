#ifndef __SENSOR_H__
#define __SENSOR_H__

//	user include
#include "type.h"
#include "SHT31.h"
#include "cam.h"

#include "MPU6050.h"
#include "uHMC5883L.h"
#include "BMP180.h"

#include "FM_Radio.h"
#include "APDS9960_Gesture.h"
#include "solar_panel_ctrl.h"

// sys_data
typedef struct SYS_DATA{
  float heating_panel_temp = 0.0;
  float inside_temp = 0.0;
  float inside_humi = 0.0;
  float posture_x = 0.0;
  float posture_y = 0.0;
  float posture_z = 0.0;
  float accelerated = 0.0;
  float heading = 0.0;
  float gps_latitude = 110.988356;
  float gps_longitude = 19.646334;
  float pressure = 0.0;
  float current_speed = 0.0;
  int pm25 = 0;
  
  bool is_solar_panel_on = false;
}SENSOR_Type;

enum communicate {
  COMM_SOH = 0x01,
  COMM_EOT = 0x04,
  COMM_ACK = 0x06,
  COMM_NAK = 0x15,
  COMM_CAN = 0x18,
  COMM_FAK = 0xFF,

  // command
  COMM_REQUEST_SAT_DATA = 254,
  COMM_OPEN_SOLAR_PANEL = 253,
  COMM_CLOSE_SOLAR_PANEL = 252,
  COMM_RESET_SYS = 251,
  COMM_TURN_ON_HEATER = 250,
  COMM_TURN_OFF_HEATER = 249,
  // COMM_TAKE_PHOTO = 248,
  COMM_PRE_CAPTURE = 247,
  COMM_GET_PIC_LEN = 246,
  COMM_SEND_PIC_DATA = 127,
  COMM_RESET = 244,
  COMM_FM_ON = 243,
  COMM_FM_OFF = 242,
  COMM_OPEN_GESTURE = 241,	//*****  add by WMY
  COMM_CLOSE_GESTURE = 240	//*****  add by WMY
};

enum Option_Index {
  // comm execute index
  opt_Pre_Capture = 0,
  opt_Get_Pic_Len = 1,
  opt_Send_Pic_data = 2,
  opt_Turn_On_Heater = 3,
  opt_Turn_Off_Heater = 4,
  opt_Open_Solar_Panel = 5,
  opt_Close_Solar_Panel = 6,
  opt_Request_Sat_Data = 7,
  opt_Reset = 8,
  opt_Turn_On_FM = 9,
  opt_Turn_Off_FM = 10,
  opt_Turn_On_GESTURE,	//*****  add by WMY
  opt_Turn_Off_GESTURE,	//*****  add by WMY	
  opt_default
};

enum Gesture_Type
{
	DIR_Hand_default = 0,
	DIR_Hand_UP,
	DIR_Hand_DOWN,
	DIR_Hand_LEFT, 
	DIR_Hand_RIGHT,
	DIR_Hand_NEAR,
	DIR_Hand_FAR,
	DIR_Hand_NONE
};

extern bool sht31Flag;
extern SENSOR_Type sys_data;
extern volatile uint8_t operation_index;
extern bool is_opt_busy;
extern bool is_listen;

extern SoftwareSerial GPSSerial;

#define RF_SERIAL Serial3

void Sensor_Init(void);
void GPS_Handle(void);
void serialEvent_GPS(void);
void getAccel_Data(void);
bool RF_data_update(void);
bool Serial_data_update(void);
void comm_Handle(void);
void comm_run(HardwareSerial* port);

#endif



