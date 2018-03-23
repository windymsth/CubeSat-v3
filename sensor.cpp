#include "sensor.h"
//	arduino include
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"

SENSOR_Type sys_data;

//GPS vary
String GPS_data = "";
String GPS_lat_lon = "";

//mpu6050
MPU6050 accelgyro;
float Axyz[3];
float Gxyz[3];

char fmbuf[512] = {};
String pm_str[256] = {};

bool sht31Flag = false;
bool is_opt_busy = false;
volatile bool heater_state = false;
volatile uint8_t operation_index = 0;
static float temp_Speed = 0;
bool is_listen = true;

SoftwareSerial GPSSerial(A14, A15);//RX TX
//#define RF_SERIAL Serial3

#define CONSTANTS_ONE_G 9.80665f    /* m/s^2    */

void Sensor_Init(void) {
  
  GPSSerial.begin(9600);
	Serial.println("GPS Serial Start...");
  
	accelgyro.initialize();
  Serial.println("MPU6050 Init Finished!");
  
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	
  RF_SERIAL.begin(9600);
//  RF_SERIAL.setTimeout(1000);
  cam_init();
  operation_index = opt_default;
  sys_data.is_solar_panel_on = false;
}


void serialEvent_GPS() {
  String inString = "";
  if (is_listen) { 
    GPSSerial.listen();
    if (GPSSerial.available()) {
      char inChar = GPSSerial.read();
      if (inChar == '$') {           //'$' means a new line of data
        GPSSerial.setTimeout(100);
        String inString = GPSSerial.readStringUntil('\n');
//        Serial.println(inString);
        if (inString.substring(0, 5) == "GPGGA")
        {
          GPS_lat_lon = inString;
//                  Serial.println(">>>>>>>>>>GPGGA");
//                  Serial.println(GPS_lat_lon);
        }
        if (inString.substring(0, 5) == "GPVTG") {  //"GPGGA" is a line of message which include the message of possition, then send it with Serial
          GPS_data = inString;
//                  Serial.println("<<<<<<<<<<GPVTG");
//                  Serial.println(GPS_data);
        }
      }//end if (inChar == '$')

    }
  }
}

void GPS_Handle(void) {
  char *p;
  char buf[512] = {};
  static byte gpsCount = 0;
  if (++ gpsCount >= 1) {
    if (GPS_data != "" && GPS_data.length() > 0) {
      GPS_data.toCharArray(buf, GPS_data.length());
      p = strtok(buf, ",");
      // p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      sys_data.current_speed = atof(p);
      //      Serial.print("Speed: ");
      //      Serial.print(sys_data.gps_latitude, 4);
      p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      temp_Speed = atof(p) * 1000 / 3600;
      if (temp_Speed > 0.3f)
        sys_data.current_speed = atof(p) * 1000 / 3600;
      else
        sys_data.current_speed  = 0;
//      Serial.print(", ");
//      Serial.println(sys_data.current_speed, 4);
    }
    if (GPS_data != "" && GPS_lat_lon.length() > 0) {
      GPS_lat_lon.toCharArray(buf, GPS_lat_lon.length());
      p = strtok(buf, ",");
      // p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      sys_data.gps_latitude = atof(p);
     // Serial.print("GPS_Lat_Lon: ");
     // Serial.print(sys_data.gps_latitude, 4);
      p = strtok(NULL, ",");
      p = strtok(NULL, ",");
      sys_data.gps_longitude = atof(p);
//      Serial.print(", ");
//      Serial.println(sys_data.gps_longitude, 4);
    }
    gpsCount = 0;
  } else {
    GPS_data = "";
    GPS_lat_lon = "";
  }
}

void getAccel_Data(void)
{
  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);//
  Axyz[0] = (double) ax / 16384; //2g * ax / 32768;  -->16384
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384 + 0.063;
  sys_data.posture_x = Axyz[0];
  sys_data.posture_y = Axyz[1];
  sys_data.posture_z = Axyz[2];
  // compass heading
  if (compass_found)
    sys_data.heading = getHeading();
  Gxyz[0] = (double) gx * 250 / 32768; //
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;

  sys_data.accelerated = sqrt((Axyz[0] * CONSTANTS_ONE_G) * (Axyz[0] * CONSTANTS_ONE_G) + (Axyz[1] * CONSTANTS_ONE_G) * (Axyz[1] * CONSTANTS_ONE_G)
                              + (Axyz[2] * CONSTANTS_ONE_G) * (Axyz[2] * CONSTANTS_ONE_G));
  if (sys_data.accelerated < 9.8)
    sys_data.accelerated = 9.80;
   // Serial.print(sys_data.accelerated); Serial.print("\r\n");
   // Serial.print("Accelerated:\t");
   // Serial.print(Axyz[0]); Serial.print("\t");
   // Serial.print(Axyz[1]); Serial.print("\t");
   // Serial.print(Axyz[2]); Serial.print("\r\n");
   // Serial.print("Angular:\t");
   // Serial.print(Gxyz[0]); Serial.print("\t");
   // Serial.print(Gxyz[1]); Serial.print("\t");
   // Serial.print(Gxyz[2]); Serial.print("\r\n");
   // Serial.print("Accelerated Speed:\t");
   // Serial.print(ax); Serial.print("\t");
   // Serial.print(ay); Serial.print("\t");
   // Serial.println(az);
   // Serial.print("Angular Speed:\t");
   // Serial.print(gx); Serial.print("\t");
   // Serial.print(gy); Serial.print("\t");
   // Serial.println(gz);
}

bool RF_data_update(void) {
  int checkValue = 1;

  RF_SERIAL.print("{");
  //
  RF_SERIAL.print("\"HEAT_TMP\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.heating_panel_temp);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.heating_panel_temp + checkValue);
  RF_SERIAL.print("},");
  // string.toFloat
  //
  RF_SERIAL.print("\"IN_TMP\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.inside_temp);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.inside_temp + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"IN_HUMI\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.inside_humi);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.inside_humi + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"POS_X\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.posture_x);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.posture_x + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"POS_Y\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.posture_y);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.posture_y + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"POS_Z\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.posture_z);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.posture_z + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"HEAD\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.heading);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.heading + checkValue);
  RF_SERIAL.print("},");

  // GPS
  RF_SERIAL.print("\"GPS_LAT\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.gps_latitude, 4);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.gps_latitude + checkValue, 4);
  RF_SERIAL.print("},");

  // GPS
  RF_SERIAL.print("\"GPS_LON\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.gps_longitude, 4);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.gps_longitude + checkValue, 4);
  RF_SERIAL.print("},");

  // PM2.5
  RF_SERIAL.print("\"PM2_5\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.pm25);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.pm25 + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"PRESSURE\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.pressure);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.pressure + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"CUR_SPEED\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.current_speed);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.current_speed + checkValue);
  RF_SERIAL.print("},");

  //
  RF_SERIAL.print("\"ACCE_SPEED\":");
  RF_SERIAL.print("{");
  RF_SERIAL.print("\"value\": ");
  RF_SERIAL.print(sys_data.accelerated);
  RF_SERIAL.print(",\"check\": ");
  RF_SERIAL.print(sys_data.accelerated + checkValue);
  RF_SERIAL.print("},");

  RF_SERIAL.print("}");
  RF_SERIAL.println("");
  //RF_SERIAL.print(0xFF);
  return true;
}

void serialEvent3() {
  if (RF_SERIAL.available()) {
    //Serial.println("Enter timer1 interrupt!");
    comm_run(&RF_SERIAL);
    // RF_Serial_test();
  }
}

void comm_Handle(void) {
  switch (operation_index) {
    case opt_Pre_Capture:
      //Serial.println("opt_Pre_Capture");
      RF_SERIAL.write(COMM_PRE_CAPTURE);
      if (-1 == preCapture()) {
        //        Serial.println("preCapture");
        operation_index == opt_default;
        break;
      }
      if (-1 == Capture()) {
        //        Serial.println("Capture");
        operation_index == opt_default;
        break;
      }

      if (0 == GetData()) {
        //        Serial.println("GetData Succeed!");
      } 
			else {
        //        Serial.println("GetData Error!");
      }
      operation_index = opt_default;
      break;

    case opt_Get_Pic_Len:
      //Serial.println("opt_Get_Pic_Len");
      operation_index = opt_default;
      RF_SERIAL.println(picTotalLen);
      Serial.println(picTotalLen);
      //picTotalLen = 0;
      break;

    case opt_Send_Pic_data:
      //Serial.println("opt_Send_Pic_data");
      RF_SERIAL.write(COMM_SEND_PIC_DATA);
      sendData();
      break;

    /*case opt_Turn_On_Heater:
      operation_index = opt_default;  //
      set_heater_on();
      RF_SERIAL.write(COMM_TURN_ON_HEATER);
      heater_state = true;
      break;

      case opt_Turn_Off_Heater:
      operation_index = opt_default;  //
      set_heater_off();
      RF_SERIAL.write(COMM_TURN_OFF_HEATER);
      heater_state = false;
      break;*/
    case opt_Turn_On_FM:
      operation_index = opt_default;  //
      RF_SERIAL.write(COMM_FM_ON);
      set_FM_OnOff();
      break;

    case opt_Turn_Off_FM:
      operation_index = opt_default;  //
      RF_SERIAL.write(COMM_FM_OFF);
      set_FM_OnOff();
      break;
			
    case opt_Open_Solar_Panel:
      operation_index = opt_default;  //
      is_opt_busy = true;
      is_listen = false;
      GPSSerial.end();	            //*****  modify by WJG
      myservoR.attach(SERVO_PIN_R);	//*****  modify by WJG
      myservoL.attach(SERVO_PIN_L);	//*****  modify by WJG
      RF_SERIAL.write(COMM_OPEN_SOLAR_PANEL);
      set_solar_panel_up();
      //RF_SERIAL.write(COMM_FAK);
      is_opt_busy = false;
      break;

    case opt_Close_Solar_Panel:
      operation_index = opt_default;  //
      is_opt_busy = true;
      RF_SERIAL.write(COMM_CLOSE_SOLAR_PANEL);
      //RF_SERIAL.write(COMM_FAK);
      set_solar_panel_down();	//*****  modify by WJG
      myservoR.detach();	    //*****  modify by WJG
      myservoL.detach();	    //*****  modify by WJG
      GPSSerial.begin(9600);	//*****  modify by WJG
      is_listen = true;
      is_opt_busy = false;
      break;

    case opt_Request_Sat_Data:
      //Serial.println("opt_Request_Sat_Data");
      operation_index = opt_default;
      RF_data_update();                     //根据上位机指令才更新，不是自动更新
      break;

    case opt_Reset:
      operation_index = opt_default;
      RF_SERIAL.write(COMM_RESET);
      //RF_SERIAL.write(COMM_FAK);
      break;
    default: break;
  }
}

void comm_run(HardwareSerial* port) {
	char data;
  if (port->available()) {
    data = port->read();
    //port->write(data);
    //    Serial.print("Reveived data: 0x");
    //    Serial.println(data);
    
    if (data == '&')
    {
      fmbuf[512] = "\0";
      String inString = port->readStringUntil('\n');
      inString.replace("&", "");
      delay(50);
      //      Serial.print(" ---------------inString:");
      //      Serial.println(inString);
      inString.trim();
      if (inString.length() > 0) {
        inString.toCharArray(fmbuf, inString.length() + 1);
        //        Serial.print("fmbuf:");
        //        Serial.println(fmbuf);
        SetFrequency(fmbuf);
      }
    }
	
    if (is_opt_busy == true) {
      return;
    }

    switch ((uint8_t)data) {
      case COMM_SOH:
        operation_index = opt_default;
        break;

      case COMM_EOT:  // End of transmit
        break;

      case COMM_ACK:
        break;

      case COMM_NAK:
        break;

      case COMM_CAN:  // Cancle anyway
        break;

      case COMM_REQUEST_SAT_DATA:
        operation_index = opt_Request_Sat_Data;
        break;

      case COMM_OPEN_SOLAR_PANEL:
        operation_index = opt_Open_Solar_Panel;
        break;

      case COMM_CLOSE_SOLAR_PANEL:
        operation_index = opt_Close_Solar_Panel;
        break;

      case COMM_FM_ON:
        operation_index = opt_Turn_On_FM;
        break;

      case COMM_FM_OFF:
        operation_index = opt_Turn_Off_FM;
        break;

      case COMM_TURN_ON_HEATER:
        operation_index = opt_Turn_On_Heater;
        break;

      case COMM_TURN_OFF_HEATER:
        operation_index = opt_Turn_Off_Heater;
        break;

      case COMM_PRE_CAPTURE:
        operation_index = opt_Pre_Capture;
        break;

      case COMM_GET_PIC_LEN:
        operation_index = opt_Get_Pic_Len;
        break;

      case COMM_SEND_PIC_DATA:
        operation_index = opt_Send_Pic_data;
        break;

      case COMM_RESET:
        operation_index = opt_Reset;
        break;

			case COMM_OPEN_GESTURE:
        operation_index = opt_Turn_On_GESTURE;
        break;

      case COMM_CLOSE_GESTURE:
        operation_index = opt_Turn_Off_GESTURE;
        break;
				
      default: break;
    }
  }
}


