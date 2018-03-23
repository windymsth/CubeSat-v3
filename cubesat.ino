#include <Arduino.h>
#include "type.h"
#include "userRGB.h"
#include "userPM2_5.h"
#include "sensor.h"

//***** 2017-10-31
#define ENABLE  1
#define DISABLE 0
#define PROCESSING_SERIAL_DEBUG   ENABLE
//#define PROCESSING_SERIAL_DEBUG   DISABLE
/*-------Ver Config--------------*/
#define Normal_COMMSAT  ENABLE
//#define AItuling_COMMSAT  ENABLE
//#define Gesture_COMMSAT   ENABLE
//#define BlthGCS_COMMSAT   ENABLE
volatile uint32_t timeSlice = 0;

#ifdef  AItuling_COMMSAT
  SoftwareSerial AISerial(A10, A11);
#endif

#ifdef  BlthGCS_COMMSAT
  SoftwareSerial BLTSerial(A10, A11);//RX TX
#endif

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial Init OK!");

  Wire.begin();
  Serial.println("IIC bus Enable...");
	solar_Servo_Init();
 #ifdef BlthGCS_COMMSAT
  BLTSerial.begin(115200);
  Serial.println("bluetooth Serial Start...");
 #endif
  Sensor_Init();
  WS2812B_Init();
  G5_Init();
	SHT31_Init();
	Compass_hmc5883l_Init(); 
 
  BMP180_Init();
  FM_Init();
//  APDS9960_Init();
  Serial.println("Periheral Init finished!");
  timeSlice = millis();

  #ifdef  AItuling_COMMSAT
    AISerial.begin(115200);
    Serial.println("AI Serial start!");
    delay(20000);
    AISerial.println("$MCU+TLK++ON&*");
    Serial.println("AI Serial code send finished!");
  #endif
}

uint8_t sliceType = pm2_5Type;
void loop()
{
  static uint8_t task_Heartbag = 0;
  static uint8_t task_WS2812 = 0;
  static uint8_t task_Bmp180 = 0;
  static uint8_t task_Gesture = 0;
  static uint8_t task_G5 = 0;
  static uint8_t task_SHT31 = 0;
  static uint8_t task_getAccel = 0;
  static uint8_t task_GPS = 0;
  static uint8_t task_comm = 0;
  if ( millis() - timeSlice >= 10 ) {
	  #if PROCESSING_SERIAL_DEBUG
      if( millis() - timeSlice >= 10 ) {
  			Serial.print("SliceTime: ");
  			Serial.println( millis() - timeSlice, DEC);
  			} 
    #endif
    
    timeSlice = millis();

    if( ++task_Heartbag >= 100) { task_Heartbag = 0;
      static unsigned int heartCnt = 0;
      Serial.print("Heart count: ");
      Serial.println(++heartCnt, DEC);
    }
    
    if( sys_data.is_solar_panel_on == false) {
      if( ++task_WS2812 >= 1) { task_WS2812 = 0;
        #if PROCESSING_SERIAL_DEBUG
          Serial.println("WS2812 in");
        #endif
        
        WS2812B_Handle();
        #if PROCESSING_SERIAL_DEBUG
          Serial.println("WS2812 out");
        #endif
        }
    }
    
//    if( ++task_comm >= 50) { task_comm = 0;
//      #if PROCESSING_SERIAL_DEBUG
//        Serial.println("RFserial in");
//      #endif
//      
//      RF_data_update();
//      #if PROCESSING_SERIAL_DEBUG
//        Serial.println("RFserial out");
//      #endif
//      }		
//    if( ++task_Gesture >= 2) {
//      #if PROCESSING_SERIAL_DEBUG
//        Serial.println("Gesture");
//      #endif
//      
//      task_Gesture = 0;
//      Gesture_Ctrl_Mode();
//          
		G5_Handle();
//--------------------------------------------------------------------------		
      switch (sliceType) {  // have 5 tasks, one loop neads 50ms;
				case pm2_5Type: 
				  if ( ++task_G5 >= 20) { task_G5 = 0; 
						if (flagee) {
						flagee = false;
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("G5_LV");
            #endif
              
						G5_LV();
						}
				  }
					break;
					
				case sht31Type: 
					if ( ++task_SHT31 >= 10) {  task_SHT31 = 0;
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("SHT31");
            #endif
            
						SHT31_Handle();
           
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("SHT31 out");
            #endif
					}	//**end if( ++task_SHT31 >= 1)
					break;
					
				case mpuType:   
					if ( ++task_getAccel >= 1) {  task_getAccel = 0;
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("IMU");
            #endif
            
					  getAccel_Data();
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("IMU out");
            #endif
					  }
					break;
					
        case bmp180Type:
					if ( ++task_Bmp180 >= 10) { task_Bmp180 = 0;
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("BMP180");
            #endif
            
					  Bmp180_Handle();
            #if PROCESSING_SERIAL_DEBUG
              Serial.println("BMP180 out");
            #endif
					  }
          break;
					
				case gpsType:   
					if ( ++task_GPS >= 20) {  task_GPS = 0;            
            #if PROCESSING_SERIAL_DEBUG
            Serial.println("GPS");
            #endif
            
						serialEvent_GPS();
						GPS_Handle();
            #if PROCESSING_SERIAL_DEBUG
            Serial.println("GPS out");
            #endif
						}
					break;
			}// end: switch
			
    if (++ sliceType > gpsType) sliceType = pm2_5Type;
    comm_Handle();
    
   #ifdef BlthGCS_COMMSAT
     Serial_data_update();
   #endif
  }// end: if ( millis() - timeSlice >= 10 )
}// end: loop()

#ifdef BlthGCS_COMMSAT
bool Serial_data_update(void) {
  int checkValue = 1;

  BLTSerial.print("{");
  //
  BLTSerial.print("\"HEAT_TMP\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.heating_panel_temp);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.heating_panel_temp + checkValue);
  BLTSerial.print("},");
  // string.toFloat
  //
  BLTSerial.print("\"IN_TMP\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.inside_temp);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.inside_temp + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"IN_HUMI\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.inside_humi);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.inside_humi + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"POS_X\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.posture_x);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.posture_x + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"POS_Y\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.posture_y);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.posture_y + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"POS_Z\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.posture_z);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.posture_z + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"HEAD\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.heading);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.heading + checkValue);
  BLTSerial.print("},");

  // GPS
  BLTSerial.print("\"GPS_LAT\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.gps_latitude, 4);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.gps_latitude + checkValue, 4);
  BLTSerial.print("},");

  // GPS
  BLTSerial.print("\"GPS_LON\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.gps_longitude, 4);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.gps_longitude + checkValue, 4);
  BLTSerial.print("},");

  // PM2.5
  BLTSerial.print("\"PM2_5\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.pm25);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.pm25 + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"PRESSURE\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.pressure);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.pressure + checkValue);
  BLTSerial.print("},");
  //
  BLTSerial.print("\"CUR_SPEED\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.current_speed);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.current_speed + checkValue);
  BLTSerial.print("},");

  //
  BLTSerial.print("\"ACCE_SPEED\":");
  BLTSerial.print("{");
  BLTSerial.print("\"value\": ");
  BLTSerial.print(sys_data.accelerated);
  BLTSerial.print(",\"check\": ");
  BLTSerial.print(sys_data.accelerated + checkValue);
  BLTSerial.print("},");

  BLTSerial.print("}");
  BLTSerial.println("");
  //RF_SERIAL.print(0xFF);
  return true;
}
#endif

