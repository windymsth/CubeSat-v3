#include "solar_panel_ctrl.h"
#include <Arduino.h>
#include <Servo.h>

volatile char L_servo_current_deg = L_SERVOR_DOWN;
volatile char R_servo_current_deg = R_SERVOR_DOWN;

Servo myservoR, myservoL;

void tim4_RCPWM_Init()
{
			//16闁告帒妫濋。鍫曟晬閿燂拷1MHz闁挎稑鐣眓t = 20000闁挎冻鎷�
}
void solar_Servo_Init()
{
  Serial.println("Servo attach!");
	myservoL.attach(SERVO_PIN_L);
	myservoR.attach(SERVO_PIN_R);
	myservoL.write(L_SERVOR_DOWN);
	myservoR.write(R_SERVOR_DOWN);
  Serial.println("Servo attach finished!");
	delay(500);
	myservoR.detach();
	myservoL.detach();
  Serial.println("Servo detach");
}
void set_solar_panel_up() {
		Serial.println("act up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg < L_SERVOR_UP) 
				myservoL.write( L_servo_current_deg++ );
			if( R_servo_current_deg > R_SERVOR_UP) 
				myservoR.write( R_servo_current_deg-- );
			delay(30);
		}
}

void set_solar_panel_down() {
	Serial.println("act down");
  if (sys_data.is_solar_panel_on == true) {
    sys_data.is_solar_panel_on = false;			
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg > L_SERVOR_DOWN) 
				myservoL.write( L_servo_current_deg-- );
			if( R_servo_current_deg < R_SERVOR_DOWN) 
				myservoR.write( R_servo_current_deg++ );
			delay(30);
		}
  }
}

void set_solar_panel_left_up() {
		Serial.println("act left up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( L_servo_current_deg < L_SERVOR_UP )
				myservoL.write( L_servo_current_deg++ );
			else break;
			delay(30);
		}
}

void set_solar_panel_right_up() {
		Serial.println("act right up");
    sys_data.is_solar_panel_on = true;
		for(int i=0;i<65;i++)
		{
			if( R_servo_current_deg > R_SERVOR_UP ) 
				myservoR.write( R_servo_current_deg-- );
			else break;
			delay(30);
		}
}



