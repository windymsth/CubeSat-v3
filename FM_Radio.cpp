#include "FM_Radio.h"

SoftwareSerial FMSerial(A12, A13);
char fbuf[512] = {};

//FM
void FM_Init()
{
  FMSerial.begin(38400);
  FMSerial.setTimeout(1000);
}

//�
void SetFrequency(char *p)
{
  FMSerial.write(p);
  FMSerial.setTimeout(1000);
}

//
void set_FM_OnOff()
{
  FMSerial.write("AT+PAUS");
}

void ReadFrequency()
{
//  if (is_listen)
//  {
//    FMSerial.listen(); //
//    delay(200);
//    if (FMSerial.available())
//    {
//      fbuf[512] = "\0";
//      char inChar = FMSerial.read();
//      Serial.print("-------------------------------:");
//      Serial.println(inChar);
//      if (inChar == 'F') {           //'F' means a new line of data
//        String inString = Serial2.readStringUntil('\n');
//        Serial.print(" ---------------inString:");
//        Serial.println(inString);
//        inString.trim();
//        inString.toCharArray(fbuf, inString.length());
//      }
//      else if (inChar == 'V') {           //'F' means a new line of data
//        fbuf[512] = "\0";
//        String inString = Serial2.readStringUntil('\n');
//        Serial.print(" ---------------inString:");
//        Serial.println(inString);
//        inString.trim();
//        inString.toCharArray(fbuf, inString.length());
//      }
//      RF_SERIAL.write(fbuf);
//    }
//  }
}



