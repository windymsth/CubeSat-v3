#include "SHT31.h"

// SHT31 Temperature&Huminity Sensor
SHT31 sht31 = SHT31();

void SHT31_Init() {
		boolean sht31Status = sht31.begin();
    if( sht31Status == true ) Serial.println("SHT31 init done.\n");
    else Serial.println("SHT31 init failed.\n");
}

void SHT31_Handle(void) {
    if ( sht31.getTempHum() ) { 
      sys_data.inside_temp = sht31.getTemperature();
      sys_data.inside_humi = sht31.getHumidity();
      sht31Flag = true;
    }
    else {
      sht31Flag = false;
    }
}

SHT31::SHT31() {
}

boolean SHT31::begin(uint8_t i2caddr) {
//  Wire.begin();
  _i2caddr = i2caddr;
  reset();
  if( readStatus() != 0x00)
//  readStatus() == 0x40
    return true;
  else 
    return false;
}

float SHT31::getTemperature(void) {
  if (! getTempHum()) return NAN;
  return temp;
}


float SHT31::getHumidity(void) {
  if (! getTempHum()) return NAN;
  return humidity;
}

uint16_t SHT31::readStatus(void) {
  writeCommand(SHT31_READSTATUS);
  Wire.requestFrom(_i2caddr, (uint8_t)3);
  uint16_t stat = Wire.read();
  stat <<= 8;
  stat |= Wire.read();
  Serial.println(stat, HEX);
  return stat;
}

void SHT31::reset(void) {
  writeCommand(SHT31_SOFTRESET);
  delay(10);
}

void SHT31::heater(boolean h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
}

uint8_t SHT31::crc8(const uint8_t *data, int len) {
  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);
  
  for ( int j = len; j; --j ) {
      crc ^= *data++;

      for ( int i = 8; i; --i ) {
	crc = ( crc & 0x80 )
	  ? (crc << 1) ^ POLYNOMIAL
	  : (crc << 1);
      }
  }
  return crc; 
}

void SHT31::StartMes(void) {
	
	// writeCommand(SHT31_MEAS_LOWREP);
	// Wire.requestFrom(_i2caddr, (uint8_t)6);
	// Serial.println( Wire.available(), DEC);
}

boolean SHT31::getTempHum(void) {
  uint8_t readbuffer[6];
	//*****�
//  writeCommand(SHT31_MEAS_HIGHREP);
  writeCommand(SHT31_MEAS_LOWREP);
	delay(5);
  Wire.requestFrom(_i2caddr, (uint8_t)6);
  if (Wire.available() != 6)	return false;
  //*****�
  
  for (uint8_t i=0; i<6; i++) {
    readbuffer[i] = Wire.read();
  }
  uint16_t ST, SRH;
  ST = readbuffer[0];
  ST <<= 8;
  ST |= readbuffer[1];

  if (readbuffer[2] != crc8(readbuffer, 2)) return false;

  SRH = readbuffer[3];
  SRH <<= 8;
  SRH |= readbuffer[4];

  if (readbuffer[5] != crc8(readbuffer+3, 2)) return false;
 
  double stemp = ST;
  stemp *= 175;
  stemp /= 0xffff;
  stemp = -45 + stemp;
  temp = stemp;
	
  double shum = SRH;
  shum *= 100;
  shum /= 0xFFFF;
  humidity = shum;
	
  return true;
}

void SHT31::writeCommand(uint16_t cmd) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();      
}



