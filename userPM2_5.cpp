#include "userPM2_5.h"

//SoftwareSerial G5Serial(A14, A15);

#define LENG 60
byte pBuf[LENG];

G5ST_type g;

bool flagee = false;

unsigned int G5Val(byte *buf, int a, int b) {
  unsigned int v = buf[a] << 8;
  if (b > 0) v += buf[b];
  return v;
}

void G5Exp() {
  //------------------------------------------------
  g.len = G5Val(&pBuf[2], 0, 1);     //[0][1]=2*13+2 
  //------------------------------------------------
  
  g.cf10 = G5Val(&pBuf[2], 2, 3);    //[2][3]=PM1.0=ug/m3
  g.cf25 = G5Val(&pBuf[2], 4, 5);    //[4][5]=PM2.5=ug/m3
  g.cf1X = G5Val(&pBuf[2], 6, 7);    //[6][7]=PM10 =ug/m3
  //------------------------------------------------
  
  g.pm10 = G5Val(&pBuf[2], 8, 9);     //[8][9]  =PM1.0=ug/m3
  g.pm25 = G5Val(&pBuf[2], 10, 11);   //[10][11]=PM2.5=ug/m3
  //g.pm1X = G5Val(&pBuf[2], 12, 13);   //[12][13]=PM10 =ug/m3
  //------------------------------------------------
  
  g.um03 = G5Val(&pBuf[2], 14, 15);   //[14][15]=0.3um
  g.um05 = G5Val(&pBuf[2], 16, 17);   //[16][17]=0.5um
  g.um10 = G5Val(&pBuf[2], 18, 19);   //[18][19]=1.0um
  g.um25 = G5Val(&pBuf[2], 20, 21);   //[20][21]=2.5um
  g.um50 = G5Val(&pBuf[2], 22, 23);   //[22][23]=5.0um
  g.um1X = G5Val(&pBuf[2], 24, 25);   //[24][25]=10.0um
  //------------------------------------------------
  g.hcho = G5Val(&pBuf[2], 26, 27);   //[26][27]=x/1000mg/m3
  g.tmps = G5Val(&pBuf[2], 28, 29);   //[28][29]=x/10�
  g.hums = G5Val(&pBuf[2], 30, 31);   //[30][31]=x/10%
  //------------------------------------------------  
  g.empt = G5Val(&pBuf[2], 32, 33);   //[32][33]�
  //------------------------------------------------
  g.vers = G5Val(&pBuf[2], 34, -1);   //[34]
  g.errs = G5Val(&pBuf[2], 35, -1);   //[35]
  g.chek = G5Val(&pBuf[2], 36, 37);   //[36][37]
  //------------------------------------------------
}

void G5Feed(byte c) {
	
  static int state = 0;
  static int count = 0;
//  pBuf[count++] = c;
//  if (count == 30) G5Exp();
  if( 0x42 == c && state == 0 ) state = 1;
  else if(state == 0) {
    count = 0;
    return ;
  }
	if( state == 1 )	pBuf[count++] = c;
  if (count == 30) {
        count = 0;
        state = 0;
        G5Exp();
        flagee = true;
  }
  if (count >= 60) count = 0;
}

void G5_Init(void) {
	Serial2.begin(9600);
  char cnt = 0;
  while( cnt-- >0) { 
  G5_Handle();
  G5_LV();
  Serial.println(sys_data.pm25);
  delay(10);
  }
}

void G5_Handle(void) {
	if(Serial2.available()) {
		byte c = Serial2.read();
//		Serial.print(c ,HEX);Serial.print(" | ");
		G5Feed(c);
	}
}

void G5_LV(void) {
  if(g.pm25 < 500)
  sys_data.pm25 = g.pm25;
  int chekTemp = 0;
  for (int i = 0; i < 30; ++i) {
    chekTemp += pBuf[i];
  }
  if (chekTemp == g.chek) {
    if( g.pm25 < 0 )
      g.pm25 = 0;
    sys_data.pm25 = g.pm25;
  }
}




