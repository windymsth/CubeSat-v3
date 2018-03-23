#ifndef __USERPM2_5_H__
#define __USERPM2_5_H__

//	arduino include
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "sensor.h"

//闁猴拷閿熶粙鎸婇ˉ锟�5闁汇劌瀚弳鐔煎箲椤旂晫澹愮�殿噯鎷�
typedef struct _G5_type { //0x42;0x4d;
  //------------------------------------------------
  int len;    //[0][1]=2*13+2 閻㈩垎鍥ㄦ瘣閹艰揪鎷�
  //------------------------------------------------
  //闁哄秴娲ら崳顖涳紣濡ゅ啰鐓堥柣妞绘櫆缁佹寧鎯旈敂鑲╃CF=1闁挎冻鎷�
  int cf10;   //[2][3]=PM1.0=ug/m3
  int cf25;   //[4][5]=PM2.5=ug/m3
  int cf1X;   //[6][7]=PM10 =ug/m3
  //------------------------------------------------
  //濠㈠爢鍕瘻闁绘粠鍨伴。銊︾▔閿燂拷
  int pm10;   //[8][9]  =PM1.0=ug/m3
  int pm25;   //[10][11]=PM2.5=ug/m3
  int pm1X;   //[12][13]=PM10 =ug/m3
  //------------------------------------------------
  //0.1闁告娲ㄩ埞鏍ь潩閺傛崘鍘瑇xum闁烩晜娼欑欢鐐达紣濡ゅ啰鐓堥柣妞绘櫃闁叉粓寮敓锟�
  unsigned int um03;   //[14][15]=0.3um
  int um05;   //[16][17]=0.5um
  int um10;   //[18][19]=1.0um
  int um25;   //[20][21]=2.5um
  int um50;   //[22][23]=5.0um
  int um1X;   //[24][25]=10.0um
  //------------------------------------------------
  //闁汇垺鐓￠崯妤�霉閹惧啿顔婇柡浣规緲閿熸枻鎷�/1000mg/m3//HCHO
  int hcho;   //[26][27]=x/1000mg/m3
  //------------------------------------------------
  //婵炴挴鏅涚�癸拷/10闁斥晪鎷�
  int tmps;   //[28][29]=x/10闁斥晪鎷�
  //------------------------------------------------
  //婵狅拷閸喖顔�/10%  
  int hums;   //[30][31]=x/10%
  //------------------------------------------------  
  int empt;   //[32][33]濞ｅ洦绻勯弳锟�
  //------------------------------------------------
  int vers;   //[34]闁绘鐗婂﹢浼村矗閿燂拷
  int errs;   //[35]闂佹寧鐟ㄩ銈夊矗閿燂拷
  //------------------------------------------------
  int chek;   //[36][37]闁哄稄绻濋悰娆撳椽閿燂拷
  //------------------------------------------------
} G5ST_type;

extern G5ST_type g;
extern bool flagee;

void G5Feed(byte c);
void G5_Init(void);
void G5_Handle(void);
void G5Exp();
void G5_LV(void);

#endif


