#include "userRGB.h"
#include "stdlib.h"

#define RGB_PIN 27

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, RGB_PIN,
                            NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS,
                            NEO_GRB            + NEO_KHZ800);
//
const uint16_t colors[] = {
  matrix.Color(79, 0, 255),
  matrix.Color(0, 255, 89),
  matrix.Color(255, 255, 0)
};
/*
	WS2812B_Init
	WS2812B�
*/
void WS2812B_Init(void) {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(5);
  matrix.setTextColor(colors[0]); //�
}
//
int pass = 0;
/*
	WS2812B_Handle
	WS2812B
*/
void WS2812B_Handle(void) {
  static uint8_t rgbCount = 0;
  char string[2];
  char txt[30];
  sprintf( txt, "COMMSAT PM2.5:%03d", sys_data.pm25);
  int n = ( strlen(txt) + 1) * 6;    //
  static int x = matrix.width();

  if (++ rgbCount >= 10) {
    rgbCount = 0;
    matrix.fillScreen(0);
    matrix.setCursor(x, 0);
    matrix.print(txt);
    if (--x < -n) {
      x = matrix.width();
      if (++pass >= 3) pass = 0;              
      matrix.setTextColor(colors[pass]);     
    }
    matrix.show();
  }
}




