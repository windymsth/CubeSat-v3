#include "uHMC5883L.h"

#define sample_num_mdate  200
//Compass
HMC5883L compass;
bool compass_found = false;

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

volatile float Mxyz[3];

void Compass_hmc5883l_Init() {
  if (!compass.begin())
  {
    Serial.println("compass init failed\n");
    return;
  } else {
    compass_found = true;
    Serial.println("compass init done.\n");
  }
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_1);
}

float getHeading() {
  float heading;
  getCompass_Data();
  //get_calibration_Data(); // compass data has been calibrated here
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
  return heading;
}

void getCompass_Data()
{
  Vector raw = compass.readRaw();
  Mxyz[0] = raw.XAxis;
  Mxyz[1] = raw.YAxis;
  Mxyz[2] = raw.ZAxis;
  //  Serial.print("Compass:\t");
  //  Serial.print(Mxyz[0]); Serial.print("\t");
  //  Serial.print(Mxyz[1]); Serial.print("\t");
  //  Serial.println(Mxyz[2]);
}

void get_one_sample_date_mxyz() {
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

void get_calibration_Data () {
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();

    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];

  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;
}



