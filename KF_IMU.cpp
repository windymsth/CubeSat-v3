//#include "KF_IMU.h"
////============================================================================
//#define SENSOR_MAX_G 8.0f    //constant g    //tobe fixed to 8g. but IMU need to correct at the same time
//#define SENSOR_MAX_W 2000.0f  //deg/s
//#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
//#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)
////============================================================================
//#define M_PI_F                                3.1415926f
//#define CONSTANTS_ONE_G                       9.80665f    /* m/s^2    */
////============================================================================
//#define GYRO_CALC_TIME                        3000000l //us//
//#define so3_comp_params_Kp 1.0f
//#define so3_comp_params_Ki  0.05f
////============================================================================
//imu_t imu;
////mpu6050
//MPU6050 accelgyro;
//int16_t accx, accy, accz, gyrox, gyroy, gyroz;//
////============================================================================
////void IMU_show() {
////  lg(imu.roll); lg(','); lg(imu.pitch); lg(','); lg(imu.yaw); lg();
////  //lg(',');  //lg(',');
////}
//void MPU6050_Init()
//{
//  accelgyro.initialize();
//  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//}
//void MPU6050_read() {
//  accelgyro.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);//
//}
////===================================================================================================
//void IMU_loop(void) {
//  //-----------------------------------------------------------------------
//  MPU6050_read();//MPU6050_show();
//  //-----------------------------------------------------------------------
//  imu_read_physical();
//  if (!imu_offset_is_ok()) return;
//  imu_offset();
//  //-----------------------------------------------------------------------
//  float dt = 0.01f;   //s
//  static uint32_t tPrev = 0; //us
//  dt = (tPrev > 0) ? (micros() - tPrev) / 1000000.0f : 0; tPrev = micros();
//  //-----------------------------------------------------------------------
//  NonlinearSO3AHRSupdate(
//    imu.gyroReal[0], imu.gyroReal[1], imu.gyroReal[2], //rad/s
//    imu.accbReal[0], imu.accbReal[1], imu.accbReal[2], //m/s^2
//    imu.magbReal[0], imu.magbReal[1], imu.magbReal[2],
//    so3_comp_params_Kp, so3_comp_params_Ki,
//    dt);
//  //-----------------------------------------------------------------------
//  imu_quaternion_euler();
//  //-----------------------------------------------------------------------
//}
//void imu_read_physical(void) {//
//  //----------------------------------------------------------------------
//  imu.accADC[0] = accx;   imu.accADC[1] = accy;     imu.accADC[2] = accz;
//  imu.gyroADC[0] = gyrox; imu.gyroADC[1] = gyroy;   imu.gyroADC[2] = gyroz;
//  imu.magADC[0] = 0.0f;   imu.magADC[1] = 0.0f;     imu.magADC[2] = 0.0f;
//  //----------------------------------------------------------------------
//  for (int i = 0; i < 3; i++)  {//tutn to physical
//    imu.accRaw[i] = (float)imu.accADC[i] * ACC_SCALE * CONSTANTS_ONE_G ;
//    imu.gyroRaw[i] = (float)imu.gyroADC[i] * GYRO_SCALE * M_PI_F / 180.f; //deg/s
//    imu.magRaw[i] = (float)imu.magADC[i];
//  }
//  //----------------------------------------------------------------------
//}
//void imu_offset() {
//  //-------------------------------------------------------------
//  for (int i = 0; i < 3; i++) {
//    imu.gyroReal[i] = imu.gyroRaw[i] - imu.gyroOffset[i];//rad/s
//    imu.accbReal[i] = imu.accRaw[i]; //m/s^2
//    imu.magbReal[i] = imu.magRaw[i]; //
//  }
//  //-------------------------------------------------------------
//}
////===================================================================================================
////need to calc gyro offset before imu start working
//static uint16_t offset_count = 0; static uint32_t startTime = 0; //us
//static float gyro_offsets_sum[3] = { 0.0f, 0.0f, 0.0f };//gyro_offsets[3] = { 0.0f, 0.0f, 0.0f },
//boolean imu_offset_is_ok(void) {//
//  if (imu.readys)
//    return true;
//  else {
//    if (startTime == 0) startTime = micros();
//    //-------------------------------------------------
//    gyro_offsets_sum[0] += imu.gyroRaw[0];
//    gyro_offsets_sum[1] += imu.gyroRaw[1];
//    gyro_offsets_sum[2] += imu.gyroRaw[2];
//    offset_count++;
//    //-------------------------------------------------
//    if (micros() > startTime + GYRO_CALC_TIME) {
//      imu.gyroOffset[0] = gyro_offsets_sum[0] / offset_count;
//      imu.gyroOffset[1] = gyro_offsets_sum[1] / offset_count;
//      imu.gyroOffset[2] = gyro_offsets_sum[2] / offset_count;
//
//      gyro_offsets_sum[0] = 0; gyro_offsets_sum[1] = 0; gyro_offsets_sum[2] = 0;
//      startTime = 0; offset_count = 0; imu.readys = 1;
//
//      return true;
//    }
//    //-------------------------------------------------
//    return false;
//    //-------------------------------------------------
//  }
//  //-------------------------------------------------
//}
////===================================================================================================
////===================================================================================================
////===================================================================================================
////! Auxiliary variables to reduce number of repeated operations
//static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;      /** quaternion of sensor frame relative to auxiliary frame */
//static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;  /** 
//static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation�*/
//static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; //
//static boolean bFilterInit = false;//
////===================================================================================================
//void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz) {
//  //! Using accelerometer, sense the gravity vector. Using magnetometer, sense yaw.
//  //---------------------------------------------------------------------------------------------------
//  float initialRoll, initialPitch;
//  float cosRoll, sinRoll, cosPitch, sinPitch;
//  float magX, magY;
//  float initialHdg, cosHeading, sinHeading;
//  //---------------------------------------------------------------------------------------------------
//  initialRoll = atan2(-ay, -az);
//  initialPitch = atan2(ax, -az);
//  cosRoll = cosf(initialRoll); sinRoll = sinf(initialRoll);
//  cosPitch = cosf(initialPitch); sinPitch = sinf(initialPitch);
//  //---------------------------------------------------------------------------------------------------
//  magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
//  magY = my * cosRoll - mz * sinRoll;
//  //---------------------------------------------------------------------------------------------------
//  initialHdg = atan2f(-magY, magX);
//  cosRoll = cosf(initialRoll * 0.5f);     sinRoll = sinf(initialRoll * 0.5f);
//  cosPitch = cosf(initialPitch * 0.5f);   sinPitch = sinf(initialPitch * 0.5f);
//  cosHeading = cosf(initialHdg * 0.5f);   sinHeading = sinf(initialHdg * 0.5f);
//  //---------------------------------------------------------------------------------------------------
//  q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
//  q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
//  q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
//  q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
//  //---------------------------------------------------------------------------------------------------
//  // auxillary variables to reduce number of repeated operations, for 1st pass
//  q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
//  q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
//  q2q2 = q2 * q2; q2q3 = q2 * q3;
//  q3q3 = q3 * q3;
//  //---------------------------------------------------------------------------------------------------
//}
//void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt) {
//  //---------------------------------------------------------------------------------------------------
//  float recipNorm, halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
//  //---------------------------------------------------------------------------------------------------
//  if (!bFilterInit) {
//    NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz); bFilterInit = true;
//  }
//  //---------------------------------------------------------------------------------------------------
//  if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
//    //-------------------------------------------------------------------------
//    float hx, hy, hz, bx, bz, halfwx, halfwy, halfwz;
//    //-------------------------------------------------------------------------
//    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
//    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
//    //-------------------------------------------------------------------------
//    // Reference direction of Earth's magnetic field
//    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
//    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
//    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
//    bx = sqrt(hx * hx + hy * hy);
//    bz = hz;
//    //-------------------------------------------------------------------------
//    // Estimated direction of magnetic field
//    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
//    //-------------------------------------------------------------------------
//    // Error is sum of cross product between estimated direction and measured direction of field vectors
//    //
//    halfex += (my * halfwz - mz * halfwy);
//    halfey += (mz * halfwx - mx * halfwz);
//    halfez += (mx * halfwy - my * halfwx);
//    //-------------------------------------------------------------------------
//  }
//  //
//  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//    //-------------------------------------------------------------------------
//    float halfvx, halfvy, halfvz; float accNorm = 0;
//    //-------------------------------------------------------------------------
//    // Normalise accelerometer measurement
//    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
//    //-------------------------------------------------------------------------
//    // Estimated direction of gravity and magnetic field
//    halfvx = q1q3 - q0q2; halfvy = q0q1 + q2q3; halfvz = q0q0 - 0.5f + q3q3;
//    // Error is sum of cross product between estimated direction and measured direction of field vectors
//    halfex += ay * halfvz - az * halfvy;
//    halfey += az * halfvx - ax * halfvz;
//    halfez += ax * halfvy - ay * halfvx;
//    //-------------------------------------------------------------------------
//  }
//  //-------------------------------------------------------------------------
//  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
//  if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
//    // Compute and apply integral feedback if enabled
//    if (twoKi > 0.0f) {
//      //integral error scaled by Ki
//      gyro_bias[0] += twoKi * halfex * dt;
//      gyro_bias[1] += twoKi * halfey * dt;
//      gyro_bias[2] += twoKi * halfez * dt;
//      //apply integral feedback
//      gx += gyro_bias[0]; gy += gyro_bias[1]; gz += gyro_bias[2];
//    } else {//prevent integral windup
//      gyro_bias[0] = 0.0f; gyro_bias[1] = 0.0f; gyro_bias[2] = 0.0f;
//    }
//    //-------------------------------------------------------------------------
//    // Apply proportional feedback
//    gx += twoKp * halfex; gy += twoKp * halfey; gz += twoKp * halfez;
//    //-------------------------------------------------------------------------
//  }
//  //-------------------------------------------------------------------------
//  //! Integrate rate of change of quaternion
//#if 0
//  //pre-multiply common factors
//  gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
//#endif
//  //-------------------------------------------------------------------------
//  //
//  // Time derivative of quaternion.
//  dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
//  dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
//  dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
//  dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
//  q0 += dt * dq0; q1 += dt * dq1; q2 += dt * dq2; q3 += dt * dq3;
//  // Normalise quaternion
//  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
//  //-------------------------------------------------------------------------
//  // Auxiliary variables to avoid repeated arithmetic
//  // 
//  q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
//  q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
//  q2q2 = q2 * q2; q2q3 = q2 * q3;
//  q3q3 = q3 * q3;
//  //-------------------------------------------------------------------------
//}
//void imu_quaternion_euler() {
//  //-------------------------------------------------------------------------------------------------
//  imu.q[0] = q0; imu.q[1] = q1; imu.q[2] = q2; imu.q[3] = q3;
//  //-------------------------------------------------------------------------------------------------
//  ///**< init: identity matrix�*/
//  float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };
//  // Convert q->R, This R converts inertial frame to body frame.
//  Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  // 11
//  Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3);  // 12
//  Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2);  // 13
//  Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3);  // 21
//  Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;  // 22
//  Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1);  // 23
//  Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2);  // 31
//  Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1);  // 32
//  Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;  // 33
//  //-------------------------------------------------------------------------------------------------
//  //Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
//  //Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
//  //-------------------------------------------------------------------------------------------------
//  imu.rollRad   = atan2f(Rot_matrix[5], Rot_matrix[8]);
//  imu.pitchRad  = -asinf(Rot_matrix[2]);
//  imu.yawRad    = atan2f(Rot_matrix[1], Rot_matrix[0]);
//  //---------------------------------------------------------------------
//  imu.roll      = imu.rollRad * 180.0f / M_PI_F;
//  imu.pitch     = imu.pitchRad * 180.0f / M_PI_F;
//  imu.yaw       = imu.yawRad * 180.0f / M_PI_F;
//  //-------------------------------------------------------------------------------------------------
//  sys_data.posture_x = imu.roll;
//    Serial.print("Posture:\t");
//    Serial.print(imu.roll); Serial.print("\t");
//    Serial.print(imu.pitch); Serial.print("\t");
//    Serial.println(imu.yaw);
//}
////===================================================================================================
//float invSqrt(float number) {
//  volatile long i;
//  volatile float x, y;
//  volatile const float f = 1.5F;
//  x = number * 0.5F; y = number;
//  i = * (( long * ) &y);
//  i = 0x5f375a86 - ( i >> 1 );
//  y = * (( float * ) &i);
//  y = y * ( f - ( x * y * y ) );
//  return y;
//}
////===================================================================================================




