#ifndef Receive_H
#define Receive_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class Receive {
private:
  // MPU6050 accelerometer and gyro data paths.
  float Acc_b[3], Rate_b[3];
  float Acc_a[3], Rate_a[3];
  float Acc_ab[3], Rate_ab[3];
  float Acc_raw[3], Rate_raw[3];

  const int MPU_addr = 0x68;

  float a[2] = {1.64927209f, -0.70219636f};
  float b[3] = {0.01323107f, 0.02646213f, 0.01323107f};
  float accX[6] = {0}, accY[6] = {0}, accZ[6] = {0};
  float gyroX[6] = {0}, gyroY[6] = {0}, gyroZ[6] = {0};

  float offsetRate[3] = {0};
  float offset_Arate[3] = {0};
  float offset_ABrate[3] = {0};
  float offsetRate_Raw[3] = {0};

  float alpha = 0.1f;
  float Beta = 0.005f;
  float a_accx[2] = {0}, a_accy[2] = {0}, a_accz[2] = {0};
  float a_ratex[2] = {0}, a_ratey[2] = {0}, a_ratez[2] = {0};
  float ab_accx[2] = {0}, ab_accy[2] = {0}, ab_accz[2] = {0};
  float ab_ratex[2] = {0}, ab_ratey[2] = {0}, ab_ratez[2] = {0};
  float abs_accx[2] = {0}, abs_accy[2] = {0}, abs_accz[2] = {0};
  float abs_ratex[2] = {0}, abs_ratey[2] = {0}, abs_ratez[2] = {0};

  float *Butterworth_filter(float value, float arr[]);
  void Alpha_filter(float raw_data, float data[]);
  void Alpha_Beta_filter(float raw_data, float data_one[], float data_two[],
                         float dt);

public:
  Receive();

  void Offset();
  void DataRead(float dt);
  void Receive_get(float Rate[], float Acc[], int choose);
};

#endif
