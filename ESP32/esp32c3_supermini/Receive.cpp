#include "Receive.h"

Receive::Receive() {
}

// Wakes the MPU6050 and estimates gyro offsets while the board is stationary.
void Receive::Offset() {
  Wire.beginTransmission(MPU_addr);
  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 connection OK.");
  } else {
    Serial.println("MPU6050 connection failed.");
    while (true) {
      delay(10);
    }
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  const int sampleCount = 100;
  float rawRateSum[3] = {0.0f, 0.0f, 0.0f};
  float butterworthRateSum[3] = {0.0f, 0.0f, 0.0f};
  float alphaRateSum[3] = {0.0f, 0.0f, 0.0f};
  float alphaBetaRateSum[3] = {0.0f, 0.0f, 0.0f};

  // First pass establishes the unfiltered gyro bias. Accelerometer values
  // retain gravity and therefore are not offset-calibrated.
  for (int sample = 0; sample < sampleCount; ++sample) {
    DataRead(0.01f);
    for (uint8_t axis = 0; axis < 3; ++axis) {
      rawRateSum[axis] += Rate_raw[axis];
    }
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    offsetRate_Raw[axis] = rawRateSum[axis] / sampleCount;
  }

  // Second pass establishes the bias of every selectable gyro filter path.
  for (int sample = 0; sample < sampleCount; ++sample) {
    DataRead(0.01f);
    for (uint8_t axis = 0; axis < 3; ++axis) {
      butterworthRateSum[axis] += Rate_b[axis];
      alphaRateSum[axis] += Rate_a[axis];
      alphaBetaRateSum[axis] += Rate_ab[axis];
    }
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    offsetRate[axis] = butterworthRateSum[axis] / sampleCount;
    offset_Arate[axis] = alphaRateSum[axis] / sampleCount;
    offset_ABrate[axis] = alphaBetaRateSum[axis] / sampleCount;
  }

  Serial.println("Gyro offset calculation done.");
}

// Reads MPU6050 acceleration and angular rate, then updates all filter paths.
void Receive::DataRead(float dt) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);  // Accelerometer: +/-8 g.
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);

  const int16_t accXRawCount = Wire.read() << 8 | Wire.read();
  const int16_t accYRawCount = Wire.read() << 8 | Wire.read();
  const int16_t accZRawCount = Wire.read() << 8 | Wire.read();
  const float accXRaw = (float)accXRawCount / 4096.0f;
  const float accYRaw = (float)accYRawCount / 4096.0f;
  const float accZRaw = (float)accZRawCount / 4096.0f;

  Butterworth_filter(accXRaw, accX);
  Butterworth_filter(accYRaw, accY);
  Butterworth_filter(accZRaw, accZ);
  Acc_b[0] = accX[0];
  Acc_b[1] = accY[0];
  Acc_b[2] = accZ[0];

  Alpha_filter(accXRaw, a_accx);
  Alpha_filter(accYRaw, a_accy);
  Alpha_filter(accZRaw, a_accz);
  Acc_a[0] = a_accx[0];
  Acc_a[1] = a_accy[0];
  Acc_a[2] = a_accz[0];

  Alpha_Beta_filter(accXRaw, ab_accx, abs_accx, dt);
  Alpha_Beta_filter(accYRaw, ab_accy, abs_accy, dt);
  Alpha_Beta_filter(accZRaw, ab_accz, abs_accz, dt);
  Acc_ab[0] = ab_accx[0];
  Acc_ab[1] = ab_accy[0];
  Acc_ab[2] = ab_accz[0];

  Acc_raw[0] = accXRaw;
  Acc_raw[1] = accYRaw;
  Acc_raw[2] = accZRaw;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x08);  // Gyroscope: +/-500 deg/s.
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);

  const int16_t gyroXRawCount = Wire.read() << 8 | Wire.read();
  const int16_t gyroYRawCount = Wire.read() << 8 | Wire.read();
  const int16_t gyroZRawCount = Wire.read() << 8 | Wire.read();
  const float gyroXRaw = (float)gyroXRawCount / 65.5f;
  const float gyroYRaw = (float)gyroYRawCount / 65.5f;
  const float gyroZRaw = (float)gyroZRawCount / 65.5f;

  Butterworth_filter(gyroXRaw, gyroX);
  Butterworth_filter(gyroYRaw, gyroY);
  Butterworth_filter(gyroZRaw, gyroZ);
  Rate_b[0] = gyroX[0] - offsetRate[0];
  Rate_b[1] = gyroY[0] - offsetRate[1];
  Rate_b[2] = gyroZ[0] - offsetRate[2];

  Alpha_filter(gyroXRaw, a_ratex);
  Alpha_filter(gyroYRaw, a_ratey);
  Alpha_filter(gyroZRaw, a_ratez);
  Rate_a[0] = a_ratex[0] - offset_Arate[0];
  Rate_a[1] = a_ratey[0] - offset_Arate[1];
  Rate_a[2] = a_ratez[0] - offset_Arate[2];

  Alpha_Beta_filter(gyroXRaw, ab_ratex, abs_ratex, dt);
  Alpha_Beta_filter(gyroYRaw, ab_ratey, abs_ratey, dt);
  Alpha_Beta_filter(gyroZRaw, ab_ratez, abs_ratez, dt);
  Rate_ab[0] = ab_ratex[0] - offset_ABrate[0];
  Rate_ab[1] = ab_ratey[0] - offset_ABrate[1];
  Rate_ab[2] = ab_ratez[0] - offset_ABrate[2];

  Rate_raw[0] = gyroXRaw - offsetRate_Raw[0];
  Rate_raw[1] = gyroYRaw - offsetRate_Raw[1];
  Rate_raw[2] = gyroZRaw - offsetRate_Raw[2];
}

void Receive::Receive_get(float Rate[], float Acc[], int choose) {
  if (choose == 1) {
    Rate[0] = Rate_b[0]; Rate[1] = Rate_b[1]; Rate[2] = Rate_b[2];
    Acc[0] = Acc_b[0]; Acc[1] = Acc_b[1]; Acc[2] = Acc_b[2];
  } else if (choose == 2) {
    Rate[0] = Rate_a[0]; Rate[1] = Rate_a[1]; Rate[2] = Rate_a[2];
    Acc[0] = Acc_a[0]; Acc[1] = Acc_a[1]; Acc[2] = Acc_a[2];
  } else if (choose == 3) {
    Rate[0] = Rate_ab[0]; Rate[1] = Rate_ab[1]; Rate[2] = Rate_ab[2];
    Acc[0] = Acc_ab[0]; Acc[1] = Acc_ab[1]; Acc[2] = Acc_ab[2];
  } else {
    Rate[0] = Rate_raw[0]; Rate[1] = Rate_raw[1]; Rate[2] = Rate_raw[2];
    Acc[0] = Acc_raw[0]; Acc[1] = Acc_raw[1]; Acc[2] = Acc_raw[2];
  }
}

float *Receive::Butterworth_filter(float value, float arr[]) {
  arr[3] = value;
  arr[0] = a[0] * arr[1] + a[1] * arr[2] +
           b[0] * arr[3] + b[1] * arr[4] + b[2] * arr[5];
  arr[2] = arr[1];
  arr[1] = arr[0];
  arr[5] = arr[4];
  arr[4] = arr[3];
  return arr;
}

void Receive::Alpha_filter(float rawData, float data[]) {
  data[0] = data[1] + alpha * (rawData - data[1]);
  data[1] = data[0];
}

void Receive::Alpha_Beta_filter(float rawData,
                                float position[],
                                float velocity[],
                                float dt) {
  if (dt <= 0.0f) {
    dt = 0.01f;
  }

  const float predictedPosition = position[1] + dt * velocity[1];
  const float residual = rawData - predictedPosition;
  position[0] = predictedPosition + alpha * residual;
  velocity[0] = velocity[1] + (Beta / dt) * residual;
  position[1] = position[0];
  velocity[1] = velocity[0];
}
