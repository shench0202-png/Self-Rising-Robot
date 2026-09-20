#include "RobotRuntime.h"

#include <math.h>

static const float kRadToDeg = 57.29577951308232f;
static const float kDegToRad = 0.017453292519943295f;

// Clamps a floating-point value to a closed interval.
static float clampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

// Stores the IMU object and runtime configuration.
ImuEulerRuntime::ImuEulerRuntime(Receive &imu,
                                 const ImuEulerConfig &config)
    : imu_(imu), config_(config) {
}

// Initializes the MPU6050-only quaternion EKF path.
bool ImuEulerRuntime::begin() {
  mpuDetected_ = i2cDevicePresent(config_.mpuAddress);

  Serial.println(mpuDetected_ ? "MPU6050 detected." : "MPU6050 not found.");
  Serial.println("Using 6-axis quaternion EKF (gyro + accelerometer).");

  if (!mpuDetected_) {
    return false;
  }

  imu_.Offset();
  lastUpdateMs_ = millis();
  attitudeInitialized_ = false;
  quaternion_[0] = 1.0f;
  quaternion_[1] = 0.0f;
  quaternion_[2] = 0.0f;
  quaternion_[3] = 0.0f;
  gyroBiasRad_[0] = 0.0f;
  gyroBiasRad_[1] = 0.0f;
  gyroBiasRad_[2] = 0.0f;
  initializeEkfCovariance();
  return true;
}

// Reads MPU6050 data, filters it through Receive, and updates quaternion/Euler.
bool ImuEulerRuntime::update(unsigned long nowMs, AttitudeEstimate &estimate) {
  estimate.dt = computeDt(nowMs);

  imu_.DataRead(estimate.dt);
  imu_.Receive_get(estimate.rate, estimate.acc, config_.filterMode);
  if (!attitudeInitialized_) {
    initializeEkfFromAccel(estimate.acc);
  }
  if (attitudeInitialized_) {
    predictQuaternionEkf(estimate.rate, estimate.dt);
    correctQuaternionEkfFromAccel(estimate.acc);
  }
  quaternionToEuler(estimate.eulerDeg);
  for (uint8_t i = 0; i < 4; ++i) {
    estimate.quaternion[i] = quaternion_[i];
  }

  const float gyroSum = fabsf(estimate.rate[0]) +
                        fabsf(estimate.rate[1]) +
                        fabsf(estimate.rate[2]);
  estimate.isStill = gyroSum < config_.stillGyroSumThresholdDps;
  estimate.eulerNearSingularity = isEulerNearSingularity(estimate.eulerDeg[1]);

  return true;
}

// Probes an I2C address and returns true when a device ACKs.
bool ImuEulerRuntime::i2cDevicePresent(int address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Calculates elapsed time in seconds and protects filters from bad dt values.
float ImuEulerRuntime::computeDt(unsigned long nowMs) {
  float dt = (nowMs - lastUpdateMs_) / 1000.0f;
  lastUpdateMs_ = nowMs;
  if (dt <= 0.0f || dt > 0.2f) {
    dt = 0.01f;
  }
  return dt;
}

// Builds a yaw-zero quaternion from the measured gravity direction.
void ImuEulerRuntime::initializeEkfFromAccel(const float acc[3]) {
  const float ax = acc[0];
  const float ay = acc[1];
  const float az = acc[2];
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1.0e-6f) {
    return;
  }

  const float roll = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float halfRoll = 0.5f * roll;
  const float halfPitch = 0.5f * pitch;
  const float cr = cosf(halfRoll);
  const float sr = sinf(halfRoll);
  const float cp = cosf(halfPitch);
  const float sp = sinf(halfPitch);

  // Same yaw=0 quaternion convention as RL/robo1_env.py.
  quaternion_[0] = cr * cp;
  quaternion_[1] = sr * cp;
  quaternion_[2] = cr * sp;
  quaternion_[3] = -sr * sp;
  normalizeQuaternion();
  gyroBiasRad_[0] = 0.0f;
  gyroBiasRad_[1] = 0.0f;
  gyroBiasRad_[2] = 0.0f;
  initializeEkfCovariance();
  attitudeInitialized_ = true;
}

// Initializes covariance for [attitude error xyz, gyro bias error xyz].
void ImuEulerRuntime::initializeEkfCovariance() {
  const float attitudeStd = config_.initialAttitudeStdDeg * kDegToRad;
  const float biasStd = config_.initialGyroBiasStdDps * kDegToRad;
  const float attitudeVariance = attitudeStd * attitudeStd;
  const float biasVariance = biasStd * biasStd;

  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      covariance_[row][column] = 0.0f;
    }
    covariance_[row][row] = row < 3 ? attitudeVariance : biasVariance;
  }
}

// EKF prediction: propagate the nominal quaternion and the 6x6 error
// covariance with angular rate minus the estimated gyro bias.
void ImuEulerRuntime::predictQuaternionEkf(const float rateDps[3], float dt) {
  if (!attitudeInitialized_ || dt <= 0.0f) {
    return;
  }

  const float omega[3] = {
      rateDps[0] * kDegToRad - gyroBiasRad_[0],
      rateDps[1] * kDegToRad - gyroBiasRad_[1],
      rateDps[2] * kDegToRad - gyroBiasRad_[2]};
  const float omegaNorm = sqrtf(omega[0] * omega[0] +
                                omega[1] * omega[1] +
                                omega[2] * omega[2]);
  const float rotationAngle = omegaNorm * dt;
  float deltaQuaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  if (rotationAngle > 1.0e-7f) {
    const float halfAngle = 0.5f * rotationAngle;
    const float vectorScale = sinf(halfAngle) / omegaNorm;
    deltaQuaternion[0] = cosf(halfAngle);
    deltaQuaternion[1] = omega[0] * vectorScale;
    deltaQuaternion[2] = omega[1] * vectorScale;
    deltaQuaternion[3] = omega[2] * vectorScale;
  } else {
    const float halfDt = 0.5f * dt;
    deltaQuaternion[1] = omega[0] * halfDt;
    deltaQuaternion[2] = omega[1] * halfDt;
    deltaQuaternion[3] = omega[2] * halfDt;
  }

  const float qw = quaternion_[0];
  const float qx = quaternion_[1];
  const float qy = quaternion_[2];
  const float qz = quaternion_[3];
  quaternion_[0] = qw * deltaQuaternion[0] -
                   qx * deltaQuaternion[1] -
                   qy * deltaQuaternion[2] -
                   qz * deltaQuaternion[3];
  quaternion_[1] = qw * deltaQuaternion[1] +
                   qx * deltaQuaternion[0] +
                   qy * deltaQuaternion[3] -
                   qz * deltaQuaternion[2];
  quaternion_[2] = qw * deltaQuaternion[2] -
                   qx * deltaQuaternion[3] +
                   qy * deltaQuaternion[0] +
                   qz * deltaQuaternion[1];
  quaternion_[3] = qw * deltaQuaternion[3] +
                   qx * deltaQuaternion[2] -
                   qy * deltaQuaternion[1] +
                   qz * deltaQuaternion[0];
  normalizeQuaternion();

  float transition[6][6] = {{0.0f}};
  for (uint8_t index = 0; index < 6; ++index) {
    transition[index][index] = 1.0f;
  }
  const float omegaSkew[3][3] = {
      {0.0f, -omega[2], omega[1]},
      {omega[2], 0.0f, -omega[0]},
      {-omega[1], omega[0], 0.0f}};
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t column = 0; column < 3; ++column) {
      transition[row][column] -= omegaSkew[row][column] * dt;
    }
    transition[row][row + 3] = -dt;
  }

  float intermediate[6][6] = {{0.0f}};
  float predictedCovariance[6][6] = {{0.0f}};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        intermediate[row][column] +=
            transition[row][index] * covariance_[index][column];
      }
    }
  }
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        predictedCovariance[row][column] +=
            intermediate[row][index] * transition[column][index];
      }
    }
  }

  const float gyroNoise = config_.gyroNoiseStdDps * kDegToRad;
  const float biasWalkNoise = config_.gyroBiasWalkStdDps * kDegToRad;
  const float attitudeProcessVariance =
      gyroNoise * gyroNoise * dt * dt;
  const float biasProcessVariance =
      biasWalkNoise * biasWalkNoise * dt;
  for (uint8_t index = 0; index < 6; ++index) {
    predictedCovariance[index][index] +=
        index < 3 ? attitudeProcessVariance : biasProcessVariance;
  }
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      covariance_[row][column] = predictedCovariance[row][column];
    }
  }
  stabilizeCovariance();
}

// EKF measurement update. A normalized accelerometer observes gravity, so it
// corrects roll/pitch and observable gyro bias while yaw remains gyro-only.
void ImuEulerRuntime::correctQuaternionEkfFromAccel(const float accG[3]) {
  const float accNorm = sqrtf(accG[0] * accG[0] +
                              accG[1] * accG[1] +
                              accG[2] * accG[2]);
  if (accNorm < config_.accelCorrectionMinG ||
      accNorm > config_.accelCorrectionMaxG) {
    return;
  }

  const float measurement[3] = {
      accG[0] / accNorm,
      accG[1] / accNorm,
      accG[2] / accNorm};
  float expected[3];
  gravityDirectionBody(expected);
  const float innovation[3] = {
      measurement[0] - expected[0],
      measurement[1] - expected[1],
      measurement[2] - expected[2]};

  // Right-multiplicative attitude error gives dh/d(delta_theta) = skew(h).
  float measurementJacobian[3][6] = {{0.0f}};
  measurementJacobian[0][1] = -expected[2];
  measurementJacobian[0][2] = expected[1];
  measurementJacobian[1][0] = expected[2];
  measurementJacobian[1][2] = -expected[0];
  measurementJacobian[2][0] = -expected[1];
  measurementJacobian[2][1] = expected[0];

  float covarianceTimesHTranspose[6][3] = {{0.0f}};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 3; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        covarianceTimesHTranspose[row][column] +=
            covariance_[row][index] * measurementJacobian[column][index];
      }
    }
  }

  const float accelerationDeviation = fabsf(accNorm - 1.0f);
  const float measurementStd = config_.accelDirectionStd *
                               (1.0f + 5.0f * accelerationDeviation);
  const float measurementVariance = measurementStd * measurementStd;
  float innovationCovariance[3][3] = {{0.0f}};
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t column = 0; column < 3; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        innovationCovariance[row][column] +=
            measurementJacobian[row][index] *
            covarianceTimesHTranspose[index][column];
      }
      if (row == column) {
        innovationCovariance[row][column] += measurementVariance;
      }
    }
  }

  float innovationCovarianceInverse[3][3];
  if (!invert3x3(innovationCovariance, innovationCovarianceInverse)) {
    return;
  }

  float kalmanGain[6][3] = {{0.0f}};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 3; ++column) {
      for (uint8_t index = 0; index < 3; ++index) {
        kalmanGain[row][column] +=
            covarianceTimesHTranspose[row][index] *
            innovationCovarianceInverse[index][column];
      }
    }
  }

  float errorState[6] = {0.0f};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t index = 0; index < 3; ++index) {
      errorState[row] += kalmanGain[row][index] * innovation[index];
    }
  }

  const float correctionNorm = sqrtf(errorState[0] * errorState[0] +
                                     errorState[1] * errorState[1] +
                                     errorState[2] * errorState[2]);
  const float maximumCorrection = config_.maxCorrectionDeg * kDegToRad;
  if (correctionNorm > maximumCorrection && correctionNorm > 1.0e-9f) {
    const float correctionScale = maximumCorrection / correctionNorm;
    errorState[0] *= correctionScale;
    errorState[1] *= correctionScale;
    errorState[2] *= correctionScale;
  }
  applyAttitudeError(errorState);

  const float maximumBias = config_.maxGyroBiasDps * kDegToRad;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    gyroBiasRad_[axis] = clampFloat(
        gyroBiasRad_[axis] + errorState[axis + 3],
        -maximumBias,
        maximumBias);
  }

  // Joseph covariance update preserves symmetry and positive semi-definiteness.
  float identityMinusKh[6][6] = {{0.0f}};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      identityMinusKh[row][column] = row == column ? 1.0f : 0.0f;
      for (uint8_t index = 0; index < 3; ++index) {
        identityMinusKh[row][column] -=
            kalmanGain[row][index] * measurementJacobian[index][column];
      }
    }
  }

  float intermediate[6][6] = {{0.0f}};
  float updatedCovariance[6][6] = {{0.0f}};
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        intermediate[row][column] +=
            identityMinusKh[row][index] * covariance_[index][column];
      }
    }
  }
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      for (uint8_t index = 0; index < 6; ++index) {
        updatedCovariance[row][column] +=
            intermediate[row][index] * identityMinusKh[column][index];
      }
      for (uint8_t index = 0; index < 3; ++index) {
        updatedCovariance[row][column] +=
            measurementVariance * kalmanGain[row][index] *
            kalmanGain[column][index];
      }
    }
  }
  for (uint8_t row = 0; row < 6; ++row) {
    for (uint8_t column = 0; column < 6; ++column) {
      covariance_[row][column] = updatedCovariance[row][column];
    }
  }
  stabilizeCovariance();
}

void ImuEulerRuntime::applyAttitudeError(const float deltaTheta[3]) {
  const float angle = sqrtf(deltaTheta[0] * deltaTheta[0] +
                            deltaTheta[1] * deltaTheta[1] +
                            deltaTheta[2] * deltaTheta[2]);
  float correction[4] = {1.0f,
                         0.5f * deltaTheta[0],
                         0.5f * deltaTheta[1],
                         0.5f * deltaTheta[2]};
  if (angle > 1.0e-7f) {
    const float halfAngle = 0.5f * angle;
    const float vectorScale = sinf(halfAngle) / angle;
    correction[0] = cosf(halfAngle);
    correction[1] = deltaTheta[0] * vectorScale;
    correction[2] = deltaTheta[1] * vectorScale;
    correction[3] = deltaTheta[2] * vectorScale;
  }

  const float qw = quaternion_[0];
  const float qx = quaternion_[1];
  const float qy = quaternion_[2];
  const float qz = quaternion_[3];
  quaternion_[0] = qw * correction[0] - qx * correction[1] -
                   qy * correction[2] - qz * correction[3];
  quaternion_[1] = qw * correction[1] + qx * correction[0] +
                   qy * correction[3] - qz * correction[2];
  quaternion_[2] = qw * correction[2] - qx * correction[3] +
                   qy * correction[0] + qz * correction[1];
  quaternion_[3] = qw * correction[3] + qx * correction[2] -
                   qy * correction[1] + qz * correction[0];
  normalizeQuaternion();
}

void ImuEulerRuntime::gravityDirectionBody(float gravity[3]) const {
  const float qw = quaternion_[0];
  const float qx = quaternion_[1];
  const float qy = quaternion_[2];
  const float qz = quaternion_[3];
  gravity[0] = 2.0f * (qx * qz - qw * qy);
  gravity[1] = 2.0f * (qw * qx + qy * qz);
  gravity[2] = qw * qw - qx * qx - qy * qy + qz * qz;
}

bool ImuEulerRuntime::invert3x3(const float input[3][3],
                                float output[3][3]) const {
  const float c00 = input[1][1] * input[2][2] -
                    input[1][2] * input[2][1];
  const float c01 = input[1][2] * input[2][0] -
                    input[1][0] * input[2][2];
  const float c02 = input[1][0] * input[2][1] -
                    input[1][1] * input[2][0];
  const float c10 = input[0][2] * input[2][1] -
                    input[0][1] * input[2][2];
  const float c11 = input[0][0] * input[2][2] -
                    input[0][2] * input[2][0];
  const float c12 = input[0][1] * input[2][0] -
                    input[0][0] * input[2][1];
  const float c20 = input[0][1] * input[1][2] -
                    input[0][2] * input[1][1];
  const float c21 = input[0][2] * input[1][0] -
                    input[0][0] * input[1][2];
  const float c22 = input[0][0] * input[1][1] -
                    input[0][1] * input[1][0];
  const float determinant = input[0][0] * c00 +
                            input[0][1] * c01 +
                            input[0][2] * c02;
  if (!isfinite(determinant) || fabsf(determinant) < 1.0e-12f) {
    return false;
  }

  const float inverseDeterminant = 1.0f / determinant;
  output[0][0] = c00 * inverseDeterminant;
  output[0][1] = c10 * inverseDeterminant;
  output[0][2] = c20 * inverseDeterminant;
  output[1][0] = c01 * inverseDeterminant;
  output[1][1] = c11 * inverseDeterminant;
  output[1][2] = c21 * inverseDeterminant;
  output[2][0] = c02 * inverseDeterminant;
  output[2][1] = c12 * inverseDeterminant;
  output[2][2] = c22 * inverseDeterminant;
  return true;
}

void ImuEulerRuntime::stabilizeCovariance() {
  for (uint8_t row = 0; row < 6; ++row) {
    if (!isfinite(covariance_[row][row]) ||
        covariance_[row][row] < 1.0e-9f) {
      covariance_[row][row] = 1.0e-9f;
    }
    for (uint8_t column = row + 1; column < 6; ++column) {
      float symmetricValue =
          0.5f * (covariance_[row][column] + covariance_[column][row]);
      if (!isfinite(symmetricValue)) {
        symmetricValue = 0.0f;
      }
      covariance_[row][column] = symmetricValue;
      covariance_[column][row] = symmetricValue;
    }
  }
}

// Converts quaternion to ZYX principal Euler angles. Pitch is intentionally
// limited to [-90, 90] deg; beyond 90 deg, roll/yaw shift by about 180 deg.
void ImuEulerRuntime::quaternionToEuler(float eulerDeg[3]) const {
  const float qw = quaternion_[0];
  const float qx = quaternion_[1];
  const float qy = quaternion_[2];
  const float qz = quaternion_[3];

  const float sinRoll = 2.0f * (qw * qx + qy * qz);
  const float cosRoll = 1.0f - 2.0f * (qx * qx + qy * qy);
  const float roll = atan2f(sinRoll, cosRoll);

  float sinPitch = 2.0f * (qw * qy - qz * qx);
  sinPitch = clampFloat(sinPitch, -1.0f, 1.0f);
  const float pitch = asinf(sinPitch);

  const float sinYaw = 2.0f * (qw * qz + qx * qy);
  const float cosYaw = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float yaw = atan2f(sinYaw, cosYaw);

  eulerDeg[0] = roll * kRadToDeg;
  eulerDeg[1] = pitch * kRadToDeg;
  eulerDeg[2] = yaw * kRadToDeg;
}

void ImuEulerRuntime::normalizeQuaternion() {
  const float norm = sqrtf(quaternion_[0] * quaternion_[0] +
                           quaternion_[1] * quaternion_[1] +
                           quaternion_[2] * quaternion_[2] +
                           quaternion_[3] * quaternion_[3]);
  if (norm < 1.0e-9f) {
    quaternion_[0] = 1.0f;
    quaternion_[1] = 0.0f;
    quaternion_[2] = 0.0f;
    quaternion_[3] = 0.0f;
    return;
  }
  quaternion_[0] /= norm;
  quaternion_[1] /= norm;
  quaternion_[2] /= norm;
  quaternion_[3] /= norm;
}

// Flags the pitch region where full ZYX Euler yaw/roll coupling becomes unsafe.
bool ImuEulerRuntime::isEulerNearSingularity(float pitchDeg) const {
  return fabsf(pitchDeg) >= config_.singularityPitchLimitDeg;
}

// Packs the four policy observations in the trained order.
void buildPolicyInput(float rollRad,
                      float pitchRad,
                      float servo1TargetRad,
                      float servo2TargetRad,
                      float observation[NN_INPUT_DIM]) {
  observation[0] = rollRad;
  observation[1] = pitchRad;
  observation[2] = servo1TargetRad;
  observation[3] = servo2TargetRad;
}

// Runs NN inference and optionally clamps actions before servo use.
bool runPolicy(NeuralNetwork &policy,
               const float observation[NN_INPUT_DIM],
               float action[NN_OUTPUT_DIM],
               bool clampAction) {
  if (!policy.forward(observation, action)) {
    return false;
  }

  if (clampAction) {
    for (uint8_t i = 0; i < NN_OUTPUT_DIM; ++i) {
      action[i] = NeuralNetwork::clamp(action[i], -1.0f, 1.0f);
    }
  }

  return true;
}

// Builds policy input and runs the policy as one convenience call.
bool computePolicyIo(NeuralNetwork &policy,
                     float rollRad,
                     float pitchRad,
                     float servo1TargetRad,
                     float servo2TargetRad,
                     PolicyIo &io,
                     bool clampAction) {
  buildPolicyInput(rollRad, pitchRad, servo1TargetRad, servo2TargetRad,
                   io.observation);
  return runPolicy(policy, io.observation, io.action, clampAction);
}

// Configures one ESP32 LEDC output for standard 50 Hz servo pulses.
bool LedcServo::begin(uint8_t pin,
                      uint32_t frequencyHz,
                      int minPulseUs,
                      int maxPulseUs) {
  if (frequencyHz == 0 || minPulseUs <= 0 || maxPulseUs <= minPulseUs) {
    return false;
  }

  if (attached_) {
    detach();
  }

  pin_ = pin;
  frequencyHz_ = frequencyHz;
  minPulseUs_ = minPulseUs;
  maxPulseUs_ = maxPulseUs;
  attached_ = ledcAttach(pin_, frequencyHz_, kResolutionBits);
  return attached_;
}

// Maps 0..180 degrees linearly into the configured servo pulse span.
bool LedcServo::writeDegrees(int angleDeg) {
  if (!attached_) {
    return false;
  }

  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  const int pulseSpanUs = maxPulseUs_ - minPulseUs_;
  const int pulseUs = minPulseUs_ +
                      (pulseSpanUs * angleDeg + 90) / 180;
  return ledcWrite(pin_, pulseUsToDuty(pulseUs));
}

void LedcServo::detach() {
  if (!attached_) {
    return;
  }
  ledcWrite(pin_, 0);
  ledcDetach(pin_);
  attached_ = false;
}

// Converts pulse width to LEDC duty using 64-bit math to avoid overflow.
uint32_t LedcServo::pulseUsToDuty(int pulseUs) const {
  const uint32_t maxDuty = (1UL << kResolutionBits) - 1UL;
  const uint64_t scaled =
      (uint64_t)pulseUs * (uint64_t)frequencyHz_ * (uint64_t)maxDuty;
  return (uint32_t)((scaled + 500000ULL) / 1000000ULL);
}

// Creates an unattached servo controller.
RobotServoControl::RobotServoControl() {
}

// Attaches both servo objects and writes zero-radian targets.
bool RobotServoControl::begin(LedcServo &servo1, LedcServo &servo2,
                              const ServoControlConfig &config) {
  servo1_ = &servo1;
  servo2_ = &servo2;
  config_ = config;

  const bool servo1Ready = servo1_->begin(
      config_.servo1Pin,
      (uint32_t)config_.servoHz,
      1500 + config_.servo1OffsetUs - config_.servo1PulseRangeUs,
      1500 + config_.servo1OffsetUs + config_.servo1PulseRangeUs);
  const bool servo2Ready = servo2_->begin(
      config_.servo2Pin,
      (uint32_t)config_.servoHz,
      1500 + config_.servo2OffsetUs - config_.servo2PulseRangeUs,
      1500 + config_.servo2OffsetUs + config_.servo2PulseRangeUs);
  if (!servo1Ready || !servo2Ready) {
    servo1_->detach();
    servo2_->detach();
    servo1_ = nullptr;
    servo2_ = nullptr;
    return false;
  }

  setTargetsRad(0.0f, 0.0f);
  return writeTargets();
}

// Returns both target angles to the neutral zero-radian pose.
void RobotServoControl::resetTargets() {
  setTargetsRad(0.0f, 0.0f);
  writeTargets();
}

// Stores target angles after applying each physical servo's target limit.
void RobotServoControl::setTargetsRad(float servo1TargetRad,
                                      float servo2TargetRad) {
  servo1TargetRad_ = clampFloat(servo1TargetRad,
                                -config_.servo1TargetLimitRad,
                                config_.servo1TargetLimitRad);
  servo2TargetRad_ = clampFloat(servo2TargetRad,
                                -config_.servo2TargetLimitRad,
                                config_.servo2TargetLimitRad);
}

// Converts NN action increments into servo target angles.
bool RobotServoControl::applyPolicyAction(const float action[NN_OUTPUT_DIM],
                                          bool writeNow) {
  if (action == nullptr) {
    return false;
  }

  const float action0 = clampFloat(action[0], -1.0f, 1.0f);
  const float action1 = clampFloat(action[1], -1.0f, 1.0f);

  setTargetsRad(servo1TargetRad_ + action0 * config_.targetDeltaRad,
                servo2TargetRad_ + action1 * config_.targetDeltaRad);

  if (writeNow) {
    return writeTargets();
  }
  return true;
}

// Sends the current target angles to the attached servo objects.
bool RobotServoControl::writeTargets() {
  if (servo1_ == nullptr || servo2_ == nullptr) {
    return false;
  }

  const bool servo1Written = servo1_->writeDegrees(
      targetRadToServoDegrees(servo1TargetRad_,
                              config_.invertServo1,
                              config_.servo1MinDeg,
                              config_.servo1MaxDeg));
  const bool servo2Written = servo2_->writeDegrees(
      targetRadToServoDegrees(servo2TargetRad_,
                              config_.invertServo2,
                              config_.servo2MinDeg,
                              config_.servo2MaxDeg));
  return servo1Written && servo2Written;
}

// Maps radians around zero to the servo API range and applies the configured
// final output limit after direction inversion.
int RobotServoControl::targetRadToServoDegrees(float targetRad,
                                               bool invert,
                                               int minDeg,
                                               int maxDeg) const {
  int targetDeg = (int)roundf(targetRad * kRadToDeg);
  targetDeg = (int)clampFloat((float)targetDeg, -90.0f, 90.0f);
  if (invert) {
    targetDeg = -targetDeg;
  }
  const int commandDeg = targetDeg + 90;
  const int lowerDeg = minDeg < maxDeg ? minDeg : maxDeg;
  const int upperDeg = minDeg < maxDeg ? maxDeg : minDeg;
  return (int)clampFloat((float)commandDeg,
                         (float)lowerDeg,
                         (float)upperDeg);
}
