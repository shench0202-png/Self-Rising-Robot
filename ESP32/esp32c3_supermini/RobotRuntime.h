#ifndef ROBOT_RUNTIME_H
#define ROBOT_RUNTIME_H

#include <Arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>

#include "NeuralNetwork.h"
#include "Receive.h"

#if !defined(ARDUINO_ARCH_ESP32)
#error "esp32c3_supermini requires the Espressif ESP32 Arduino core."
#endif

// Runtime settings for the MPU6050-only quaternion error-state EKF.
struct ImuEulerConfig {
  int mpuAddress = 0x68;
  int filterMode = 3;  // Receive_get(): 1=Butterworth, 2=Alpha, 3=Alpha-Beta, 4=Raw.
  float stillGyroSumThresholdDps = 3.0f;
  float singularityPitchLimitDeg = 85.0f;
  float accelCorrectionMinG = 0.70f;
  float accelCorrectionMaxG = 1.30f;
  float gyroNoiseStdDps = 1.0f;
  float gyroBiasWalkStdDps = 0.05f;
  float accelDirectionStd = 0.06f;
  float initialAttitudeStdDeg = 5.0f;
  float initialGyroBiasStdDps = 1.0f;
  float maxCorrectionDeg = 20.0f;
  float maxGyroBiasDps = 10.0f;
};

// Holds one IMU update result. Neural-network policy only needs eulerDeg[0],
// eulerDeg[1], servo1 target, and servo2 target.
struct AttitudeEstimate {
  float dt = 0.01f;
  float rate[3] = {0.0f, 0.0f, 0.0f};
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float eulerDeg[3] = {0.0f, 0.0f, 0.0f};
  bool isStill = false;
  bool eulerNearSingularity = false;
};

// Reads MPU6050 data and tracks a quaternion with a 6-state multiplicative EKF:
// three small-angle attitude errors plus three gyro-bias errors. Gyro drives
// prediction and normalized acceleration supplies the gravity correction.
// Euler roll/pitch are derived from the quaternion with the same principal
// ranges as the MuJoCo training code:
// roll [-180, 180] deg and pitch [-90, 90] deg.
class ImuEulerRuntime {
public:
  ImuEulerRuntime(Receive &imu,
                  const ImuEulerConfig &config = ImuEulerConfig());

  // Detects MPU6050, wakes it, and calculates gyro offset.
  bool begin();

  // Reads one IMU sample and updates Euler angles.
  bool update(unsigned long nowMs, AttitudeEstimate &estimate);

  // Returns whether MPU6050 was detected during begin().
  bool mpuDetected() const { return mpuDetected_; }

private:
  Receive &imu_;
  ImuEulerConfig config_;
  unsigned long lastUpdateMs_ = 0;
  bool mpuDetected_ = false;
  bool attitudeInitialized_ = false;
  float quaternion_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float gyroBiasRad_[3] = {0.0f, 0.0f, 0.0f};
  float covariance_[6][6] = {{0.0f}};

  // Checks whether an I2C device responds at the requested address.
  bool i2cDevicePresent(int address);

  // Computes loop dt in seconds and guards against invalid timing spikes.
  float computeDt(unsigned long nowMs);

  // Initializes roll/pitch from gravity, yaw=0, and EKF covariance.
  void initializeEkfFromAccel(const float acc[3]);

  // EKF prediction from gyro, including gyro-bias and covariance propagation.
  void predictQuaternionEkf(const float rateDps[3], float dt);

  // EKF gravity-direction measurement update from normalized acceleration.
  void correctQuaternionEkfFromAccel(const float accG[3]);

  // Initializes the 6x6 error covariance from configured standard deviations.
  void initializeEkfCovariance();

  // Applies a right-multiplicative small-angle correction to the quaternion.
  void applyAttitudeError(const float deltaTheta[3]);

  // Returns expected unit gravity direction in the sensor/body frame.
  void gravityDirectionBody(float gravity[3]) const;

  // Inverts a 3x3 innovation matrix. Returns false when nearly singular.
  bool invert3x3(const float input[3][3], float output[3][3]) const;

  // Keeps covariance symmetric and its diagonal positive and finite.
  void stabilizeCovariance();

  // Converts the tracked quaternion to MuJoCo-compatible principal Euler angles.
  void quaternionToEuler(float eulerDeg[3]) const;

  // Keeps the quaternion at unit length after every integration step.
  void normalizeQuaternion();

  // Marks when full ZYX Euler angles would be near gimbal-lock behavior.
  bool isEulerNearSingularity(float pitchDeg) const;

};

struct PolicyIo {
  float observation[NN_INPUT_DIM] = {0.0f, 0.0f, 0.0f, 0.0f};
  float action[NN_OUTPUT_DIM] = {0.0f, 0.0f};
};

// Packs the neural-network observation in the same order as the training env:
// [roll, pitch, servo1_target, servo2_target], all in radians.
void buildPolicyInput(float rollRad,
                      float pitchRad,
                      float servo1TargetRad,
                      float servo2TargetRad,
                      float observation[NN_INPUT_DIM]);

// Runs the neural network and optionally clamps action outputs to [-1, 1].
bool runPolicy(NeuralNetwork &policy,
               const float observation[NN_INPUT_DIM],
               float action[NN_OUTPUT_DIM],
               bool clampAction = true);

// Convenience wrapper: build the observation and run the policy in one call.
bool computePolicyIo(NeuralNetwork &policy,
                     float rollRad,
                     float pitchRad,
                     float servo1TargetRad,
                     float servo2TargetRad,
                     PolicyIo &io,
                     bool clampAction = true);

struct ServoControlConfig {
  uint8_t servo1Pin = 10;
  uint8_t servo2Pin = 20;
  int servo1OffsetUs = 0;
  int servo2OffsetUs = 0;
  int servo1PulseRangeUs = 1000;
  int servo2PulseRangeUs = 1000;
  int servo1MinDeg = 10;
  int servo1MaxDeg = 170;
  int servo2MinDeg = 0;
  int servo2MaxDeg = 180;
  int servoHz = 50;
  float targetDeltaRad = 0.08f;
  float servo1TargetLimitRad = 1.3962634f;
  float servo2TargetLimitRad = 1.55f;
  bool invertServo1 = true;
  bool invertServo2 = false;
};

// Minimal servo driver using the ESP32 Arduino core's built-in LEDC hardware
// PWM. No external ESP32Servo library is required.
class LedcServo {
public:
  // ESP32-C3 LEDC timers support at most 14-bit duty resolution.
  static constexpr uint8_t kResolutionBits = 14;

  bool begin(uint8_t pin,
             uint32_t frequencyHz,
             int minPulseUs,
             int maxPulseUs);
  bool writeDegrees(int angleDeg);
  bool attached() const { return attached_; }
  void detach();

private:
  uint8_t pin_ = 0;
  uint32_t frequencyHz_ = 0;
  int minPulseUs_ = 1000;
  int maxPulseUs_ = 2000;
  bool attached_ = false;

  uint32_t pulseUsToDuty(int pulseUs) const;
};

// Owns two servo target angles and converts policy action increments into
// LEDC PWM commands.
class RobotServoControl {
public:
  RobotServoControl();

  // Attaches both servo outputs using the ESP32 core's built-in LEDC driver.
  bool begin(LedcServo &servo1, LedcServo &servo2,
             const ServoControlConfig &config = ServoControlConfig());

  // Resets both target angles to zero radians and writes them to the servos.
  void resetTargets();

  // Sets target angles in radians, clamped to each servo's target limit.
  void setTargetsRad(float servo1TargetRad, float servo2TargetRad);

  // Applies NN action as target += action * targetDeltaRad.
  bool applyPolicyAction(const float action[NN_OUTPUT_DIM], bool writeNow = true);

  // Writes current target angles to physical servos.
  bool writeTargets();

  // Returns current servo1 target in radians.
  float servo1TargetRad() const { return servo1TargetRad_; }

  // Returns current servo2 target in radians.
  float servo2TargetRad() const { return servo2TargetRad_; }

private:
  LedcServo *servo1_ = nullptr;
  LedcServo *servo2_ = nullptr;
  ServoControlConfig config_;
  float servo1TargetRad_ = 0.0f;
  float servo2TargetRad_ = 0.0f;

  // Converts a target angle in radians to a limited 0..180 degree command.
  int targetRadToServoDegrees(float targetRad,
                              bool invert,
                              int minDeg,
                              int maxDeg) const;
};

#endif
