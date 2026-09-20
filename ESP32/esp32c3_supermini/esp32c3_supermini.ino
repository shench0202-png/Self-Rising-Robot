#include "RobotRuntime.h"
#include "policy_network.h"

#include <math.h>

/*
  ESP32-C3 SuperMini + GY-521/MPU6050 + robo1_getup_ppo_beckup3 runtime.
  The MPU6050 board and servo supply must share ground with the ESP32-C3.

  Serial commands at 115200 baud:
    G - start one 14-second get-up attempt
    S - stop policy and return both targets to zero
    Z - return both targets to zero without starting policy
    H - print command help

  The exported policy input is:
    [roll_rad, pitch_rad, servo1_target_rad, servo2_target_rad]

  Euler representation intentionally matches MuJoCo:
    roll  = [-180, 180] deg
    pitch = [ -90,  90] deg
  A physical pitch beyond 90 deg folds back below 90 deg while roll/yaw shift
  by about 180 deg. The internal quaternion remains continuous.
*/

Receive imu;
ImuEulerRuntime imuEuler(imu);
AttitudeEstimate attitude;

// The requested startup sweep enables physical PWM output. Keep the mechanism
// unloaded during the first power-on check of the 10..170 degree range.
const bool ENABLE_POLICY_CONTROL = true;
const bool ENABLE_SERVO_OUTPUT = true;

const unsigned long POLICY_PERIOD_MS = 20;  // 50 Hz, matches MuJoCo.
const unsigned long GETUP_TIMEOUT_MS = 14000;
const unsigned long UPRIGHT_HOLD_MS = 700;
const unsigned long TELEMETRY_PERIOD_MS = 100;
const float UPRIGHT_TILT_RAD = 0.25f;
// Automatically start PPO after the IMU reports a sustained fallen pose.
// A lower re-arm angle provides hysteresis and prevents repeated triggering.
const bool ENABLE_IMU_AUTO_START = true;
const float AUTO_START_TILT_RAD = 0.5235988f;  // 30 deg.
const float AUTO_REARM_TILT_RAD = 0.3490659f;  // 20 deg.
const unsigned long AUTO_START_HOLD_MS = 250;
const float TARGET_DELTA_RAD = 0.08f;
const float SERVO1_TARGET_LIMIT_RAD = 1.3962634f;  // 80 deg -> servo 10..170 deg.
const float SERVO2_TARGET_LIMIT_RAD = 1.55f;
const float DEG_TO_RAD_LOCAL = 0.017453292519943295f;
const float RAD_TO_DEG_LOCAL = 57.29577951308232f;

// Change only after observing the four known poses over Serial. With the Euler
// guard enabled, the expected policy values are roll_pos=(+90,0),
// roll_neg=(-90,0), pitch_pos=(0,+85), and pitch_neg=(0,-85), in degrees.
const bool POLICY_SWAP_ROLL_PITCH = false;
const float POLICY_ROLL_SIGN = 1.0f;
const float POLICY_PITCH_SIGN = 1.0f;
const bool ENABLE_POLICY_EULER_GUARD = true;
const float POLICY_PITCH_LOCK_DEG = 85.0f;
const float POLICY_PITCH_RELEASE_DEG = 80.0f;
const float POLICY_ROLL_REJOIN_MAX_DELTA_DEG = 90.0f;

// ESP32-C3 SuperMini: IO6=SDA, IO7=SCL, IO10/IO20=servo PWM.
// GPIO8 drives the active-low onboard LED; GPIO18/19 carry native USB.
const uint8_t SDA_PIN = 6;
const uint8_t SCL_PIN = 7;
const uint8_t SERVO1_PIN = 10;
const uint8_t SERVO2_PIN = 20;
const uint8_t STATUS_LED_PIN = 8;
const unsigned long STATUS_LED_TOGGLE_MS = 500;  // 1 Hz: one full blink per second.
const unsigned long STARTUP_SERVO_STEP_MS = 20;  // 1 degree per 50 Hz PWM frame.
const unsigned long STARTUP_SERVO_DWELL_MS = 300;
// This servo needs about 500-2500 us for its full 0-180 degree travel. The
// final angle limits below still keep servo 1 inside 10-170 degrees.
const int SERVO1_OFFSET_US = 0;
const int SERVO2_OFFSET_US = 0;
const int SERVO1_PULSE_RANGE_US = 1000;
const int SERVO2_PULSE_RANGE_US = 1000;
const int SERVO1_MIN_DEG = 10;
const int SERVO1_MAX_DEG = 170;
const int SERVO2_MIN_DEG = 0;
const int SERVO2_MAX_DEG = 180;
const bool SERVO1_INVERT = true;
const bool SERVO2_INVERT = false;

LedcServo servo1;
LedcServo servo2;
RobotServoControl servoControl;

bool statusLedOn = false;
unsigned long lastStatusLedToggleMs = 0;
unsigned long statusLedToggleIntervalMs = STATUS_LED_TOGGLE_MS;
bool policyRunning = false;
float policyObservation[OBS_DIM] = {0.0f, 0.0f, 0.0f, 0.0f};
float policyAction[ACTION_DIM] = {0.0f, 0.0f};
float policyTargetRad[ACTION_DIM] = {0.0f, 0.0f};
unsigned long lastPolicyMs = 0;
unsigned long policyStartMs = 0;
unsigned long uprightStartMs = 0;
unsigned long autoStartCandidateMs = 0;
unsigned long lastPrintMs = 0;
bool autoStartArmed = true;
bool policyPitchLocked = false;
float lockedPolicyRollDeg = 0.0f;
float lockedPolicyPitchDeg = 0.0f;
float mappedPolicyRollDeg = 0.0f;
float mappedPolicyPitchDeg = 0.0f;

void updateStatusLed() {
  const unsigned long nowMs = millis();
  if (nowMs - lastStatusLedToggleMs >= statusLedToggleIntervalMs) {
    lastStatusLedToggleMs = nowMs;
    statusLedOn = !statusLedOn;
    digitalWrite(STATUS_LED_PIN, statusLedOn ? LOW : HIGH);
  }
}

void waitWithStatusLed(unsigned long durationMs) {
  const unsigned long startMs = millis();
  while (millis() - startMs < durationMs) {
    updateStatusLed();
    delay(5);
  }
}

void haltWithStatusLed() {
  statusLedToggleIntervalMs = 100;  // Rapid blinking indicates initialization failure.
  while (true) waitWithStatusLed(100);
}

void runStartupServoSweep(LedcServo &servo, uint8_t servoNumber) {
  const uint8_t angles[] = {90, 10, 90, 170, 90};
  int currentAngle = 90;
  Serial.print("SERVO_STARTUP_TEST\t");
  Serial.println(servoNumber);

  for (uint8_t targetAngle : angles) {
    // Move one degree per PWM frame instead of jumping directly to the target.
    // This gives about 50 deg/s and reduces startup shock to the mechanism.
    do {
      if (currentAngle < targetAngle) {
        ++currentAngle;
      } else if (currentAngle > targetAngle) {
        --currentAngle;
      }

      if (!servo.writeDegrees(currentAngle)) {
        Serial.println("Servo PWM write failed.");
        haltWithStatusLed();
      }
      waitWithStatusLed(STARTUP_SERVO_STEP_MS);
    } while (currentAngle != targetAngle);

    waitWithStatusLed(STARTUP_SERVO_DWELL_MS);
  }
}

float clampLocal(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

float wrapDegrees180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

// Returns the yaw-independent angle between the robot body Z axis and world Z.
// Unlike principal Euler pitch, this display/stop metric covers 0..180 deg.
float bodyTiltRad(const AttitudeEstimate &data) {
  const float qx = data.quaternion[1];
  const float qy = data.quaternion[2];
  const float bodyZWorldZ = clampLocal(
      1.0f - 2.0f * (qx * qx + qy * qy), -1.0f, 1.0f);
  return acosf(bodyZWorldZ);
}

// Updates a policy-only Euler guard. The quaternion and raw Euler telemetry are
// left untouched. At +/-85 deg pitch, roll is frozen before the ZYX Euler
// representation can jump by about 180 deg. The lock stays active across the
// folded branch and releases only after pitch is below 80 deg and raw roll has
// rejoined the stored branch.
void updateMappedPolicyAngles() {
  float rollDeg = attitude.eulerDeg[0];
  float pitchDeg = attitude.eulerDeg[1];
  if (POLICY_SWAP_ROLL_PITCH) {
    const float temporary = rollDeg;
    rollDeg = pitchDeg;
    pitchDeg = temporary;
  }
  rollDeg = POLICY_ROLL_SIGN * rollDeg;
  pitchDeg = POLICY_PITCH_SIGN * pitchDeg;

  if (!ENABLE_POLICY_EULER_GUARD) {
    policyPitchLocked = false;
    mappedPolicyRollDeg = rollDeg;
    mappedPolicyPitchDeg = pitchDeg;
    return;
  }

  if (!policyPitchLocked && fabsf(pitchDeg) >= POLICY_PITCH_LOCK_DEG) {
    policyPitchLocked = true;
    lockedPolicyRollDeg = rollDeg;
    lockedPolicyPitchDeg = pitchDeg >= 0.0f
                               ? POLICY_PITCH_LOCK_DEG
                               : -POLICY_PITCH_LOCK_DEG;
  }

  if (policyPitchLocked) {
    const float rollDeltaDeg = fabsf(
        wrapDegrees180(rollDeg - lockedPolicyRollDeg));
    const bool pitchInsideRelease =
        fabsf(pitchDeg) <= POLICY_PITCH_RELEASE_DEG;
    const bool rollRejoined =
        rollDeltaDeg < POLICY_ROLL_REJOIN_MAX_DELTA_DEG;
    if (pitchInsideRelease && rollRejoined) {
      policyPitchLocked = false;
    }
  }

  if (policyPitchLocked) {
    mappedPolicyRollDeg = lockedPolicyRollDeg;
    mappedPolicyPitchDeg = lockedPolicyPitchDeg;
  } else {
    mappedPolicyRollDeg = rollDeg;
    mappedPolicyPitchDeg = pitchDeg;
  }
}

void mappedPolicyAngles(float &rollRad, float &pitchRad) {
  rollRad = mappedPolicyRollDeg * DEG_TO_RAD_LOCAL;
  pitchRad = mappedPolicyPitchDeg * DEG_TO_RAD_LOCAL;
}

void writePolicyTargets() {
  if (ENABLE_SERVO_OUTPUT) {
    servoControl.setTargetsRad(policyTargetRad[0], policyTargetRad[1]);
    servoControl.writeTargets();
  }
}

void centerPolicyTargets() {
  policyTargetRad[0] = 0.0f;
  policyTargetRad[1] = 0.0f;
  policyAction[0] = 0.0f;
  policyAction[1] = 0.0f;
  writePolicyTargets();
}

void disarmAutoStart() {
  autoStartArmed = false;
  autoStartCandidateMs = 0;
}

void startPolicy(unsigned long nowMs) {
  if (!ENABLE_POLICY_CONTROL) {
    Serial.println("POLICY_DISABLED");
    return;
  }
  disarmAutoStart();
  centerPolicyTargets();
  policyRunning = true;
  lastPolicyMs = 0;
  // Use the loop's timestamp throughout this control iteration. Calling
  // millis() again here can produce a start time newer than runPolicyAt50Hz's
  // nowMs; unsigned subtraction would then wrap and trigger TIMEOUT instantly.
  policyStartMs = nowMs;
  uprightStartMs = 0;
  Serial.println("POLICY_START");
}

void stopPolicy(const char *reason) {
  disarmAutoStart();
  policyRunning = false;
  centerPolicyTargets();
  Serial.print("POLICY_STOP\t");
  Serial.println(reason);
}

// Starts PPO when roll/pitch tilt exceeds 30 degrees for 250 ms. After one
// attempt, the trigger remains disarmed until the mechanism returns below 20
// degrees, avoiding immediate restart after a timeout or a manual stop.
void updateImuAutoStart(unsigned long nowMs) {
  if (!ENABLE_IMU_AUTO_START || !ENABLE_POLICY_CONTROL || policyRunning) {
    autoStartCandidateMs = 0;
    return;
  }

  const float tiltRad = bodyTiltRad(attitude);
  if (!autoStartArmed) {
    if (tiltRad <= AUTO_REARM_TILT_RAD) {
      autoStartArmed = true;
      Serial.println("AUTO_TRIGGER_ARMED");
    }
    return;
  }

  if (tiltRad < AUTO_START_TILT_RAD) {
    autoStartCandidateMs = 0;
    return;
  }

  if (autoStartCandidateMs == 0) {
    autoStartCandidateMs = nowMs;
    Serial.print("AUTO_TRIGGER_PENDING\t");
    Serial.println(tiltRad * RAD_TO_DEG_LOCAL, 2);
    return;
  }

  if (nowMs - autoStartCandidateMs >= AUTO_START_HOLD_MS) {
    Serial.print("AUTO_TRIGGER\t");
    Serial.println(tiltRad * RAD_TO_DEG_LOCAL, 2);
    startPolicy(nowMs);
  }
}

void printCommandHelp() {
  Serial.println("COMMANDS: G=start, S=stop+center, Z=center, H=help");
  Serial.println("Policy: robo1_getup_ppo_beckup3.zip, 50 Hz");
  Serial.println(ENABLE_IMU_AUTO_START
                     ? "Auto-start: tilt >= 30 deg for 250 ms; re-arm <= 20 deg"
                     : "Auto-start: DISABLED");
  Serial.println(ENABLE_SERVO_OUTPUT
                     ? "Servo output: ENABLED"
                     : "Servo output: DISABLED (inference/telemetry only)");
}

void processSerialCommands(unsigned long nowMs) {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    if (command == 'G' || command == 'g') {
      startPolicy(nowMs);
    } else if (command == 'S' || command == 's') {
      stopPolicy("USER");
    } else if (command == 'Z' || command == 'z') {
      disarmAutoStart();
      centerPolicyTargets();
      Serial.println("TARGETS_CENTERED");
    } else if (command == 'H' || command == 'h' || command == '?') {
      printCommandHelp();
    }
  }
}

void runPolicyAt50Hz(unsigned long nowMs) {
  if (!policyRunning || !ENABLE_POLICY_CONTROL) {
    return;
  }
  if (lastPolicyMs != 0 && nowMs - lastPolicyMs < POLICY_PERIOD_MS) {
    return;
  }
  lastPolicyMs = nowMs;

  float rollRad = 0.0f;
  float pitchRad = 0.0f;
  mappedPolicyAngles(rollRad, pitchRad);
  policyObservation[0] = rollRad;
  policyObservation[1] = pitchRad;
  policyObservation[2] = policyTargetRad[0];
  policyObservation[3] = policyTargetRad[1];

  forward_policy(policyObservation, policyAction);
  policyAction[0] = clampLocal(policyAction[0], -1.0f, 1.0f);
  policyAction[1] = clampLocal(policyAction[1], -1.0f, 1.0f);
  policyTargetRad[0] = clampLocal(
      policyTargetRad[0] + policyAction[0] * TARGET_DELTA_RAD,
      -SERVO1_TARGET_LIMIT_RAD, SERVO1_TARGET_LIMIT_RAD);
  policyTargetRad[1] = clampLocal(
      policyTargetRad[1] + policyAction[1] * TARGET_DELTA_RAD,
      -SERVO2_TARGET_LIMIT_RAD, SERVO2_TARGET_LIMIT_RAD);

  // The policy-only Euler guard freezes pitch at +/-85 deg and prevents the
  // artificial roll jump. It does not stop inference or alter the quaternion.
  writePolicyTargets();

  const bool upright = bodyTiltRad(attitude) < UPRIGHT_TILT_RAD;
  if (upright) {
    if (uprightStartMs == 0) uprightStartMs = nowMs;
    if (nowMs - uprightStartMs >= UPRIGHT_HOLD_MS) {
      stopPolicy("UPRIGHT");
      return;
    }
  } else {
    uprightStartMs = 0;
  }

  if (nowMs - policyStartMs >= GETUP_TIMEOUT_MS) {
    stopPolicy("TIMEOUT");
  }
}

void printTelemetry(const AttitudeEstimate &data) {
  float policyRollRad = 0.0f;
  float policyPitchRad = 0.0f;
  mappedPolicyAngles(policyRollRad, policyPitchRad);

  Serial.print("P\t");
  Serial.print(policyRunning ? 1 : 0);
  Serial.print("\t");
  Serial.print(data.eulerDeg[0], 2);
  Serial.print("\t");
  Serial.print(data.eulerDeg[1], 2);
  Serial.print("\t");
  Serial.print(data.eulerDeg[2], 2);
  Serial.print("\t");
  Serial.print(bodyTiltRad(data) * RAD_TO_DEG_LOCAL, 2);
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print("\t");
    Serial.print(data.quaternion[i], 5);
  }
  Serial.print("\t");
  Serial.print(policyRollRad, 5);
  Serial.print("\t");
  Serial.print(policyPitchRad, 5);
  Serial.print("\t");
  Serial.print(policyTargetRad[0], 5);
  Serial.print("\t");
  Serial.print(policyTargetRad[1], 5);
  Serial.print("\t");
  Serial.print(policyAction[0], 5);
  Serial.print("\t");
  Serial.print(policyAction[1], 5);
  Serial.print("\t");
  Serial.println((data.eulerNearSingularity || policyPitchLocked) ? 1 : 0);
}

void setup() {
  // GPIO8 is sampled at boot, so configure it only after the sketch starts.
  digitalWrite(STATUS_LED_PIN, HIGH);
  pinMode(STATUS_LED_PIN, OUTPUT);
  lastStatusLedToggleMs = millis();

  Serial.begin(115200);
  const unsigned long serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 1500) {
    waitWithStatusLed(10);
  }

  if (ENABLE_SERVO_OUTPUT) {
    ServoControlConfig servoConfig;
    servoConfig.servo1Pin = SERVO1_PIN;
    servoConfig.servo2Pin = SERVO2_PIN;
    servoConfig.servo1OffsetUs = SERVO1_OFFSET_US;
    servoConfig.servo2OffsetUs = SERVO2_OFFSET_US;
    servoConfig.servo1PulseRangeUs = SERVO1_PULSE_RANGE_US;
    servoConfig.servo2PulseRangeUs = SERVO2_PULSE_RANGE_US;
    servoConfig.servo1MinDeg = SERVO1_MIN_DEG;
    servoConfig.servo1MaxDeg = SERVO1_MAX_DEG;
    servoConfig.servo2MinDeg = SERVO2_MIN_DEG;
    servoConfig.servo2MaxDeg = SERVO2_MAX_DEG;
    servoConfig.servo1TargetLimitRad = SERVO1_TARGET_LIMIT_RAD;
    servoConfig.servo2TargetLimitRad = SERVO2_TARGET_LIMIT_RAD;
    servoConfig.invertServo1 = SERVO1_INVERT;
    servoConfig.invertServo2 = SERVO2_INVERT;
    if (!servoControl.begin(servo1, servo2, servoConfig) ||
        !servo1.attached() || !servo2.attached()) {
      Serial.println("Servo initialization failed.");
      haltWithStatusLed();
    }
    runStartupServoSweep(servo1, 1);
    runStartupServoSweep(servo2, 2);
    servoControl.resetTargets();
    Serial.println("SERVO_STARTUP_TEST_DONE");
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  waitWithStatusLed(200);

  if (!imuEuler.begin()) {
    Serial.println("Runtime stopped: check MPU6050 I2C wiring and power.");
    haltWithStatusLed();
  }

  Serial.println("ESP32-C3 quaternion get-up runtime started.");
  printCommandHelp();
}

void loop() {
  const unsigned long nowMs = millis();
  updateStatusLed();
  imuEuler.update(nowMs, attitude);
  updateMappedPolicyAngles();
  processSerialCommands(nowMs);
  updateImuAutoStart(nowMs);
  runPolicyAt50Hz(nowMs);

  if (nowMs - lastPrintMs >= TELEMETRY_PERIOD_MS) {
    lastPrintMs = nowMs;
    printTelemetry(attitude);
  }

  delay(5);
}
