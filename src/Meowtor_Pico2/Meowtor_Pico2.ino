
/*
  Meowtor 2026 - Raspberry Pi Pico 2 competition controller

  Board package: Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower.
  Board: Raspberry Pi Pico 2.
  Libraries: Adafruit VL53L0X, Adafruit VL53L1X, Adafruit MPU6050,
             Adafruit Unified Sensor, Servo.

  Hardware assignment (from the team's wiring sheet):
    GP0  UART0 TX -> ESP32-CAM U0R/GPIO3
    GP1  UART0 RX <- ESP32-CAM U0T/GPIO1
    GP4/GP5 I2C SDA/SCL
    GP6/7/8 XSHUT front/left/right
    GP13 DRV8833 nFAULT (optional)
    GP16/17 DRV8833 AIN1/AIN2
    GP18 steering servo signal
    GP20 start pushbutton to GND (INPUT_PULLUP)

  WRO 2026 behaviour:
    one start button; autonomous; 3-minute safety timeout; 3 laps;
    red passed on right; green passed on left; parallel parking; motor stop.

  ANTi-inspired techniques adapted to our no-encoder platform:
    non-blocking state machine, gyro heading turns, PID wall following,
    speed ramping, vision target offset, short detection memory, staged park,
    UART timeout handling and sensor fallbacks. This is independent C++ code.

  IMPORTANT: Every value in the CALIBRATION block must be tested on the real
  vehicle. No untested program can know the exact steering centre, motor speed,
  turn time, camera geometry or parking timing of a physical robot.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

enum ChallengeMode : uint8_t { OPEN_CHALLENGE, OBSTACLE_CHALLENGE };
enum DriveDirection : int8_t { DIR_UNKNOWN = 0, DIR_CW = 1, DIR_CCW = -1 };
enum RunState : uint8_t {
  WAIT_START,
  EXIT_PARKING,
  DETERMINE_DIRECTION,
  CRUISE,
  CORNER_TURN,
  AVOID_TRACK,
  AVOID_RECOVER,
  OPEN_FINISH,
  SEEK_PARKING,
  PARK_FORWARD,
  PARK_REVERSE_TURN,
  PARK_REVERSE_STRAIGHT,
  PARK_FINAL_ADJUST,
  FINISHED,
  FAULT_STOP
};
enum SignColour : uint8_t { SIGN_NONE, SIGN_RED, SIGN_GREEN };

// ========================= CALIBRATION ===============================
constexpr ChallengeMode CHALLENGE_MODE = OBSTACLE_CHALLENGE;
constexpr bool START_FROM_PARKING = false;
constexpr int8_t FORCED_DIRECTION = 0;  // 0 auto, +1 CW, -1 CCW
constexpr bool MOTOR_REVERSED = false;  // true if positive PWM drives backward

// Steering values confirmed by the team's bench test.
constexpr int STEER_CENTRE = 90;
constexpr int STEER_FULL_LEFT = 135;
constexpr int STEER_FULL_RIGHT = 45;
constexpr int STEER_SAFE_LEFT = 126;
constexpr int STEER_SAFE_RIGHT = 54;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

// Motor PWM. Start conservatively; increase only after reliable runs.
constexpr int PWM_CRUISE = 165;
constexpr int PWM_VISION_LOST = 115;
constexpr int PWM_CORNER = 120;
constexpr int PWM_AVOID = 112;
constexpr int PWM_PARK = 82;
constexpr int PWM_REVERSE = 76;
constexpr int PWM_RAMP_STEP = 3;

// Distance geometry (millimetres).
constexpr int INNER_WALL_TARGET_MM = 330;
constexpr int CORNER_FRONT_MM = 430;
constexpr int EMERGENCY_FRONT_MM = 85;
constexpr int OBSTACLE_FRONT_MM = 520;
constexpr int SENSOR_MAX_MM = 3000;

// ANTi-style smooth target offsets. The camera is 160 pixels wide.
constexpr int RED_TARGET_X = 43;     // red remains left => robot passes right
constexpr int GREEN_TARGET_X = 117; // green remains right => robot passes left
constexpr int MIN_SIGN_HEIGHT = 10;
constexpr int MIN_SIGN_AREA = 10;
constexpr uint32_t SIGN_MEMORY_MS = 420;

// Controller gains. Output is steering degrees.
constexpr float WALL_KP = 0.075f;
constexpr float WALL_KD = 0.040f;
constexpr float HEADING_KP = 0.48f;
constexpr float VISION_KP = 0.42f;
constexpr float VISION_KD = 0.18f;
constexpr float GYRO_SIGN = 1.0f; // set -1 if measured turn direction is reversed
constexpr float CORNER_ANGLE_DEG = 90.0f;
constexpr float TURN_TOLERANCE_DEG = 5.0f;

// Timing constants; tune using video and measured track tests.
constexpr uint32_t RUN_TIMEOUT_MS = 178000;
constexpr uint32_t SENSOR_PERIOD_MS = 35;
constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t DIRECTION_SAMPLE_MS = 650;
constexpr uint32_t CORNER_COOLDOWN_MS = 850;
constexpr uint32_t CORNER_MIN_MS = 320;
constexpr uint32_t AVOID_MIN_MS = 300;
constexpr uint32_t AVOID_RECOVER_MS = 360;
constexpr uint32_t OPEN_FINISH_DRIVE_MS = 900;
constexpr uint32_t EXIT_PARK_MS = 850;
constexpr uint32_t PARK_FORWARD_MS = 520;
constexpr uint32_t PARK_REVERSE_TURN_MS = 760;
constexpr uint32_t PARK_REVERSE_STRAIGHT_MS = 620;
constexpr uint32_t PARK_FINAL_ADJUST_MS = 300;

// ========================= PINS / ADDRESSES ==========================
constexpr uint8_t UART_TX_PIN = 0, UART_RX_PIN = 1;
constexpr uint8_t SDA_PIN = 4, SCL_PIN = 5;
constexpr uint8_t XSHUT_FRONT = 6, XSHUT_LEFT = 7, XSHUT_RIGHT = 8;
constexpr uint8_t DRIVER_FAULT_PIN = 13;
constexpr uint8_t MOTOR_AIN1 = 16, MOTOR_AIN2 = 17;
constexpr uint8_t SERVO_PIN = 18, START_BUTTON_PIN = 20;
constexpr uint8_t FRONT_ADDR = 0x30, LEFT_ADDR = 0x31, RIGHT_ADDR = 0x32;

Servo steering;
Adafruit_VL53L1X frontToF(XSHUT_FRONT);
Adafruit_VL53L0X leftToF;
Adafruit_VL53L0X rightToF;
Adafruit_MPU6050 imu;

bool frontReady = false, leftReady = false, rightReady = false, imuReady = false;
int frontMm = -1, leftMm = -1, rightMm = -1;
uint32_t frontFreshAt = 0, leftFreshAt = 0, rightFreshAt = 0;

struct VisionData {
  uint32_t sequence = 0;
  int redX = -1, redH = -1, redArea = 0;
  int greenX = -1, greenH = -1, greenArea = 0;
  int magentaCount = 0, magentaX1 = -1, magentaX2 = -1, magentaH = -1;
  uint32_t receivedAt = 0;
  bool readySeen = false;
} vision;

RunState state = WAIT_START;
DriveDirection direction = DIR_UNKNOWN;
SignColour trackedSign = SIGN_NONE;
uint32_t stateStartedAt = 0, runStartedAt = 0, lastCornerAt = 0;
uint32_t lastSensorAt = 0, lastControlAt = 0, lastStatusAt = 0;
uint32_t lastSignAt = 0;
int cornerCount = 0, lapCount = 0;
int currentPwm = 0, commandedSteer = STEER_CENTRE;
float yawDeg = 0.0f, targetYawDeg = 0.0f, gyroBiasDps = 0.0f;
uint32_t lastImuUs = 0;
float lastWallError = 0.0f, lastVisionError = 0.0f;
int directionLeftSum = 0, directionRightSum = 0, directionSamples = 0;
String uartLine;

const char *stateName(RunState s) {
  switch (s) {
    case WAIT_START: return "WAIT";
    case EXIT_PARKING: return "EXIT_PARK";
    case DETERMINE_DIRECTION: return "DIRECTION";
    case CRUISE: return "CRUISE";
    case CORNER_TURN: return "CORNER";
    case AVOID_TRACK: return "AVOID";
    case AVOID_RECOVER: return "RECOVER";
    case OPEN_FINISH: return "OPEN_FINISH";
    case SEEK_PARKING: return "SEEK_PARK";
    case PARK_FORWARD: return "PARK_FORWARD";
    case PARK_REVERSE_TURN: return "PARK_REVERSE_TURN";
    case PARK_REVERSE_STRAIGHT: return "PARK_STRAIGHTEN";
    case PARK_FINAL_ADJUST: return "PARK_ADJUST";
    case FINISHED: return "FINISHED";
    default: return "FAULT";
  }
}

void enterState(RunState next) {
  state = next;
  stateStartedAt = millis();
  lastWallError = 0;
  lastVisionError = 0;
}

bool validDistance(int value) {
  return value > 0 && value <= SENSOR_MAX_MM;
}

float wrapAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void setSteering(int degrees) {
  commandedSteer = constrain(degrees, STEER_FULL_RIGHT, STEER_FULL_LEFT);
  steering.write(commandedSteer);
}

void setMotorRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  const int output = MOTOR_REVERSED ? -pwm : pwm;
  if (output > 0) {
    analogWrite(MOTOR_AIN1, output);
    analogWrite(MOTOR_AIN2, 0);
  } else if (output < 0) {
    analogWrite(MOTOR_AIN1, 0);
    analogWrite(MOTOR_AIN2, -output);
  } else {
    analogWrite(MOTOR_AIN1, 0);
    analogWrite(MOTOR_AIN2, 0);
  }
  currentPwm = pwm;
}

void rampMotorToward(int target) {
  target = constrain(target, -255, 255);
  int next = currentPwm;
  if (next < target) next = min(target, next + PWM_RAMP_STEP);
  if (next > target) next = max(target, next - PWM_RAMP_STEP);
  setMotorRaw(next);
}

void stopRobot() {
  setMotorRaw(0);
  setSteering(STEER_CENTRE);
}

bool initializeSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(20);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  // Each ToF boots at 0x29. Bring up and re-address one at a time.
  frontReady = frontToF.begin(FRONT_ADDR, &Wire);
  if (frontReady) {
    frontToF.setTimingBudget(50);
    frontReady = frontToF.startRanging();
  }

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(12);
  leftReady = leftToF.begin(0x29, false, &Wire) && leftToF.setAddress(LEFT_ADDR);
  if (leftReady) leftReady = leftToF.startRangeContinuous(50);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(12);
  rightReady = rightToF.begin(0x29, false, &Wire) && rightToF.setAddress(RIGHT_ADDR);
  if (rightReady) rightReady = rightToF.startRangeContinuous(50);

  imuReady = imu.begin(0x68, &Wire);
  if (imuReady) {
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  return frontReady && (leftReady || rightReady) && imuReady;
}

void readDistances() {
  const uint32_t now = millis();
  if (frontReady && frontToF.dataReady()) {
    const int d = frontToF.distance();
    frontToF.clearInterrupt();
    if (validDistance(d)) { frontMm = d; frontFreshAt = now; }
  }
  if (leftReady && leftToF.isRangeComplete()) {
    const int d = leftToF.readRangeResult();
    if (leftToF.readRangeStatus() == 0 && validDistance(d)) {
      leftMm = d; leftFreshAt = now;
    }
  }
  if (rightReady && rightToF.isRangeComplete()) {
    const int d = rightToF.readRangeResult();
    if (rightToF.readRangeStatus() == 0 && validDistance(d)) {
      rightMm = d; rightFreshAt = now;
    }
  }
  if (now - frontFreshAt > 500) frontMm = -1;
  if (now - leftFreshAt > 500) leftMm = -1;
  if (now - rightFreshAt > 500) rightMm = -1;
}

float readGyroZ() {
  if (!imuReady) return 0.0f;
  sensors_event_t a = {}, g = {}, t = {};
  if (!imu.getEvent(&a, &g, &t)) return 0.0f;
  return g.gyro.z * 57.2957795f * GYRO_SIGN;
}

void updateYaw() {
  const uint32_t nowUs = micros();
  if (!lastImuUs) { lastImuUs = nowUs; return; }
  const float dt = min(0.1f, (nowUs - lastImuUs) / 1000000.0f);
  lastImuUs = nowUs;
  yawDeg += (readGyroZ() - gyroBiasDps) * dt;
}

void calibrateGyroAutomatically() {
  if (!imuReady) return;
  double sum = 0.0;
  const int samples = 240;
  for (int i = 0; i < samples; ++i) {
    sum += readGyroZ();
    delay(5);
  }
  gyroBiasDps = float(sum / samples);
  yawDeg = 0;
  targetYawDeg = 0;
  lastImuUs = micros();
}

int splitCsv(char *line, char *parts[], int maximum) {
  int count = 0;
  char *token = strtok(line, ",");
  while (token && count < maximum) {
    parts[count++] = token;
    token = strtok(nullptr, ",");
  }
  return count;
}

void parseVisionLine(String line) {
  line.trim();
  if (line.startsWith("READY,VISION")) {
    vision.readySeen = true;
    vision.receivedAt = millis();
    return;
  }
  if (!line.startsWith("V,")) return;

  char buffer[150];
  line.toCharArray(buffer, sizeof(buffer));
  char *p[14] = {};
  if (splitCsv(buffer, p, 14) != 12) return;
  vision.sequence = strtoul(p[1], nullptr, 10);
  vision.redX = atoi(p[2]); vision.redH = atoi(p[3]); vision.redArea = atoi(p[4]);
  vision.greenX = atoi(p[5]); vision.greenH = atoi(p[6]); vision.greenArea = atoi(p[7]);
  vision.magentaCount = atoi(p[8]); vision.magentaX1 = atoi(p[9]);
  vision.magentaX2 = atoi(p[10]); vision.magentaH = atoi(p[11]);
  vision.receivedAt = millis();
}

void serviceVisionUart() {
  while (Serial1.available()) {
    const char c = char(Serial1.read());
    if (c == '\n') {
      parseVisionLine(uartLine);
      uartLine = "";
    } else if (c != '\r' && uartLine.length() < 145) {
      uartLine += c;
    } else if (uartLine.length() >= 145) {
      uartLine = "";
    }
  }
}

bool visionFresh(uint32_t age = 500) {
  return millis() - vision.receivedAt <= age;
}

SignColour visibleSign() {
  if (!visionFresh()) return SIGN_NONE;
  const bool red = vision.redX >= 0 && vision.redH >= MIN_SIGN_HEIGHT &&
                   vision.redArea >= MIN_SIGN_AREA;
  const bool green = vision.greenX >= 0 && vision.greenH >= MIN_SIGN_HEIGHT &&
                     vision.greenArea >= MIN_SIGN_AREA;
  if (red && green) return vision.redArea >= vision.greenArea ? SIGN_RED : SIGN_GREEN;
  if (red) return SIGN_RED;
  if (green) return SIGN_GREEN;
  return SIGN_NONE;
}

int signX(SignColour colour) {
  return colour == SIGN_RED ? vision.redX : vision.greenX;
}

int signHeight(SignColour colour) {
  return colour == SIGN_RED ? vision.redH : vision.greenH;
}

int wallFollowSteering() {
  float wallError = 0.0f;
  if (direction == DIR_CW && validDistance(rightMm)) {
    wallError = INNER_WALL_TARGET_MM - rightMm;
  } else if (direction == DIR_CCW && validDistance(leftMm)) {
    wallError = leftMm - INNER_WALL_TARGET_MM;
  } else if (validDistance(leftMm) && validDistance(rightMm)) {
    wallError = (leftMm - rightMm) * 0.25f;
  }
  const float derivative = wallError - lastWallError;
  lastWallError = wallError;
  const float headingError = wrapAngle(targetYawDeg - yawDeg);
  const float correction = WALL_KP * wallError + WALL_KD * derivative +
                           HEADING_KP * headingError;
  return constrain(int(roundf(STEER_CENTRE + correction)),
                   STEER_SAFE_RIGHT, STEER_SAFE_LEFT);
}

bool closeObstacle(SignColour colour) {
  if (colour == SIGN_NONE) return false;
  const bool imageClose = signHeight(colour) >= 22;
  const bool rangeClose = validDistance(frontMm) && frontMm <= OBSTACLE_FRONT_MM;
  return imageClose || rangeClose;
}

int obstacleTrackingSteering(SignColour colour) {
  const int targetX = colour == SIGN_RED ? RED_TARGET_X : GREEN_TARGET_X;
  const float error = targetX - signX(colour);
  const float derivative = error - lastVisionError;
  lastVisionError = error;
  int command = int(roundf(STEER_CENTRE + VISION_KP * error + VISION_KD * derivative));
  return constrain(command, STEER_FULL_RIGHT, STEER_FULL_LEFT);
}

void beginCorner() {
  const float delta = direction == DIR_CW ? -CORNER_ANGLE_DEG : CORNER_ANGLE_DEG;
  targetYawDeg += delta;
  enterState(CORNER_TURN);
  lastCornerAt = millis();
}

void finishCornerIfReady() {
  const float headingError = wrapAngle(targetYawDeg - yawDeg);
  const bool minimumTime = millis() - stateStartedAt >= CORNER_MIN_MS;
  if (minimumTime && fabsf(headingError) <= TURN_TOLERANCE_DEG) {
    cornerCount++;
    lapCount = cornerCount / 4;
    lastCornerAt = millis();
    if (cornerCount >= 12) {
      enterState(CHALLENGE_MODE == OPEN_CHALLENGE ? OPEN_FINISH : SEEK_PARKING);
    } else {
      enterState(CRUISE);
    }
  }
}

void chooseDirection() {
  if (FORCED_DIRECTION == 1) direction = DIR_CW;
  else if (FORCED_DIRECTION == -1) direction = DIR_CCW;
  else if (directionSamples >= 4) {
    const int leftAverage = directionLeftSum / directionSamples;
    const int rightAverage = directionRightSum / directionSamples;
    direction = rightAverage < leftAverage ? DIR_CW : DIR_CCW;
  } else {
    direction = DIR_CW; // safe compile-time fallback; verify before competition
  }
  targetYawDeg = yawDeg;
  enterState(CRUISE);
}

void handleEmergencyConditions() {
  if (state == WAIT_START || state == FINISHED || state == FAULT_STOP) return;
  if (millis() - runStartedAt >= RUN_TIMEOUT_MS) {
    enterState(FAULT_STOP);
    return;
  }
  if (digitalRead(DRIVER_FAULT_PIN) == LOW) {
    enterState(FAULT_STOP);
    return;
  }
  // A stale camera is dangerous in the obstacle challenge. Give it time to
  // recover, then stop rather than blindly pass a traffic sign.
  if (CHALLENGE_MODE == OBSTACLE_CHALLENGE &&
      millis() - vision.receivedAt > 2200) {
    enterState(FAULT_STOP);
    return;
  }
  if (validDistance(frontMm) && frontMm < EMERGENCY_FRONT_MM &&
      state != PARK_REVERSE_TURN && state != PARK_REVERSE_STRAIGHT) {
    enterState(FAULT_STOP);
  }
}

void controlStep() {
  handleEmergencyConditions();
  const uint32_t inState = millis() - stateStartedAt;

  switch (state) {
    case WAIT_START:
      stopRobot();
      break;

    case EXIT_PARKING:
      setSteering(direction == DIR_CCW ? STEER_SAFE_LEFT : STEER_SAFE_RIGHT);
      rampMotorToward(PWM_PARK);
      if (inState >= EXIT_PARK_MS) enterState(DETERMINE_DIRECTION);
      break;

    case DETERMINE_DIRECTION:
      setSteering(STEER_CENTRE);
      rampMotorToward(PWM_PARK);
      if (validDistance(leftMm) && validDistance(rightMm)) {
        directionLeftSum += leftMm;
        directionRightSum += rightMm;
        directionSamples++;
      }
      if (inState >= DIRECTION_SAMPLE_MS) chooseDirection();
      break;

    case CRUISE: {
      const SignColour seen = visibleSign();
      if (CHALLENGE_MODE == OBSTACLE_CHALLENGE && seen != SIGN_NONE) {
        trackedSign = seen;
        lastSignAt = millis();
        if (closeObstacle(seen)) {
          enterState(AVOID_TRACK);
          break;
        }
      }
      if (validDistance(frontMm) && frontMm <= CORNER_FRONT_MM &&
          millis() - lastCornerAt >= CORNER_COOLDOWN_MS) {
        beginCorner();
        break;
      }
      setSteering(wallFollowSteering());
      rampMotorToward(visionFresh() ? PWM_CRUISE : PWM_VISION_LOST);
      break;
    }

    case CORNER_TURN: {
      const float error = wrapAngle(targetYawDeg - yawDeg);
      setSteering(error > 0 ? STEER_FULL_LEFT : STEER_FULL_RIGHT);
      rampMotorToward(PWM_CORNER);
      finishCornerIfReady();
      break;
    }

    case AVOID_TRACK: {
      const SignColour seen = visibleSign();
      if (seen != SIGN_NONE) {
        trackedSign = seen;
        lastSignAt = millis();
        setSteering(obstacleTrackingSteering(trackedSign));
      } else if (millis() - lastSignAt <= SIGN_MEMORY_MS) {
        setSteering(trackedSign == SIGN_RED ? STEER_SAFE_RIGHT : STEER_SAFE_LEFT);
      } else if (inState >= AVOID_MIN_MS) {
        enterState(AVOID_RECOVER);
      }
      rampMotorToward(PWM_AVOID);
      break;
    }

    case AVOID_RECOVER:
      // Small counter-steer returns to the lane without ANTi-incompatible
      // encoder odometry or the wide fixed arcs seen in slower robots.
      setSteering(trackedSign == SIGN_RED ? STEER_CENTRE + 16 : STEER_CENTRE - 16);
      rampMotorToward(PWM_AVOID);
      if (inState >= AVOID_RECOVER_MS) {
        trackedSign = SIGN_NONE;
        targetYawDeg = yawDeg;
        enterState(CRUISE);
      }
      break;

    case OPEN_FINISH:
      setSteering(wallFollowSteering());
      rampMotorToward(PWM_PARK);
      if (inState >= OPEN_FINISH_DRIVE_MS) enterState(FINISHED);
      break;

    case SEEK_PARKING:
      setSteering(wallFollowSteering());
      rampMotorToward(PWM_PARK);
      if (visionFresh() && vision.magentaCount >= 1 && vision.magentaH >= 13) {
        enterState(PARK_FORWARD);
      }
      break;

    case PARK_FORWARD:
      setSteering(STEER_CENTRE);
      rampMotorToward(PWM_PARK);
      if (inState >= PARK_FORWARD_MS ||
          (vision.magentaH >= 36 && visionFresh())) {
        enterState(PARK_REVERSE_TURN);
      }
      break;

    case PARK_REVERSE_TURN: {
      // Outer wall is left when travelling CW, right when travelling CCW.
      const int reverseTurn = direction == DIR_CW ? STEER_SAFE_RIGHT : STEER_SAFE_LEFT;
      setSteering(reverseTurn);
      rampMotorToward(-PWM_REVERSE);
      if (inState >= PARK_REVERSE_TURN_MS) enterState(PARK_REVERSE_STRAIGHT);
      break;
    }

    case PARK_REVERSE_STRAIGHT:
      setSteering(direction == DIR_CW ? STEER_SAFE_LEFT : STEER_SAFE_RIGHT);
      rampMotorToward(-PWM_REVERSE);
      if (inState >= PARK_REVERSE_STRAIGHT_MS) enterState(PARK_FINAL_ADJUST);
      break;

    case PARK_FINAL_ADJUST:
      setSteering(STEER_CENTRE);
      rampMotorToward(PWM_PARK / 2);
      if (inState >= PARK_FINAL_ADJUST_MS) enterState(FINISHED);
      break;

    case FINISHED:
    case FAULT_STOP:
      stopRobot();
      break;
  }
}

void sendTelemetry() {
  // Optional diagnostic response to ESP. It is wired, not wireless.
  Serial1.print("T,"); Serial1.print(frontMm); Serial1.print(',');
  Serial1.print(leftMm); Serial1.print(','); Serial1.print(rightMm); Serial1.print(',');
  Serial1.print(yawDeg, 1); Serial1.print(','); Serial1.print(lapCount); Serial1.print(',');
  Serial1.println(stateName(state));
}

void waitForReleasedStartButton() {
  static bool previousPressed = false;
  const bool pressed = digitalRead(START_BUTTON_PIN) == LOW;
  if (state == WAIT_START && previousPressed && !pressed) {
    yawDeg = 0;
    targetYawDeg = 0;
    lastImuUs = micros();
    runStartedAt = millis();
    direction = FORCED_DIRECTION == 1 ? DIR_CW :
                FORCED_DIRECTION == -1 ? DIR_CCW : DIR_UNKNOWN;
    enterState(START_FROM_PARKING ? EXIT_PARKING : DETERMINE_DIRECTION);
  }
  previousPressed = pressed;
}

void setup() {
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
#if defined(ARDUINO_ARCH_RP2040)
  analogWriteFreq(20000);
  analogWriteRange(255);
#endif
  setMotorRaw(0);
  steering.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setSteering(STEER_CENTRE);

  Serial1.setTX(UART_TX_PIN);
  Serial1.setRX(UART_RX_PIN);
  Serial1.begin(115200);
  uartLine.reserve(150);

  const bool essentialSensors = initializeSensors();
  calibrateGyroAutomatically();
  if (!essentialSensors) enterState(FAULT_STOP);
  else enterState(WAIT_START);
  Serial1.println("READY,PICO,MEOWTOR2026");
}

void loop() {
  serviceVisionUart();
  waitForReleasedStartButton();

  const uint32_t now = millis();
  if (now - lastSensorAt >= SENSOR_PERIOD_MS) {
    lastSensorAt = now;
    readDistances();
    updateYaw();
  }
  if (now - lastControlAt >= CONTROL_PERIOD_MS) {
    lastControlAt = now;
    controlStep();
  }
  if (now - lastStatusAt >= 250) {
    lastStatusAt = now;
    sendTelemetry();
  }
}
