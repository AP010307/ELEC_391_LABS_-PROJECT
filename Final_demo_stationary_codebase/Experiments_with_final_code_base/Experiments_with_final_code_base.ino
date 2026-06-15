#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// ============================================================
//  BLE SETUP
// ============================================================
#define BUFFER_SIZE 20

BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC",
  BLERead | BLEWrite | BLENotify,
  BUFFER_SIZE,
  false
);

// ============================================================
//  PID CONTROLLER STRUCT
// ============================================================
typedef struct {
  float Kp, Ki, Kd, tau;
  float limMin, limMax, limMinInt, limMaxInt, T;
  float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void PIDController_Init(PIDController *pid) {
  pid->integrator = 0;
  pid->prevError = 0;
  pid->differentiator = 0;
  pid->prevMeasurement = 0;
  pid->out = 0;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
  float error = setpoint - measurement;

  float proportional = pid->Kp * error;

  pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

  if (pid->integrator > pid->limMaxInt) {
    pid->integrator = pid->limMaxInt;
  } else if (pid->integrator < pid->limMinInt) {
    pid->integrator = pid->limMinInt;
  }

  // Optional integrator leak. Uncomment if slow drift returns.
  // pid->integrator *= 0.995f;

  pid->differentiator =
    -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
      + (2.0f * pid->tau - pid->T) * pid->differentiator)
    / (2.0f * pid->tau + pid->T);

  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }

  pid->prevError = error;
  pid->prevMeasurement = measurement;

  return pid->out;
}

// ============================================================
//  PIN ASSIGNMENTS
// ============================================================
const int LEFT_IN1  = 6;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 5;

const int LEFT_ENC_A  = 7;
const int LEFT_ENC_B  = 8;
const int RIGHT_ENC_A = 2;
const int RIGHT_ENC_B = 4;

// ============================================================
//  ENCODER STATE
// ============================================================
volatile long encL = 0;
volatile long encR = 0;

long prevEncL_yaw = 0;
long prevEncR_yaw = 0;

// If forward position goes the wrong direction, flip these signs.
const int ENC_SIGN_L = +1;
const int ENC_SIGN_R = +1;

// ============================================================
//  ROBOT GEOMETRY
// ============================================================
const float COUNTS_PER_REV = 480.0f;     // confirm for your gearmotor
const float WHEEL_DIAMETER_CM = 8.024f;   // 80.24 mm
const float WHEEL_CIRC_CM = PI * WHEEL_DIAMETER_CM;

// Measure this: distance between left and right wheel contact centers.
const float WHEEL_BASE_CM = 25.3f;        // EDIT after measuring

// ============================================================
//  LOOP / IMU FILTER
// ============================================================
const float LOOP_HZ = 200.0f;
const unsigned long LOOP_US = (unsigned long)(1000000.0f / LOOP_HZ);

const float TAU = 0.75f;

unsigned long lastLoopUs = 0;

float angle = 0.0f;
float gyroBias = 0.0f;

float ax = 0.0f;
float ay = 0.0f;
float az = 1.0f;

float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

// ============================================================
//  MOTOR LAYER
// ============================================================
const int DB_L = 17;
const int DB_R = 16;

const float U_MAX = 255.0f;
const float U_DEADZONE = 2.0f;

const int FWD_SIGN_L = -1;
const int FWD_SIGN_R = +1;

// Left motor was faster, so reduce it.
const float LEFT_MOTOR_SCALE  = 0.92f;
const float RIGHT_MOTOR_SCALE = 1.00f;

// ============================================================
//  INNER BALANCE PID CONFIG
// ============================================================
const float BASE_OFFSET = 0.9f;
const int OUTPUT_SIGN = -1;

const float FALL_LIMIT = 35.0f;

const float I_MAX = 30.0f;
const float DERIV_TAU = 0.02f;

PIDController pid;

// ============================================================
//  DRIVE / TURN COMMAND SETTINGS
// ============================================================
const float FWD_LEAN_FWD  =  0.2f;
const float FWD_LEAN_BACK = -0.3f;

const float LEFTTURN  = 5.0f;
const float RIGHTTURN = 6.0f;

const int LEAN_SIGN = +1;
const int TURN_SIGN = +1;

float targetdriveLean = 0.0f;
float targetturnBias  = 0.0f;

float driveLean = 0.0f;
float turnBias  = 0.0f;

const float LEAN_RAMP = 0.005f;
const float TURN_RAMP = 0.25f;

// ============================================================
//  YAW CORRECTION
// ============================================================
const float YAW_KP_FWD   = 0.06f;
const float YAW_KP_BACK  = 0.02f;
const float YAW_KP_STILL = 0.03f;
const float YAW_TRIM_MAX = 10.0f;

// ============================================================
//  STATIONARY POSITION HOLD
// ============================================================
const float POS_LIMIT_CM = 4.0f;
const float POS_OPT_LIMIT_CM = 2.0f;

const float POS_HOLD_KP = 0.06f;       // deg/cm
const float POS_HOLD_KD = 0.015f;      // deg/(cm/s)
const float POS_HOLD_LEAN_MAX = 0.8f;  // deg

float holdPositionCm = 0.0f;
float prevHoldErrorCm = 0.0f;

// ============================================================
//  50 CM MOVE CONTROL
// ============================================================
const float MOVE_DISTANCE_CM = 50.0f;
const float MOVE_STOP_TOL_CM = 3.0f;
const float MOVE_STOP_SPEED_CM_S = 3.0f;

const float MOVE_SPEED_MAX_CM_S = 12.0f;     // requirement is at least 10 cm/s
const float MOVE_POS_TO_SPEED_KP = 0.8f;     // position error -> target speed
const float MOVE_SPEED_KP = 0.040f;          // speed error -> lean angle
const float MOVE_LEAN_MAX = 0.65f;           // safer than 0.9 at first

float moveTargetCm = 0.0f;
float prevMovePosCm = 0.0f;
float moveVelCmS = 0.0f;
float targetSpeedCmS = 0.0f;
float prevMoveErrorCm = 0.0f;

// ============================================================
//  TURN CONTROL
// ============================================================
const float TURN_ANGLE_DEG = 45.0f;
const float TURN_STOP_TOL_DEG = 3.0f;

const float TURN_HEADING_KP = 0.35f;
const float TURN_HEADING_KD = 0.04f;
const float TURN_BIAS_MAX = 22.0f;

float turnStartHeadingDeg = 0.0f;
float turnTargetHeadingDeg = 0.0f;
float prevHeadingErrorDeg = 0.0f;

// ============================================================
//  ROBOT MODE
// ============================================================
enum RobotMode {
  MODE_HOLD,
  MODE_MOVE_FWD_50,
  MODE_MOVE_BACK_50,
  MODE_TURN_LEFT_45,
  MODE_TURN_RIGHT_45
};

RobotMode mode = MODE_HOLD;

bool armed = true;
unsigned long lastPrint = 0;

// ============================================================
//  ENCODER ISRs
// ============================================================
void isrLeft() {
  if (digitalRead(LEFT_ENC_B)) encL++;
  else encL--;
}

void isrRight() {
  if (digitalRead(RIGHT_ENC_B)) encR++;
  else encR--;
}

// ============================================================
//  BASIC HELPERS
// ============================================================
float accelAngleDeg(float x, float y, float z) {
  return atan2(y, z) * 180.0f / PI;
}

float gyroPitchRate(float rx, float ry, float rz) {
  return -rx;
}

float moveToward(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }

  return current;
}

void resetPID() {
  pid.integrator = 0;
  pid.differentiator = 0;
  pid.prevError = 0;
  pid.prevMeasurement = angle;
}

void resetHoldPosition() {
  holdPositionCm = getPositionCm();
  prevHoldErrorCm = 0.0f;
}

void resetYawMemory() {
  noInterrupts();
  prevEncL_yaw = encL;
  prevEncR_yaw = encR;
  interrupts();
}

// ============================================================
//  POSITION / HEADING HELPERS
// ============================================================
float getLeftDistanceCm() {
  long currentL;

  noInterrupts();
  currentL = encL;
  interrupts();

  float counts = ENC_SIGN_L * currentL;
  return (counts / COUNTS_PER_REV) * WHEEL_CIRC_CM;
}

float getRightDistanceCm() {
  long currentR;

  noInterrupts();
  currentR = encR;
  interrupts();

  float counts = ENC_SIGN_R * currentR;
  return (counts / COUNTS_PER_REV) * WHEEL_CIRC_CM;
}

float getPositionCm() {
  float leftCm = getLeftDistanceCm();
  float rightCm = getRightDistanceCm();

  return 0.5f * (leftCm + rightCm);
}

float getHeadingDeg() {
  float leftCm = getLeftDistanceCm();
  float rightCm = getRightDistanceCm();

  float headingRad = (rightCm - leftCm) / WHEEL_BASE_CM;
  return headingRad * 180.0f / PI;
}

// ============================================================
//  MOTOR FUNCTIONS
// ============================================================
void setMotorRaw(int in1, int in2, int cmd) {
  cmd = constrain(cmd, -255, 255);

  if (cmd >= 0) {
    analogWrite(in1, 255);
    analogWrite(in2, 255 - cmd);
  } else {
    analogWrite(in2, 255);
    analogWrite(in1, 255 + cmd);
  }
}

int compensate(float u, int db) {
  float a = fabs(u);

  if (a < U_DEADZONE) {
    return 0;
  }

  int out = (int)(db + (a / U_MAX) * (255.0f - db) + 0.5f);
  out = constrain(out, 0, 255);

  return (u >= 0) ? out : -out;
}

void driveLR(float uL, float uR) {
  uL *= LEFT_MOTOR_SCALE;
  uR *= RIGHT_MOTOR_SCALE;

  setMotorRaw(LEFT_IN1, LEFT_IN2, compensate(FWD_SIGN_L * uL, DB_L));
  setMotorRaw(RIGHT_IN1, RIGHT_IN2, compensate(FWD_SIGN_R * uR, DB_R));
}

void coast() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}

// ============================================================
//  ENCODER YAW CORRECTION
// ============================================================
float getEncoderYawCorrection(float movementCommand) {
  long currentL, currentR;

  noInterrupts();
  currentL = encL;
  currentR = encR;
  interrupts();

  long dL = currentL - prevEncL_yaw;
  long dR = currentR - prevEncR_yaw;

  prevEncL_yaw = currentL;
  prevEncR_yaw = currentR;

  long diff = dL - dR;

  if (abs(diff) <= 1) {
    return 0.0f;
  }

  float kpYaw;

  if (movementCommand > 0.05f) {
    kpYaw = YAW_KP_FWD;
  } else if (movementCommand < -0.05f) {
    kpYaw = YAW_KP_BACK;
  } else {
    kpYaw = YAW_KP_STILL;
  }

  float correction = kpYaw * diff;
  correction = constrain(correction, -YAW_TRIM_MAX, YAW_TRIM_MAX);

  return correction;
}

// ============================================================
//  MODE START HELPERS
// ============================================================
void enterHoldMode() {
  mode = MODE_HOLD;

  targetdriveLean = 0.0f;
  targetturnBias = 0.0f;
  driveLean = 0.0f;
  turnBias = 0.0f;

  resetHoldPosition();
  prevMoveErrorCm = 0.0f;
  prevHeadingErrorDeg = 0.0f;
  resetYawMemory();
}

void startMove(float distanceCm) {
  float currentPos = getPositionCm();

  moveTargetCm = currentPos + distanceCm;
  prevMovePosCm = currentPos;
  moveVelCmS = 0.0f;
  targetSpeedCmS = 0.0f;

  targetturnBias = 0.0f;
  turnBias = 0.0f;

  resetYawMemory();

  if (distanceCm > 0) {
    mode = MODE_MOVE_FWD_50;
  } else {
    mode = MODE_MOVE_BACK_50;
  }
}

void startTurn(float angleDeg) {
  turnStartHeadingDeg = getHeadingDeg();
  turnTargetHeadingDeg = turnStartHeadingDeg + angleDeg;

  // Avoid derivative kick on first turn step
  prevHeadingErrorDeg = turnTargetHeadingDeg - turnStartHeadingDeg;

  targetdriveLean = 0.0f;
  driveLean = 0.0f;

  resetYawMemory();

  if (angleDeg > 0) {
    mode = MODE_TURN_RIGHT_45;
  } else {
    mode = MODE_TURN_LEFT_45;
  }
}

// ============================================================
//  BLE / SERIAL COMMAND HANDLING
// ============================================================
void handleCommand(String c) {
  c.trim();
  c.toUpperCase();

  if (c == "FORWARD" || c == "F" || c == "UP") {
    startMove(MOVE_DISTANCE_CM);
  } else if (c == "BACKWARD" || c == "B" || c == "DOWN") {
    startMove(-MOVE_DISTANCE_CM);
  } else if (c == "LEFT" || c == "L") {
    startTurn(-TURN_ANGLE_DEG);
  } else if (c == "RIGHT" || c == "R") {
    startTurn(TURN_ANGLE_DEG);
  } else if (c == "STOP" || c == "S" || c == "A") {
    enterHoldMode();
  } else if (c == "ARM") {
    armed = true;
    resetPID();
    enterHoldMode();
  } else if (c == "KILL" || c == "Z") {
    armed = false;
    coast();
    enterHoldMode();
  } else {
    enterHoldMode();
  }

  Serial.print("cmd: ");
  Serial.println(c);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  while (!Serial && millis() < 3000) {}

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  coast();

  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRight, RISING);

  if (!IMU.begin()) {
    Serial.println("IMU FAIL");
    while (1) {}
  }

  Serial.println("Calibrating gyro bias - hold STILL...");
  delay(500);

  const int N = 500;
  float sum = 0.0f;
  int got = 0;

  while (got < N) {
    if (IMU.gyroscopeAvailable()) {
      float a, b, c;
      IMU.readGyroscope(a, b, c);
      sum += gyroPitchRate(a, b, c);
      got++;
    }
  }

  gyroBias = sum / N;

  while (!IMU.accelerationAvailable()) {}
  IMU.readAcceleration(ax, ay, az);
  angle = accelAngleDeg(ax, ay, az);

  PIDController_Init(&pid);

  // Your current working balance gains
  pid.Kp = 6.9f;
  pid.Ki = 100.0f;
  pid.Kd = 0.7f;
  pid.tau = DERIV_TAU;

  pid.limMin = -U_MAX;
  pid.limMax = U_MAX;
  pid.limMinInt = -I_MAX;
  pid.limMaxInt = I_MAX;
  pid.T = 1.0f / LOOP_HZ;

  resetPID();

  enterHoldMode();

  if (!BLE.begin()) {
    Serial.println("BLE FAIL");
    while (1) {}
  }

  BLE.setLocalName("BLE-DEVICE_TEAM6");
  BLE.setDeviceName("BLE-DEVICE_TEAM6");

  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);

  customCharacteristic.writeValue("ready");
  BLE.advertise();

  Serial.println("ARMED + balancing.");
  Serial.println("Commands: F=move 50cm, B=back 50cm, L=left 45deg, R=right 45deg, S=hold");

  lastLoopUs = micros();
}

// ============================================================
//  MAIN CONTROL STEP
// ============================================================
void controlStep() {
  unsigned long now = micros();

  if (now - lastLoopUs < LOOP_US) {
    return;
  }

  float dt = (now - lastLoopUs) * 1e-6f;
  lastLoopUs = now;

  // -------------------- IMU angle estimate --------------------
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  float accAng = accelAngleDeg(ax, ay, az);
  float rate = gyroPitchRate(gx, gy, gz) - gyroBias;

  float k = TAU / (TAU + dt);
  angle = k * (angle + rate * dt) + (1.0f - k) * accAng;

  // -------------------- Encoder measurements --------------------
  float posCm = getPositionCm();
  float headingDeg = getHeadingDeg();

  float driveLeanCmd = 0.0f;
  float turnBiasCmd = 0.0f;

  float posErrorCm = 0.0f;
  float posVelCmS = 0.0f;
  float headingErrorDeg = 0.0f;

  // -------------------- Outer-loop mode control --------------------
  if (mode == MODE_HOLD) {
    posErrorCm = posCm - holdPositionCm;
    posVelCmS = (posErrorCm - prevHoldErrorCm) / dt;
    prevHoldErrorCm = posErrorCm;

    driveLeanCmd = -(POS_HOLD_KP * posErrorCm + POS_HOLD_KD * posVelCmS);
    driveLeanCmd = constrain(driveLeanCmd, -POS_HOLD_LEAN_MAX, POS_HOLD_LEAN_MAX);

    turnBiasCmd = 0.0f;
  }

else if (mode == MODE_MOVE_FWD_50 || mode == MODE_MOVE_BACK_50) {
  posErrorCm = moveTargetCm - posCm;

  // Actual robot velocity
  moveVelCmS = (posCm - prevMovePosCm) / dt;
  prevMovePosCm = posCm;

  // As it approaches the target, target speed automatically decreases
  targetSpeedCmS = MOVE_POS_TO_SPEED_KP * posErrorCm;
  targetSpeedCmS = constrain(targetSpeedCmS,
                             -MOVE_SPEED_MAX_CM_S,
                              MOVE_SPEED_MAX_CM_S);

  // Lean based on speed error
  float speedErrorCmS = targetSpeedCmS - moveVelCmS;

  driveLeanCmd = MOVE_SPEED_KP * speedErrorCmS;
  driveLeanCmd = constrain(driveLeanCmd, -MOVE_LEAN_MAX, MOVE_LEAN_MAX);

  turnBiasCmd = 0.0f;

  if (fabs(posErrorCm) < MOVE_STOP_TOL_CM &&
      fabs(moveVelCmS) < MOVE_STOP_SPEED_CM_S) {
    enterHoldMode();
    driveLeanCmd = 0.0f;
    turnBiasCmd = 0.0f;
  }
}

  else if (mode == MODE_TURN_LEFT_45 || mode == MODE_TURN_RIGHT_45) {
    headingErrorDeg = turnTargetHeadingDeg - headingDeg;
    float headingRateDegS = (headingErrorDeg - prevHeadingErrorDeg) / dt;
    prevHeadingErrorDeg = headingErrorDeg;

    driveLeanCmd = 0.0f;

    turnBiasCmd = TURN_HEADING_KP * headingErrorDeg + TURN_HEADING_KD * headingRateDegS;
    turnBiasCmd = constrain(turnBiasCmd, -TURN_BIAS_MAX, TURN_BIAS_MAX);

    if (fabs(headingErrorDeg) < TURN_STOP_TOL_DEG) {
        enterHoldMode();
        driveLeanCmd = 0.0f;
        turnBiasCmd = 0.0f;
    }
  }

  targetdriveLean = driveLeanCmd;
  targetturnBias = turnBiasCmd;

  float leanRampNow;

  if (mode == MODE_HOLD) {
    leanRampNow = LEAN_RAMP;       // keep stationary smooth
    } else {
    leanRampNow = 0.03f;           // brake faster during movement
  } 

driveLean = moveToward(driveLean, targetdriveLean, leanRampNow);
  turnBias  = moveToward(turnBias,  targetturnBias,  TURN_RAMP);

  // -------------------- Inner balance PID --------------------
  pid.T = dt;

  float setpoint = BASE_OFFSET + driveLean;

  float pidOut = PIDController_Update(&pid, setpoint, angle);
  float uOut = OUTPUT_SIGN * pidOut;

  // -------------------- Yaw correction --------------------
  float yawCorrection = 0.0f;

  if (fabs(targetturnBias) < 0.1f && fabs(turnBias) < 0.1f) {
    yawCorrection = getEncoderYawCorrection(driveLean);
  }

  float uL = uOut + turnBias + yawCorrection;
  float uR = uOut - turnBias - yawCorrection;

  // -------------------- Safety / motor output --------------------
  if (armed && fabs(angle) < FALL_LIMIT) {
    driveLR(uL, uR);
  } else {
    coast();
    resetPID();
    enterHoldMode();
  }

  // -------------------- Debug print --------------------
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print("mode=");
    Serial.print((int)mode);

    Serial.print(" ang=");
    Serial.print(angle, 1);

    Serial.print(" sp=");
    Serial.print(setpoint, 2);

    Serial.print(" pos=");
    Serial.print(posCm, 1);

    Serial.print(" pErr=");
    Serial.print(posErrorCm, 1);

    Serial.print(" head=");
    Serial.print(headingDeg, 1);

    Serial.print(" hErr=");
    Serial.print(headingErrorDeg, 1);

    Serial.print(" lean=");
    Serial.print(driveLean, 2);

    Serial.print(" u=");
    Serial.print(uOut, 0);

    Serial.print(" yawC=");
    Serial.print(yawCorrection, 1);

    if (mode == MODE_HOLD) {
      if (fabs(posErrorCm) <= POS_OPT_LIMIT_CM) {
        Serial.print(" POS_OPT");
      } else if (fabs(posErrorCm) <= POS_LIMIT_CM) {
        Serial.print(" POS_OK");
      } else {
        Serial.print(" POS_OUT");
      }
    }

    Serial.println();
  }
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  BLEDevice central = BLE.central();

  if (central && central.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);

    if (customCharacteristic.written()) {
      int len = customCharacteristic.valueLength();

      if (len > BUFFER_SIZE) {
        len = BUFFER_SIZE;
      }

      const unsigned char *d = customCharacteristic.value();

      char buf[BUFFER_SIZE + 1];
      memcpy(buf, d, len);
      buf[len] = '\0';

      handleCommand(String(buf));
      customCharacteristic.writeValue("ok");
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');

    if (s.length()) {
      handleCommand(s);
    }
  }

  controlStep();
}