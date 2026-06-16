#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

#define BUFFER_SIZE 20
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC",
  BLERead | BLEWrite | BLENotify,
  BUFFER_SIZE,
  false
);

//  PID CONTROLLER STRUCT
typedef struct {
  float Kp, Ki, Kd, tau;
  float limMin, limMax, limMinInt, limMaxInt, T;
  float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void PIDController_Init(PIDController *pid) {
  pid->integrator      = 0;
  pid->prevError       = 0;
  pid->differentiator  = 0;
  pid->prevMeasurement = 0;
  pid->out             = 0;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
  float error       = setpoint - measurement;
  float proportional = pid->Kp * error;

  pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
  if      (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
  else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

  pid->differentiator =
    -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
      + (2.0f * pid->tau - pid->T) * pid->differentiator)
    / (2.0f * pid->tau + pid->T);

  pid->out = proportional + pid->integrator + pid->differentiator;
  if      (pid->out > pid->limMax) pid->out = pid->limMax;
  else if (pid->out < pid->limMin) pid->out = pid->limMin;

  pid->prevError       = error;
  pid->prevMeasurement = measurement;
  return pid->out;
}

//  ENUMS  — declared early so all functions can reference them
enum RobotMode {
  MODE_HOLD,
  MODE_MOVE_FWD_50,
  MODE_MOVE_BACK_50,
  MODE_TURN_LEFT_45,
  MODE_TURN_RIGHT_45
};

enum SequenceStep {
  SEQ_IDLE,
  SEQ_MOVE,
  SEQ_PAUSE_AFTER_MOVE,
  SEQ_TURN_FIRST,
  SEQ_PAUSE_MID_TURN,
  SEQ_TURN_SECOND,
  SEQ_PAUSE_AFTER_TURN
};

//  FORWARD DECLARATIONS — lets functions call each other freely
float getPositionCm();
void  enterHoldMode();
void  startPause(SequenceStep nextStep, unsigned long durationMs);
void  controlStep(float dt);

//  PIN ASSIGNMENTS
const int LEFT_IN1  = 6, LEFT_IN2  = 9;
const int RIGHT_IN1 = 3, RIGHT_IN2 = 5;
const int LEFT_ENC_A  = 7, LEFT_ENC_B  = 8;
const int RIGHT_ENC_A = 2, RIGHT_ENC_B = 4;

//  ENCODER STATE
volatile long encL = 0, encR = 0;
long prevEncL_yaw = 0, prevEncR_yaw = 0;

const int ENC_SIGN_L = +1;
const int ENC_SIGN_R = +1;

//  ROBOT GEOMETRY
const float COUNTS_PER_REV    = 480.0f;
const float WHEEL_DIAMETER_CM = 8.024f;
const float WHEEL_CIRC_CM     = PI * WHEEL_DIAMETER_CM;
const float WHEEL_BASE_CM     = 25.3f;

//  LOOP / IMU FILTER
const float        LOOP_HZ = 200.0f;
const unsigned long LOOP_US = (unsigned long)(1000000.0f / LOOP_HZ);

const float TAU = 0.75f;

unsigned long lastLoopUs  = 0;   // for dt calculation
unsigned long lastLoop = 0;   // for fixed-rate gate (never drifts)
float angle = 0.0f, gyroBias = 0.0f;
float ax = 0.0f, ay = 0.0f, az = 1.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;

//  MOTOR LAYER
const int   DB_L = 17, DB_R = 16;
const float U_MAX = 255.0f, U_DEADZONE = 2.0f;
const int   FWD_SIGN_L = -1, FWD_SIGN_R = +1;
const float LEFT_MOTOR_SCALE  = 0.92f;
const float RIGHT_MOTOR_SCALE = 1.00f;

//  INNER BALANCE PID CONFIG
const float BASE_OFFSET = 0.9f;
const int   OUTPUT_SIGN = -1;
const float FALL_LIMIT  = 35.0f;
const float I_MAX       = 30.0f;
const float DERIV_TAU   = 0.02f;

PIDController pid;

//  DRIVE / TURN COMMAND SETTINGS
const float FWD_LEAN_FWD  =  0.2f;
const float FWD_LEAN_BACK = -0.5f;
const float LEFTTURN      =  5.0f;
const float RIGHTTURN      =  6.0f;
const int   LEAN_SIGN     = +1;
const int   TURN_SIGN     = +1;

float targetdriveLean = 0.0f, targetturnBias = 0.0f;
float driveLean       = 0.0f, turnBias       = 0.0f;

const float LEAN_RAMP = 0.005f;
const float TURN_RAMP = 0.25f;

//  YAW CORRECTION
const float YAW_KP_FWD   = 0.06f;
const float YAW_KP_BACK  = 0.02f;
const float YAW_KP_STILL = 0.03f;
const float YAW_TRIM_MAX = 10.0f;

//  STATIONARY POSITION HOLD
const float POS_LIMIT_CM     = 4.0f;
const float POS_OPT_LIMIT_CM = 2.0f;
const float POS_HOLD_KP       = 0.06f;
const float POS_HOLD_KD       = 0.015f;
const float POS_HOLD_LEAN_MAX = 0.8f;

float holdPositionCm  = 0.0f;
float prevHoldErrorCm = 0.0f;

// ============================================================
//  50 CM MOVE CONTROL
// ============================================================
const float MOVE_DISTANCE_CM     = 50.0f;
const float MOVE_STOP_TOL_CM     = 3.0f;
const float MOVE_STOP_SPEED_CM_S = 3.0f;
const float MOVE_SPEED_MAX_CM_S  = 10.0f;
const float MOVE_POS_TO_SPEED_KP = 0.6f;
const float MOVE_SPEED_KP        = 0.040f;
const float MOVE_LEAN_MAX        = 0.65f;

float moveTargetCm    = 0.0f;
float prevMovePosCm   = 0.0f;
float moveVelCmS      = 0.0f;
float targetSpeedCmS  = 0.0f;
float prevMoveErrorCm = 0.0f;

//  TURN CONTROL
const float TURN_ANGLE_DEG    = 45.0f;
const float TURN_STOP_TOL_DEG = 3.0f;
const float TURN_HEADING_KP   = 0.35f;
const float TURN_HEADING_KD   = 0.04f;
const float TURN_BIAS_MAX     = 22.0f;

float turnStartHeadingDeg  = 0.0f;
float turnTargetHeadingDeg = 0.0f;
float prevHeadingErrorDeg  = 0.0f;

//  PAUSE / SEQUENCE TIMING
const unsigned long PAUSE_AFTER_MOVE_MS   = 6000UL;
const unsigned long PAUSE_BETWEEN_TURN_MS = 2500UL;
const unsigned long PAUSE_AFTER_TURN_MS   = 6000UL;

SequenceStep  seqStep      = SEQ_IDLE;
unsigned long pauseStartMs = 0;
int           pendingTurnDir = 0;

//  MODE STATE
RobotMode mode = MODE_HOLD;
bool      armed = true;
unsigned long lastPrint = 0;


//  ENCODER ISRs
void isrLeft()  { if (digitalRead(LEFT_ENC_B)) encL++; else encL--; }
void isrRight() { if (digitalRead(RIGHT_ENC_B)) encR++; else encR--; }

//  BASIC HELPERS
float accelAngleDeg(float x, float y, float z) {
  return atan2(y, z) * 180.0f / PI;
}

float gyroPitchRate(float rx, float ry, float rz) { return -rx; }

float moveToward(float current, float target, float step) {
  if      (current < target) { current += step; if (current > target) current = target; }
  else if (current > target) { current -= step; if (current < target) current = target; }
  return current;
}

void resetPID() {
  pid.integrator     = 0;
  pid.differentiator = 0;
  pid.prevError      = 0;
  pid.prevMeasurement = angle;
}

//  POSITION / HEADING HELPERS
float getLeftDistanceCm() {
  long v; noInterrupts(); v = encL; interrupts();
  return (ENC_SIGN_L * v / COUNTS_PER_REV) * WHEEL_CIRC_CM;
}

float getRightDistanceCm() {
  long v; noInterrupts(); v = encR; interrupts();
  return (ENC_SIGN_R * v / COUNTS_PER_REV) * WHEEL_CIRC_CM;
}

float getPositionCm() {
  return 0.5f * (getLeftDistanceCm() + getRightDistanceCm());
}

float getHeadingDeg() {
  float headingRad = (getRightDistanceCm() - getLeftDistanceCm()) / WHEEL_BASE_CM;
  return headingRad * 180.0f / PI;
}

void resetHoldPosition() {
  holdPositionCm  = getPositionCm();
  prevHoldErrorCm = 0.0f;
}

void resetYawMemory() {
  noInterrupts(); prevEncL_yaw = encL; prevEncR_yaw = encR; interrupts();
}

//  MOTOR FUNCTIONS
void setMotorRaw(int in1, int in2, int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd >= 0) { analogWrite(in1, 255); analogWrite(in2, 255 - cmd); }
  else          { analogWrite(in2, 255); analogWrite(in1, 255 + cmd); }
}

int compensate(float u, int db) {
  float a = fabs(u);
  if (a < U_DEADZONE) return 0;
  int out = (int)(db + (a / U_MAX) * (255.0f - db) + 0.5f);
  out = constrain(out, 0, 255);
  return (u >= 0) ? out : -out;
}

void driveLR(float uL, float uR) {
  uL *= LEFT_MOTOR_SCALE; uR *= RIGHT_MOTOR_SCALE;
  setMotorRaw(LEFT_IN1, LEFT_IN2, compensate(FWD_SIGN_L * uL, DB_L));
  setMotorRaw(RIGHT_IN1, RIGHT_IN2, compensate(FWD_SIGN_R * uR, DB_R));
}

void coast() {
  analogWrite(LEFT_IN1, 0); analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0); analogWrite(RIGHT_IN2, 0);
}


//  ENCODER YAW CORRECTION
float getEncoderYawCorrection(float movementCommand) {
  long currentL, currentR;
  noInterrupts(); currentL = encL; currentR = encR; interrupts();

  long dL = currentL - prevEncL_yaw;
  long dR = currentR - prevEncR_yaw;
  prevEncL_yaw = currentL;
  prevEncR_yaw = currentR;

  long diff = dL - dR;
  if (abs(diff) <= 1) return 0.0f;

  float kpYaw;
  if (movementCommand >  0.05f) kpYaw = YAW_KP_FWD;
  else if (movementCommand < -0.05f) kpYaw = YAW_KP_BACK;
  else kpYaw = YAW_KP_STILL;

  return constrain(kpYaw * diff, -YAW_TRIM_MAX, YAW_TRIM_MAX);
}

//  INNER MODE TRANSITIONS
void enterHoldMode() {
  mode            = MODE_HOLD;
  targetdriveLean = 0.0f;
  targetturnBias  = 0.0f;
  driveLean       = 0.0f;
  turnBias        = 0.0f;
  prevMoveErrorCm     = 0.0f;
  prevHeadingErrorDeg = 0.0f;
  // Soft-clamp integrator instead of zeroing — prevents torque jerk on stop
  pid.integrator = constrain(pid.integrator, -5.0f, 5.0f);
  resetHoldPosition();
  resetYawMemory();
}

void startMove(float distanceCm) {
  float currentPos = getPositionCm();
  moveTargetCm   = currentPos + distanceCm;
  prevMovePosCm  = currentPos;
  moveVelCmS     = 0.0f;
  targetSpeedCmS = 0.0f;
  targetturnBias = 0.0f;
  turnBias       = 0.0f;
  resetYawMemory();
  mode = (distanceCm > 0) ? MODE_MOVE_FWD_50 : MODE_MOVE_BACK_50;
}

void startTurnInner(float angleDeg) {
  turnStartHeadingDeg  = getHeadingDeg();
  turnTargetHeadingDeg = turnStartHeadingDeg + angleDeg;
  prevHeadingErrorDeg  = angleDeg;   // avoid derivative kick on first tick
  targetdriveLean      = 0.0f;
  driveLean            = 0.0f;
  resetYawMemory();
  mode = (angleDeg > 0) ? MODE_TURN_RIGHT_45 : MODE_TURN_LEFT_45;
}

//  SEQUENCE HELPERS
void startPause(SequenceStep nextStep, unsigned long durationMs) {
  enterHoldMode();
  pauseStartMs = millis();
  seqStep      = nextStep;
}

void triggerMove(float distanceCm) {
  startMove(distanceCm);
  seqStep = SEQ_MOVE;
}

void triggerFirstTurn(int dir) {
  pendingTurnDir = dir;
  startTurnInner(dir * TURN_ANGLE_DEG);
  seqStep = SEQ_TURN_FIRST;
}

//  COMMAND HANDLER
void handleCommand(String c) {
  c.trim(); c.toUpperCase();

  if      (c == "FORWARD"  || c == "F" || c == "UP")   triggerMove(MOVE_DISTANCE_CM);
  else if (c == "BACKWARD" || c == "B" || c == "DOWN")  triggerMove(-MOVE_DISTANCE_CM);
  else if (c == "LEFT" || c == "L") triggerFirstTurn(-1);
  else if (c == "RIGHT" || c == "R") triggerFirstTurn(+1);
  else if (c == "STOP" || c == "S" || c == "A") { seqStep = SEQ_IDLE; enterHoldMode(); }
  else { seqStep = SEQ_IDLE; enterHoldMode(); }

  Serial.print("cmd: "); Serial.println(c);
}

//  SEQUENCE STATE MACHINE
void updateSequence() {
  unsigned long now = millis();

  switch (seqStep) {
    case SEQ_IDLE:
      break;

    case SEQ_MOVE:
      if (mode == MODE_HOLD)
        startPause(SEQ_PAUSE_AFTER_MOVE, PAUSE_AFTER_MOVE_MS);
      break;

    case SEQ_PAUSE_AFTER_MOVE:
      if (now - pauseStartMs >= PAUSE_AFTER_MOVE_MS) {
        seqStep = SEQ_IDLE;
      }
      break;

    case SEQ_TURN_FIRST:
      if (mode == MODE_HOLD)
        startPause(SEQ_PAUSE_MID_TURN, PAUSE_BETWEEN_TURN_MS);
      break;

    case SEQ_PAUSE_MID_TURN:
      if (now - pauseStartMs >= PAUSE_BETWEEN_TURN_MS) {
        startTurnInner(pendingTurnDir * TURN_ANGLE_DEG);
        seqStep = SEQ_TURN_SECOND;
      }
      break;

    case SEQ_TURN_SECOND:
      if (mode == MODE_HOLD)
        startPause(SEQ_PAUSE_AFTER_TURN, PAUSE_AFTER_TURN_MS);
      break;

    case SEQ_PAUSE_AFTER_TURN:
      if (now - pauseStartMs >= PAUSE_AFTER_TURN_MS) {
        seqStep = SEQ_IDLE;
      }
      break;
  }
}

//  SETUP
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  while (!Serial && millis() < 3000) {}

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  coast();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRight, RISING);

  if (!IMU.begin()) { Serial.println("IMU FAIL"); while (1) {} }

  Serial.println("Calibrating gyro - hold STILL...");
  delay(500);

  const int N = 500; 
  float sum = 0.0f; 
  int got = 0;
  while (got < N) {
    if (IMU.gyroscopeAvailable()) {
      float a, b, c; IMU.readGyroscope(a, b, c);
      sum += gyroPitchRate(a, b, c); got++;
    }
  }
  gyroBias = sum / N;

  while (!IMU.accelerationAvailable()) {}
  IMU.readAcceleration(ax, ay, az);
  angle = accelAngleDeg(ax, ay, az);

  PIDController_Init(&pid);
  pid.Kp = 6.9f; pid.Ki = 100.0f; pid.Kd = 0.7f;
  pid.tau = DERIV_TAU;
  pid.limMin = -U_MAX;  pid.limMax = U_MAX;
  pid.limMinInt = -I_MAX;  pid.limMaxInt = I_MAX;
  pid.T = 1.0f / LOOP_HZ;
  resetPID();

  enterHoldMode();

  if (!BLE.begin()) { Serial.println("BLE FAIL"); while (1) {} }
  BLE.setLocalName("BLE-DEVICE_TEAM6");
  BLE.setDeviceName("BLE-DEVICE_TEAM6");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("ready");
  BLE.advertise();

  Serial.println("READY");
  lastLoopUs = micros();
  lastLoop   = micros();
}

//  MAIN CONTROL STEP
void controlStep(float dt) {

  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  float accAng = accelAngleDeg(ax, ay, az);
  float rate = gyroPitchRate(gx, gy, gz) - gyroBias;
  float k = TAU / (TAU + dt);
  angle = k * (angle + rate * dt) + (1.0f - k) * accAng;

  float posCm = getPositionCm();
  float headingDeg = getHeadingDeg();

  float driveLeanCmd = 0.0f;
  float turnBiasCmd = 0.0f;
  float posErrorCm = 0.0f;
  float headingErrorDeg = 0.0f;

  // ── Outer control loop ──
  if (mode == MODE_HOLD) {
    posErrorCm = posCm - holdPositionCm;
    float posVelCmS = (posErrorCm - prevHoldErrorCm) / dt;
    prevHoldErrorCm = posErrorCm;

    driveLeanCmd = -(POS_HOLD_KP * posErrorCm + POS_HOLD_KD * posVelCmS);
    driveLeanCmd = constrain(driveLeanCmd, -POS_HOLD_LEAN_MAX, POS_HOLD_LEAN_MAX);
    turnBiasCmd  = 0.0f;
  }

  else if (mode == MODE_MOVE_FWD_50 || mode == MODE_MOVE_BACK_50) {
    posErrorCm  = moveTargetCm - posCm;
    moveVelCmS  = (posCm - prevMovePosCm) / dt;
    prevMovePosCm = posCm;

    targetSpeedCmS = constrain(
      MOVE_POS_TO_SPEED_KP * posErrorCm,
      -MOVE_SPEED_MAX_CM_S, MOVE_SPEED_MAX_CM_S
    );

    float speedErrorCmS = targetSpeedCmS - moveVelCmS;
    driveLeanCmd = constrain(MOVE_SPEED_KP * speedErrorCmS, -MOVE_LEAN_MAX, MOVE_LEAN_MAX);
    turnBiasCmd  = 0.0f;

    if (fabs(posErrorCm) < MOVE_STOP_TOL_CM && fabs(moveVelCmS) < MOVE_STOP_SPEED_CM_S) {
      enterHoldMode();
      driveLeanCmd = 0.0f;
      turnBiasCmd  = 0.0f;
    }
  }

  else if (mode == MODE_TURN_LEFT_45 || mode == MODE_TURN_RIGHT_45) {
    headingErrorDeg = turnTargetHeadingDeg - headingDeg;
    float headingRateDegS = (headingErrorDeg - prevHeadingErrorDeg) / dt;
    prevHeadingErrorDeg = headingErrorDeg;

    driveLeanCmd = 0.0f;
    turnBiasCmd  = constrain(
      TURN_HEADING_KP * headingErrorDeg + TURN_HEADING_KD * headingRateDegS,
      -TURN_BIAS_MAX, TURN_BIAS_MAX
    );

    if (fabs(headingErrorDeg) < TURN_STOP_TOL_DEG) {
      enterHoldMode();
      driveLeanCmd = 0.0f;
      turnBiasCmd  = 0.0f;
    }
  }

  targetdriveLean = driveLeanCmd;
  targetturnBias  = turnBiasCmd;

  float leanRampNow = (mode == MODE_HOLD) ? LEAN_RAMP : 0.03f;
  driveLean = moveToward(driveLean, targetdriveLean, leanRampNow);
  turnBias  = moveToward(turnBias,  targetturnBias,  TURN_RAMP);

  pid.T = dt;
  float setpoint = BASE_OFFSET + driveLean;
  float pidOut   = PIDController_Update(&pid, setpoint, angle);
  float uOut     = OUTPUT_SIGN * pidOut;

  float yawCorrection = 0.0f;
  if (fabs(targetturnBias) < 0.1f && fabs(turnBias) < 0.1f)
    yawCorrection = getEncoderYawCorrection(driveLean);

  float uL = uOut + turnBias + yawCorrection;
  float uR = uOut - turnBias - yawCorrection;

  if (armed && fabs(angle) < FALL_LIMIT) {
    driveLR(uL, uR);
  } else {
    coast(); resetPID();
    seqStep = SEQ_IDLE;
    enterHoldMode();
    if (armed && fabs(angle) >= FALL_LIMIT) {
      armed = false;
      Serial.println("FALLEN - send ARM to re-arm");
    }
  }

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("seq=");   Serial.print((int)seqStep);
    Serial.print(" mode="); Serial.print((int)mode);
    Serial.print(" ang=");  Serial.print(angle, 1);
    Serial.print(" sp=");   Serial.print(setpoint, 2);
    Serial.print(" pos=");  Serial.print(posCm, 1);
    Serial.print(" pErr="); Serial.print(posErrorCm, 1);
    Serial.print(" head="); Serial.print(headingDeg, 1);
    Serial.print(" hErr="); Serial.print(headingErrorDeg, 1);
    Serial.print(" lean="); Serial.print(driveLean, 2);
    Serial.print(" u=");    Serial.print(uOut, 0);
    Serial.print(" yawC="); Serial.print(yawCorrection, 1);
    if (mode == MODE_HOLD) {
      if      (fabs(posErrorCm) <= POS_OPT_LIMIT_CM) Serial.print(" POS_OPT");
      else if (fabs(posErrorCm) <= POS_LIMIT_CM)     Serial.print(" POS_OK");
      else                                            Serial.print(" POS_OUT");
    }
    Serial.println();
  }
}

//  MAIN LOOP
void loop() {
  // ── Fixed-rate gate 
  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;   // busy-wait; BLE.central() not called here
  lastLoop += LOOP_US;                    // += keeps rate exact, avoids drift

  float dt = (now - lastLoopUs) * 1e-6f;
  lastLoopUs = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / LOOP_HZ;   // clamp on first tick

  BLE.poll();
  BLEDevice central = BLE.central();

  if (central && central.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (customCharacteristic.written()) {
      int len = customCharacteristic.valueLength();
      if (len > BUFFER_SIZE) len = BUFFER_SIZE;
      const unsigned char *d = customCharacteristic.value();
      char buf[BUFFER_SIZE + 1];
      memcpy(buf, d, len); buf[len] = '\0';
      handleCommand(String(buf));
      customCharacteristic.writeValue("ok");
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    if (s.length()) handleCommand(s);
  }

  controlStep(dt);
  updateSequence();
}
