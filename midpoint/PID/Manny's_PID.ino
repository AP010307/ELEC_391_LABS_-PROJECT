// ============================================================
//  ELEC 391 — Self-Balancing Robot
//  IMU + Complementary Filter + PID controller
//
//  Board:  Arduino Nano 33 BLE Sense Rev2
//  IMU:    BMI270 (via Arduino_BMI270_BMM150 library)
// ============================================================

#include <Arduino_BMI270_BMM150.h>

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
//  MOTOR PARAMETERS
// ============================================================
const int LEFT_DB  = 42;
const int RIGHT_DB = 41;
const int MAX_PWM  = 255;

// ============================================================
//  IMU / FILTER PARAMETERS
// ============================================================
const float GYRO_WEIGHT  = 0.95;
const float ACCEL_WEIGHT = 1.0 - GYRO_WEIGHT;
const float TILT_DEADZONE = 2.0;   // degrees — motors off within this of target

// ============================================================
//  PID GAINS  (tune Kp first, then Kd, then Ki)
// ============================================================
float Kp = 15.0;
float Ki = 0.0;
float Kd = 1.0;

float target_angle = -2.0;  // adjust ±1-2° to match physical balance point

// ============================================================
//  PID STATE  (globals — never redeclare inside pid())
// ============================================================
float integral   = 0.0;
float prev_error = 0.0;
const float INTEGRAL_CLAMP = 50.0;

// ============================================================
//  COMPLEMENTARY FILTER STATE
// ============================================================
float roll_angle   = 0.0;
unsigned long prev_time = 0;
bool firstLoop = true;

// ============================================================
//  ENCODER COUNTS
// ============================================================
volatile long leftCount  = 0;
volatile long rightCount = 0;

// ============================================================
//  ENCODER ISRs
// ============================================================
void leftA_ISR()  { if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount--; else leftCount++; }
void leftB_ISR()  { if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount++; else leftCount--; }
void rightA_ISR() { if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount--; else rightCount++; }
void rightB_ISR() { if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount++; else rightCount--; }

// ============================================================
//  MOTOR DRIVER
//  Matches reference: left = -pwm, right = +pwm for same direction
//  Deadband added to magnitude before sign is applied
// ============================================================
void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void stopMotors() {
  analogWrite(LEFT_IN1, 0);  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0); analogWrite(RIGHT_IN2, 0);
}

// ============================================================
//  TILT ANGLE — complementary filter (matches reference structure)
// ============================================================
float readTilt() {
  float ax, ay, az, gx, gy, gz;

  if (!IMU.accelerationAvailable()) return roll_angle;
  IMU.readGyroscope(gx, gy, gz);
  IMU.readAcceleration(ax, ay, az);

  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0;
  prev_time = now;

  if (firstLoop || dt <= 0 || dt > 0.1) {
    firstLoop = false;
    float accel_roll = atan2(ay, az) * 180.0 / PI;
    roll_angle = accel_roll;   // seed on first valid read
    return roll_angle;
  }

  float accel_roll = atan2(ay, az) * 180.0 / PI;
  float gyro_roll  = roll_angle + gx * dt;
  roll_angle = GYRO_WEIGHT * gyro_roll + ACCEL_WEIGHT * accel_roll;

  return roll_angle;
}

// ============================================================
//  PID CONTROLLER
// ============================================================
float pid(float error, float dt) {
  if (dt <= 0) return 0;

  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  float I = Ki * integral;

  float D = Kd * (error - prev_error) / dt;
  prev_error = error;

  return P + I + D;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Motor pins
  pinMode(LEFT_IN1,  OUTPUT); pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  // Encoder pins + ISRs
  pinMode(LEFT_ENC_A,  INPUT); pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT); pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  // IMU init
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }

  prev_time = millis();
  Serial.println("Ready — Serial Plotter: Tilt | Error | PWM");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0;  // used by pid()

  float tilt  = readTilt();   // prev_time updated inside readTilt
  float error = tilt - target_angle;

  // Tilt deadzone — stop motors when close enough to upright
  if (abs(error) < TILT_DEADZONE) {
    stopMotors();
    Serial.print("Tilt:"); Serial.print(tilt, 2); Serial.print("\t");
    Serial.print("Error:"); Serial.print(error, 2); Serial.print("\t");
    Serial.println("PWM:0");
    return;
  }

  float output = pid(error, dt);

  // Add deadband offset to magnitude so motors respond to small corrections
  int mag = abs((int)output);
  if (mag > 0) mag += LEFT_DB;
  mag = constrain(mag, 0, MAX_PWM);
  int pwm = (output >= 0) ? mag : -mag;

  // Left motor inverted relative to right — matches reference
  setMotor(LEFT_IN1,  LEFT_IN2,  -pwm);
  setMotor(RIGHT_IN1, RIGHT_IN2,  pwm);

  // Serial Plotter
  Serial.print("Tilt:");  Serial.print(tilt, 2);  Serial.print("\t");
  Serial.print("Error:"); Serial.print(error, 2); Serial.print("\t");
  Serial.print("PWM:");   Serial.println(pwm);
}
