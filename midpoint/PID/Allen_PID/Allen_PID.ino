#include <Arduino_BMI270_BMM150.h>
#include "PID.h"
#include "Motor.h"

// ============================================================
//  ENCODER PIN ASSIGNMENTS
// ============================================================
const int LEFT_ENC_A  = 7;
const int LEFT_ENC_B  = 8;
const int RIGHT_ENC_A = 2;
const int RIGHT_ENC_B = 4;

// ============================================================
//  MOTOR PARAMETERS
// ============================================================
const int LEFT_DB  = 17;
const int RIGHT_DB = 16;
const int MAX_PWM  = 255;

// ============================================================
//  IMU / FILTER PARAMETERS
// ============================================================
const float GYRO_WEIGHT  = 0.95f;
const float ACCEL_WEIGHT = 1.0f - GYRO_WEIGHT;
const float TILT_DEADZONE = 2.0f;

// ============================================================
//  BALANCE VARIABLES
// ============================================================
float target_angle = 0.0f;
float roll_angle = 0.0f;

// Encoder counts
volatile long leftCount = 0;
volatile long rightCount = 0;

// ============================================================
//  BALANCE PID
// ============================================================
PIDController balancePID = {
  10.0f,    // Kp
  0.0f,     // Ki
  0.4f,     // Kd

  0.02f,    // tau derivative filter

  -255.0f,  // output min
   255.0f,  // output max

  -50.0f,   // integrator min
   50.0f,   // integrator max

  0.01f     // T = 10 ms = 100 Hz
};

// ============================================================
//  ENCODER ISRs
// ============================================================
void leftA_ISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount--;
  else leftCount++;
}

void leftB_ISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount++;
  else leftCount--;
}

void rightA_ISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount--;
  else rightCount++;
}

void rightB_ISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount++;
  else rightCount--;
}

// ============================================================
//  READ TILT ANGLE
// ============================================================
float readTilt(float dt) {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float accel_roll = atan2(ay, az) * 180.0f / PI;

    // Depending on IMU orientation, gx may need to be gy or gz.
    float gyro_roll = roll_angle + gx * dt;

    roll_angle = GYRO_WEIGHT * gyro_roll + ACCEL_WEIGHT * accel_roll;
  }

  return roll_angle;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Motor_Init();

  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }

  PIDController_Init(&balancePID);

  Serial.println("Ready — Serial Plotter: Tilt | PID Output | PWM");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  static unsigned long lastControlTime = 0;
  unsigned long now = micros();

  // Run at fixed 100 Hz
  if (now - lastControlTime < 10000) {
    return;
  }

  lastControlTime = now;

  float tilt = readTilt(balancePID.T);

  float output = PIDController_Update(&balancePID, target_angle, tilt);

  // If motor direction is wrong, flip this:
  // output = -output;

  int pwm = 0;

  if (fabs(tilt - target_angle) > TILT_DEADZONE) {
    int mag = abs((int)output);

    // Add motor deadband compensation only if command is nonzero
    if (mag > 0) {
      mag += max(LEFT_DB, RIGHT_DB);
    }

    if (mag > MAX_PWM) {
      mag = MAX_PWM;
    }

    pwm = (output >= 0) ? mag : -mag;
  }

  Motor_Drive(pwm, -pwm);

  Serial.print("Tilt:");
  Serial.print(tilt, 2);

  Serial.print("\tPID:");
  Serial.print(output, 2);

  Serial.print("\tPWM:");
  Serial.println(pwm);
}