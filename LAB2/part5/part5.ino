
#include "Arduino_BMI270_BMM150.h"

// ---------- PIN ASSIGNMENTS ----------
// EDIT to match your wiring
const int LEFT_ENC_A  = 7;
const int LEFT_ENC_B  = 8;
const int RIGHT_ENC_A = 2;
const int RIGHT_ENC_B = 4;

const int LEFT_IN1  = 6;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 5;

// ---------- DEADBAND (from Part 1a) ----------
const int LEFT_DEADBAND  = 41;   // EDIT with your measured value
const int RIGHT_DEADBAND = 40;   // EDIT with your measured value
const int DEADBAND = min(LEFT_DEADBAND, RIGHT_DEADBAND);

// ---------- CONTROL PARAMETERS ----------
// Kp = PWM units per degree of tilt
// EDIT this to tune the response:
//   - Too small: motors barely respond to large tilts
//   - Too large: motors saturate at small tilts
//   - Start around 5 and adjust from there
const float KP = 10;

// Threshold below which we don't drive the motors (avoid jitter at 0 tilt)
const float TILT_DEADZONE_DEG = 1.0;   // EDIT if needed

// Max PWM (don't change unless you have a reason)
const int MAX_PWM = 255;

// ---------- ENCODER CONSTANTS ----------
const long COUNTS_PER_REV = 1920;

// ---------- ENCODER STATE ----------
volatile long leftCount  = 0;
volatile long rightCount = 0;
long lastLeftCount  = 0;
long lastRightCount = 0;
unsigned long lastRPMTime = 0;
float leftRPM  = 0.0;
float rightRPM = 0.0;


float gyro_weight = 0.95;
float accel_weight = 1.00 - gyro_weight;
float roll_angle = 0.0;
unsigned long previous_time = 0;


// ============================================================
//  ISRs
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
//  MOTOR HELPER (brake-mode PWM)
// ============================================================
void setMotor(int in1Pin, int in2Pin, int speed) {
  // Clamp to valid range
  if (speed >  MAX_PWM) speed =  MAX_PWM;
  if (speed < -MAX_PWM) speed = -MAX_PWM;

  if (speed >= 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  }
}

// ============================================================
//  TILT ANGLE FROM ACCELEROMETER
//  Returns tilt in degrees from vertical.
//  Sign convention: positive = tilt in one direction, negative = other.
//
//  NOTE: Which axes to use depends on how the Arduino is physically
//  mounted on the robot. Try atan2(ax, az) first. If it doesn't
//  respond to your tilt direction, try atan2(ay, az) instead.
// ============================================================
float readTiltAngle() {
  float ax, ay, az;
  float gx, gy, gz;
  // float roll_angle = 0.0;

  if (IMU.accelerationAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    IMU.readAcceleration(ax, ay, az);

    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0; // seconds
    previous_time = current_time;

    float accel_roll = atan2(ay, az) * 180.0 / PI;
    float gyro_roll = roll_angle + gx * dt;

    roll_angle  = gyro_weight * gyro_roll + accel_weight * accel_roll;

  
  }

    return roll_angle;
}

// ============================================================
//  RPM CALCULATION (non-blocking - updated every ~200 ms)
// ============================================================
void updateRPM() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRPMTime;

  if (elapsed >= 200) {   // update every 200 ms
    noInterrupts();
    long curLeft  = leftCount;
    long curRight = rightCount;
    interrupts();

    float dt = elapsed / 1000.0;
    leftRPM  = ((curLeft  - lastLeftCount)  / dt) * 60.0 / COUNTS_PER_REV;
    rightRPM = ((curRight - lastRightCount) / dt) * 60.0 / COUNTS_PER_REV;

    lastLeftCount  = curLeft;
    lastRightCount = curRight;
    lastRPMTime    = now;
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU!");
    while (1);   // halt
  }
  Serial.print("Accelerometer sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  // Encoder pins
  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  // Motor pins
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  setMotor(LEFT_IN1,  LEFT_IN2,  0);
  setMotor(RIGHT_IN1, RIGHT_IN2, 0);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  lastRPMTime = millis();

  Serial.println("=== Part 5: Proportional Tilt Control ===");
  Serial.print("Deadband: "); Serial.println(DEADBAND);
  Serial.print("Kp: ");       Serial.println(KP);
  Serial.println("Tilt the robot to drive motors. Tilt = 0 -> stopped.");
  Serial.println();
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  // 1. Read tilt
  float tilt = readTiltAngle();

  // 2. Compute proportional command
  float command = KP * tilt;   // raw signed PWM-equivalent

  // 3. Apply deadband compensation and dead zone
  int pwm = 0;
  if (abs(tilt) > TILT_DEADZONE_DEG) {
    // Magnitude: deadband + proportional part
    int mag = DEADBAND + abs((int)command);
    if (mag > MAX_PWM) mag = MAX_PWM;
    pwm = (command >= 0) ? mag : -mag;
  }

  // 4. Drive both motors in the same direction
  setMotor(LEFT_IN1,  LEFT_IN2,  -pwm);
  setMotor(RIGHT_IN1, RIGHT_IN2, pwm);

  // 5. Update RPM measurements
  updateRPM();

  // 6. Print status (every loop, ~50 ms cycle)
  Serial.print("Tilt: ");    Serial.print(tilt, 1);
  Serial.print(" deg");
  Serial.print("   PWM: ");  Serial.print(pwm);
  Serial.print("   L RPM: "); Serial.print(leftRPM, 1);
  Serial.print("   R RPM: "); Serial.println(rightRPM, 1);

  // delay(20);   // 20 Hz control loop
}
