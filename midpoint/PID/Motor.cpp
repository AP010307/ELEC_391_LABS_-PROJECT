#include "Motor.h"

// ============================================================
//  PIN ASSIGNMENTS
// ============================================================
const int LEFT_IN1  = 6;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 5;

// ============================================================
//  MOTOR PARAMETERS
// ============================================================
const int MAX_PWM = 255;

void Motor_Init() {
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  Motor_Stop();
}

void Motor_Set(int in1Pin, int in2Pin, int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed > 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else if (speed < 0) {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
  }
}

void Motor_Drive(int leftSpeed, int rightSpeed) {
  Motor_Set(LEFT_IN1, LEFT_IN2, leftSpeed);
  Motor_Set(RIGHT_IN1, RIGHT_IN2, rightSpeed);
}

void Motor_Stop() {
  Motor_Drive(0, 0);
}x