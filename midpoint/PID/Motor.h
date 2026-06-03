#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Motor pin setup
void Motor_Init();

// Drive both motors
void Motor_Drive(int leftSpeed, int rightSpeed);

// Stop both motors
void Motor_Stop();

// Individual motor helper
void Motor_Set(int in1Pin, int in2Pin, int speed);

#endif