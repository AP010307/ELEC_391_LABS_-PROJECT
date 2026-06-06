#include <Arduino.h>
#include "motor.h"

static const int LEFT_IN1    = 6,  LEFT_IN2    = 9;
static const int RIGHT_IN1   = 3,  RIGHT_IN2   = 5;
static const int LEFT_ENC_A  = 7,  LEFT_ENC_B  = 8;
static const int RIGHT_ENC_A = 2,  RIGHT_ENC_B = 4;

static const int   DB_L       = 17;
static const int   DB_R       = 16;
static const float U_MAX      = 255.0f;
static const float U_DEADZONE = 2.0f;
static const int   FWD_SIGN_L = -1;
static const int   FWD_SIGN_R = +1;

static volatile long s_encL = 0;
static volatile long s_encR = 0;

static void isrLeft()  { s_encL += digitalRead(LEFT_ENC_B)  ? 1 : -1; }
static void isrRight() { s_encR += digitalRead(RIGHT_ENC_B) ? 1 : -1; }

static void setMotorRaw(int in1, int in2, int cmd) {
    cmd = constrain(cmd, -255, 255);
    if (cmd >= 0) {
        analogWrite(in1, 255);
        analogWrite(in2, 255 - cmd);
    } else {
        analogWrite(in2, 255);
        analogWrite(in1, 255 + cmd);
    }
}

static int compensate(float u, int db) {
    float a = fabsf(u);
    if (a < U_DEADZONE) return 0;
    int out = (int)(db + (a / U_MAX) * (255.0f - (float)db) + 0.5f);
    out = constrain(out, 0, 255);
    return (u >= 0.0f) ? out : -out;
}

void motor_init() {
    pinMode(LEFT_IN1,    OUTPUT); pinMode(LEFT_IN2,    OUTPUT);
    pinMode(RIGHT_IN1,   OUTPUT); pinMode(RIGHT_IN2,   OUTPUT);
    pinMode(LEFT_ENC_A,  INPUT);  pinMode(LEFT_ENC_B,  INPUT);
    pinMode(RIGHT_ENC_A, INPUT);  pinMode(RIGHT_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isrRight, RISING);
    motor_coast();
}

void motor_coast() {
    analogWrite(LEFT_IN1,  0); analogWrite(LEFT_IN2,  0);
    analogWrite(RIGHT_IN1, 0); analogWrite(RIGHT_IN2, 0);
}

void motor_driveLR(float uL, float uR) {
    setMotorRaw(LEFT_IN1,  LEFT_IN2,  compensate((float)FWD_SIGN_L * uL, DB_L));
    setMotorRaw(RIGHT_IN1, RIGHT_IN2, compensate((float)FWD_SIGN_R * uR, DB_R));
}

long motor_getEncL() { return s_encL; }
long motor_getEncR() { return s_encR; }
