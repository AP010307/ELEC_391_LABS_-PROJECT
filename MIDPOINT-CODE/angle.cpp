#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>
#include "angle.h"

static const float TAU = 0.75f;

static float s_angle    = 0.0f;
static float s_gyroBias = 0.0f;

static float accelAngleDeg(float x, float y, float z) {
    return atan2f(y, z) * 180.0f / (float)M_PI;
}

static float gyroPitchRate(float rx, float ry, float rz) {
    (void)ry; (void)rz;
    return -rx; /* should be negative sign */
}

bool angle_init() {
    return IMU.begin();
}

void angle_calibrate() {
    const int N   = 500;
    float     sum = 0.0f;
    int       got = 0;

    while (got < N) {
        if (IMU.gyroscopeAvailable()) {
            float rx, ry, rz;
            IMU.readGyroscope(rx, ry, rz);
            sum += gyroPitchRate(rx, ry, rz);
            got++;
        }
    }
    s_gyroBias = sum / (float)N;

    while (!IMU.accelerationAvailable()) {}
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    s_angle = accelAngleDeg(ax, ay, az);
}

float angle_update(float dt) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float accAng = accelAngleDeg(ax, ay, az);
    float rate   = gyroPitchRate(gx, gy, gz) - s_gyroBias;
    float k      = TAU / (TAU + dt);

    s_angle = k * (s_angle + rate * dt) + (1.0f - k) * accAng;
    return s_angle;
}

float angle_get() {
    return s_angle;
}
