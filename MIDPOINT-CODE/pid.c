#include "pid.h"

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

void PIDController_Init(PIDController *pid) {
    pid->integrator      = 0.0f;
    pid->prevError       = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out             = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error        = setpoint - measurement;
    float proportional = pid->Kp * error;

    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    pid->integrator  = clampf(pid->integrator, pid->limMinInt, pid->limMaxInt);

    /* Derivative on measurement avoids kick on setpoint changes */
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    pid->out = clampf(proportional + pid->integrator + pid->differentiator,
                      pid->limMin, pid->limMax);

    pid->prevError       = error;
    pid->prevMeasurement = measurement;
    return pid->out;
}
