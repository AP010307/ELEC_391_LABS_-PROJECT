#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp, Ki, Kd, tau;
    float limMin, limMax;
    float limMinInt, limMaxInt;
    float T;
    float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif
