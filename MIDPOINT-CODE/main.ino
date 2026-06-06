#include <Arduino.h>
#include "pid.h"
#include "angle.h"
#include "motor.h"
#include "ble.h"

/* Control configuration */
static const float BASE_OFFSET  = -0.8f;
static const int   OUTPUT_SIGN  = -1;
static const float FALL_LIMIT   = 35.0f;
static const float I_MAX        = 30.0f;
static const float DERIV_TAU    = 0.02f;

/* Drive command parameters */
static const float FWD_LEAN_FWD  =  0.4f;
static const float FWD_LEAN_BACK = -0.5f;
static const float LEFTTURN      =  5.0f;
static const float RIGHTTURN     =  6.0f;
static const float LEAN_RAMP     =  0.005f;
static const float TURN_RAMP     =  0.25f;

/* Loop timing */
static const float        LOOP_HZ = 200.0f;
static const unsigned long LOOP_US = (unsigned long)(1e6f / LOOP_HZ);

static PIDController  pid;
static unsigned long  lastLoopUs = 0;
static unsigned long  lastPrintMs = 0;

static float targetdriveLean = 0.0f;
static float targetturnBias  = 0.0f;
static float driveLean       = 0.0f;
static float turnBias        = 0.0f;

static float moveToward(float current, float target, float step) {
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

static void resetPID() {
    pid.integrator      = 0.0f;
    pid.differentiator  = 0.0f;
    pid.prevError       = 0.0f;
    pid.prevMeasurement = angle_get();
}

static void handleCommand(const char *cmd) {
    char upper[32];
    int i = 0;
    while (cmd[i] && i < (int)sizeof(upper) - 1) {
        upper[i] = toupper((unsigned char)cmd[i]);
        i++;
    }
    upper[i] = '\0';

    /* Trim leading/trailing whitespace is not performed here;
       caller should ensure clean input if needed. */

    if (strcmp(upper, "FORWARD") == 0 || strcmp(upper, "F") == 0 ||
        strcmp(upper, "UP") == 0) {
        targetdriveLean =  FWD_LEAN_FWD;
        targetturnBias  =  0.0f;
    } else if (strcmp(upper, "BACKWARD") == 0 || strcmp(upper, "B") == 0 ||
               strcmp(upper, "DOWN") == 0) {
        targetdriveLean =  FWD_LEAN_BACK;
        targetturnBias  =  0.0f;
    } else if (strcmp(upper, "LEFT") == 0 || strcmp(upper, "L") == 0) {
        targetdriveLean =  0.0f;
        targetturnBias  = -LEFTTURN;
    } else if (strcmp(upper, "RIGHT") == 0 || strcmp(upper, "R") == 0) {
        targetdriveLean =  0.0f;
        targetturnBias  =  RIGHTTURN;
    } else if (strcmp(upper, "STOP") == 0 || strcmp(upper, "S") == 0 ||
               strcmp(upper, "A") == 0) {
        targetdriveLean = 0.0f;
        targetturnBias  = 0.0f;
        driveLean       = 0.0f;
        turnBias        = 0.0f;
    }

    Serial.print("cmd: ");
    Serial.println(upper);
}

static void controlStep() {
    unsigned long now = micros();
    if (now - lastLoopUs < LOOP_US) return;

    float dt    = (float)(now - lastLoopUs) * 1e-6f;
    lastLoopUs  = now;

    float angle = angle_update(dt);

    driveLean = moveToward(driveLean, targetdriveLean, LEAN_RAMP);
    turnBias  = moveToward(turnBias,  targetturnBias,  TURN_RAMP);

    float setpoint = BASE_OFFSET + driveLean;
    pid.T          = dt;

    float pidOut = PIDController_Update(&pid, setpoint, angle);
    float uOut   = (float)OUTPUT_SIGN * pidOut;

    float uL = uOut + turnBias;
    float uR = uOut - turnBias;

    if (fabsf(angle) < FALL_LIMIT) {
        motor_driveLR(uL, uR);
    } else {
        motor_coast();
        resetPID();
        driveLean = 0.0f;
        turnBias  = 0.0f;
    }

    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        Serial.print("ang=");    Serial.print(angle, 1);
        Serial.print(" sp=");    Serial.print(setpoint, 1);
        Serial.print(" u=");     Serial.print(uOut, 0);
        Serial.print(" turn=");  Serial.println(turnBias, 0);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    pinMode(LED_BUILTIN, OUTPUT);
    motor_init();

    if (!angle_init()) {
        Serial.println("IMU FAIL");
        while (1) {}
    }

    Serial.println("Calibrating - hold still...");
    delay(500);
    angle_calibrate();

    PIDController_Init(&pid);
    pid.Kp        = 5.0f;
    pid.Ki        = 40.0f;
    pid.Kd        = 1.0f;
    pid.tau       = DERIV_TAU;
    pid.limMin    = -255.0f;
    pid.limMax    =  255.0f;
    pid.limMinInt = -I_MAX;
    pid.limMaxInt =  I_MAX;
    resetPID();

    if (!ble_init(handleCommand)) {
        Serial.println("BLE FAIL");
        while (1) {}
    }

    Serial.println("Running. Cmds: F B L R S");
    lastLoopUs = micros();
}

void loop() {
    ble_service();

    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        if (s.length()) handleCommand(s.c_str());
    }

    controlStep();
}
