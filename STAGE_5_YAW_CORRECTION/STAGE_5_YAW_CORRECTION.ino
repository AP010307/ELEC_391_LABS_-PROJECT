#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

#define BUFFER_SIZE 20
BLEService        customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead|BLEWrite|BLENotify, BUFFER_SIZE, false);

// =================== PIDController (uploaded implementation) ===================
typedef struct {
  float Kp, Ki, Kd, tau;
  float limMin, limMax, limMinInt, limMaxInt, T;
  float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void PIDController_Init(PIDController *pid){
  pid->integrator=0; pid->prevError=0; pid->differentiator=0;
  pid->prevMeasurement=0; pid->out=0;
}
float PIDController_Update(PIDController *pid, float setpoint, float measurement){
  float error = setpoint - measurement;
  float proportional = pid->Kp * error;
  pid->integrator += 0.5f*pid->Ki*pid->T*(error + pid->prevError);

  if(pid->integrator > pid->limMaxInt) {
    pid->integrator = pid->limMaxInt;
  }
  else if(pid->integrator < pid->limMinInt) {
    pid->integrator = pid->limMinInt;
  }

  // Slowly bleed off integrator to prevent long-term drift
  // pid->integrator *= 0.990f;
  pid->differentiator = -(2.0f*pid->Kd*(measurement - pid->prevMeasurement)
                        + (2.0f*pid->tau - pid->T)*pid->differentiator)
                        / (2.0f*pid->tau + pid->T);
  pid->out = proportional + pid->integrator + pid->differentiator;
  if(pid->out > pid->limMax)      pid->out = pid->limMax;
  else if(pid->out < pid->limMin) pid->out = pid->limMin;
  pid->prevError = error;
  pid->prevMeasurement = measurement;
  return pid->out;
}
// ==============================================================================

const int LEFT_IN1=6, LEFT_IN2=9, RIGHT_IN1=3, RIGHT_IN2=5;
const int LEFT_ENC_A=7, LEFT_ENC_B=8, RIGHT_ENC_A=2, RIGHT_ENC_B=4;
volatile long encL=0, encR=0;
long prevEncL = 0;
long prevEncR = 0;

float yawTrim = 0.0f;

const float YAW_KP_FWD  = 0.06f;   // stronger correction while drifting/moving forward
const float YAW_KP_BACK = 0.02f;   // weaker correction while drifting/moving backward
const float YAW_TRIM_MAX = 10.0f;


void isrLeft()  { if(digitalRead(LEFT_ENC_B))  encL++; else encL--; }
void isrRight() { if(digitalRead(RIGHT_ENC_B)) encR++; else encR--; }

// ===== timing / filter =====
const float        LOOP_HZ=200.0;
const unsigned long LOOP_US=(unsigned long)(1e6/LOOP_HZ);
const float        TAU=0.75;
unsigned long lastLoopUs=0;
float angle=0, gyroBias=0, ax=0,ay=0,az=1,gx=0,gy=0,gz=0;

// ===== motor layer =====
const int   DB_L=17, DB_R=16;
const float U_MAX=255.0, U_DEADZONE=2.0;
const int   FWD_SIGN_L=-1, FWD_SIGN_R=+1;

// ===== HARDCODED control config =====
const float BASE_OFFSET = 0.;     // validated balance setpoint (deg)
const int   OUTPUT_SIGN = -1;       // verified
const float FALL_LIMIT  = 35.0;
const float I_MAX       = 30.0;
const float DERIV_TAU   = 0.02;

// ===== drive command biases (set by handleCommand) =====
const float FWD_LEAN_FWD = 0.4;
const float FWD_LEAN_BACK = -0.5;
const float LEFTTURN     = 5.0;        // differential bias for steering (tune)
const float RIGHTTURN     = 6.0;        // differential bias for steering (tune)
const int   LEAN_SIGN = +1;         // flip if FORWARD drives backward
const int   TURN_SIGN = +1;         // flip if LEFT/RIGHT are swapped
float targetdriveLean = 0.0;
float targetturnBias  = 0.0;

float driveLean = 0.0;
float turnBias = 0.0;

const float LEAN_RAMP = 0.005;
const float TURN_RAMP = 0.25;

const float LEFT_MOTOR_SCALE  = 0.92f;  // reduce left motor
const float RIGHT_MOTOR_SCALE = 1.00f;

PIDController pid;
bool armed=true;                    // auto-armed after calibration
unsigned long lastPrint=0;

float accelAngleDeg(float x,float y,float z){ return atan2(y,z)*180.0/PI; }
float gyroPitchRate(float rx,float ry,float rz){ return -rx; }

void setMotorRaw(int in1,int in2,int cmd){
  cmd=constrain(cmd,-255,255);
  if(cmd>=0){ analogWrite(in1,255); analogWrite(in2,255-cmd); }
  else      { analogWrite(in2,255); analogWrite(in1,255+cmd); }
}
int compensate(float u,int db){
  float a=fabs(u); if(a<U_DEADZONE) return 0;
  int out=(int)(db + (a/U_MAX)*(255.0-db) + 0.5);
  out=constrain(out,0,255); return (u>=0)?out:-out;
}
void driveLR(float uL,float uR){
  // setMotorRaw(LEFT_IN1,LEFT_IN2,  compensate(FWD_SIGN_L*uL,DB_L));
  // setMotorRaw(RIGHT_IN1,RIGHT_IN2, compensate(FWD_SIGN_R*uR,DB_R));
  uL *= LEFT_MOTOR_SCALE;
  uR *= RIGHT_MOTOR_SCALE;

  setMotorRaw(LEFT_IN1, LEFT_IN2, compensate(FWD_SIGN_L * uL, DB_L));
  setMotorRaw(RIGHT_IN1, RIGHT_IN2, compensate(FWD_SIGN_R * uR, DB_R));


}
void coast(){ analogWrite(LEFT_IN1,0);analogWrite(LEFT_IN2,0);
              analogWrite(RIGHT_IN1,0);analogWrite(RIGHT_IN2,0); }
void resetPID(){ pid.integrator=0; pid.differentiator=0;
                 pid.prevError=0; pid.prevMeasurement=angle; }

// ---------- command handling (BLE or serial) ----------
void handleCommand(String c){
  c.trim(); 
  c.toUpperCase();

  if(c=="FORWARD"||c=="F"||c=="UP") {
    targetdriveLean = LEAN_SIGN * FWD_LEAN_FWD;
    targetturnBias = 0;
  }
  else if(c=="BACKWARD"||c=="B"||c=="DOWN") {
    targetdriveLean = LEAN_SIGN * FWD_LEAN_BACK;
    targetturnBias = 0;
  }
  else if(c=="LEFT"||c=="L") {
    targetdriveLean = 0;
    targetturnBias = -TURN_SIGN * LEFTTURN;
  }
  else if(c=="RIGHT"||c=="R") {
    targetdriveLean = 0;
    targetturnBias = TURN_SIGN * RIGHTTURN;
  }
  else if(c=="STOP"||c=="S"||c=="A") {
    targetdriveLean = 0;
    targetturnBias = 0;
    driveLean = 0;
    turnBias = 0;
    // resetPID();
  }
  else {
    targetdriveLean = 0;
    targetturnBias = 0;
    driveLean = 0;
    turnBias = 0;
    resetPID();
  }

  Serial.print("cmd: "); 
  Serial.println(c);
}

float getEncoderYawCorrection(float movementCommand) {
  long currentL, currentR;

  noInterrupts();
  currentL = encL;
  currentR = encR;
  interrupts();

  long dL = currentL - prevEncL;
  long dR = currentR - prevEncR;

  prevEncL = currentL;
  prevEncR = currentR;

  long diff = dL - dR;
  Serial.print(" dL=");
  Serial.print(dL);
  Serial.print(" dR=");
  Serial.print(dR);
  Serial.print(" diff=");
  Serial.println(diff);

  if (abs(diff) <= 1) {
    return 0.0f;
  }

  float kpYaw;

  if (movementCommand > 0.05f) {
    kpYaw = YAW_KP_FWD;
  } else if (movementCommand < -0.05f) {
    kpYaw = YAW_KP_BACK;
  } else {
    kpYaw = 0.03f;   // standing/balancing-in-place
  }

  float correction = kpYaw * diff;
  correction = constrain(correction, -YAW_TRIM_MAX, YAW_TRIM_MAX);

  return correction;
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<3000){}
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(LEFT_IN1,OUTPUT);pinMode(LEFT_IN2,OUTPUT);
  pinMode(RIGHT_IN1,OUTPUT);pinMode(RIGHT_IN2,OUTPUT); //coast();
  pinMode(LEFT_ENC_A,INPUT);pinMode(LEFT_ENC_B,INPUT);
  pinMode(RIGHT_ENC_A,INPUT);pinMode(RIGHT_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),isrLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),isrRight,RISING);

  if(!IMU.begin()){ Serial.println("IMU FAIL"); while(1){} }

  // gyro bias cal - hold still
  Serial.println("Calibrating gyro bias - hold STILL...");
  delay(500);
  const int N=500; float sum=0; int got=0;
  while(got<N){ if(IMU.gyroscopeAvailable()){ float a,b,c;
    IMU.readGyroscope(a,b,c); sum+=gyroPitchRate(a,b,c); got++; } }
  gyroBias=sum/N;
  while(!IMU.accelerationAvailable()){} IMU.readAcceleration(ax,ay,az);
  angle=accelAngleDeg(ax,ay,az);

  // hardcoded PID
  PIDController_Init(&pid);
  pid.Kp=7; pid.Ki=100.0; pid.Kd=1.1; pid.tau=DERIV_TAU;
  pid.limMin=-U_MAX; pid.limMax=U_MAX;
  pid.limMinInt=-I_MAX; pid.limMaxInt=I_MAX;
  resetPID();

  // BLE
  if(!BLE.begin()){ Serial.println("BLE FAIL"); while(1){} }
  BLE.setLocalName("BLE-DEVICE"); BLE.setDeviceName("BLE-DEVICE");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("ready");
  BLE.advertise();

  Serial.println("ARMED + balancing. BLE advertising. Cmds: F B L R S (Z=kill)");
  lastLoopUs=micros();
}

void controlStep(){
  unsigned long now=micros();
  if(now-lastLoopUs < LOOP_US) return;
  float dt=(now-lastLoopUs)*1e-6;
  lastLoopUs=now;

  IMU.readAcceleration(ax,ay,az);
  IMU.readGyroscope(gx,gy,gz);
  float accAng=accelAngleDeg(ax,ay,az);
  float rate=gyroPitchRate(gx,gy,gz)-gyroBias;
  float k=TAU/(TAU+dt);
  angle=k*(angle+rate*dt)+(1.0-k)*accAng;

  pid.T=dt;
  driveLean = moveToward(driveLean, targetdriveLean, LEAN_RAMP);
  turnBias  = moveToward(turnBias,  targetturnBias,  TURN_RAMP);
  float setpoint = BASE_OFFSET + driveLean;        // forward/back = lean setpoint
  float pidOut = PIDController_Update(&pid, setpoint, angle);
  float uOut = OUTPUT_SIGN*pidOut;

  float yawCorrection = 0.0f;

  if (fabs(targetturnBias) < 0.1f && fabs(turnBias) < 0.1f) {
    yawCorrection = getEncoderYawCorrection(driveLean);
  }

  float uL = uOut + turnBias + yawCorrection;
  float uR = uOut - turnBias - yawCorrection;

  if(armed && fabs(angle)<FALL_LIMIT){
    driveLR(uL,uR);
  } else {
    coast(); resetPID();
    driveLean=0; turnBias=0;                        // recover to balance-in-place
  }


  if(millis()-lastPrint>=100){
    lastPrint=millis();
    Serial.print("ang=");Serial.print(angle,1);
    Serial.print(" sp=");Serial.print(setpoint,1);
    Serial.print(" u=");Serial.print(uOut,0);
    Serial.print(" turn=");Serial.print(turnBias,0);
    Serial.print(" yawC=");
    Serial.print(yawCorrection, 1);
    Serial.print(armed?" [ARMED]":" [KILL]");
    Serial.println();
  }
}

float moveToward(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}


void loop(){
  // ---- non-blocking BLE servicing ----
  BLEDevice central = BLE.central();              // polls radio, returns immediately
  if(central && central.connected()){
    digitalWrite(LED_BUILTIN,HIGH);
    if(customCharacteristic.written()){
      int len=customCharacteristic.valueLength();
      const unsigned char* d=customCharacteristic.value();
      char buf[len+1]; memcpy(buf,d,len); buf[len]='\0';
      handleCommand(String(buf));
      customCharacteristic.writeValue("ok");
    }
  } else {
    digitalWrite(LED_BUILTIN,LOW);
  }

  // ---- serial test input (type commands without the app) ----
  if(Serial.available()){
    String s=Serial.readStringUntil('\n');
    if(s.length()) handleCommand(s);
  }

  // ---- balance loop runs EVERY iteration (fixed-rate, self-gating) ----
  controlStep();
}

