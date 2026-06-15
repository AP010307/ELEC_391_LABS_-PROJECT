#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

#define BUFFER_SIZE 20
BLEService        customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead|BLEWrite|BLENotify, BUFFER_SIZE, false);

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
  if(pid->integrator > pid->limMaxInt)      pid->integrator = pid->limMaxInt;
  else if(pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;
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

const int LEFT_IN1=6, LEFT_IN2=9, RIGHT_IN1=3, RIGHT_IN2=5;
const int LEFT_ENC_A=7, LEFT_ENC_B=8, RIGHT_ENC_A=2, RIGHT_ENC_B=4;
volatile long encL=0, encR=0;
long prevEncL = 0;
long prevEncR = 0;
void isrLeft()  { if(digitalRead(LEFT_ENC_B))  encL++; else encL--; }
void isrRight() { if(digitalRead(RIGHT_ENC_B)) encR++; else encR--; }

//YAW ADJUSTMENTS
const float YAW_KP_FWD   = 0.06f;
const float YAW_KP_BACK  = 0.02f;
const float YAW_KP_STILL = 0.03f;
const float YAW_TRIM_MAX = 10.0f;

const float        TAU=0.75;
unsigned long lastLoopUs=0;
float angle=0, gyroBias=0, ax=0,ay=0,az=1,gx=0,gy=0,gz=0;

// gyro Z bias for yaw integration during turns
float gyroBiasZ=0;

const int   DB_L=17, DB_R=16;
const float U_MAX=255.0, U_DEADZONE=2.0;
const int   FWD_SIGN_L=-1, FWD_SIGN_R=+1;

const float BASE_OFFSET = 0.9;
const int   OUTPUT_SIGN = -1;
const float FALL_LIMIT  = 35.0;
const float I_MAX       = 30.0;
const float DERIV_TAU   = 0.02;


//TUNE FOR FORWARD AND BACKWARD COMMAND
const float FWD_LEAN_FWD  =  1.0;
const float FWD_LEAN_BACK = -1.0;

//TUNE FOR TURN SPEED
//NEED LESS OR EQUAL TO 2 SEC
const float TURNBIAS      =  2.5; 


const int   LEAN_SIGN     = +1;
const int   TURN_SIGN     = +1;

float driveLean = 0.0;
float turnBias  = 0.0;
float targetdriveLean = 0.0;
float targetturnBias  = 0.0;
const float LEAN_RAMP = 0.005;
const float TURN_RAMP = 0.25;

const float LEFT_MOTOR_SCALE  = 0.92f;
const float RIGHT_MOTOR_SCALE = 1.00f;

//MOVE TARGETS - DISTANCE
const float COUNTS_PER_CM = 73.74f; //FINE TUNE
const float MOVE_DIST_CM  = 50.0f;
const long  MOVE_TARGET_COUNTS = (long)(MOVE_DIST_CM * COUNTS_PER_CM);

//MOVE TARGETS - TURN ANGLE
const float TURN_TARGET_DEG    = 45.0f; //FINE TUNE 



// ===== move state machine =====
enum MoveState { IDLE, MOVING_FB, TURNING };
MoveState moveState = IDLE;
long  moveStartPos = 0;
float turnIntegral = 0.0f;   // integrated gyro-Z degrees during a turn

PIDController pid;
bool armed=true;
unsigned long lastPrint=0;

float accelAngleDeg(float x,float y,float z){ return atan2(y,z)*180.0/PI; }
float gyroPitchRate(float rx,float ry,float rz){ return -rx; }

// distance channel = (encL + encR)/2  (verified for this robot)
long readDist(){ 
  long l,r; 
  noInterrupts(); 
  l=encL; 
  r=encR; 
  interrupts(); 
  return (l + r)/2; 
  }

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
  uL *= LEFT_MOTOR_SCALE;
  uR *= RIGHT_MOTOR_SCALE;
  setMotorRaw(LEFT_IN1, LEFT_IN2, compensate(FWD_SIGN_L * uL, DB_L));
  setMotorRaw(RIGHT_IN1, RIGHT_IN2, compensate(FWD_SIGN_R * uR, DB_R));
}
void coast(){ analogWrite(LEFT_IN1,0);analogWrite(LEFT_IN2,0);
              analogWrite(RIGHT_IN1,0);analogWrite(RIGHT_IN2,0); }
void resetPID(){ pid.integrator=0; pid.differentiator=0;
                 pid.prevError=0; pid.prevMeasurement=angle; }

void enterPark(){
  moveState = IDLE;
  targetdriveLean = 0; targetturnBias = 0;
  driveLean = 0; turnBias = 0;
  noInterrupts(); 
  prevEncL = encL; 
  prevEncR = encR; interrupts();  // fresh yaw-trim baseline
}

void startForward(){
  moveStartPos = readDist();
  targetdriveLean = LEAN_SIGN * FWD_LEAN_FWD;
  targetturnBias = 0;
  moveState = MOVING_FB;
}
void startBackward(){
  moveStartPos = readDist();
  targetdriveLean = LEAN_SIGN * FWD_LEAN_BACK;
  targetturnBias = 0;
  moveState = MOVING_FB;
}
void startTurn(int dir){   // dir +1 = left (+Z), -1 = right (-Z)
  turnIntegral = 0.0f;
  targetturnBias = dir * TURN_SIGN * TURNBIAS;
  targetdriveLean = 0;
  moveState = TURNING;
}

void handleCommand(String c){
  c.trim(); c.toUpperCase();
  if(c=="FORWARD"||c=="F"||c=="UP")        startForward();
  else if(c=="BACKWARD"||c=="B"||c=="DOWN")startBackward();
  else if(c=="LEFT"||c=="L")               startTurn(+1);  // left = +Z
  else if(c=="RIGHT"||c=="R")              startTurn(-1);  // right = -Z
  else if(c=="STOP"||c=="S"||c=="A")       enterPark();
  else { enterPark(); resetPID(); }
  Serial.print("cmd: "); Serial.println(c);
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

  if (abs(diff) <= 1) return 0.0f;

  float kpYaw;

  if (movementCommand > 0.05f)       kpYaw = YAW_KP_FWD;
  else if (movementCommand < -0.05f) kpYaw = YAW_KP_BACK;
  else                               kpYaw = YAW_KP_STILL;

  float correction = kpYaw * diff;
  return constrain(correction, -YAW_TRIM_MAX, YAW_TRIM_MAX);
}

float moveToward(float current,float target,float step){
  if(current<target){ current+=step; if(current>target) current=target; }
  else if(current>target){ current-=step; if(current<target) current=target; }
  return current;
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<3000){}
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(LEFT_IN1,OUTPUT);pinMode(LEFT_IN2,OUTPUT);
  pinMode(RIGHT_IN1,OUTPUT);pinMode(RIGHT_IN2,OUTPUT);
  pinMode(LEFT_ENC_A,INPUT);pinMode(LEFT_ENC_B,INPUT);
  pinMode(RIGHT_ENC_A,INPUT);pinMode(RIGHT_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),isrLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),isrRight,RISING);

  if(!IMU.begin()){ Serial.println("IMU FAIL"); while(1){} }

  Serial.println("Calibrating gyro bias - hold STILL...");
  delay(500);
  const int N=500; float sumP=0, sumZ=0; int got=0;
  while(got<N){
    if(IMU.gyroscopeAvailable()){
      float a,b,c; IMU.readGyroscope(a,b,c);
      sumP += gyroPitchRate(a,b,c);   // pitch axis (negated rx) for balance
      sumZ += c;                      // z axis for yaw turns
      got++;
    }
  }
  gyroBias  = sumP/N;
  gyroBiasZ = sumZ/N;
  while(!IMU.accelerationAvailable()){} IMU.readAcceleration(ax,ay,az);
  angle=accelAngleDeg(ax,ay,az);

  PIDController_Init(&pid);

  pid.Kp=6.9; 
  pid.Ki=100.00; 
  pid.Kd=0.7; 
  
  pid.tau=DERIV_TAU;
  pid.limMin=-U_MAX;
  pid.limMax=U_MAX;
  pid.limMinInt=-I_MAX; 
  pid.limMaxInt=I_MAX;
  resetPID();

  noInterrupts(); 
  prevEncL = encL; 
  prevEncR = encR; 
  interrupts();

  if(!BLE.begin()){ Serial.println("BLE FAIL"); while(1){} }
  BLE.setLocalName("BLE-DEVICE"); BLE.setDeviceName("BLE-DEVICE");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("ready");
  BLE.advertise();

  Serial.println("ARMED. F=fwd50 B=back50 L=left45 R=right45 S=stop");
  lastLoopUs=micros();
}

void controlStep(){
  // data-gated: run once per fresh gyro sample (~100 Hz, clean dt)
  if(!IMU.gyroscopeAvailable()) return;

  unsigned long now=micros();
  float dt=(now-lastLoopUs)*1e-6;
  lastLoopUs=now;
  if(dt<=0.0f || dt>0.1f) dt=0.01f;

  IMU.readGyroscope(gx,gy,gz);

  if(IMU.accelerationAvailable()) IMU.readAcceleration(ax,ay,az);

  float accAng=accelAngleDeg(ax,ay,az);

  float rate=gyroPitchRate(gx,gy,gz)-gyroBias;

  float k=TAU/(TAU+dt);

  angle=k*(angle+rate*dt)+(1.0-k)*accAng;

  // ---- move completion checks ----
  if(moveState==MOVING_FB){

    long traveled = readDist() - moveStartPos;

    if(labs(traveled) >= MOVE_TARGET_COUNTS) enterPark();    // reached 50 cm

  } else if(moveState==TURNING){

    turnIntegral += (gz - gyroBiasZ) * dt;                 // integrate yaw deg

    if(fabs(turnIntegral) >= TURN_TARGET_DEG) enterPark();   // reached 45 deg
  }

  pid.T=dt;

  driveLean = moveToward(driveLean, targetdriveLean, LEAN_RAMP);
  turnBias  = moveToward(turnBias,  targetturnBias,  TURN_RAMP);

  // yaw heading trim only while parked (per-loop-delta, original method)
  float yawCorrection = 0.0f;
  if(moveState==IDLE){
    yawCorrection = getEncoderYawCorrection(driveLean);
  } else {
    noInterrupts(); 
    prevEncL = encL; 
    prevEncR = encR; 
    interrupts();  // keep baseline fresh during moves
  }

  float setpoint = BASE_OFFSET + driveLean;
  float pidOut = PIDController_Update(&pid, setpoint, angle);
  float uOut = OUTPUT_SIGN*pidOut;

  float uL = uOut + turnBias + yawCorrection;
  float uR = uOut - turnBias - yawCorrection;

  if(armed && fabs(angle)<FALL_LIMIT){
    driveLR(uL,uR);
  } else {
    coast(); 
    resetPID();
    enterPark();
  }

  if(millis()-lastPrint>=200){
    lastPrint=millis();
    const char* st = moveState==IDLE?"IDLE":(moveState==MOVING_FB?"MOVE":"TURN");
    Serial.print(st);
    Serial.print(" ang=");Serial.print(angle,1);
    Serial.print(" dist=");Serial.print(readDist()-moveStartPos);
    Serial.print(" tDeg=");Serial.print(turnIntegral,1);
    Serial.print(" yawC=");Serial.print(yawCorrection,1);
    Serial.print(" u=");Serial.print(uOut,0);
    Serial.println();
  }
}

void loop(){
  BLEDevice central = BLE.central();

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

  if(Serial.available()){
    String s=Serial.readStringUntil('\n');
    if(s.length()) handleCommand(s);
  }

  controlStep();
}
