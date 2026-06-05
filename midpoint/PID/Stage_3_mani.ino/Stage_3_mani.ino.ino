/*
 * ELEC 391 Self-Balancing Robot - STAGE 3 v2: PIDController implementation
 * Board: Arduino Nano 33 BLE Sense (Rev2, BMI270)
 *
 * SAME as Stage 3: tilt calc (bias-corrected complementary filter, negated
 * gyro rate), motor layer (deadband-compensated forward-correct drive),
 * fixed-rate loop, fall cutoff, OUTPUT_SIGN, live serial tuning, telemetry.
 *
 * CHANGED: the inline PID is replaced by the uploaded PIDController struct -
 * trapezoidal integrator, integrator-clamp anti-pwindup, and a BAND-LIMITED
 * derivative-on-measurement (filter time constant tau).
 *
 * NOTE ON Kd SCALE: this differentiator is scaled differently than the old
 * "-Kd*rate" term - expect Kd values roughly an order of magnitude LARGER
 * than before. Re-tune from scratch; tune by behavior, not the old number.
 *
 * Serial (115200, Newline):
 *   E arm   S stop   X flip output sign
 *   P<x> I<x> D<x>   F<x> deriv-filter tau   O<x> target angle   ? state
 */

#include <Arduino_BMI270_BMM150.h>   // Rev1: swap to <Arduino_LSM9DS1.h>

// =================== PIDController (uploaded implementation) ===================
typedef struct {
  float Kp, Ki, Kd;
  float tau;                 // derivative low-pass filter time constant
  float limMin, limMax;      // output limits
  float limMinInt, limMaxInt;// integrator limits
  float T;                   // sample time (s)
  float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void PIDController_Init(PIDController *pid) {
  pid->integrator = 0.0f; pid->prevError = 0.0f;
  pid->differentiator = 0.0f; pid->prevMeasurement = 0.0f;
  pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
  float error = setpoint - measurement;
  float proportional = pid->Kp * error;

  pid->integrator = pid->integrator
                  + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
  if (pid->integrator > pid->limMaxInt)      pid->integrator = pid->limMaxInt;
  else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

  pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

  pid->out = proportional + pid->integrator + pid->differentiator;
  if (pid->out > pid->limMax)      pid->out = pid->limMax;
  else if (pid->out < pid->limMin) pid->out = pid->limMin;

  pid->prevError = error;
  pid->prevMeasurement = measurement;
  return pid->out;
}
// ==============================================================================

const int LEFT_IN1=6, LEFT_IN2=9, RIGHT_IN1=3, RIGHT_IN2=5;
const int LEFT_ENC_A=7, LEFT_ENC_B=8, RIGHT_ENC_A=2, RIGHT_ENC_B=4;

volatile long encL=0, encR=0;
void isrLeft()  { if (digitalRead(LEFT_ENC_B))  encL++; else encL--; }
void isrRight() { if (digitalRead(RIGHT_ENC_B)) encR++; else encR--; }

// ===== timing / filter (unchanged) =====
const float        LOOP_HZ=200.0;
const unsigned long LOOP_US=(unsigned long)(1e6/LOOP_HZ);
const float        TAU=0.75;          // complementary-filter time constant
unsigned long lastLoopUs=0;
float angle=0, gyroBias=0;
float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;

// ===== motor layer (unchanged) =====
const int   DB_L=17, DB_R=16;
const float U_MAX=255.0, U_DEADZONE=2.0;
const int   FWD_SIGN_L=-1, FWD_SIGN_R=+1;

// ===== control config =====
float targetAngle=0.0;
int   OUTPUT_SIGN=-1;
const float FALL_LIMIT=35.0;
const float I_MAX=120.0;              // integrator clamp (output units)
float DERIV_TAU=0.02;                 // derivative filter tau (tunable via F)

PIDController pid;
bool  armed=false;
float uOut=0;
unsigned long lastPrint=0;

float accelAngleDeg(float x,float y,float z){ return atan2(y,z)*180.0/PI; }
float gyroPitchRate(float rx,float ry,float rz){ return -rx; } // negated (Gate 4)

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
void drive(float u){
  setMotorRaw(LEFT_IN1,LEFT_IN2,  compensate(FWD_SIGN_L*u,DB_L));
  setMotorRaw(RIGHT_IN1,RIGHT_IN2, compensate(FWD_SIGN_R*u,DB_R));
}
void coast(){ analogWrite(LEFT_IN1,0);analogWrite(LEFT_IN2,0);
              analogWrite(RIGHT_IN1,0);analogWrite(RIGHT_IN2,0); }

void configurePID(){
  pid.tau=DERIV_TAU;
  pid.limMin=-U_MAX;       pid.limMax=U_MAX;
  pid.limMinInt=-I_MAX;    pid.limMaxInt=I_MAX;
}
// reset filter/integrator state without a derivative kick
void resetPID(){
  pid.integrator=0; pid.differentiator=0;
  pid.prevError=0;  pid.prevMeasurement=angle;
}

void printState(){
  Serial.print("Kp=");Serial.print(pid.Kp);Serial.print(" Ki=");Serial.print(pid.Ki);
  Serial.print(" Kd=");Serial.print(pid.Kd);Serial.print(" tau=");Serial.print(pid.tau,3);
  Serial.print(" target=");Serial.print(targetAngle);
  Serial.print(" sign=");Serial.print(OUTPUT_SIGN);
  Serial.print(" armed=");Serial.println(armed);
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis()<3000){}
  if(!IMU.begin()){ Serial.println("IMU FAIL"); while(1){} }

  pinMode(LEFT_IN1,OUTPUT);pinMode(LEFT_IN2,OUTPUT);
  pinMode(RIGHT_IN1,OUTPUT);pinMode(RIGHT_IN2,OUTPUT); coast();
  pinMode(LEFT_ENC_A,INPUT);pinMode(LEFT_ENC_B,INPUT);
  pinMode(RIGHT_ENC_A,INPUT);pinMode(RIGHT_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),isrLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A),isrRight,RISING);

  // ---- gyro bias cal ----
  Serial.println("Calibrating gyro bias - hold STILL...");
  delay(500);
  const int N=500; float sum=0; int got=0;
  while(got<N){ if(IMU.gyroscopeAvailable()){ float a,b,c;
    IMU.readGyroscope(a,b,c); sum+=gyroPitchRate(a,b,c); got++; } }
  gyroBias=sum/N;
  while(!IMU.accelerationAvailable()){} IMU.readAcceleration(ax,ay,az);
  angle=accelAngleDeg(ax,ay,az);

  // ---- PID init ----
  PIDController_Init(&pid);
  pid.Kp=5; pid.Ki=40.0; pid.Kd=1;   // re-tune (Kd scale differs - expect larger)
  configurePID();
  resetPID();

  Serial.println("Ready. Tilt forward -> u>0 (disarmed). Then E to arm.");
  Serial.println("Cmds: E S X P<x> I<x> D<x> F<x> O<x> ?");
  printState();
  lastLoopUs=micros();
}

void loop(){
  // ---- serial commands ----
  if(Serial.available()){
    String s=Serial.readStringUntil('\n'); s.trim();
    if(s.length()){
      char c=toupper(s.charAt(0)); float v=s.substring(1).toFloat();
      switch(c){
        case 'E': armed=true; resetPID(); Serial.println("ARMED"); break;
        case 'S': armed=false; coast(); Serial.println("STOP"); break;
        case 'X': OUTPUT_SIGN=-OUTPUT_SIGN; Serial.print("sign=");Serial.println(OUTPUT_SIGN); break;
        case 'P': pid.Kp=v; printState(); break;
        case 'I': pid.Ki=v; pid.integrator=0; printState(); break;
        case 'D': pid.Kd=v; printState(); break;
        case 'F': DERIV_TAU=v; pid.tau=v; printState(); break;
        case 'A': targetAngle=v; printState(); break;
        case '?': printState(); break;
      }
    }
  }

  // ---- fixed-rate control loop ----
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

  // ---- PID (uploaded implementation) ----
  pid.T=dt;                                   // actual dt drives integrator+diff
  float pidOut=PIDController_Update(&pid, targetAngle, angle);
  uOut=OUTPUT_SIGN*pidOut;

  // ---- drive or cut ----
  if(armed && fabs(angle)<FALL_LIMIT){
    drive(uOut);
  } else {
    coast(); resetPID(); uOut=0;              // fallen or disarmed
  }

  // ---- telemetry (20 Hz) ----
  if(millis()-lastPrint>=50){
    lastPrint=millis();
    float pTerm=OUTPUT_SIGN*pid.Kp*(targetAngle-angle);
    Serial.print("ang=");Serial.print(angle,1);
    Serial.print(" err=");Serial.print(targetAngle-angle,1);
    Serial.print(" P=");Serial.print(pTerm,0);
    Serial.print(" I=");Serial.print(OUTPUT_SIGN*pid.integrator,0);
    Serial.print(" D=");Serial.print(OUTPUT_SIGN*pid.differentiator,0);
    Serial.print(" u=");Serial.print(uOut,0);
    Serial.print(armed?" [ARMED]":" [off]");
    Serial.print(" dt=");Serial.print(dt,4);
    
    Serial.println();
  }
  }


