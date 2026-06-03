/*
 * Integrates the verified foundation:
 *   - fixed-rate loop, actual-dt integration (micros)
 *   - bias-corrected complementary angle, gyro rate negated (Gate 4)
 *   - deadband-compensated, forward-correct drive() (Stage 2b, signs flipped)
 *   - integral clamp (anti-windup), fall cutoff, LIVE serial gain tuning
 *
 * BRING-UP ORDER (do not skip step 2):
 *   1. Hold robot STILL -> gyro bias calibrates at boot.
 *   2. SIGN CHECK (motors disarmed): tilt the robot so it would fall FORWARD.
 *      The printed u must go POSITIVE (= forward drive, gets wheels under it).
 *      If u goes NEGATIVE, send X to flip OUTPUT_SIGN, re-check.
 *   3. Stand it at balance, send E to arm. S to stop. Tune live.
 *
 * Serial (115200, Newline):
 *   E arm   S stop   X flip output sign
 *   P<x> I<x> D<x> set gains    O<x> set target/offset angle    ? print state
 */

#include <Arduino_BMI270_BMM150.h>   // Rev1: swap to <Arduino_LSM9DS1.h>

const int LEFT_IN1=6, LEFT_IN2=9, RIGHT_IN1=3, RIGHT_IN2=5;
const int LEFT_ENC_A=7, LEFT_ENC_B=8, RIGHT_ENC_A=2, RIGHT_ENC_B=4;

volatile long encL=0, encR=0;
void isrLeft()  { if (digitalRead(LEFT_ENC_B))  encL++; else encL--; }
void isrRight() { if (digitalRead(RIGHT_ENC_B)) encR++; else encR--; }

// ===== timing / filter =====
const float        LOOP_HZ=200.0;
const unsigned long LOOP_US=(unsigned long)(1e6/LOOP_HZ);
const float        TAU=0.75;
unsigned long lastLoopUs=0;
float angle=0, gyroBias=0;
float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;

// ===== motor layer (measured + verified) =====
const int   DB_L=17, DB_R=16;
const float U_MAX=255.0, U_DEADZONE=2.0;
const int   FWD_SIGN_L=-1, FWD_SIGN_R=+1;   // positive u = forward (verified)

// ===== PID (live-tunable) =====
float Kp=10.0, Ki=0.0, Kd=0.0;     // starting guesses - tune live
float targetAngle=0.0;             // Gate 5 setpoint, trim live for drift
int   OUTPUT_SIGN=+1;              // sign-coherence flip (step 2)
const float FALL_LIMIT=35.0;       // deg -> cut motors past this
const float I_MAX=120.0;           // anti-windup clamp on Ki*integral

bool  armed=false;
float integral=0;
float uOut=0, pT=0, iT=0, dT=0, err=0, rate=0;
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

void printState(){
  Serial.print("Kp=");Serial.print(Kp);Serial.print(" Ki=");Serial.print(Ki);
  Serial.print(" Kd=");Serial.print(Kd);Serial.print(" target=");Serial.print(targetAngle);
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

  Serial.println("Calibrating gyro bias - hold STILL...");
  delay(500);
  const int N=500; float sum=0; int got=0;
  while(got<N){ if(IMU.gyroscopeAvailable()){ float a,b,c;
    IMU.readGyroscope(a,b,c); sum+=gyroPitchRate(a,b,c); got++; } }
  gyroBias=sum/N;
  while(!IMU.accelerationAvailable()){} IMU.readAcceleration(ax,ay,az);
  angle=accelAngleDeg(ax,ay,az);

  Serial.println("Ready. STEP 2: tilt forward, check u>0 (motors disarmed).");
  Serial.println("Then E to arm. Cmds: E S X P<x> I<x> D<x> O<x> ?");
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
        case 'E': armed=true; integral=0; Serial.println("ARMED"); break;
        case 'S': armed=false; coast(); Serial.println("STOP"); break;
        case 'X': OUTPUT_SIGN=-OUTPUT_SIGN; Serial.print("sign="); Serial.println(OUTPUT_SIGN); break;
        case 'P': Kp=v; printState(); break;
        case 'I': Ki=v; integral=0; printState(); break;
        case 'D': Kd=v; printState(); break;
        case 'O': targetAngle=v; printState(); break;
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
  rate=gyroPitchRate(gx,gy,gz)-gyroBias;
  float k=TAU/(TAU+dt);
  angle=k*(angle+rate*dt)+(1.0-k)*accAng;

  // ---- PID ----
  err=targetAngle-angle;
  integral+=err*dt;
  iT=Ki*integral;
  iT=constrain(iT,-I_MAX,I_MAX);           // anti-windup clamp
  if(Ki>0) integral=iT/Ki;                 // back-calculate clamped accumulator
  pT=Kp*err;
  dT=-Kd*rate;                             // derivative on measurement (gyro)
  float uRaw=OUTPUT_SIGN*(pT+iT+dT);
  uOut=constrain(uRaw,-U_MAX,U_MAX);

  // ---- drive or cut ----
  if(armed && fabs(angle)<FALL_LIMIT){
    drive(uOut);
  } else {
    coast(); integral=0; uOut=0;           // fallen or disarmed
  }

  // ---- telemetry (20 Hz) ----
  if(millis()-lastPrint>=50){
    lastPrint=millis();
    Serial.print("ang=");Serial.print(angle,1);
    Serial.print(" err=");Serial.print(err,1);
    Serial.print(" P=");Serial.print(pT,0);
    Serial.print(" I=");Serial.print(iT,0);
    Serial.print(" D=");Serial.print(dT,0);
    Serial.print(" u=");Serial.print(uOut,0);
    Serial.print(armed?" [ARMED]":" [off]");
    Serial.print(" dt=");Serial.print(dt);

    Serial.println();
  }
}
