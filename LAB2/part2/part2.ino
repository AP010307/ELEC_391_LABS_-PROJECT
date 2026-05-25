int pwm50 = 148 ;  // 50% of usable range - confirmed value from part1b

// Encoder pins (yellow = A, white = B from Pololu wire colors)
const int LEFT_ENC_A  = 7;   
const int LEFT_ENC_B  = 8;   
const int RIGHT_ENC_A = 2;   
const int RIGHT_ENC_B = 4;  

// Motor driver input pins (DRV8871 IN1, IN2)
// Only used here to force motors OFF during the manual test
const int LEFT_IN1  = 6;     
const int LEFT_IN2  = 9;    
const int RIGHT_IN1 = 3;    
const int RIGHT_IN2 = 5;     

// ---------- ENCODER COUNT VARIABLES ----------
// 'volatile' is required because these are modified inside ISRs
volatile long leftCount  = 0;
volatile long rightCount = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }   // wait for serial monitor 

  // Encoder pins as inputs
  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  // Force motors OFF (safety - in case pins were in an unknown state)
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  analogWrite(LEFT_IN1,  0);
  analogWrite(LEFT_IN2,  0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);

  // // Attach interrupts on every edge (CHANGE) of all four encoder lines
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  Serial.println("=== Encoder Manual Test ===");
  Serial.println("Expected: ~1920 counts per full wheel revolution");
  Serial.println("Send 'r' in serial monitor to reset counts to 0");
  Serial.println();
}


void setMotor(int in1Pin, int in2Pin, int speed) {
  // speed: positive = forward, negative = backward
  if (speed >= 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  }
}

void loop() {
  Serial.println("Both forward");
  setMotor(LEFT_IN1, LEFT_IN2, pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, -pwm50);
  delay(3000);

  Serial.println("Both backward");
  setMotor(LEFT_IN1, LEFT_IN2, -pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, pwm50);
  delay(3000);

  Serial.println("Left fwd, Right back");
  setMotor(LEFT_IN1, LEFT_IN2, pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, pwm50);
  delay(3000);

  Serial.println("Left back, Right fwd");
  setMotor(LEFT_IN1, LEFT_IN2, -pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, -pwm50);
  delay(3000);
}
