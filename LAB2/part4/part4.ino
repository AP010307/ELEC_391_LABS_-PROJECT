

// Encoder pins (yellow = A, white = B from Pololu wire colors)
const int LEFT_ENC_A  = 7;   // D7
const int LEFT_ENC_B  = 8;   // D8
const int RIGHT_ENC_A = 2;   // D2
const int RIGHT_ENC_B = 4;   // D4

// Motor driver input pins (DRV8871 IN1, IN2)
// Only used here to force motors OFF during the manual test
const int LEFT_IN1  = 6;     //  D6
const int LEFT_IN2  = 9;     //  D9
const int RIGHT_IN1 = 3;     //  D3
const int RIGHT_IN2 = 5;     //  D5
// ---------- DEADBAND (from Part 1a) ----------

const int LEFT_DEADBAND  = 41;  
const int RIGHT_DEADBAND = 40;   

const int DEADBAND = max(LEFT_DEADBAND, RIGHT_DEADBAND);

// ---------- PWM LEVELS ----------
// Usable range = 255 - deadband
// 25% level = deadband + 0.25 * (255 - deadband)
// 75% level = deadband + 0.75 * (255 - deadband)
const int PWM_25 = DEADBAND + (255 - DEADBAND) / 4;
const int PWM_75 = DEADBAND + 3 * (255 - DEADBAND) / 4;

// ---------- ENCODER CONSTANTS ----------
const long COUNTS_PER_REV = 1920;   // full x4 quadrature, at the wheel

// ---------- ENCODER COUNT VARIABLES ----------
volatile long leftCount  = 0;
volatile long rightCount = 0;

// ============================================================
//  ISRs (same as Part 3)
// ============================================================
void leftA_ISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount--;
  else leftCount++;
}
void leftB_ISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftCount++;
  else leftCount--;
}
void rightA_ISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount--;
  else rightCount++;
}
void rightB_ISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightCount++;
  else rightCount--;
}

// ============================================================
//  MOTOR HELPER (brake-mode PWM)
//  speed: -255 to +255 (negative = reverse)
// ============================================================
void setMotor(int in1Pin, int in2Pin, int speed) {
  if (speed >= 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  setMotor(LEFT_IN1,  LEFT_IN2,  0);
  setMotor(RIGHT_IN1, RIGHT_IN2, 0);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  Serial.println("=== Part 4: Live Encoder Feedback ===");
  Serial.print("Deadband used: "); Serial.println(DEADBAND);
  Serial.print("25% PWM = "); Serial.println(PWM_25);
  Serial.print("75% PWM = "); Serial.println(PWM_75);
  Serial.println();
}

// ============================================================
//  RPM MEASUREMENT
//  Samples count difference over a 1-second window
// ============================================================
void measureAndPrintRPM(int pwm) {
  // Snapshot starting counts
  noInterrupts();
  long startLeft  = leftCount;
  long startRight = rightCount;
  interrupts();

  unsigned long startTime = millis();
  delay(1000);   // 1-second measurement window
  unsigned long endTime = millis();

  // Snapshot ending counts
  noInterrupts();
  long endLeft  = leftCount;
  long endRight = rightCount;
  interrupts();

  // Calculate RPM
  // RPM = (delta_counts / delta_seconds) * (60 / counts_per_rev)
  float dt = (endTime - startTime) / 1000.0;   // seconds
  float leftRPM  = ((endLeft  - startLeft)  / dt) * 60.0 / COUNTS_PER_REV;
  float rightRPM = ((endRight - startRight) / dt) * 60.0 / COUNTS_PER_REV;

  Serial.print("PWM: ");        Serial.print(pwm);
  Serial.print("   Left RPM: ");  Serial.print(leftRPM, 1);
  Serial.print("   Right RPM: "); Serial.println(rightRPM, 1);
}

// ============================================================
//  MAIN LOOP
//  Cycles: 25% PWM -> measure -> 75% PWM -> measure -> stop -> repeat
// ============================================================
void loop() {
  // --- 25% PWM ---
  Serial.println("\n--- Running at 25% of usable range ---");
  setMotor(LEFT_IN1,  LEFT_IN2,  PWM_25);
  setMotor(RIGHT_IN1, RIGHT_IN2, PWM_25);
  delay(1000);   // let motors reach steady state
  measureAndPrintRPM(PWM_25);
  measureAndPrintRPM(PWM_25);   // take a couple of readings
  measureAndPrintRPM(PWM_25);

  // --- 75% PWM ---
  Serial.println("\n--- Running at 75% of usable range ---");
  setMotor(LEFT_IN1,  LEFT_IN2,  PWM_75);
  setMotor(RIGHT_IN1, RIGHT_IN2, PWM_75);
  delay(1000);   // settle time
  measureAndPrintRPM(PWM_75);
  measureAndPrintRPM(PWM_75);
  measureAndPrintRPM(PWM_75);

  // --- STOP ---
  Serial.println("\n--- Stopping for 3 seconds ---");
  setMotor(LEFT_IN1,  LEFT_IN2,  0);
  setMotor(RIGHT_IN1, RIGHT_IN2, 0);
  delay(3000);
}
