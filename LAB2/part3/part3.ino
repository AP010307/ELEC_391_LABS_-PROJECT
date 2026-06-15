

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



// ---------- ENCODER COUNT VARIABLES ----------
volatile long leftCount  = 0;
volatile long rightCount = 0;

// ------ INTERRUPT SERVICE ROUTINES (ISRs) -------------
//  Full x4 quadrature: interrupts fire on both edges of A and B.
//  Direction logic compares A and B levels at the moment of edge.

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


// SETUP
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

  // Attach interrupts on every edge (CHANGE) of all four encoder lines
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);

  Serial.println("=== Encoder Manual Test ===");
  Serial.println("Expected: ~1920 counts per full wheel revolution");
  Serial.println("Send 'r' in serial monitor to reset counts to 0");
  Serial.println();
}

// --------------- MAIN LOOP --------------------
//  Prints live encoder counts; listens for 'r' to reset.

void loop() {



  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      // Disable interrupts briefly so the reset is atomic
      noInterrupts();
      leftCount  = 0;
      rightCount = 0;
      interrupts();
      Serial.println("--- COUNTS RESET ---");
    }
  }

  noInterrupts();
  if (abs(leftCount) >= 1920) {
  leftCount = 0;
}
if (abs(rightCount) >= 1920) {
  rightCount = 0;
}
  long L = leftCount;
  long R = rightCount;
  interrupts();



  Serial.print("Left: ");  Serial.print(L); 
  Serial.print("   Right: "); Serial.println(R);


  delay(100); 

}
