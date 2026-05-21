const int LEFT_ENC_A = 2, LEFT_ENC_B = 4;
const int RIGHT_ENC_A = 7, RIGHT_ENC_B = 8;

volatile long leftCount = 0;
volatile long rightCount = 0;

// Full ×4 quadrature: interrupt on both edges of both channels
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

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightB_ISR, CHANGE);
  
  Serial.println("Send 'r' to reset counts");
}

void loop() {
  // Reset on serial command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      leftCount = 0;
      rightCount = 0;
      Serial.println("--- RESET ---");
    }
  }
  
  Serial.print("Left: "); Serial.print(leftCount);
  Serial.print("  Right: "); Serial.println(rightCount);
  delay(100);
}
