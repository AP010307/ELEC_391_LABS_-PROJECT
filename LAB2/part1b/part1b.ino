const int LEFT_IN1 = D3, LEFT_IN2 = D5;
const int LEFT_ENC_A = D2, LEFT_ENC_B = D4;

volatile long encoderCount = 0;

void encoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) encoderCount++;
  else encoderCount--;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), encoderISR, RISING);
}



//255 − deadband from part1a
int pwmLevels[] = {40, 94,148, 201, 255};

void loop() {
  for (int i = 0; i < 5; i++) {
    encoderCount = 0;
    analogWrite(LEFT_IN1, pwmLevels[i]);
    analogWrite(LEFT_IN2, 0);
    Serial.print("PWM = "); Serial.println(pwmLevels[i]);
    delay(3000);   // hold each level long enough to capture on scope
  }
  analogWrite(LEFT_IN1, 0);
}
