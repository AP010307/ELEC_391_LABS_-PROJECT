
//255 − deadband from part1a
int pwmLevels[] = {...};

void loop() {
  for (int i = 0; i < 4; i++) {
    analogWrite(LEFT_IN1, pwmLevels[i]);
    analogWrite(LEFT_IN2, 0);
    Serial.print("PWM = "); Serial.println(pwmLevels[i]);
    delay(3000);   // hold each level long enough to capture on scope
  }
  analogWrite(LEFT_IN1, 0);
  while(1);
}
