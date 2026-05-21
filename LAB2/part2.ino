int pwm50 = ... ;  // 50% of usable range - confirmed value from part1b

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
  setMotor(RIGHT_IN1, RIGHT_IN2, pwm50);
  delay(3000);

  Serial.println("Both backward");
  setMotor(LEFT_IN1, LEFT_IN2, -pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, -pwm50);
  delay(3000);

  Serial.println("Left fwd, Right back");
  setMotor(LEFT_IN1, LEFT_IN2, pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, -pwm50);
  delay(3000);

  Serial.println("Left back, Right fwd");
  setMotor(LEFT_IN1, LEFT_IN2, -pwm50);
  setMotor(RIGHT_IN1, RIGHT_IN2, pwm50);
  delay(3000);
}
