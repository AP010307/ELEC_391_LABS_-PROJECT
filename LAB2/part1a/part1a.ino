const int LEFT_ENC_A  = 7;
const int LEFT_ENC_B  = 8;
const int RIGHT_ENC_A = 2;
const int RIGHT_ENC_B = 4;

const int LEFT_IN1  = 6;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 5;

volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    encoderCountLeft++;
  } else {
    encoderCountLeft--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    encoderCountRight++;
  } else {
    encoderCountRight--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
}

void loop() {
  for (int pwm = 0; pwm <= 255; pwm++) {
    encoderCountLeft = 0;
    encoderCountRight = 0;

    analogWrite(LEFT_IN1, pwm);
    analogWrite(LEFT_IN2, 0);

    analogWrite(RIGHT_IN1, pwm);
    analogWrite(RIGHT_IN2, 0);

    delay(500);

    noInterrupts();
    long leftCounts = encoderCountLeft;
    long rightCounts = encoderCountRight;
    interrupts();

    Serial.print("PWM: ");
    Serial.print(pwm);

    Serial.print("  Left Counts: ");
    Serial.print(leftCounts);

    Serial.print("  Right Counts: ");
    Serial.println(rightCounts);

  if (abs(leftCounts) > 10) {
      Serial.print("Left motor deadband PWM = ");
      Serial.println(pwm);
  }

  if (abs(rightCounts) > 10) {
    Serial.print("Right motor deadband PWM = ");
    Serial.println(pwm);
  }
  }
}