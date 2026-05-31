



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



//255 − deadband from part1a
int pwmLevels[] = {40, 94,148, 201, 255};

void loop() {
  for (int i = 0; i < 5; i++) {
    encoderCountLeft = 0;
    encoderCountRight = 0;
    analogWrite(LEFT_IN1, pwmLevels[i]);
    analogWrite(LEFT_IN2, 0);
    analogWrite(RIGHT_IN1, pwmLevels[i]);
    analogWrite(RIGHT_IN2, 0);
    Serial.print("PWM = "); Serial.println(pwmLevels[i]);
    delay(3000);   // hold each level long enough to capture on scope
  }
  analogWrite(LEFT_IN1, 0);
}
