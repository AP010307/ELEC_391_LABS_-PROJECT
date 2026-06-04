#include <ArduinoBLE.h>

#define BUFFER_SIZE 20

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

const int LEFT_IN1  = 6;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 3;
const int RIGHT_IN2 = 5;

const int MOTOR_SPEED = 160;   // 0-255
const int MAX_PWM = 255;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopMotors();

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("BLE-DEVICE");
  BLE.setDeviceName("BLE-DEVICE");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) {
       // Get the length of the received data
        int length = customCharacteristic.valueLength();

        // Read the received data
        const unsigned char* receivedData = customCharacteristic.value();

        // Create a properly terminated string
        char receivedString[length + 1]; // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0'; // Null-terminate the string

        String command = String(receivedString);
        command.trim();
        command.toUpperCase();

        // Print the received data to the Serial Monitor
        Serial.print("Received data: ");
        Serial.println(command);

        handleCommand(command);

        // Optionally, respond by updating the characteristic's value
        customCharacteristic.writeValue("Data received");
      }
    }

    stopMotors();

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }
}

// ---------- COMMAND HANDLING ----------
void handleCommand(String command) {
  if (command == "FORWARD" || command == "F" || command == "UP") {
    moveForward(MOTOR_SPEED);
  }
  else if (command == "BACKWARD" || command == "B" || command == "DOWN") {
    moveBackward(MOTOR_SPEED);
  }
  else if (command == "LEFT" || command == "L") {
    turnLeft(MOTOR_SPEED);
  }
  else if (command == "RIGHT" || command == "R") {
    turnRight(MOTOR_SPEED);
  }
  else if (command == "STOP" || command == "S" || command == "A") {
    stopMotors();
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    stopMotors();
  }
}

// ---------- MOTOR FUNCTIONS ----------
void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  }
  else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  }
  else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void moveForward(int speed) {
  setMotor(LEFT_IN1, LEFT_IN2, speed);
  setMotor(RIGHT_IN1, RIGHT_IN2, -speed);
}

void moveBackward(int speed) {
  setMotor(LEFT_IN1, LEFT_IN2, -speed);
  setMotor(RIGHT_IN1, RIGHT_IN2, speed);
}

void turnLeft(int speed) {
  setMotor(LEFT_IN1, LEFT_IN2, speed);
  setMotor(RIGHT_IN1, RIGHT_IN2, speed);
}

void turnRight(int speed) {
  setMotor(LEFT_IN1, LEFT_IN2, -speed);
  setMotor(RIGHT_IN1, RIGHT_IN2, -speed);
}

void stopMotors() {
  setMotor(LEFT_IN1, LEFT_IN2, 0);
  setMotor(RIGHT_IN1, RIGHT_IN2, 0);
}
