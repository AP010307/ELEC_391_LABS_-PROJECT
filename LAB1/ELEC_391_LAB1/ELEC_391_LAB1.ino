#include "Arduino_BMI270_BMM150.h"

float gyro_weight = 0.99;
float accel_weight = 1.00 - gyro_weight;

float roll_angle = 0.0;
float pitch_angle = 0.0;

unsigned long previous_time = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");


  Serial.println();
  Serial.println("accel_roll_deg, accel_pitch_deg, gyro_roll_deg, gyro_pitch_deg, roll_deg, pitch_deg");

  previous_time = millis();
}



void loop() {
  
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.gyroscopeAvailable()&& IMU.accelerationAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    IMU.readAcceleration(ax, ay, az);
  }
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0; // seconds
    previous_time = current_time;

    // Accelerometer angle estimate
    // Roll = rotation around X axis
    // Pitch = rotation around Y axis
    float accel_roll = atan2(ay, az) * 180.0 / PI;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Gyro integration
    float gyro_roll = roll_angle + gx * dt;
    float gyro_pitch = pitch_angle + gy * dt;

    // Complementary filter
    roll_angle = gyro_weight * gyro_roll + accel_weight * accel_roll;
    pitch_angle = gyro_weight * gyro_pitch + accel_weight * accel_pitch;

    // Print as CSV
    Serial.print(accel_roll);
    Serial.print(",");
    Serial.print(accel_pitch);
    Serial.print(",");
    Serial.print(gyro_roll);
    Serial.print(",");
    Serial.print(gyro_pitch);
    Serial.print(",");
    Serial.print(roll_angle);
    Serial.print(",");
    Serial.println(pitch_angle);
  }


