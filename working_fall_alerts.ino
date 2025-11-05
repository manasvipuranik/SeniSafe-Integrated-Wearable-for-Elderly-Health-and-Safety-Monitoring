#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int buzzerPin = 23;
const int ledPin = 2;
const float FALL_THRESHOLD = 1.5;
const float TILT_ANGLE_THRESHOLD = 45.0;
const float GYRO_CHANGE_THRESHOLD = 3.0;  // Threshold for angular velocity to detect falls
long fallStartTime = 0;
bool fallConfirmed = false;

// Variables for filtering noisy data
float accelX_avg = 0, accelY_avg = 0, accelZ_avg = 0;
float gyroX_avg = 0, gyroY_avg = 0, gyroZ_avg = 0;
const int WS = 5; // window size, number of readings to be consider for moving average calculated later
float aX_window[WS], aY_window[WS], aZ_window[WS];
float gX_window[WS], gY_window[WS], gZ_window[WS];
int WI = 0; // Window Index

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050");
    while (1);
  }

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float accelX = accel.acceleration.x;
  float accelY = accel.acceleration.y;
  float accelZ = accel.acceleration.z;
  float gyroX = gyro.gyro.x;
  float gyroY = gyro.gyro.y;
  float gyroZ = gyro.gyro.z;

  //Angle between the acceleration on two axes
  float tiltAngleX = atan2(accelY, accelZ) * 180 / PI;
  float tiltAngleY = atan2(accelX, accelZ) * 180 / PI;

  filterData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  detectFall(accelX_avg, accelY_avg, accelZ_avg, gyroX_avg, gyroY_avg, gyroZ_avg, tiltAngleX, tiltAngleY);
}

// Function to filter data using moving average
// This is used to reduce noise from the sensor data, minimizing fall alerts
void filterData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
  accelX_window[WI] = accelX;
  accelY_window[WI] = accelY;
  accelZ_window[WI] = accelZ;
  gyroX_window[WI] = gyroX;
  gyroY_window[WI] = gyroY;
  gyroZ_window[WI] = gyroZ;

  accelX_avg = (accelX_avg * (WS - 1) + accelX) / WS;
  accelY_avg = (accelY_avg * (WS - 1) + accelY) / WS;
  accelZ_avg = (accelZ_avg * (WS - 1) + accelZ) / WS;
  gyroX_avg = (gyroX_avg * (WS - 1) + gyroX) / WS;
  gyroY_avg = (gyroY_avg * (WS - 1) + gyroY) / WS;
  gyroZ_avg = (gyroZ_avg * (WS - 1) + gyroZ) / WS;

  WI = (WI + 1) % WS; // Moves to the next slot in the buffer
}

void detectFall(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float tiltAngleX, float tiltAngleY) {
  float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  float totalGyroChange = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  if (totalAcceleration < FALL_THRESHOLD || totalGyroChange > GYRO_CHANGE_THRESHOLD) {
    if (!fallConfirmed) {
      fallStartTime = millis();
      fallConfirmed = true;
      Serial.println("Possible fall detected, waiting for confirmation...");
    }
  }

  if (fallConfirmed && (millis() - fallStartTime > 5000)) {  // 5-second confirmation
    if (abs(tiltAngleX) > TILT_ANGLE_THRESHOLD || abs(tiltAngleY) > TILT_ANGLE_THRESHOLD) {
      Serial.println("Fall confirmed!");
      alertFall();  // Trigger buzzer and LED alert
    } else {
      Serial.println("False alarm, no fall.");
    }
    fallConfirmed = false;
  }
}

// Alert function to trigger buzzer and built-in LED
void alertFall() {
  for (int i = 0; i < 5; i++) {
    tone(buzzerPin, 4500, 1000);      //4.5 MHz frequency, 1 second duration
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}