#include "MPU.h"
#include "PID.h"
#include "Motor.h"
#include <Wire.h>

MPU mpu;
PID pid(4.0, 0.6, 0.7);
MotorControl motors(D5, D6, D7, D8);

float targetYaw;
int baseSpeed = 255;
float carVelocity = 0.17; // m/s (adjust this after testing)


// ---------------- FUNCTION: drive for specific seconds ----------------
void driveTime(float seconds) {
  unsigned long travelTime = (unsigned long)(seconds * 1000);
  unsigned long startTime = millis();

  while (millis() - startTime < travelTime) {
    if (mpu.check()) {
      mpu.update();
      float currentYaw = mpu.getYawAngle();
      float correction = pid.compute(currentYaw, targetYaw);
      motors.setMotors(baseSpeed + 30 - correction, baseSpeed + correction);
    }
  }
  motors.stop();
}

// ---------------- FUNCTION: drive for distance in meters ----------------
void driveDistance(float distance, float velocity = carVelocity) {
  unsigned long travelTime = (unsigned long)((distance / velocity) * 1000);
  unsigned long startTime = millis();

  while (millis() - startTime < travelTime) {
    if (mpu.check()) {
      mpu.update();
      float currentYaw = mpu.getYawAngle();
      float correction = pid.compute(currentYaw, targetYaw);
      motors.setMotors(baseSpeed + 30 - correction, baseSpeed + correction);
    }
  }
  motors.stop();
}


void setup() {
  Serial.begin(115200);
  motors.init();
  delay(200);
  Wire.begin();
  if (mpu.check()) {
    mpu.begin();
    Serial.println("MPU connected!");
  } else {
    Serial.println("MPU cant connect");
  }

  targetYaw = mpu.getYawAngle();
  pid.reset();
}

void loop() {
  // Example usage:
//  driveTime(5);        // drive straight for 5 seconds
//  delay(2000);
  driveDistance(1);  // drive straight for 2 meters
  delay(5000);

  // stop forever
  while (true);
}
