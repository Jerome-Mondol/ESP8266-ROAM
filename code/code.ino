#include "MPU.h"
#include "PID.h"
#include "Motor.h"
#include <Wire.h>

MPU mpu;
PID pid(4.0, 0.5, 0.3);
MotorControl motors(D5, D6, D7, D8);
float targetYaw;
int baseSpeed = 255;


void setup() {
  Serial.begin(115200);
  motors.init();
  delay(200);
  Wire.begin();
  motors.init();
  if(mpu.check()) {
    mpu.begin();
    Serial.println("MPU connected!");
  } else {
    Serial.println("MPU cant connect");
  }

  targetYaw = mpu.getYawAngle();
  pid.reset();
}

void loop() {
  if(mpu.check()) {
    mpu.update();
    float currentYaw = mpu.getYawAngle();
    Serial.println(currentYaw);
    float correction = pid.compute(currentYaw, targetYaw);
    motors.setMotors(baseSpeed + 30  - correction, baseSpeed + correction);

  }
}
