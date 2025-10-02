  #include <Arduino.h>
  #include "MPU.h"
  #include "PID.h"
  #include "Motor.h"
  
  MPU imu;
  PID pid(5.0, 0.02, 0.023);

  MotorControl motors(D5, D6, D7, D8);
  
  void setup() {
      Serial.begin(9600);  // make sure Serial Monitor is 9600 baud
      Wire.begin();
  
      if (imu.begin()) {
          Serial.println("MPU6050 initialized!");
          imu.calibrate();  // blocks for a few seconds while calibrating
          delay(1000); // 1 second pause to let MPU stabilize

      } else {
          Serial.println("MPU6050 connection FAILED!");
          while(1);  // stop if not connected
      } 
  }
  
  unsigned long lastTime = 0;
  float yawAngle = 0;
  
  void loop() {
      unsigned long now = millis();
      if (millis() < 1000) yawAngle = 0; // ignore gyro drift for first second
      float dt = (now - lastTime) / 1000.0; // seconds
      lastTime = now;
  
      imu.update();
      float yawRate = imu.getYawRate();
  
      yawAngle += yawRate * dt;  // integrate yaw
      float correction = pid.compute(yawAngle, 0);  // target heading = 0
  
      int rampTime = 1000; // 1 second
int baseSpeed = map(millis(), 0, rampTime, 0, 200);
baseSpeed = constrain(baseSpeed, 0, 200);

      correction = constrain(correction, -baseSpeed, baseSpeed);
      int leftMotorSpeed  = constrain(baseSpeed - correction , 0, 255);
      int rightMotorSpeed = constrain(baseSpeed + correction , 0, 255);
  
      motors.setMotors(-leftMotorSpeed, rightMotorSpeed);
  }
  
