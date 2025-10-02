#include <Arduino.h>
#include "MPU.h"
#include "PID.h"
#include "Motor.h"
#include "Straight.h"

MPU imu;
PID pid(5.0, 0.05, 0.023);

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

    motors.init();
    goStraight(255, 7.0);   // go straight at speed 200 for 7 seconds
    delay(500);
}

unsigned long lastTime = 0;
float yawAngle = 0;

void loop() {
    // main loop left empty — use goStraight() from setup or call it here when needed
}

