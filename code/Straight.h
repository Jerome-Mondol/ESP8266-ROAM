#ifndef STRAIGHT_H
#define STRAIGHT_H

#include <Arduino.h>
#include "MPU.h"
#include "PID.h"
#include "Motor.h"

// These are defined in the main sketch (code.ino)
extern MPU imu;
extern PID pid;
extern MotorControl motors;

// Motor trim (adjust if vehicle pulls to one side). Positive trims increase left motor output.
static const int MOTOR_TRIM = -6; // tune this to correct bias: try -10..+10

// Go straight for timeSeconds (seconds) at given speed (0..255)
inline void goStraight(int speed, float timeSeconds) {
    if (timeSeconds <= 0 || speed <= 0) return;

    // short pre-stabilize: let IMU settle and discard transient gyro readings
    unsigned long stabStart = millis();
    while (millis() - stabStart < 300) {
        imu.update();
        delay(5);
    }

    unsigned long startTime = millis();
    unsigned long lastTime = startTime;
    float integratedYaw = 0.0f;

    pid.reset();

    float smoothCorrection = 0.0f;
    const float alpha = 0.18f; // EMA smoothing factor
    const float deadbandNormal = 5.0f; // degrees of heading error ignored during steady-state
    const float deadbandStartup = 12.0f; // larger deadband during first few hundred ms
    const unsigned long startupIgnore = 300; // ms

    // ramp parameters to avoid sudden jerk at start
    const unsigned long rampMs = 100; // time to reach requested speed

    motors.setMotors(0, 0);

    while ((millis() - startTime) < (unsigned long)(timeSeconds * 1000.0f)) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        lastTime = now;

        imu.update();
        float yawRate = imu.getYawRate();
        integratedYaw += yawRate * dt;

        // compute baseSpeed ramp
        unsigned long sinceStart = now - startTime;
        float rampFactor = (rampMs == 0) ? 1.0f : min(1.0f, (float)sinceStart / (float)rampMs);
        int baseSpeed = (int)(rampFactor * (float)speed);
        baseSpeed = constrain(baseSpeed, 0, speed);

        // Use existing PID (error = target - input). target = 0 heading
        float correction = pid.compute(integratedYaw, 0);
        correction = constrain(correction, -speed, speed);

        // Smooth and use larger deadband during early transient
        smoothCorrection = alpha * correction + (1.0f - alpha) * smoothCorrection;
        float deadband = (sinceStart < startupIgnore) ? deadbandStartup : deadbandNormal;
        if (abs(smoothCorrection) < deadband) smoothCorrection = 0.0f;

        int leftMotorSpeed  = constrain((int)(baseSpeed - smoothCorrection + MOTOR_TRIM), -255, 255);
        int rightMotorSpeed = constrain((int)(baseSpeed + smoothCorrection - MOTOR_TRIM), -255, 255);

        motors.setMotors(-leftMotorSpeed + 50, rightMotorSpeed);
        delay(10);
    }

    motors.setMotors(0, 0);
}

#endif // STRAIGHT_H
