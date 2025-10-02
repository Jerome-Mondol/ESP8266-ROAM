#ifndef TURN_H
#define TURN_H

#include <Arduino.h>
#include "MPU.h"
#include "PID.h"
#include "Motor.h"

// Use the global instances from sketch
extern MPU imu;
extern PID pid;
extern MotorControl motors;

// Normalize angle to [-180, 180]
static inline float normalizeAngle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

// Pivot turn in place to a relative angle (degrees). Positive = CCW, Negative = CW.
// timeLimitSeconds: optional max time to attempt the turn
inline void pivotTurnDegrees(float degrees, float timeLimitSeconds = 5.0f) {
    // small stabilization
    unsigned long stabStart = millis();
    while (millis() - stabStart < 200) {
        imu.update();
        delay(5);
    }

    pid.reset();

    unsigned long startTime = millis();
    unsigned long lastTime = startTime;

    float integratedYaw = 0.0f; // relative angle moved since start

    const float tolerance = 1.0f; // degrees for coarse stop
    const float fineTolerance = 0.5f; // tighter for fine tuning
    const int basePivot = 160; // base PWM for pivot (tune if needed)

    // target expressed as signed degrees (relative)
    float target = normalizeAngle(degrees);

    // Coarse pivot: use PID to reduce remaining angle quickly but ensure motors are opposite sign
    while (true) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        lastTime = now;

        imu.update();
        integratedYaw += imu.getYawRate() * dt;
        integratedYaw = normalizeAngle(integratedYaw);

        float remaining = normalizeAngle(target - integratedYaw);

        if (abs(remaining) <= tolerance) break;
        if ((now - startTime) > (unsigned long)(timeLimitSeconds * 1000.0f)) break;

        // PID on remaining angle (we want remaining -> 0)
        float pidOut = pid.compute(0.0f, remaining);
        // limit correction so we don't invert motor signs
        float maxAdj = basePivot - 30;
        float adj = constrain(pidOut, -maxAdj, maxAdj);

        // Compute a pivot power that always yields opposite motor directions
        int dir = (remaining >= 0.0f) ? 1 : -1; // positive -> CCW
        int power = basePivot - (int)abs(adj);
        if (power < 40) power = 40; // ensure enough torque to move

        int leftMotorSpeed  = dir * power;
        int rightMotorSpeed = -dir * power;

        // left motor is inverted in this project (other code uses -leftSpeed), keep consistent
        motors.setMotors(-leftMotorSpeed, rightMotorSpeed);
        delay(10);
    }

    // stop briefly before fine tuning
    motors.setMotors(0, 0);
    delay(40);

    // Fine tune: small slow corrections until within fineTolerance or short timeout
    unsigned long fineStart = millis();
    unsigned long fineLast = fineStart;
    while (millis() - fineStart < 400) {
        unsigned long now = millis();
        float dt = (now - fineLast) / 1000.0f; if (dt <= 0) dt = 0.001f; fineLast = now;

        imu.update();
        integratedYaw += imu.getYawRate() * dt;
        integratedYaw = normalizeAngle(integratedYaw);

        float remaining = normalizeAngle(target - integratedYaw);
        if (abs(remaining) <= fineTolerance) break;

        int dir = (remaining >= 0.0f) ? 1 : -1;
        int smallPower = 60; // low power fine tune
        int leftMotorSpeed  = dir * smallPower;
        int rightMotorSpeed = -dir * smallPower;
        motors.setMotors(-leftMotorSpeed, rightMotorSpeed);
        delay(12);
    }

    motors.setMotors(0, 0);
}

#endif // TURN_H
