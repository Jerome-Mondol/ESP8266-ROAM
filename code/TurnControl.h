
#ifndef TURNCONTROL_H
#define TURNCONTROL_H

#include "PID.h"
#include "MPU.h"
#include "Motor.h"

class TurnControl {
public:
    TurnControl(MPU &m, MotorControl &motors) 
      : imu(m), motorCtrl(motors), pid(2.0, 0.0, 0.25) {}

    void update(float dt) {
        imu.update();
        yaw += imu.getYawRate() * dt;  // integrate rate to get heading

        float correction = pid.compute(yaw, 0);  // target heading = 0
        correction = constrain(correction, -baseSpeed, baseSpeed);

        motorCtrl.setMotors(baseSpeed - correction, baseSpeed + correction);
    }

    void setBaseSpeed(int speed) { baseSpeed = speed; }
    void stop() { motorCtrl.stop(); }

    void resetYaw() { yaw = 0; pid.reset(); }

private:
    MPU &imu;
    MotorControl &motorCtrl;
    PID pid;
    int baseSpeed = 150;
    float yaw = 0;  // integrated yaw angle
};

#endif
