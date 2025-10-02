#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl(int left1, int left2, int right1, int right2) 
      : AIN1(left1), AIN2(left2), BIN1(right1), BIN2(right2) {}

    void init() {
        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        stop();
    }

    void setMotors(int leftPWM, int rightPWM) {
        setMotor(AIN1, AIN2, leftPWM);
        setMotor(BIN1, BIN2, rightPWM);
    }

    void stop() {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }

private:
    int AIN1, AIN2, BIN1, BIN2;

    void setMotor(int in1, int in2, int pwm) {
        pwm = constrain(pwm, -255, 255);

        if (pwm > 0) {
            analogWrite(in1, pwm);
            digitalWrite(in2, LOW);
        } else if (pwm < 0) {
            digitalWrite(in1, LOW);
            analogWrite(in2, -pwm);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
        }
    }
};

#endif
