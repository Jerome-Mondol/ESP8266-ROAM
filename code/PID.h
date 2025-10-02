#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float p, float i, float d) : kp(p), ki(i), kd(d) { reset(); }

    float compute(float input, float target = 0) {
        error = target - input;
        integral += error;
        derivative = error - previousError;
        previousError = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        error = derivative = integral = previousError = 0;
    }

private:
    float kp, ki, kd;
    float error, derivative, integral, previousError;
};

#endif
