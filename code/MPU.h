#ifndef MPU_H
#define MPU_H

#include <MPU6050.h>
#include <Arduino.h>

class MPU {
public:
    bool begin() {
        Wire.begin();
        mpu.initialize();
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
        return mpu.testConnection();
    }

    // Call this regularly to update filtered yaw rate
    void update() {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // raw Z gyro
        float rawGyroZ = (float)gz / 65.5 - calibrateOffset;

        // apply simple moving average filter
        buffer[idx] = rawGyroZ;
        idx = (idx + 1) % bufferSize;

        float sum = 0;
        for (int i = 0; i < bufferSize; i++) sum += buffer[i];

        yawRate = sum / bufferSize;
    }

    float getYawRate() { return yawRate; }

    // call at startup to calibrate zero point
    void calibrate() {
        Serial.println("Calibrating MPU... Keep it still!");
        float sum = 0;
        const int calibSamples = 2000;
        for (int i = 0; i < calibSamples; i++) {
            int16_t gx, gy, gz;
            mpu.getRotation(&gx, &gy, &gz);
            sum += (float)gz / 65.5;  // deg/s
            delay(1);
        }
        calibrateOffset = sum / calibSamples;
        Serial.print("Calibration done! Offset: ");
        Serial.println(calibrateOffset);

        // initialize smoothing buffer to avoid startup spikes
        for (int i = 0; i < bufferSize; i++) buffer[i] = 0;
        idx = 0;
    }

private:
    MPU6050 mpu;
    float gyroz = 0;
    float yawRate = 0;
    float calibrateOffset = 0;

    // --- smoothing buffer ---
    static const int bufferSize = 10;  // moving average over 10 samples
    float buffer[bufferSize] = {0};
    int idx = 0;
};

#endif
