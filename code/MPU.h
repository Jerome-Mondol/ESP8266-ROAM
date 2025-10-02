#ifndef MPU_H
#define MPU_H
#include <MPU6050_6Axis_MotionApps20.h>


  

class MPU {
  public:
  void begin() {
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    calibrate();
  }

  void update() {
    unsigned long now = millis();
    float dt = (now - lastTime)/1000.0;
    lastTime = now;
    
    int16_t raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az;
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    ax = raw_ax / 16384.0;
    ay = raw_ay / 16384.0;
    az = raw_az / 16384.0;
    
    gx = raw_gx / 65.5;
    gy = raw_gy / 65.5;
    gz = raw_gz / 65.5;

    yawAngle = alpha * (yawAngle + getYawRate() * dt);
  }

  void calibrate() {
    float sum = 0;
    for(int i = 0; i < samples; i++) {
      update();
      sum += gz;
      delay(2);
    }
    gzOffset = sum / samples;
    
  }

  bool check() {
    if(mpu.testConnection()) return true;
    else return false;
  }

  float getYawRate() { return gz - gzOffset; }
  float getYawAngle() { return yawAngle; }
  private:
  MPU6050 mpu;
  float ax, ay, az;
  float gx, gy, gz; 
  int samples = 2000;
  float alpha = 0.98;

  unsigned long lastTime = 0;
  float gzOffset = 0;
  float yawAngle = 0;
};


#endif
