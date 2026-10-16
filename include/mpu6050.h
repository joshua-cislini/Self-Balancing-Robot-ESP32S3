#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>  // Serial, F(), delay()
#include <Wire.h>     // Wire

class Mpu6050 {
public:
  Mpu6050(uint8_t addr);  // I2C addresses are 7-bit, use uint8_t

  void begin();
  void calibrateGyro();
  void calibrateAccel();
  void updateIMUdata();
  void serialPrintDebug() const;  // Marked const

  // Getters marked const because they don't modify internal variables
  float getAccelX() const;
  float getAccelY() const;
  float getAccelZ() const;
  float getGyroX() const;
  float getGyroY() const;
  float getGyroZ() const;
  float getTemp() const;

private:
  static constexpr uint16_t NUMBER_OF_CALIBRATION_ITERATIONS = 500;
  static constexpr float GYRO_SCALE_FACTOR = 1.0f / 131.0f;
  float _accScaleFactor;

  uint8_t _address;

  float _AcXcalibrationOffset, _AcYcalibrationOffset, _AcZcalibrationOffset;
  float _GyXcalibrationOffset, _GyYcalibrationOffset, _GyZcalibrationOffset;

  int16_t _AcX, _AcY, _AcZ, _Tmp, _GyX, _GyY, _GyZ;
  float _accelX, _accelY, _accelZ, _gyroX, _gyroY, _gyroZ;
};

#endif