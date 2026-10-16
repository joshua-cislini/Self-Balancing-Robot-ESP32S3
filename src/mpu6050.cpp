#include "mpu6050.h"

// Hardware and default offsets initialized in the initializer list
Mpu6050::Mpu6050(uint8_t addr)
  : _accScaleFactor(1.0f / 8192.0f),
    _address(addr),
    _AcXcalibrationOffset(0.0f),
    _AcYcalibrationOffset(-300.0f),
    _AcZcalibrationOffset(0.0f),
    _GyXcalibrationOffset(0.0f),
    _GyYcalibrationOffset(0.0f),
    _GyZcalibrationOffset(0.0f),
    _AcX(0), _AcY(0), _AcZ(0), _Tmp(0), _GyX(0), _GyY(0), _GyZ(0),
    _accelX(0.0f), _accelY(0.0f), _accelZ(0.0f),
    _gyroX(0.0f), _gyroY(0.0f), _gyroZ(0.0f) {}

void Mpu6050::begin() {
  Wire.beginTransmission(_address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up MPU-6050
  Wire.endTransmission(true);
}

void Mpu6050::calibrateGyro() {
  int32_t gyXSum = 0, gyYSum = 0, gyZSum = 0;

  for (uint16_t i = 0; i < NUMBER_OF_CALIBRATION_ITERATIONS; i++) {
    updateIMUdata();
    gyXSum += _GyX;
    gyYSum += _GyY;
    gyZSum += _GyZ;
    delay(1);
  }

  _GyXcalibrationOffset = -(float)gyXSum * (1.0f / NUMBER_OF_CALIBRATION_ITERATIONS);
  _GyYcalibrationOffset = -(float)gyYSum * (1.0f / NUMBER_OF_CALIBRATION_ITERATIONS);
  _GyZcalibrationOffset = -(float)gyZSum * (1.0f / NUMBER_OF_CALIBRATION_ITERATIONS);
}

void Mpu6050::calibrateAccel() {
  int32_t xSum = 0, ySum = 0, zSum = 0;  // Fixed from float to exact 32-bit int

  for (uint16_t i = 0; i < NUMBER_OF_CALIBRATION_ITERATIONS; i++) {
    updateIMUdata();
    xSum += _AcX;
    ySum += _AcY;
    zSum += _AcZ;
    delay(1);
  }

  _AcXcalibrationOffset = -(float)xSum * (1.0f / NUMBER_OF_CALIBRATION_ITERATIONS);
  _AcYcalibrationOffset = -(float)ySum * (1.0f / NUMBER_OF_CALIBRATION_ITERATIONS);
  _accScaleFactor = 1.0f / ((float)zSum / NUMBER_OF_CALIBRATION_ITERATIONS);
}

void Mpu6050::updateIMUdata() {
  Wire.beginTransmission(_address);
  Wire.write(0x3B);  // Start register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);

  // Wire.requestFrom actually returns a uint8_t byte count
  uint8_t numberOfBytes = Wire.requestFrom(_address, (uint8_t)14, (uint8_t) true);

  if (numberOfBytes == 14) {
    // FIX: Read into a buffer sequentially to prevent Undefined Behavior byte-swapping
    uint8_t buf[14];
    for (uint8_t i = 0; i < 14; i++) {
      buf[i] = Wire.read();
    }

    _AcX = (buf[0] << 8) | buf[1];
    _AcY = (buf[2] << 8) | buf[3];
    _AcZ = (buf[4] << 8) | buf[5];
    _Tmp = (buf[6] << 8) | buf[7];
    _GyX = (buf[8] << 8) | buf[9];
    _GyY = (buf[10] << 8) | buf[11];
    _GyZ = (buf[12] << 8) | buf[13];

    _accelX = (_AcX + _AcXcalibrationOffset) * _accScaleFactor;
    _accelY = (_AcY + _AcYcalibrationOffset) * _accScaleFactor;
    _accelZ = (_AcZ + _AcZcalibrationOffset) * _accScaleFactor;

    _gyroX = (_GyX + _GyXcalibrationOffset) * GYRO_SCALE_FACTOR;
    _gyroY = (_GyY + _GyYcalibrationOffset) * GYRO_SCALE_FACTOR;
    _gyroZ = (_GyZ + _GyZcalibrationOffset) * GYRO_SCALE_FACTOR;
  }
}

void Mpu6050::serialPrintDebug() const {
  // Use F() macro to keep these string literals out of SRAM
  Serial.print(F("AccelX: "));
  Serial.print(getAccelX());
  Serial.print(F("  AccelY: "));
  Serial.print(getAccelY());
  Serial.print(F("  AccelZ: "));
  Serial.print(getAccelZ());
  Serial.print(F("  GyroX: "));
  Serial.print(getGyroX());
  Serial.print(F("  GyroY: "));
  Serial.print(getGyroY());
  Serial.print(F("  GyroZ: "));
  Serial.println(getGyroZ());
}

float Mpu6050::getAccelX() const {
  return _accelX;
}
float Mpu6050::getAccelY() const {
  return _accelY;
}
float Mpu6050::getAccelZ() const {
  return _accelZ;
}
float Mpu6050::getGyroX() const {
  return _gyroX;
}
float Mpu6050::getGyroY() const {
  return _gyroY;
}
float Mpu6050::getGyroZ() const {
  return _gyroZ;
}
float Mpu6050::getTemp() const {
  return (_Tmp * (1.0f / 340.00f)) + 36.53f; // instead of return (_Tmp / 340.00f) + 36.53f; Wrapping (1.0f / 340.00f) in parentheses forces the compiler to resolve it to ~0.00294f during compilation, changing the runtime operation to a much faster multiplication.
}