#include "GY87.h"
#include <Arduino.h> // Thêm để dùng PI

GY87::GY87(uint8_t sdaPin, uint8_t sclPin) {
  _sdaPin = sdaPin;
  _sclPin = sclPin;
}

bool GY87::begin() {
  Wire.begin(_sdaPin, _sclPin);
  Wire.setClock(100000);

  // MPU6050
  bool success = writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00); // Tắt ngủ
  success &= writeRegister(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);   // ±2g
  success &= writeRegister(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);    // ±250°/s
  success &= writeRegister(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02);    // Bypass I2C
  if (!success) return false;

  // BMP180
  success = readBMP180Calibration();
  if (!success) return false;

  // HMC5883L
  success = writeRegister(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70); // 8 mẫu, 15Hz
  success &= writeRegister(HMC5883L_ADDR, HMC5883L_MODE, 0x00);   // Đo liên tục
  if (!success) return false;

  return true;
}

bool GY87::read() {
  bool success = readMPU6050();
  success &= readBMP180();
  success &= readHMC5883L();
  if (success) {
    heading = atan2(my, mx) * 180 / PI;
    if (heading < 0) heading += 360;
  }
  return success;
}

bool GY87::writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool GY87::readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(addr, length);
  for (uint8_t i = 0; i < length && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool GY87::readMPU6050() {
  uint8_t buffer[14];
  if (!readRegisters(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }
  ax = (buffer[0] << 8) | buffer[1];
  ay = (buffer[2] << 8) | buffer[3];
  az = (buffer[4] << 8) | buffer[5];
  gx = (buffer[8] << 8) | buffer[9];
  gy = (buffer[10] << 8) | buffer[11];
  gz = (buffer[12] << 8) | buffer[13];
  return true;
}

bool GY87::readBMP180Calibration() {
  uint8_t buffer[22];
  if (!readRegisters(BMP180_ADDR, BMP180_CAL_AC1, buffer, 22)) {
    return false;
  }
  ac1 = (buffer[0] << 8) | buffer[1];
  ac2 = (buffer[2] << 8) | buffer[3];
  ac3 = (buffer[4] << 8) | buffer[5];
  ac4 = (buffer[6] << 8) | buffer[7];
  ac5 = (buffer[8] << 8) | buffer[9];
  ac6 = (buffer[10] << 8) | buffer[11];
  b1 = (buffer[12] << 8) | buffer[13];
  b2 = (buffer[14] << 8) | buffer[15];
  mb = (buffer[16] << 8) | buffer[17];
  mc = (buffer[18] << 8) | buffer[19];
  md = (buffer[20] << 8) | buffer[21];
  return (ac1 != 0 && ac4 != 0 && ac5 != 0);
}

bool GY87::readBMP180() {
  if (!writeRegister(BMP180_ADDR, BMP180_CONTROL, BMP180_TEMP)) {
    return false;
  }
  delay(5);
  uint8_t buffer[2];
  if (!readRegisters(BMP180_ADDR, BMP180_DATA, buffer, 2)) {
    return false;
  }
  int32_t ut = (buffer[0] << 8) | buffer[1];

  int32_t x1 = ((ut - ac6) * ac5) >> 15;
  int32_t x2 = (mc << 11) / (x1 + md);
  int32_t b5 = x1 + x2;
  temperature = ((b5 + 8) >> 4) / 10.0;

  if (!writeRegister(BMP180_ADDR, BMP180_CONTROL, BMP180_PRESSURE)) {
    return false;
  }
  delay(5);
  if (!readRegisters(BMP180_ADDR, BMP180_DATA, buffer, 2)) {
    return false;
  }
  int32_t up = (buffer[0] << 8) | buffer[1];

  int32_t b6 = b5 - 4000;
  x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((ac1 * 4 + x3) << 0) + 2) / 4;
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (50000 >> 0);
  if (b7 < 0x80000000) {
    pressure = (b7 * 2) / b4;
  } else {
    pressure = (b7 / b4) * 2;
  }
  x1 = (pressure >> 8) * (pressure >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * pressure) >> 16;
  pressure = pressure + ((x1 + x2 + 3791) >> 4);
  return true;
}

bool GY87::readHMC5883L() {
  uint8_t buffer[6];
  if (!readRegisters(HMC5883L_ADDR, HMC5883L_DATA_X_H, buffer, 6)) {
    return false;
  }
  mx = (buffer[0] << 8) | buffer[1];
  mz = (buffer[2] << 8) | buffer[3];
  my = (buffer[4] << 8) | buffer[5];
  return true;
}