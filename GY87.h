#ifndef GY87_H
#define GY87_H

#include <Wire.h>

class GY87 {
public:
  // Khởi tạo với chân I2C
  GY87(uint8_t sdaPin, uint8_t sclPin);

  // Khởi động cảm biến
  bool begin();

  // Đọc tất cả dữ liệu
  bool read();

  // Dữ liệu công khai
  int16_t ax, ay, az; // Gia tốc (raw, ±2g)
  int16_t gx, gy, gz; // Con quay (raw, ±250°/s)
  float temperature;  // Nhiệt độ (°C)
  int32_t pressure;   // Áp suất (Pa)
  int16_t mx, my, mz; // Từ trường (raw)
  float heading;      // Góc la bàn (độ)

private:
  uint8_t _sdaPin, _sclPin;

  // Địa chỉ I2C
  static const uint8_t MPU6050_ADDR = 0x68;
  static const uint8_t BMP180_ADDR = 0x77;
  static const uint8_t HMC5883L_ADDR = 0x1E;

  // Thanh ghi MPU6050
  static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
  static const uint8_t MPU6050_INT_PIN_CFG = 0x37;
  static const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
  static const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
  static const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
  static const uint8_t MPU6050_GYRO_XOUT_H = 0x43;

  // Thanh ghi BMP180
  static const uint8_t BMP180_CAL_AC1 = 0xAA;
  static const uint8_t BMP180_CONTROL = 0xF4;
  static const uint8_t BMP180_DATA = 0xF6;
  static const uint8_t BMP180_TEMP = 0x2E;
  static const uint8_t BMP180_PRESSURE = 0x34;

  // Thanh ghi HMC5883L
  static const uint8_t HMC5883L_CONFIG_A = 0x00;
  static const uint8_t HMC5883L_MODE = 0x02;
  static const uint8_t HMC5883L_DATA_X_H = 0x03;

  // Biến hiệu chỉnh BMP180
  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;

  // Hàm nội bộ
  bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
  bool readMPU6050();
  bool readBMP180Calibration();
  bool readBMP180();
  bool readHMC5883L();
};

#endif