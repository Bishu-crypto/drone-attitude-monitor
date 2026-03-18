#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)len);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.read();
}

int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(500);
  mpuWrite(PWR_MGMT_1, 0x00);
  delay(100);
  Serial.println("Reading raw data...\n");
}

void loop() {
  uint8_t raw[14];
  mpuRead(ACCEL_XOUT_H, raw, 14);

  float ax = toInt16(raw[0],  raw[1])  / 16384.0f;
  float ay = toInt16(raw[2],  raw[3])  / 16384.0f;
  float az = toInt16(raw[4],  raw[5])  / 16384.0f;
  float gx = toInt16(raw[8],  raw[9])  / 131.0f;
  float gy = toInt16(raw[10], raw[11]) / 131.0f;
  float gz = toInt16(raw[12], raw[13]) / 131.0f;

  Serial.printf("Accel(g)  ax:%6.3f  ay:%6.3f  az:%6.3f  |  "
                "Gyro(dps) gx:%7.2f  gy:%7.2f  gz:%7.2f\n",
                ax, ay, az, gx, gy, gz);
  delay(100);
}