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

float offAx=0, offAy=0, offAz=0;
float offGx=0, offGy=0, offGz=0;

void getRaw(float &ax, float &ay, float &az,
            float &gx, float &gy, float &gz) {
  uint8_t raw[14];
  mpuRead(ACCEL_XOUT_H, raw, 14);
  ax = toInt16(raw[0],  raw[1])  / 16384.0f;
  ay = toInt16(raw[2],  raw[3])  / 16384.0f;
  az = toInt16(raw[4],  raw[5])  / 16384.0f;
  gx = toInt16(raw[8],  raw[9])  / 131.0f;
  gy = toInt16(raw[10], raw[11]) / 131.0f;
  gz = toInt16(raw[12], raw[13]) / 131.0f;
}

void calibrate(int samples = 1000) {
  Serial.println("Calibrating — keep board FLAT and STILL for 2 seconds...");
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
  float ax,ay,az,gx,gy,gz;

  for (int i = 0; i < samples; i++) {
    getRaw(ax,ay,az,gx,gy,gz);
    sax+=ax; say+=ay; saz+=az;
    sgx+=gx; sgy+=gy; sgz+=gz;
    delay(2);
  }

  offAx = sax/samples;
  offAy = say/samples;
  offAz = saz/samples - 1.0f;
  offGx = sgx/samples;
  offGy = sgy/samples;
  offGz = sgz/samples;

  Serial.printf("Offsets → ax:%.4f ay:%.4f az:%.4f | gx:%.4f gy:%.4f gz:%.4f\n\n",
                offAx, offAy, offAz, offGx, offGy, offGz);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(500);
  mpuWrite(PWR_MGMT_1, 0x00);
  delay(100);
  calibrate(1000);
  Serial.println("Corrected live readings:\n");
}

void loop() {
  float ax,ay,az,gx,gy,gz;
  getRaw(ax,ay,az,gx,gy,gz);

  ax-=offAx; ay-=offAy; az-=offAz;
  gx-=offGx; gy-=offGy; gz-=offGz;

  Serial.printf("Accel(g)  ax:%6.3f  ay:%6.3f  az:%6.3f  |  "
                "Gyro(dps) gx:%7.2f  gy:%7.2f  gz:%7.2f\n",
                ax, ay, az, gx, gy, gz);
  delay(100);
}