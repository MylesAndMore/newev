#include "imu.h"

IMU::IMU() : z(0.f) {}

void IMU::init() {
  lsm.init();
  lsm.enableDefault();
  lsm.writeReg(LSM6::CTRL2_G, 0b10001000); // Full scale 1000dps
  // Find Z offset
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data
    while((!lsm.readReg(LSM6::STATUS_REG)) & 0x08);
    lsm.read();
    total += lsm.g.z;
  }
  offsetZ = total / 1024;
  reset();
}

void IMU::reset() {
  z = 0.f;
  lastUpdate = micros();
}

void IMU::update() {
  lsm.readGyro();
  int16_t rateZ = lsm.g.z - offsetZ;
  uint16_t t = micros();
  float dt = (t - lastUpdate) / 1e6; // Delta-time in seconds
  lastUpdate = t;
  // Convert raw rate to degrees per second
  float dps = rateZ * 0.035f; // 0.035 dps/digit
  // Integrate to get change in angle (degrees)
  z += dps * dt;
}
