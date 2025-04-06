#pragma once

#include <LSM6.h>

class IMU {
  public:
    float z;

    IMU();
    void init();
    void reset();
    void update();

  private:
    LSM6 lsm;
    int16_t offsetZ; 
    uint16_t lastUpdate;
};
