#include <PID_v1.h>
#include <Romi32U4.h>
#include <Wire.h>
#include "imu.h"

#define DIST_TO_TRAVEL 3 // meters
#define BASE_SPEED 300 // 0-300

Romi32U4ButtonA button;
Romi32U4Motors motors;
Romi32U4Encoders encoders;
IMU imu;

#define COUNTS_PER_REV 1437.09 // See https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html
#define WHEEL_CIRCUMFERENCE_M M_PI * 0.07 // Romi wheels are 70mm in diameter
int64_t totalCountsL = 0, totalCountsR = 0;

#define lerp(a, b, t) (a + t * (b - a))
#define DRIVE_LERP 0.007
#define DRIVE_KP 7.5
#define DRIVE_KI 0.5
#define DRIVE_KD 1.6
float prevLeft = 0, prevRight = 0;
double pidIn, pidOut, pidSet;
PID pid(&pidIn, &pidOut, &pidSet, DRIVE_KP, DRIVE_KI, DRIVE_KD, DIRECT);

void drive(int16_t left, int16_t right) {
  // Use linear interpolation to smooth out sharp accelerations
  float speedLeft = lerp(prevLeft, left, DRIVE_LERP);
  prevLeft = speedLeft;
  float speedRight = lerp(prevRight, right, DRIVE_LERP);
  prevRight = speedRight;
  motors.setSpeeds((int16_t)speedLeft, (int16_t)speedRight);
}

float get_dist_traveled() {
  // Accumulate counts in 64-bit integers to prevent overflow
  totalCountsL += encoders.getCountsAndResetLeft();
  totalCountsR += encoders.getCountsAndResetRight();
  float totalCounts = (totalCountsL + totalCountsR) / 2; // Average of left and right encoders to increase accuracy
  return (totalCounts / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Delay so gyro is calibrated when still
  delay(500);
  encoders.init();
  imu.init();
  pid.SetOutputLimits(-150, 150); // Half of motor full range
  pid.SetMode(AUTOMATIC);
  // Turn LED on to indicate that all initialization is complete
  ledYellow(1);
  // Wait until button is presed to exit setup and begin routine
  while (!button.getSingleDebouncedRelease());
}

void loop() {
  // Compute deviation from desired to percieved degree value
  imu.update();
  pidIn = imu.z;
  pid.Compute();
  // Apply control signal to drivetrain
  int16_t speedL = BASE_SPEED - pidOut;
  int16_t speedR = BASE_SPEED + pidOut;
  drive(speedL, speedR);
  // Stop program execution when distance has been achieved
  if (get_dist_traveled() >= DIST_TO_TRAVEL) {
    motors.setSpeeds(0, 0);
    while (true);
  }
}
