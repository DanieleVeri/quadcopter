#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU9250.h>

void setup_imu();

void calib_imu();

void debug_imu();

struct Asset {
  double roll, pitch, yaw;
  Asset()
    : roll(0), pitch(0), yaw(0) {}

  void reset() {
    roll = 0;
    pitch = 0;
    yaw = 0;
  }

  void print()
  {
    Serial.print(roll);
    Serial.print("    ");
    Serial.print(pitch);
    Serial.print("    ");
    Serial.println(yaw);
  }

};

Asset get_angles(Asset& angles);

Asset get_rates(Asset& rates);

#endif