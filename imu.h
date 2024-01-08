#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU9250.h>

void setup_imu();

void calib_imu();

void debug_imu();

struct Attitude {
  double roll, pitch, yaw;
  Attitude()
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

struct Linear {
  double x, y, z;
  Linear()
    : x(0), y(0), z(0) {}

  void reset() {
    x = 0;
    y = 0;
    z = 0;
  }

  void print()
  {
    Serial.print(x);
    Serial.print("    ");
    Serial.print(y);
    Serial.print("    ");
    Serial.println(z);
  }

};

void get_linear_acc(Linear& acc, const Attitude& angles);

void get_angles(Attitude& angles);

void get_rates(Attitude& rates);

#endif