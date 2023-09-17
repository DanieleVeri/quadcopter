#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU9250.h>

void setup_imu();

void calib_imu();

void debug_imu(); 

typedef struct 
{
  float roll, pitch, yaw;
}Asset;

Asset get_asset();

#endif