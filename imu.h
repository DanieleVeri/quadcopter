#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MPU9250.h>

void setup_imu();

void calib_imu();

void print_roll_pitch_yaw();

void debug_imu(); 

#endif