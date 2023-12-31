#include "Arduino.h"
#include "imu.h"
#include "config.h"
#include <math.h>

static MPU9250 mpu;


static void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}

void setup_imu() {
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  mpu.verbose(true);
  mpu.calibrateAccelGyro();
}

void calib_imu() {
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();

  print_calibration();
  mpu.verbose(false);
}

void print_roll_pitch_yaw() {
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 2);
}

void debug_imu() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
  }
}

void get_rates(Asset& rates) {
  if (mpu.update()) {
    rates.roll = -mpu.getGyroY() + ROLL_RATE_BIAS;
    rates.pitch = -mpu.getGyroX() + PITCH_RATE_BIAS;
    rates.yaw = mpu.getGyroZ() + YAW_RATE_BIAS;  // angular velocity

    rates.roll = rates.roll * cos(IMU_TO_FRAME_YAW) - rates.pitch * sin(IMU_TO_FRAME_YAW);
    rates.pitch = rates.roll * sin(IMU_TO_FRAME_YAW) + rates.pitch * cos(IMU_TO_FRAME_YAW);
  }
}

void get_angles(Asset& angles) {
  static uint32_t prev_ms = millis();
  // Waits 10s to be passed before reading the angles
  if (millis() - prev_ms < 10000)
    return;
  angles.roll = mpu.getPitch() + IMU_TO_FRAME_ROLL;
  angles.pitch = -mpu.getRoll() + IMU_TO_FRAME_PITCH;
  angles.yaw = mpu.getYaw();
  
  angles.roll = angles.roll * cos(IMU_TO_FRAME_YAW) - angles.pitch * sin(IMU_TO_FRAME_YAW);
  angles.pitch = angles.roll * sin(IMU_TO_FRAME_YAW) + angles.pitch * cos(IMU_TO_FRAME_YAW);
}

void get_linear_acc(Linear& acc, const Asset& angles)
{
  acc.x = mpu.getAccX() - sin(angles.roll * DEG_TO_RAD);
  acc.y = mpu.getAccY() + sin(angles.pitch * DEG_TO_RAD);
  acc.z = mpu.getAccZ();
}
