#include "config.h"
#include "motor.h"
#include "imu.h"
#include "flight_control.h"
#include "radio.h"
#include "rangefinder.h"


void setup() {
  Serial.begin(115200);

  Serial.println("\n Starting firmware setup...");

#if OPERATING_MODE == MODE_FLIGHT
  setup_motors();
  arm_motors();
  setup_imu();
  //calib_imu();
  register_radio_interrupt();
  setup_rangefinder();
  setup_pid();
#endif

#if OPERATING_MODE == MODE_CALIB_MOTORS
  setup_motors();
  calib_motors();
#endif

#if OPERATING_MODE == MODE_CALIB_IMU
  calib_imu();
#endif

  Serial.println("\n Firmware ready");
}

void loop() {
#if OPERATING_MODE == MODE_FLIGHT
  control_loop();
#endif
}