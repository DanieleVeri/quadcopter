#include "config.h"
#include "motor.h"
#include "imu.h"
#include "flight_control.h"
#include "radio.h"

// #define MODE_CALIB_MOTORS
// #define MODE_CALIB_IMU
 #define MODE_FLIGHT


void setup() 
{
    Serial.begin(115200);

    Serial.println("\n Starting firmware setup");

  #ifdef MODE_FLIGHT
    setup_motors();
    arm_motors();
    setup_imu();
    register_radio_interrupt();
    setup_pid();
  #endif

  #ifdef MODE_CALIB_MOTORS
    setup_motors();
    calib_motors();
  #endif

  #ifdef MODE_CALIB_IMU
    calib_imu();
  #endif

  Serial.println("\n Firmware ready");

}

void loop()
{
  #ifdef MODE_FLIGHT
    control_loop();
  #endif

}