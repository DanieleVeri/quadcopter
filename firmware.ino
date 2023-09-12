#include "config.h"
#include "motor.h"
#include "imu.h"
#include "flight_control.h"
#include "radio.h"

void setup() 
{
    Serial.begin(115200);
    setup_motors();
    arm_motors();
    //calib_motors();
    //calib_imu();
    setup_imu();
    setup_radio();
    //setup_pid();
}

int i = 0;
void loop()
{
  if (i++ % 10 == 0)
      debug_radio();
    else
      debug_imu();
}