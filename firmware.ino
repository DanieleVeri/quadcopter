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
    //setup_radio();
    register_radio_interrupt();
}

int i = 0;
void loop()
{

      debug_radio();
      debug_imu();
}