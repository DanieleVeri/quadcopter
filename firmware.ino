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

    setup_imu();
    //calib_imu();

    register_radio_interrupt();

    setup_pid();
}

void loop()
{
    control_loop();
}