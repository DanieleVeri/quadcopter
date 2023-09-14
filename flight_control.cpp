#include "flight_control.h"

#include "imu.h"
#include "radio.h"
#include "motor.h"


struct ControlValues {
  ControlValues(): 
    input(0),
    output(0),
    set_point(0) 
  {}

  double input;
  double output;
  double set_point;
};

static ControlValues roll;
static ControlValues pitch;
static ControlValues yaw;
static ControlValues angleX;
static ControlValues angleY;

static PID PID_roll(&roll.input, &roll.output, &roll.set_point, 
  ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,DIRECT);

static PID PID_pitch(&pitch.input, &pitch.output, &pitch.set_point, 
  PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,DIRECT);

static PID PID_yaw(&yaw.input, &yaw.output, &yaw.set_point, 
  YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,DIRECT);

static PID PID_angleX(&angleX.input, &angleX.output, &angleX.set_point, 
  ANGLEX_PID_KP,ANGLEX_PID_KI,ANGLEX_PID_KD,DIRECT);
  
static PID PID_angleY(&angleY.input, &angleY.output, &angleY.set_point, 
  ANGLEY_PID_KP,ANGLEY_PID_KI,ANGLEY_PID_KD,DIRECT);

void setup_pid()
{

}

void control_loop()
{
    roll.input = get_roll();
    pitch.input = get_pitch();
    yaw.input = get_yaw();

    roll.set_point = rx_val[0];
    pitch.set_point = rx_val[1];
    yaw.set_point = rx_val[2];
    int throttle = rx_val[3];

    PID_roll.Compute();
    PID_pitch.Compute();
    PID_yaw.Compute();

    int m0_val = throttle + roll.output + pitch.output + yaw.output;
    int m1_val = throttle - roll.output + pitch.output - yaw.output;
    int m2_val = throttle + roll.output - pitch.output - yaw.output;
    int m3_val = throttle - roll.output - pitch.output + yaw.output;

    set_motor_values(m0_val, m1_val, m2_val, m3_val);
}