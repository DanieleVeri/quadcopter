#include "flight_control.h"

#include "config.h"


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
    // PID_roll.reset();
    // PID_pitch.reset();
    // PID_yaw.reset();
    // PID_angleX.reset();
    // PID_angleY.reset();
}