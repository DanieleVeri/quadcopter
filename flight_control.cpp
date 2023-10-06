#include "Arduino.h"
#include "AutoPID.h"
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

  void reset()
  {
    input = 0;
    output = 0;
    set_point = 0;
  }
};

static ControlValues roll;
static ControlValues pitch;
static ControlValues yaw;
static int throttle;

const int out_minmax = 200;
const int bangbang_value = 200;

static AutoPID PID_roll(&roll.input, &roll.set_point, &roll.output,
  -out_minmax, out_minmax,
  ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD);

static AutoPID PID_pitch(&pitch.input, &pitch.set_point, &pitch.output,
 -out_minmax, out_minmax,
  PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD);

static AutoPID PID_yaw(&yaw.input, &yaw.set_point, &yaw.output,
 -out_minmax, out_minmax,
  YAW_PID_KP,YAW_PID_KI,YAW_PID_KD);

void debug_state()
{
    Serial.println("input");
    Serial.println(roll.input);
    Serial.println(pitch.input);
    Serial.println(yaw.input);

    Serial.println("setpoint");
    Serial.println(roll.set_point);
    Serial.println(pitch.set_point);
    Serial.println(yaw.set_point);
    Serial.println(throttle);

    Serial.println("output");
    Serial.println(roll.output);
    Serial.println(pitch.output);
    Serial.println(yaw.output);
}

void print_motor_values(int a, int b, int c, int d)
{
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms < 200) 
        return;
    prev_ms = millis();
    debug_state();
    Serial.println("motors");
    Serial.println(a);
    Serial.println(b);
    Serial.println(c);
    Serial.println(d);
}

void setup_pid()
{
  PID_roll.setTimeStep(100);
  PID_roll.setBangBang(bangbang_value);
  PID_pitch.setTimeStep(100);
  PID_pitch.setBangBang(bangbang_value);
  PID_yaw.setTimeStep(100);
  PID_yaw.setBangBang(bangbang_value);
}

void control_loop()
{
    Asset asset = get_asset();
    roll.input = asset.roll;
    pitch.input = asset.pitch;
    yaw.input = asset.yaw;

    sample_throttle();
    roll.set_point = map(get_rx_roll(), 1000, 2000, -45, 45);
    pitch.set_point = map(get_rx_pitch(), 1000, 2000, -45, 45);
    yaw.set_point = map(get_rx_yaw(), 1000, 2000, -45, 45);
    throttle = map(get_rx_throttle(), 1000, 1900, 1000, 2000);

    // Controller not connected: reset state
    if (get_rx_throttle() == 0)
    {
      roll.reset();
      pitch.reset();
      yaw.reset();
      throttle = 0;
      PID_roll.stop();
      PID_pitch.stop();
      PID_yaw.stop();
      set_motor_values(0,0,0,0);
      return;
    }

    PID_roll.run();
    PID_pitch.run();
    PID_yaw.run();

    // Mixer
    int m0_val = throttle + roll.output + pitch.output + yaw.output;
    int m1_val = throttle - roll.output + pitch.output - yaw.output;
    int m2_val = throttle + roll.output - pitch.output - yaw.output;
    int m3_val = throttle - roll.output - pitch.output + yaw.output;

    print_motor_values(m0_val, m1_val, m2_val, m3_val);

    set_motor_values(m0_val, m1_val, m2_val, m3_val);
}