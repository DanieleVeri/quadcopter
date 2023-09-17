#include "Arduino.h"
#include "PID_v1.h"
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
static int throttle;

static PID PID_roll(&roll.input, &roll.output, &roll.set_point, 
  ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,DIRECT);

static PID PID_pitch(&pitch.input, &pitch.output, &pitch.set_point, 
  PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,DIRECT);

static PID PID_yaw(&yaw.input, &yaw.output, &yaw.set_point, 
  YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,DIRECT);

void debug_state(int a, int b, int c, int d)
{
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms < 1000) 
        return;
    prev_ms = millis();

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

    Serial.println("motors");
    Serial.println(a);
    Serial.println(b);
    Serial.println(c);
    Serial.println(d);
}

void setup_pid()
{
  PID_roll.SetMode(AUTOMATIC);
  PID_roll.SetOutputLimits(-100,100);
  roll.output = 0;
  PID_pitch.SetMode(AUTOMATIC);
  PID_pitch.SetOutputLimits(-100,100);
  pitch.output = 0;
  PID_yaw.SetMode(AUTOMATIC);
  PID_yaw.SetOutputLimits(-100,100);
  yaw.output = 0;
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
    throttle = map(get_rx_throttle(), 1000, 2000, 700, 2000);
    // Controller not connected
    if (roll.set_point < -45)
    {
      roll.set_point = 0;
      pitch.set_point = 0;
      yaw.set_point = 0;
    }

    PID_roll.Compute();
    PID_pitch.Compute();
    PID_yaw.Compute();

    int m0_val = throttle + roll.output + pitch.output + yaw.output;
    int m1_val = throttle - roll.output + pitch.output - yaw.output;
    int m2_val = throttle + roll.output - pitch.output - yaw.output;
    int m3_val = throttle - roll.output - pitch.output + yaw.output;

    debug_state(m0_val, m1_val, m2_val, m3_val);

    set_motor_values(m0_val, m1_val, m2_val, m3_val);
}