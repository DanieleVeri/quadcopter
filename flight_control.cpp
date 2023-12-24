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
static ControlValues roll_F;
static ControlValues pitch_F;
static ControlValues yaw;
static int throttle;

static AutoPID PID_roll(&roll.input, &roll.set_point, &roll.output,
  -OUT_MINMAX, OUT_MINMAX,
  ROLL_PID_KP,0,0);

static AutoPID PID_roll_F(&roll_F.input, &roll_F.set_point, &roll_F.output,
  -OUT_MINMAX, OUT_MINMAX,
  ROLL_PID_FKP,ROLL_PID_KI,ROLL_PID_KD);

static AutoPID PID_pitch(&pitch.input, &pitch.set_point, &pitch.output,
 -OUT_MINMAX, OUT_MINMAX,
  PITCH_PID_KP,0,0);

static AutoPID PID_pitch_F(&pitch_F.input, &pitch_F.set_point, &pitch_F.output,
 -OUT_MINMAX, OUT_MINMAX,
  PITCH_PID_FKP,PITCH_PID_KI,PITCH_PID_KD);

static AutoPID PID_yaw(&yaw.input, &yaw.set_point, &yaw.output,
 -OUT_MINMAX, OUT_MINMAX,
  YAW_PID_KP,YAW_PID_KI,YAW_PID_KD);

void debug_state()
{
    Serial.println("input");
    Serial.print(roll.input);
    Serial.print(" - ");
    Serial.println(roll_F.input);
    Serial.print(pitch.input);
    Serial.print(" - ");
    Serial.println(pitch_F.input);
    Serial.println(yaw.input);

    Serial.println("setpoint");
    Serial.println(roll.set_point);
    Serial.println(pitch.set_point);
    Serial.println(yaw.set_point);
    Serial.println(throttle);

    Serial.println("output");
    Serial.println(roll.output + roll_F.output);
    Serial.println(pitch.output + pitch_F.output);
    Serial.println(yaw.output);
}

void print_motor_values(int a, int b, int c, int d)
{
    static uint32_t prev_ms = millis();
    static uint32_t ticks = 0;
    ticks++;
    if (millis() - prev_ms < 1000) 
        return;
    prev_ms = millis();
    Serial.print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ ticks:");
    Serial.println(ticks);
    ticks = 0;
    debug_state();
    Serial.println("motors");
    Serial.println(a);
    Serial.println(b);
    Serial.println(c);
    Serial.println(d);
}
void setup_pid()
{
  PID_roll.setTimeStep(TIMESTEP);
  PID_roll.setBangBang(BANGBANG);
  PID_pitch.setTimeStep(TIMESTEP);
  PID_pitch.setBangBang(BANGBANG);
  PID_roll_F.setTimeStep(TIMESTEP);
  PID_roll_F.setBangBang(BANGBANG);
  PID_pitch_F.setTimeStep(TIMESTEP);
  PID_pitch_F.setBangBang(BANGBANG);
  PID_yaw.setTimeStep(TIMESTEP);
  PID_yaw.setBangBang(BANGBANG);
}

float prev_roll=0, prev_pitch=0, prev_yaw=0;

void control_loop()
{
    Asset asset = get_asset();
    roll.input = asset.roll;
    pitch.input = asset.pitch;
    roll_F.input = (1-LPFD) * prev_roll + LPFD * asset.roll;
    pitch_F.input = (1-LPFD) * prev_pitch + LPFD * asset.pitch;
    yaw.input = (1-LPFD) * prev_yaw + LPFD * asset.yaw;
    prev_roll = roll_F.input;
    prev_pitch = pitch_F.input;
    prev_yaw = yaw.input;

    sample_throttle();
    roll.set_point = map(get_rx_roll(), 1000, 2000, -90, 90);
    pitch.set_point = map(get_rx_pitch(), 1000, 2000, -90, 90);
    yaw.set_point = map(get_rx_yaw(), 1000, 2000, -90, 90);
    throttle = map(get_rx_throttle(), 1000, 1900, 1000, 2000);

    // Controller not connected: reset state
    if (get_rx_throttle() == 0)
    {
      roll.reset();
      pitch.reset();
      roll_F.reset();
      pitch_F.reset();
      yaw.reset();
      throttle = 0;
      PID_roll.stop();
      PID_pitch.stop();
      PID_roll_F.stop();
      PID_pitch_F.stop();
      PID_yaw.stop();
      set_motor_values(0,0,0,0);
      return;
    }

    PID_roll.run();
    PID_pitch.run();
    PID_roll_F.run();
    PID_pitch_F.run();
    PID_yaw.run();
    
    // Mixer
    int m0_val = throttle + (roll.output+roll_F.output) + (pitch.output+pitch_F.output) + yaw.output;
    int m1_val = throttle - (roll.output+roll_F.output) + (pitch.output+pitch_F.output) - yaw.output;
    int m2_val = throttle + (roll.output+roll_F.output) - (pitch.output+pitch_F.output) - yaw.output;
    int m3_val = throttle - (roll.output+roll_F.output) - (pitch.output+pitch_F.output) + yaw.output;

    print_motor_values(m0_val, m1_val, m2_val, m3_val);

    set_motor_values(m0_val, m1_val, m2_val, m3_val);
}