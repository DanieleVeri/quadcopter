#include "Arduino.h"
#include "AutoPID.h"
#include "flight_control.h"
#include "imu.h"
#include "radio.h"
#include "motor.h"


// State variables
static Asset angles;
static Asset rates;
static Asset filtered_rates;

static int throttle;
static Asset angles_setpoint;  // --> radio_input
static Asset rates_setpoint;   // --> angles_output

static Asset rates_output;

static int mixer_output[4];

// PID controllers
static AutoPID PID_roll_angle(&angles.roll, &angles_setpoint.roll, &rates_setpoint.roll,
                             -OUT_MINMAX, OUT_MINMAX,
                             ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);

static AutoPID PID_pitch_angle(&angles.pitch, &angles_setpoint.pitch, &rates_setpoint.pitch,
                              -OUT_MINMAX, OUT_MINMAX,
                              PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);

static AutoPID PID_roll_rate(&filtered_rates.roll, &rates_setpoint.roll, &rates_output.roll,
                             -OUT_MINMAX, OUT_MINMAX,
                             ROLL_RATE_PID_KP, ROLL_RATE_PID_KI, ROLL_RATE_PID_KD);

static AutoPID PID_pitch_rate(&filtered_rates.pitch, &rates_setpoint.pitch, &rates_output.pitch,
                              -OUT_MINMAX, OUT_MINMAX,
                              PITCH_RATE_PID_KP, PITCH_RATE_PID_KI, PITCH_RATE_PID_KD);

static AutoPID PID_yaw_rate(&filtered_rates.yaw, &rates_setpoint.yaw, &rates_output.yaw,
                            -OUT_MINMAX, OUT_MINMAX,
                            YAW_RATE_PID_KP, YAW_RATE_PID_KI, YAW_RATE_PID_KD);

void setup_pid() {
  PID_roll_angle.setTimeStep(TIMESTEP);
  PID_roll_angle.setBangBang(BANGBANG);
  PID_pitch_angle.setTimeStep(TIMESTEP);
  PID_pitch_angle.setBangBang(BANGBANG);
  PID_roll_rate.setTimeStep(TIMESTEP);
  PID_roll_rate.setBangBang(BANGBANG);
  PID_pitch_rate.setTimeStep(TIMESTEP);
  PID_pitch_rate.setBangBang(BANGBANG);
  PID_yaw_rate.setTimeStep(TIMESTEP);
  PID_yaw_rate.setBangBang(BANGBANG);
}

void get_radio_input() {
  sample_throttle();
  throttle = map(get_rx_throttle(), 1000, 1900, 1000, 2000);
  angles_setpoint.roll = map(get_rx_roll(), 1000, 2000, -20, 20);
  angles_setpoint.pitch = map(get_rx_pitch(), 1000, 2000, -20, 20);
  rates_setpoint.yaw = map(get_rx_yaw(), 1000, 2000, -45, 45);
}

void control_loop() {
  get_angles(angles);
  get_rates(rates);
  low_pass_filter(rates, filtered_rates);
  get_radio_input();

  #if !DEBUG
  // Controller not connected: reset state
  if (get_rx_throttle() == 0) {
    angles.reset();
    rates.reset();
    filtered_rates.reset();
    rates_setpoint.reset();
    angles_setpoint.reset();
    throttle = 0;

    PID_roll_angle.stop();
    PID_pitch_angle.stop();
    PID_roll_rate.stop();
    PID_pitch_rate.stop();
    PID_yaw_rate.stop();

    int zero[4] = {0,0,0,0};
    set_motor_values(zero);
    return;
  }
  #endif

  PID_roll_angle.run();
  PID_pitch_angle.run();
  PID_roll_rate.run();
  PID_pitch_rate.run();
  PID_yaw_rate.run();

  // Mixer
  mixer_output[0] = throttle + (rates_output.roll) + (rates_output.pitch) + rates_output.yaw;
  mixer_output[1] = throttle - (rates_output.roll) + (rates_output.pitch) - rates_output.yaw;
  mixer_output[2] = throttle + (rates_output.roll) - (rates_output.pitch) - rates_output.yaw;
  mixer_output[3] = throttle - (rates_output.roll) - (rates_output.pitch) + rates_output.yaw;

  #if DEBUG
  debug_state();
  #endif

  #if ENABLE_MOTORS
  set_motor_values(mixer_output);
  #endif
}

void debug_state() {
  static uint32_t prev_ms = 0;
  static uint32_t ticks = 0;
  ticks++;
  if (millis() - prev_ms < 500)
    return;
  prev_ms = millis();
  Serial.println("======== DEBUG STATE ========");
  Serial.print("Ticks: ");
  Serial.println(ticks);
  ticks = 0;
  Serial.println("Angles");
  angles.print();
  Serial.println("Rates");
  rates.print();
  Serial.println("Filtered rates");
  filtered_rates.print();
  Serial.println("Rates setpoints");
  rates_setpoint.print();
  Serial.println("Angles setpoints");
  angles_setpoint.print();
  Serial.print("Throttle: ");
  Serial.println(throttle);
  Serial.println("Rates outputs");
  rates_output.print();
  Serial.println("Motor outputs");
  for (int i = 0; i < 4; i++) {
    Serial.print(mixer_output[i]);
    Serial.print("  ");
  }
  Serial.print("\n");
}

void low_pass_filter(const Asset& new_asset, Asset& filtered_asset) {
  static Asset cum;
  filtered_asset.roll = (1 - LPFD) * new_asset.roll + LPFD * cum.roll;
  filtered_asset.pitch = (1 - LPFD) * new_asset.pitch + LPFD * cum.pitch;
  filtered_asset.yaw = (1 - LPFD) * new_asset.yaw + LPFD * cum.yaw;
  cum.roll = filtered_asset.roll;
  cum.pitch = filtered_asset.pitch;
  cum.yaw = filtered_asset.yaw;
}