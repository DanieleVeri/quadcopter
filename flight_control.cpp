#include "Arduino.h"
#include "AutoPID.h"
#include "flight_control.h"
#include "imu.h"
#include "radio.h"
#include "motor.h"
#include "rangefinder.h"


// State variables
static Linear acc;
static Attitude angles;
static Attitude rates;
static Attitude filtered_rates;
static double ground_dist = 0;

// RC input
static Attitude angles_setpoint;
static int rc_throttle = 0;

// Control values
static Attitude rates_setpoint;  // --> angles_output
static Attitude rates_output;
static const double acc_setpoint = 1;
static double acc_correction = 0;

// Outputs
static int output_throttle = 0;
static int mixer_output[4] = { 0, 0, 0, 0 };

// PID controllers
static AutoPID PID_altitude_control(&acc.z, &acc_setpoint, &acc_correction,
                                    -OUT_MINMAX, OUT_MINMAX,
                                    ALTITUDE_PID_KP, 0, 0);

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

void get_radio_input() {
  sample_throttle();
  rc_throttle = map(get_rx_throttle(), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH, RC_MIN_THROTTLE, RC_MAX_THROTTLE);
  angles_setpoint.roll = map(get_rx_roll(), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH, -RC_ROLL_BOUND, RC_ROLL_BOUND);
  angles_setpoint.pitch = map(get_rx_pitch(), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH, RC_PITCH_BOUND, -RC_PITCH_BOUND);
  rates_setpoint.yaw = map(get_rx_yaw(), MIN_PULSE_LENGTH, MAX_PULSE_LENGTH, -RC_YAW_BOUND, RC_YAW_BOUND);
}

static uint32_t last_connected_ts = 0;

void control_loop() {
  sample_throttle();
#if DEBUG
  const bool radio_connected = true;
#else
  const bool radio_connected = (get_rx_throttle() > 0);
#endif

  // Radio input
  bool lost_signal = false;
  if (radio_connected) {
    last_connected_ts = millis();
    get_radio_input();
  } else {
    // Allow lost of signal < 1s
    if (millis() - last_connected_ts > 1000) {
      lost_signal = true;
    }
  }

  if (!lost_signal) {
    // Update state
    get_angles(angles);
    get_linear_acc(acc, angles);
    get_rates(rates);
    low_pass_filter(rates, filtered_rates);
    get_range(ground_dist, angles);

    // Compute PID
    PID_altitude_control.run();
    PID_roll_angle.run();
    PID_pitch_angle.run();
    PID_roll_rate.run();
    PID_pitch_rate.run();
    PID_yaw_rate.run();

    // Altitude control
    const bool is_flying = (ground_dist > GROUND_DIST_FLYING_THRESHOLD && ground_dist < RANGEFINDER_MAX_DIST);
    const bool maintain_altitude = (rc_throttle > START_ALTITUDE_CONTROL_THROTTLE && rc_throttle < STOP_ALTITUDE_CONTROL_THROTTLE);
    if (maintain_altitude && is_flying) {
      output_throttle = rc_throttle - acc_correction;
    } else {
      output_throttle = rc_throttle;
    }

    // Prevent free fall from more than 10cm
    if (is_flying)
      output_throttle = max(rc_throttle, SMOOTH_FALL_PWM);

    // Thrust correction depending on attitude
    output_throttle = MIN_PULSE_LENGTH + ((float)(output_throttle - MIN_PULSE_LENGTH)) / (cos(angles.roll * DEG_TO_RAD) * cos(angles.pitch * DEG_TO_RAD));

    // Mixer
    mixer_output[0] = output_throttle + (rates_output.roll) + (rates_output.pitch) + rates_output.yaw;
    mixer_output[1] = output_throttle - (rates_output.roll) + (rates_output.pitch) - rates_output.yaw;
    mixer_output[2] = output_throttle + (rates_output.roll) - (rates_output.pitch) - rates_output.yaw;
    mixer_output[3] = output_throttle - (rates_output.roll) - (rates_output.pitch) + rates_output.yaw;

  } else {
    // Reset state
    acc.reset();
    angles.reset();
    rates.reset();
    filtered_rates.reset();
    angles_setpoint.reset();
    rates_setpoint.reset();
    angles_setpoint.reset();

    // Reset PID
    PID_altitude_control.stop();
    PID_roll_angle.stop();
    PID_pitch_angle.stop();
    PID_roll_rate.stop();
    PID_pitch_rate.stop();
    PID_yaw_rate.stop();

    // Zero the mixer output
    memset(mixer_output, 0, sizeof(int) * 4);
  }

  // Power the motors
#if !DEBUG
  set_motor_values(mixer_output);
#endif

#if DEBUG
  debug_state();
#endif
}

void setup_pid() {
  PID_altitude_control.setTimeStep(TIMESTEP);
  PID_altitude_control.setBangBang(BANGBANG);

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
  Serial.println("Linear acceleration (g)");
  acc.print();
  Serial.print("Ground distance (cm): ");
  Serial.println(ground_dist);
  Serial.print("Acc z correction (cm): ");
  Serial.println(acc_correction);
  Serial.println("Angles (deg)");
  angles.print();
  Serial.println("Rates (deg/s)");
  rates.print();
  Serial.println("Filtered rates (deg/s)");
  filtered_rates.print();
  Serial.print("Throttle RC (pwm): ");
  Serial.println(rc_throttle);
  Serial.print("Throttle output (pwm): ");
  Serial.println(output_throttle);
  Serial.println("Angles setpoints (deg)");
  angles_setpoint.print();
  Serial.println("Rates setpoints (deg/s)");
  rates_setpoint.print();
  Serial.println("Rates outputs (deg/s)");
  rates_output.print();
  Serial.println("Motor outputs (pwm)");
  for (int i = 0; i < 4; i++) {
    Serial.print(mixer_output[i]);
    Serial.print("  ");
  }
  Serial.print("\n");
}

void low_pass_filter(const Attitude& new_attitude, Attitude& filtered_attitude) {
  static Attitude cum;
  filtered_attitude.roll = (1 - LPFD) * new_attitude.roll + LPFD * cum.roll;
  filtered_attitude.pitch = (1 - LPFD) * new_attitude.pitch + LPFD * cum.pitch;
  filtered_attitude.yaw = (1 - LPFD) * new_attitude.yaw + LPFD * cum.yaw;
  cum.roll = filtered_attitude.roll;
  cum.pitch = filtered_attitude.pitch;
  cum.yaw = filtered_attitude.yaw;
}
