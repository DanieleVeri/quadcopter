#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "imu.h"


struct ControlValues {
  ControlValues()
    : input(0),
      output(0),
      set_point(0) {}

  double input;
  double output;
  double set_point;

  void reset() {
    input = 0;
    output = 0;
    set_point = 0;
  }
};

void setup_pid();

void control_loop();

void debug_state();

void low_pass_filter(const Attitude& new_attitude, Attitude& filtered_attitude);

#endif