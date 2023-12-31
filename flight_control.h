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

void setup_rangefinder();

void get_range(float& distance);

void setup_pid();

void control_loop();

void debug_state();

void low_pass_filter(const Asset& new_asset, Asset& filtered_asset);

#endif