#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include "config.h"


void register_radio_interrupt();

void sample_throttle();

int get_rx_roll();
int get_rx_pitch();
int get_rx_yaw();
int get_rx_throttle();

void debug_radio();

#endif