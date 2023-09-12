#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include "config.h"

void setup_radio();

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);

void debug_radio();

#endif