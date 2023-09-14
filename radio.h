#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include "config.h"

volatile static int rx_val[6] {0,0,0,0,0,0};


int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);

void debug_radio();

void register_radio_interrupt();


#endif