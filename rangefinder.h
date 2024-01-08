#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "imu.h"

void setup_rangefinder();

void get_range(double& distance, const Attitude& angles);

#endif