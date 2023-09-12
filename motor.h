#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

#define MIN_PULSE_LENGTH 1000 
#define MAX_PULSE_LENGTH 2000

void display_calib_instructions();
void setup_motors(); 
void calib_motors();
void arm_motors();

#endif