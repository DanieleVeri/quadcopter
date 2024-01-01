#ifndef CONFIG_H
#define CONFIG_H


// Operating modes
#define MODE_FLIGHT 0
#define MODE_CALIB_MOTORS 1
#define MODE_CALIB_IMU 2
#define OPERATING_MODE MODE_FLIGHT  // <-- set the operating mode
#define DEBUG 0                     // <-- disable motors and print debug messages

// Sensor bias
#define IMU_TO_FRAME_ROLL -6.0
#define IMU_TO_FRAME_PITCH 6.0
#define IMU_TO_FRAME_YAW -6.0 * DEG_TO_RAD
#define ROLL_RATE_BIAS 1.04
#define PITCH_RATE_BIAS 0.7
#define YAW_RATE_BIAS -1.5

// PID common
#define OUT_MINMAX 300
#define BANGBANG 300
#define TIMESTEP 10

// Pid config
#define ROLL_PID_KP 1.1
#define ROLL_PID_KI 0
#define ROLL_PID_KD 0

#define PITCH_PID_KP 1.1
#define PITCH_PID_KI 0
#define PITCH_PID_KD 0

#define ROLL_RATE_PID_KP 1
#define ROLL_RATE_PID_KI 0
#define ROLL_RATE_PID_KD 0.25

#define PITCH_RATE_PID_KP 1
#define PITCH_RATE_PID_KI 0
#define PITCH_RATE_PID_KD 0.25

#define YAW_RATE_PID_KP 5.0
#define YAW_RATE_PID_KI 0
#define YAW_RATE_PID_KD 0

// Angular rate low pass filter coefficient
#define LPFD 0.2

// Smooth fall
#define SMOOTH_FALL_DIST 10
#define SMOOTH_FALL_PWM 1350

// Motor pins
#define PIN_MOTOR_0 6
#define PIN_MOTOR_1 3
#define PIN_MOTOR_2 4
#define PIN_MOTOR_3 5

// Radio pins
#define RX_PIN_ROLL 19
#define RX_PIN_PITCH 18
#define RX_PIN_YAW 2
#define RX_PIN_THROTTLE 9
#define RX_PIN_AUX1 10
#define RX_PIN_AUX2 11

// Rangefinder
#define TRIGGER_PIN 45
#define ECHO_PIN 37

#endif