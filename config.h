#ifndef CONFIG_H
#define CONFIG_H


// Operating modes
#define MODE_FLIGHT 0
#define MODE_CALIB_MOTORS 1
#define MODE_CALIB_IMU 2
#define OPERATING_MODE MODE_FLIGHT  // <-- set the operating mode
// Debug switch
#define DEBUG 1                     // <-- debug mode

// Sensor bias
#define ROLL_BIAS 0.9
#define PITCH_BIAS 4.9
#define ROLL_RATE_BIAS 1.04
#define PITCH_RATE_BIAS 0.7
#define YAW_RATE_BIAS -1.5
#define IMU_TILT -6.0 * 0.0174533

// PID common
#define OUT_MINMAX 300
#define BANGBANG 300
#define TIMESTEP 10

// Low pass filter
#define LPFD 0.2

// Pid config
#define ROLL_PID_KP 1.0
#define ROLL_PID_KI 0.0
#define ROLL_PID_KD 0

#define PITCH_PID_KP 1.0
#define PITCH_PID_KI 0.0
#define PITCH_PID_KD 0

#define ROLL_RATE_PID_KP 1.0
#define ROLL_RATE_PID_KI 0.0
#define ROLL_RATE_PID_KD 0

#define PITCH_RATE_PID_KP 1.0
#define PITCH_RATE_PID_KI 0.0
#define PITCH_RATE_PID_KD 0

#define YAW_RATE_PID_KP 1.0
#define YAW_RATE_PID_KI 0
#define YAW_RATE_PID_KD 0

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

#endif