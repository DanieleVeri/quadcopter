#ifndef CONFIG_H
#define CONFIG_H


// Operating modes
#define MODE_FLIGHT 0
#define MODE_CALIB_MOTORS 1
#define MODE_CALIB_IMU 2
#define OPERATING_MODE MODE_FLIGHT            // <-- set the operating mode
#define DEBUG 0                               // <-- disable motors and print debug messages

// RC bounds
#define RC_PITCH_BOUND 30                     // [-30, 30] deg
#define RC_ROLL_BOUND 30                      // [-30, 30] deg
#define RC_YAW_BOUND 90                       // [-90, 90] deg/s
#define RC_MIN_THROTTLE 1000                  // pwm
#define RC_MAX_THROTTLE 2000                  // pwm

// Altitude control
#define RANGEFINDER_MAX_DIST 400              // cm
#define GROUND_DIST_FLYING_THRESHOLD 10       // cm
#define SMOOTH_FALL_PWM 1350                  // pwm
#define START_ALTITUDE_CONTROL_THROTTLE 1400  // pwm
#define STOP_ALTITUDE_CONTROL_THROTTLE 1600   // pwm

// Sensor bias
#define IMU_TO_FRAME_ROLL -6.0                // deg
#define IMU_TO_FRAME_PITCH 6.0                // deg
#define IMU_TO_FRAME_YAW -6.0                 // deg
#define ROLL_RATE_BIAS 1.04                   // deg/s
#define PITCH_RATE_BIAS 0.7                   // deg/s
#define YAW_RATE_BIAS -1.5                    // deg/s

// PID config
#define ROLL_PID_KP 1.1
#define ROLL_PID_KI 0
#define ROLL_PID_KD 0

#define PITCH_PID_KP 1.1
#define PITCH_PID_KI 0
#define PITCH_PID_KD 0

#define ROLL_RATE_PID_KP 0.95
#define ROLL_RATE_PID_KI 0
#define ROLL_RATE_PID_KD 0.25

#define PITCH_RATE_PID_KP 0.95
#define PITCH_RATE_PID_KI 0
#define PITCH_RATE_PID_KD 0.25

#define YAW_RATE_PID_KP 5.0
#define YAW_RATE_PID_KI 0
#define YAW_RATE_PID_KD 0

#define ALTITUDE_PID_KP 200

#define OUT_MINMAX 300
#define BANGBANG 300
#define TIMESTEP 10

// Angular rate low pass filter coefficient
#define LPFD 0.2

// Motor pins
#define PIN_MOTOR_0 6       // pwm pin
#define PIN_MOTOR_1 3       // pwm pin
#define PIN_MOTOR_2 4       // pwm pin
#define PIN_MOTOR_3 5       // pwm pin

// Radio pins
#define RX_PIN_ROLL 19      // interrupt pin
#define RX_PIN_PITCH 18     // interrupt pin
#define RX_PIN_YAW 2        // interrupt pin
#define RX_PIN_THROTTLE 9   // pwm pin
#define RX_PIN_AUX1 10      // pwm pin (unused)
#define RX_PIN_AUX2 11      // pwm pin (unused)

// Rangefinder pins
#define TRIGGER_PIN 45      // pwm pin
#define ECHO_PIN 37

#endif