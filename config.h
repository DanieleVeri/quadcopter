#ifndef CONFIG_H
#define CONFIG_H

// Pid config
#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define ROLL_PID_MIN  -50.0
#define ROLL_PID_MAX  50.0
#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -50.0
#define PITCH_PID_MAX  50.0
#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  -50.0
#define YAW_PID_MAX  50.0
#define ANGLEX_PID_KP 5.0
#define ANGLEX_PID_KI 0.02
#define ANGLEX_PID_KD -0.015
#define ANGLEX_PID_MIN -100.0
#define ANGLEX_PID_MAX 100.0
#define ANGLEY_PID_KP 5.0
#define ANGLEY_PID_KI 0.02
#define ANGLEY_PID_KD -0.015
#define ANGLEY_PID_MIN -100.0
#define ANGLEY_PID_MAX 100.0

// Radio config
#define THROTTLE_RMIN  1104
#define THROTTLE_RMAX  1892
#define THROTTLE_WMIN  MOTOR_ARM_START
#define THROTTLE_WMAX  MOTOR_MAX_LEVEL

#define ROLL_RMIN  1040
#define ROLL_RMAX  1976
#define ROLL_WMIN  -15
#define ROLL_WMAX  15

#define PITCH_RMIN  1140
#define PITCH_RMAX  1932
#define PITCH_WMIN  -15
#define PITCH_WMAX  15

#define YAW_RMIN  1028
#define YAW_RMAX  2004
#define YAW_WMIN  -45
#define YAW_WMAX  45

#define RKNOB_RMIN  1008
#define RKNOB_RMAX  2036
#define RKNOB_WMIN  -230
#define RKNOB_WMAX  230

#define LKNOB_RMIN  976
#define LKNOB_RMAX  2004
#define LKNOB_WMIN  0
#define LKNOB_WMAX  500

#define RX_RATE_SENSITIVITY  3
#define RX_ANGLE_DAMPNING  20.0  //D-term dampning
#define RX_EXPO

// Motor PWM levels
#define MOTOR_ZERO_LEVEL  125
#define MOTOR_ARM_START  140
#define MOTOR_MAX_LEVEL  254


// Motor pins
#define PIN_MOTOR_0  3
#define PIN_MOTOR_1  4
#define PIN_MOTOR_2  5
#define PIN_MOTOR_3  6

// Radio pins
#define RX_PIN_ROLL  19     //PCINT2
#define RX_PIN_PITCH  18    //PCINT3
#define RX_PIN_YAW  9      //PCINT1
#define RX_PIN_THROTTLE  2  //INT6
#define RX_PIN_AUX1  10  //PCINT4
#define RX_PIN_AUX2  11      //INT2

#endif