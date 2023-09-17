#ifndef CONFIG_H
#define CONFIG_H

// Sensor bias
#define ROLL_BIAS 1
#define PITCH_BIAS 2.2
#define YAW_BIAS -1.2

// Pid config
#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001

// Motor pins
#define PIN_MOTOR_0  5     
#define PIN_MOTOR_1  3     
#define PIN_MOTOR_2  6     
#define PIN_MOTOR_3  4

// Radio pins
#define RX_PIN_ROLL  19     //PCINT2
#define RX_PIN_PITCH  18    //PCINT3
#define RX_PIN_YAW  2      //PCINT1
#define RX_PIN_THROTTLE  9  //INT6
#define RX_PIN_AUX1  10  //PCINT4
#define RX_PIN_AUX2  11      //INT2

#endif