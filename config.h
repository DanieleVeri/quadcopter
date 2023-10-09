#ifndef CONFIG_H
#define CONFIG_H

// Sensor bias
#define ROLL_BIAS 0
#define PITCH_BIAS 0
#define YAW_BIAS -1.2

// PID common
#define OUT_MINMAX 300
#define BANGBANG 300
#define TIMESTEP 10

// Low pass filter
#define LPFD 0.9f

// Pid config
#define ROLL_PID_FKP  3.1
#define ROLL_PID_KP  0
#define ROLL_PID_KI  0
#define ROLL_PID_KD  0

#define PITCH_PID_FKP 3.1
#define PITCH_PID_KP 0
#define PITCH_PID_KI  0
#define PITCH_PID_KD  0

#define YAW_PID_KP  5
#define YAW_PID_KI  0
#define YAW_PID_KD  0

// Motor pins
#define PIN_MOTOR_0  6     
#define PIN_MOTOR_1  3     
#define PIN_MOTOR_2  4     
#define PIN_MOTOR_3  5     

// Radio pins
#define RX_PIN_ROLL  19     
#define RX_PIN_PITCH  18    
#define RX_PIN_YAW  2      
#define RX_PIN_THROTTLE  9  
#define RX_PIN_AUX1  10  
#define RX_PIN_AUX2  11     

#endif-