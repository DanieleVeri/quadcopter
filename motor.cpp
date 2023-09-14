#include "motor.h"


static Servo mot0, mot1, mot2, mot3;


void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}

void setup_motors() {
    mot0.attach(PIN_MOTOR_0, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot1.attach(PIN_MOTOR_1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot2.attach(PIN_MOTOR_2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot3.attach(PIN_MOTOR_3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}

void calib_motors() 
{
    displayInstructions();
    if (Serial.available()) 
    {   
        char data;
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : 
                Serial.println("Sending minimum throttle");
                mot0.writeMicroseconds(MIN_PULSE_LENGTH);
                mot1.writeMicroseconds(MIN_PULSE_LENGTH);
                mot2.writeMicroseconds(MIN_PULSE_LENGTH);
                mot3.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                mot0.writeMicroseconds(MAX_PULSE_LENGTH);
                mot1.writeMicroseconds(MAX_PULSE_LENGTH);
                mot2.writeMicroseconds(MAX_PULSE_LENGTH);
                mot3.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                delay(1000);
                Serial.print(" 2");
                delay(1000);
                Serial.println(" 1...");
                delay(1000);

                for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
                    Serial.print("Pulse length = ");
                    Serial.println(i);
                    mot0.writeMicroseconds(i);
                    mot1.writeMicroseconds(i);
                    mot2.writeMicroseconds(i);
                    mot3.writeMicroseconds(i);
                    delay(200);
                }

                Serial.println("STOP");
                mot0.writeMicroseconds(MIN_PULSE_LENGTH);
                mot1.writeMicroseconds(MIN_PULSE_LENGTH);
                mot2.writeMicroseconds(MIN_PULSE_LENGTH);
                mot3.writeMicroseconds(MIN_PULSE_LENGTH);
            break;
        }
    }
}

void arm_motors()
{
    mot0.writeMicroseconds(MIN_PULSE_LENGTH);
    mot1.writeMicroseconds(MIN_PULSE_LENGTH);
    mot2.writeMicroseconds(MIN_PULSE_LENGTH);
    mot3.writeMicroseconds(MIN_PULSE_LENGTH);
}

void set_motor_values(int v1, int v2, int v3, int v4)
{
    mot0.writeMicroseconds(v1);
    mot1.writeMicroseconds(v2);
    mot2.writeMicroseconds(v3);
    mot3.writeMicroseconds(v4);
}