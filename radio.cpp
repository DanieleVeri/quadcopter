#include "radio.h"

void setup_radio()
{
    pinMode(RX_PIN_ROLL, INPUT);
    pinMode(RX_PIN_PITCH, INPUT);
    pinMode(RX_PIN_YAW, INPUT);
    pinMode(RX_PIN_THROTTLE, INPUT);
    pinMode(RX_PIN_AUX1, INPUT);
    pinMode(RX_PIN_AUX2, INPUT);
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void debug_radio()
{
    int ch1Value = readChannel(RX_PIN_ROLL, -100, 100, 0);
    int ch2Value = readChannel(RX_PIN_PITCH, -100, 100, 0);
    int ch3Value = readChannel(RX_PIN_YAW, -100, 100, -100);
    int ch4Value = readChannel(RX_PIN_THROTTLE, -100, 100, 0);
    int ch5Value = readChannel(RX_PIN_AUX1, -100, 100, 0);
    int ch6Value = readChannel(RX_PIN_AUX2, -100, 100, 0);
    
    Serial.print("Ch1: ");
    Serial.print(ch1Value);
    Serial.print(" | Ch2: ");
    Serial.print(ch2Value);
    Serial.print(" | Ch3: ");
    Serial.print(ch3Value);
    Serial.print(" | Ch4: ");
    Serial.print(ch4Value);
    Serial.print(" | Ch5: ");
    Serial.print(ch5Value);
    Serial.print(" | Ch6: ");
    Serial.println(ch6Value);
}