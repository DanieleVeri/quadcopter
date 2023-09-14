#include "radio.h"


static volatile int rx_st[6] {0,0,0,0,0,0};

static int pin_map[6] = {
    RX_PIN_ROLL,
    RX_PIN_PITCH,
    RX_PIN_YAW,
    RX_PIN_THROTTLE,
    RX_PIN_AUX1,
    RX_PIN_AUX2
};

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue)
{
    int ch = pulseIn(channelInput, HIGH, 30000);
    if (ch < 100) return defaultValue;
    return map(ch, 1000, 2000, minLimit, maxLimit);
}

void debug_radio()
{
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms < 1000) 
        return;
    prev_ms = millis();
    Serial.println("radio channels");
    rx_val[3] = readChannel(RX_PIN_THROTTLE, -100, 100, 0);
    for (int i = 0; i < 6; i++)
    {
        Serial.println(rx_val[i]);
    }
}

template<unsigned int CH>
void int_signal_rising();

template<unsigned int CH>
void int_signal_falling();


template<unsigned int CH>
void int_signal_rising()
{    
    attachInterrupt(digitalPinToInterrupt(pin_map[CH]), int_signal_falling<CH>, FALLING);
    rx_st[CH] = micros();
}

template<unsigned int CH>
void int_signal_falling()
{    
    attachInterrupt(digitalPinToInterrupt(pin_map[CH]), int_signal_rising<CH>, RISING);
    rx_val[CH] = micros() - rx_st[CH];    
}

void register_radio_interrupt()
{
    pinMode(RX_PIN_ROLL, INPUT);
    pinMode(RX_PIN_PITCH, INPUT);
    pinMode(RX_PIN_YAW, INPUT);
    pinMode(RX_PIN_THROTTLE, INPUT);
    pinMode(RX_PIN_AUX1, INPUT);
    pinMode(RX_PIN_AUX2, INPUT);

    attachInterrupt(digitalPinToInterrupt(RX_PIN_ROLL), int_signal_rising<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(RX_PIN_PITCH), int_signal_rising<1>, RISING);
    attachInterrupt(digitalPinToInterrupt(RX_PIN_YAW), int_signal_rising<2>, RISING);
}
