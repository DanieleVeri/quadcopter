#include "radio.h"

volatile int rx_val[6] {0,0,0,0,0,0};

static volatile int rx_st[6] {0,0,0,0,0,0};

static int pin_map[6] = {
    RX_PIN_ROLL,
    RX_PIN_PITCH,
    RX_PIN_YAW,
    RX_PIN_THROTTLE,
    RX_PIN_AUX1,
    RX_PIN_AUX2
};

void debug_radio()
{
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms < 1000) 
        return;
    prev_ms = millis();
    Serial.println("radio channels");
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

void sample_throttle()
{
    static uint32_t prev_ms = millis();
    if (millis() - prev_ms < 100) 
        return;
    prev_ms = millis();
    rx_val[3] = pulseIn(RX_PIN_THROTTLE, HIGH, 30000);
}

int get_rx_roll()
{
    return rx_val[0];
}

int get_rx_pitch()
{
    return rx_val[1];
}

int get_rx_yaw()
{
    return rx_val[2];
}

int get_rx_throttle()
{
    return rx_val[3];
}