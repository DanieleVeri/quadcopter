#include "rangefinder.h"

#include "Arduino.h"
#include "config.h"

void setup_rangefinder() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void get_range(float& distance) {
  static uint32_t prev_ms = 0;
  // Samplerate 2 Hz
  if (millis() - prev_ms < 500)
    return;
  prev_ms = millis();
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  auto duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.0343) / 2;
}