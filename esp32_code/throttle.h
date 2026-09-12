#ifndef THROTTLE_H
#define THROTTLE_H

#include <Arduino.h>

// Hardware Pin Configuration
// Pin used for analog output to EV controller (must be an ESP32 DAC capable pin)
constexpr int THROTTLE_DAC_PIN = 25; 

// Safety & Analog Voltage Parameters
// ESP32 DAC outputs 0-3.3V mapped to 8-bit resolution (0 - 255)
// ~0.8V Idle -> ~62 DAC | ~3.3V Max -> 255 DAC (Adjust values as needed for EV Controller)
constexpr uint8_t THROTTLE_MIN_DAC = 62;   // The DAC value that corresponds to ~0.8V Idle signal threshold
constexpr uint8_t THROTTLE_MAX_DAC = 255;  // The DAC value that corresponds to ~3.3V Full Throttle

// Initializes the throttle subsystem, setting a safe idle voltage
void setupThrottle();

// Sets a raw 8-bit DAC value to the throttle output pin
void setThrottleRaw(uint8_t dacValue);

// Sets the throttle using a normalized float value
// Input: 0.0f (Idle) to 1.0f (Full Throttle)
// Handles mapping and safety limits automatically
void setThrottleNormalized(float val); 

// Forces the throttle output to a 0V / Minimum safe idle state
void stopThrottle();

#endif /* THROTTLE_H */