#include "throttle.h"

// Stores the currently commanded DAC value for the throttle
static uint8_t currentDacValue = 0;

// Initializes the throttle output and guarantees a safe start state
void setupThrottle() {
  // Ensure DAC pin is set to idle/0 voltage at boot for safety
  stopThrottle();
}

// Directly writes a raw 8-bit value (0-255) to the DAC pin
void setThrottleRaw(uint8_t dacValue) {
  currentDacValue = dacValue;
  dacWrite(THROTTLE_DAC_PIN, currentDacValue); // Native ESP32 DAC write
}

// Sets the throttle based on a normalized range [0.0, 1.0]
void setThrottleNormalized(float val) {
  // Clamp input range to ensure it stays between [0.0, 1.0]
  if (val < 0.0f) val = 0.0f;
  if (val > 1.0f) val = 1.0f;

  // Map the normalized float value to the calibrated DAC output range
  // Calculates the delta and adds it to the MIN_DAC offset
  uint8_t dacVal = THROTTLE_MIN_DAC + (uint8_t)(val * (THROTTLE_MAX_DAC - THROTTLE_MIN_DAC));
  
  // Output the calculated raw value
  setThrottleRaw(dacVal);
}

// Safely zeroes the throttle output
void stopThrottle() {
  // Force output to 0V / Minimum safe idle state to stop the vehicle
  setThrottleRaw(0); // Set to 0 or THROTTLE_MIN_DAC depending on EV controller requirements
}