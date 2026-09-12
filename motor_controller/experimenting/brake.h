#ifndef BRAKE_H
#define BRAKE_H

#include <Arduino.h>

// BTS7960 Hardware Pins
// Configuration for the brake linear actuator motor driver
constexpr int BRAKE_RPWM_PIN  = 16; // Extend PWM signal pin
constexpr int BRAKE_LPWM_PIN  = 17; // Retract PWM signal pin
constexpr int BRAKE_R_EN_PIN  = 5;  // Right Enable pin for driver
constexpr int BRAKE_L_EN_PIN  = 18; // Left Enable pin for driver
constexpr int BRAKE_LIMIT_PIN = 19; // Max Brake Limit Switch input pin

// LEDC Settings (ESP32 Hardware PWM)
constexpr int BRAKE_PWM_FREQ = 10000;       // PWM frequency (10 kHz)
constexpr int BRAKE_PWM_RESOLUTION = 8;     // 8-bit resolution (0-255)
constexpr uint8_t BRAKE_MANUAL_SPEED = 255; // Default full speed (100% duty cycle)

// LEDC Hardware Channels 2 & 3
// ESP32 channels allocated for brake PWM control
constexpr int BRAKE_RPWM_CHANNEL = 2;
constexpr int BRAKE_LPWM_CHANNEL = 3;

// Safety Limit: Reduced pulse timer to 1000ms to prevent mechanical over-extension
constexpr unsigned long BRAKE_PULSE_MS = 1500;

// Defines the current operating state of the brake actuator
enum BrakeState {
  BRAKE_IDLE,       // Actuator is stationary/unpowered
  BRAKE_EXTENDING,  // Actuator is currently extending
  BRAKE_RETRACTING  // Actuator is currently retracting
};

// Initializes the brake hardware (pins and PWM configuration)
void setupBrake();

// Initiates the extension of the brake actuator (applies brakes)
void triggerBrakeExtend(uint8_t speed = BRAKE_MANUAL_SPEED);

// Initiates the retraction of the brake actuator (releases brakes)
void triggerBrakeRetract(uint8_t speed = BRAKE_MANUAL_SPEED);

// Stops the actuator immediately by setting PWM duty cycle to 0
void stopActuator();

// State machine handler for the brake; should be called frequently in the main loop.
// Stops the actuator after BRAKE_PULSE_MS.
void updateBrake();

// Returns true if the brake is considered to be applied/extended
bool isBrakeApplied();

#endif /* BRAKE_H */