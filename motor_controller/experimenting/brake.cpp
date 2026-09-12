#include "brake.h"

// Internal state tracking for the brake actuator
static BrakeState currentState = BRAKE_IDLE;
static unsigned long pulseStartTime = 0; // Timestamp when the current action started
static bool brakeApplied = false;        // Logical state tracking

// Initializes the brake actuator hardware
void setupBrake() {
  // Configure motor driver enable pins as outputs
  pinMode(BRAKE_R_EN_PIN, OUTPUT);
  pinMode(BRAKE_L_EN_PIN, OUTPUT);

  // Enable the driver chips continuously
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Attach PWM functionality to the control pins using ESP32 LEDC API
  ledcAttachChannel(BRAKE_RPWM_PIN, BRAKE_PWM_FREQ, BRAKE_PWM_RESOLUTION, BRAKE_RPWM_CHANNEL);
  ledcAttachChannel(BRAKE_LPWM_PIN, BRAKE_PWM_FREQ, BRAKE_PWM_RESOLUTION, BRAKE_LPWM_CHANNEL);

  // Ensure the actuator starts in a safe, unpowered state
  stopActuator();
}

// Commands the actuator to extend (apply the brake)
void triggerBrakeExtend(uint8_t speed) {
  // Mark the brake as logically applied
  brakeApplied = true;

  // Prevent restarting the timer and redundant calls if already extending
  if (currentState == BRAKE_EXTENDING) return;

  // Ensure motor driver is enabled
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Drive the motor in the 'extend' direction
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, speed);

  // Record the start time and update the state machine
  pulseStartTime = millis();
  currentState = BRAKE_EXTENDING;
}

// Commands the actuator to retract (release the brake)
void triggerBrakeRetract(uint8_t speed) {
  // Mark the brake as logically released
  brakeApplied = false;

  // Prevent restarting the timer and redundant calls if already retracting
  if (currentState == BRAKE_RETRACTING) return;

  // Ensure motor driver is enabled
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Drive the motor in the 'retract' direction
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, speed);

  // Record the start time and update the state machine
  pulseStartTime = millis();
  currentState = BRAKE_RETRACTING;
}

// Halts any ongoing movement of the brake actuator
void stopActuator() {
  // Set both PWM channels to 0 to cut power to the motor
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, 0);
  currentState = BRAKE_IDLE;
}

// Non-blocking update function to manage brake action durations
void updateBrake() {
  // Check if an active pulse timer has expired
  if (currentState != BRAKE_IDLE) {
    // If the time since the pulse started exceeds the configured maximum duration
    if (millis() - pulseStartTime >= BRAKE_PULSE_MS) {
      stopActuator(); // Automatically cuts PWM to 0A current draw to prevent damage
    }
  }
}

// Returns the logical state of the brake (applied or not)
bool isBrakeApplied() {
  return brakeApplied;
}