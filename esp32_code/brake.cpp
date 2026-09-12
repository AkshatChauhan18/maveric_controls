#include "brake.h"

// Internal state tracking for the brake actuator
static BrakeState currentState = BRAKE_IDLE;
static unsigned long lastUpdateTime = 0;    // Timestamp when position was last updated
static unsigned long currentPositionMs = 0; // Estimated position (0 = fully retracted, BRAKE_PULSE_MS = fully extended)
static bool brakeApplied = false;           // Logical state tracking

// Internal helper to update currentPositionMs based on elapsed time since last update
static void updatePosition() {
  if (currentState == BRAKE_IDLE) {
    lastUpdateTime = millis();
    return;
  }

  unsigned long now = millis();
  unsigned long dt = now - lastUpdateTime;
  lastUpdateTime = now;

  if (currentState == BRAKE_EXTENDING) {
    currentPositionMs += dt;
    if (currentPositionMs >= BRAKE_PULSE_MS) {
      currentPositionMs = BRAKE_PULSE_MS;
    }
  } else if (currentState == BRAKE_RETRACTING) {
    if (dt >= currentPositionMs) {
      currentPositionMs = 0;
    } else {
      currentPositionMs -= dt;
    }
  }
}

// Initializes the brake actuator hardware
void setupBrake() {
  // Configure motor driver enable pins as outputs
  pinMode(BRAKE_R_EN_PIN, OUTPUT);
  pinMode(BRAKE_L_EN_PIN, OUTPUT);

  // Configure hardware brake limit switch pin with internal pullup
  pinMode(BRAKE_LIMIT_PIN, INPUT_PULLUP);

  // Enable the driver chips continuously
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Attach PWM functionality to the control pins using ESP32 LEDC API
  ledcAttachChannel(BRAKE_RPWM_PIN, BRAKE_PWM_FREQ, BRAKE_PWM_RESOLUTION, BRAKE_RPWM_CHANNEL);
  ledcAttachChannel(BRAKE_LPWM_PIN, BRAKE_PWM_FREQ, BRAKE_PWM_RESOLUTION, BRAKE_LPWM_CHANNEL);

  // Ensure the actuator starts in a safe, unpowered state
  // stopActuator();
  currentPositionMs = 0;
}

// Commands the actuator to extend (apply the brake)
void triggerBrakeExtend(uint8_t speed) {
  brakeApplied = true;

  // Update accumulated position from previous state before changing direction
  updatePosition();

  // Check hardware limit switch pin (active LOW)
  if (digitalRead(BRAKE_LIMIT_PIN) == LOW) {
    currentPositionMs = BRAKE_PULSE_MS;
    stopActuator();
    return;
  }

  // Prevent extending if already at maximum position limit
  if (currentPositionMs >= BRAKE_PULSE_MS) {
    stopActuator();
    return;
  }

  // Prevent redundant calls if already extending
  if (currentState == BRAKE_EXTENDING) return;

  // Ensure motor driver is enabled
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Drive the motor in the 'extend' direction
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, speed);

  // Record start time and update state machine
  lastUpdateTime = millis();
  currentState = BRAKE_EXTENDING;
}

// Commands the actuator to retract (release the brake)
void triggerBrakeRetract(uint8_t speed) {
  brakeApplied = false;

  // Update accumulated position from previous state before changing direction
  updatePosition();

  // this was commented as it was stoping the brake actuator to go to its rest position
  // If already at fully retracted position (0ms), no need to run motor further
  // if (currentPositionMs == 0) {
  //   stopActuator();
  //   return;
  // }

  // Prevent redundant calls if already retracting
  if (currentState == BRAKE_RETRACTING) return;

  // Ensure motor driver is enabled
  digitalWrite(BRAKE_R_EN_PIN, HIGH);
  digitalWrite(BRAKE_L_EN_PIN, HIGH);

  // Drive the motor in the 'retract' direction
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, speed);

  // Record start time and update state machine
  lastUpdateTime = millis();
  currentState = BRAKE_RETRACTING;
}

// Halts any ongoing movement of the brake actuator
void stopActuator() {
  // Update position for any movement prior to stopping
  updatePosition();

  // Set both PWM channels to 0 to cut power to the motor
  ledcWriteChannel(BRAKE_RPWM_CHANNEL, 0);
  ledcWriteChannel(BRAKE_LPWM_CHANNEL, 0);
  currentState = BRAKE_IDLE;
}

// Non-blocking update function to manage brake action durations
void updateBrake() {
  if (currentState == BRAKE_IDLE) return;

  // Update position based on elapsed movement
  updatePosition();

  if (currentState == BRAKE_EXTENDING) {
    // Check hardware limit switch
    if (digitalRead(BRAKE_LIMIT_PIN) == LOW) {
      currentPositionMs = BRAKE_PULSE_MS;
      stopActuator();
      return;
    }

    if (currentPositionMs >= BRAKE_PULSE_MS) {
      stopActuator();
    }
  } else if (currentState == BRAKE_RETRACTING) {
    if (currentPositionMs == 0) {
      stopActuator();
    }
  }
}

// Returns the logical state of the brake (applied or not)
bool isBrakeApplied() {
  return brakeApplied && (currentPositionMs > 0);
}

// Returns current estimated position in ms
unsigned long getBrakePositionMs() {
  updatePosition();
  return currentPositionMs;
}

// Resets estimated brake position
void resetBrakePosition(unsigned long posMs) {
  currentPositionMs = (posMs > BRAKE_PULSE_MS) ? BRAKE_PULSE_MS : posMs;
}