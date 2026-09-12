#include <Arduino.h>
#include "ibus_receiver.h"
#include "steering.h"
#include "brake.h"
#include "throttle.h"

// (ARYAN) START: Added struct KartCommand to receive binary data from the laptop controller.
#pragma pack(push, 1) // Ensure 1-byte alignment for serial communication
struct KartCommand {
  int8_t  steering;   // -100 to 100
  uint8_t throttle;   // 0 to 100
  uint8_t brake;      // 0 or 1
};
#pragma pack(pop)
// (ARYAN) END: struct KartCommand

static IbusReceiver ibus;
bool rcControlMode = true;
unsigned long lastLaptopCommandTime = 0;

// Prevent repeated serial brake packets from retriggering
// the brake pulse. 0->1 extends, 1->0 retracts.
static uint8_t previousSerialBrake = 0; // Tracks the last time we received a command from the laptop

void processSerialCommands();
void printMenu();

void setup() {
  Serial.begin(115200);
  delay(200);

  setupBrake();
  triggerBrakeRetract(BRAKE_MANUAL_SPEED);
  delay(2000);

  ibus.begin();
  setupSteering();
  setupThrottle();

  printMenu();
}

void loop() {
  ibus.poll();

  // (ARYAN) START: Implemented Channel 6 as a Master Switch with hysteresis for switching between RC and Laptop modes.
  // --- Master Switch: Channel 6 for RC vs Laptop Mode ---
  // ONLY update the mode when the RC transmitter is actually connected.
  // When RC is off (failsafe), we do NOT change the mode — emergency stop handles it.
  // Uses channelReceived() to read the actual raw switch value, not the failsafe-substituted one.
  if (!ibus.isFailsafe()) {
    uint16_t chMode = ibus.channelReceived(6);
    if (chMode > 1650) {
      rcControlMode = false;
        previousSerialBrake = 0;  // Switch UP → Laptop Mode
    } else if (chMode < 1350) {
      rcControlMode = true;   // Switch DOWN → RC Mode (default)
    }
  }
  // (ARYAN) END: Master Switch for RC vs Laptop Mode

  //--- Brake Actuator (Channel 5) - Active in BOTH RC and Laptop modes ---
  // Physical RC Channel 5 is a universal safety override and must ALWAYS work whenever RC is connected
  if (!ibus.isFailsafe()) {
    uint16_t chBrake = ibus.channel(5);

    static bool stickWasUp = false;
    static bool stickWasDown = false;

    bool stickIsUp   = (chBrake > 1650);
    bool stickIsDown = (chBrake < 1350);

    if (stickIsUp && !stickWasUp) {
      triggerBrakeExtend(BRAKE_MANUAL_SPEED);
    } 
    else if (stickIsDown && !stickWasDown) {
      triggerBrakeRetract(BRAKE_MANUAL_SPEED);
    }

    stickWasUp   = stickIsUp;
    stickWasDown = stickIsDown;
  }

  // --- Requirement 2: Encoder Disconnection / Failure Protection ---
  // Throttle killed & steering PID disabled. Brake untouched.
  if (isEncoderFault()) {
    disablePID();
    stopThrottle();
  } 
  // --- Requirement 1: Controller Off / Loss of Signal Failsafe ---
  // Throttle killed & steering PID disabled. Brake untouched.
  else if (ibus.isFailsafe()) {
    disablePID();
    stopThrottle();
  } 
  // --- Laptop Watchdog Failsafe ---
  // If we are in laptop mode and haven't received a command in 500ms, the laptop crashed or disconnected.
  else if (!rcControlMode && (millis() - lastLaptopCommandTime > 500)) {
    disablePID();
    stopThrottle();
  }
  else if (rcControlMode) {
    //--- Steering (Channel 1) ---
    float rawSteerNorm = ibus.channelNormalised(1);
    if (abs(rawSteerNorm) < 0.03f) rawSteerNorm = 0.0f;

    static float filteredSteerNorm = 0.0f;
    const float alpha = 0.15f;
    filteredSteerNorm = filteredSteerNorm + alpha * (rawSteerNorm - filteredSteerNorm);

    long targetPos = (long)(filteredSteerNorm * (filteredSteerNorm >= 0 ? MAX_POS_LIMIT : abs(MIN_POS_LIMIT)));

    // Drive PID to live RC target position
    enablePID(targetPos);

    // --- Throttle Safety Cutoff ---
    if (isBrakeApplied()) {
      stopThrottle();
    } else {
      float throttleNorm = ibus.channelUnipolar(3);
      setThrottleNormalized(throttleNorm);
    }
  }
  else {
    // Laptop Mode (active commands coming in via processSerialCommands())
    if (isBrakeApplied()) {
      stopThrottle();
    }
  }

  // --- Subsystem Background Tasks ---
  updatePID();
  updateBrake();
  checkManualLimits();
  processSerialCommands();

  // Telemetry (10 Hz)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();

    Serial.printf("RC Mode: %s | FS: %s | Enc Fault: %s | CH1 Steer: %04d | CH3 Thr: %04d | Steer Pos: %ld | Target: %ld | Brake: %s (%lums) | BRAKE_VALUE: %d\n",
                  rcControlMode ? "RC" : "AUTONOMOUS",
                  ibus.isFailsafe() ? "RC OFF" : "RC ON",
                  isEncoderFault() ? "YES" : "NO",
                  ibus.channel(1),
                  ibus.channel(3),
                  getEncoderCount(),
                  targetPosition,
                  isBrakeApplied() ? "ENGAGED" : "RELEASED",
                  getBrakePositionMs(),
                  ibus.channel(5));
  }
}

// (ARYAN) START: Rewrote processSerialCommands to handle a 3-byte binary struct instead of individual character commands.
void processSerialCommands() {
  if (Serial.available() <= 0) return;

  // Check if we have enough bytes for our binary struct
  if (Serial.available() >= sizeof(KartCommand)) {
    KartCommand cmd;
    Serial.readBytes((char*)&cmd, sizeof(KartCommand));
    
    // (ARYAN) START: Fixed Failsafe Bug. Laptop is now completely blocked if the RC is turned off or if there is an encoder fault.
    // Only apply if we are NOT in RC Mode AND the system is completely safe
    if (!rcControlMode && !ibus.isFailsafe() && !isEncoderFault()) {
    // (ARYAN) END: Fixed Failsafe Bug
      
      // Update the watchdog timer because we received a valid command!
      lastLaptopCommandTime = millis();
      
      // 1. Steering
      float steerNorm = cmd.steering / 100.0f;
      long targetPos = (long)(steerNorm * (steerNorm >= 0 ? MAX_POS_LIMIT : abs(MIN_POS_LIMIT)));
      enablePID(targetPos);

      // 2. Brake (RC brake switch takes priority over laptop commands)
      bool rcBrakeEngaged = (!ibus.isFailsafe() && ibus.channel(5) > 1650);
      if (!rcBrakeEngaged) {
        if (cmd.brake == 1 && previousSerialBrake == 0) {
          triggerBrakeExtend(BRAKE_MANUAL_SPEED);
        }
        else if (cmd.brake == 0 && previousSerialBrake == 1) {
          triggerBrakeRetract(BRAKE_MANUAL_SPEED);
        }
      }

      previousSerialBrake = cmd.brake;

      // 3. Throttle
      if (isBrakeApplied() || cmd.brake == 1) {
        stopThrottle();
      } else {
        float throttleNorm = cmd.throttle / 100.0f;
        setThrottleNormalized(throttleNorm);
      }
    }
  } else {
    // If it's a single byte, it might be a calibration/safety command
    char cmd = Serial.read();
    cmd = toupper(cmd);
    
    switch (cmd) {
      case 'C':
        clearEncoderFault();
        enablePID(0); 
        Serial.println("[CMD] Centering Steering (Targeting Position 0)...");
        break;
      case 'Z':
        zeroEncoderCount();
        targetPosition = 0;
        disablePID();
        stopActuator();
        resetBrakePosition(0);
        stopThrottle();
        Serial.println("[CMD] Encoder ZEROED, Brake position zeroed, and fault cleared.");
        break;
      case 'S':
        stopSteeringMotor();
        stopActuator();
        stopThrottle();
        Serial.println("[CMD] STOP All Motors & Throttle");
        break;
      default:
        break;
    }
  }

  // (ARYAN) START: Fixed Whitespace Bug. Deleted the while-loop that was accidentally deleting binary values like 10 (\n) and 13 (\r).
  // (ARYAN) END: Fixed Whitespace Bug
}
// (ARYAN) END: Rewrote processSerialCommands

// (ARYAN) START: Updated the print menu to reflect the new hybrid RC/Laptop control system and removed old character commands.
void printMenu() {
  Serial.println("\n==================================================");
  Serial.println("   Integrated RC Vehicle Steering, Brake & Throttle");
  Serial.println("==================================================");
  Serial.println("--- Operating Mode ---");
  Serial.println("  Toggle Channel 6 on RC Remote to switch modes");
  Serial.println("  (Laptop control via 3-byte binary struct)");
  Serial.println("--- System Calibration Controls (Keyboard) ---");
  Serial.println("  C     : Center Steering (PID Drive to Position 0)");
  Serial.println("  Z     : ZERO Encoder Count & Clear Fault");
  Serial.println("  S     : STOP All Motors & Throttle");
  Serial.println("==================================================\n");
}
// (ARYAN) END: Updated the print menu