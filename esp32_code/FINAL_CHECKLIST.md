# Final Rover Testing Checklist

This document combines all conditions from the system audit (`checklist.md`) and the requirements file (`CONDITION.txt`). Use this checklist to verify that the rover's firmware and control scripts satisfy all safety and functional constraints.

## 1. Condition: Remote Controller OFF (Emergency Stop & Failsafe)
- **Setup:** Turn the physical remote controller to the **OFF** state. Attempt to send commands from the RC (which is off) and from the laptop (Autonomous Mode).
- **Desired Observation:** The rover must enter an immediate Emergency Stop state (`isFailsafe()` should trigger). The brakes are automatically engaged. Neither RC commands nor laptop commands should have any effect. Both RC and Autonomous modes are non-functional.

## 2. Condition: Remote Controller ON (Initial State)
- **Setup:** Turn the remote controller **ON**. Do not touch the mode switch (Channel 6) yet.
- **Desired Observation:** The RC initializes as the main and primary controller. Autonomous mode (laptop controller) commands are blocked and ignored. Brakes can automatically disengage when the rover/RC turns on (depending on the brake switch position).

## 3. Condition: Channel 6 UP (RC Mode - Default)
- **Setup:** Flip the Channel 6 switch on the remote controller to the **UP** position.
- **Desired Observation:** The rover is controlled exclusively by the RC controls (steering and throttle). Autonomous (laptop) commands are completely ignored. Note: This UP position behavior must be the default mode of the rover on boot.

## 4. Condition: Channel 6 DOWN (Autonomous/Laptop Mode)
- **Setup:** Flip the Channel 6 switch on the remote controller to the **DOWN** position.
- **Desired Observation:** The rover is controlled exclusively by the laptop (Autonomous mode). RC steering and throttle commands are completely ignored. The brake switch(channel 5) on the RC should still work.

## 5. Condition: Mutual Exclusion
- **Setup:** Toggle the Channel 6 switch between UP and DOWN while attempting to send commands simultaneously from both the RC and the laptop.
- **Desired Observation:** Only one controller can dictate movement at any given time. When Channel 6 is UP, only the RC controls the rover. When Channel 6 is DOWN, only the laptop controls the rover(Except for channel 5 - BrakesC). They must never conflict or control the rover simultaneously.

## 6. Condition: Autonomous Mode Dependency on RC Connection
- **Setup:** Switch to Autonomous mode (Channel 6 DOWN) and actively send commands from the laptop. While the rover is responding to the laptop, turn **OFF** the remote controller.
- **Desired Observation:** Autonomous mode must immediately stop working. The rover must go into an Emergency Stop state (brakes engaged, movement blocked). All further laptop commands are rejected because the RC is no longer connected.

## 7. Condition: Brake Switch Logic (Channel 5)
- **Setup:** Ensure the RC is connected. Toggle the brake switch (Channel 5) UP and DOWN. Also, try turning the RC ON and OFF, and the rover ON and OFF.
- **Desired Observation:** 
  - When Channel 5 is **UP**, brakes are **disengaged**.
  - When Channel 5 is **DOWN**, brakes are **engaged**.
  - When turning the RC **OFF**, the brakes should automatically **engage**.
  - When turning the RC **ON**, the brakes should **disengage** (assuming Channel 5 is UP).

## 8. Condition: Universal Brake Override (Brake over RC Mode / Auto Mode)
- **Setup:** Run the rover in RC Mode (Channel 6 UP). Flip the brake switch (Channel 5) DOWN. Then, switch to Autonomous Mode (Channel 6 DOWN) and flip the brake switch (Channel 5) DOWN while sending movement commands.
- **Desired Observation:** The brake switch (Channel 5) operates completely independently of the active control mode. Regardless of whether the rover is in RC mode or Autonomous mode, flipping Channel 5 DOWN must immediately apply the brakes and halt the rover.

## 9. Condition: Communication Failure Triggers Emergency Stop
- **Setup:** Start the laptop controller, verify it connects to the ESP32, then physically unplug the USB cable while the script is actively running.
- **Desired Observation:** The script should immediately detect a write error, attempt to send a zero-command (0 steering, 0 throttle, 0 brake) as a fallback, and then exit cleanly. This connection loss must result in an Emergency Stop on the rover side.
