# Autonomous Go-Kart Controller System

UPDATED : 12 September 2026


A robust, safety-critical drive-by-wire and ROS 2 control system for an Ackermann-steering electric go-kart / autonomous ground vehicle (AGV). 

This repository contains:
1. **ESP32 Firmware (`esp32_code/`)**: Embedded real-time control system for closed-loop steering PID, DAC throttle output, linear actuator braking, FlySky iBUS radio telemetry, and failsafe enforcement.
2. **ROS 2 Package (`motor_controller/`)**: Production-ready ROS 2 package bridging standard `/cmd_vel` (`geometry_msgs/msg/Twist`) navigation messages to the vehicle's low-latency 3-byte binary serial protocol using Ackermann kinematics and input slew-rate filtering.
3. **Standalone Teleoperation Utility (`laptop_controller.cpp`)**: Zero-dependency C++ keyboard teleop for testing the vehicle directly over USB-UART without spinning up a full ROS 2 stack.

---

## Table of Contents

- [1. System Architecture](#1-system-architecture)
- [2. Hardware Pinout & Peripherals](#2-hardware-pinout--peripherals)
- [3. Operating Modes & Control Hierarchy](#3-operating-modes--control-hierarchy)
- [4. Multi-Layer Safety & Failsafe Logic](#4-multi-layer-safety--failsafe-logic)
- [5. Serial Wire Protocol & Telemetry](#5-serial-wire-protocol--telemetry)
- [6. Repository Structure](#6-repository-structure)
- [7. ESP32 Firmware Deep Dive](#7-esp32-firmware-deep-dive)
- [8. ROS 2 `motor_controller` Package Deep Dive](#8-ros-2-motor_controller-package-deep-dive)
- [9. Standalone Laptop Controller](#9-standalone-laptop-controller)
- [10. Build & Installation Guide](#10-build--installation-guide)
- [11. Testing & Pre-Flight Verification](#11-testing--pre-flight-verification)

---

## 1. System Architecture

```
                                  +---------------------------------------------------+
                                  |            FlySky RC Transmitter (iBUS)           |
                                  |  - Ch1: Steer  - Ch3: Throttle  - Ch5: E-Brake    |
                                  |  - Ch6: Master Mode (RC Mode vs Autonomous Mode)  |
                                  +---------------------------------------------------+
                                                            | (2.4 GHz RF Link)
                                                            v
+------------------------------------+             +----------------------------------+
|      High-Level Compute            |             |   FlySky FS-iA6B Receiver        |
|  (ROS 2 Navigation / Laptop Node)  |             +----------------------------------+
|                                    |                              | (iBUS Serial @ 115200)
|  Publishes: /cmd_vel (Twist)       |                              |
|  Maps: Ackermann Kinematics        |                              v
|  Outputs: 3-Byte Binary Struct     |                 [ Pin 34: UART2 RX ]
+------------------------------------+                              |
                  |                                                 |
                  | (USB-UART @ 115200 Baud)                        |
                  +-----------------------+                         |
                                          v                         v
                           +-----------------------------------------------+
                           |            ESP32 Microcontroller              |
                           |                                               |
                           |  - Master Mode Selector (Channel 6)           |
                           |  - Universal RC Brake Override (Channel 5)    |
                           |  - Watchdog & Encoder Stall Detectors         |
                           |  - Steering Position PID via PCNT Hardware    |
                           |  - DAC Voltage Generator for EV Throttle      |
                           |  - Timed H-Bridge Actuator for Braking        |
                           +-----------------------------------------------+
                                 |                 |                 |
                +----------------+                 |                 +----------------+
                |                                  |                                  |
                v                                  v                                  v
+--------------------------------+ +--------------------------------+ +--------------------------------+
|       Steering Subsystem       | |       Throttle Subsystem       | |        Brake Subsystem         |
|                                | |                                | |                                |
| - BTS7960 Driver (PWM/DIR)     | | - ESP32 Internal DAC (Pin 25)  | | - BTS7960 H-Bridge (PWM L/R)   |
| - Optical Quadrature Encoder   | | - Linear 0.8V - 3.3V Output    | | - Linear Brake Actuator        |
|   read via ESP32 PCNT          | | - Electric Vehicle Motor       | | - Hardware Limit Switch        |
| - Closed-loop PID control      | |   Speed Controller (e.g. Kelly)| | - Timed Stroke Tracking        |
+--------------------------------+ +--------------------------------+ +--------------------------------+
```

---

## 2. Hardware Pinout & Peripherals

The system is hosted on an **ESP32 DevKit** board interfacing directly with the kart's physical actuation hardware and sensors.

### Pinout Mapping

| ESP32 GPIO | Peripheral / Mode | Target Hardware | Function / Description |
| :--- | :--- | :--- | :--- |
| **GPIO 34** | UART2 RX (Input Only) | FS-iA6B iBUS Out | Receives 32-byte 14-channel serial RC frames at 115200 baud. |
| **GPIO 26** | LEDC PWM Channel 0 | BTS7960 PWM | Steering motor PWM speed control (5 kHz frequency, 8-bit resolution). |
| **GPIO 27** | Digital Output | BTS7960 DIR | Steering motor direction signal (`HIGH` = Decreases Count / Left, `LOW` = Right). |
| **GPIO 14** | Pulse Counter (PCNT) | Optical Encoder Ch A | Steering feedback quadrature channel A. |
| **GPIO 4**  | Pulse Counter (PCNT) | Optical Encoder Ch B | Steering feedback quadrature channel B. |
| **GPIO 25** | DAC Output (8-bit) | EV Motor Controller | Analog throttle input: `0V` = Stop, `~0.8V` (`DAC 62`) = Idle, `~3.3V` (`DAC 255`) = Full Throttle. |
| **GPIO 16** | LEDC PWM Channel 2 | BTS7960 RPWM | Brake actuator extend control (10 kHz PWM, applies brakes). |
| **GPIO 17** | LEDC PWM Channel 3 | BTS7960 LPWM | Brake actuator retract control (10 kHz PWM, releases brakes). |
| **GPIO 5**  | Digital Output | BTS7960 R_EN | Right enable for brake motor driver (`HIGH` = Enabled). |
| **GPIO 18** | Digital Output | BTS7960 L_EN | Left enable for brake motor driver (`HIGH` = Enabled). |
| **GPIO 19** | Digital Input (Pullup) | Limit Switch | Brake maximum stroke limit switch (Active `LOW`). |
| **GPIO 1/3**| UART0 (TX/RX) | PC / Onboard USB | Primary serial stream for 3-byte binary commands and 10 Hz ASCII telemetry. |

---

## 3. Operating Modes & Control Hierarchy

Operating mode arbitration is handled by the ESP32 using **Channel 6** of the physical FlySky RC transmitter.

### Mode Selection Matrix

| RC Channel 6 State | Transmitter Link | Active Control Mode | Behavior & Authority |
| :--- | :--- | :--- | :--- |
| **DOWN (< 1350 µs)** | Connected (Signal OK) | **RC Mode (Default)** | Rover is driven manually via the RC transmitter. Serial commands from the laptop / ROS 2 are completely ignored. |
| **UP (> 1650 µs)** | Connected (Signal OK) | **Autonomous / Laptop Mode** | Rover responds to 3-byte binary commands over USB serial. RC sticks (Ch1/Ch3) are ignored, but **RC Brake (Ch5) retains override**. |
| **Any Position** | **Disconnected / Powered OFF** | **Emergency Failsafe** | **Autonomous and RC modes are both blocked.** Steering PID is killed, throttle is zeroed, and brakes are engaged. |

> [!IMPORTANT]
> **RC Transmitter as a Physical Dead-Man Key:**
> Autonomous mode can **NEVER** run if the RC transmitter is turned off or out of range. If the remote loses connection while running in Autonomous mode, the system immediately trips into emergency failsafe.

---

## 4. Multi-Layer Safety & Failsafe Logic

Safety is built in depth across both the host software and micro-controller firmware:

1. **RC Link Failsafe (`ibus.isFailsafe()`)**:
   - The iBUS receiver layer enforces a 100 ms frame timeout.
   - If transmitter frames stop arriving (RC turned off or out of range), failsafe is asserted: throttle is cut to 0, steering PID is disabled, and movement is blocked.
2. **Universal RC Brake Override (Channel 5)**:
   - Physical RC Channel 5 is a universal safety override.
   - Even in Autonomous/Laptop mode (Channel 6 UP), flipping Channel 5 DOWN applies the brakes and cuts the throttle instantly.
3. **Autonomous Watchdog Timer**:
   - In Autonomous mode, the ESP32 expects valid 3-byte serial packets at least every 500 ms.
   - If the laptop script crashes, freezes, or disconnects, the watchdog immediately disables steering PID and sets throttle to zero.
4. **Steering Encoder Stall & Disconnection Fault Detection**:
   - If the steering motor is commanded with non-zero PWM but the encoder count fails to change within `ENCODER_TIMEOUT_MS = 100 ms`, the firmware raises an `encoderFault`.
   - Throttle is immediately cut and steering PID is shut down to prevent motor burnout, mechanical stripping, or open-loop runaways.
5. **Throttle-Brake Interlock**:
   - Whenever brakes are engaged (`cmd.brake == 1` or `isBrakeApplied()`), the throttle is strictly locked to 0V / minimum DAC value.
6. **Host Communication Failure Detection**:
   - If the serial cable is unplugged while the ROS 2 node or laptop controller is driving, the host detects write failure, attempts an emergency zero-packet `{steering: 0, throttle: 0, brake: 1}`, and logs a critical error.

---

## 5. Serial Wire Protocol & Telemetry

Communication between the host computer (ROS 2 / laptop) and the ESP32 operates over standard UART at **115200 baud, 8N1**.

### Downlink: Host -> ESP32 (`KartCommand`)

The firmware receives packed 3-byte binary frames without variable headers to minimize latency:

```cpp
#pragma pack(push, 1)
struct KartCommand {
    int8_t  steering;   // -100 to 100 (maps to encoder target -20000 to +20000)
    uint8_t throttle;   // 0 to 100    (maps to DAC value 62 to 255)
    uint8_t brake;      // 0 (released) or 1 (engaged)
};
#pragma pack(pop)
static_assert(sizeof(KartCommand) == 3, "KartCommand must be exactly 3 bytes");
```

- **Transmission Rate**: 20 Hz (every 50 ms).
- **Steering**: `-100` (Full Left) to `+100` (Full Right).
- **Throttle**: `0` (Idle, ~0.8V DAC) to `100` (Max speed, ~3.3V DAC).
- **Brake**: `0` (Retract linear actuator) or `1` (Extend linear actuator).

### Calibration & Diagnostic ASCII Commands

When a single ASCII byte is sent (e.g. via serial terminal):
- `'C'`: Centers the steering (drives PID to position 0).
- `'Z'`: Zeroes the encoder count, resets the estimated brake position, and clears encoder faults.
- `'S'`: Emergency stop (kills all motors, brakes, and throttle).

### Uplink: ESP32 -> Host (Telemetry Stream)

The ESP32 broadcasts human-readable telemetry at 10 Hz:
```
RC Mode: <RC|AUTONOMOUS> | FS: <RC ON|RC OFF> | Enc Fault: <YES|NO> | CH1 Steer: <pwm> | CH3 Thr: <pwm> | Steer Pos: <ticks> | Target: <ticks> | Brake: <ENGAGED|RELEASED> (<ms>) | BRAKE_VALUE: <ch5>
```

The ROS 2 node continuously parses this telemetry to populate diagnostics and warn operators if `/cmd_vel` is being discarded due to RC mode or failsafe.

---

## 6. Repository Structure

```
.
├── .gitignore
├── README.md                            # Comprehensive system documentation
│
├── esp32_code/                          # ESP32 Arduino / C++ Firmware
│   ├── esp32_code.ino                   # Main Arduino sketch (loop, watchdog, serial dispatch)
│   ├── ibus_config.h                    # iBUS UART and channel threshold configuration
│   ├── ibus_receiver.h                  # iBUS protocol parser class declaration
│   ├── ibus_receiver.cpp                # Non-blocking iBUS frame decoder & checksum validator
│   ├── steering.h                       # Steering PID & PCNT hardware driver headers
│   ├── steering.cpp                     # Pulse counter quadrature decoding, PID loop, fault detection
│   ├── throttle.h                       # DAC pin definitions and voltage thresholds
│   ├── throttle.cpp                     # Safe DAC throttle scaling and zeroing logic
│   ├── brake.h                          # BTS7960 brake actuator driver header
│   ├── brake.cpp                        # Timed stroke state machine & limit switch integration
│   ├── laptop_controller.cpp            # Standalone terminal keyboard teleop program
│   ├── kart_remote                      # Precompiled macOS (Apple Silicon) binary for laptop teleop
│   └── FINAL_CHECKLIST.md               # Safety audit & pre-flight test checklist
│
└── motor_controller/                    # ROS 2 Colcon Package
    ├── package.xml                      # ROS 2 dependencies (rclcpp, geometry_msgs, diagnostic_updater)
    ├── CMakeLists.txt                   # ament_cmake build rules & test targets
    ├── config/
    │   └── motor_controller.yaml        # Kart kinematics, slew rates, serial port parameters
    ├── include/
    │   └── motor_controller/
    │       └── kart_command.hpp         # Pure C++ Ackermann kinematic model & telemetry parser
    ├── launch/
    │   └── motor_controller.launch.py   # Launch script with parameter substitution
    ├── src/
    │   └── motor_controller_node.cpp    # ROS 2 node: /cmd_vel subscriber, serial writer, diagnostics
    └── test/
        └── test_kart_command.cpp        # Standalone kinematic & packing unit test suite
```

---

## 7. ESP32 Firmware Deep Dive

Located in `esp32_code/`, the firmware is built using the Arduino-ESP32 framework.

### Steering Subsystem (`steering.cpp` / `steering.h`)
- **Hardware Quadrature Decoding**: Leverages ESP32 hardware Pulse Counter (PCNT) peripheral (`driver/pulse_cnt.h`) across GPIO 14 and GPIO 4 to track optical encoder counts without wasting CPU cycles on GPIO interrupts.
- **Closed-Loop PID Control**:
  - Proportional Gain: $K_p = 0.20$
  - Integral Gain: $K_i = 0.001$ (clamped with anti-windup to $\pm 1000$)
  - Derivative Gain: $K_d = 0.02$
  - Output deadband of 10 counts and static friction compensation (`minPWM = 25`, `maxPWM = 150`).
- **Encoder Stall Detection**: If commanded motor PWM is active but no encoder ticks occur for $> 100\text{ ms}$, the system trips an encoder fault, cuts throttle, and disables the PID loop.

### Throttle Subsystem (`throttle.cpp` / `throttle.h`)
- Uses ESP32 internal 8-bit DAC on **GPIO 25**.
- `THROTTLE_MIN_DAC = 62` maps to $\approx 0.8\text{ V}$ (safe EV controller idle threshold).
- `THROTTLE_MAX_DAC = 255` maps to $\approx 3.3\text{ V}$ (full speed).
- `stopThrottle()` unconditionally forces 0 V on the pin.

### Brake Subsystem (`brake.cpp` / `brake.h`)
- Drives a high-force linear actuator via a BTS7960 H-bridge using ESP32 LEDC PWM hardware (10 kHz).
- **Position Tracking**: Tracks stroke duration up to `BRAKE_PULSE_MS = 1500 ms` to avoid burning out the actuator motor while ensuring full mechanical clamping.
- **Limit Switch**: Reads GPIO 19 (`INPUT_PULLUP`). When triggered (`LOW`), immediately stops extension and locks position to maximum.

### iBUS Receiver Subsystem (`ibus_receiver.cpp` / `ibus_receiver.h`)
- Reads 32-byte FlySky serial packets over UART2 (`GPIO 34`).
- Validates two-byte header (`0x20 0x40`) and standard 16-bit inverted checksum.
- Implements frame-rate calculation and auto-resynchronization on byte gaps $> 3000\ \mu\text{s}$.

---

## 8. ROS 2 `motor_controller` Package Deep Dive

The `motor_controller` package bridges high-level ROS 2 navigation planners to the ESP32.

### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/msg/Twist`): Linear velocity ($v_x$ in m/s) and angular velocity ($\omega_z$ in rad/s).

### Published Topics
- `/kart_command` (`std_msgs/msg/Float32MultiArray`): Mirror of commanded values `[steering_pct, throttle_pct, brake_state]`.
- `/diagnostics` (`diagnostic_updater/DiagnosticArray`): Comprehensive hardware diagnostics, including serial port health, command watchdog, telemetry rate, and RC transmitter status.

### Kinematics & Ackermann Mapping (`kart_command.hpp`)
1. **Dynamic Ackermann Steering ($v \ge v_{\min}$)**:
   $$\delta = \arctan\left(\frac{L \cdot \omega_z}{|v_x|}\right)$$
   Where $L$ is the wheelbase (default $1.20\text{ m}$). The angle $\delta$ is then normalized against `max_steer_angle` ($0.52\text{ rad} \approx 30^\circ$) to produce a percentage between $-100\%$ and $+100\%$.
2. **Standstill / Low-Speed Steering ($v < v_{\min}$)**:
   Below `ackermann_min_speed` ($0.2\text{ m/s}$), the singularity is bypassed by mapping $\omega_z$ directly as a steering angle command:
   $$\delta = \text{clamp}\left(\frac{\omega_z}{\omega_{z,\max}}, -1.0, 1.0\right) \cdot \delta_{\max}$$
   This allows teleop operators and planners to point the wheels while the vehicle is stopped.
3. **Negative Velocity / Reversing**:
   Because the electric kart hardware lacks a reverse gear or reverse controller signal, any negative linear velocity request ($v_x < -\text{deadband}$) is translated into an active **brake command**.
4. **Explicit Zero Command**:
   A commanded linear velocity of $0.0\text{ m/s}$ triggers active braking (`brake = 1`) to prevent rollaway.

### Slew Rate Limiting & Input Filtering
- **Steering Slew Rate**: Limits maximum steering velocity to `steer_slew_limit` (default $15\%\text{ per tick} = 300\%\text{/s}$ at 20 Hz).
- **Steering EMA Filter**: Exponential Moving Average filter with time constant $\tau = 0.1\text{ s}$ suppresses noise from high-frequency planners or keyboard teleop.
- **Asymmetric Throttle Slew Rate**:
  - Acceleration: `throttle_accel_limit = 5.0` ($100\%\text{/s}$ ramp-up for smooth starts).
  - Deceleration: `throttle_decel_limit = 100.0` (instant throttle release for safety).

### Configurable Parameters (`motor_controller.yaml`)

```yaml
motor_controller:
  ros__parameters:
    port: /dev/ttyUSB0
    baud: 115200
    wheelbase: 1.20            # meters, front axle to rear axle
    max_steer_angle: 0.52      # radians of road wheel at 100% steering
    max_speed: 1.5             # m/s reached at 100% throttle
    max_angular_z: 1.0         # rad/s mapped to full lock at zero speed
    ackermann_min_speed: 0.2   # m/s threshold for Ackermann model
    speed_deadband: 0.02       # m/s deadband
    invert_steering: false     # invert steering direction if needed
    brake_on_zero_cmd: false   # apply brake across deadband
    throttle_accel_limit: 5.0  # % per tick (at 20 Hz, 5% = 100%/sec)
    throttle_decel_limit: 100.0# % per tick
    steer_slew_limit: 15.0     # % per tick
    steer_filter_tau: 0.1      # EMA time constant (seconds)
    cmd_timeout_ms: 750.0      # Watchdog timeout for /cmd_vel
    control_rate_hz: 20.0      # Main control loop frequency
    debug_serial: false        # Log tx/rx packets to console
```

---

## 9. Standalone Laptop Controller

For direct testing without running ROS 2, `esp32_code/laptop_controller.cpp` provides a standalone terminal interface.

### Features
- Auto-detects ESP32 serial ports across Linux (`/dev/ttyUSB*`, `/dev/ttyACM*`) and macOS (`/dev/cu.usbserial*`, `/dev/cu.wchusbserial*`).
- Sets terminal to non-blocking raw mode.
- Sends 3-byte binary commands at 20 Hz.
- Implements auto-centering steering decay and automatic throttle ramp-down when keys are released.

### Key Bindings

| Key | Action |
| :--- | :--- |
| `W` | Accelerate / Increase Throttle (Ramps down when released) |
| `A` | Steer Left (Auto-centers when released) |
| `D` | Steer Right (Auto-centers when released) |
| `Space` | Engage Brakes (Hold to brake) |
| `Q` | Quit controller cleanly and send stop packet |

---

## 10. Build & Installation Guide

### Prerequisites
- **For ESP32 Firmware**: Arduino IDE or `arduino-cli` with the `esp32` board package installed.
- **For ROS 2 Package**: ROS 2 (Humble, Iron, or Rolling) installed on Linux/macOS.
- **For Laptop Controller**: Standard C++17 compiler (`g++` or `clang++`).

### Flashing the ESP32 Firmware

1. Open `esp32_code/esp32_code.ino` in the Arduino IDE.
2. Under **Tools -> Board**, select **ESP32 Dev Module**.
3. Connect the ESP32 via USB and select the appropriate serial port.
4. Verify and Upload the sketch.

### Compiling & Running the Standalone Laptop Controller

To compile the C++ teleop utility from source:

```bash
cd esp32_code
g++ -std=c++17 -pthread laptop_controller.cpp -o kart_remote
./kart_remote
```

*(Note: On Apple Silicon Macs, the precompiled binary `./kart_remote` is already available in `esp32_code/`)*.

### Building & Running the ROS 2 Package

In your ROS 2 workspace (e.g. `~/ros2_ws`):

```bash
# 1. Copy or symlink the package into your colcon workspace src directory
mkdir -p ~/ros2_ws/src
ln -s /path/to/experimenting/motor_controller ~/ros2_ws/src/

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select motor_controller

# 3. Source the workspace overlay
source install/setup.bash

# 4. Launch the motor controller node
ros2 launch motor_controller motor_controller.launch.py
```

### Running Kinematic Unit Tests

The kinematics, ramp limiting, and wire packing can be verified without hardware:

```bash
# Via colcon:
colcon test --packages-select motor_controller

# Or directly compiling the test file:
g++ -std=c++17 -I motor_controller/include motor_controller/test/test_kart_command.cpp -o test_kart_command
./test_kart_command
# Output: "kart_command: all checks passed"
```

---

## 11. Testing & Pre-Flight Verification

Before testing the kart with motor power connected, follow the safety validation protocol outlined in [`esp32_code/FINAL_CHECKLIST.md`](esp32_code/FINAL_CHECKLIST.md):

1. **Remote Controller OFF Check**:
   - Turn RC transmitter **OFF**.
   - Verify ESP32 serial console reports `FS: RC OFF`.
   - Send commands from the laptop / ROS 2; verify steering and throttle remain completely disabled and brakes engage.
2. **Channel 6 UP/DOWN Mode Isolation**:
   - Turn RC transmitter **ON**.
   - Flip Channel 6 **DOWN**: Verify manual RC controls operate the steering and throttle; verify laptop commands are discarded.
   - Flip Channel 6 **UP**: Verify laptop / ROS 2 commands control steering and throttle; verify RC steering stick is ignored.
3. **Channel 5 Universal Brake Override**:
   - With Channel 6 **UP** (Autonomous Mode) and sending active throttle/steering commands from the laptop, flip RC Channel 5 **DOWN**.
   - Verify brakes engage immediately and throttle drops to 0.
4. **USB Disconnect Emergency Stop**:
   - While actively running `laptop_controller` or ROS 2, unplug the USB cable.
   - Verify host application detects serial write failure and exits cleanly, and the ESP32 watchdog trips within 500 ms.
