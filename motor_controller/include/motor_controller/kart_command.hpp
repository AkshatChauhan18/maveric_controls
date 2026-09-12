#ifndef MOTOR_CONTROLLER__KART_COMMAND_HPP_
#define MOTOR_CONTROLLER__KART_COMMAND_HPP_

// Pure Twist -> go-kart command mapping. No ROS, no serial, so it can be
// checked on a laptop without the vehicle (see test/test_kart_command.cpp).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace motor_controller
{

// Wire format expected by experimenting.ino::processSerialCommands().
// Three bytes, no header, no checksum, no terminator: the firmware frames
// purely on "three bytes are available". So this must always be written as ONE
// whole 3-byte write, and nothing else may ever be sent on this port. A stray
// 4th byte does not merely corrupt one packet -- the firmware re-reads a lone
// leftover byte as one of its single-char commands, and 'S' (== 83) is a
// perfectly plausible steering percentage, which would stop the motors
// mid-drive.
#pragma pack(push, 1)
struct KartCommand
{
  int8_t  steering;  // -100..100, scaled to steering encoder +-20000 on the ESP32
  uint8_t throttle;  // 0..100,    scaled to DAC 62..255 on the ESP32
  uint8_t brake;     // 0 or 1
};
#pragma pack(pop)
static_assert(sizeof(KartCommand) == 3, "KartCommand must match the ESP32 struct byte for byte");

// Every field here is a physical property of the kart, so every one of them is
// a parameter the vehicle has to be measured for -- the defaults are only
// enough to drive at walking pace without hurting anything.
struct KartLimits
{
  double wheelbase           = 1.20;  // m, front axle to rear axle
  double max_steer_angle     = 0.52;  // rad of road wheel at steering = +-100
  double max_speed           = 5.0;   // m/s the kart reaches at throttle = 100
  double max_angular_z       = 1.0;   // rad/s that maps to full lock at standstill
  double ackermann_min_speed = 0.2;   // m/s below which atan(L*w/v) is degenerate
  double speed_deadband      = 0.02;  // m/s treated as "no drive requested"
  bool   invert_steering     = false; // flip if positive angular.z steers the wrong way
  bool   brake_on_zero_cmd   = false; // also brake small positive residual velocities
};

// Twist -> steering percent (-100..100).
//
// Away from standstill, geometry converts ROS's body yaw rate into the front
// road-wheel angle: delta = atan(wheelbase * angular.z / linear.x).  The ESP32
// then converts this percentage to its encoder position.  Near zero speed the
// yaw-rate equation is singular, so angular.z is instead treated as a bounded
// direct steering request.  This lets teleop centre/position the wheels while
// stopped without producing an arbitrary full-lock command.
inline double steer_percent_from_twist(double v, double w, const KartLimits & lim)
{
  if (lim.max_steer_angle <= 0.0) {
    return 0.0;
  }

  double delta = 0.0;
  if (std::abs(v) >= lim.ackermann_min_speed && lim.wheelbase > 0.0) {
    // This kart has no reverse command; using |v| keeps steering direction
    // consistent with a stopped/reverse request, which will be braked below.
    delta = std::atan(lim.wheelbase * w / std::abs(v));
  } else if (lim.max_angular_z > 0.0) {
    delta = std::clamp(w / lim.max_angular_z, -1.0, 1.0) * lim.max_steer_angle;
  }

  double pct = (delta / lim.max_steer_angle) * 100.0;
  if (lim.invert_steering) {
    pct = -pct;
  }
  return std::clamp(pct, -100.0, 100.0);
}

// Twist -> throttle percent (0..100).
//
// Open loop: the ESP32 turns this straight into a DAC voltage for the EV
// controller and nothing anywhere measures road speed, so this is a
// feed-forward guess scaled by max_speed, not tracking of linear.x. Calibrate
// max_speed against GPS/wheel speed if the number needs to mean anything.
inline double throttle_percent_from_twist(double v, const KartLimits & lim)
{
  if (v < lim.speed_deadband || lim.max_speed <= 0.0) {
    return 0.0;
  }
  return std::clamp((v / lim.max_speed) * 100.0, 0.0, 100.0);
}

// Twist -> brake (0/1).
//
// The protocol has no reverse, so a negative linear.x is read as "stop" -- the
// only thing this kart can actually do about a backwards request, and better
// than ignoring it. An explicit zero-velocity command is also a stop request,
// so it applies the brake. brake_on_zero_cmd extends that braking behavior to
// tiny positive residual velocities inside speed_deadband.
inline bool brake_from_twist(double v, const KartLimits & lim)
{
  if (v < -lim.speed_deadband) {
    return true;  // backwards request -> stop
  }
  // An explicit zero-velocity Twist is a stop command, not a coast request:
  // always brake so the kart halts instead of rolling. brake_on_zero_cmd keeps
  // its meaning of braking across the whole deadband (tiny residual v).
  if (v <= 0.0) {
    return true;
  }
  return lim.brake_on_zero_cmd && v < lim.speed_deadband;
}

// Linear slew toward target, in percent per control tick. Accel and decel are
// separate because letting off must not be as gentle as applying: a slow ramp
// down is a kart that keeps driving after the command stopped.
inline double ramp_toward(double current, double target, double accel, double decel)
{
  const bool easing_off = std::abs(target) < std::abs(current) || (current * target < 0.0);
  const double limit = easing_off ? decel : accel;
  const double diff  = target - current;
  if (limit <= 0.0 || std::abs(diff) <= limit) {
    return target;
  }
  return (diff > 0.0) ? current + limit : current - limit;
}

// Round + saturate into the wire types. Done in one place so the packing can be
// checked without a serial port attached.
inline KartCommand pack_kart_command(double steer_pct, double throttle_pct, bool brake)
{
  KartCommand cmd;
  cmd.steering = static_cast<int8_t>(std::lround(std::clamp(steer_pct, -100.0, 100.0)));
  cmd.throttle = static_cast<uint8_t>(std::lround(std::clamp(throttle_pct, 0.0, 100.0)));
  cmd.brake    = brake ? 1 : 0;
  // The firmware cuts throttle whenever the brake is applied; mirror it here so
  // the byte on the wire never disagrees with the intent.
  if (cmd.brake) {
    cmd.throttle = 0;
  }
  return cmd;
}

}  // namespace motor_controller

// Telemetry parser: extracts structured status from firmware's human-readable lines.
// Keeps string matching in one tested place so firmware version changes don't
// silently break RC mode / failsafe / encoder fault detection.
namespace motor_controller
{

struct TelemetryStatus
{
  bool rc_mode_active      = false;
  bool rc_failsafe_active  = false;
  bool encoder_fault_active = false;
  bool valid               = false;  // true if line contained recognizable telemetry
};

// Parses one firmware telemetry line. Returns status with valid=false for
// unrecognized lines (banners, menus, command echoes).
inline TelemetryStatus parse_telemetry(const std::string & line)
{
  TelemetryStatus status;
  if (line.find("RC Mode:") == std::string::npos) {
    return status;  // not a telemetry line
  }
  status.valid = true;
  status.rc_mode_active      = (line.find("RC Mode: ACTIVE") != std::string::npos);
  status.rc_failsafe_active  = (line.find("FS: YES") != std::string::npos);
  status.encoder_fault_active = (line.find("Enc Fault: YES") != std::string::npos);
  return status;
}

}  // namespace motor_controller

#endif  // MOTOR_CONTROLLER__KART_COMMAND_HPP_
