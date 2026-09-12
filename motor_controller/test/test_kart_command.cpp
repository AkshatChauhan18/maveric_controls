// Self-check for the Twist -> kart command mapping. No ROS, no gtest, no serial
// port: `colcon test` or just run the binary. Asserts only, so a failure aborts.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "motor_controller/kart_command.hpp"

using namespace motor_controller;

namespace {

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) < tol; }

KartLimits defaults()
{
  KartLimits lim;  // wheelbase 1.2, max_steer 0.52, max_speed 5, max_angular_z 1
  return lim;
}

void test_straight_at_speed()
{
  const KartLimits lim = defaults();
  assert(near(steer_percent_from_twist(2.0, 0.0, lim), 0.0));
  assert(near(throttle_percent_from_twist(2.0, lim), 40.0));
  assert(!brake_from_twist(2.0, lim));
}

void test_ackermann_steering()
{
  const KartLimits lim = defaults();
  // delta = atan(L*w/v), then scale delta to the ESP32's -100..100 range.
  const double expect = std::atan(1.20 * 0.5 / 2.0) / 0.52 * 100.0;
  assert(near(steer_percent_from_twist(2.0, 0.5, lim), expect));
  // Turning the other way is symmetric.
  assert(near(steer_percent_from_twist(2.0, -0.5, lim), -expect));
  // The same yaw rate needs less road-wheel angle at a higher speed.
  assert(std::abs(steer_percent_from_twist(4.0, 0.5, lim)) < std::abs(expect));
  // Negative velocity is not driven by this kart, but maps consistently before
  // brake_from_twist turns that request into a stop.
  assert(near(steer_percent_from_twist(-2.0, 0.5, lim), expect));
}

void test_standstill_branch()
{
  const KartLimits lim = defaults();
  // Below ackermann_min_speed, angular.z is a direct normalised steer request,
  // so teleop can still centre / crank the wheels with v == 0.
  assert(near(steer_percent_from_twist(0.0, 1.0, lim), 100.0));
  assert(near(steer_percent_from_twist(0.0, 0.5, lim), 50.0));
  assert(near(steer_percent_from_twist(0.0, 0.0, lim), 0.0));
  // And it saturates instead of wrapping past full lock.
  assert(near(steer_percent_from_twist(0.0, 5.0, lim), 100.0));
  assert(near(steer_percent_from_twist(0.0, -5.0, lim), -100.0));
}

void test_reverse_is_a_brake_and_does_not_change_steering()
{
  const KartLimits lim = defaults();
  assert(brake_from_twist(-1.0, lim));
  assert(near(throttle_percent_from_twist(-1.0, lim), 0.0));
  const double fwd = steer_percent_from_twist(1.0, 0.2, lim);
  assert(fwd > 1.0 && fwd < 99.0);
  assert(near(steer_percent_from_twist(-1.0, 0.2, lim), fwd));
}

void test_zero_command_always_brakes()
{
  KartLimits lim = defaults();
  // An explicit zero-velocity Twist is a stop command, so it must brake even
  // when brake_on_zero_cmd is false.
  assert(brake_from_twist(0.0, lim));
  lim.brake_on_zero_cmd = true;
  assert(brake_from_twist(0.0, lim));
  // Deadband, not exact zero: floating-point noise off a planner must not
  // decide between coasting and braking.
  assert(brake_from_twist(0.001, lim));
  assert(!brake_from_twist(0.5, lim));
}

void test_throttle_saturates_and_inverts()
{
  KartLimits lim = defaults();
  assert(near(throttle_percent_from_twist(50.0, lim), 100.0));
  assert(near(throttle_percent_from_twist(0.0, lim), 0.0));
  lim.invert_steering = true;
  assert(near(steer_percent_from_twist(0.0, 1.0, lim), -100.0));
}

void test_ramp()
{
  // Accel is limited, decel is not.
  assert(near(ramp_toward(0.0, 100.0, 5.0, 100.0), 5.0));
  assert(near(ramp_toward(100.0, 0.0, 5.0, 100.0), 0.0));
  // Braking through zero into the other direction counts as easing off, so it
  // gets the decel rate -- but the rate still caps the step (150 of swing at
  // 100/tick is two ticks, not one).
  assert(near(ramp_toward(50.0, -100.0, 5.0, 100.0), -50.0));
  // Growing the magnitude further from zero is acceleration, even going more
  // negative, so it gets the accel rate.
  assert(near(ramp_toward(-50.0, -100.0, 5.0, 100.0), -55.0));
  assert(near(ramp_toward(-50.0, 0.0, 5.0, 100.0), 0.0));
  // Reaching the target does not overshoot it.
  assert(near(ramp_toward(98.0, 100.0, 5.0, 100.0), 100.0));
  assert(near(ramp_toward(-100.0, 100.0, 5.0, 5.0), -95.0));
}

void test_packing()
{
  // Rounding, saturation, and the brake/throttle interlock.
  KartCommand c = pack_kart_command(56.4, 39.6, false);
  assert(c.steering == 56);
  assert(c.throttle == 40);
  assert(c.brake == 0);

  c = pack_kart_command(500.0, 500.0, false);
  assert(c.steering == 100);
  assert(c.throttle == 100);

  c = pack_kart_command(-500.0, 0.0, false);
  assert(c.steering == -100);

  // Brake wins: never put a non-zero throttle byte on the wire alongside it.
  c = pack_kart_command(0.0, 80.0, true);
  assert(c.brake == 1);
  assert(c.throttle == 0);
}

}  // namespace

int main()
{
  test_straight_at_speed();
  test_ackermann_steering();
  test_standstill_branch();
  test_reverse_is_a_brake_and_does_not_change_steering();
  test_zero_command_always_brakes();
  test_throttle_saturates_and_inverts();
  test_ramp();
  test_packing();
  std::printf("kart_command: all checks passed\n");
  return 0;
}
