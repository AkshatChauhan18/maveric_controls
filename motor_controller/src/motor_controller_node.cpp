// ROS 2 bridge from /cmd_vel to the go-kart ESP32 in experimenting/.
//
// The kart is Ackermann: one steering actuator (position PID against a
// quadrature encoder, +-20000 counts), one throttle (DAC voltage into an EV
// controller), one brake (linear actuator, on/off). That is nothing like the
// four-DDSM differential rover this node used to drive, so the JSON protocol,
// the per-wheel command fan-out and the joint_states/temperature telemetry are
// all gone -- the firmware speaks a 3-byte binary struct at 20 Hz and reports
// back in plain text.
//
// Two things about the firmware that shape this node:
//   1. It only obeys us when RC channel 6 is switched UP (laptop mode) and
//      there is no RC failsafe and no encoder fault. Otherwise every packet is
//      dropped silently, so /cmd_vel does nothing and nothing says why. Hence
//      the telemetry parsing below, which exists purely to log that.
//   2. It frames on "3 bytes available" with no header and no checksum. See
//      kart_command.hpp -- one 3-byte write per tick, and nothing else on this
//      port, ever.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

#include "motor_controller/kart_command.hpp"

using motor_controller::KartCommand;
using motor_controller::KartLimits;

namespace {
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return 0;
  }
}
}  // namespace

class SerialPort
{
public:
  SerialPort() = default;

  bool open_port(const std::string & port, int baud)
  {
    // Blocking open: kernel handles partial writes, no retry loop needed.
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      return false;
    }
    if (!configure_port(baud)) {
      close_port();
      return false;
    }
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    buffer_.clear();
    consume_pos_ = 0;
  }

  bool is_open() const { return fd_ >= 0; }

  // Complete a packet before returning. A blocking TTY normally accepts all
  // three bytes at once, but POSIX still permits an interrupted/partial write.
  // There is only one writer to this fd, so retrying preserves packet order.
  // tcdrain() then waits for the complete packet to leave the host's TTY
  // queue. This is important because the fixed ESP32 receiver has no framing:
  // do not begin the next packet while the previous one is still queued.
  bool write_exact(const void * data, size_t len)
  {
    if (fd_ < 0 || data == nullptr || len == 0) return false;
    const auto * bytes = static_cast<const uint8_t *>(data);
    size_t written = 0;
    while (written < len) {
      const ssize_t n = ::write(fd_, bytes + written, len - written);
      if (n > 0) {
        written += static_cast<size_t>(n);
        continue;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    while (::tcdrain(fd_) != 0) {
      if (errno != EINTR) {
        return false;
      }
    }
    return true;
  }

  bool read_lines(const std::function<void(const std::string &)> & on_line, int timeout_ms)
  {
    if (fd_ < 0) {
      return false;
    }
    pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int ret = ::poll(&pfd, 1, timeout_ms);
    if (ret <= 0) {
      return false;
    }

    // Reuse member buffer to avoid stack allocation on hot path.
    constexpr size_t kReadBufSize = 4096;
    read_buf_.resize(kReadBufSize);
    const ssize_t n = ::read(fd_, read_buf_.data(), kReadBufSize);
    if (n <= 0) {
      return false;
    }
    buffer_.append(read_buf_.data(), static_cast<size_t>(n));

    // Offset-based scanning: no O(n) erase per line.
    bool got_line = false;
    size_t pos;
    while ((pos = buffer_.find('\n', consume_pos_)) != std::string::npos) {
      size_t start = consume_pos_;
      size_t end   = pos;
      if (end > start && buffer_[end - 1] == '\r') {
        --end;
      }
      if (end > start) {
        on_line(std::string(buffer_.data() + start, end - start));
        got_line = true;
      }
      consume_pos_ = pos + 1;
    }

    if (consume_pos_ > buffer_.size() / 2) {
      buffer_.erase(0, consume_pos_);
      consume_pos_ = 0;
    }

    return got_line;
  }

private:
  bool configure_port(int baud)
  {
    termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = to_speed(baud);
    if (speed == 0) {
      return false;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    // Blocking read: VMIN=1, VTIME=0 blocks until at least 1 byte available
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    return tcsetattr(fd_, TCSANOW, &tty) == 0;
  }

  int         fd_{-1};
  std::string buffer_;
  size_t      consume_pos_{0};
  std::vector<char> read_buf_;
};

class MotorControllerNode : public rclcpp::Node
{
public:
  MotorControllerNode()
  : Node("motor_controller")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud", 115200);

    // Vehicle geometry and limits. Cached in lim_; refreshed via param callback.
    lim_.wheelbase           = declare_parameter<double>("wheelbase", 1.20);
    lim_.max_steer_angle     = declare_parameter<double>("max_steer_angle", 0.52);
    lim_.max_speed           = declare_parameter<double>("max_speed", 5.0);
    lim_.max_angular_z       = declare_parameter<double>("max_angular_z", 1.0);
    lim_.ackermann_min_speed = declare_parameter<double>("ackermann_min_speed", 0.2);
    lim_.speed_deadband      = declare_parameter<double>("speed_deadband", 0.02);
    lim_.invert_steering     = declare_parameter<bool>("invert_steering", false);
    lim_.brake_on_zero_cmd   = declare_parameter<bool>("brake_on_zero_cmd", false);

    // Slew limits in percent per control tick.
    throttle_accel_limit_ = declare_parameter<double>("throttle_accel_limit", 5.0);
    throttle_decel_limit_ = declare_parameter<double>("throttle_decel_limit", 100.0);
    steer_slew_limit_     = declare_parameter<double>("steer_slew_limit", 15.0);

    // Steering input filter: exponential moving average (EMA) time constant in seconds.
    // 0.0 = disabled. Typical: 0.05–0.2. Higher = smoother but more lag.
    steer_filter_tau_ = declare_parameter<double>("steer_filter_tau", 0.1);

    // teleop_twist_keyboard publishes per key event, not at a fixed rate. Its
    // initial keyboard auto-repeat delay is commonly about 500 ms, so 300 ms
    // makes a held I/J/L command repeatedly time out between key repeats.
    cmd_timeout_ms_      = declare_parameter<double>("cmd_timeout_ms", 750.0);
    control_rate_hz_     = declare_parameter<double>("control_rate_hz", 20.0);
    debug_serial_        = declare_parameter<bool>("debug_serial", false);

    // Parameter callback to refresh cached values on ros2 param set
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&MotorControllerNode::on_parameters_changed, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("kart_command", 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MotorControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    // Diagnostics
    diag_updater_.setHardwareID("motor_controller");
    diag_updater_.add("Serial Port", this, &MotorControllerNode::diagnose_serial);
    diag_updater_.add("Command Watchdog", this, &MotorControllerNode::diagnose_cmd_watchdog);
    diag_updater_.add("Telemetry Watchdog", this, &MotorControllerNode::diagnose_telemetry_watchdog);
    diag_updater_.add("RC Status", this, &MotorControllerNode::diagnose_rc_status);
    diag_timer_ = create_wall_timer(std::chrono::seconds(1),
      std::bind(&diagnostic_updater::Updater::force_update, &diag_updater_));

    last_cmd_time_ = this->now();
    last_telemetry_time_ = this->now();
    last_reopen_attempt_ = this->now();
    if (control_rate_hz_ <= 0.0) {
      control_rate_hz_ = 20.0;
    }
    const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&MotorControllerNode::control_loop, this));

    if (!serial_.open_port(port_, baud_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s at %d baud", port_.c_str(), baud_);
      throw std::runtime_error("serial open failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "Kart bridge up on %s @ %d. The ESP32 ignores /cmd_vel unless RC channel 6 is UP "
      "(laptop mode) with no failsafe and no encoder fault.",
      port_.c_str(), baud_);

    running_.store(true);
    read_thread_ = std::thread([this]() { read_loop(); });
  }

  ~MotorControllerNode() override
  {
    running_.store(false);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    // Park it: zero throttle, wheels centred, brake on. Without this the
    // firmware coasts for up to 500 ms after this node exits before its own
    // watchdog fires, and it never brakes on its own.
    if (serial_.is_open()) {
      const KartCommand stop = motor_controller::pack_kart_command(0.0, 0.0, true);
      serial_.write_exact(&stop, sizeof(stop));
    }
    serial_.close_port();
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const std::string & name = p.get_name();
      if (name == "wheelbase")           lim_.wheelbase = p.as_double();
      else if (name == "max_steer_angle")     lim_.max_steer_angle = p.as_double();
      else if (name == "max_speed")           lim_.max_speed = p.as_double();
      else if (name == "max_angular_z")       lim_.max_angular_z = p.as_double();
      else if (name == "ackermann_min_speed") lim_.ackermann_min_speed = p.as_double();
      else if (name == "speed_deadband")      lim_.speed_deadband = p.as_double();
      else if (name == "invert_steering")     lim_.invert_steering = p.as_bool();
      else if (name == "brake_on_zero_cmd")   lim_.brake_on_zero_cmd = p.as_bool();
      else if (name == "throttle_accel_limit") throttle_accel_limit_ = p.as_double();
      else if (name == "throttle_decel_limit") throttle_decel_limit_ = p.as_double();
      else if (name == "steer_slew_limit")     steer_slew_limit_ = p.as_double();
      else if (name == "steer_filter_tau")   steer_filter_tau_ = p.as_double();
      else if (name == "cmd_timeout_ms")       cmd_timeout_ms_ = p.as_double();
      else if (name == "control_rate_hz")      control_rate_hz_ = p.as_double();
      else if (name == "debug_serial")         debug_serial_ = p.as_bool();
      // Ignore unknown params
    }
    return result;
  }

  void diagnose_serial(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (serial_.is_open()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial port open");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port closed");
    }
    stat.add("Port", port_);
    stat.add("Baud", baud_);
  }

  void diagnose_cmd_watchdog(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const double age_s = (this->now() - last_cmd_time_).seconds();
    const double timeout_s = cmd_timeout_ms_ / 1000.0;
    if (age_s > timeout_s) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "No /cmd_vel for " + std::to_string(age_s) + "s (timeout " + std::to_string(timeout_s) + "s)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Last /cmd_vel " + std::to_string(age_s) + "s ago");
    }
    stat.add("Age (s)", age_s);
    stat.add("Timeout (s)", timeout_s);
  }

  void diagnose_telemetry_watchdog(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const double age_s = (this->now() - last_telemetry_time_).seconds();
    const double timeout_s = 2.0 * cmd_timeout_ms_ / 1000.0;
    if (age_s > timeout_s) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "No telemetry for " + std::to_string(age_s) + "s (timeout " + std::to_string(timeout_s) + "s)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Last telemetry " + std::to_string(age_s) + "s ago");
    }
    stat.add("Age (s)", age_s);
    stat.add("Timeout (s)", timeout_s);
  }

  void diagnose_rc_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (rc_mode_active_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "RC mode ACTIVE — /cmd_vel ignored");
    } else if (rc_failsafe_active_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "RC failsafe ACTIVE — /cmd_vel blocked");
    } else if (encoder_fault_active_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Encoder FAULT — steering PID disabled");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Laptop mode active");
    }
    stat.add("RC Mode Active", rc_mode_active_);
    stat.add("RC Failsafe", rc_failsafe_active_);
    stat.add("Encoder Fault", encoder_fault_active_);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = msg->linear.x;
    const double w = msg->angular.z;
    if (!std::isfinite(v) || !std::isfinite(w)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected non-finite /cmd_vel (NaN/inf) -- keeping last valid command.");
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_cmd_time_ = this->now();
    cmd_v_ = v;
    cmd_w_ = w;
  }

  void control_loop()
  {
    double target_steer;
    double target_throttle;
    bool   brake;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      const int64_t timeout_ns = static_cast<int64_t>(cmd_timeout_ms_ * 1e6);
      const bool stale = (this->now() - last_cmd_time_).nanoseconds() > timeout_ns;

      if (stale) {
        target_steer    = 0.0;
        target_throttle = 0.0;
        brake           = true;
      } else {
        target_steer    = motor_controller::steer_percent_from_twist(cmd_v_, cmd_w_, lim_);
        target_throttle = motor_controller::throttle_percent_from_twist(cmd_v_, lim_);
        brake           = motor_controller::brake_from_twist(cmd_v_, lim_);

        // EMA filter on steering target to smooth jittery angular.z input
        // alpha = dt / (tau + dt); tau=0 disables filter.
        const double dt = 1.0 / control_rate_hz_;
        if (steer_filter_tau_ > 0.0) {
          const double alpha = dt / (steer_filter_tau_ + dt);
          if (!steer_filter_initialized_) {
            steer_filtered_ = target_steer;
            steer_filter_initialized_ = true;
          } else {
            steer_filtered_ = alpha * target_steer + (1.0 - alpha) * steer_filtered_;
          }
          target_steer = steer_filtered_;
        } else {
          steer_filter_initialized_ = false;  // reset if filter disabled
        }
      }

      current_steer_ = motor_controller::ramp_toward(current_steer_, target_steer, steer_slew_limit_, steer_slew_limit_);

      if (brake) {
        current_throttle_ = 0.0;
      } else {
        current_throttle_ = motor_controller::ramp_toward(current_throttle_, target_throttle, throttle_accel_limit_, throttle_decel_limit_);
      }
    }

    const KartCommand cmd = motor_controller::pack_kart_command(current_steer_, current_throttle_, brake);

    if (!serial_.write_exact(&cmd, sizeof(cmd))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Serial write to the kart failed (port unplugged?)");
      try_reopen_port();
    }

    if (debug_serial_) {
      RCLCPP_INFO(
        get_logger(), "tx: steer=%d throttle=%u brake=%u",
        cmd.steering, cmd.throttle, cmd.brake);
    }

    std_msgs::msg::Float32MultiArray cmd_msg;
    cmd_msg.data = {
      static_cast<float>(cmd.steering),
      static_cast<float>(cmd.throttle),
      static_cast<float>(cmd.brake)
    };
    cmd_pub_->publish(cmd_msg);
  }

  void read_loop()
  {
    while (running_.load(std::memory_order_relaxed)) {
      serial_.read_lines(
        [this](const std::string & line) { handle_telemetry(line); },
        50);
    }
  }

  // Attempt to reopen the serial port after a write failure. Guarded by a
  // backoff so a persistently-dead port does not thrash the filesystem.
  void try_reopen_port()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (serial_.is_open()) {
      serial_.close_port();
    }
    const auto now = this->now();
    const double since_last = (now - last_reopen_attempt_).seconds();
    if (since_last < 2.0) {
      return;  // backoff: at most one reopen attempt every 2 s
    }
    last_reopen_attempt_ = now;
    if (serial_.open_port(port_, baud_)) {
      RCLCPP_WARN(get_logger(), "Reopened serial port %s after write failure.", port_.c_str());
    }
  }

  // The firmware telemetry is a human-readable line at 10 Hz, not JSON:
  //   "RC Mode: ACTIVE | FS: NO | Enc Fault: NO | CH1 Steer: 1500 | ..."
  // The only thing this node must act on is whether the kart is listening at
  // all: in RC mode, in failsafe, or with an encoder fault, processSerialCommands
  // discards our packets and /cmd_vel does nothing whatsoever -- silently, which
  // is the failure worth a log line.
  void handle_telemetry(const std::string & line)
  {
    if (debug_serial_) {
      RCLCPP_INFO(get_logger(), "rx: %s", line.c_str());
    }

    if (line.find("RC Mode:") == std::string::npos) {
      return;  // banner / menu / command echo, nothing to read
    }

    // A valid telemetry line arrived: refresh diagnostics so we know the
    // firmware is still talking. Telemetry must never gate the command path:
    // the ESP32 is the safety authority and can still receive valid binary
    // commands when its text telemetry is disabled, delayed, or malformed.
    // /cmd_vel freshness is the host-side command watchdog.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_telemetry_time_ = this->now();
      rc_mode_active_     = (line.find("RC Mode: ACTIVE") != std::string::npos);
      rc_failsafe_active_ = (line.find("FS: YES") != std::string::npos);
      encoder_fault_active_ = (line.find("Enc Fault: YES") != std::string::npos);
    }

    if (rc_mode_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Kart is in RC mode -- /cmd_vel is being ignored. Flip RC channel 6 UP for laptop mode.");
    }
    if (rc_failsafe_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "RC failsafe active (transmitter off?) -- the kart blocks /cmd_vel until the RC link is back.");
    }
    if (encoder_fault_active_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Steering encoder fault -- throttle and steering PID are disabled on the kart. "
        "Clear it at the firmware serial console ('Z' to re-zero) before driving.");
    }
  }

  std::string port_;
  int    baud_{115200};
  double cmd_timeout_ms_{300.0};
  double control_rate_hz_{20.0};
  bool   debug_serial_{false};

  // Cached parameters (refreshed via param callback)
  KartLimits lim_;
  double throttle_accel_limit_{5.0};
  double throttle_decel_limit_{100.0};
  double steer_slew_limit_{15.0};
  double steer_filter_tau_{0.1};

  // RC/telemetry status for diagnostics
  bool rc_mode_active_{false};
  bool rc_failsafe_active_{false};
  bool encoder_fault_active_{false};

  // Steering EMA filter state
  double steer_filtered_{0.0};
  bool steer_filter_initialized_{false};

  // Latest /cmd_vel, and the slew-limited outputs actually on the wire.
  double cmd_v_{0.0};
  double cmd_w_{0.0};
  double current_steer_{0.0};
  double current_throttle_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_telemetry_time_;
  rclcpp::Time last_reopen_attempt_;

  SerialPort serial_;
  std::mutex state_mutex_;
  std::atomic<bool> running_{false};
  std::thread read_thread_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  diagnostic_updater::Updater diag_updater_{this};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
