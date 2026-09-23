#ifndef RANGER_BASE_NAVLITE_CHASSIS_TRACE_HPP_
#define RANGER_BASE_NAVLITE_CHASSIS_TRACE_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace westonrobot {

// Diagnostic-only cache. Never controls a command, transition, or watchdog.
// Called only from the existing messenger thread; no new ROS/CAN work or lock.
class NavliteChassisTrace {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  void Command(double vx, double vy, double wz, Time now) {
    input_ = {{vx, vy, wz}};
    command_at_ = now;
    ++command_seq_;
    command_seen_ = true;
  }

  void Submitted(double linear, double steering, double angular,
                 const char * reason, Time now) {
    sdk_ = {{linear, steering, angular}};
    sdk_at_ = now;
    sdk_reason_ = reason;
    ++sdk_seq_;
    sdk_seen_ = true;
  }

  void Feedback(double linear, double angular, double steering,
                std::array<double, 4> wheel_speed,
                std::array<double, 4> wheel_angle,
                Time core_group_stamp, Time actuator_group_stamp, Time now) {
    raw_ = {{linear, steering, angular}};
    wheel_speed_ = wheel_speed;
    wheel_angle_ = wheel_angle;
    core_stamp_ = core_group_stamp;
    actuator_stamp_ = actuator_group_stamp;
    feedback_read_at_ = now;
    ++feedback_read_seq_;
    feedback_seen_ = true;
  }

  void Odometry(double vx, double vy, double wz, int64_t ros_stamp_ns, Time now) {
    odom_ = {{vx, vy, wz}};
    odom_stamp_ns_ = ros_stamp_ns;
    odom_at_ = now;
    odom_seen_ = true;
  }

  int desired_mode = -1;
  int actual_mode = -1;
  bool mode_changing = false;
  const char * mode_state = "unknown";
  bool stop_stable = false;
  double mode_elapsed_sec = 0;
  // Set only for branches that did not call SetMotionCommand in this callback.
  bool callback_submitted = false;
  const char * callback_reason = "no_command_seen";

  std::string Event(Time now) {
    // Hysteresis affects log selection only. Raw signed values are retained.
    input_signs_ = Signs(input_, input_signs_);
    // Steering alone is not SDK motion: x + angular are the motion arguments.
    sdk_signs_ = Signs({{sdk_[0], 0, sdk_[2]}}, sdk_signs_);
    odom_signs_ = Signs(odom_, odom_signs_);
    const bool gap = command_seen_ && Age(now, command_at_) > 0.5;
    const bool changed = !logged_ || input_signs_ != logged_input_signs_ ||
      sdk_signs_ != logged_sdk_signs_ || gap != logged_gap_ ||
      odom_signs_ != logged_odom_signs_ || odom_seen_ != logged_odom_seen_ ||
      command_seen_ != logged_command_seen_ || sdk_seen_ != logged_sdk_seen_ ||
      desired_mode != logged_desired_ || actual_mode != logged_actual_ ||
      mode_changing != logged_changing_ || mode_state != logged_mode_state_ ||
      callback_reason != logged_reason_;
    const bool active = command_seen_ && (!gap || mode_changing ||
      std::strcmp(mode_state, "stable") != 0 ||
      (odom_seen_ && odom_signs_ != std::array<int, 3>{{0, 0, 0}}));
    if (!changed && (!active || Age(now, logged_at_) < 1.0)) return {};
    logged_ = true;
    logged_at_ = now;
    logged_input_signs_ = input_signs_;
    logged_sdk_signs_ = sdk_signs_;
    logged_odom_signs_ = odom_signs_;
    logged_odom_seen_ = odom_seen_;
    logged_gap_ = gap;
    logged_command_seen_ = command_seen_;
    logged_sdk_seen_ = sdk_seen_;
    logged_desired_ = desired_mode;
    logged_actual_ = actual_mode;
    logged_changing_ = mode_changing;
    logged_mode_state_ = mode_state;
    logged_reason_ = callback_reason;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "NAVLITE chassis schema=1 event=" << (changed ? "change" : "sample")
        << " steady_ns=" << std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count()
        << " input_known=" << command_seen_ << " input_seq=" << command_seq_
        << " input_age_sec=" << KnownAge(command_seen_, now, command_at_)
        << " input_gap=" << gap << " input_gap_threshold_sec=0.5"
        << " input_source_stamp=unknown input_state=" << State(command_seen_, input_signs_)
        << " input_vx=" << input_[0] << " input_vy=" << input_[1]
        << " input_wz=" << input_[2]
        << " in=(" << input_[0] << ',' << input_[1] << ',' << input_[2] << ')'
        << " input_exact_zero=" << (input_[0] == 0 && input_[1] == 0 && input_[2] == 0)
        << " sdk_known=" << sdk_seen_ << " sdk_submit_seq=" << sdk_seq_
        << " sdk_submit_age_sec=" << KnownAge(sdk_seen_, now, sdk_at_)
        << " sdk_state=" << State(sdk_seen_, sdk_signs_)
        << " sdk_linear=" << sdk_[0] << " sdk_steer=" << sdk_[1]
        << " sdk_angular=" << sdk_[2] << " reason=" << sdk_reason_
        << " sdk_exact_zero=" << (sdk_[0] == 0 && sdk_[2] == 0)
        << " callback_reason=" << callback_reason
        << " callback_submitted=" << callback_submitted
        << " publication=observation can_tx_confirmed=unknown"
        << " desired_mode=" << desired_mode << " actual_mode=" << actual_mode
        << " mode_changing=" << mode_changing << " mode_state=" << mode_state
        << " stop_stable=" << stop_stable << " mode_elapsed_sec=" << mode_elapsed_sec
        << " feedback_known=" << feedback_seen_
        << " feedback_is_cache=1 motion_fresh=unknown"
        << " feedback_read_seq=" << feedback_read_seq_
        << " feedback_read_age_sec=" << KnownAge(feedback_seen_, now, feedback_read_at_)
        << " core_group_stamp_ns=" << Stamp(core_stamp_)
        << " core_group_age_sec=" << KnownAge(core_stamp_ != Time{}, now, core_stamp_)
        << " actuator_group_stamp_ns=" << Stamp(actuator_stamp_)
        << " actuator_group_age_sec=" << KnownAge(actuator_stamp_ != Time{}, now, actuator_stamp_)
        << " motion_can_age_sec=unknown motion_can_seq=unknown"
        << " wheel_can_age_sec=unknown wheel_can_seq=unknown"
        << " group_stamp_kind=sdk_receive_steady_not_per_frame"
        << " raw_linear=" << raw_[0] << " raw_steer=" << raw_[1]
        << " raw_angular=" << raw_[2]
        << " wheel_speed=" << wheel_speed_[0] << ',' << wheel_speed_[1] << ','
        << wheel_speed_[2] << ',' << wheel_speed_[3]
        << " wheel_angle=" << wheel_angle_[0] << ',' << wheel_angle_[1] << ','
        << wheel_angle_[2] << ',' << wheel_angle_[3]
        << " odom_known=" << odom_seen_ << " odom_stamp_ns=" << odom_stamp_ns_
        << " odom_state=" << State(odom_seen_, odom_signs_)
        << " odom_stamp_kind=driver_publish_time odom_cache_age_sec="
        << KnownAge(odom_seen_, now, odom_at_)
        << " odom_vx=" << odom_[0] << " odom_vy=" << odom_[1]
        << " odom_wz=" << odom_[2]
        << " actual=(" << odom_[0] << ',' << odom_[1] << ',' << odom_[2] << ')'
        << " actual_kind=wheel_odom_cache";
    return out.str();
  }

 private:
  static double Age(Time now, Time stamp) {
    return std::chrono::duration<double>(now - stamp).count();
  }
  static double KnownAge(bool known, Time now, Time stamp) {
    return known && stamp <= now ? Age(now, stamp) : std::numeric_limits<double>::quiet_NaN();
  }
  static int64_t Stamp(Time stamp) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
  }
  static std::array<int, 3> Signs(std::array<double, 3> value, std::array<int, 3> prior) {
    for (std::size_t i = 0; i < value.size(); ++i) {
      if (!std::isfinite(value[i])) prior[i] = 2;
      else if (value[i] == 0.0 || std::abs(value[i]) <= 0.002) prior[i] = 0;
      else if (std::abs(value[i]) >= 0.005) prior[i] = value[i] > 0 ? 1 : -1;
    }
    return prior;
  }
  static const char * State(bool known, std::array<int, 3> signs) {
    if (!known) return "unknown";
    for (int s : signs) if (s == 2) return "invalid";
    for (int s : signs) if (s != 0) return "nonzero";
    return "near_zero";
  }
  std::array<double, 3> input_{{0, 0, 0}}, sdk_{{0, 0, 0}}, raw_{{0, 0, 0}}, odom_{{0, 0, 0}};
  std::array<double, 4> wheel_speed_{{0, 0, 0, 0}}, wheel_angle_{{0, 0, 0, 0}};
  Time command_at_{}, sdk_at_{}, feedback_read_at_{}, core_stamp_{}, actuator_stamp_{}, odom_at_{};
  Time logged_at_{};
  bool command_seen_ = false, sdk_seen_ = false, feedback_seen_ = false, odom_seen_ = false;
  uint64_t command_seq_ = 0, sdk_seq_ = 0, feedback_read_seq_ = 0;
  int64_t odom_stamp_ns_ = 0;
  const char * sdk_reason_ = "unknown";
  std::array<int, 3> input_signs_{{0, 0, 0}}, sdk_signs_{{0, 0, 0}};
  std::array<int, 3> odom_signs_{{0, 0, 0}}, logged_odom_signs_{{0, 0, 0}};
  std::array<int, 3> logged_input_signs_{{0, 0, 0}}, logged_sdk_signs_{{0, 0, 0}};
  bool logged_ = false, logged_gap_ = false, logged_command_seen_ = false;
  bool logged_sdk_seen_ = false, logged_changing_ = false;
  bool logged_odom_seen_ = false;
  int logged_desired_ = -1, logged_actual_ = -1;
  const char * logged_mode_state_ = "unknown";
  const char * logged_reason_ = "no_command_seen";
};

}  // namespace westonrobot
#endif
