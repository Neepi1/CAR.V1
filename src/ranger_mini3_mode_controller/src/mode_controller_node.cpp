#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "ranger_mini3_mode_controller/ranger_motion_mode.hpp"
#include "ranger_msgs/msg/motion_state.hpp"
#include "ranger_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using ranger_mini3_mode_controller::RangerMotionMode;
using ranger_mini3_mode_controller::ranger_motion_mode_code;
using ranger_mini3_mode_controller::ranger_motion_mode_from_code;
using ranger_mini3_mode_controller::ranger_motion_mode_json;
using ranger_mini3_mode_controller::ranger_motion_mode_name;
using ranger_mini3_mode_controller::ranger_motion_mode_short_name;

namespace {

enum class MotionMode : int {
  kDualAckermann = 0,
  kCrab = 1,
  kSpin = 2,
  kPark = 3,
};

struct RangerCommand {
  MotionMode mode{MotionMode::kDualAckermann};
  double linear_mps{0.0};
  double lateral_mps{0.0};
  double yaw_radps{0.0};
  double steering_rad{0.0};
  bool valid{true};
  std::string reason;
};

struct ReversePermit {
  bool enabled{false};
  std::chrono::steady_clock::time_point stamp{};
};

struct ActualMotionMode {
  RangerMotionMode mode{RangerMotionMode::UNKNOWN};
  std::chrono::steady_clock::time_point stamp{};
  std::string source;
};

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

std::string legacyModeName(MotionMode mode) {
  switch (mode) {
    case MotionMode::kDualAckermann:
      return "dual_ackermann";
    case MotionMode::kCrab:
      return "crab";
    case MotionMode::kSpin:
      return "spin";
    case MotionMode::kPark:
      return "park";
  }
  return "unknown";
}

RangerMotionMode desiredRangerMode(MotionMode mode) {
  switch (mode) {
    case MotionMode::kDualAckermann:
      return RangerMotionMode::DUAL_ACKERMAN;
    case MotionMode::kCrab:
      return RangerMotionMode::PARALLEL;
    case MotionMode::kSpin:
      return RangerMotionMode::SPINNING;
    case MotionMode::kPark:
      return RangerMotionMode::DUAL_ACKERMAN;
  }
  return RangerMotionMode::UNKNOWN;
}

double dualAckermannInnerAngleFromTwist(
  double vx_mps,
  double wz_radps,
  double wheelbase_m,
  double track_m,
  double max_angle_rad,
  double eps = 1.0e-6) {
  if (std::abs(vx_mps) < eps || std::abs(wz_radps) < eps) {
    return 0.0;
  }
  const double kappa = wz_radps / vx_mps;
  if (std::abs(kappa) < eps) {
    return 0.0;
  }
  const double radius = 1.0 / std::abs(kappa);
  const double denom = std::max(radius - track_m / 2.0, eps);
  const double angle = std::atan((wheelbase_m / 2.0) / denom);
  return std::copysign(std::min(angle, max_angle_rad), kappa);
}

double dualAckermannVirtualAngleFromTwist(
  double vx_mps,
  double wz_radps,
  double wheelbase_m,
  double max_angle_rad,
  double eps = 1.0e-6) {
  if (std::abs(vx_mps) < eps || std::abs(wz_radps) < eps) {
    return 0.0;
  }
  const double angle = std::atan((wz_radps / vx_mps) * wheelbase_m / 2.0);
  return clamp(angle, -max_angle_rad, max_angle_rad);
}

double maxInnerAckermannYawRate(
  double vx_mps,
  double wheelbase_m,
  double track_m,
  double max_angle_rad,
  double eps = 1.0e-6) {
  if (std::abs(vx_mps) < eps) {
    return 0.0;
  }
  const double radius =
    (wheelbase_m / 2.0) / std::max(std::tan(max_angle_rad), eps) + track_m / 2.0;
  return std::abs(vx_mps) / std::max(radius, eps);
}

double maxVirtualAckermannYawRate(
  double vx_mps,
  double wheelbase_m,
  double max_angle_rad,
  double eps = 1.0e-6) {
  if (std::abs(vx_mps) < eps) {
    return 0.0;
  }
  const double kappa = std::tan(max_angle_rad) * 2.0 / std::max(wheelbase_m, eps);
  return std::abs(vx_mps) * kappa;
}

double speedLimitForAngle(double angle_rad, double normal_limit_mps, double high_angle_limit_mps) {
  if (std::abs(angle_rad) > 20.0 * M_PI / 180.0) {
    return high_angle_limit_mps;
  }
  return normal_limit_mps;
}

}  // namespace

class RangerMini3ModeController : public rclcpp::Node {
 public:
  RangerMini3ModeController() : Node("ranger_mini3_mode_controller") {
    declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_safe");
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel");
    declare_parameter<double>("wheelbase_m", 0.494);
    declare_parameter<double>("track_m", 0.364);
    declare_parameter<double>("max_linear_mps", 1.5);
    declare_parameter<double>("max_linear_high_angle_mps", 0.75);
    declare_parameter<double>("max_ackermann_angle_rad", 0.698);
    declare_parameter<double>("max_spin_radps", 3.259);
    declare_parameter<double>("max_lateral_mps", 0.08);
    declare_parameter<double>("max_crab_yaw_radps", 0.15);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("cmd_timeout_s", 0.25);
    declare_parameter<std::string>("mode_policy", "auto");
    declare_parameter<bool>("allow_reverse", false);
    declare_parameter<std::string>("lateral_policy", "reject");
    declare_parameter<std::string>("dual_ackermann_angle", "inner");
    declare_parameter<double>("spin_steering_threshold_rad", 0.698);
    declare_parameter<double>("spin_enter_steering_threshold_rad", -1.0);
    declare_parameter<double>("spin_exit_steering_threshold_rad", -1.0);
    declare_parameter<double>("spin_enter_debounce_s", 0.4);
    declare_parameter<double>("spin_min_hold_s", 1.0);
    declare_parameter<double>("auto_spin_max_linear_mps", 0.08);
    declare_parameter<bool>("spin_on_high_curvature_while_moving", false);
    declare_parameter<double>("linear_epsilon_mps", 0.02);
    declare_parameter<double>("lateral_epsilon_mps", 0.02);
    declare_parameter<double>("yaw_epsilon_radps", 0.02);
    declare_parameter<bool>("publish_zero_on_startup", true);
    declare_parameter<std::string>("status_topic", "/ranger_mini3_mode_controller/status");
    declare_parameter<std::string>("desired_mode_topic", "/ranger_mini3/desired_motion_mode");
    declare_parameter<std::string>("actual_motion_state_topic", "/motion_state");
    declare_parameter<std::string>("actual_system_state_topic", "/system_state");
    declare_parameter<double>("actual_motion_mode_max_age_sec", 0.5);
    declare_parameter<double>("mode_alignment_warn_after_sec", 0.25);
    declare_parameter<std::string>("forced_mode_topic", "/ranger_mini3/forced_mode");
    declare_parameter<std::string>("reverse_enable_topic", "/ranger_mini3/allow_reverse");
    declare_parameter<std::string>("docking_reverse_enable_topic", "/ranger_mini3/docking_allow_reverse");
    declare_parameter<std::string>("teleop_reverse_enable_topic", "/ranger_mini3/teleop_allow_reverse");
    declare_parameter<double>("reverse_enable_timeout_s", 0.75);
    declare_parameter<std::string>("park_topic", "/ranger_mini3/park");

    wheelbase_m_ = get_parameter("wheelbase_m").as_double();
    track_m_ = get_parameter("track_m").as_double();
    max_linear_mps_ = get_parameter("max_linear_mps").as_double();
    max_linear_high_angle_mps_ = get_parameter("max_linear_high_angle_mps").as_double();
    max_ackermann_angle_rad_ = get_parameter("max_ackermann_angle_rad").as_double();
    max_spin_radps_ = get_parameter("max_spin_radps").as_double();
    max_lateral_mps_ = get_parameter("max_lateral_mps").as_double();
    max_crab_yaw_radps_ = get_parameter("max_crab_yaw_radps").as_double();
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    mode_policy_ = get_parameter("mode_policy").as_string();
    allow_reverse_ = get_parameter("allow_reverse").as_bool();
    lateral_policy_ = get_parameter("lateral_policy").as_string();
    dual_ackermann_angle_ = get_parameter("dual_ackermann_angle").as_string();
    const double legacy_spin_threshold = get_parameter("spin_steering_threshold_rad").as_double();
    const double enter_threshold = get_parameter("spin_enter_steering_threshold_rad").as_double();
    const double exit_threshold = get_parameter("spin_exit_steering_threshold_rad").as_double();
    spin_enter_steering_threshold_rad_ = enter_threshold > 0.0 ? enter_threshold : legacy_spin_threshold;
    spin_exit_steering_threshold_rad_ =
      exit_threshold > 0.0 ? exit_threshold : std::min(spin_enter_steering_threshold_rad_ * 0.65, spin_enter_steering_threshold_rad_ - 0.05);
    if (spin_exit_steering_threshold_rad_ >= spin_enter_steering_threshold_rad_) {
      warnThrottled("spin_thresholds", "spin_exit_steering_threshold_rad must be lower than spin_enter_steering_threshold_rad; using 65% of enter threshold.");
      spin_exit_steering_threshold_rad_ = spin_enter_steering_threshold_rad_ * 0.65;
    }
    spin_enter_debounce_s_ = std::max(0.0, get_parameter("spin_enter_debounce_s").as_double());
    spin_min_hold_s_ = std::max(0.0, get_parameter("spin_min_hold_s").as_double());
    auto_spin_max_linear_mps_ = std::max(0.0, get_parameter("auto_spin_max_linear_mps").as_double());
    spin_on_high_curvature_while_moving_ = get_parameter("spin_on_high_curvature_while_moving").as_bool();
    reverse_enable_timeout_s_ = get_parameter("reverse_enable_timeout_s").as_double();
    actual_motion_mode_max_age_sec_ =
      std::max(0.05, get_parameter("actual_motion_mode_max_age_sec").as_double());
    mode_alignment_warn_after_sec_ =
      std::max(0.0, get_parameter("mode_alignment_warn_after_sec").as_double());
    linear_eps_ = get_parameter("linear_epsilon_mps").as_double();
    lateral_eps_ = get_parameter("lateral_epsilon_mps").as_double();
    yaw_eps_ = get_parameter("yaw_epsilon_radps").as_double();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("desired_mode_topic").as_string(), 10);

    motion_state_sub_ = create_subscription<ranger_msgs::msg::MotionState>(
      get_parameter("actual_motion_state_topic").as_string(),
      10,
      [this](const ranger_msgs::msg::MotionState::SharedPtr msg) {
        updateActualMotionMode(msg->motion_mode, "motion_state");
      });

    system_state_sub_ = create_subscription<ranger_msgs::msg::SystemState>(
      get_parameter("actual_system_state_topic").as_string(),
      10,
      [this](const ranger_msgs::msg::SystemState::SharedPtr msg) {
        updateActualMotionMode(msg->motion_mode, "system_state");
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_in_topic").as_string(),
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = std::chrono::steady_clock::now();
      });

    forced_mode_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("forced_mode_topic").as_string(),
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) { onForcedMode(msg->data); });

    subscribeReversePermit("legacy", get_parameter("reverse_enable_topic").as_string());
    subscribeReversePermit("docking", get_parameter("docking_reverse_enable_topic").as_string());
    subscribeReversePermit("teleop", get_parameter("teleop_reverse_enable_topic").as_string());

    park_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("park_topic").as_string(),
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) { park_requested_ = msg->data; });

    const double rate_hz = std::max(get_parameter("publish_rate_hz").as_double(), 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() { onTimer(); });

    if (get_parameter("publish_zero_on_startup").as_bool()) {
      publishCommand(geometry_msgs::msg::Twist{}, MotionMode::kDualAckermann, "startup_zero", true, "");
    }

    RCLCPP_INFO(
      get_logger(),
      "Ranger Mini 3 C++ mode controller started: %s -> %s, policy=%s, lateral_policy=%s, allow_reverse=%s, spin_enter=%.3f rad, spin_exit=%.3f rad, auto_spin_max_vx=%.3f m/s, moving_curvature_spin=%s, actual topics=%s,%s",
      get_parameter("cmd_vel_in_topic").as_string().c_str(),
      get_parameter("cmd_vel_out_topic").as_string().c_str(),
      mode_policy_.c_str(),
      lateral_policy_.c_str(),
      allow_reverse_ ? "true" : "false",
      spin_enter_steering_threshold_rad_,
      spin_exit_steering_threshold_rad_,
      auto_spin_max_linear_mps_,
      spin_on_high_curvature_while_moving_ ? "true" : "false",
      get_parameter("actual_motion_state_topic").as_string().c_str(),
      get_parameter("actual_system_state_topic").as_string().c_str());
  }

 private:
  void updateActualMotionMode(const std::uint8_t code, const std::string & source) {
    actual_motion_mode_ = ActualMotionMode{
      ranger_motion_mode_from_code(code),
      std::chrono::steady_clock::now(),
      source};
  }

  std::optional<double> actualMotionModeAgeSec(const std::chrono::steady_clock::time_point now) const {
    if (!actual_motion_mode_) {
      return std::nullopt;
    }
    return std::chrono::duration<double>(now - actual_motion_mode_->stamp).count();
  }

  void subscribeReversePermit(const std::string & source, const std::string & topic) {
    if (topic.empty() || subscribed_reverse_topics_.count(topic) > 0) {
      return;
    }
    subscribed_reverse_topics_.insert(topic);
    reverse_enable_subs_.push_back(create_subscription<std_msgs::msg::Bool>(
      topic,
      10,
      [this, source](const std_msgs::msg::Bool::SharedPtr msg) {
        auto & permit = reverse_permits_by_source_[source];
        permit.enabled = msg->data;
        permit.stamp = std::chrono::steady_clock::now();
      }));
  }

  void onForcedMode(const std::string & input) {
    const std::string mode = lowercase(input);
    if (mode.empty() || mode == "auto") {
      forced_policy_.clear();
    } else if (mode == "dual_ackerman" || mode == "dual_ackermann" || mode == "ackermann" ||
      mode == "front_rear_ackermann" || mode == "motion_mode_dual_ackerman" || mode == "0")
    {
      forced_policy_ = "dual_ackermann";
    } else if (mode == "crab" || mode == "lateral" || mode == "sideways" || mode == "parallel" ||
      mode == "motion_mode_parallel" || mode == "1")
    {
      forced_policy_ = "crab";
    } else if (mode == "spin" || mode == "spinning" || mode == "motion_mode_spinning" || mode == "2") {
      forced_policy_ = "spin";
    } else if (mode == "park") {
      forced_policy_ = "park";
    } else {
      warnThrottled(
        "bad_mode",
        "Unknown forced mode '" + input +
        "'. Use auto, dual_ackerman, parallel/crab, spinning/spin, park, or official enum code 0/1/2.");
      return;
    }
    RCLCPP_INFO(get_logger(), "forced mode set to %s", forced_policy_.empty() ? "auto" : forced_policy_.c_str());
  }

  void onTimer() {
    const RangerCommand command = computeCommand(std::chrono::steady_clock::now());
    if (!command.valid) {
      publishCommand(geometry_msgs::msg::Twist{}, command.mode, "invalid_stop", false, command.reason);
      warnThrottled("invalid_cmd", command.reason);
      return;
    }
    publishCommand(commandToTwist(command), command.mode, "ok", true, command.reason);
  }

  RangerCommand computeCommand(std::chrono::steady_clock::time_point now) {
    if (park_requested_ || forced_policy_ == "park") {
      resetSpinHysteresis();
      return RangerCommand{MotionMode::kPark};
    }

    if (last_cmd_time_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(now - last_cmd_time_).count() > cmd_timeout_s_) {
      resetSpinHysteresis();
      return RangerCommand{MotionMode::kDualAckermann};
    }

    double vx = last_cmd_.linear.x;
    double vy = last_cmd_.linear.y;
    double wz = last_cmd_.angular.z;

    if (std::abs(vx) < linear_eps_) {
      vx = 0.0;
    }
    if (std::abs(vy) < lateral_eps_) {
      vy = 0.0;
    }
    if (std::abs(wz) < yaw_eps_) {
      wz = 0.0;
    }
    const bool reverse_allowed = effectiveAllowReverse(now);
    if (!reverse_allowed && vx < 0.0) {
      vx = 0.0;
    }

    std::string policy = forced_policy_.empty() ? mode_policy_ : forced_policy_;
    policy = lowercase(policy);
    if (policy == "dual_ackermann" || policy == "ackermann" || policy == "front_rear_ackermann") {
      return makeDualAckermann(vx, wz, reverse_allowed, now);
    }
    if (policy == "crab" || policy == "lateral" || policy == "sideways" || policy == "parallel") {
      resetSpinHysteresis();
      return makeCrab(vx, vy, wz, reverse_allowed);
    }
    if (policy == "spin") {
      resetSpinHysteresis();
      return makeSpin(wz, "forced_spin");
    }
    if (policy != "auto") {
      warnThrottled("policy", "Unknown mode_policy '" + policy + "', falling back to auto.");
    }

    bool has_vx = std::abs(vx) >= linear_eps_;
    bool has_vy = std::abs(vy) >= lateral_eps_;
    bool has_wz = std::abs(wz) >= yaw_eps_;
    if (has_vy) {
      if (lateral_policy_ == "discard") {
        vy = 0.0;
        has_vy = false;
      } else if (lateral_policy_ == "crab" || lateral_policy_ == "allow") {
        resetSpinHysteresis();
        return makeCrab(vx, vy, wz, reverse_allowed);
      } else {
        resetSpinHysteresis();
        RangerCommand invalid;
        invalid.valid = false;
        invalid.reason =
          "Lateral / crab commands are disabled for this Ranger Mini 3 profile; linear.y is not allowed.";
        return invalid;
      }
    }
    if (!has_vx && !has_vy && !has_wz) {
      resetSpinHysteresis();
      return RangerCommand{MotionMode::kDualAckermann};
    }

    if (has_wz && !has_vx && !has_vy) {
      resetSpinHysteresis();
      return makeSpin(wz);
    }
    return makeDualAckermann(vx, wz, reverse_allowed, now);
  }

  RangerCommand makeSpin(double wz, const std::string & reason = "") const {
    RangerCommand command;
    command.mode = MotionMode::kSpin;
    command.yaw_radps = clamp(wz, -max_spin_radps_, max_spin_radps_);
    command.reason = reason;
    return command;
  }

  RangerCommand makeCrab(double vx, double vy, double wz, bool reverse_allowed) const {
    if (!reverse_allowed) {
      vx = std::max(0.0, vx);
    }

    RangerCommand command;
    command.mode = MotionMode::kCrab;
    command.linear_mps = clamp(vx, -max_linear_high_angle_mps_, max_linear_high_angle_mps_);
    command.lateral_mps = clamp(vy, -max_lateral_mps_, max_lateral_mps_);
    command.yaw_radps = clamp(wz, -max_crab_yaw_radps_, max_crab_yaw_radps_);
    return command;
  }

  bool effectiveAllowReverse(const std::chrono::steady_clock::time_point now) const {
    if (allow_reverse_) {
      return true;
    }
    for (const auto & entry : reverse_permits_by_source_) {
      const auto & permit = entry.second;
      if (!permit.enabled || permit.stamp.time_since_epoch().count() == 0) {
        continue;
      }
      if (std::chrono::duration<double>(now - permit.stamp).count() <= reverse_enable_timeout_s_) {
        return true;
      }
    }
    return false;
  }

  RangerCommand makeDualAckermann(
    double vx,
    double wz,
    bool reverse_allowed,
    const std::chrono::steady_clock::time_point now) {
    if (!reverse_allowed) {
      vx = std::max(0.0, vx);
    }
    const double raw_angle = rawAckermannAngle(vx, wz);
    const bool low_speed_spin_allowed = std::abs(vx) <= auto_spin_max_linear_mps_;
    if (spin_on_high_curvature_while_moving_ || low_speed_spin_allowed) {
      const std::string spin_reason = spinHysteresisReason(raw_angle, now);
      if (!spin_reason.empty()) {
        return makeSpin(wz, spin_reason);
      }
    } else {
      resetSpinHysteresis();
    }

    double angle = 0.0;
    double max_wz = 0.0;
    if (dual_ackermann_angle_ == "virtual") {
      angle = dualAckermannVirtualAngleFromTwist(vx, wz, wheelbase_m_, max_ackermann_angle_rad_);
      max_wz = maxVirtualAckermannYawRate(vx, wheelbase_m_, max_ackermann_angle_rad_);
    } else {
      angle = dualAckermannInnerAngleFromTwist(vx, wz, wheelbase_m_, track_m_, max_ackermann_angle_rad_);
      max_wz = maxInnerAckermannYawRate(vx, wheelbase_m_, track_m_, max_ackermann_angle_rad_);
    }

    if (max_wz > 0.0) {
      wz = clamp(wz, -max_wz, max_wz);
    } else {
      wz = 0.0;
    }
    const double limit = speedLimitForAngle(angle, max_linear_mps_, max_linear_high_angle_mps_);

    RangerCommand command;
    command.mode = MotionMode::kDualAckermann;
    command.linear_mps = clamp(vx, -limit, limit);
    command.yaw_radps = wz;
    command.steering_rad = angle;
    return command;
  }

  std::string spinHysteresisReason(
    const double raw_angle,
    const std::chrono::steady_clock::time_point now) {
    const double abs_angle = std::abs(raw_angle);
    if (spin_latched_) {
      if (std::chrono::duration<double>(now - spin_enter_time_).count() < spin_min_hold_s_) {
        return "spin_min_hold";
      }
      if (abs_angle > spin_exit_steering_threshold_rad_) {
        return "spin_hysteresis";
      }
      resetSpinHysteresis();
      return "";
    }

    if (abs_angle > spin_enter_steering_threshold_rad_) {
      if (spin_candidate_since_.time_since_epoch().count() == 0) {
        spin_candidate_since_ = now;
      }
      if (std::chrono::duration<double>(now - spin_candidate_since_).count() >= spin_enter_debounce_s_) {
        spin_latched_ = true;
        spin_enter_time_ = now;
        return "spin_enter";
      }
      return "";
    }

    spin_candidate_since_ = {};
    return "";
  }

  void resetSpinHysteresis() {
    spin_latched_ = false;
    spin_candidate_since_ = {};
    spin_enter_time_ = {};
  }

  double rawAckermannAngle(double vx, double wz) const {
    if (std::abs(vx) < 1.0e-6 || std::abs(wz) < 1.0e-6) {
      return 0.0;
    }
    const double radius = std::abs(vx) / std::abs(wz);
    const double angle = std::atan((wheelbase_m_ / 2.0) / std::max(radius, 1.0e-6));
    return std::copysign(angle, wz * vx);
  }

  geometry_msgs::msg::Twist commandToTwist(const RangerCommand & command) const {
    geometry_msgs::msg::Twist msg;
    switch (command.mode) {
      case MotionMode::kPark:
        return msg;
      case MotionMode::kSpin:
        msg.angular.z = command.yaw_radps;
        return msg;
      case MotionMode::kCrab:
        msg.linear.x = command.linear_mps;
        msg.linear.y = command.lateral_mps;
        msg.angular.z = command.yaw_radps;
        return msg;
      case MotionMode::kDualAckermann:
        msg.linear.x = command.linear_mps;
        msg.angular.z = command.yaw_radps;
        return msg;
    }
    return msg;
  }

  void publishCommand(
    const geometry_msgs::msg::Twist & cmd,
    MotionMode mode,
    const std::string & state,
    bool valid,
    const std::string & reason) {
    cmd_pub_->publish(cmd);

    const auto now = std::chrono::steady_clock::now();
    const RangerMotionMode desired_mode = desiredRangerMode(mode);
    const auto actual_age = actualMotionModeAgeSec(now);
    const bool actual_available = actual_motion_mode_.has_value();
    const bool actual_fresh =
      actual_available && actual_age.has_value() && actual_age.value() <= actual_motion_mode_max_age_sec_;
    const bool mode_aligned =
      actual_fresh && actual_motion_mode_->mode == desired_mode;
    const bool motion_commanded =
      std::abs(cmd.linear.x) > linear_eps_ || std::abs(cmd.linear.y) > lateral_eps_ ||
      std::abs(cmd.angular.z) > yaw_eps_;
    std::string alignment_state = "actual_unavailable";
    if (actual_fresh) {
      alignment_state = mode_aligned ? "aligned" : "waiting_actual_motion_mode";
    } else if (actual_available) {
      alignment_state = "actual_stale";
    }

    if (valid && motion_commanded && !mode_aligned && actual_fresh) {
      const auto & actual = *actual_motion_mode_;
      const double mismatch_age =
        desired_mode_mismatch_since_.time_since_epoch().count() == 0 ?
        0.0 :
        std::chrono::duration<double>(now - desired_mode_mismatch_since_).count();
      if (desired_mode_mismatch_since_.time_since_epoch().count() == 0 ||
        last_desired_mode_for_mismatch_ != desired_mode ||
        last_actual_mode_for_mismatch_ != actual.mode)
      {
        desired_mode_mismatch_since_ = now;
        last_desired_mode_for_mismatch_ = desired_mode;
        last_actual_mode_for_mismatch_ = actual.mode;
      } else if (mismatch_age >= mode_alignment_warn_after_sec_) {
        warnThrottled(
          "mode_alignment",
          std::string("desired motion mode ") + ranger_motion_mode_name(desired_mode) +
          " but actual " + ranger_motion_mode_name(actual.mode) +
          " from /" + actual.source + "; odom guards must use actual motion_mode");
      }
    } else {
      desired_mode_mismatch_since_ = {};
    }

    std_msgs::msg::String mode_msg;
    mode_msg.data = ranger_motion_mode_json(desired_mode, legacyModeName(mode));
    mode_pub_->publish(mode_msg);

    std::ostringstream out;
    out << "{\"state\":\"" << state << "\",\"valid\":" << (valid ? "true" : "false")
        << ",\"desired_mode\":\"" << legacyModeName(mode) << "\",\"reason\":\"" << reason
        << "\",\"desired_motion_mode\":{\"code\":" << static_cast<int>(ranger_motion_mode_code(desired_mode))
        << ",\"name\":\"" << ranger_motion_mode_name(desired_mode)
        << "\",\"short\":\"" << ranger_motion_mode_short_name(desired_mode)
        << "\",\"legacy\":\"" << legacyModeName(mode) << "\"}"
        << ",\"actual_motion_mode\":{";
    if (actual_available) {
      out << "\"available\":true,\"fresh\":" << (actual_fresh ? "true" : "false")
          << ",\"code\":" << static_cast<int>(ranger_motion_mode_code(actual_motion_mode_->mode))
          << ",\"name\":\"" << ranger_motion_mode_name(actual_motion_mode_->mode)
          << "\",\"short\":\"" << ranger_motion_mode_short_name(actual_motion_mode_->mode)
          << "\",\"source\":\"" << actual_motion_mode_->source << "\"";
      if (actual_age.has_value()) {
        out << ",\"age_sec\":" << (std::round(actual_age.value() * 10.0) / 10.0);
      }
    } else {
      out << "\"available\":false,\"fresh\":false,\"code\":255,\"name\":\"MOTION_MODE_UNKNOWN\","
          << "\"short\":\"unknown\",\"source\":\"\"";
    }
    out << "},\"mode_aligned\":" << (mode_aligned ? "true" : "false")
        << ",\"mode_alignment_state\":\"" << alignment_state
        << "\",\"cmd_out\":{\"linear_x\":" << cmd.linear.x << ",\"linear_y\":" << cmd.linear.y
        << ",\"angular_z\":" << cmd.angular.z << "}}";
    const std::string status = out.str();
    if (status != last_status_) {
      std_msgs::msg::String msg;
      msg.data = status;
      status_pub_->publish(msg);
      last_status_ = status;
    }
  }

  void warnThrottled(const std::string & key, const std::string & message, double period_s = 2.0) {
    const auto now = std::chrono::steady_clock::now();
    auto iter = last_warn_by_key_.find(key);
    if (iter != last_warn_by_key_.end() &&
      std::chrono::duration<double>(now - iter->second).count() < period_s) {
      return;
    }
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    last_warn_by_key_[key] = now;
  }

  static std::string lowercase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return text;
  }

  double wheelbase_m_{0.494};
  double track_m_{0.364};
  double max_linear_mps_{1.5};
  double max_linear_high_angle_mps_{0.75};
  double max_ackermann_angle_rad_{0.698};
  double max_spin_radps_{3.259};
  double max_lateral_mps_{0.08};
  double max_crab_yaw_radps_{0.15};
  double cmd_timeout_s_{0.25};
  std::string mode_policy_{"auto"};
  bool allow_reverse_{false};
  std::string lateral_policy_{"reject"};
  std::string dual_ackermann_angle_{"inner"};
  double spin_enter_steering_threshold_rad_{0.785};
  double spin_exit_steering_threshold_rad_{0.489};
  double spin_enter_debounce_s_{0.4};
  double spin_min_hold_s_{1.0};
  double auto_spin_max_linear_mps_{0.08};
  bool spin_on_high_curvature_while_moving_{false};
  double reverse_enable_timeout_s_{0.75};
  double actual_motion_mode_max_age_sec_{0.5};
  double mode_alignment_warn_after_sec_{0.25};
  double linear_eps_{0.02};
  double lateral_eps_{0.02};
  double yaw_eps_{0.02};
  bool park_requested_{false};
  std::optional<ActualMotionMode> actual_motion_mode_;
  std::chrono::steady_clock::time_point desired_mode_mismatch_since_{};
  RangerMotionMode last_desired_mode_for_mismatch_{RangerMotionMode::UNKNOWN};
  RangerMotionMode last_actual_mode_for_mismatch_{RangerMotionMode::UNKNOWN};
  std::map<std::string, ReversePermit> reverse_permits_by_source_;
  std::set<std::string> subscribed_reverse_topics_;
  std::string forced_policy_;
  std::string last_status_;
  bool spin_latched_{false};
  std::chrono::steady_clock::time_point spin_candidate_since_{};
  std::chrono::steady_clock::time_point spin_enter_time_{};
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  std::map<std::string, std::chrono::steady_clock::time_point> last_warn_by_key_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<ranger_msgs::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Subscription<ranger_msgs::msg::SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forced_mode_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> reverse_enable_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr park_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangerMini3ModeController>());
  rclcpp::shutdown();
  return 0;
}
