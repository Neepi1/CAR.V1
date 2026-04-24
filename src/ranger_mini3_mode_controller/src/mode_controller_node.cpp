#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace {

enum class MotionMode : int {
  kDualAckermann = 0,
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

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

std::string modeName(MotionMode mode) {
  switch (mode) {
    case MotionMode::kDualAckermann:
      return "dual_ackermann";
    case MotionMode::kSpin:
      return "spin";
    case MotionMode::kPark:
      return "park";
  }
  return "unknown";
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
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("cmd_timeout_s", 0.25);
    declare_parameter<std::string>("mode_policy", "auto");
    declare_parameter<bool>("allow_reverse", false);
    declare_parameter<std::string>("lateral_policy", "reject");
    declare_parameter<std::string>("dual_ackermann_angle", "inner");
    declare_parameter<double>("spin_steering_threshold_rad", 0.698);
    declare_parameter<double>("linear_epsilon_mps", 0.02);
    declare_parameter<double>("lateral_epsilon_mps", 0.02);
    declare_parameter<double>("yaw_epsilon_radps", 0.02);
    declare_parameter<bool>("publish_zero_on_startup", true);
    declare_parameter<std::string>("status_topic", "/ranger_mini3_mode_controller/status");
    declare_parameter<std::string>("desired_mode_topic", "/ranger_mini3/desired_motion_mode");
    declare_parameter<std::string>("forced_mode_topic", "/ranger_mini3/forced_mode");
    declare_parameter<std::string>("park_topic", "/ranger_mini3/park");

    wheelbase_m_ = get_parameter("wheelbase_m").as_double();
    track_m_ = get_parameter("track_m").as_double();
    max_linear_mps_ = get_parameter("max_linear_mps").as_double();
    max_linear_high_angle_mps_ = get_parameter("max_linear_high_angle_mps").as_double();
    max_ackermann_angle_rad_ = get_parameter("max_ackermann_angle_rad").as_double();
    max_spin_radps_ = get_parameter("max_spin_radps").as_double();
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    mode_policy_ = get_parameter("mode_policy").as_string();
    allow_reverse_ = get_parameter("allow_reverse").as_bool();
    lateral_policy_ = get_parameter("lateral_policy").as_string();
    dual_ackermann_angle_ = get_parameter("dual_ackermann_angle").as_string();
    spin_steering_threshold_rad_ = get_parameter("spin_steering_threshold_rad").as_double();
    linear_eps_ = get_parameter("linear_epsilon_mps").as_double();
    lateral_eps_ = get_parameter("lateral_epsilon_mps").as_double();
    yaw_eps_ = get_parameter("yaw_epsilon_radps").as_double();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(), 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("desired_mode_topic").as_string(), 10);

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
      "Ranger Mini 3 C++ mode controller started: %s -> %s, policy=%s, lateral_policy=%s, allow_reverse=%s",
      get_parameter("cmd_vel_in_topic").as_string().c_str(),
      get_parameter("cmd_vel_out_topic").as_string().c_str(),
      mode_policy_.c_str(),
      lateral_policy_.c_str(),
      allow_reverse_ ? "true" : "false");
  }

 private:
  void onForcedMode(const std::string & input) {
    const std::string mode = lowercase(input);
    if (mode.empty() || mode == "auto") {
      forced_policy_.clear();
    } else if (mode == "dual_ackermann" || mode == "ackermann" || mode == "front_rear_ackermann") {
      forced_policy_ = "dual_ackermann";
    } else if (mode == "spin" || mode == "park") {
      forced_policy_ = mode;
    } else {
      warnThrottled(
        "bad_mode",
        "Unknown forced mode '" + input + "'. Use auto, dual_ackermann, spin, or park.");
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
      return RangerCommand{MotionMode::kPark};
    }

    if (last_cmd_time_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(now - last_cmd_time_).count() > cmd_timeout_s_) {
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
    if (!allow_reverse_ && vx < 0.0) {
      vx = 0.0;
    }

    std::string policy = forced_policy_.empty() ? mode_policy_ : forced_policy_;
    policy = lowercase(policy);
    if (policy == "dual_ackermann" || policy == "ackermann" || policy == "front_rear_ackermann") {
      return makeDualAckermann(vx, wz);
    }
    if (policy == "spin") {
      return makeSpin(wz);
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
      } else {
        RangerCommand invalid;
        invalid.valid = false;
        invalid.reason =
          "Lateral / crab commands are disabled for this Ranger Mini 3 profile; linear.y is not allowed.";
        return invalid;
      }
    }
    if (!has_vx && !has_vy && !has_wz) {
      return RangerCommand{MotionMode::kDualAckermann};
    }

    if (has_wz && !has_vx && !has_vy) {
      return makeSpin(wz);
    }
    return makeDualAckermann(vx, wz);
  }

  RangerCommand makeSpin(double wz) const {
    RangerCommand command;
    command.mode = MotionMode::kSpin;
    command.yaw_radps = clamp(wz, -max_spin_radps_, max_spin_radps_);
    return command;
  }

  RangerCommand makeDualAckermann(double vx, double wz) const {
    if (!allow_reverse_) {
      vx = std::max(0.0, vx);
    }
    const double raw_angle = rawAckermannAngle(vx, wz);
    if (std::abs(raw_angle) > spin_steering_threshold_rad_) {
      return makeSpin(wz);
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

    std_msgs::msg::String mode_msg;
    mode_msg.data = modeName(mode);
    mode_pub_->publish(mode_msg);

    std::ostringstream out;
    out << "{\"state\":\"" << state << "\",\"valid\":" << (valid ? "true" : "false")
        << ",\"desired_mode\":\"" << modeName(mode) << "\",\"reason\":\"" << reason
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
  double cmd_timeout_s_{0.25};
  std::string mode_policy_{"auto"};
  bool allow_reverse_{false};
  std::string lateral_policy_{"reject"};
  std::string dual_ackermann_angle_{"inner"};
  double spin_steering_threshold_rad_{0.698};
  double linear_eps_{0.02};
  double lateral_eps_{0.02};
  double yaw_eps_{0.02};
  bool park_requested_{false};
  std::string forced_policy_;
  std::string last_status_;
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  std::map<std::string, std::chrono::steady_clock::time_point> last_warn_by_key_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forced_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr park_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangerMini3ModeController>());
  rclcpp::shutdown();
  return 0;
}
