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
  kSideSlip = 3,
  kPark = 4,
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

struct PublishDecision {
  geometry_msgs::msg::Twist input;
  geometry_msgs::msg::Twist output;
  MotionMode predicted_mode{MotionMode::kDualAckermann};
  bool has_input{false};
  bool valid{true};
  std::string state{"ok"};
  std::string reason;
  std::string diff_reason;
};

void appendDiffReason(std::string & reason, const std::string & next) {
  if (next.empty()) {
    return;
  }
  if (!reason.empty()) {
    reason += ",";
  }
  reason += next;
}

std::string legacyModeName(MotionMode mode) {
  switch (mode) {
    case MotionMode::kDualAckermann:
      return "dual_ackermann";
    case MotionMode::kCrab:
      return "crab";
    case MotionMode::kSpin:
      return "spin";
    case MotionMode::kSideSlip:
      return "side_slip";
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
    case MotionMode::kSideSlip:
      return RangerMotionMode::SIDE_SLIP;
    case MotionMode::kPark:
      return RangerMotionMode::DUAL_ACKERMAN;
  }
  return RangerMotionMode::UNKNOWN;
}

}  // namespace

class RangerMini3ModeController : public rclcpp::Node {
 public:
  RangerMini3ModeController() : Node("ranger_mini3_mode_controller") {
    declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_safe");
    declare_parameter<std::string>(
      "cmd_vel_out_topic", "/ranger_mini3/mode_controller_shadow_cmd_vel");
    declare_parameter<std::string>("mode_controller_profile", "official_passthrough");
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("cmd_timeout_s", 0.25);
    declare_parameter<bool>("allow_reverse", false);
    declare_parameter<std::string>("lateral_policy", "reject");
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

    const auto requested_profile = lowercase(get_parameter("mode_controller_profile").as_string());
    if (!requested_profile.empty() && requested_profile != mode_controller_profile_) {
      RCLCPP_WARN(
        get_logger(),
        "mode_controller_profile '%s' is no longer supported; legacy custom Ackermann shaping was removed and official_passthrough is forced",
        requested_profile.c_str());
    }
    cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
    allow_reverse_ = get_parameter("allow_reverse").as_bool();
    lateral_policy_ = get_parameter("lateral_policy").as_string();
    reverse_enable_timeout_s_ = get_parameter("reverse_enable_timeout_s").as_double();
    actual_motion_mode_max_age_sec_ =
      std::max(0.05, get_parameter("actual_motion_mode_max_age_sec").as_double());
    mode_alignment_warn_after_sec_ =
      std::max(0.0, get_parameter("mode_alignment_warn_after_sec").as_double());
    linear_eps_ = get_parameter("linear_epsilon_mps").as_double();
    lateral_eps_ = get_parameter("lateral_epsilon_mps").as_double();
    yaw_eps_ = get_parameter("yaw_epsilon_radps").as_double();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
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
      PublishDecision startup;
      startup.output = geometry_msgs::msg::Twist{};
      startup.predicted_mode = MotionMode::kDualAckermann;
      startup.state = "startup_zero";
      startup.diff_reason = "startup_zero";
      publishDecision(startup);
    }

    RCLCPP_INFO(
      get_logger(),
      "Ranger Mini 3 C++ mode controller started: %s -> %s, mode_controller_profile=%s, cmd_vel_passthrough=true, lateral_policy=%s, allow_reverse=%s, legacy_custom_ackermann_removed=true, actual topics=%s,%s",
      get_parameter("cmd_vel_in_topic").as_string().c_str(),
      get_parameter("cmd_vel_out_topic").as_string().c_str(),
      mode_controller_profile_.c_str(),
      lateral_policy_.c_str(),
      allow_reverse_ ? "true" : "false",
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
    } else if (mode == "parallel" ||
      mode == "motion_mode_parallel" || mode == "1")
    {
      forced_policy_ = "crab";
    } else if (mode == "crab" || mode == "lateral" || mode == "sideways" ||
      mode == "side_slip" || mode == "sideslip" || mode == "motion_mode_side_slip" || mode == "3")
    {
      forced_policy_ = "side_slip";
    } else if (mode == "spin" || mode == "spinning" || mode == "motion_mode_spinning" || mode == "2") {
      forced_policy_ = "spin";
    } else if (mode == "park") {
      forced_policy_ = "park";
    } else {
      warnThrottled(
        "bad_mode",
        "Unknown forced mode '" + input +
        "'. Use auto, dual_ackerman, parallel, side_slip/crab/lateral, spinning/spin, park, or official enum code 0/1/2/3.");
      return;
    }
    RCLCPP_INFO(get_logger(), "forced mode set to %s", forced_policy_.empty() ? "auto" : forced_policy_.c_str());
  }

  void onTimer() {
    const auto now = std::chrono::steady_clock::now();
    publishDecision(computeOfficialPassthrough(now));
  }

  bool hasFreshCommand(const std::chrono::steady_clock::time_point now) const {
    return last_cmd_time_.time_since_epoch().count() != 0 &&
           std::chrono::duration<double>(now - last_cmd_time_).count() <= cmd_timeout_s_;
  }

  bool sourceReversePermitActive(
    const std::string & source,
    const std::chrono::steady_clock::time_point now) const
  {
    const auto iter = reverse_permits_by_source_.find(source);
    if (iter == reverse_permits_by_source_.end()) {
      return false;
    }
    const auto & permit = iter->second;
    return permit.enabled &&
           permit.stamp.time_since_epoch().count() != 0 &&
           std::chrono::duration<double>(now - permit.stamp).count() <= reverse_enable_timeout_s_;
  }

  MotionMode predictedModeFromTwist(const geometry_msgs::msg::Twist & twist) const {
    const bool has_vx = std::abs(twist.linear.x) >= linear_eps_;
    const bool has_vy = std::abs(twist.linear.y) >= lateral_eps_;
    const bool has_wz = std::abs(twist.angular.z) >= yaw_eps_;
    if (has_vy) {
      return MotionMode::kCrab;
    }
    if (!has_vx && has_wz) {
      return MotionMode::kSpin;
    }
    return MotionMode::kDualAckermann;
  }

  bool lateralAllowed() const {
    const std::string policy = lowercase(lateral_policy_);
    return policy == "allow" || policy == "crab" || policy == "parallel";
  }

  bool forcedLateralAllowed() const {
    return forced_policy_ == "crab" || forced_policy_ == "parallel" || forced_policy_ == "side_slip";
  }

  PublishDecision computeOfficialPassthrough(const std::chrono::steady_clock::time_point now) {
    PublishDecision decision;
    decision.state = "ok";
    decision.valid = true;

    if (last_cmd_time_.time_since_epoch().count() != 0) {
      decision.has_input = true;
      decision.input = last_cmd_;
      decision.output = last_cmd_;
      decision.predicted_mode = predictedModeFromTwist(last_cmd_);
    }

    if (park_requested_ || forced_policy_ == "park") {
      decision.output = geometry_msgs::msg::Twist{};
      decision.predicted_mode = MotionMode::kPark;
      decision.state = "park_zero";
      decision.reason = "park requested";
      appendDiffReason(decision.diff_reason, "park_requested");
      return decision;
    }

    if (!hasFreshCommand(now)) {
      decision.output = geometry_msgs::msg::Twist{};
      decision.predicted_mode = MotionMode::kDualAckermann;
      decision.state = "timeout_zero";
      decision.reason = "command timeout";
      appendDiffReason(decision.diff_reason, "timeout_zero");
      return decision;
    }

    if (!forced_policy_.empty()) {
      decision.reason = forcedLateralAllowed() ?
        "forced lateral mode active under official_passthrough" :
        "forced_mode diagnostic-only under official_passthrough";
    }

    if (decision.output.linear.x < 0.0 && !effectiveAllowReverse(now)) {
      decision.output.linear.x = 0.0;
      appendDiffReason(decision.diff_reason, "reverse_not_allowed");
    }

    if (std::abs(decision.output.linear.y) >= lateral_eps_ && !lateralAllowed() &&
      !forcedLateralAllowed())
    {
      decision.output.linear.y = 0.0;
      appendDiffReason(decision.diff_reason, "lateral_not_allowed");
    }

    if (std::abs(decision.output.linear.y) >= lateral_eps_) {
      decision.predicted_mode = forced_policy_ == "side_slip" ?
        MotionMode::kSideSlip : MotionMode::kCrab;
    }

    return decision;
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

  bool outputDiffersFromInput(const PublishDecision & decision) const {
    if (!decision.has_input) {
      return std::abs(decision.output.linear.x) > 1.0e-9 ||
             std::abs(decision.output.linear.y) > 1.0e-9 ||
             std::abs(decision.output.angular.z) > 1.0e-9;
    }
    return std::abs(decision.output.linear.x - decision.input.linear.x) > 1.0e-9 ||
           std::abs(decision.output.linear.y - decision.input.linear.y) > 1.0e-9 ||
           std::abs(decision.output.angular.z - decision.input.angular.z) > 1.0e-9;
  }

  void publishDecision(const PublishDecision & decision) {
    cmd_pub_->publish(decision.output);

    const auto now = std::chrono::steady_clock::now();
    const MotionMode mode = decision.predicted_mode;
    const RangerMotionMode desired_mode = desiredRangerMode(mode);
    const auto actual_age = actualMotionModeAgeSec(now);
    const bool actual_available = actual_motion_mode_.has_value();
    const bool actual_fresh =
      actual_available && actual_age.has_value() && actual_age.value() <= actual_motion_mode_max_age_sec_;
    const bool mode_aligned =
      actual_fresh && actual_motion_mode_->mode == desired_mode;
    const bool motion_commanded =
      std::abs(decision.output.linear.x) > linear_eps_ ||
      std::abs(decision.output.linear.y) > lateral_eps_ ||
      std::abs(decision.output.angular.z) > yaw_eps_;
    std::string alignment_state = "actual_unavailable";
    if (actual_fresh) {
      alignment_state = mode_aligned ? "aligned" : "waiting_actual_motion_mode";
    } else if (actual_available) {
      alignment_state = "actual_stale";
    }

    if (decision.valid && motion_commanded && !mode_aligned && actual_fresh) {
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

    const bool output_diff_from_input = outputDiffersFromInput(decision);
    const bool passthrough_preserves_twist = decision.has_input && !output_diff_from_input;
    const bool docking_reverse_active = sourceReversePermitActive("docking", now);
    const bool teleop_reverse_active = sourceReversePermitActive("teleop", now);
    const bool legacy_reverse_active = sourceReversePermitActive("legacy", now);
    const bool reverse_permit_active =
      allow_reverse_ || docking_reverse_active || teleop_reverse_active || legacy_reverse_active;
    const std::string desired_source = "predicted_from_cmd_vel_safe";

    std::ostringstream out;
    out << "{\"state\":\"" << decision.state << "\",\"valid\":"
        << (decision.valid ? "true" : "false")
        << ",\"mode_controller_profile\":\"" << mode_controller_profile_
        << "\",\"legacy_custom_ackermann_removed\":true"
        << ",\"cmd_vel_passthrough\":true"
        << ",\"passthrough_preserves_twist\":" << (passthrough_preserves_twist ? "true" : "false")
        << ",\"output_diff_from_input\":" << (output_diff_from_input ? "true" : "false")
        << ",\"diff_reason\":\"" << decision.diff_reason
        << "\",\"desired_mode\":\"" << legacyModeName(mode) << "\",\"reason\":\"" << decision.reason
        << "\",\"desired_motion_mode\":{\"code\":" << static_cast<int>(ranger_motion_mode_code(desired_mode))
        << ",\"name\":\"" << ranger_motion_mode_name(desired_mode)
        << "\",\"short\":\"" << ranger_motion_mode_short_name(desired_mode)
        << "\",\"legacy\":\"" << legacyModeName(mode)
        << "\",\"source\":\"" << desired_source << "\"}"
        << ",\"desired_motion_mode_source\":\"" << desired_source << "\""
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
    out << "},\"actual_motion_mode_source\":\""
        << (actual_available ? actual_motion_mode_->source : "")
        << "\",\"mode_aligned\":" << (mode_aligned ? "true" : "false")
        << ",\"motion_mode_matched\":" << (mode_aligned ? "true" : "false")
        << ",\"mode_alignment_state\":\"" << alignment_state
        << "\",\"reverse_permit_active\":" << (reverse_permit_active ? "true" : "false")
        << ",\"docking_reverse_permit_active\":" << (docking_reverse_active ? "true" : "false")
        << ",\"teleop_reverse_permit_active\":" << (teleop_reverse_active ? "true" : "false")
        << ",\"cmd_in\":{\"available\":" << (decision.has_input ? "true" : "false")
        << ",\"linear_x\":" << decision.input.linear.x
        << ",\"linear_y\":" << decision.input.linear.y
        << ",\"angular_z\":" << decision.input.angular.z
        << "},\"cmd_out\":{\"linear_x\":" << decision.output.linear.x
        << ",\"linear_y\":" << decision.output.linear.y
        << ",\"angular_z\":" << decision.output.angular.z << "}}";
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

  double cmd_timeout_s_{0.25};
  std::string mode_controller_profile_{"official_passthrough"};
  bool allow_reverse_{false};
  std::string lateral_policy_{"reject"};
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
