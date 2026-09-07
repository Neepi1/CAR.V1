#include "robot_nav_config/elevator_aware_progress_checker.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config {
namespace {

bool limits_are_valid(const PoseProgressLimits &limits) {
  return std::isfinite(limits.required_translation_m) &&
         std::isfinite(limits.required_yaw_rad) &&
         std::isfinite(limits.time_allowance_sec) &&
         limits.required_translation_m > 0.0 && limits.required_yaw_rad > 0.0 &&
         limits.time_allowance_sec > 0.0;
}

} // namespace

void ElevatorAwareProgressChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &plugin_name) {
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error(
        "ElevatorAwareProgressChecker parent node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  plugin_name_ = plugin_name;

  const auto declare_double = [&node, this](const char *suffix,
                                            const double default_value) {
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + suffix, rclcpp::ParameterValue(default_value));
  };
  declare_double(".required_movement_radius", 0.03);
  declare_double(".required_movement_angle", 0.05);
  declare_double(".movement_time_allowance", 12.0);
  declare_double(".elevator_required_movement_radius", 0.015);
  declare_double(".elevator_required_movement_angle", 0.015);
  declare_double(".elevator_movement_time_allowance", 20.0);
  declare_double(".elevator_progress_state_timeout", 0.50);
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".elevator_progress_state_topic",
      rclcpp::ParameterValue(
          std::string("/ranger_mini3/nav_elevator_scoped_progress_state")));

  PoseProgressLimits normal;
  PoseProgressLimits elevator;
  node->get_parameter(plugin_name_ + ".required_movement_radius",
                      normal.required_translation_m);
  node->get_parameter(plugin_name_ + ".required_movement_angle",
                      normal.required_yaw_rad);
  node->get_parameter(plugin_name_ + ".movement_time_allowance",
                      normal.time_allowance_sec);
  node->get_parameter(plugin_name_ + ".elevator_required_movement_radius",
                      elevator.required_translation_m);
  node->get_parameter(plugin_name_ + ".elevator_required_movement_angle",
                      elevator.required_yaw_rad);
  node->get_parameter(plugin_name_ + ".elevator_movement_time_allowance",
                      elevator.time_allowance_sec);
  node->get_parameter(plugin_name_ + ".elevator_progress_state_timeout",
                      progress_state_timeout_sec_);
  node->get_parameter(plugin_name_ + ".elevator_progress_state_topic",
                      progress_state_topic_);

  if (!limits_are_valid(normal) || !limits_are_valid(elevator) ||
      !std::isfinite(progress_state_timeout_sec_) ||
      progress_state_timeout_sec_ <= 0.0 || progress_state_topic_.empty()) {
    throw std::invalid_argument(
        "ElevatorAwareProgressChecker requires positive finite limits "
        "and a non-empty progress-state topic");
  }

  policy_ = std::make_unique<ElevatorAwareProgressPolicy>(normal, elevator);
  progress_state_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
      progress_state_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&ElevatorAwareProgressChecker::on_elevator_progress_state, this,
                std::placeholders::_1));

  RCLCPP_INFO(logger_,
              "Elevator-aware progress checker configured: "
              "normal=%.3fm/%.3frad/%.1fs elevator=%.3fm/%.3frad/%.1fs "
              "progress_state=%s/%.2fs",
              normal.required_translation_m, normal.required_yaw_rad,
              normal.time_allowance_sec, elevator.required_translation_m,
              elevator.required_yaw_rad, elevator.time_allowance_sec,
              progress_state_topic_.c_str(), progress_state_timeout_sec_);
}

bool ElevatorAwareProgressChecker::check(
    geometry_msgs::msg::PoseStamped &current_pose) {
  if (!clock_) {
    return false;
  }
  const auto now = clock_->now();
  const PoseProgressSample sample{
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      tf2::getYaw(current_pose.pose.orientation),
  };

  std::lock_guard<std::mutex> lock(mutex_);
  if (!policy_) {
    return false;
  }
  ElevatorScopedProgressState progress_state =
      ElevatorScopedProgressState::kOrdinary;
  if (last_progress_state_time_ && last_progress_state_) {
    const double progress_state_age_sec =
        (now - *last_progress_state_time_).seconds();
    if (progress_state_age_sec >= 0.0 &&
        progress_state_age_sec <= progress_state_timeout_sec_) {
      progress_state = *last_progress_state_;
    }
  }
  const bool elevator_scoped =
      elevator_scoped_progress_is_active(progress_state);
  const bool profile_changed = last_elevator_scoped_.has_value() &&
                               *last_elevator_scoped_ != elevator_scoped;
  last_elevator_scoped_ = elevator_scoped;
  const bool progressing =
      policy_->check(sample, now.seconds(), progress_state);
  if (profile_changed) {
    RCLCPP_INFO(logger_, "Progress checker profile changed to %s",
                elevator_scoped ? "elevator_scoped" : "normal");
  }
  return progressing;
}

void ElevatorAwareProgressChecker::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (policy_) {
    policy_->reset();
  }
  // A completed/cancelled controller action must not lend its still-fresh
  // elevator heartbeat to the next ordinary navigation action.
  last_progress_state_time_.reset();
  last_progress_state_.reset();
  last_elevator_scoped_.reset();
}

void ElevatorAwareProgressChecker::on_elevator_progress_state(
    const std_msgs::msg::UInt8::SharedPtr message) {
  if (!clock_ || !message ||
      !elevator_scoped_progress_state_is_valid(message->data)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_progress_state_time_ = clock_->now();
  last_progress_state_ =
      static_cast<ElevatorScopedProgressState>(message->data);
}

} // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(robot_nav_config::ElevatorAwareProgressChecker,
                       nav2_core::ProgressChecker)
