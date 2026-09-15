#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_nav_config/elevator_aware_progress_policy.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"
#include "robot_nav_config/navigation_recovery/recovery_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace robot_nav_config {

class ElevatorAwareProgressChecker : public nav2_core::ProgressChecker {
public:
  ElevatorAwareProgressChecker() = default;
  ~ElevatorAwareProgressChecker() override = default;

  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &plugin_name) override;

  bool check(geometry_msgs::msg::PoseStamped &current_pose) override;
  void reset() override;

private:
  void
  on_elevator_progress_state(const std_msgs::msg::UInt8::SharedPtr message);

  rclcpp::Logger logger_{rclcpp::get_logger("ElevatorAwareProgressChecker")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr progress_state_sub_;
  std::string plugin_name_;
  std::string progress_state_topic_;
  double progress_state_timeout_sec_{0.50};

  std::mutex mutex_;
  std::optional<rclcpp::Time> last_progress_state_time_;
  std::optional<ElevatorScopedProgressState> last_progress_state_;
  std::optional<bool> last_elevator_scoped_;
  std::unique_ptr<ElevatorAwareProgressPolicy> policy_;
  std::shared_ptr<navigation_recovery::RecoveryState> ordinary_recovery_state_;
};

} // namespace robot_nav_config
