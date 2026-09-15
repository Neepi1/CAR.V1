#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "robot_nav_config/ordinary_local_path_repair.hpp"
#include "robot_nav_config/ordinary_local_path_repair_worker.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"

namespace robot_nav_config
{

struct OrdinaryLocalPathRepairUpdate
{
  bool hold_position{false};
  std::optional<nav_msgs::msg::Path> replacement_path;
};

class OrdinaryLocalPathRepairRuntime
{
public:
  OrdinaryLocalPathRepairRuntime() = default;
  ~OrdinaryLocalPathRepairRuntime() = default;

  void configure(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::string & parameter_prefix,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    double terminal_control_distance_m);
  void activate();
  void deactivate();
  void set_plan(const nav_msgs::msg::Path & path);
  OrdinaryLocalPathRepairUpdate update(
    const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::TwistStamped compute_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::function<geometry_msgs::msg::TwistStamped()> & compute);
  ElevatorScopedProgressState progress_state() const;

private:
  enum class Mode
  {
    kTracking,
    kBlockagePending,
    kReplanning,
    kWaitClear,
  };

  enum class ApplyStatus
  {
    kFailed,
    kOriginalPathCleared,
    kReplacementReady,
  };

  struct ApplyResult
  {
    ApplyStatus status{ApplyStatus::kFailed};
    std::optional<nav_msgs::msg::Path> replacement_path;
  };

  std::optional<OrdinaryLocalPathRepairRequest> make_request(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path) const;
  std::optional<nav_msgs::msg::Path> transform_path(
    const nav_msgs::msg::Path & path,
    const std::string & target_frame) const;
  bool goal_is_in_terminal_scope(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path) const;
  ApplyResult apply_result(
    const geometry_msgs::msg::PoseStamped & pose,
    const OrdinaryLocalPathRepairWorkResult & result);
  void reset_state_locked();
  void publish_progress();

  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("OrdinaryLocalPathRepairRuntime")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr progress_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr repair_path_pub_;
  std::unique_ptr<OrdinaryLocalPathRepairWorker> worker_;

  mutable std::mutex mutex_;
  nav_msgs::msg::Path reference_path_;
  OrdinaryLocalPathRepairParameters parameters_;
  Mode mode_{Mode::kTracking};
  bool enabled_{true};
  bool mppi_primary_{false};
  bool control_waiting_{false};
  double collision_margin_m_{0.08};
  double preferred_margin_m_{0.0};
  double minimum_goal_distance_m_{0.75};
  double inspection_period_sec_{0.20};
  double persistence_sec_{0.20};
  double retry_period_sec_{0.50};
  std::string progress_topic_;
  std::string repair_path_topic_;
  std::size_t plan_generation_{0U};
  bool blockage_observed_{false};
  std::chrono::steady_clock::time_point blockage_since_;
  bool have_last_submit_time_{false};
  bool terminal_scope_active_{false};
  std::chrono::steady_clock::time_point last_submit_time_;
};

}  // namespace robot_nav_config
