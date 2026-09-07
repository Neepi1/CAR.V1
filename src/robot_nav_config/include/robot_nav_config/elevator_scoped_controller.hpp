#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "robot_nav_config/elevator_localization_replan_gate.hpp"
#include "robot_nav_config/elevator_scoped_blockage.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "robot_nav_config/elevator_scoped_execution_session.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"
#include "robot_nav_config/elevator_scoped_plan_update.hpp"
#include "robot_nav_config/elevator_scoped_replan_progress.hpp"
#include "robot_nav_config/elevator_scoped_replan_worker.hpp"
#include "robot_nav_config/elevator_scoped_route.hpp"
#include "robot_nav_config/elevator_scoped_search.hpp"
#include "robot_nav_config/terminal_pose_handoff.hpp"
#include "robot_interfaces/srv/set_elevator_navigation_session.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace robot_nav_config {

class ElevatorScopedController : public nav2_core::Controller {
public:
  ElevatorScopedController() = default;
  ~ElevatorScopedController() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path &path) override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker *goal_checker) override;

  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

private:
  std::optional<TerminalPoseError>
  pose_error(const geometry_msgs::msg::PoseStamped &pose);
  std::optional<ElevatorLocalizationReplanObservation>
  canonical_goal_in_control_frame();
  ElevatorScopedPathClearanceResult
  command_clearance(const geometry_msgs::msg::PoseStamped &pose,
                    const TerminalVelocityCommand &command,
                    const ElevatorScopedCommandProjection &projection);
  bool schedule_route_revision(
      const geometry_msgs::msg::PoseStamped &pose,
      std::optional<ElevatorScopedPathClearanceResult> blocking_evidence =
          std::nullopt);
  bool apply_route_revision_result();
  void reset_execution_state_locked(bool clear_plan);
  void handle_execution_session(
      const std::shared_ptr<
          robot_interfaces::srv::SetElevatorNavigationSession::Request>
          request,
      std::shared_ptr<
          robot_interfaces::srv::SetElevatorNavigationSession::Response>
          response);
  std::size_t current_target_index_locked() const;
  void publish_permits(bool lateral, bool reverse);
  void publish_progress_state(ElevatorScopedProgressState state);
  static const char *phase_name(TerminalControlPhase phase);
  static const char *route_phase_name(ElevatorScopedRoutePhase phase);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ElevatorScopedController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
      lateral_permit_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
      reverse_permit_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      progress_state_pub_;
  rclcpp::Service<
      robot_interfaces::srv::SetElevatorNavigationSession>::SharedPtr
      execution_session_service_;
  std::string lateral_permit_topic_;
  std::string reverse_permit_topic_;
  std::string progress_state_topic_;

  std::mutex mutex_;
  ElevatorScopedExecutionSession execution_session_;
  bool plugin_active_{false};
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped final_goal_;
  std::vector<ElevatorScopedRouteSegment> route_segments_;
  std::size_t route_segment_index_{0U};
  bool route_execution_active_{false};
  std::size_t plan_generation_{0U};
  TerminalHandoffParameters parameters_;
  TerminalPoseHandoffController controller_;
  ElevatorScopedBlockage blockage_;
  ElevatorScopedReplanProgress replan_progress_;
  std::optional<std::chrono::steady_clock::time_point>
      last_route_revision_attempt_;
  ElevatorScopedSearchParameters route_revision_parameters_;
  ElevatorLocalizationReplanGate localization_replan_gate_;
  bool route_revision_enabled_{true};
  bool command_clearance_check_enabled_{true};
  double route_revision_interval_sec_{0.5};
  double costmap_lookahead_m_{0.15};
  double blocked_timeout_sec_{3.0};
  double speed_scale_{1.0};
  ElevatorScopedReplanWorker replan_worker_;
};

} // namespace robot_nav_config
