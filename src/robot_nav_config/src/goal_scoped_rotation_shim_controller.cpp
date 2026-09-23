#include "robot_nav_config/goal_scoped_rotation_shim_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_nav_config/terminal_rotation_braking.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config
{

void GoalScopedRotationShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  terminal_costmap_ros_ = costmap_ros;
  nav2_rotation_shim_controller::RotationShimController::configure(
    parent, std::move(name), std::move(tf), costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("GoalScopedRotationShimController parent node expired");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_once", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_change_xy_threshold", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_change_yaw_threshold", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".same_goal_rearm_after_idle_sec", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_rotation_braking_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_max_distance_m", rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_max_abs_forward_m", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_minimum_abs_lateral_m", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_dominance_ratio", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_path_length_ratio", rclcpp::ParameterValue(1.80));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_path_cross_track_m", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_goal_xy_tolerance_m", rclcpp::ParameterValue(0.06));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_goal_yaw_tolerance_rad", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_yaw_exit_tolerance_rad", rclcpp::ParameterValue(0.035));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_entry_m", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_exit_m", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_forward_entry_m", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_forward_exit_m", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_yaw_kp", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_yaw_min_speed_radps", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_yaw_max_speed_radps", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_kp", rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_max_speed_mps", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_forward_kp", rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_forward_max_speed_mps", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_reverse_max_speed_mps", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_settle_linear_speed_mps", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_settle_angular_speed_radps", rclcpp::ParameterValue(
      0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_settle_duration_sec", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_timeout_sec", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_costmap_lookahead_m", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_lateral_permit_topic",
    rclcpp::ParameterValue(std::string("/ranger_mini3/nav_terminal_lateral_enable")));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_handoff_reverse_permit_topic",
    rclcpp::ParameterValue(std::string("/ranger_mini3/nav_terminal_reverse_enable")));
  node->get_parameter(plugin_name_ + ".rotate_to_heading_once", rotate_to_heading_once_);
  node->get_parameter(plugin_name_ + ".goal_change_xy_threshold", goal_change_xy_threshold_);
  node->get_parameter(plugin_name_ + ".goal_change_yaw_threshold", goal_change_yaw_threshold_);
  node->get_parameter(
    plugin_name_ + ".same_goal_rearm_after_idle_sec", same_goal_rearm_after_idle_sec_);
  node->get_parameter(
    plugin_name_ + ".terminal_rotation_braking_enabled", terminal_rotation_braking_enabled_);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_enabled", terminal_handoff_parameters_.enabled);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_max_distance_m",
    terminal_handoff_parameters_.max_distance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_max_abs_forward_m",
    terminal_handoff_parameters_.max_abs_forward_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_minimum_abs_lateral_m",
    terminal_handoff_parameters_.minimum_abs_lateral_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_dominance_ratio",
    terminal_handoff_parameters_.lateral_dominance_ratio);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_path_length_ratio",
    terminal_handoff_parameters_.path_length_ratio_threshold);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_path_cross_track_m",
    terminal_handoff_parameters_.path_cross_track_threshold_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_goal_xy_tolerance_m",
    terminal_handoff_parameters_.goal_xy_tolerance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_goal_yaw_tolerance_rad",
    terminal_handoff_parameters_.goal_yaw_tolerance_rad);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_yaw_exit_tolerance_rad",
    terminal_handoff_parameters_.yaw_exit_tolerance_rad);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_entry_m",
    terminal_handoff_parameters_.lateral_entry_tolerance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_exit_m",
    terminal_handoff_parameters_.lateral_exit_tolerance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_forward_entry_m",
    terminal_handoff_parameters_.forward_entry_tolerance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_forward_exit_m",
    terminal_handoff_parameters_.forward_exit_tolerance_m);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_yaw_kp", terminal_handoff_parameters_.yaw_kp);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_yaw_min_speed_radps",
    terminal_handoff_parameters_.yaw_min_speed_radps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_yaw_max_speed_radps",
    terminal_handoff_parameters_.yaw_max_speed_radps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_kp", terminal_handoff_parameters_.lateral_kp);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_max_speed_mps",
    terminal_handoff_parameters_.lateral_max_speed_mps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_forward_kp", terminal_handoff_parameters_.forward_kp);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_forward_max_speed_mps",
    terminal_handoff_parameters_.forward_max_speed_mps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_reverse_max_speed_mps",
    terminal_handoff_parameters_.reverse_max_speed_mps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_settle_linear_speed_mps",
    terminal_handoff_parameters_.settle_linear_speed_threshold_mps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_settle_angular_speed_radps",
    terminal_handoff_parameters_.settle_angular_speed_threshold_radps);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_settle_duration_sec",
    terminal_handoff_parameters_.settle_stable_duration_sec);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_timeout_sec",
    terminal_handoff_parameters_.total_timeout_sec);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_costmap_lookahead_m", terminal_costmap_lookahead_m_);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_lateral_permit_topic",
    terminal_lateral_permit_topic_);
  node->get_parameter(
    plugin_name_ + ".terminal_handoff_reverse_permit_topic",
    terminal_reverse_permit_topic_);
  goal_change_xy_threshold_ = std::max(0.0, goal_change_xy_threshold_);
  goal_change_yaw_threshold_ = std::max(0.0, goal_change_yaw_threshold_);
  same_goal_rearm_after_idle_sec_ = std::max(0.0, same_goal_rearm_after_idle_sec_);
  terminal_handoff_parameters_.max_distance_m = std::max(
    terminal_handoff_parameters_.goal_xy_tolerance_m,
    terminal_handoff_parameters_.max_distance_m);
  terminal_handoff_parameters_.max_abs_forward_m = std::max(
    0.0, terminal_handoff_parameters_.max_abs_forward_m);
  terminal_handoff_parameters_.minimum_abs_lateral_m = std::max(
    0.0, terminal_handoff_parameters_.minimum_abs_lateral_m);
  terminal_handoff_parameters_.lateral_dominance_ratio = std::max(
    1.0, terminal_handoff_parameters_.lateral_dominance_ratio);
  terminal_handoff_parameters_.path_length_ratio_threshold = std::max(
    1.0, terminal_handoff_parameters_.path_length_ratio_threshold);
  terminal_handoff_parameters_.path_cross_track_threshold_m = std::max(
    0.0, terminal_handoff_parameters_.path_cross_track_threshold_m);
  terminal_handoff_parameters_.goal_xy_tolerance_m = std::max(
    0.001, terminal_handoff_parameters_.goal_xy_tolerance_m);
  terminal_handoff_parameters_.goal_yaw_tolerance_rad = std::max(
    0.001, terminal_handoff_parameters_.goal_yaw_tolerance_rad);
  terminal_handoff_parameters_.yaw_exit_tolerance_rad = std::clamp(
    terminal_handoff_parameters_.yaw_exit_tolerance_rad,
    0.001, terminal_handoff_parameters_.goal_yaw_tolerance_rad);
  terminal_handoff_parameters_.lateral_exit_tolerance_m = std::max(
    0.001, terminal_handoff_parameters_.lateral_exit_tolerance_m);
  terminal_handoff_parameters_.lateral_entry_tolerance_m = std::max(
    terminal_handoff_parameters_.lateral_exit_tolerance_m,
    terminal_handoff_parameters_.lateral_entry_tolerance_m);
  terminal_handoff_parameters_.forward_exit_tolerance_m = std::max(
    0.001, terminal_handoff_parameters_.forward_exit_tolerance_m);
  terminal_handoff_parameters_.forward_entry_tolerance_m = std::max(
    terminal_handoff_parameters_.forward_exit_tolerance_m,
    terminal_handoff_parameters_.forward_entry_tolerance_m);
  terminal_handoff_parameters_.yaw_kp = std::max(0.0, terminal_handoff_parameters_.yaw_kp);
  terminal_handoff_parameters_.yaw_min_speed_radps = std::max(
    0.0, terminal_handoff_parameters_.yaw_min_speed_radps);
  terminal_handoff_parameters_.yaw_max_speed_radps = std::max(
    terminal_handoff_parameters_.yaw_min_speed_radps,
    terminal_handoff_parameters_.yaw_max_speed_radps);
  terminal_handoff_parameters_.lateral_kp = std::max(
    0.0, terminal_handoff_parameters_.lateral_kp);
  terminal_handoff_parameters_.lateral_max_speed_mps = std::max(
    0.0, terminal_handoff_parameters_.lateral_max_speed_mps);
  terminal_handoff_parameters_.forward_kp = std::max(
    0.0, terminal_handoff_parameters_.forward_kp);
  terminal_handoff_parameters_.forward_max_speed_mps = std::max(
    0.0, terminal_handoff_parameters_.forward_max_speed_mps);
  terminal_handoff_parameters_.reverse_max_speed_mps = std::max(
    0.0, terminal_handoff_parameters_.reverse_max_speed_mps);
  terminal_handoff_parameters_.settle_linear_speed_threshold_mps = std::max(
    0.0, terminal_handoff_parameters_.settle_linear_speed_threshold_mps);
  terminal_handoff_parameters_.settle_angular_speed_threshold_radps = std::max(
    0.0, terminal_handoff_parameters_.settle_angular_speed_threshold_radps);
  terminal_handoff_parameters_.settle_stable_duration_sec = std::max(
    0.0, terminal_handoff_parameters_.settle_stable_duration_sec);
  terminal_handoff_parameters_.total_timeout_sec = std::max(
    1.0, terminal_handoff_parameters_.total_timeout_sec);
  terminal_costmap_lookahead_m_ = std::max(0.02, terminal_costmap_lookahead_m_);
  terminal_handoff_controller_.set_parameters(terminal_handoff_parameters_);
  goal_scope_.set_thresholds(goal_change_xy_threshold_, goal_change_yaw_threshold_);

  const auto permit_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  if (!terminal_lateral_permit_topic_.empty()) {
    terminal_lateral_permit_pub_ =
      node->create_publisher<std_msgs::msg::Bool>(terminal_lateral_permit_topic_, permit_qos);
  }
  if (!terminal_reverse_permit_topic_.empty()) {
    terminal_reverse_permit_pub_ =
      node->create_publisher<std_msgs::msg::Bool>(terminal_reverse_permit_topic_, permit_qos);
  }
  ordinary_local_repair_runtime_ =
    std::make_unique<OrdinaryLocalPathRepairRuntime>();
  ordinary_local_repair_runtime_->configure(
    node, plugin_name_, tf_, terminal_costmap_ros_,
    terminal_handoff_parameters_.max_distance_m);
  // A recovery request only prepares a path-specific state reset. It never
  // changes controller lifecycle, publishes velocity or grants motion.
  ordinary_recovery_state_ = navigation_recovery::for_node(node->get_node_base_interface().get());
  using Prepare = robot_nav_config::srv::PrepareOrdinaryNavigationRecovery;
  ordinary_recovery_service_ = node->create_service<Prepare>(
    "~/prepare_ordinary_navigation_recovery",
    [state = ordinary_recovery_state_, clock = node->get_clock()](
      const Prepare::Request::SharedPtr request, Prepare::Response::SharedPtr response) {
      const auto observation = state->inspect(request->path,
        rclcpp::Time(request->attempt_started).nanoseconds(), clock->now().nanoseconds());
      response->matched = observation.matched;
      response->recoverable = observation.recoverable;
      response->waiting = observation.waiting;
      response->progress_epoch = observation.progress_epoch;
      if (request->inspect_only) {return;}
      response->prepared = state->prepare(request->path,
        rclcpp::Time(request->attempt_started).nanoseconds(), response->reason,
        rclcpp::Time(request->task_started).nanoseconds());
    });

  RCLCPP_INFO(
    logger_,
    "Goal-scoped RotationShim configured: rotate_to_heading_once=%s "
    "goal_change_xy_threshold=%.3f goal_change_yaw_threshold=%.3f "
    "same_goal_rearm_after_idle_sec=%.2f terminal_rotation_braking_enabled=%s",
    rotate_to_heading_once_ ? "true" : "false",
    goal_change_xy_threshold_,
    goal_change_yaw_threshold_,
    same_goal_rearm_after_idle_sec_,
    terminal_rotation_braking_enabled_ ? "true" : "false");
  RCLCPP_WARN(logger_,
    "NAVLITE controller event=diagnostics_ready schema=1 "
    "plugin=GoalScopedRotationShimController output_layer=outer_controller "
    "zero_repeat_max_hz=1");
  RCLCPP_INFO(
    logger_,
    "Ranger terminal handoff: enabled=%s envelope=%.2fm max_forward=%.2fm "
    "path_ratio=%.2f cross_track=%.2fm speeds(yaw/lateral/forward/reverse)="
    "%.2f/%.2f/%.2f/%.2f",
    terminal_handoff_parameters_.enabled ? "true" : "false",
    terminal_handoff_parameters_.max_distance_m,
    terminal_handoff_parameters_.max_abs_forward_m,
    terminal_handoff_parameters_.path_length_ratio_threshold,
    terminal_handoff_parameters_.path_cross_track_threshold_m,
    terminal_handoff_parameters_.yaw_max_speed_radps,
    terminal_handoff_parameters_.lateral_max_speed_mps,
    terminal_handoff_parameters_.forward_max_speed_mps,
    terminal_handoff_parameters_.reverse_max_speed_mps);
}

void GoalScopedRotationShimController::activate()
{
  nav2_rotation_shim_controller::RotationShimController::activate();
  ordinary_recovery_state_->set_active(true);
  if (terminal_lateral_permit_pub_ && !terminal_lateral_permit_pub_->is_activated()) {
    terminal_lateral_permit_pub_->on_activate();
  }
  if (terminal_reverse_permit_pub_ && !terminal_reverse_permit_pub_->is_activated()) {
    terminal_reverse_permit_pub_->on_activate();
  }
  if (ordinary_local_repair_runtime_) {
    ordinary_local_repair_runtime_->activate();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_scope_.reset();
    startup_alignment_guard_.reset();
    terminal_handoff_controller_.reset();
    have_last_compute_time_ = false;
    suppressed_same_goal_replans_ = 0U;
  }
  publish_terminal_permits(false, false);
}

void GoalScopedRotationShimController::deactivate()
{
  ordinary_recovery_state_->set_active(false);
  recovery_alignment_active_ = false;
  publish_terminal_permits(false, false);
  if (ordinary_local_repair_runtime_) {
    ordinary_local_repair_runtime_->deactivate();
  }
  if (terminal_lateral_permit_pub_ && terminal_lateral_permit_pub_->is_activated()) {
    terminal_lateral_permit_pub_->on_deactivate();
  }
  if (terminal_reverse_permit_pub_ && terminal_reverse_permit_pub_->is_activated()) {
    terminal_reverse_permit_pub_->on_deactivate();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    terminal_handoff_controller_.reset();
  }
  nav2_rotation_shim_controller::RotationShimController::deactivate();
}

void GoalScopedRotationShimController::cleanup()
{
  if (ordinary_recovery_state_) {
    ordinary_recovery_state_->set_active(false);
  }
  ordinary_recovery_service_.reset();
  ordinary_recovery_state_.reset();
  nav2_rotation_shim_controller::RotationShimController::cleanup();
}

GoalSignature GoalScopedRotationShimController::goal_signature(
  const nav_msgs::msg::Path & path) const
{
  const auto & goal = path.poses.back();
  GoalSignature signature;
  signature.frame_id = goal.header.frame_id.empty() ? path.header.frame_id : goal.header.frame_id;
  signature.x = goal.pose.position.x;
  signature.y = goal.pose.position.y;
  signature.yaw = tf2::getYaw(goal.pose.orientation);
  return signature;
}

void GoalScopedRotationShimController::setPlan(const nav_msgs::msg::Path & path)
{
  bool preserve_startup = false;
  const bool recovery_rearm = ordinary_recovery_state_ &&
    ordinary_recovery_state_->observe_plan(path, &preserve_startup);
  if (ordinary_local_repair_runtime_) {
    ordinary_local_repair_runtime_->set_plan(path);
  }
  if (path.poses.empty()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminal_handoff_controller_.reset();
    }
    publish_terminal_permits(false, false);
    nav2_rotation_shim_controller::RotationShimController::setPlan(path);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  recovery_alignment_active_ = recovery_rearm;
  recovery_alignment_logged_ = false;
  if (preserve_startup) {
    goal_scope_.mark_startup_alignment_consumed();
    terminal_handoff_controller_.reset();
    in_rotation_ = false;
    last_angular_vel_ = 0.0;
  }
  if (recovery_rearm) {
    goal_scope_.reset();
    startup_alignment_guard_.reset();
    terminal_handoff_controller_.reset();
    in_rotation_ = false;
    last_angular_vel_ = 0.0;
    RCLCPP_WARN(logger_,
      "[ordinary-recovery] consumed exact replanned path; startup alignment rearmed; poses=%zu",
      path.poses.size());
  }
  const auto now_steady = std::chrono::steady_clock::now();
  const bool idle_rearm = !preserve_startup && rotate_to_heading_once_ && goal_scope_.have_goal() &&
    have_last_compute_time_ && same_goal_rearm_after_idle_sec_ > 0.0 &&
    std::chrono::duration<double>(now_steady - last_compute_time_).count() >=
    same_goal_rearm_after_idle_sec_;
  if (idle_rearm) {
    goal_scope_.reset();
    startup_alignment_guard_.reset();
  }

  const bool goal_changed = goal_scope_.observe_goal(goal_signature(path));
  const bool startup_alignment_consumed = goal_scope_.startup_alignment_consumed();

  current_path_ = path;
  primary_controller_->setPlan(path);
  position_goal_checker_->reset();

  if (!rotate_to_heading_once_ || goal_changed || !startup_alignment_consumed) {
    path_updated_ = true;
    if (goal_changed) {
      in_rotation_ = false;
      startup_alignment_guard_.reset();
      terminal_handoff_controller_.reset();
      publish_terminal_permits(false, false);
      RCLCPP_INFO(
        logger_,
        "Goal-scoped RotationShim armed for a new navigation goal%s",
        idle_rearm ? " after controller idle" : "");
    }
    return;
  }

  path_updated_ = false;
  in_rotation_ = false;
  ++suppressed_same_goal_replans_;
  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 2000,
    "Goal-scoped RotationShim suppressed same_goal_replan count=%zu; "
    "updated path was forwarded to the primary controller",
    suppressed_same_goal_replans_);
}

GoalScopedRotationShimController::StartupHeadingMeasurement
GoalScopedRotationShimController::startup_path_heading_error(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path path;
  double sampling_distance = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path = current_path_;
    sampling_distance = forward_sampling_distance_;
  }

  StartupHeadingMeasurement measurement;
  if (path.poses.size() < 2U) {
    measurement.path_too_short = true;
    return measurement;
  }

  const auto & start = path.poses.front().pose.position;
  std::optional<geometry_msgs::msg::PoseStamped> sampled_pose;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & candidate = path.poses[index];
    const double dx = candidate.pose.position.x - start.x;
    const double dy = candidate.pose.position.y - start.y;
    if (std::hypot(dx, dy) >= sampling_distance) {
      sampled_pose = candidate;
      break;
    }
  }
  if (!sampled_pose.has_value()) {
    measurement.path_too_short = true;
    return measurement;
  }

  if (sampled_pose->header.frame_id.empty()) {
    sampled_pose->header.frame_id = path.header.frame_id;
  }
  if (sampled_pose->header.frame_id.empty() || pose.header.frame_id.empty()) {
    return measurement;
  }

  geometry_msgs::msg::PoseStamped sampled_in_pose_frame;
  if (sampled_pose->header.frame_id == pose.header.frame_id) {
    sampled_in_pose_frame = *sampled_pose;
  } else {
    // A zero stamp asks TF for the latest complete transform instead of a
    // transform a few milliseconds newer than odom->base_link.
    sampled_pose->header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        *sampled_pose, sampled_in_pose_frame, *tf_, pose.header.frame_id))
    {
      return measurement;
    }
  }

  const double dx = sampled_in_pose_frame.pose.position.x - pose.pose.position.x;
  const double dy = sampled_in_pose_frame.pose.position.y - pose.pose.position.y;
  if (std::hypot(dx, dy) <= 1.0e-6) {
    return measurement;
  }

  const double target_yaw = std::atan2(dy, dx);
  const double pose_yaw = tf2::getYaw(pose.pose.orientation);
  measurement.error = angles::shortest_angular_distance(pose_yaw, target_yaw);
  return measurement;
}

geometry_msgs::msg::TwistStamped GoalScopedRotationShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (ordinary_recovery_state_) {
    ordinary_recovery_state_->observe_control();
  }
  bool startup_alignment_pending = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_compute_time_ = std::chrono::steady_clock::now();
    have_last_compute_time_ = true;
    startup_alignment_pending =
      rotate_to_heading_once_ && !goal_scope_.startup_alignment_consumed();
  }

  if (startup_alignment_pending) {
    const auto measurement = startup_path_heading_error(pose);
    StartupAlignmentDecision decision;
    bool rotation_started = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      decision = startup_alignment_guard_.evaluate(
        measurement.error,
        measurement.path_too_short,
        angular_dist_threshold_,
        angular_disengage_threshold_);
      rotation_started = startup_alignment_guard_.rotation_started();
      if (recovery_alignment_active_ && !recovery_alignment_logged_ && measurement.error) {
        RCLCPP_WARN(logger_, "[ordinary-recovery] startup initial_yaw_error=%.6f decision=%s",
          *measurement.error,
          decision == StartupAlignmentDecision::kRotate ? "rotate" : "already_aligned");
        recovery_alignment_logged_ = true;
      }

      if (decision == StartupAlignmentDecision::kComplete) {
        path_updated_ = false;
        in_rotation_ = false;
        goal_scope_.mark_startup_alignment_consumed();
        if (recovery_alignment_active_) {
          RCLCPP_WARN(logger_,
            "[ordinary-recovery] startup complete final_yaw_error=%.6f path_too_short=%s",
            measurement.error.value_or(std::numeric_limits<double>::quiet_NaN()),
            measurement.path_too_short ? "true" : "false");
          recovery_alignment_active_ = false;
        }
      } else {
        path_updated_ = true;
        in_rotation_ = rotation_started;
      }
    }

    if (decision == StartupAlignmentDecision::kHold) {
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      last_angular_vel_ = 0.0;
      publish_terminal_permits(false, false);
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Startup RotationShim is holding zero command until the path heading can be measured");
      log_navlite_output("startup_heading_unavailable", command, pose);
      return command;
    }

    if (decision == StartupAlignmentDecision::kRotate) {
      publish_terminal_permits(false, false);
      auto command = computeRotateToHeadingCommand(*measurement.error, pose, velocity);
      last_angular_vel_ = command.twist.angular.z;
      log_navlite_output("startup_rotation", command, pose);
      return command;
    }
  }

  OrdinaryLocalPathRepairUpdate local_repair_update;
  if (ordinary_local_repair_runtime_) {
    local_repair_update = ordinary_local_repair_runtime_->update(pose);
  }
  if (local_repair_update.replacement_path.has_value()) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_path_ = *local_repair_update.replacement_path;
    primary_controller_->setPlan(*local_repair_update.replacement_path);
    position_goal_checker_->reset();
    path_updated_ = false;
    in_rotation_ = false;
  }
  if (local_repair_update.hold_position) {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    last_angular_vel_ = 0.0;
    publish_terminal_permits(false, false);
    log_navlite_output("repair_hold_position", command, pose);
    return command;
  }

  const auto pose_error = terminal_pose_error(pose);
  if (pose_error.has_value()) {
    bool should_evaluate_handoff = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      should_evaluate_handoff =
        terminal_handoff_controller_.phase() == TerminalControlPhase::kInactive;
    }

    if (should_evaluate_handoff) {
      const auto path_metrics = terminal_path_metrics(pose);
      if (path_metrics.has_value() && should_start_terminal_handoff(
          *pose_error, *path_metrics, terminal_handoff_parameters_))
      {
        const double now_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          terminal_handoff_controller_.begin(now_sec);
        }
        RCLCPP_WARN(
          logger_,
          "Ranger terminal handoff started inside FollowPath: distance=%.3fm "
          "forward=%.3fm lateral=%.3fm yaw=%.3frad path_length=%.3fm "
          "chord=%.3fm ratio=%.2f cross_track=%.3fm",
          pose_error->distance_m,
          pose_error->forward_m,
          pose_error->lateral_m,
          pose_error->yaw_rad,
          path_metrics->length_m,
          path_metrics->chord_m,
          path_metrics->length_m / std::max(1.0e-6, path_metrics->chord_m),
          path_metrics->max_cross_track_m);
      }
    }

    bool terminal_controller_started = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminal_controller_started =
        terminal_handoff_controller_.phase() != TerminalControlPhase::kInactive;
    }
    if (terminal_controller_started) {
      return terminal_handoff_command(pose, velocity, *pose_error);
    }
  }

  publish_terminal_permits(false, false);

  const auto compute_primary = [&]() {
      return nav2_rotation_shim_controller::RotationShimController::computeVelocityCommands(
        pose, velocity, goal_checker);
    };
  auto command = ordinary_local_repair_runtime_ ?
    ordinary_local_repair_runtime_->compute_command(pose, compute_primary) : compute_primary();

  constexpr double kPureRotationLinearEpsilon = 1.0e-6;
  if (terminal_rotation_braking_enabled_ &&
    std::abs(command.twist.linear.x) <= kPureRotationLinearEpsilon &&
    std::abs(command.twist.linear.y) <= kPureRotationLinearEpsilon &&
    std::abs(command.twist.angular.z) > 0.0)
  {
    const auto yaw_error = terminal_goal_yaw_error(pose, goal_checker);
    if (yaw_error.has_value()) {
      const double requested_speed = command.twist.angular.z;
      command.twist.angular.z = limit_terminal_rotation_speed(
        requested_speed, *yaw_error, max_angular_accel_);
      RCLCPP_DEBUG_THROTTLE(
        logger_, *clock_, 1000,
        "Terminal RotationShim braking: yaw_error=%.4f requested_wz=%.4f limited_wz=%.4f "
        "max_angular_decel=%.4f",
        *yaw_error, requested_speed, command.twist.angular.z, max_angular_accel_);
    }
  }

  const bool no_control = ordinary_local_repair_runtime_ &&
    ordinary_local_repair_runtime_->progress_state() ==
    ElevatorScopedProgressState::kOrdinaryLocalWaitClear;
  log_navlite_output(no_control ? "mppi_no_valid_control" : "rotation_shim_return", command, pose);
  return command;
}

void GoalScopedRotationShimController::log_navlite_output(
  const char * branch, const geometry_msgs::msg::TwistStamped & command,
  const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & v = command.twist;
  const bool zero = v.linear.x == 0.0 && v.linear.y == 0.0 && v.angular.z == 0.0;
  if (!navlite_output_log_.observe(std::string(branch) + (zero ? ":zero" : ":nonzero"), zero)) {
    return;
  }
  RCLCPP_WARN(logger_,
    "NAVLITE controller branch=%s zero=%d out=(%.6f,%.6f,%.6f) "
    "pose=(%.6f,%.6f) frame=%s source_stamp=%d.%09u",
    branch, zero, v.linear.x, v.linear.y, v.angular.z,
    pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str(),
    pose.header.stamp.sec, pose.header.stamp.nanosec);
}

std::optional<TerminalPoseError> GoalScopedRotationShimController::terminal_pose_error(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path = current_path_;
  }
  if (path.poses.empty() || pose.header.frame_id.empty()) {
    return std::nullopt;
  }

  auto goal = path.poses.back();
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = path.header.frame_id;
  }
  if (goal.header.frame_id.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped goal_in_pose_frame;
  if (goal.header.frame_id == pose.header.frame_id) {
    goal_in_pose_frame = goal;
  } else {
    goal.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        goal, goal_in_pose_frame, *tf_, pose.header.frame_id))
    {
      return std::nullopt;
    }
  }

  const double dx = goal_in_pose_frame.pose.position.x - pose.pose.position.x;
  const double dy = goal_in_pose_frame.pose.position.y - pose.pose.position.y;
  const double goal_yaw = tf2::getYaw(goal_in_pose_frame.pose.orientation);
  const double pose_yaw = tf2::getYaw(pose.pose.orientation);
  if (!std::isfinite(dx) || !std::isfinite(dy) ||
    !std::isfinite(goal_yaw) || !std::isfinite(pose_yaw))
  {
    return std::nullopt;
  }

  TerminalPoseError error;
  error.distance_m = std::hypot(dx, dy);
  error.forward_m = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  error.lateral_m = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  error.yaw_rad = angles::shortest_angular_distance(pose_yaw, goal_yaw);
  return error;
}

std::optional<TerminalPathMetrics> GoalScopedRotationShimController::terminal_path_metrics(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path = current_path_;
  }
  if (path.poses.size() < 2U || pose.header.frame_id.empty()) {
    return std::nullopt;
  }

  std::vector<geometry_msgs::msg::PoseStamped> transformed;
  transformed.reserve(path.poses.size());
  for (auto path_pose : path.poses) {
    if (path_pose.header.frame_id.empty()) {
      path_pose.header.frame_id = path.header.frame_id;
    }
    if (path_pose.header.frame_id.empty()) {
      return std::nullopt;
    }
    if (path_pose.header.frame_id == pose.header.frame_id) {
      transformed.push_back(std::move(path_pose));
      continue;
    }

    geometry_msgs::msg::PoseStamped transformed_pose;
    path_pose.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        path_pose, transformed_pose, *tf_, pose.header.frame_id))
    {
      return std::nullopt;
    }
    transformed.push_back(std::move(transformed_pose));
  }

  const double start_x = pose.pose.position.x;
  const double start_y = pose.pose.position.y;
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < transformed.size(); ++index) {
    const auto & point = transformed[index].pose.position;
    const double distance = std::hypot(point.x - start_x, point.y - start_y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  const auto & goal = transformed.back().pose.position;
  TerminalPathMetrics metrics;
  metrics.chord_m = std::hypot(goal.x - start_x, goal.y - start_y);
  metrics.length_m = nearest_distance;
  for (std::size_t index = nearest_index + 1U; index < transformed.size(); ++index) {
    const auto & previous = transformed[index - 1U].pose.position;
    const auto & current = transformed[index].pose.position;
    metrics.length_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }

  const double line_dx = goal.x - start_x;
  const double line_dy = goal.y - start_y;
  const double line_length_squared = line_dx * line_dx + line_dy * line_dy;
  if (line_length_squared <= 1.0e-9) {
    return metrics;
  }
  for (std::size_t index = nearest_index; index < transformed.size(); ++index) {
    const auto & point = transformed[index].pose.position;
    const double projection = std::clamp(
      ((point.x - start_x) * line_dx + (point.y - start_y) * line_dy) /
      line_length_squared,
      0.0, 1.0);
    const double closest_x = start_x + projection * line_dx;
    const double closest_y = start_y + projection * line_dy;
    metrics.max_cross_track_m = std::max(
      metrics.max_cross_track_m,
      std::hypot(point.x - closest_x, point.y - closest_y));
  }
  return metrics;
}

geometry_msgs::msg::TwistStamped
GoalScopedRotationShimController::terminal_handoff_command(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  const TerminalPoseError & error)
{
  TerminalControlInput input;
  input.error = error;
  input.actual_linear_x_mps = velocity.linear.x;
  input.actual_linear_y_mps = velocity.linear.y;
  input.actual_angular_z_radps = velocity.angular.z;
  input.now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  TerminalControlPhase previous_phase;
  TerminalControlOutput output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_phase = terminal_handoff_controller_.phase();
    output = terminal_handoff_controller_.update(input);
  }

  if (output.phase != previous_phase) {
    RCLCPP_INFO(
      logger_,
      "Ranger terminal handoff phase %s -> %s: distance=%.3f forward=%.3f "
      "lateral=%.3f yaw=%.3f",
      terminal_phase_name(previous_phase), terminal_phase_name(output.phase),
      error.distance_m, error.forward_m, error.lateral_m, error.yaw_rad);
  }
  if (output.failed) {
    publish_terminal_permits(false, false);
    throw std::runtime_error("Ranger terminal handoff timed out or received invalid odometry");
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  command.twist.linear.x = output.command.linear_x;
  command.twist.linear.y = output.command.linear_y;
  command.twist.angular.z = output.command.angular_z;

  const bool command_clear = terminal_command_is_clear(pose, output.command);
  if (!command_clear) {
    command.twist = geometry_msgs::msg::Twist{};
    publish_terminal_permits(false, false);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Ranger terminal handoff is holding zero: projected footprint is not clear");
    log_navlite_output("terminal_footprint_not_clear", command, pose);
    return command;
  }

  publish_terminal_permits(output.lateral_permit, output.reverse_permit);
  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 500,
    "Ranger terminal handoff phase=%s error=(%.3f,%.3f,%.3f,%.3f) "
    "cmd=(%.3f,%.3f,%.3f)",
    terminal_phase_name(output.phase), error.distance_m, error.forward_m,
    error.lateral_m, error.yaw_rad, command.twist.linear.x,
    command.twist.linear.y, command.twist.angular.z);
  log_navlite_output(terminal_phase_name(output.phase), command, pose);
  return command;
}

bool GoalScopedRotationShimController::terminal_command_is_clear(
  const geometry_msgs::msg::PoseStamped & pose,
  const TerminalVelocityCommand & command)
{
  const double linear_norm = std::hypot(command.linear_x, command.linear_y);
  if (linear_norm <= 1.0e-9 && std::abs(command.angular_z) <= 1.0e-9) {
    return true;
  }
  if (!terminal_costmap_ros_ || terminal_costmap_ros_->getCostmap() == nullptr) {
    return false;
  }

  auto check_pose = pose;
  const std::string costmap_frame = terminal_costmap_ros_->getGlobalFrameID();
  if (check_pose.header.frame_id.empty() || costmap_frame.empty()) {
    return false;
  }
  if (check_pose.header.frame_id != costmap_frame) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    check_pose.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        check_pose, transformed_pose, *tf_, costmap_frame))
    {
      return false;
    }
    check_pose = std::move(transformed_pose);
  }

  auto * costmap = terminal_costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  const auto footprint = terminal_costmap_ros_->getRobotFootprint();
  const double start_yaw = tf2::getYaw(check_pose.pose.orientation);
  if (!std::isfinite(start_yaw)) {
    return false;
  }

  constexpr int kSamples = 3;
  for (int sample = 0; sample <= kSamples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(kSamples);
    double sample_x = check_pose.pose.position.x;
    double sample_y = check_pose.pose.position.y;
    double sample_yaw = start_yaw;
    if (linear_norm > 1.0e-9) {
      const double body_x = ratio * terminal_costmap_lookahead_m_ *
        command.linear_x / linear_norm;
      const double body_y = ratio * terminal_costmap_lookahead_m_ *
        command.linear_y / linear_norm;
      sample_x += std::cos(start_yaw) * body_x - std::sin(start_yaw) * body_y;
      sample_y += std::sin(start_yaw) * body_x + std::cos(start_yaw) * body_y;
    } else {
      sample_yaw += ratio * std::copysign(0.10, command.angular_z);
    }

    const double footprint_cost = checker.footprintCostAtPose(
      sample_x, sample_y, sample_yaw, footprint);
    if (!std::isfinite(footprint_cost) || footprint_cost < 0.0 ||
      footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      return false;
    }
  }
  return true;
}

void GoalScopedRotationShimController::publish_terminal_permits(
  const bool lateral, const bool reverse)
{
  std_msgs::msg::Bool message;
  if (terminal_lateral_permit_pub_ && terminal_lateral_permit_pub_->is_activated()) {
    message.data = lateral;
    terminal_lateral_permit_pub_->publish(message);
  }
  if (terminal_reverse_permit_pub_ && terminal_reverse_permit_pub_->is_activated()) {
    message.data = reverse;
    terminal_reverse_permit_pub_->publish(message);
  }
}

const char * GoalScopedRotationShimController::terminal_phase_name(
  const TerminalControlPhase phase)
{
  switch (phase) {
    case TerminalControlPhase::kInactive:
      return "inactive";
    case TerminalControlPhase::kSettling:
      return "settling";
    case TerminalControlPhase::kYaw:
      return "yaw";
    case TerminalControlPhase::kLateral:
      return "lateral";
    case TerminalControlPhase::kForward:
      return "forward";
    case TerminalControlPhase::kComplete:
      return "complete";
    case TerminalControlPhase::kFailed:
      return "failed";
  }
  return "unknown";
}

std::optional<double> GoalScopedRotationShimController::terminal_goal_yaw_error(
  const geometry_msgs::msg::PoseStamped & pose,
  nav2_core::GoalChecker * goal_checker)
{
  if (goal_checker == nullptr || current_path_.poses.empty()) {
    return std::nullopt;
  }

  auto goal = current_path_.poses.back();
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = current_path_.header.frame_id;
  }
  goal.header.stamp = clock_->now();

  geometry_msgs::msg::PoseStamped goal_in_pose_frame;
  if (goal.header.frame_id.empty() || goal.header.frame_id == pose.header.frame_id) {
    goal_in_pose_frame = goal;
  } else if (!nav2_util::transformPoseInTargetFrame(
      goal, goal_in_pose_frame, *tf_, pose.header.frame_id))
  {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 1000,
      "Terminal RotationShim braking skipped: failed to transform goal from %s to %s",
      goal.header.frame_id.c_str(), pose.header.frame_id.c_str());
    return std::nullopt;
  }

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  goal_checker->getTolerances(pose_tolerance, velocity_tolerance);
  const double xy_tolerance = std::max(
    std::abs(pose_tolerance.position.x), std::abs(pose_tolerance.position.y));
  const double dx = goal_in_pose_frame.pose.position.x - pose.pose.position.x;
  const double dy = goal_in_pose_frame.pose.position.y - pose.pose.position.y;
  if (!std::isfinite(xy_tolerance) || xy_tolerance <= 0.0 ||
    std::hypot(dx, dy) > xy_tolerance)
  {
    return std::nullopt;
  }

  constexpr double kTwoPi = 6.28318530717958647692;
  const double pose_yaw = tf2::getYaw(pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_in_pose_frame.pose.orientation);
  return std::remainder(goal_yaw - pose_yaw, kTwoPi);
}

}  // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(
  robot_nav_config::GoalScopedRotationShimController,
  nav2_core::Controller)
