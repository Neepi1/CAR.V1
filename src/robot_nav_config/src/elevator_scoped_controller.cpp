#include "robot_nav_config/elevator_scoped_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "robot_nav_config/elevator_speed_limit.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void signature_mix(std::uint64_t &signature, const std::uint64_t value) {
  signature ^= value;
  signature *= kFnvPrime;
}

std::uint64_t quantized_signature_value(const double value,
                                        const double scale) {
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::llround(value * scale)));
}

std::uint64_t make_route_revision_signature(
    const nav2_costmap_2d::Costmap2D &costmap,
    const std::size_t plan_generation, const std::size_t route_segment_index,
    const ElevatorScopedPose &start,
    const std::optional<ElevatorScopedPathClearanceResult> &blocking_evidence) {
  std::uint64_t signature = kFnvOffsetBasis;
  signature_mix(signature, plan_generation);
  signature_mix(signature, route_segment_index);
  signature_mix(signature, costmap.getSizeInCellsX());
  signature_mix(signature, costmap.getSizeInCellsY());
  signature_mix(signature,
                quantized_signature_value(costmap.getResolution(), 1.0e6));
  signature_mix(signature,
                quantized_signature_value(costmap.getOriginX(), 1.0e4));
  signature_mix(signature,
                quantized_signature_value(costmap.getOriginY(), 1.0e4));
  signature_mix(signature, quantized_signature_value(start.x, 20.0));
  signature_mix(signature, quantized_signature_value(start.y, 20.0));
  signature_mix(signature, quantized_signature_value(start.yaw, 100.0));
  if (blocking_evidence) {
    signature_mix(signature, static_cast<std::uint64_t>(
                                 blocking_evidence->clearance.status));
    signature_mix(signature, blocking_evidence->clearance.cell_x);
    signature_mix(signature, blocking_evidence->clearance.cell_y);
    signature_mix(signature, blocking_evidence->clearance.cost);
    signature_mix(signature, blocking_evidence->sample_index);
  }
  const auto cell_count = static_cast<std::size_t>(costmap.getSizeInCellsX()) *
                          costmap.getSizeInCellsY();
  const auto *cells = costmap.getCharMap();
  for (std::size_t index = 0U; index < cell_count; ++index) {
    signature_mix(signature, cells[index]);
  }
  return signature;
}

bool blockage_can_trigger_route_revision(
    const ElevatorScopedClearanceStatus status) noexcept {
  return status == ElevatorScopedClearanceStatus::kOutOfMap ||
         status == ElevatorScopedClearanceStatus::kLethalObstacle ||
         status == ElevatorScopedClearanceStatus::kUnknownSpace;
}

} // namespace

void ElevatorScopedController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ElevatorScopedController parent node expired");
  }
  node_ = parent;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  if (!tf_ || !costmap_ros_ || !costmap_ros_->getCostmap()) {
    throw std::runtime_error(
        "ElevatorScopedController requires TF and a local costmap");
  }

  const auto declare_double = [&node, this](const char *suffix,
                                            const double value) {
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + suffix,
                                                 rclcpp::ParameterValue(value));
  };
  declare_double(".max_distance_m", 2.5);
  declare_double(".goal_xy_tolerance_m", 0.06);
  declare_double(".goal_yaw_tolerance_rad", 0.05);
  declare_double(".yaw_exit_tolerance_rad", 0.035);
  declare_double(".lateral_entry_m", 0.04);
  declare_double(".lateral_exit_m", 0.025);
  declare_double(".forward_entry_m", 0.04);
  declare_double(".forward_exit_m", 0.025);
  declare_double(".yaw_kp", 1.20);
  declare_double(".yaw_min_speed_radps", 0.08);
  declare_double(".yaw_max_speed_radps", 0.50);
  declare_double(".lateral_kp", 0.80);
  declare_double(".lateral_max_speed_mps", 0.40);
  declare_double(".forward_kp", 0.80);
  declare_double(".forward_max_speed_mps", 0.40);
  declare_double(".reverse_max_speed_mps", 0.40);
  declare_double(".settle_linear_speed_mps", 0.01);
  declare_double(".settle_angular_speed_radps", 0.02);
  declare_double(".settle_duration_sec", 0.30);
  declare_double(".total_timeout_sec", 45.0);
  declare_double(".blocked_timeout_sec", 3.0);
  declare_double(".costmap_lookahead_m", 0.15);
  declare_double(".route_revision_interval_sec", 0.5);
  declare_double(".route_revision_time_limit_sec", 1.00);
  declare_double(".localization_replan_translation_trigger_m", 0.08);
  declare_double(".localization_replan_yaw_trigger_rad", 0.08);
  declare_double(".localization_replan_stable_translation_m", 0.015);
  declare_double(".localization_replan_stable_yaw_rad", 0.015);
  declare_double(".localization_replan_stable_duration_sec", 0.60);
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".route_revision_enabled",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".command_clearance_check_enabled",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".prefer_clear_startup_spin",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".reverse_entry_staging_sequence",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".reverse_docking_sequence",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".localization_replan_enabled",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".route_revision_maximum_expansions",
      rclcpp::ParameterValue(60000));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".lateral_permit_topic",
      rclcpp::ParameterValue(
          std::string("/ranger_mini3/nav_terminal_lateral_enable")));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".reverse_permit_topic",
      rclcpp::ParameterValue(
          std::string("/ranger_mini3/nav_terminal_reverse_enable")));
  nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + ".progress_state_topic",
      rclcpp::ParameterValue(
          std::string("/ranger_mini3/nav_elevator_scoped_progress_state")));

  node->get_parameter(plugin_name_ + ".max_distance_m",
                      parameters_.max_distance_m);
  node->get_parameter(plugin_name_ + ".goal_xy_tolerance_m",
                      parameters_.goal_xy_tolerance_m);
  node->get_parameter(plugin_name_ + ".goal_yaw_tolerance_rad",
                      parameters_.goal_yaw_tolerance_rad);
  node->get_parameter(plugin_name_ + ".yaw_exit_tolerance_rad",
                      parameters_.yaw_exit_tolerance_rad);
  node->get_parameter(plugin_name_ + ".lateral_entry_m",
                      parameters_.lateral_entry_tolerance_m);
  node->get_parameter(plugin_name_ + ".lateral_exit_m",
                      parameters_.lateral_exit_tolerance_m);
  node->get_parameter(plugin_name_ + ".forward_entry_m",
                      parameters_.forward_entry_tolerance_m);
  node->get_parameter(plugin_name_ + ".forward_exit_m",
                      parameters_.forward_exit_tolerance_m);
  node->get_parameter(plugin_name_ + ".yaw_kp", parameters_.yaw_kp);
  node->get_parameter(plugin_name_ + ".yaw_min_speed_radps",
                      parameters_.yaw_min_speed_radps);
  node->get_parameter(plugin_name_ + ".yaw_max_speed_radps",
                      parameters_.yaw_max_speed_radps);
  node->get_parameter(plugin_name_ + ".lateral_kp", parameters_.lateral_kp);
  node->get_parameter(plugin_name_ + ".lateral_max_speed_mps",
                      parameters_.lateral_max_speed_mps);
  node->get_parameter(plugin_name_ + ".forward_kp", parameters_.forward_kp);
  node->get_parameter(plugin_name_ + ".forward_max_speed_mps",
                      parameters_.forward_max_speed_mps);
  node->get_parameter(plugin_name_ + ".reverse_max_speed_mps",
                      parameters_.reverse_max_speed_mps);
  node->get_parameter(plugin_name_ + ".settle_linear_speed_mps",
                      parameters_.settle_linear_speed_threshold_mps);
  node->get_parameter(plugin_name_ + ".settle_angular_speed_radps",
                      parameters_.settle_angular_speed_threshold_radps);
  node->get_parameter(plugin_name_ + ".settle_duration_sec",
                      parameters_.settle_stable_duration_sec);
  node->get_parameter(plugin_name_ + ".total_timeout_sec",
                      parameters_.total_timeout_sec);
  node->get_parameter(plugin_name_ + ".blocked_timeout_sec",
                      blocked_timeout_sec_);
  node->get_parameter(plugin_name_ + ".costmap_lookahead_m",
                      costmap_lookahead_m_);
  node->get_parameter(plugin_name_ + ".route_revision_enabled",
                      route_revision_enabled_);
  node->get_parameter(plugin_name_ + ".command_clearance_check_enabled",
                      command_clearance_check_enabled_);
  bool prefer_clear_startup_spin = false;
  node->get_parameter(plugin_name_ + ".prefer_clear_startup_spin",
                      prefer_clear_startup_spin);
  bool reverse_entry_staging_sequence = false;
  node->get_parameter(plugin_name_ + ".reverse_entry_staging_sequence",
                      reverse_entry_staging_sequence);
  bool reverse_docking_sequence = false;
  node->get_parameter(plugin_name_ + ".reverse_docking_sequence",
                      reverse_docking_sequence);
  const int selected_route_policies =
      static_cast<int>(prefer_clear_startup_spin) +
      static_cast<int>(reverse_entry_staging_sequence) +
      static_cast<int>(reverse_docking_sequence);
  if (selected_route_policies > 1) {
    throw std::runtime_error(
            "ElevatorScopedController route policies are mutually exclusive");
  }
  node->get_parameter(plugin_name_ + ".route_revision_interval_sec",
                      route_revision_interval_sec_);
  ElevatorLocalizationReplanParameters localization_replan_gate_parameters;
  node->get_parameter(plugin_name_ + ".localization_replan_enabled",
                      localization_replan_gate_parameters.enabled);
  node->get_parameter(
      plugin_name_ + ".localization_replan_translation_trigger_m",
      localization_replan_gate_parameters.translation_trigger_m);
  node->get_parameter(plugin_name_ + ".localization_replan_yaw_trigger_rad",
                      localization_replan_gate_parameters.yaw_trigger_rad);
  node->get_parameter(plugin_name_ +
                          ".localization_replan_stable_translation_m",
                      localization_replan_gate_parameters.stable_translation_m);
  node->get_parameter(plugin_name_ + ".localization_replan_stable_yaw_rad",
                      localization_replan_gate_parameters.stable_yaw_rad);
  node->get_parameter(plugin_name_ + ".localization_replan_stable_duration_sec",
                      localization_replan_gate_parameters.stable_duration_sec);
  double route_revision_time_limit_sec = 1.00;
  node->get_parameter(plugin_name_ + ".route_revision_time_limit_sec",
                      route_revision_time_limit_sec);
  int route_revision_maximum_expansions = 60000;
  node->get_parameter(plugin_name_ + ".route_revision_maximum_expansions",
                      route_revision_maximum_expansions);
  node->get_parameter(plugin_name_ + ".lateral_permit_topic",
                      lateral_permit_topic_);
  node->get_parameter(plugin_name_ + ".reverse_permit_topic",
                      reverse_permit_topic_);
  node->get_parameter(plugin_name_ + ".progress_state_topic",
                      progress_state_topic_);

  parameters_.enabled = true;
  parameters_.max_distance_m =
      std::clamp(parameters_.max_distance_m, 0.06, 2.5);
  parameters_.goal_xy_tolerance_m =
      std::max(0.001, parameters_.goal_xy_tolerance_m);
  parameters_.goal_yaw_tolerance_rad =
      std::max(0.001, parameters_.goal_yaw_tolerance_rad);
  parameters_.yaw_exit_tolerance_rad =
      std::clamp(parameters_.yaw_exit_tolerance_rad, 0.001,
                 parameters_.goal_yaw_tolerance_rad);
  parameters_.lateral_exit_tolerance_m =
      std::max(0.001, parameters_.lateral_exit_tolerance_m);
  parameters_.lateral_entry_tolerance_m =
      std::max(parameters_.lateral_exit_tolerance_m,
               parameters_.lateral_entry_tolerance_m);
  parameters_.forward_exit_tolerance_m =
      std::max(0.001, parameters_.forward_exit_tolerance_m);
  parameters_.forward_entry_tolerance_m =
      std::max(parameters_.forward_exit_tolerance_m,
               parameters_.forward_entry_tolerance_m);
  parameters_.yaw_kp = std::max(0.0, parameters_.yaw_kp);
  parameters_.yaw_min_speed_radps =
      std::max(0.0, parameters_.yaw_min_speed_radps);
  parameters_.yaw_max_speed_radps =
      std::clamp(parameters_.yaw_max_speed_radps,
                 parameters_.yaw_min_speed_radps, 0.50);
  parameters_.lateral_kp = std::max(0.0, parameters_.lateral_kp);
  parameters_.lateral_max_speed_mps =
      std::clamp(parameters_.lateral_max_speed_mps, 0.0, 0.40);
  parameters_.forward_kp = std::max(0.0, parameters_.forward_kp);
  parameters_.forward_max_speed_mps =
      std::clamp(parameters_.forward_max_speed_mps, 0.0, 0.40);
  parameters_.reverse_max_speed_mps =
      std::clamp(parameters_.reverse_max_speed_mps, 0.0, 0.40);
  parameters_.settle_linear_speed_threshold_mps =
      std::max(0.0, parameters_.settle_linear_speed_threshold_mps);
  parameters_.settle_angular_speed_threshold_radps =
      std::max(0.0, parameters_.settle_angular_speed_threshold_radps);
  parameters_.settle_stable_duration_sec =
      std::max(0.0, parameters_.settle_stable_duration_sec);
  parameters_.total_timeout_sec =
      std::clamp(parameters_.total_timeout_sec, 1.0, 60.0);
  blocked_timeout_sec_ = std::clamp(blocked_timeout_sec_, 0.5, 10.0);
  costmap_lookahead_m_ = std::clamp(costmap_lookahead_m_, 0.02, 0.20);
  route_revision_interval_sec_ =
      std::clamp(route_revision_interval_sec_, 0.20, 2.0);
  route_revision_parameters_.sampling.max_distance_m =
      parameters_.max_distance_m;
  route_revision_parameters_.route_policy =
      reverse_docking_sequence
          ? ElevatorScopedRoutePolicy::kReverseDockingSequence
          : (reverse_entry_staging_sequence
                ? ElevatorScopedRoutePolicy::kReverseEntryStagingSequence
                : (prefer_clear_startup_spin
                ? ElevatorScopedRoutePolicy::kPreferClearStartupSpin
                : ElevatorScopedRoutePolicy::kPreserveIngressHeading));
  route_revision_parameters_.sampling.translation_step_m = 0.025;
  route_revision_parameters_.sampling.rotation_step_rad = 0.05;
  route_revision_parameters_.search_grid_step_m = 0.05;
  route_revision_parameters_.search_radius_m = parameters_.max_distance_m;
  route_revision_parameters_.goal_connect_distance_m = 0.10;
  localization_replan_gate_parameters.translation_trigger_m = std::clamp(
      localization_replan_gate_parameters.translation_trigger_m, 0.06, 0.60);
  localization_replan_gate_parameters.yaw_trigger_rad = std::clamp(
      localization_replan_gate_parameters.yaw_trigger_rad, 0.05, 0.80);
  localization_replan_gate_parameters.stable_translation_m = std::clamp(
      localization_replan_gate_parameters.stable_translation_m, 0.005, 0.04);
  localization_replan_gate_parameters.stable_yaw_rad = std::clamp(
      localization_replan_gate_parameters.stable_yaw_rad, 0.005, 0.04);
  localization_replan_gate_parameters.stable_duration_sec = std::clamp(
      localization_replan_gate_parameters.stable_duration_sec, 0.20, 2.0);
  route_revision_time_limit_sec =
      std::clamp(route_revision_time_limit_sec, 0.10, 1.50);
  localization_replan_gate_.set_parameters(localization_replan_gate_parameters);
  route_revision_parameters_.maximum_expansions = static_cast<std::size_t>(
      std::clamp(route_revision_maximum_expansions, 1000, 100000));
  route_revision_parameters_.maximum_search_time_sec =
      route_revision_time_limit_sec;
  controller_.set_parameters(parameters_);

  const auto permit_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  lateral_permit_pub_ = node->create_publisher<std_msgs::msg::Bool>(
      lateral_permit_topic_, permit_qos);
  reverse_permit_pub_ = node->create_publisher<std_msgs::msg::Bool>(
      reverse_permit_topic_, permit_qos);
  progress_state_pub_ = node->create_publisher<std_msgs::msg::UInt8>(
      progress_state_topic_, permit_qos);
  execution_session_service_ =
      node->create_service<
          robot_interfaces::srv::SetElevatorNavigationSession>(
          "~/" + plugin_name_ + "/execution_session",
          std::bind(&ElevatorScopedController::handle_execution_session, this,
                    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      logger_,
      "Elevator scoped controller configured: envelope=%.2fm timeout=%.1fs "
      "blocked_wait_notice=%.1fs full_route_revision=%s/%.2fs budget=%.0fms "
      "localization_correction_diagnostic=%s/%.2fm/%.2frad "
      "route_policy=%s speeds"
      "(yaw/lateral/forward/reverse)=%.2f/%.2f/%.2f/%.2f",
      parameters_.max_distance_m, parameters_.total_timeout_sec,
      blocked_timeout_sec_, route_revision_enabled_ ? "enabled" : "disabled",
      route_revision_interval_sec_,
      route_revision_parameters_.maximum_search_time_sec * 1000.0,
      localization_replan_gate_parameters.enabled ? "enabled" : "disabled",
      localization_replan_gate_parameters.translation_trigger_m,
      localization_replan_gate_parameters.yaw_trigger_rad,
      reverse_docking_sequence ? "reverse_docking_sequence" :
      (reverse_entry_staging_sequence ? "reverse_entry_staging_sequence" :
      (prefer_clear_startup_spin ? "clear_startup_spin_then_escape"
                                 : "preserve_ingress_heading")),
      parameters_.yaw_max_speed_radps, parameters_.lateral_max_speed_mps,
      parameters_.forward_max_speed_mps, parameters_.reverse_max_speed_mps);
}

void ElevatorScopedController::cleanup() {
  replan_worker_.stop();
  publish_permits(false, false);
  execution_session_service_.reset();
  lateral_permit_pub_.reset();
  reverse_permit_pub_.reset();
  progress_state_pub_.reset();
  costmap_ros_.reset();
  tf_.reset();
  clock_.reset();
  node_.reset();
}

void ElevatorScopedController::activate() {
  replan_worker_.start();
  if (lateral_permit_pub_ && !lateral_permit_pub_->is_activated()) {
    lateral_permit_pub_->on_activate();
  }
  if (reverse_permit_pub_ && !reverse_permit_pub_->is_activated()) {
    reverse_permit_pub_->on_activate();
  }
  if (progress_state_pub_ && !progress_state_pub_->is_activated()) {
    progress_state_pub_->on_activate();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    execution_session_.reset();
    plugin_active_ = true;
    reset_execution_state_locked(true);
  }
  publish_permits(false, false);
}

void ElevatorScopedController::deactivate() {
  publish_permits(false, false);
  publish_progress_state(ElevatorScopedProgressState::kOrdinary);
  replan_worker_.stop();
  if (lateral_permit_pub_ && lateral_permit_pub_->is_activated()) {
    lateral_permit_pub_->on_deactivate();
  }
  if (reverse_permit_pub_ && reverse_permit_pub_->is_activated()) {
    reverse_permit_pub_->on_deactivate();
  }
  if (progress_state_pub_ && progress_state_pub_->is_activated()) {
    progress_state_pub_->on_deactivate();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  plugin_active_ = false;
  execution_session_.reset();
  reset_execution_state_locked(true);
}

void ElevatorScopedController::setPlan(const nav_msgs::msg::Path &path) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!plugin_active_ || !execution_session_.has_active_session()) {
      RCLCPP_ERROR(
          logger_,
          "Elevator scoped plan rejected: no explicit elevator execution "
          "session is active for controller %s",
          plugin_name_.c_str());
      return;
    }
  }
  std::vector<ElevatorScopedPose> poses;
  poses.reserve(path.poses.size());
  for (const auto &pose : path.poses) {
    poses.push_back({
        pose.pose.position.x,
        pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation),
    });
  }
  auto segments = make_elevator_scoped_route_segments(poses);
  const auto segment_count = segments.size();
  geometry_msgs::msg::PoseStamped replacement_goal =
      path.poses.empty() ? geometry_msgs::msg::PoseStamped{} : path.poses.back();
  if (replacement_goal.header.frame_id.empty()) {
    replacement_goal.header.frame_id = path.header.frame_id;
  }
  const ElevatorScopedGoalSignature replacement_signature{
      replacement_goal.header.frame_id,
      replacement_goal.pose.position.x,
      replacement_goal.pose.position.y,
      tf2::getYaw(replacement_goal.pose.orientation),
  };
  ElevatorScopedPlanUpdateDecision decision;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::optional<ElevatorScopedGoalSignature> active_signature;
    if (!final_goal_.header.frame_id.empty()) {
      active_signature = ElevatorScopedGoalSignature{
          final_goal_.header.frame_id,
          final_goal_.pose.position.x,
          final_goal_.pose.position.y,
          tf2::getYaw(final_goal_.pose.orientation),
      };
    }
    decision = decide_elevator_scoped_plan_update(
        active_signature, route_segments_, route_segment_index_,
        route_execution_active_,
        replacement_signature, segments);
    if (decision.action == ElevatorScopedPlanUpdateAction::kResetForNewGoal) {
      path_ = path;
      final_goal_ = replacement_goal;
      route_segments_ = std::move(segments);
      route_segment_index_ = decision.new_segment_index;
      route_execution_active_ = false;
      ++plan_generation_;
      controller_.reset();
      blockage_.reset();
      replan_progress_.reset();
      localization_replan_gate_.reset();
      last_route_revision_attempt_.reset();
    } else if (
      decision.action ==
      ElevatorScopedPlanUpdateAction::kHotSwapPreserveMotion)
    {
      // The replacement path starts at the current pose and has the same
      // canonical goal and active semantic motion.  Replace only the remaining
      // suffix; keep controller phase, permits and localization continuity.
      path_ = path;
      final_goal_ = replacement_goal;
      route_segments_ = std::move(segments);
      route_segment_index_ = decision.new_segment_index;
      ++plan_generation_;
      blockage_.reset();
      replan_progress_.reset();
      last_route_revision_attempt_.reset();
    }
  }
  if (decision.action == ElevatorScopedPlanUpdateAction::kRejectInvalidPlan) {
    RCLCPP_ERROR(
        logger_,
        "Elevator scoped route rejected: the replacement path or canonical "
        "goal is invalid");
    return;
  }
  if (decision.action == ElevatorScopedPlanUpdateAction::kKeepActivePlan) {
    RCLCPP_WARN(
        logger_,
        "Elevator scoped same-goal refresh ignored: its first semantic "
        "segment is incompatible with the motion already in progress");
    return;
  }
  (void)replan_worker_.take_result();
  if (decision.action == ElevatorScopedPlanUpdateAction::kResetForNewGoal) {
    publish_permits(false, false);
    RCLCPP_INFO(
        logger_,
        "Elevator scoped new goal received: poses=%zu executable_segments=%zu; "
        "controller state reset",
        path.poses.size(), segment_count);
  } else {
    RCLCPP_INFO(
        logger_,
        "Elevator scoped same-goal route hot-swapped: poses=%zu "
        "remaining_segments=%zu; active motion preserved",
        path.poses.size(), segment_count);
  }
}

geometry_msgs::msg::TwistStamped
ElevatorScopedController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!plugin_active_ || !execution_session_.has_active_session()) {
      publish_permits(false, false);
      publish_progress_state(ElevatorScopedProgressState::kOrdinary);
      throw std::runtime_error(
          "elevator scoped controller has no active execution session");
    }
  }
  (void)apply_route_revision_result();
  const double now_sec =
      std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  ElevatorLocalizationReplanAction localization_action =
      ElevatorLocalizationReplanAction::kTrack;
  bool material_localization_correction = false;
  const auto canonical_goal = canonical_goal_in_control_frame();
  if (canonical_goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    localization_action =
        localization_replan_gate_.observe(*canonical_goal, now_sec);
    material_localization_correction =
        localization_replan_gate_.consume_correction_event();
  }
  if (material_localization_correction) {
    RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "Elevator scoped map->odom correction observed; the active map-frame "
        "target is being transformed continuously without pausing FollowPath");
  }
  if (localization_action != ElevatorLocalizationReplanAction::kTrack) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      route_execution_active_ = false;
    }
    publish_permits(false, false);
    publish_progress_state(ElevatorScopedProgressState::kOrdinary);
    throw std::runtime_error(
        "elevator scoped localization continuity observation is invalid");
  }
  const auto error = pose_error(pose);
  if (!error || error->distance_m > parameters_.max_distance_m) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      route_execution_active_ = false;
    }
    publish_permits(false, false);
    publish_progress_state(ElevatorScopedProgressState::kOrdinary);
    throw std::runtime_error(
        "elevator scoped pose error is invalid or outside its envelope");
  }

  TerminalControlPhase previous_phase;
  TerminalControlOutput output;
  double speed_scale = 1.0;
  bool advanced_segment = false;
  std::size_t completed_segment = 0U;
  std::size_t next_segment = 0U;
  std::size_t segment_count = 0U;
  ElevatorScopedRoutePhase next_route_phase =
      ElevatorScopedRoutePhase::kForward;
  ElevatorScopedRoutePhase active_route_phase = ElevatorScopedRoutePhase::kPose;
  std::size_t active_route_segment = 0U;
  std::size_t active_route_segment_count = 0U;
  TerminalPoseError active_segment_error = *error;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (controller_.phase() == TerminalControlPhase::kInactive) {
      const double now_sec =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      controller_.begin(now_sec);
      route_execution_active_ = true;
    }
    TerminalControlInput input;
    input.error = *error;
    input.actual_linear_x_mps = velocity.linear.x;
    input.actual_linear_y_mps = velocity.linear.y;
    input.actual_angular_z_radps = velocity.angular.z;
    input.now_sec = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
    if (route_segment_index_ < route_segments_.size()) {
      active_route_phase = route_segments_[route_segment_index_].phase;
      active_route_segment = route_segment_index_;
      active_route_segment_count = route_segments_.size();
      input.error = make_elevator_scoped_segment_error(
          input.error, route_segments_[route_segment_index_]);
    }
    active_segment_error = input.error;
    previous_phase = controller_.phase();
    output = controller_.update(input);
    speed_scale = speed_scale_;
    if (output.complete && route_segment_index_ + 1U < route_segments_.size()) {
      completed_segment = route_segment_index_;
      ++route_segment_index_;
      next_segment = route_segment_index_;
      segment_count = route_segments_.size();
      next_route_phase = route_segments_[route_segment_index_].phase;
      controller_.reset();
      blockage_.observe_zero_command();
      advanced_segment = true;
    } else if (output.complete) {
      route_execution_active_ = false;
    } else if (output.failed) {
      route_execution_active_ = false;
    }
  }

  if (advanced_segment) {
    publish_permits(false, false);
    publish_progress_state(ElevatorScopedProgressState::kTracking);
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    RCLCPP_INFO(
        logger_,
        "Elevator scoped route segment %zu complete; advancing to %zu/%zu (%s)",
        completed_segment + 1U, next_segment + 1U, segment_count,
        route_phase_name(next_route_phase));
    return command;
  }

  if (output.phase != previous_phase) {
    RCLCPP_INFO(logger_,
                "Elevator scoped controller phase %s -> %s: distance=%.3f "
                "forward=%.3f lateral=%.3f yaw=%.3f planned=%s segment=%zu/%zu",
                phase_name(previous_phase), phase_name(output.phase),
                error->distance_m, error->forward_m, error->lateral_m,
                error->yaw_rad, route_phase_name(active_route_phase),
                active_route_segment + 1U, active_route_segment_count);
  }
  if (output.failed) {
    publish_permits(false, false);
    publish_progress_state(ElevatorScopedProgressState::kOrdinary);
    std::string session_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      session_id = execution_session_.active_session_id();
    }
    RCLCPP_ERROR(
        logger_,
        "Elevator scoped controller failed: controller=%s session=%s "
        "reason=%s",
        plugin_name_.c_str(), session_id.c_str(),
        terminal_control_failure_reason_name(output.failure_reason));
    throw std::runtime_error(
        std::string("elevator scoped controller failed: ") +
        terminal_control_failure_reason_name(output.failure_reason));
  }

  TerminalVelocityCommand limited = output.command;
  limited.linear_x *= speed_scale;
  limited.linear_y *= speed_scale;
  if (speed_scale <= 0.0) {
    limited.angular_z = 0.0;
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  command.twist.linear.x = limited.linear_x;
  command.twist.linear.y = limited.linear_y;
  command.twist.angular.z = limited.angular_z;

  const bool nonzero =
      std::hypot(limited.linear_x, limited.linear_y) > 1.0e-9 ||
      std::abs(limited.angular_z) > 1.0e-9;
  const auto projection = make_elevator_scoped_command_projection(
      active_segment_error, limited, costmap_lookahead_m_, 0.10);
  const auto clearance = command_clearance_check_enabled_
      ? command_clearance(pose, limited, projection)
      : ElevatorScopedPathClearanceResult{};
  if (command_clearance_check_enabled_ && nonzero && !clearance.is_clear()) {
    command.twist = geometry_msgs::msg::Twist{};
    publish_permits(false, false);
    bool persistent_block_notice = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      blockage_.observe_blocked(now);
      persistent_block_notice = blockage_.expired(now, blocked_timeout_sec_);
    }
    const bool replan_scheduled =
        blockage_can_trigger_route_revision(clearance.clearance.status) &&
        schedule_route_revision(pose, clearance);
    publish_progress_state(replan_scheduled || replan_worker_.busy()
                               ? ElevatorScopedProgressState::kReplanning
                               : ElevatorScopedProgressState::kWaitClear);
    if (persistent_block_notice) {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Elevator scoped controller remains in WAIT_CLEAR: the local "
          "footprint is blocked; holding zero until the costmap changes or a "
          "checked full route revision is available");
    }
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Elevator scoped controller is holding zero: projected footprint "
        "is not clear; status=%s sample=%zu cell=(%u,%u) cost=%u "
        "planned=%s segment=%zu/%zu",
        elevator_scoped_clearance_status_name(clearance.clearance.status),
        clearance.sample_index, clearance.clearance.cell_x,
        clearance.clearance.cell_y,
        static_cast<unsigned int>(clearance.clearance.cost),
        route_phase_name(active_route_phase), active_route_segment + 1U,
        active_route_segment_count);
    return command;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (nonzero) {
      blockage_.observe_verified_nonzero_command();
    } else {
      blockage_.observe_zero_command();
    }
  }
  publish_permits(output.lateral_permit && std::abs(limited.linear_y) > 0.0,
                  output.reverse_permit && limited.linear_x < 0.0);
  publish_progress_state(ElevatorScopedProgressState::kTracking);
  return command;
}

void ElevatorScopedController::setSpeedLimit(const double &speed_limit,
                                             const bool &percentage) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double configured_max = std::max({
      parameters_.lateral_max_speed_mps,
      parameters_.forward_max_speed_mps,
      parameters_.reverse_max_speed_mps,
  });
  speed_scale_ =
      elevator_speed_limit_scale(speed_limit, percentage, configured_max);
}

void ElevatorScopedController::reset_execution_state_locked(
    const bool clear_plan) {
  controller_.reset();
  blockage_.reset();
  replan_progress_.reset();
  localization_replan_gate_.reset();
  last_route_revision_attempt_.reset();
  route_segment_index_ = 0U;
  route_execution_active_ = false;
  ++plan_generation_;
  if (clear_plan) {
    path_ = nav_msgs::msg::Path{};
    final_goal_ = geometry_msgs::msg::PoseStamped{};
    route_segments_.clear();
  }
}

void ElevatorScopedController::handle_execution_session(
    const std::shared_ptr<
        robot_interfaces::srv::SetElevatorNavigationSession::Request> request,
    std::shared_ptr<
        robot_interfaces::srv::SetElevatorNavigationSession::Response>
        response) {
  using SessionService =
      robot_interfaces::srv::SetElevatorNavigationSession;
  ElevatorScopedExecutionSessionTransition transition;
  bool reset_performed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->operation == SessionService::Request::OP_BEGIN) {
      if (!plugin_active_) {
        response->detail = "controller_not_active";
        response->active_session_id = execution_session_.active_session_id();
        return;
      }
      transition = execution_session_.begin(request->session_id);
    } else if (request->operation == SessionService::Request::OP_END) {
      transition = execution_session_.end(request->session_id);
    } else {
      response->detail = "invalid_operation";
      response->active_session_id = execution_session_.active_session_id();
      return;
    }
    if (transition.accepted && transition.reset_required) {
      reset_execution_state_locked(true);
      reset_performed = true;
    }
    response->accepted = transition.accepted;
    response->reset_performed = reset_performed;
    response->active_session_id = execution_session_.active_session_id();
    response->detail = transition.detail;
  }
  if (reset_performed) {
    (void)replan_worker_.take_result();
    publish_permits(false, false);
    publish_progress_state(ElevatorScopedProgressState::kOrdinary);
  }
  if (transition.accepted) {
    RCLCPP_INFO(
        logger_,
        "Elevator controller session %s: controller=%s session=%s reset=%s "
        "terminal_reason=%s",
        request->operation == SessionService::Request::OP_BEGIN ?
        "BEGIN" : "END",
        plugin_name_.c_str(), request->session_id.c_str(),
        reset_performed ? "true" : "false",
        request->terminal_reason.c_str());
  } else {
    RCLCPP_WARN(
        logger_,
        "Elevator controller session request rejected: controller=%s "
        "session=%s detail=%s",
        plugin_name_.c_str(), request->session_id.c_str(),
        transition.detail);
  }
}

std::optional<ElevatorLocalizationReplanObservation>
ElevatorScopedController::canonical_goal_in_control_frame() {
  geometry_msgs::msg::PoseStamped goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal = final_goal_;
  }
  if (goal.header.frame_id.empty() || !costmap_ros_ || !tf_) {
    return std::nullopt;
  }
  const auto control_frame = costmap_ros_->getGlobalFrameID();
  if (control_frame.empty()) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseStamped transformed_goal;
  if (goal.header.frame_id == control_frame) {
    transformed_goal = goal;
  } else {
    goal.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(goal, transformed_goal, *tf_,
                                               control_frame)) {
      return std::nullopt;
    }
  }
  const double yaw = tf2::getYaw(transformed_goal.pose.orientation);
  if (!std::isfinite(transformed_goal.pose.position.x) ||
      !std::isfinite(transformed_goal.pose.position.y) || !std::isfinite(yaw)) {
    return std::nullopt;
  }
  return ElevatorLocalizationReplanObservation{
      transformed_goal.pose.position.x,
      transformed_goal.pose.position.y,
      yaw,
  };
}

std::optional<TerminalPoseError> ElevatorScopedController::pose_error(
    const geometry_msgs::msg::PoseStamped &pose) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped canonical_goal;
  std::size_t target_index = 0U;
  bool use_canonical_goal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path = path_;
    canonical_goal = final_goal_;
    target_index = current_target_index_locked();
    use_canonical_goal = route_segments_.empty() ||
                         route_segment_index_ >= route_segments_.size() ||
                         route_segments_[route_segment_index_].phase ==
                             ElevatorScopedRoutePhase::kPose;
  }
  if ((path.poses.empty() && !use_canonical_goal) ||
      pose.header.frame_id.empty()) {
    return std::nullopt;
  }
  geometry_msgs::msg::PoseStamped goal;
  if (use_canonical_goal) {
    goal = canonical_goal;
  } else {
    target_index = std::min(target_index, path.poses.size() - 1U);
    goal = path.poses[target_index];
  }
  if (goal.header.frame_id.empty() && !path.header.frame_id.empty()) {
    goal.header.frame_id = path.header.frame_id;
  }
  if (goal.header.frame_id.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped transformed_goal;
  if (goal.header.frame_id == pose.header.frame_id) {
    transformed_goal = goal;
  } else {
    goal.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(goal, transformed_goal, *tf_,
                                               pose.header.frame_id)) {
      return std::nullopt;
    }
  }

  const double dx = transformed_goal.pose.position.x - pose.pose.position.x;
  const double dy = transformed_goal.pose.position.y - pose.pose.position.y;
  const double goal_yaw = tf2::getYaw(transformed_goal.pose.orientation);
  const double pose_yaw = tf2::getYaw(pose.pose.orientation);
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(goal_yaw) ||
      !std::isfinite(pose_yaw)) {
    return std::nullopt;
  }

  TerminalPoseError error;
  error.distance_m = std::hypot(dx, dy);
  error.forward_m = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  error.lateral_m = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  error.yaw_rad = angles::shortest_angular_distance(pose_yaw, goal_yaw);
  return error;
}

std::size_t ElevatorScopedController::current_target_index_locked() const {
  if (path_.poses.empty()) {
    return 0U;
  }
  if (route_segments_.empty() ||
      route_segment_index_ >= route_segments_.size()) {
    return path_.poses.size() - 1U;
  }
  return std::min(route_segments_[route_segment_index_].end_index,
                  path_.poses.size() - 1U);
}

ElevatorScopedPathClearanceResult ElevatorScopedController::command_clearance(
    const geometry_msgs::msg::PoseStamped &pose,
    const TerminalVelocityCommand &command,
    const ElevatorScopedCommandProjection &projection) {
  ElevatorScopedPathClearanceResult result;
  const double linear_norm = std::hypot(command.linear_x, command.linear_y);
  if (linear_norm <= 1.0e-9 && std::abs(command.angular_z) <= 1.0e-9) {
    result.clearance.status = ElevatorScopedClearanceStatus::kClear;
    return result;
  }
  if (!costmap_ros_ || !costmap_ros_->getCostmap()) {
    return result;
  }

  auto check_pose = pose;
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  if (check_pose.header.frame_id.empty() || costmap_frame.empty()) {
    result.clearance.status =
        ElevatorScopedClearanceStatus::kTransformUnavailable;
    return result;
  }
  if (check_pose.header.frame_id != costmap_frame) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    check_pose.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(check_pose, transformed_pose,
                                               *tf_, costmap_frame)) {
      result.clearance.status =
          ElevatorScopedClearanceStatus::kTransformUnavailable;
      return result;
    }
    check_pose = std::move(transformed_pose);
  }

  auto *costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double start_yaw = tf2::getYaw(check_pose.pose.orientation);
  if (!std::isfinite(start_yaw)) {
    return result;
  }

  constexpr int kSamples = 3;
  std::vector<ElevatorScopedPathSample> samples;
  samples.reserve(kSamples + 1U);
  for (int sample = 0; sample <= kSamples; ++sample) {
    const double ratio =
        static_cast<double>(sample) / static_cast<double>(kSamples);
    double sample_x = check_pose.pose.position.x;
    double sample_y = check_pose.pose.position.y;
    double sample_yaw = start_yaw;
    if (linear_norm > 1.0e-9) {
      const double body_x =
          ratio * projection.translation_m * command.linear_x / linear_norm;
      const double body_y =
          ratio * projection.translation_m * command.linear_y / linear_norm;
      sample_x += std::cos(start_yaw) * body_x - std::sin(start_yaw) * body_y;
      sample_y += std::sin(start_yaw) * body_x + std::cos(start_yaw) * body_y;
    } else {
      sample_yaw +=
          ratio * std::copysign(projection.rotation_rad, command.angular_z);
    }
    ElevatorScopedMotionPhase phase = ElevatorScopedMotionPhase::kYaw;
    if (linear_norm > 1.0e-9) {
      phase =
          std::abs(command.linear_y) > std::abs(command.linear_x)
              ? ElevatorScopedMotionPhase::kLateral
              : (command.linear_x < 0.0 ? ElevatorScopedMotionPhase::kReverse
                                        : ElevatorScopedMotionPhase::kForward);
    }
    samples.push_back({phase, {sample_x, sample_y, sample_yaw}});
  }
  const auto lock_wait_started = std::chrono::steady_clock::now();
  result = evaluate_elevator_scoped_path_clearance_locked(
      *costmap, footprint, samples);
  const double lock_wait_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - lock_wait_started).count();
  if (lock_wait_ms > 20.0) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Elevator command clearance waited %.1fms for the local costmap "
        "snapshot; contention did not become a false obstacle",
        lock_wait_ms);
  }
  return result;
}

bool ElevatorScopedController::schedule_route_revision(
    const geometry_msgs::msg::PoseStamped &pose,
    const std::optional<ElevatorScopedPathClearanceResult> blocking_evidence) {
  if (!route_revision_enabled_ || !costmap_ros_ ||
      !costmap_ros_->getCostmap() || !tf_ ||
      replan_worker_.busy()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped final_goal;
  std::size_t captured_generation = 0U;
  std::size_t captured_route_segment = 0U;
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_route_revision_attempt_ &&
        std::chrono::duration<double>(now - *last_route_revision_attempt_)
                .count() < route_revision_interval_sec_) {
      return false;
    }
    final_goal = final_goal_;
    captured_generation = plan_generation_;
    captured_route_segment = route_segment_index_;
  }
  if (final_goal.header.frame_id.empty() || pose.header.frame_id.empty()) {
    return false;
  }

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  if (costmap_frame.empty()) {
    return false;
  }
  const auto transform_to =
      [this](geometry_msgs::msg::PoseStamped source,
             const std::string &target_frame,
             geometry_msgs::msg::PoseStamped &transformed) -> bool {
    if (source.header.frame_id == target_frame) {
      transformed = std::move(source);
      return true;
    }
    source.header.stamp = builtin_interfaces::msg::Time{};
    return nav2_util::transformPoseInTargetFrame(source, transformed, *tf_,
                                                 target_frame);
  };

  geometry_msgs::msg::PoseStamped local_start;
  geometry_msgs::msg::PoseStamped local_goal;
  if (!transform_to(pose, costmap_frame, local_start) ||
      !transform_to(final_goal, costmap_frame, local_goal)) {
    return false;
  }
  const ElevatorScopedPose search_start{
      local_start.pose.position.x,
      local_start.pose.position.y,
      tf2::getYaw(local_start.pose.orientation),
  };
  const ElevatorScopedPose search_goal{
      local_goal.pose.position.x,
      local_goal.pose.position.y,
      tf2::getYaw(local_goal.pose.orientation),
  };
  if (!elevator_scoped_pose_is_finite(search_start) ||
      !elevator_scoped_pose_is_finite(search_goal)) {
    return false;
  }

  auto *costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3U) {
    return false;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_snapshot;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
        *costmap->getMutex(), std::try_to_lock);
    if (!costmap_lock.owns_lock()) {
      return false;
    }
    costmap_snapshot = std::make_shared<nav2_costmap_2d::Costmap2D>(
        costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
        costmap->getResolution(), costmap->getOriginX(), costmap->getOriginY(),
        0U);
    const auto cell_count =
        static_cast<std::size_t>(costmap->getSizeInCellsX()) *
        costmap->getSizeInCellsY();
    std::copy_n(costmap->getCharMap(), cell_count,
                costmap_snapshot->getCharMap());
  }

  const auto request_signature = make_route_revision_signature(
      *costmap_snapshot, captured_generation, captured_route_segment,
      search_start, blocking_evidence);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (captured_generation != plan_generation_ ||
        !replan_progress_.should_attempt_replan(request_signature)) {
      return false;
    }
  }

  ElevatorScopedReplanRequest request;
  request.plan_generation = captured_generation;
  request.request_signature = request_signature;
  request.costmap_frame = costmap_frame;
  request.costmap = std::move(costmap_snapshot);
  request.footprint = footprint;
  request.start = search_start;
  request.goal = search_goal;
  request.parameters = route_revision_parameters_;
  if (!replan_worker_.submit(std::move(request))) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (captured_generation == plan_generation_) {
      last_route_revision_attempt_ = now;
      replan_progress_.observe_replan_attempt(request_signature);
    }
  }
  RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Elevator scoped obstacle route revision scheduled outside the control "
      "loop: generation=%zu segment=%zu budget=%.0fms",
      captured_generation, captured_route_segment + 1U,
      route_revision_parameters_.maximum_search_time_sec * 1000.0);
  return true;
}

bool ElevatorScopedController::apply_route_revision_result() {
  auto completed = replan_worker_.take_result();
  if (!completed) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (completed->plan_generation != plan_generation_) {
      return false;
    }
  }
  const auto &search = completed->search;
  if (!search.succeeded()) {
    RCLCPP_WARN(
        logger_,
        "Elevator scoped obstacle route revision unavailable: search=%s "
        "expanded=%zu block=%s cell=(%u,%u) cost=%u; "
        "remaining in WAIT_CLEAR until fresh evidence permits another attempt",
        elevator_scoped_search_status_name(search.status),
        search.expanded_nodes,
        elevator_scoped_clearance_status_name(search.direct_path_block.status),
        search.direct_path_block.cell_x, search.direct_path_block.cell_y,
        static_cast<unsigned int>(search.direct_path_block.cost));
    return false;
  }
  if (search.path.samples.empty() || completed->costmap_frame.empty()) {
    RCLCPP_WARN(logger_, "Elevator scoped replan returned an empty path");
    return false;
  }
  const auto forget_transient_attempt = [this, &completed]() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (completed->plan_generation == plan_generation_) {
      replan_progress_.forget_replan_attempt(completed->request_signature);
    }
  };
  if (!costmap_ros_ || !costmap_ros_->getCostmap() ||
      costmap_ros_->getGlobalFrameID() != completed->costmap_frame) {
    forget_transient_attempt();
    RCLCPP_WARN(
        logger_,
        "Elevator scoped route revision discarded before commit: live costmap "
        "frame is unavailable or changed; the same evidence may retry");
    return false;
  }
  auto *live_costmap = costmap_ros_->getCostmap();
  ElevatorScopedPathClearanceResult live_validation;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
        *live_costmap->getMutex(), std::try_to_lock);
    if (!costmap_lock.owns_lock()) {
      forget_transient_attempt();
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "Elevator scoped route revision waits for a current costmap "
          "validation; the same evidence may retry");
      return false;
    }
    live_validation = evaluate_elevator_scoped_path_clearance(
        *live_costmap, costmap_ros_->getRobotFootprint(), search.path.samples);
  }
  if (!live_validation.is_clear()) {
    RCLCPP_WARN(
        logger_,
        "Elevator scoped route revision discarded: live swept footprint "
        "changed during search; status=%s sample=%zu cell=(%u,%u) cost=%u",
        elevator_scoped_clearance_status_name(live_validation.clearance.status),
        live_validation.sample_index, live_validation.clearance.cell_x,
        live_validation.clearance.cell_y,
        static_cast<unsigned int>(live_validation.clearance.cost));
    return false;
  }
  std::string active_path_frame;
  geometry_msgs::msg::PoseStamped canonical_final_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (completed->plan_generation != plan_generation_) {
      return false;
    }
    active_path_frame = path_.header.frame_id;
    canonical_final_goal = final_goal_;
  }
  std::string route_frame = canonical_final_goal.header.frame_id;
  if (route_frame.empty()) {
    route_frame = active_path_frame;
  }
  if (route_frame.empty()) {
    return false;
  }

  nav_msgs::msg::Path replanned_path;
  replanned_path.header.frame_id = route_frame;
  replanned_path.header.stamp = clock_->now();
  replanned_path.poses.reserve(search.path.samples.size());
  std::vector<ElevatorScopedPose> route_poses;
  route_poses.reserve(replanned_path.poses.capacity());
  const auto append_stamped_pose = [&replanned_path, &route_poses](
                                       geometry_msgs::msg::PoseStamped pose) {
    pose.header = replanned_path.header;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    if (!std::isfinite(pose.pose.position.x) ||
        !std::isfinite(pose.pose.position.y) || !std::isfinite(yaw)) {
      return false;
    }
    route_poses.push_back({pose.pose.position.x, pose.pose.position.y, yaw});
    replanned_path.poses.push_back(std::move(pose));
    return true;
  };
  const auto transform_to_route_frame =
      [this, &route_frame](geometry_msgs::msg::PoseStamped source,
                           geometry_msgs::msg::PoseStamped &transformed) {
        if (source.header.frame_id == route_frame) {
          transformed = std::move(source);
          return true;
        }
        if (source.header.frame_id.empty()) {
          return false;
        }
        source.header.stamp = builtin_interfaces::msg::Time{};
        return nav2_util::transformPoseInTargetFrame(source, transformed, *tf_,
                                                     route_frame);
      };
  for (const auto &sample : search.path.samples) {
    geometry_msgs::msg::PoseStamped source;
    source.header.frame_id = completed->costmap_frame;
    source.pose.position.x = sample.pose.x;
    source.pose.position.y = sample.pose.y;
    source.pose.orientation.z = std::sin(sample.pose.yaw * 0.5);
    source.pose.orientation.w = std::cos(sample.pose.yaw * 0.5);
    geometry_msgs::msg::PoseStamped transformed;
    if (!transform_to_route_frame(std::move(source), transformed) ||
        !append_stamped_pose(std::move(transformed))) {
      return false;
    }
  }
  if (replanned_path.poses.empty()) {
    return false;
  }
  geometry_msgs::msg::PoseStamped exact_goal = canonical_final_goal;
  geometry_msgs::msg::PoseStamped transformed_exact_goal;
  if (!transform_to_route_frame(std::move(exact_goal),
                                transformed_exact_goal)) {
    return false;
  }
  replanned_path.poses.back().pose = transformed_exact_goal.pose;
  route_poses.back() = {
      transformed_exact_goal.pose.position.x,
      transformed_exact_goal.pose.position.y,
      tf2::getYaw(transformed_exact_goal.pose.orientation),
  };
  auto route_segments = make_elevator_scoped_route_segments(route_poses);
  const auto route_segment_count = route_segments.size();
  bool motion_preserved = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (completed->plan_generation != plan_generation_) {
      return false;
    }
    if (!replan_progress_.try_commit_route_revision(
            completed->request_signature)) {
      return false;
    }
    const ElevatorScopedGoalSignature replacement_signature{
        canonical_final_goal.header.frame_id,
        canonical_final_goal.pose.position.x,
        canonical_final_goal.pose.position.y,
        tf2::getYaw(canonical_final_goal.pose.orientation),
    };
    std::optional<ElevatorScopedGoalSignature> active_signature;
    if (!final_goal_.header.frame_id.empty()) {
      active_signature = ElevatorScopedGoalSignature{
          final_goal_.header.frame_id,
          final_goal_.pose.position.x,
          final_goal_.pose.position.y,
          tf2::getYaw(final_goal_.pose.orientation),
      };
    }
    const auto update = decide_elevator_scoped_plan_update(
        active_signature, route_segments_, route_segment_index_,
        route_execution_active_,
        replacement_signature, route_segments);
    motion_preserved = update.action ==
        ElevatorScopedPlanUpdateAction::kHotSwapPreserveMotion;
    path_ = std::move(replanned_path);
    route_segments_ = std::move(route_segments);
    route_segment_index_ = 0U;
    if (!motion_preserved) {
      controller_.reset();
    }
    blockage_.reset();
  }
  RCLCPP_WARN(
      logger_,
      "Elevator scoped obstacle route revision accepted: map-anchored "
      "segments=%zu detour=%s expanded=%zu length=%.3fm motion_preserved=%s; "
      "the historical route was replaced and later changed evidence may "
      "replace this revision",
      route_segment_count, search.used_detour ? "true" : "false",
      search.expanded_nodes, search.path_length_m,
      motion_preserved ? "true" : "false");
  return true;
}

void ElevatorScopedController::publish_permits(const bool lateral,
                                               const bool reverse) {
  std_msgs::msg::Bool message;
  if (lateral_permit_pub_ && lateral_permit_pub_->is_activated()) {
    message.data = lateral;
    lateral_permit_pub_->publish(message);
  }
  if (reverse_permit_pub_ && reverse_permit_pub_->is_activated()) {
    message.data = reverse;
    reverse_permit_pub_->publish(message);
  }
}

void ElevatorScopedController::publish_progress_state(
    const ElevatorScopedProgressState state) {
  if (progress_state_pub_ && progress_state_pub_->is_activated()) {
    std_msgs::msg::UInt8 message;
    message.data = static_cast<std::uint8_t>(state);
    progress_state_pub_->publish(message);
  }
}

const char *
ElevatorScopedController::phase_name(const TerminalControlPhase phase) {
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

const char *ElevatorScopedController::route_phase_name(
    const ElevatorScopedRoutePhase phase) {
  switch (phase) {
  case ElevatorScopedRoutePhase::kYaw:
    return "yaw";
  case ElevatorScopedRoutePhase::kLateral:
    return "lateral";
  case ElevatorScopedRoutePhase::kForward:
    return "forward";
  case ElevatorScopedRoutePhase::kReverse:
    return "reverse";
  case ElevatorScopedRoutePhase::kPose:
    return "final_pose";
  }
  return "unknown";
}

} // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(robot_nav_config::ElevatorScopedController,
                       nav2_core::Controller)
