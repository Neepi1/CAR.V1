#include "robot_api_server/features/navigation/navigation_feature_module.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"
#include "robot_api_server/features/docking/lifecycle/pre_navigation_undock_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/configuration/navigation_configuration_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_execution_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_executor.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/features/safety/safety_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::navigation
{

namespace
{

void validate_dependencies(const NavigationFeatureModuleDependencies & dependencies)
{
  if (dependencies.maps == nullptr ||
    dependencies.map_runtime_state_store == nullptr ||
    dependencies.mapping == nullptr ||
    dependencies.localization == nullptr ||
    dependencies.floor_switch == nullptr ||
    dependencies.elevator == nullptr ||
    dependencies.safety == nullptr ||
    dependencies.teleop == nullptr ||
    dependencies.dock_contact_interlock == nullptr ||
    dependencies.docking_job_store == nullptr ||
    dependencies.docking_runtime == nullptr ||
    dependencies.pre_navigation_undock == nullptr ||
    dependencies.runtime_mode == nullptr ||
    dependencies.delayed_side_effect_unknown_count == nullptr ||
    !dependencies.api_runtime_running ||
    !dependencies.post_relocalization_settle_json)
  {
    throw std::invalid_argument("NavigationFeatureModule requires every dependency");
  }
}

}  // namespace

class NavigationFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    configuration::NavigationConfiguration configuration,
    NavigationFeatureModuleDependencies dependencies)
  : node_(node), dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    NavigationActionRuntimePorts action_ports;
    action_ports.delayed_side_effect_started = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_add(
          1U, std::memory_order_acq_rel);
      };
    action_ports.delayed_side_effect_resolved = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_sub(
          1U, std::memory_order_acq_rel);
      };
    action_ports.latch_safety_stop = [this]() {
        dependencies_.safety->latch_stop_for_unproven_navigation_terminal();
      };

    NavigationModulePorts module_ports;
    module_ports.action = std::move(action_ports);
    module_ports.pre_goal_dock_snapshot = [this](
      const NavigationPreGoalDockContext & context) {
        return pre_goal_dock_snapshot(context);
      };
    module_ports.floor_runtime_interlock_response = [this](
      const std::string & operation) {
        return dependencies_.floor_switch->interlock_response(operation);
      };
    module_ports.runtime_neighbor_snapshot = [this](const bool refresh_mapping) {
        if (refresh_mapping) {
          (void)dependencies_.mapping->snapshot(true);
        }
        NavigationRuntimeNeighborSnapshot snapshot;
        snapshot.runtime = dependencies_.runtime_mode->snapshot();
        snapshot.mode_transition_owner = dependencies_.runtime_mode->transition_owner();
        snapshot.mapping_start_job_running =
          dependencies_.mapping->snapshot(false).start_job_running;
        {
          std::lock_guard<std::mutex> lock(dependencies_.docking_job_store->mutex());
          snapshot.docking_job_running =
            dependencies_.docking_job_store->job_unsafe().state == "running";
        }
        return snapshot;
      };
    module_ports.set_navigation_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & message,
      const bool healthy) {
        dependencies_.runtime_mode->set_navigation(active, state, message, healthy);
      };
    module_ports.read_runtime_map_context = [this]() {
        return dependencies_.map_runtime_state_store->read_runtime_map_context();
      };
    module_ports.write_runtime_map_context = [this](
      const MapManifest & manifest,
      const std::string & state,
      const bool confirmed,
      const std::string & message) {
        dependencies_.map_runtime_state_store->write_runtime_map_context(
          manifest, state, confirmed, message);
      };
    module_ports.write_last_navigation_map_selection = [this](
      const MapManifest & manifest, const std::string & reason) {
        dependencies_.map_runtime_state_store->write_last_navigation_map_selection(
          manifest, reason);
      };
    module_ports.clear_terminal_speed_limit = [this]() {
        terminal_runtime_->clear_speed_limit();
      };
    module_ports.publish_zero_motion = [this]() {
        dependencies_.teleop->clear_command();
        dependencies_.teleop->publish_zero_burst();
        terminal_runtime_->publish_zero_burst();
      };
    module_ports.safety_snapshot = [this]() {
        return dependencies_.safety->snapshot();
      };
    module_ports.post_relocalization_settle_json = [this]() {
        return dependencies_.post_relocalization_settle_json();
      };
    module_ports.post_undock_settle_json = [this]() {
        std::lock_guard<std::mutex> lock(dependencies_.docking_job_store->mutex());
        return dependencies_.docking_job_store->post_undock_settle_json_locked();
      };
    module_ports.run_goal_job = [this](
      const std::uint64_t job_id,
      const nav2_msgs::action::NavigateToPose::Goal & goal,
      const StoredPose & target,
      const std::string & pose_id,
      const std::string & building_id,
      const std::string & floor_id,
      const NavigationPreGoalDockSnapshot & dock_snapshot) {
        goal_executor_->run_guarded(
          job_id,
          goal,
          target,
          pose_id,
          building_id,
          floor_id,
          dock_snapshot);
      };

    module_ = std::make_unique<NavigationModule>(
      node_,
      std::move(callback_group),
      dependencies_.maps->global_costmap_lifecycle_client(),
      *dependencies_.maps,
      *dependencies_.localization,
      *dependencies_.elevator,
      std::move(configuration.module),
      std::move(module_ports));

    NavigationTerminalRuntimePorts terminal_ports;
    terminal_ports.running = [this]() {
        return dependencies_.api_runtime_running();
      };
    terminal_ports.cancel_requested = [this](
      const std::uint64_t job_id, std::string & detail) {
        return goal_execution_->navigation_goal_cancel_requested(job_id, detail);
      };
    terminal_ports.safety_hard_blocked = [this](std::string & detail) {
        return dependencies_.safety->hard_blocked(detail);
      };
    terminal_ports.verify_final_pose = [this](
      const StoredPose & target, const bool require_fresh_pose) {
        return goal_execution_->verify_navigation_final_pose(target, require_fresh_pose);
      };
    terminal_ports.update_final_pose = [this](
      const std::uint64_t job_id,
      const FinalPoseCheck & check,
      const std::string & reason) {
        goal_execution_->update_navigation_goal_final_pose_fields(job_id, check, reason);
      };
    terminal_ports.set_job_phase = [this](
      const std::uint64_t job_id,
      const std::string & phase,
      const std::string & detail) {
        (void)module_->mission_runtime().update(
          job_id,
          [&phase, &detail](NavigationGoalJob & job) {
            job.phase = phase;
            job.detail = detail;
          });
      };
    terminal_ports.publish_motion_mode = [this](const std::string & mode) {
        dependencies_.docking_runtime->publish_forced_mode(mode);
      };
    terminal_ports.current_robot_pose = [this]() {
        return dependencies_.localization->current_robot_pose_snapshot();
      };
    terminal_ports.pose_in_frame = [this](const std::string & frame_id) {
        TerminalFramePoseSnapshot result;
        const auto pose = dependencies_.localization->pose_in_frame(frame_id);
        result.available = pose.available;
        result.x = pose.x;
        result.y = pose.y;
        result.yaw = pose.yaw;
        result.received_at = pose.received_at;
        return result;
      };
    terminal_ports.dock_contact_blocked = [this](std::string & detail) {
        const auto dock_check = dependencies_.dock_contact_interlock->snapshot();
        detail = dock_check.auto_undock_reason;
        return dock_check.final_auto_undock_required;
      };
    terminal_runtime_ = std::make_unique<NavigationTerminalRuntimeModule>(
      node_,
      module_->terminal_control(),
      std::move(configuration.terminal_runtime),
      std::move(terminal_ports));

    NavigationGoalExecutionPorts execution_ports;
    execution_ports.floor_runtime_operation_blocked = [this](
      const std::string & operation,
      std::string & detail,
      std::string * reason_code) {
        return dependencies_.floor_switch->operation_blocked(
          operation, detail, reason_code);
      };
    execution_ports.undock_before_navigation = [this](
      const NavigationPreGoalDockSnapshot & dock_check,
      std::string & detail,
      bool & undock_performed) {
        return undock_before_navigation(dock_check, detail, undock_performed);
      };
    execution_ports.bridge_safe_for_goal_start = [this](
      const std::string & context, std::string & detail) {
        return bridge_safe_for_goal_start(context, detail);
      };
    execution_ports.bridge_readiness_snapshot = [this]() {
        return bridge_readiness_snapshot(
          dependencies_.localization->bridge_status_snapshot());
      };
    execution_ports.request_amcl_nomotion_update = [this](
      const std::string & context,
      std::string & detail,
      const std::chrono::nanoseconds timeout) {
        return dependencies_.localization->request_amcl_nomotion_update(
          context, detail, timeout);
      };
    execution_ports.request_bridge_correction_pause = [this](
      const bool paused,
      std::string & detail,
      const std::chrono::nanoseconds timeout,
      const bool enabled) {
        return dependencies_.localization->request_bridge_correction_pause(
          paused, detail, timeout, enabled);
      };
    execution_ports.robot_pose_snapshot = [this](
      const bool require_fresh, std::string & error) {
        if (require_fresh) {
          return dependencies_.localization->wait_for_current_robot_pose(true, error);
        }
        error.clear();
        return dependencies_.localization->current_robot_pose_snapshot();
      };
    execution_ports.safety_hard_blocked = [this](std::string & detail) {
        return dependencies_.safety->hard_blocked(detail);
      };
    execution_ports.docking_job_running = [this]() {
        std::lock_guard<std::mutex> lock(dependencies_.docking_job_store->mutex());
        return dependencies_.docking_job_store->job_unsafe().state == "running";
      };
    execution_ports.dock_contact_blocked = [this](std::string & detail) {
        const auto check = dependencies_.dock_contact_interlock->snapshot();
        detail = check.auto_undock_reason;
        return check.final_auto_undock_required;
      };
    execution_ports.cancel_active_goal = [this](std::string & detail) {
        return module_->cancel_active_goal(detail);
      };
    execution_ports.cancel_goal_for_handoff = [this](
      const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
      std::string & detail) {
        return module_->cancel_goal_for_api_handoff(goal_handle, detail);
      };
    execution_ports.request_goal_cancel = [this](const std::string & reason) {
        return module_->request_goal_cancel(reason);
      };
    execution_ports.navigation_runtime_active = [this]() {
        return dependencies_.runtime_mode->snapshot().navigation_active;
      };
    execution_ports.set_navigation_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & message,
      const bool healthy) {
        dependencies_.runtime_mode->set_navigation(active, state, message, healthy);
      };
    execution_ports.timestamp_now = []() {return utc_timestamp_iso8601();};
    execution_ports.goal_stamp = [this]() {return node_.now();};
    execution_ports.delayed_side_effect_started = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_add(
          1U, std::memory_order_acq_rel);
      };
    execution_ports.delayed_side_effect_resolved = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_sub(
          1U, std::memory_order_acq_rel);
      };
    execution_ports.warn = [this](const std::string & warning) {
        RCLCPP_WARN(node_.get_logger(), "%s", warning.c_str());
      };

    goal_execution_ = std::make_unique<NavigationGoalExecutionModule>(
      std::move(configuration.goal_execution),
      module_->action_runtime(),
      module_->mission_runtime(),
      module_->completion_policy(),
      module_->bridge_wait(),
      module_->terminal_control(),
      *terminal_runtime_,
      std::move(execution_ports));

    goal_executor_ = std::make_unique<NavigationGoalExecutor>(
      std::move(configuration.goal_executor),
      module_->action_runtime(),
      module_->mission_runtime(),
      *goal_execution_);
  }

  ~Impl()
  {
    shutdown();
  }

  NavigationPreGoalDockSnapshot pre_goal_dock_snapshot(
    const NavigationPreGoalDockContext & context) const
  {
    const auto check = dependencies_.dock_contact_interlock->snapshot();
    const features::docking::PreNavigationDockCheckContext check_context{
      context.operation,
      context.pose_id,
      context.building_id,
      context.floor_id,
      context.frame_id,
      context.has_direct_pose_query};
    NavigationPreGoalDockSnapshot snapshot;
    snapshot.auto_undock_required = check.final_auto_undock_required;
    snapshot.pre_navigation_recovery_required = check.pre_navigation_recovery_required;
    snapshot.pre_navigation_blocked = check.safety_interlock_state_block;
    snapshot.clear_stale_safety_interlock_required =
      check.clear_stale_safety_interlock_required;
    snapshot.can_auto_undock = check.can_auto_undock;
    snapshot.docking_active_not_docked_block = check.docking_active_not_docked_block;
    snapshot.runtime_state_undocking = check.runtime_state_undocking;
    snapshot.docking_status_indicates_undocking = check.docking_status_indicates_undocking;
    snapshot.bms_contact = check.bms.contact;
    snapshot.live_bms_charging_contact_stable = check.live_bms_charging_contact_stable;
    snapshot.dock_latch_indicates_docked = check.dock_latch_indicates_docked;
    snapshot.dock_contact_latch_stale = check.dock_contact_latch_stale;
    snapshot.dock_contact_latch_contradicted_by_live_state =
      check.dock_contact_latch_contradicted_by_live_state;
    snapshot.latch_valid_for_auto_undock = check.latch_valid_for_auto_undock;
    snapshot.strong_live_docked = check.strong_live_docked;
    snapshot.dock_contact_latch_age_sec = check.dock_contact_latch_age_sec;
    snapshot.dock_contact_latch_source = check.dock_contact_latch_source;
    snapshot.auto_undock_reason = check.auto_undock_reason;
    snapshot.pre_navigation_recovery_action = check.pre_navigation_recovery_action;
    snapshot.pre_navigation_block_reason = check.pre_navigation_block_reason;
    snapshot.resolved_dock_id = check.resolved_dock_id;
    snapshot.dock_zone_state = check.dock_zone.state;
    snapshot.dock_zone_reason = check.dock_zone.reason;
    snapshot.dock_zone_distance_m = check.dock_zone.distance_m;
    snapshot.dock_occupancy_state = check.dock_occupancy_state;
    snapshot.dock_occupancy_evidence = check.dock_occupancy_evidence;
    snapshot.dock_occupancy_reason = check.dock_occupancy_reason;
    snapshot.charging_session_latched = check.charging_session_latched;
    snapshot.charging_session_age_sec = check.charging_session_age_sec;
    snapshot.charging_session_last_confirmed_at =
      check.charging_session_last_confirmed_at;
    snapshot.full_charge_idle_on_dock = check.full_charge_idle_on_dock;
    snapshot.runtime_docking_state = check.runtime.docking_state;
    snapshot.runtime_docking_status = check.runtime.docking_status;
    snapshot.json = dependencies_.dock_contact_interlock->pre_navigation_check_json(
      check, check_context);
    snapshot.render_json = [this, check](
      const NavigationPreGoalDockContext & render_context) {
        return dependencies_.dock_contact_interlock->pre_navigation_check_json(
          check,
          features::docking::PreNavigationDockCheckContext{
          render_context.operation,
          render_context.pose_id,
          render_context.building_id,
          render_context.floor_id,
          render_context.frame_id,
          render_context.has_direct_pose_query});
      };
    return snapshot;
  }

  bool undock_before_navigation(
    const NavigationPreGoalDockSnapshot & dock_check,
    std::string & detail,
    bool & undock_performed) const
  {
    features::docking::PreNavigationUndockRequest request;
    request.auto_undock_required = dock_check.auto_undock_required;
    request.pre_navigation_blocked = dock_check.pre_navigation_blocked;
    request.docking_active_not_docked_block =
      dock_check.docking_active_not_docked_block;
    request.runtime_state_undocking = dock_check.runtime_state_undocking;
    request.docking_status_indicates_undocking =
      dock_check.docking_status_indicates_undocking;
    request.charging_contact_at_gate = dock_check.bms_contact;
    request.runtime_docking_state = dock_check.runtime_docking_state;
    request.auto_undock_reason = dock_check.auto_undock_reason;
    request.recovery_action = dock_check.pre_navigation_recovery_action;
    request.pre_navigation_block_reason = dock_check.pre_navigation_block_reason;
    request.resolved_dock_id = dock_check.resolved_dock_id;
    request.reconcile_evidence =
      "dock_zone=" + dock_check.dock_zone_state +
      " distance_m=" + std::to_string(dock_check.dock_zone_distance_m) +
      " reason=" + dock_check.dock_zone_reason;
    return dependencies_.pre_navigation_undock->run_if_needed(
      request, detail, undock_performed);
  }

  BridgeReadinessSnapshot bridge_readiness_snapshot(
    const features::localization::BridgeStatusSnapshot & bridge) const
  {
    BridgeReadinessSnapshot snapshot;
    snapshot.available = bridge.available;
    snapshot.has_map_to_odom = bridge.has_map_to_odom;
    snapshot.map_to_odom_publisher_owner = bridge.map_to_odom_publisher_owner;
    snapshot.map_odom_correction_paused = bridge.map_odom_correction_paused;
    snapshot.correction_pause_reason = bridge.correction_pause_reason;
    snapshot.map_odom_frozen_due_to_pause = bridge.map_odom_frozen_due_to_pause;
    snapshot.correction_active = bridge.correction_active;
    snapshot.safe_for_goal_start = bridge.safe_for_goal_start;
    snapshot.current_sequence = bridge.current_sequence;
    snapshot.target_sequence = bridge.target_sequence;
    snapshot.remaining_translation_error_m = bridge.remaining_translation_error_m;
    snapshot.remaining_yaw_error_rad = bridge.remaining_yaw_error_rad;
    snapshot.amcl_input_enabled = bridge.amcl_input_enabled;
    snapshot.amcl_degraded_reason = bridge.amcl_degraded_reason;
    snapshot.amcl_status_source = bridge.amcl_status_source;
    snapshot.amcl_status_age_ms = bridge.amcl_status_age_ms;
    snapshot.amcl_process_ready = bridge.amcl_process_ready;
    snapshot.amcl_seeded = bridge.amcl_seeded;
    snapshot.amcl_nomotion_pose_received = bridge.amcl_nomotion_pose_received;
    snapshot.amcl_static_standby = bridge.amcl_static_standby;
    snapshot.amcl_tracking_ready = bridge.amcl_tracking_ready;
    snapshot.amcl_correction_ready = bridge.amcl_correction_ready;
    snapshot.amcl_correction_pending = bridge.amcl_correction_pending;
    snapshot.amcl_not_moving_no_update_ok = bridge.amcl_not_moving_no_update_ok;
    snapshot.localization_degraded = bridge.localization_degraded;
    return snapshot;
  }

  bool bridge_status_safe_for_goal_start(
    const features::localization::BridgeStatusSnapshot & bridge,
    const std::string & context,
    std::string & detail) const
  {
    const auto decision = evaluate_bridge_readiness(
      bridge_readiness_snapshot(bridge), BridgeReadinessPurpose::kGoalStart, context);
    detail = decision.detail;
    return decision.safe;
  }

  bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const
  {
    return bridge_status_safe_for_goal_start(
      dependencies_.localization->bridge_status_snapshot(), context, detail);
  }

  void shutdown()
  {
    if (shutdown_) {
      return;
    }
    shutdown_ = true;
    if (module_) {
      module_->mission_runtime().shutdown();
      module_->join_cancel_worker();
    }
  }

  rclcpp::Node & node_;
  NavigationFeatureModuleDependencies dependencies_;
  std::unique_ptr<NavigationModule> module_;
  std::unique_ptr<NavigationTerminalRuntimeModule> terminal_runtime_;
  std::unique_ptr<NavigationGoalExecutionModule> goal_execution_;
  std::unique_ptr<NavigationGoalExecutor> goal_executor_;
  bool shutdown_{false};
};

NavigationFeatureModule::NavigationFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  configuration::NavigationConfiguration configuration,
  NavigationFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      std::move(dependencies)))
{
}

NavigationFeatureModule::~NavigationFeatureModule() = default;

NavigationModule & NavigationFeatureModule::module() {return *impl_->module_;}

NavigationActionRuntime & NavigationFeatureModule::action_runtime()
{
  return impl_->module_->action_runtime();
}

NavigationMissionRuntime & NavigationFeatureModule::mission_runtime()
{
  return impl_->module_->mission_runtime();
}

NavigationCompletionPolicy & NavigationFeatureModule::completion_policy()
{
  return impl_->module_->completion_policy();
}

NavigationTerminalControl & NavigationFeatureModule::terminal_control()
{
  return impl_->module_->terminal_control();
}

NavigationTerminalRuntimeModule & NavigationFeatureModule::terminal_runtime()
{
  return *impl_->terminal_runtime_;
}

NavigationGoalExecutionModule & NavigationFeatureModule::goal_execution()
{
  return *impl_->goal_execution_;
}

BridgeReadinessSnapshot NavigationFeatureModule::bridge_readiness_snapshot(
  const features::localization::BridgeStatusSnapshot & bridge) const
{
  return impl_->bridge_readiness_snapshot(bridge);
}

bool NavigationFeatureModule::bridge_status_safe_for_goal_start(
  const features::localization::BridgeStatusSnapshot & bridge,
  const std::string & context,
  std::string & detail) const
{
  return impl_->bridge_status_safe_for_goal_start(bridge, context, detail);
}

bool NavigationFeatureModule::bridge_safe_for_goal_start(
  const std::string & context,
  std::string & detail) const
{
  return impl_->bridge_safe_for_goal_start(context, detail);
}

void NavigationFeatureModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::navigation
