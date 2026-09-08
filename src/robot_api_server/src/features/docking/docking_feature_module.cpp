#include "robot_api_server/features/docking/docking_feature_module.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_interfaces/msg/dock_safety_interlock_state.hpp"
#include "robot_interfaces/srv/reconcile_dock_interlock.hpp"

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/configuration/docking_configuration_module.hpp"
#include "robot_api_server/features/docking/configuration/docking_predock_pose_resolver.hpp"
#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_correction_pause_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_http_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_execution_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_executor.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_status_module.hpp"
#include "robot_api_server/features/docking/lifecycle/pre_navigation_undock_module.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_alignment_policy.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_control_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/floor_asset_resolver.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/catalog_activation/runtime_map_lookup.hpp"
#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_execution_module.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/features/power/power_module.hpp"
#include "robot_api_server/features/safety/safety_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"
#include "robot_api_server/infrastructure/process/deferred_work_queue.hpp"

namespace fs = std::filesystem;

namespace robot_api_server::features::docking
{

namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

void validate_dependencies(const DockingFeatureModuleDependencies & dependencies)
{
  if (dependencies.maps == nullptr ||
    dependencies.map_runtime_state_store == nullptr ||
    dependencies.mapping == nullptr ||
    dependencies.localization == nullptr ||
    dependencies.floor_switch == nullptr ||
    dependencies.safety == nullptr ||
    dependencies.teleop == nullptr ||
    dependencies.power == nullptr ||
    dependencies.runtime_mode == nullptr ||
    dependencies.delayed_side_effect_unknown_count == nullptr ||
    !dependencies.navigation_goal_running)
  {
    throw std::invalid_argument("DockingFeatureModule requires every core dependency");
  }
}

void validate_late_dependencies(const DockingFeatureLateDependencies & dependencies)
{
  if (dependencies.elevator == nullptr ||
    dependencies.navigation == nullptr ||
    dependencies.post_relocalization_settle == nullptr)
  {
    throw std::invalid_argument("DockingFeatureModule requires every late dependency");
  }
}

}  // namespace

class DockingFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    configuration::DockingConfiguration configuration,
    DockingFeatureModuleDependencies dependencies)
  : node_(node),
    callback_group_(std::move(callback_group)),
    configuration_(std::move(configuration)),
    dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    predock_alignment_policy_ =
      std::make_unique<predock_alignment::PredockAlignmentPolicy>(
      std::move(configuration_.alignment_policy));

    rclcpp::SubscriptionOptions safety_subscription_options;
    safety_subscription_options.callback_group = callback_group_;
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    dock_safety_interlock_state_sub_ =
      node_.create_subscription<robot_interfaces::msg::DockSafetyInterlockState>(
      configuration_.contact_interlock.safety_interlock_state_topic,
      state_qos,
      [this](const robot_interfaces::msg::DockSafetyInterlockState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(dock_safety_interlock_mutex_);
        dock_safety_interlock_message_ = *message;
        dock_safety_interlock_received_at_ = std::chrono::steady_clock::now();
        have_dock_safety_interlock_message_ = true;
      },
      safety_subscription_options);
    dock_safety_interlock_reconcile_client_ =
      node_.create_client<robot_interfaces::srv::ReconcileDockInterlock>(
      configuration_.contact_interlock.safety_interlock_reconcile_service,
      rmw_qos_profile_services_default,
      callback_group_);

    DockContactInterlockPorts contact_ports;
    contact_ports.runtime_snapshot = [this]() {
        return dependencies_.runtime_mode->snapshot();
      };
    contact_ports.bms_snapshot = [this]() {
        return dependencies_.power->snapshot();
      };
    contact_ports.safety_interlock_snapshot = [this]() {
        return dock_safety_interlock_snapshot();
      };
    contact_ports.dock_zone_snapshot = [this](
      const DockContactLatchSnapshot & latch,
      const DockSafetyInterlockSnapshot & safety,
      const application::runtime_mode::RuntimeModeSnapshot & runtime) {
        return evaluate_dock_zone(latch, safety, runtime);
      };
    contact_ports.navigation_goal_running = [this]() {
        return dependencies_.navigation_goal_running();
      };
    contact_ports.wall_time_seconds = []() {
        return robot_api_server::wall_time_seconds();
      };
    contact_ports.timestamp_now = []() {
        return utc_timestamp_iso8601();
      };
    contact_ports.warn = [this](const std::string & message) {
        RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
      };
    contact_interlock_ = std::make_unique<DockContactInterlockModule>(
      std::move(configuration_.contact_interlock), std::move(contact_ports));

    DockingJobStorePorts store_ports;
    store_ports.update_dock_contact_latch = [this](
      const bool docked,
      const std::string & source,
      const std::string & reason,
      const std::string & dock_id) {
        contact_interlock_->update_latch(docked, source, reason, dock_id);
      };
    store_ports.finish_runtime = [this](
      const std::string & final_state, const std::string & detail) {
        dependencies_.runtime_mode->finish_docking(final_state, detail);
      };
    store_ports.timestamp_now = []() {
        return utc_timestamp_iso8601();
      };
    store_ports.set_global_correction_paused = [this](
      const std::uint64_t job_id,
      const bool paused,
      const std::string & reason,
      std::string & detail) {
        if (!correction_pause_) {
          detail = "docking correction pause module is not initialized";
          return false;
        }
        return correction_pause_->set_paused(job_id, paused, reason, detail);
      };
    job_store_ = std::make_unique<DockingJobStore>(std::move(store_ports));

    DockingCorrectionPausePorts correction_ports;
    correction_ports.request_correction_pause = [this](
      const bool paused,
      std::string & detail,
      const std::chrono::nanoseconds timeout,
      const bool enabled) {
        return dependencies_.localization->request_bridge_correction_pause(
          paused, detail, timeout, enabled);
      };
    correction_ports.bridge_status_snapshot = [this]() {
        return dependencies_.localization->bridge_status_snapshot();
      };
    correction_ports.current_robot_pose = [this]() {
        return dependencies_.localization->current_robot_pose_snapshot();
      };
    correction_ports.warn = [this](const std::string & message) {
        RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
      };
    correction_ports.error = [this](const std::string & message) {
        RCLCPP_ERROR(node_.get_logger(), "%s", message.c_str());
      };
    correction_pause_ = std::make_unique<DockingCorrectionPauseModule>(
      std::move(configuration_.correction_pause),
      *job_store_,
      std::move(correction_ports));

    DockingRuntimePorts runtime_ports;
    runtime_ports.on_status = [this](const std::string & status) {
        if (status_) {
          status_->handle_status(status);
        }
      };
    runtime_ports.run_job = [this](const std::uint64_t job_id) {
        if (job_executor_) {
          job_executor_->run_guarded(job_id);
        }
      };
    runtime_ports.delayed_side_effect_started = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_add(
          1U, std::memory_order_acq_rel);
      };
    runtime_ports.delayed_side_effect_resolved = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_sub(
          1U, std::memory_order_acq_rel);
      };
    runtime_ports.on_undock_retry_wait = [this](const std::string & detail) {
        dependencies_.runtime_mode->set_docking(true, "undocking", detail, true);
      };
    runtime_ = std::make_unique<DockingRuntimeModule>(
      node_,
      callback_group_,
      std::move(configuration_.runtime),
      std::move(runtime_ports));

    PreNavigationUndockPorts undock_ports;
    undock_ports.clear_teleop_command = [this]() {
        dependencies_.teleop->clear_command();
      };
    undock_ports.publish_zero_motion = [this]() {
        dependencies_.teleop->publish_zero_burst();
      };
    undock_ports.prepare_controlled_undock = [this](std::string & detail) {
        return prepare_controlled_undock(detail);
      };
    undock_ports.release_stale_fine_pause = [this](
      const std::string & context, std::string & detail) {
        return correction_pause_->release_stale_if_needed(context, detail);
      };
    undock_ports.join_docking_worker = [this]() {
        runtime_->join_worker();
      };
    undock_ports.runtime_snapshot = [this]() {
        return dependencies_.runtime_mode->snapshot();
      };
    undock_ports.reconcile_stale_interlock = [this](
      const std::string & dock_id,
      const std::string & evidence,
      std::string & detail) {
        return reconcile_stale_dock_interlock(dock_id, evidence, detail);
      };
    undock_ports.ensure_manager_running = [this](std::string & detail) {
        return runtime_->ensure_manager_running(detail);
      };
    undock_ports.call_undock_with_charging_retry = [this](
      std::string & detail,
      const bool allow_charging_retry,
      PreNavigationUndockServiceObservation * observation) {
        DockingUndockServiceObservation runtime_observation;
        const bool ok = runtime_->call_undock_with_charging_retry(
          detail, allow_charging_retry, &runtime_observation);
        if (observation != nullptr) {
          observation->service_called = runtime_observation.service_called;
          observation->service_success = runtime_observation.service_success;
          observation->message = runtime_observation.message;
        }
        return ok;
      };
    undock_ports.observe_undock_status = [this](
      DockingJob & job, const std::string & status) {
        runtime_->observe_undock_status(job, status);
      };
    undock_ports.set_docking_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & detail) {
        dependencies_.runtime_mode->set_docking(active, state, detail, true);
      };
    undock_ports.set_docking_identity = [this](const std::string & dock_id) {
        dependencies_.runtime_mode->set_docking_identity(dock_id);
      };
    undock_ports.timestamp_now = []() {
        return utc_timestamp_iso8601();
      };
    undock_ports.monotonic_now = []() {
        return std::chrono::steady_clock::now();
      };
    undock_ports.sleep_for = [](const std::chrono::milliseconds duration) {
        std::this_thread::sleep_for(duration);
      };
    pre_navigation_undock_ = std::make_unique<PreNavigationUndockModule>(
      docking_start_mutex_,
      *job_store_,
      std::move(configuration_.pre_navigation_undock),
      std::move(undock_ports));
  }

  ~Impl()
  {
    shutdown();
  }

  DockSafetyInterlockSnapshot dock_safety_interlock_snapshot()
  {
    std::lock_guard<std::mutex> lock(dock_safety_interlock_mutex_);
    DockSafetyInterlockSnapshot snapshot;
    if (!have_dock_safety_interlock_message_) {
      return snapshot;
    }
    const auto & message = dock_safety_interlock_message_;
    snapshot.available = true;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - dock_safety_interlock_received_at_).count();
    snapshot.fresh = snapshot.age_sec >= 0.0 &&
      snapshot.age_sec <= configuration_.contact_interlock.safety_interlock_state_max_age_sec;
    snapshot.enabled = message.enabled;
    snapshot.memory_latched = message.memory_latched;
    snapshot.active = message.active;
    snapshot.battery_sample_fresh = message.battery_sample_fresh;
    snapshot.live_bms_contact = message.live_bms_contact;
    snapshot.no_contact_duration_sec = message.no_contact_duration_sec;
    snapshot.persistent_dock_latched = message.persistent_dock_latched;
    snapshot.persistent_dock_strong = message.persistent_dock_strong;
    snapshot.reverse_session_seen = message.reverse_session_seen;
    snapshot.reverse_permit_active = message.reverse_permit_active;
    snapshot.dock_id = message.dock_id;
    snapshot.building_id = message.building_id;
    snapshot.floor_id = message.floor_id;
    snapshot.map_id = message.map_id;
    snapshot.state = message.state;
    snapshot.reason = message.reason;
    return snapshot;
  }

  DockZoneSnapshot evaluate_dock_zone(
    const DockContactLatchSnapshot & latch,
    const DockSafetyInterlockSnapshot & safety,
    const application::runtime_mode::RuntimeModeSnapshot & runtime) const
  {
    DockZoneSnapshot zone;
    zone.dock_id = !runtime.docking_dock_id.empty() ? runtime.docking_dock_id :
      (!latch.dock_id.empty() ? latch.dock_id : safety.dock_id);
    if (zone.dock_id.empty() || zone.dock_id == "none") {
      zone.reason = "dock_identity_unavailable";
      return zone;
    }

    const auto context = dependencies_.map_runtime_state_store->read_runtime_map_context();
    if (!context || !context->confirmed) {
      zone.reason = "runtime_map_context_unconfirmed";
      return zone;
    }
    zone.building_id = context->building_id;
    zone.floor_id = context->floor_id;
    zone.map_id = context->map_id;
    const std::string expected_building = !latch.building_id.empty() ?
      latch.building_id : safety.building_id;
    const std::string expected_floor = !latch.floor_id.empty() ?
      latch.floor_id : safety.floor_id;
    const std::string expected_map = !latch.map_id.empty() ? latch.map_id : safety.map_id;
    if ((!expected_building.empty() && expected_building != context->building_id) ||
      (!expected_floor.empty() && expected_floor != context->floor_id) ||
      (!expected_map.empty() && expected_map != context->map_id))
    {
      zone.reason = "dock_and_runtime_map_identity_mismatch";
      return zone;
    }

    const auto dock_pose = find_floor_catalog_pose(
      dependencies_.maps->catalog(), context->building_id, context->floor_id, zone.dock_id);
    if (!dock_pose) {
      zone.reason = "dock_pose_not_found_in_active_floor";
      return zone;
    }
    const auto robot_pose = dependencies_.localization->current_robot_pose_snapshot();
    if (!robot_pose.available || robot_pose.frame_id != "map" ||
      robot_pose.age_sec < 0.0 ||
      robot_pose.age_sec > configuration_.contact_interlock.dock_zone_pose_max_age_sec)
    {
      zone.reason = "fresh_map_frame_robot_pose_unavailable";
      return zone;
    }

    zone.distance_m = std::hypot(
      robot_pose.x - dock_pose->x, robot_pose.y - dock_pose->y);
    if (zone.distance_m <= configuration_.contact_interlock.dock_zone_near_radius_m) {
      zone.state = "NEAR";
      zone.reason = "distance_at_or_below_near_radius";
    } else if (zone.distance_m >= configuration_.contact_interlock.dock_zone_clear_radius_m) {
      zone.state = "CLEAR";
      zone.reason = "distance_at_or_above_clear_radius";
    } else {
      zone.reason = "distance_inside_hysteresis_band";
    }
    return zone;
  }

  bool reconcile_stale_dock_interlock(
    const std::string & dock_id,
    const std::string & evidence,
    std::string & detail)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        configuration_.contact_interlock.safety_interlock_reconcile_timeout_sec));
    if (!dock_safety_interlock_reconcile_client_->wait_for_service(timeout)) {
      detail = "dock safety interlock reconcile service unavailable";
      return false;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::ReconcileDockInterlock::Request>();
    request->transaction_id = "pre-navigation-" + utc_timestamp_iso8601();
    request->dock_id = dock_id;
    request->outside_dock_zone_proven = true;
    request->evidence = evidence;
    auto future = dock_safety_interlock_reconcile_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = "timed out reconciling stale dock safety interlock";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      detail = response->message;
      return false;
    }

    const auto old_latch = contact_interlock_->read_latch();
    contact_interlock_->update_latch(
      false,
      "pre_navigation_reconcile",
      "remote_undock_proven_outside_dock_zone",
      dock_id,
      old_latch.building_id,
      old_latch.floor_id,
      old_latch.map_id,
      evidence);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto state = dock_safety_interlock_snapshot();
      if (state.available && state.fresh && !state.memory_latched && !state.active) {
        detail = "stale dock safety interlock cleared after proven remote undock";
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = "dock safety interlock remained active after reconciliation";
    return false;
  }

  // Retire one stale/conflicting docking owner before an undock transaction.
  // Both manual undock and navigation preflight call this while holding the
  // shared docking_start_mutex_, so no new docking job can race the takeover.
  bool prepare_controlled_undock(std::string & detail)
  {
    const auto active_job = job_store_->snapshot();
    if (active_job.state != "running" || active_job.phase == "undocking") {
      detail = active_job.phase == "undocking" ?
        "undocking already active" : "no conflicting docking owner";
      return true;
    }

    {
      std::lock_guard<std::mutex> lock(job_store_->mutex());
      auto & current = job_store_->job_unsafe();
      if (current.id != active_job.id || current.state != "running") {
        detail = "conflicting docking owner already reached a terminal state";
        return true;
      }
      current.cancel_requested = true;
      current.detail = "superseded_by_controlled_undock";
    }

    dependencies_.teleop->clear_command();
    dependencies_.teleop->publish_zero_burst();

    std::string navigation_detail{"no active navigation cancellation required"};
    if (complete_ && late_dependencies_.navigation != nullptr) {
      (void)late_dependencies_.navigation->module().cancel_active_goal(navigation_detail);
    }

    std::string stop_detail;
    const bool manager_stop_confirmed = runtime_->stop_if_available(stop_detail);

    dependencies_.teleop->publish_zero_burst();

    // Cancellation is visible to the worker before joining, so a worker still
    // waiting on Nav2 exits through its normal cancellation path.
    runtime_->join_worker();
    const auto after_join = job_store_->snapshot();
    if (after_join.id == active_job.id && after_join.state == "running") {
      job_store_->finish(
        active_job.id,
        true,
        "canceled",
        "superseded_by_controlled_undock; " + navigation_detail + "; " +
        stop_detail);
    }

    std::string pause_detail;
    const bool pause_released = correction_pause_->release_stale_if_needed(
      "controlled_undock_takeover", pause_detail);

    detail = "retired docking job id=" + std::to_string(active_job.id) +
      " phase=" + active_job.phase + "; " + navigation_detail + "; " +
      stop_detail + "; " + pause_detail;
    if (!manager_stop_confirmed || !pause_released) {
      detail = "active docking owner terminalized but stop proof is incomplete; " + detail;
      return false;
    }
    return true;
  }

  void complete(DockingFeatureLateDependencies dependencies)
  {
    validate_late_dependencies(dependencies);
    if (complete_) {
      throw std::logic_error("DockingFeatureModule is already complete");
    }
    late_dependencies_ = dependencies;

    predock_alignment::PredockControlPorts predock_ports;
    predock_ports.wait_for_current_pose = [this](
      const bool require_fresh, std::string & error) {
        return dependencies_.localization->wait_for_current_robot_pose(
          require_fresh, error);
      };
    predock_ports.current_pose = [this]() {
        return dependencies_.localization->current_robot_pose_snapshot();
      };
    predock_ports.safety_motion_hard_blocked = [this](std::string & detail) {
        return dependencies_.safety->hard_blocked(detail);
      };
    predock_ports.try_acquire_motion_owner = [this](const bool lateral) {
        return late_dependencies_.navigation->goal_execution().
               try_acquire_predock_motion_owner(lateral);
      };
    predock_ports.release_motion_owner = [this]() {
        late_dependencies_.navigation->goal_execution().release_predock_motion_owner();
      };
    predock_ports.request_navigation_goal_cancel = [this](const std::string & reason) {
        (void)late_dependencies_.navigation->goal_execution().request_goal_cancel(reason);
      };
    predock_ports.publish_final_yaw_zero_burst = [this]() {
        late_dependencies_.navigation->terminal_runtime().publish_zero_burst();
      };
    predock_ports.publish_command = [this](const geometry_msgs::msg::Twist & command) {
        runtime_->publish_command(command);
      };
    predock_ports.publish_forced_mode = [this](const std::string & mode) {
        runtime_->publish_forced_mode(mode);
      };
    predock_ports.clear_teleop_command = [this]() {
        dependencies_.teleop->clear_command();
      };
    predock_ports.publish_teleop_zero = [this]() {
        dependencies_.teleop->publish_zero();
      };
    predock_ports.motion_mode_snapshot = [this]() {
        const auto status_snapshot =
          late_dependencies_.navigation->terminal_runtime().
          mode_controller_status_snapshot();
        return predock_alignment::PredockMotionModeSnapshot{
        status_snapshot.available,
        status_snapshot.actual_available,
        status_snapshot.actual_fresh,
        status_snapshot.actual_motion_mode_code,
        status_snapshot.age_sec};
      };
    predock_ports.reset_actual_stop_stability = [this]() {
        late_dependencies_.navigation->terminal_runtime().
        reset_yaw_actual_stop_stability();
      };
    predock_ports.wait_for_actual_stop = [this](
      const std::string & context, std::string & detail) {
        return late_dependencies_.navigation->terminal_runtime().
               wait_for_yaw_actual_stop(context, detail);
      };
    predock_ports.yaw_stop_threshold_rad = [this](
      const double command_speed, const double tolerance) {
        return late_dependencies_.navigation->terminal_control().yaw_stop_threshold(
          command_speed, tolerance);
      };
    predock_ports.bridge_status_snapshot = [this]() {
        return dependencies_.localization->bridge_status_snapshot();
      };
    predock_ports.bridge_safe_for_goal_start = [this](
      const localization::BridgeStatusSnapshot & bridge,
      const std::string & context,
      std::string & detail) {
        return late_dependencies_.navigation->bridge_status_safe_for_goal_start(
          bridge, context, detail);
      };
    predock_ports.set_global_correction_paused = [this](
      const std::uint64_t job_id,
      const bool paused,
      const std::string & reason,
      std::string & detail) {
        return correction_pause_->set_paused(job_id, paused, reason, detail);
      };
    predock_ports.set_docking_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & message,
      const bool healthy) {
        dependencies_.runtime_mode->set_docking(active, state, message, healthy);
      };
    predock_ports.observation_snapshot = [this]() {
        const auto snapshot = runtime_->observation_snapshot();
        return predock_alignment::PredockObservationSnapshot{
        snapshot.age_sec, snapshot.usable, snapshot.detail};
      };
    predock_ports.ensure_docking_manager_running = [this](std::string & detail) {
        return runtime_->ensure_manager_running(detail);
      };
    predock_ports.floor_runtime_operation_blocked = [this](
      const std::string & operation, std::string & detail) {
        return dependencies_.floor_switch->operation_blocked(operation, detail);
      };
    predock_ports.start_fine_docking = [this](std::string & detail) {
        return runtime_->start_fine_docking(detail);
      };
    predock_control_ = std::make_unique<predock_alignment::PredockControlModule>(
      node_.get_logger(),
      std::move(configuration_.predock_control),
      *predock_alignment_policy_,
      *job_store_,
      std::move(predock_ports));

    DockingJobExecutionModulePorts execution_ports;
    execution_ports.floor_runtime_operation_blocked = [this](
      const std::string & operation,
      std::string & detail,
      std::string * reason_code) {
        return dependencies_.floor_switch->operation_blocked(
          operation, detail, reason_code);
      };
    execution_ports.resume_navigation_runtime = [this](
      const DockingJob & job, std::string & detail) {
        std::optional<MapManifest> selected_map;
        if (!job.map_id.empty()) {
          selected_map = dependencies_.maps->catalog().find_map_by_id(job.map_id);
        } else {
          selected_map = dependencies_.maps->catalog().active_floor_map(
            job.building_id, job.floor_id);
        }
        const auto response = late_dependencies_.navigation->module().resume_floor_navigation(
          job.building_id,
          job.floor_id,
          selected_map,
          late_dependencies_.elevator->capture_motion_admission_epoch());
        detail = response.body;
        return response.status < 400;
      };
    execution_ports.now = [this]() {
        return node_.now();
      };
    execution_ports.navigation_action_mutex = [this]() -> std::mutex & {
        return late_dependencies_.navigation->action_runtime().mutex();
      };
    execution_ports.navigation_action_client = [this]() {
        return late_dependencies_.navigation->action_runtime().client();
      };
    execution_ports.track_navigation_goal = [this](
      NavigateGoalHandle::SharedPtr goal_handle,
      std::string pose_id,
      std::string building_id,
      std::string floor_id) {
        late_dependencies_.navigation->action_runtime().track_goal(
          std::move(goal_handle),
          std::move(pose_id),
          std::move(building_id),
          std::move(floor_id));
      };
    execution_ports.cancel_active_navigation_goal = [this](std::string & detail) {
        return late_dependencies_.navigation->module().cancel_active_goal(detail);
      };
    execution_ports.request_navigation_goal_cancel = [this](const std::string & reason) {
        return late_dependencies_.navigation->goal_execution().request_goal_cancel(reason);
      };
    execution_ports.ordinary_final_yaw_align_active = [this]() {
        return late_dependencies_.navigation->goal_execution().
               ordinary_final_yaw_align_active();
      };
    execution_ports.record_docking_cmd_owner_conflict = [this]() {
        late_dependencies_.navigation->goal_execution().record_docking_cmd_owner_conflict();
      };
    execution_ports.publish_final_yaw_align_zero_burst = [this]() {
        late_dependencies_.navigation->goal_execution().publish_final_yaw_align_zero_burst();
      };
    execution_ports.clear_teleop_command = [this]() {
        dependencies_.teleop->clear_command();
      };
    execution_ports.publish_teleop_zero_burst = [this]() {
        dependencies_.teleop->publish_zero_burst();
      };
    execution_ports.set_navigation_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & message,
      const bool healthy) {
        dependencies_.runtime_mode->set_navigation(active, state, message, healthy);
      };
    execution_ports.set_docking_runtime_state = [this](
      const bool active,
      const std::string & state,
      const std::string & message,
      const bool healthy) {
        dependencies_.runtime_mode->set_docking(active, state, message, healthy);
      };
    execution_ports.bridge_safe_for_goal_start = [this](
      const std::string & context, std::string & detail) {
        return late_dependencies_.navigation->bridge_safe_for_goal_start(context, detail);
      };
    execution_ports.trigger_localization_and_wait_for_result = [this](
      const std::string & reason,
      std::string & detail,
      const double wait_timeout_sec,
      std::uint64_t * accepted_sequence) {
        return dependencies_.localization->trigger_localization_and_wait_for_result(
          reason, detail, wait_timeout_sec, accepted_sequence);
      };
    execution_ports.wait_for_relocalization_settle = [this](
      const std::uint64_t expected_sequence,
      const std::string & reason,
      const std::string & target_next_stage,
      const std::function<bool(std::string &)> & cancel_requested) {
        const auto result = late_dependencies_.post_relocalization_settle->wait_for_settle(
          expected_sequence, reason, target_next_stage, cancel_requested);
        return DockingRelocalizationSettleResult{
        result.ok, result.failure_code, result.detail};
      };
    execution_ports.clear_navigation_terminal_speed_limit = [this]() {
        late_dependencies_.navigation->goal_execution().
        clear_navigation_terminal_speed_limit();
      };
    execution_ports.publish_navigation_terminal_speed_limit_for_goal = [this](
      const StoredPose & target) {
        late_dependencies_.navigation->goal_execution().
        publish_navigation_terminal_speed_limit_for_goal(target);
      };
    execution_ports.clear_navigation_terminal_reverse_permit = [this](
      bool & permit_active, const std::string & context) {
        late_dependencies_.navigation->goal_execution().
        clear_navigation_terminal_reverse_permit(permit_active, context);
      };
    execution_ports.update_navigation_terminal_reverse_permit_for_goal = [this](
      const StoredPose & target,
      bool & permit_active,
      std::chrono::steady_clock::time_point & next_refresh_at,
      const std::string & context) {
        late_dependencies_.navigation->goal_execution().
        update_navigation_terminal_reverse_permit_for_goal(
          target, permit_active, next_refresh_at, context);
      };
    execution_ports.bms_charging_contact_snapshot = [this]() {
        return dependencies_.power->snapshot();
      };
    execution_ports.reset_terminal_actual_stop_stability = [this]() {
        late_dependencies_.navigation->terminal_runtime().reset_actual_stop_stability();
      };
    execution_ports.wait_for_terminal_actual_stop = [this](
      const std::string & context,
      std::string & detail,
      const bool require_dual_ackermann_mode) {
        return late_dependencies_.navigation->terminal_runtime().wait_for_actual_stop(
          context, detail, require_dual_ackermann_mode);
      };
    job_execution_ = std::make_unique<DockingJobExecutionModule>(
      std::move(configuration_.job_execution),
      *job_store_,
      *dependencies_.delayed_side_effect_unknown_count,
      std::move(execution_ports));
    job_executor_ = std::make_unique<DockingJobExecutor>(
      std::move(configuration_.job_executor),
      late_dependencies_.navigation->action_runtime(),
      *predock_control_,
      *job_execution_);

    configuration::DockingPredockPoseResolverPorts resolver_ports;
    resolver_ports.find_pose = [this](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & pose_id) {
        return find_floor_catalog_pose(
          dependencies_.maps->catalog(), building_id, floor_id, pose_id);
      };
    resolver_ports.read_poses = [this](
      const std::string & building_id, const std::string & floor_id) {
        const auto path = poses_yaml_path(
          dependencies_.maps->catalog(), building_id, floor_id);
        return fs::exists(path) ? read_floor_poses(path) : std::vector<StoredPose>{};
      };
    predock_pose_resolver_ =
      std::make_unique<configuration::DockingPredockPoseResolver>(
      std::move(configuration_.predock_pose_resolver), std::move(resolver_ports));

    DockingHttpPorts http_ports;
    http_ports.floor_runtime_interlock_response = [this](const std::string & operation) {
        return dependencies_.floor_switch->interlock_response(operation);
      };
    http_ports.mapping_start_job_running = [this]() {
        return dependencies_.mapping->snapshot(false).start_job_running;
      };
    http_ports.read_runtime_map_context = [this]() {
        return dependencies_.map_runtime_state_store->read_runtime_map_context();
      };
    http_ports.resolve_predock_pose = [this](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & dock_id,
      const StoredPose & dock_pose,
      const std::string & requested_predock_pose_id,
      std::string & source,
      std::string & error,
      int & error_status) {
        return predock_pose_resolver_->resolve(
          building_id,
          floor_id,
          dock_id,
          dock_pose,
          requested_predock_pose_id,
          source,
          error,
          error_status);
      };
    http_ports.validate_predock_pose = [this](
      const StoredPose & dock_pose,
      const StoredPose & predock_pose,
      std::string & error) {
        return predock_pose_resolver_->validate(dock_pose, predock_pose, error);
      };
    http_ports.clear_teleop_command = [this]() {
        dependencies_.teleop->clear_command();
      };
    http_ports.publish_teleop_zero_burst = [this]() {
        dependencies_.teleop->publish_zero_burst();
      };
    http_ports.prepare_controlled_undock = [this](std::string & detail) {
        return prepare_controlled_undock(detail);
      };
    http_ports.join_docking_worker = [this]() {
        runtime_->join_worker();
      };
    http_ports.launch_docking_worker = [this](
      const std::uint64_t job_id, std::string & error) {
        return runtime_->launch_worker(job_id, error);
      };
    http_ports.cancel_active_navigation_goal = [this](std::string & detail) {
        return late_dependencies_.navigation->module().cancel_active_goal(detail);
      };
    http_ports.stop_docking_if_available = [this](std::string & detail) {
        return runtime_->stop_if_available(detail);
      };
    http_ports.set_global_correction_paused = [this](
      const std::uint64_t job_id,
      const bool paused,
      const std::string & reason,
      std::string & detail) {
        return correction_pause_->set_paused(job_id, paused, reason, detail);
      };
    http_ports.finish_docking_job = [this](
      const std::uint64_t job_id,
      const bool ok,
      const std::string & final_state,
      const std::string & detail) {
        job_store_->finish(job_id, ok, final_state, detail);
      };
    http_ports.update_dock_contact_latch = [this](
      const bool docked,
      const std::string & source,
      const std::string & reason,
      const std::string & dock_id,
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id,
      const std::string & note) {
        contact_interlock_->update_latch(
          docked, source, reason, dock_id, building_id, floor_id, map_id, note);
      };
    http_ports.dock_contact_latch_json = [this]() {
        return contact_interlock_->latch_snapshot_json(contact_interlock_->read_latch());
      };
    http_ports.bms_charging_contact_snapshot = [this]() {
        return dependencies_.power->snapshot();
      };
    http_ports.docking_occupancy_snapshot = [this]() {
        const auto check = contact_interlock_->snapshot();
        DockingHttpOccupancySnapshot snapshot;
        snapshot.runtime_state_undocking = check.runtime_state_undocking;
        snapshot.docking_status_indicates_undocking =
          check.docking_status_indicates_undocking;
        snapshot.dock_latch_indicates_docked = check.dock_latch_indicates_docked;
        snapshot.inferred_docked = check.inferred_docked;
        snapshot.final_is_docked_or_charging = check.final_is_docked_or_charging;
        snapshot.can_auto_undock = check.can_auto_undock;
        snapshot.charging_session_latched = check.charging_session_latched;
        snapshot.full_charge_idle_on_dock = check.full_charge_idle_on_dock;
        snapshot.charging_session_age_sec = check.charging_session_age_sec;
        snapshot.charging_session_last_confirmed_at =
          check.charging_session_last_confirmed_at;
        snapshot.dock_occupancy_state = check.dock_occupancy_state;
        snapshot.dock_occupancy_evidence = check.dock_occupancy_evidence;
        snapshot.dock_occupancy_reason = check.dock_occupancy_reason;
        snapshot.auto_undock_reason = check.auto_undock_reason;
        snapshot.json = contact_interlock_->pre_navigation_check_json(
          check, PreNavigationDockCheckContext{"docking_state", "", "", "", "map", false});
        return snapshot;
      };
    http_ports.ensure_docking_manager_running = [this](std::string & detail) {
        return runtime_->ensure_manager_running(detail);
      };
    http_ports.call_undock_service_with_charging_retry = [this](
      std::string & detail,
      const bool allow_charging_retry,
      DockingUndockServiceObservation * observation) {
        return runtime_->call_undock_with_charging_retry(
          detail, allow_charging_retry, observation);
      };
    http_ports.record_undock_status_observation = [this](
      DockingJob & job, const std::string & status) {
        runtime_->observe_undock_status(job, status);
      };
    http_ = std::make_unique<DockingHttpModule>(
      *dependencies_.maps,
      *late_dependencies_.elevator,
      *dependencies_.runtime_mode,
      docking_start_mutex_,
      *job_store_,
      std::move(configuration_.http),
      std::move(http_ports));

    DockingStatusPorts status_ports;
    status_ports.update_dock_contact_latch = [this](
      const bool docked,
      const std::string & source,
      const std::string & reason,
      const std::string & dock_id) {
        contact_interlock_->update_latch(docked, source, reason, dock_id);
      };
    status_ports.finish_docking_job_locked = [this](
      const bool ok,
      const std::string & final_state,
      const std::string & detail) {
        return job_store_->finish_locked(ok, final_state, detail);
      };
    status_ports.set_global_correction_paused = [this](
      const std::uint64_t job_id,
      const bool paused,
      const std::string & reason,
      std::string & detail) {
        return correction_pause_->set_paused(job_id, paused, reason, detail);
      };
    status_ports.trigger_localization_and_wait_for_result = [this](
      const std::string & reason,
      std::string & detail,
      const double timeout_sec,
      std::uint64_t * sequence) {
        return dependencies_.localization->trigger_localization_and_wait_for_result(
          reason, detail, timeout_sec, sequence);
      };
    status_ports.wait_for_relocalization_settle = [this](
      const std::uint64_t expected_sequence,
      const std::string & reason,
      const std::string & target_next_stage,
      const std::function<bool(std::string &)> & cancel_requested) {
        const auto result = late_dependencies_.post_relocalization_settle->wait_for_settle(
          expected_sequence, reason, target_next_stage, cancel_requested);
        return DockingRelocalizationSettleResult{
        result.ok, result.failure_code, result.detail};
      };
    status_ports.release_stale_docking_fine_pause = [this](
      const std::string & reason, std::string & detail) {
        return correction_pause_->release_stale_if_needed(reason, detail);
      };
    status_ports.localization_visibility = [this]() {
        const auto bridge_status = dependencies_.localization->bridge_status_snapshot();
        const auto amcl_status = dependencies_.localization->read_amcl_runtime_status(
          robot_api_server::wall_time_seconds());
        const bool amcl_file_authoritative = amcl_status.available && !amcl_status.stale;
        const bool bridge_amcl_available =
          bridge_status.available && bridge_status.amcl_input_enabled;
        const bool effective_amcl_ready = bridge_amcl_available ?
          bridge_status.amcl_ready : (amcl_file_authoritative && amcl_status.ready);
        const bool effective_amcl_degraded = bridge_amcl_available ?
          bridge_status.localization_degraded :
          (amcl_file_authoritative && (amcl_status.degraded || !amcl_status.ready));
        const bool using_triggered_baseline_only =
          (bridge_amcl_available ||
          (amcl_status.available && amcl_status.mode != "disabled")) &&
          !effective_amcl_ready;
        return DockingLocalizationVisibility{
        effective_amcl_ready,
        effective_amcl_degraded,
        using_triggered_baseline_only};
      };
    status_ports.post_deferred_work = [this](std::function<void()> work) {
        return service_work_queue_.post(std::move(work));
      };
    status_ports.record_undock_status_observation = [this](
      DockingJob & job, const std::string & status) {
        runtime_->observe_undock_status(job, status);
      };
    status_ = std::make_unique<DockingStatusModule>(
      node_.get_logger(),
      *dependencies_.runtime_mode,
      *job_store_,
      std::move(configuration_.status),
      std::move(status_ports));

    complete_ = true;
  }

  DockingHttpModule & http()
  {
    if (!http_) {
      throw std::logic_error("DockingFeatureModule HTTP requested before completion");
    }
    return *http_;
  }

  void prepare_shutdown()
  {
    if (shutdown_prepared_) {
      return;
    }
    shutdown_prepared_ = true;
    service_work_queue_.shutdown();
  }

  void shutdown()
  {
    if (shutdown_) {
      return;
    }
    shutdown_ = true;
    prepare_shutdown();
    if (runtime_) {
      runtime_->shutdown();
    }
    if (status_) {
      status_->shutdown();
    }
  }

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  configuration::DockingConfiguration configuration_;
  DockingFeatureModuleDependencies dependencies_;
  DockingFeatureLateDependencies late_dependencies_;
  std::mutex docking_start_mutex_;
  std::mutex dock_safety_interlock_mutex_;
  bool have_dock_safety_interlock_message_{false};
  robot_interfaces::msg::DockSafetyInterlockState dock_safety_interlock_message_;
  std::chrono::steady_clock::time_point dock_safety_interlock_received_at_{};
  rclcpp::Subscription<robot_interfaces::msg::DockSafetyInterlockState>::SharedPtr
    dock_safety_interlock_state_sub_;
  rclcpp::Client<robot_interfaces::srv::ReconcileDockInterlock>::SharedPtr
    dock_safety_interlock_reconcile_client_;
  DeferredWorkQueue service_work_queue_;
  std::unique_ptr<predock_alignment::PredockAlignmentPolicy> predock_alignment_policy_;
  std::unique_ptr<DockContactInterlockModule> contact_interlock_;
  std::unique_ptr<DockingJobStore> job_store_;
  std::unique_ptr<DockingCorrectionPauseModule> correction_pause_;
  std::unique_ptr<DockingRuntimeModule> runtime_;
  std::unique_ptr<PreNavigationUndockModule> pre_navigation_undock_;
  std::unique_ptr<predock_alignment::PredockControlModule> predock_control_;
  std::unique_ptr<DockingJobExecutionModule> job_execution_;
  std::unique_ptr<DockingJobExecutor> job_executor_;
  std::unique_ptr<configuration::DockingPredockPoseResolver> predock_pose_resolver_;
  std::unique_ptr<DockingHttpModule> http_;
  std::unique_ptr<DockingStatusModule> status_;
  bool complete_{false};
  bool shutdown_prepared_{false};
  bool shutdown_{false};
};

DockingFeatureModule::DockingFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  configuration::DockingConfiguration configuration,
  DockingFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      std::move(dependencies)))
{
}

DockingFeatureModule::~DockingFeatureModule() = default;

void DockingFeatureModule::complete(DockingFeatureLateDependencies dependencies)
{
  impl_->complete(std::move(dependencies));
}

bool DockingFeatureModule::complete() const
{
  return impl_->complete_;
}

DockContactInterlockModule & DockingFeatureModule::contact_interlock()
{
  return *impl_->contact_interlock_;
}

DockingJobStore & DockingFeatureModule::job_store()
{
  return *impl_->job_store_;
}

DockingRuntimeModule & DockingFeatureModule::runtime()
{
  return *impl_->runtime_;
}

PreNavigationUndockModule & DockingFeatureModule::pre_navigation_undock()
{
  return *impl_->pre_navigation_undock_;
}

DockingHttpModule & DockingFeatureModule::http()
{
  return impl_->http();
}

bool DockingFeatureModule::job_running() const
{
  return impl_->job_store_->snapshot().state == "running";
}

void DockingFeatureModule::stop_predock_motion()
{
  if (impl_->predock_control_) {
    impl_->predock_control_->stop_and_release_motion();
  }
}

void DockingFeatureModule::prepare_shutdown()
{
  impl_->prepare_shutdown();
}

void DockingFeatureModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::docking
