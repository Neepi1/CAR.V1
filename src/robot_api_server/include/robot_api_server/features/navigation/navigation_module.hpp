#pragma once

#include <chrono>
#include <filesystem>
#include <cstdint>
#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/navigation/mission/navigation_completion_policy.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_policy.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/navigation_state.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_cancel_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_process_runtime.hpp"
#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"
#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/safety/safety_state.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationModuleConfig
{
  NavigationActionRuntimeConfig action;
  NavigationGoalPolicyConfig goal_policy;
  NavigationCompletionPolicyConfig completion_policy;
  NavigationBridgeWaitConfig bridge_wait;
  TerminalControlConfig terminal_control;
  NavigationProcessRuntimeConfig process;
  double lifecycle_check_timeout_sec{1.5};
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  double resume_starting_context_ttl_sec{300.0};
  bool nav2_native_goal_completion_enabled{true};
  bool api_final_yaw_align_fallback_enabled{true};
  bool navigation_final_yaw_align_enabled{true};
  bool post_nav2_final_verify_enabled{true};
  bool post_nav2_final_verify_wait_bridge_smoothing{true};
  int post_nav2_final_verify_max_retry_count{3};
  int navigation_nav2_failed_near_goal_retry_max_count{1};
  bool post_nav2_final_verify_api_velocity_correction_enabled{true};
  bool nav2_rotation_shim_enabled{true};
  double navigation_final_yaw_align_timeout_sec{8.0};
  double navigation_final_yaw_align_max_xy_drift_m{0.08};
  std::string navigation_final_yaw_align_cmd_topic{"/cmd_vel_api"};
  bool navigation_final_yaw_align_bypass_collision_monitor{true};
  std::string position_only_nav2_yaw_mode{"approach_heading"};
  std::chrono::nanoseconds cancel_action_wait{std::chrono::milliseconds(750)};
};

struct NavigationPreGoalDockContext
{
  std::string operation{"navigation_pre_goal_check"};
  std::string pose_id;
  std::string building_id;
  std::string floor_id;
  std::string frame_id;
  bool has_direct_pose_query{false};
};

struct NavigationPreGoalDockSnapshot
{
  bool auto_undock_required{false};
  bool pre_navigation_recovery_required{false};
  bool pre_navigation_blocked{false};
  bool clear_stale_safety_interlock_required{false};
  bool can_auto_undock{false};
  bool docking_active_not_docked_block{false};
  bool runtime_state_undocking{false};
  bool docking_status_indicates_undocking{false};
  bool bms_contact{false};
  bool live_bms_charging_contact_stable{false};
  bool dock_latch_indicates_docked{false};
  bool dock_contact_latch_stale{false};
  bool dock_contact_latch_contradicted_by_live_state{false};
  bool latch_valid_for_auto_undock{false};
  bool strong_live_docked{false};
  double dock_contact_latch_age_sec{-1.0};
  std::string dock_contact_latch_source;
  std::string auto_undock_reason{"not_docked"};
  std::string pre_navigation_recovery_action{"NONE"};
  std::string pre_navigation_block_reason;
  std::string resolved_dock_id;
  std::string dock_zone_state{"UNKNOWN"};
  std::string dock_zone_reason{"not_evaluated"};
  double dock_zone_distance_m{-1.0};
  std::string dock_occupancy_state{"UNKNOWN"};
  std::vector<std::string> dock_occupancy_evidence;
  std::string dock_occupancy_reason{"no_dock_evidence"};
  bool charging_session_latched{false};
  double charging_session_age_sec{-1.0};
  std::string charging_session_last_confirmed_at;
  bool full_charge_idle_on_dock{false};
  std::string runtime_docking_state;
  std::string runtime_docking_status;
  std::string json{"{}"};
  std::function<std::string(const NavigationPreGoalDockContext &)> render_json;
};

struct NavigationRuntimeNeighborSnapshot
{
  application::runtime_mode::RuntimeModeSnapshot runtime;
  std::string mode_transition_owner;
  bool mapping_start_job_running{false};
  bool docking_job_running{false};
};

struct NavigationModulePorts
{
  NavigationActionRuntimePorts action;
  std::function<NavigationPreGoalDockSnapshot(
      const NavigationPreGoalDockContext &)> pre_goal_dock_snapshot;
  std::function<std::optional<HttpResponse>(const std::string & operation)>
  floor_runtime_interlock_response;
  std::function<NavigationRuntimeNeighborSnapshot(bool refresh_mapping)>
  runtime_neighbor_snapshot;
  std::function<void(bool, const std::string &, const std::string &, bool)>
  set_navigation_runtime_state;
  std::function<std::optional<RuntimeMapContext>()> read_runtime_map_context;
  std::function<void(
      const MapManifest &,
      const std::string &,
      bool,
      const std::string &)> write_runtime_map_context;
  std::function<void(const MapManifest &, const std::string &)>
  write_last_navigation_map_selection;
  std::function<void()> clear_terminal_speed_limit;
  std::function<void()> publish_zero_motion;
  std::function<features::safety::SafetyStateSnapshot()> safety_snapshot;
  std::function<std::string()> post_relocalization_settle_json;
  std::function<std::string()> post_undock_settle_json;
  std::function<void(
      std::uint64_t,
      const nav2_msgs::action::NavigateToPose::Goal &,
      const StoredPose &,
      const std::string &,
      const std::string &,
      const std::string &,
      const NavigationPreGoalDockSnapshot &)> run_goal_job;
};

struct NavigationLifecycleSnapshot
{
  bool active{false};
  std::string detail;
};

// Composition boundary for the complete navigation feature. HTTP mission
// orchestration is migrated behind this boundary in the same module phase;
// these accessors temporarily support docking's shared Nav2 action ownership.
class NavigationModule
{
public:
  NavigationModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
    global_costmap_lifecycle_client,
    features::maps::MapsModule & maps_module,
    features::localization::LocalizationModule & localization_module,
    features::elevator::ElevatorModule & elevator_module,
    NavigationModuleConfig config,
    NavigationModulePorts ports);
  ~NavigationModule();

  NavigationModule(const NavigationModule &) = delete;
  NavigationModule & operator=(const NavigationModule &) = delete;

  NavigationActionRuntime & action_runtime();
  NavigationMissionRuntime & mission_runtime();
  NavigationCancelRuntime & cancel_runtime();
  NavigationProcessRuntime & process_runtime();
  NavigationGoalPolicy & goal_policy();
  NavigationCompletionPolicy & completion_policy();
  NavigationBridgeWait & bridge_wait();
  NavigationTerminalControl & terminal_control();
  NavigationLifecycleSnapshot lifecycle_snapshot();
  NavigationLifecycleSnapshot goal_admission_snapshot();
  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);
  bool cancel_active_goal(std::string & detail);
  bool cancel_goal_for_api_handoff(
    const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
    std::string & detail);
  bool cancel_task_for_mode_switch(const std::string & reason, std::string & detail);
  bool request_goal_cancel(const std::string & reason);
  void join_cancel_worker();
  std::string cancel_job_json() const;
  HttpResponse resume_floor_navigation(
    const std::string & building_id,
    const std::string & floor_id,
    const std::optional<MapManifest> & selected_map,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);
  void refresh_runtime_state(bool probe_lifecycle = false);
  bool process_running();
  bool stop_runtime_stack(std::string & detail);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
