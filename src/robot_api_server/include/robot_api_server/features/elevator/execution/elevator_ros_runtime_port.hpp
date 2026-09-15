#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"
#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_elevator_manager/elevator_execution_module.hpp"

namespace robot_api_server
{

class ElevatorMotionAdmissionFence;

struct ElevatorRosRuntimeOptions
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file{
    "/tmp/njrh_runtime_map_context.json"};
  std::filesystem::path hold_sequence_state_file{
    "/tmp/njrh_elevator_hold_sequence.state"};
  // Returns {navigation_idle, mapping_idle, docking_idle}. The API server
  // supplies its authoritative process/runtime snapshot; absence fails closed.
  std::function<std::array<bool, 3>()> runtime_idle_probe;
  // Returns the number of timed-out motion/runtime submissions whose eventual
  // server-side effect is still unknown. Elevator prepare must reject while
  // this is non-zero, even if the owning API job has already ended.
  std::function<std::uint64_t()> delayed_side_effect_unknown_probe;
  // Reuses robot_api_server's existing, freshness-gated map pose cache. The
  // elevator adapter must not create another /tf subscription just to choose
  // the nearby hall-call motion profile.
  std::function<std::optional<ElevatorMapPose>()> current_map_pose_probe;
  // Shared with every ordinary API motion/runtime admission seam.
  std::shared_ptr<ElevatorMotionAdmissionFence> motion_admission_fence;
  // Production disables the historical terminal recovery latch. Transient
  // cleanup still closes the admission fence, but a rejected prepare must not
  // leave it closed after the request has already failed.
  bool persistent_recovery_lock_enabled{true};

  // When enabled, the runtime consumes the two button effects through the
  // loopback 8083 arm black box. Door-open and floor-arrival observations
  // remain explicit operator confirmations in the pure execution module.
  bool arm_button_control_enabled{false};
  std::shared_ptr<ElevatorArmClient> arm_client;

  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string elevator_scoped_behavior_tree{
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_scoped_motion.xml"};
  std::string elevator_hall_call_scoped_behavior_tree{
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/"
    "navigate_elevator_hall_call_scoped_motion.xml"};
  std::string elevator_reverse_entry_staging_behavior_tree{
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/"
    "navigate_elevator_reverse_entry_staging.xml"};
  std::string elevator_reverse_docking_behavior_tree{
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_reverse_docking.xml"};
  std::string elevator_cabin_entry_direct_behavior_tree{
    "/workspaces/njrh-v3/workspace1/install/robot_nav_config/share/"
    "robot_nav_config/behavior_trees/navigate_elevator_cabin_entry_direct.xml"};
  std::string floor_switch_action{"/floor_manager/floor_switch"};
  std::string motion_hold_service{"/safety/set_motion_hold"};
  std::string recovery_hold_release_service{
    "/safety/release_motion_hold_if_execution_idle"};
  std::string mode_service{"/robot_mode/set_mode"};
  std::string correction_pause_service{
    "/robot_localization_bridge/set_correction_pause_lease"};
  std::string elevator_controller_session_service_prefix{
    "/controller_server"};

  // A non-empty transaction id is refreshed only while an explicitly scoped
  // post-call NavigateToPose action owns the exact elevator transaction.
  // robot_safety independently validates the ELEVATOR_WAIT/DOORWAY mode
  // contract, hold state, fresh permit, and id. The topic name is unchanged.
  std::string elevator_entry_collision_bypass_permit_topic{
    "/ranger_mini3/elevator_entry_collision_bypass"};

  std::string motion_allowed_topic{"/safety/motion_allowed"};
  std::string safety_status_topic{"/safety/status"};
  std::string motion_interlock_topic{"/safety/motion_interlock_state"};
  std::string operating_mode_topic{"/robot_mode/state"};
  std::string correction_pause_topic{"/localization/correction_pause_state"};
  std::string floor_switch_status_topic{"/floor_manager/transition_status"};
  std::string navigation_status_topic{"/navigate_to_pose/_action/status"};
  std::string floor_switch_action_status_topic{
    "/floor_manager/floor_switch/_action/status"};
  std::string localizer_asset_state_topic{"/global_localization/asset_state"};
  std::string localization_health_topic{"/localization/floor_health"};
  std::string localization_bridge_status_topic{
    "/localization/bridge_status"};
  std::string wheel_odom_topic{"/wheel/odom"};
  std::string local_odom_topic{"/local_state/odometry"};

  double endpoint_timeout_sec{3.0};
  double service_timeout_sec{5.0};
  double navigation_timeout_sec{180.0};
  double floor_switch_timeout_sec{90.0};
  double stop_timeout_sec{5.0};
  double navigation_idle_stable_sec{0.20};
  double state_evidence_max_age_sec{2.0};
  double stop_feedback_max_age_sec{0.20};
  double stop_settle_sec{0.50};
  double stop_linear_threshold_mps{0.02};
  double stop_angular_threshold_radps{0.03};
  double mode_lease_sec{5.0};
  double nearby_hall_call_scoped_distance_m{2.5};
  double target_map_pose_freshness_sec{0.50};
  double target_map_pose_settle_sec{0.20};
  double target_map_pose_stability_translation_m{0.03};
  double target_map_pose_stability_yaw_rad{0.03};
  double elevator_entry_collision_bypass_refresh_sec{0.20};
};

// Production ROS 2 port for the pure elevator execution module. It owns a
// small internal node/executor so blocking runtime effects never block the
// robot_api_server executor that must deliver their service/action callbacks.
class ElevatorRosRuntimePort final :
  public robot_elevator_manager::ElevatorRuntimePort
{
public:
  explicit ElevatorRosRuntimePort(ElevatorRosRuntimeOptions options);
  ~ElevatorRosRuntimePort() override;

  ElevatorRosRuntimePort(const ElevatorRosRuntimePort &) = delete;
  ElevatorRosRuntimePort & operator=(const ElevatorRosRuntimePort &) = delete;
  ElevatorRosRuntimePort(ElevatorRosRuntimePort &&) = delete;
  ElevatorRosRuntimePort & operator=(ElevatorRosRuntimePort &&) = delete;

  robot_elevator_manager::ElevatorRuntimeCapabilities capabilities()
  const noexcept override;
  robot_elevator_manager::ElevatorRuntimeResult prepare(
    const std::string & transaction_id,
    const robot_elevator_manager::FrozenElevatorRelease & release) override;
  robot_elevator_manager::ElevatorRuntimeResult apply(
    const robot_elevator_manager::ElevatorRuntimeEffect & effect) override;
  robot_elevator_manager::ElevatorRuntimeResult recover_locked(
    const robot_elevator_manager::ElevatorRuntimeCleanupContext & context)
  override;
  robot_elevator_manager::ElevatorRuntimeResult finalize_recovery(
    const robot_elevator_manager::ElevatorRuntimeCleanupContext & context)
  override;
  void request_cancel(const std::string & transaction_id) noexcept override;
  robot_elevator_manager::ElevatorRuntimeResult heartbeat(
    const std::string & transaction_id) override;
  robot_elevator_manager::ElevatorRuntimeResult poll_health(
    const std::string & transaction_id) override;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace robot_api_server
