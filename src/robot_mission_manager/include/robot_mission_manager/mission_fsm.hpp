#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace robot_mission_manager
{

enum class MissionState
{
  kIdle,
  kUsingElevator,
  kNavigatingToTarget,
  kSucceeded,
  kFailureLocked,
};

enum class EffectKind
{
  kElevatorTask,
  kNavTarget,
  kHoldAndCancel,
};

struct MissionRequest
{
  std::string mission_id;
  std::string building_id;
  std::string expected_source_floor_id;
  std::string expected_source_map_id;
  std::string target_floor_id;
  std::string target_map_id;
  std::string target_pose_id;
  std::string preferred_elevator_id;
  std::uint64_t expected_asset_epoch{0U};
  std::string expected_asset_digest;
};

struct MissionEffect
{
  EffectKind kind{EffectKind::kHoldAndCancel};
  std::string transaction_id;
  std::string mission_id;
  std::string building_id;
  std::string expected_source_floor_id;
  std::string expected_source_map_id;
  std::string target_floor_id;
  std::string target_map_id;
  std::string target_pose_id;
  std::string preferred_elevator_id;
  std::uint64_t expected_asset_epoch{0U};
  std::string expected_asset_digest;
  std::string cancel_transaction_id;
  std::string cleanup_id;
  std::string reason;
};

struct EffectCompletion
{
  std::string transaction_id;
  bool success{false};
  std::string message;
  std::string final_floor_id;
  std::string final_map_id;
  std::string final_zone;
  bool safety_hold_active{false};
  std::uint64_t final_asset_epoch{0U};
  std::string final_asset_digest;
  std::uint64_t explicit_relocalization_sequence{0U};
  bool runtime_context_valid{false};
  bool nav_goal_active{false};
  bool residual_mode_lease{false};
  bool residual_execution_lease{false};
  bool residual_correction_pause{false};
  std::string final_building_id;
};

struct MissionTransition
{
  bool accepted{false};
  bool ignored{false};
  std::string message;
  std::optional<MissionEffect> effect;
};

// A deterministic, event-driven orchestration core.
//
// The class deliberately has no ROS, clock, navigation, TF, or velocity
// dependency.  An adapter owns all I/O and must acknowledge each effect with
// its exact transaction_id before the FSM can emit the next effect.
class MissionFsm
{
public:
  explicit MissionFsm(std::string instance_id);

  MissionTransition start(const MissionRequest & request);
  MissionTransition complete_effect(const EffectCompletion & completion);
  MissionTransition abort(const std::string & reason);
  MissionTransition cancel(const std::string & reason);

  MissionState state() const noexcept;
  bool has_active_effect() const noexcept;
  std::optional<MissionEffect> active_effect() const;
  const std::string & failure_reason() const noexcept;

private:
  MissionTransition emit_elevator_task();
  MissionTransition emit_nav_target();
  MissionTransition lock_failure(const std::string & reason);
  std::string next_transaction_id();

  MissionState state_{MissionState::kIdle};
  std::optional<MissionRequest> mission_;
  std::optional<MissionEffect> active_effect_;
  std::string failure_reason_;
  std::string instance_id_;
  std::uint64_t next_transaction_sequence_{1U};
};

const char * to_string(MissionState state) noexcept;
const char * to_string(EffectKind kind) noexcept;

}  // namespace robot_mission_manager
