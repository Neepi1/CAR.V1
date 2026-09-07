#pragma once

#include <functional>
#include <string>

#include "robot_floor_manager/floor_asset_snapshot_loader.hpp"
#include "robot_floor_manager/floor_transition_core.hpp"

namespace robot_floor_manager
{

struct FloorTransitionEffectResult
{
  bool success{false};
  std::string failure_code;
  std::string detail;
  FloorTransitionEvidence evidence;
};

// Owned-runtime seam. Production uses the ROS adapter; isolated tests use an
// in-memory adapter. Implementations must return evidence for the exact effect
// and must not report success merely because a request was accepted.
class FloorTransitionRuntimePort
{
public:
  virtual ~FloorTransitionRuntimePort() = default;

  virtual FloorTransitionEffectResult perform(
    const FloorTransitionEffect & effect,
    const FloorAssetSnapshot & snapshot) = 0;
};

struct FloorTransitionExecutionResult
{
  bool success{false};
  FloorTransitionState state{FloorTransitionState::kIdle};
  std::string failure_code;
  std::string message;
  std::string active_building_id;
  std::string active_floor_id;
  std::string active_map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::uint64_t explicit_relocalization_sequence{0U};
  bool runtime_context_valid{true};
  bool recovery_required{false};
};

struct FloorTransitionFailureDisposition
{
  const char * state{"FAILED"};
  const char * stage{"FAILED"};
  bool canceled{false};
  bool recovery_locked{false};
};

using FloorTransitionCancelProbe = std::function<bool()>;
using FloorTransitionProgressSink =
  std::function<void(const FloorTransitionOutput &)>;

// Stable Action feedback contract shared with the elevator caller. The handoff
// stage is emitted only after BEGIN invalidation has advanced the core to the
// reporting barrier.
const char * floor_transition_feedback_stage(
  const FloorTransitionOutput & output) noexcept;
bool floor_transition_caller_pause_handoff_ready(
  const FloorTransitionOutput & output) noexcept;
float floor_transition_feedback_progress(
  FloorTransitionState state) noexcept;
FloorTransitionFailureDisposition floor_transition_failure_disposition(
  const FloorTransitionExecutionResult & execution) noexcept;

// Synchronous transaction module. A ROS adapter must run it outside executor
// callbacks so state subscriptions and service responses continue to progress.
class FloorTransitionExecutor
{
public:
  explicit FloorTransitionExecutor(FloorTransitionRuntimePort & runtime);

  FloorTransitionExecutionResult run(
    const FloorTransitionRequest & request,
    const FloorAssetSnapshot & snapshot,
    FloorTransitionCancelProbe cancel_requested = {},
    FloorTransitionProgressSink progress = {});

private:
  FloorTransitionExecutionResult terminal_result(
    const FloorTransitionOutput & output,
    const FloorTransitionEvidence & evidence,
    const std::string & failure_code,
    const std::string & message) const;

  FloorTransitionRuntimePort & runtime_;
};

}  // namespace robot_floor_manager
