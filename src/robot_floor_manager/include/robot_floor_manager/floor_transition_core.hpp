#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace robot_floor_manager
{

enum class FloorTransitionState
{
  kIdle,
  kVerifyingPreconditions,
  kAcquiringCorrectionPause,
  kInvalidatingRuntimeContext,
  kReportingBeginReady,
  kVerifyingPauseHandoff,
  kLoadingNavMap,
  kLoadingFilters,
  kReloadingLocalizer,
  kReleasingFloorPause,
  kTriggeringExplicitLocalization,
  kVerifyingBridgeReady,
  kClearingCostmaps,
  kVerifyingFreshCostmaps,
  kCommittingRuntimeContext,
  kCompleting,
  kComplete,
  kFailureCleanup,
  kFailedLocked,
};

enum class FloorTransitionEffectKind
{
  kNone,
  kVerifyPreconditions,
  kAcquireCorrectionPause,
  kInvalidateRuntimeContext,
  kReportBeginReady,
  kVerifyPauseHandoff,
  kLoadNavMap,
  kLoadFilters,
  kReloadLocalizer,
  kReleaseFloorPause,
  kTriggerExplicitLocalization,
  kVerifyBridgeReady,
  kClearCostmaps,
  kVerifyFreshCostmaps,
  kCommitRuntimeContext,
  kComplete,
  kHoldAndLock,
};

enum class FloorTransitionEventKind
{
  kEffectSucceeded,
  kEffectFailed,
  kCancelRequested,
};

struct FloorTransitionRequest
{
  std::string transaction_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t expected_asset_epoch{0U};
  std::string expected_asset_digest;
};

struct FloorTransitionEvidence
{
  bool motion_hold_active{false};
  bool nav_idle{false};
  bool stopped{false};
  bool floor_pause_owned{false};
  bool correction_pause_effective{false};
  bool caller_pause_released{false};
  bool runtime_context_invalid{false};
  bool runtime_context_valid{false};
  bool bridge_ready{false};
  bool global_costmap_fresh{false};
  bool local_costmap_fresh{false};
  bool safe_for_goal_start{false};
  std::string active_building_id;
  std::string active_floor_id;
  std::string active_map_id;
  std::string asset_digest;
  std::uint64_t asset_epoch{0U};
  std::uint64_t explicit_relocalization_sequence{0U};
};

struct FloorTransitionEvent
{
  FloorTransitionEventKind kind{FloorTransitionEventKind::kEffectFailed};
  std::string transaction_id;
  std::uint64_t effect_sequence{0U};
  std::string detail;
  FloorTransitionEvidence evidence;
};

struct FloorTransitionEffect
{
  FloorTransitionEffectKind kind{FloorTransitionEffectKind::kNone};
  std::string transaction_id;
  std::uint64_t sequence{0U};
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t expected_asset_epoch{0U};
  std::string expected_asset_digest;
  std::string cleanup_id;
  std::string detail;
};

struct FloorTransitionOutput
{
  bool accepted{false};
  bool ignored{false};
  bool transitioned{false};
  FloorTransitionState state{FloorTransitionState::kIdle};
  FloorTransitionEffect effect;
  bool runtime_context_valid{true};
  bool recovery_required{false};
  std::string message;
};

class FloorTransitionCore
{
public:
  FloorTransitionOutput start(const FloorTransitionRequest & request);
  FloorTransitionOutput dispatch(const FloorTransitionEvent & event);

  FloorTransitionState state() const noexcept;
  bool runtime_context_valid() const noexcept;
  bool recovery_required() const noexcept;
  std::optional<FloorTransitionRequest> active_request() const;

private:
  FloorTransitionOutput advance(const FloorTransitionEvidence & evidence);
  FloorTransitionOutput transition(
    FloorTransitionState state,
    FloorTransitionEffectKind effect,
    const std::string & detail);
  FloorTransitionOutput fail(const std::string & reason);
  FloorTransitionOutput inert(const std::string & reason, bool ignored) const;
  bool target_asset_matches(const FloorTransitionEvidence & evidence) const;
  bool target_context_matches(const FloorTransitionEvidence & evidence) const;

  FloorTransitionState state_{FloorTransitionState::kIdle};
  std::optional<FloorTransitionRequest> request_;
  FloorTransitionEffect pending_effect_;
  std::uint64_t next_effect_sequence_{1U};
  bool mutation_started_{false};
  bool runtime_context_valid_{true};
  bool recovery_required_{false};
};

const char * to_string(FloorTransitionState state) noexcept;
const char * to_string(FloorTransitionEffectKind effect) noexcept;

}  // namespace robot_floor_manager
