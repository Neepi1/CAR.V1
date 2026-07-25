#pragma once

#include <cstdint>
#include <string>

#include "robot_floor_manager/floor_transition_core.hpp"

namespace robot_floor_manager
{

enum class FloorSwitchFailureCode : std::uint16_t
{
  kNone = 0U,
  kInvalidGoal = 10U,
  kTransactionConflict = 11U,
  kAssetNotFound = 20U,
  kAssetIdentityMismatch = 21U,
  kAssetBundleInvalid = 22U,
  kAssetDigestMismatch = 23U,
  kMotionHoldUnproven = 30U,
  kNavIdleUnproven = 31U,
  kStoppedUnproven = 32U,
  kEvidenceStale = 33U,
  kRuntimeContextUnproven = 34U,
  kLocalizerReloadUnproven = 40U,
  kLiveSwitchDisabled = 41U,
  kCancelledBeforeMutation = 90U,
  kInternalError = 99U,
};

struct FloorSwitchPreflightInput
{
  FloorTransitionRequest request;
  bool live_floor_switch_enabled{false};
  bool exact_asset_identity_proven{false};
  bool localizer_reload_proven{false};
};

struct FloorSwitchPreflightDecision
{
  bool ready{false};
  bool mutation_allowed{false};
  FloorSwitchFailureCode failure_code{FloorSwitchFailureCode::kInternalError};
  std::string code;
  std::string detail;
};

FloorSwitchPreflightDecision evaluate_floor_switch_preflight(
  const FloorSwitchPreflightInput & input);

const char * to_string(FloorSwitchFailureCode code) noexcept;

}  // namespace robot_floor_manager
