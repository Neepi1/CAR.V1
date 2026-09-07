#pragma once

#include <algorithm>
#include <cmath>

namespace robot_localization_bridge
{
namespace explicit_localization
{

enum class IsaacResultAdmission
{
  kIgnoreUnarmed,
  kIgnoreBeforeArm,
  kEvaluateExplicit,
};

inline IsaacResultAdmission classify_isaac_result(
  const bool explicit_arm_active,
  const double arm_time_sec,
  const double pose_stamp_sec,
  const double prearm_slack_sec)
{
  if (!explicit_arm_active || arm_time_sec <= 0.0) {
    return IsaacResultAdmission::kIgnoreUnarmed;
  }

  const double minimum_pose_stamp_sec =
    arm_time_sec - std::max(0.0, prearm_slack_sec);
  if (pose_stamp_sec < minimum_pose_stamp_sec) {
    return IsaacResultAdmission::kIgnoreBeforeArm;
  }
  return IsaacResultAdmission::kEvaluateExplicit;
}

// An explicitly triggered Isaac result retains its measurement timestamp and
// is admitted only when odom->base_link history exists at that timestamp.
// Wall-clock delivery age alone is not a correctness proof for a queued result.
inline bool enforce_wall_age_limit(const bool explicit_trigger)
{
  return !explicit_trigger;
}

struct ForcedTranslationLimitContext
{
  bool explicit_trigger{false};
  bool floor_transition_active{false};
  bool runtime_context_valid{true};
  bool floor_transition_failed_locked{false};
  bool exact_pending_target_localizer{false};
};

template<typename PendingTarget, typename LocalizerState>
inline bool proves_exact_pending_floor_target(
  const PendingTarget & target,
  const LocalizerState & localizer)
{
  return
    !target.transaction_id.empty() &&
    localizer.transaction_id == target.transaction_id &&
    localizer.success &&
    !localizer.applying &&
    localizer.reloaded &&
    localizer.active_identity_valid &&
    localizer.requested_building_id == target.building_id &&
    localizer.requested_floor_id == target.floor_id &&
    localizer.requested_map_id == target.map_id &&
    localizer.requested_asset_epoch == target.asset_epoch &&
    localizer.requested_asset_digest == target.asset_digest &&
    localizer.active_building_id == target.building_id &&
    localizer.active_floor_id == target.floor_id &&
    localizer.active_map_id == target.map_id &&
    localizer.active_asset_epoch == target.asset_epoch &&
    localizer.active_asset_digest == target.asset_digest &&
    localizer.localizer_generation > 0U &&
    localizer.localizer_ready;
}

inline bool forced_translation_within_limit(
  const double correction_translation_m,
  const double forced_limit_m,
  const ForcedTranslationLimitContext & context)
{
  if (!std::isfinite(correction_translation_m) || correction_translation_m < 0.0) {
    return false;
  }

  const bool exact_cross_map_floor_transition =
    context.explicit_trigger &&
    context.floor_transition_active &&
    !context.runtime_context_valid &&
    !context.floor_transition_failed_locked &&
    context.exact_pending_target_localizer;
  if (exact_cross_map_floor_transition) {
    // Independent floor maps may have unrelated origins.  Their coordinate
    // delta is not physical robot travel, so the ordinary same-map distance
    // limit is not meaningful once the exact target localizer transaction is
    // proven.
    return true;
  }

  return std::isfinite(forced_limit_m) && forced_limit_m >= 0.0 &&
         correction_translation_m <= forced_limit_m;
}

}  // namespace explicit_localization
}  // namespace robot_localization_bridge
