#include "robot_floor_manager/floor_transition_evidence_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_floor_manager
{
namespace
{

constexpr std::size_t kMaximumOdomSamples = 256U;

bool contains(
  const std::vector<std::string> & values,
  const std::string & target)
{
  return std::find(values.cbegin(), values.cend(), target) != values.cend();
}

}  // namespace

void FloorTransitionEvidenceTracker::begin(
  const FloorTransitionTarget & target,
  const double steady_now_sec)
{
  target_ = target;
  begin_steady_sec_ = steady_now_sec;
  begin_localizer_generation_ = std::max(
    localization_health_received_steady_sec_ >= 0.0 ?
    localization_health_.localizer_generation : 0U,
    localizer_asset_received_steady_sec_ >= 0.0 ?
    localizer_asset_.localizer_generation : 0U);
  begin_explicit_relocalization_sequence_ =
    localization_health_received_steady_sec_ >= 0.0 ?
    localization_health_.explicit_relocalization_sequence : 0U;
  costmaps_cleared_steady_sec_ = -1.0;
  global_costmap_clear_baseline_ = global_costmap_.receive_sequence;
  local_costmap_clear_baseline_ = local_costmap_.receive_sequence;
}

void FloorTransitionEvidenceTracker::observe_motion_interlock(
  const bool hold_active,
  const std::vector<std::string> & hold_keys,
  const double received_steady_sec)
{
  motion_interlock_ = {hold_active, hold_keys, received_steady_sec};
}

void FloorTransitionEvidenceTracker::observe_nav_graph_ready(
  const bool ready,
  const double received_steady_sec)
{
  if (!ready) {
    if (!nav_graph_ready_) {
      return;
    }
    nav_graph_ready_ = false;
    nav_graph_ready_since_steady_sec_ = -1.0;
    nav_active_ = {};
    return;
  }
  if (!nav_graph_ready_) {
    nav_graph_ready_ = true;
    nav_graph_ready_since_steady_sec_ = received_steady_sec;
  }
}

void FloorTransitionEvidenceTracker::observe_nav_activity(
  const bool active,
  const double received_steady_sec)
{
  // A status callback may arrive before the graph API reports all action
  // services discovered. Preserve that authoritative sample; the graph probe
  // will establish the bootstrap generation independently.
  nav_active_ = {active, received_steady_sec};
}

void FloorTransitionEvidenceTracker::observe_wheel_odom(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const double received_steady_sec)
{
  wheel_odom_.push_back(
    {linear_x, linear_y, angular_z, received_steady_sec});
  while (wheel_odom_.size() > kMaximumOdomSamples) {
    wheel_odom_.pop_front();
  }
}

void FloorTransitionEvidenceTracker::observe_local_odom(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const double received_steady_sec)
{
  local_odom_.push_back(
    {linear_x, linear_y, angular_z, received_steady_sec});
  while (local_odom_.size() > kMaximumOdomSamples) {
    local_odom_.pop_front();
  }
}

void FloorTransitionEvidenceTracker::observe_correction_pause(
  const bool paused,
  const std::vector<std::string> & lease_keys,
  const double received_steady_sec)
{
  correction_pause_ = {paused, lease_keys, received_steady_sec};
}

void FloorTransitionEvidenceTracker::observe_localization_health(
  const LocalizationHealthEvidence & health,
  const double received_steady_sec)
{
  localization_health_ = health;
  localization_health_received_steady_sec_ = received_steady_sec;
}

void FloorTransitionEvidenceTracker::observe_localizer_asset(
  const LocalizerAssetEvidence & state,
  const double received_steady_sec)
{
  localizer_asset_ = state;
  localizer_asset_received_steady_sec_ = received_steady_sec;
}

void FloorTransitionEvidenceTracker::observe_global_costmap(
  const std::uint64_t receive_sequence,
  const double received_steady_sec)
{
  global_costmap_ = {receive_sequence, received_steady_sec};
}

void FloorTransitionEvidenceTracker::observe_local_costmap(
  const std::uint64_t receive_sequence,
  const double received_steady_sec)
{
  local_costmap_ = {receive_sequence, received_steady_sec};
}

void FloorTransitionEvidenceTracker::mark_costmaps_cleared(
  const double steady_now_sec)
{
  global_costmap_clear_baseline_ = global_costmap_.receive_sequence;
  local_costmap_clear_baseline_ = local_costmap_.receive_sequence;
  costmaps_cleared_steady_sec_ = steady_now_sec;
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::preconditions(
  const double steady_now_sec,
  const double max_age_sec,
  const double stable_duration_sec,
  const double stopped_linear_threshold_mps,
  const double stopped_angular_threshold_radps,
  const double nav_idle_bootstrap_grace_sec) const
{
  FloorTransitionEvidence evidence;
  const auto hold_key =
    std::string{"robot_floor_manager:"} + target_.transaction_id;
  evidence.motion_hold_active =
    fresh(
    motion_interlock_.received_steady_sec, steady_now_sec, max_age_sec) &&
    motion_interlock_.active &&
    contains(motion_interlock_.keys, hold_key);
  // NavigateToPose action status is event-driven. It does not publish an
  // initial empty GoalStatusArray when a fresh server has never accepted a
  // goal. Once the action server and status writer have remained discovered
  // for a bounded grace period, the absence of any retained/live status is a
  // cold-start idle proof. Any later status sample becomes authoritative and
  // remains latched until the action graph disappears or another sample
  // arrives.
  const bool status_proves_idle =
    nav_graph_ready_ &&
    nav_active_.received_steady_sec >= 0.0 &&
    !nav_active_.value;
  const bool cold_start_proves_idle =
    nav_graph_ready_ &&
    nav_graph_ready_since_steady_sec_ >= 0.0 &&
    nav_active_.received_steady_sec < 0.0 &&
    steady_now_sec >= nav_graph_ready_since_steady_sec_ &&
    steady_now_sec - nav_graph_ready_since_steady_sec_ >=
    std::max(0.0, nav_idle_bootstrap_grace_sec);
  evidence.nav_idle = status_proves_idle || cold_start_proves_idle;
  evidence.stopped =
    odom_stopped(
    wheel_odom_, steady_now_sec, max_age_sec, stable_duration_sec,
    stopped_linear_threshold_mps, stopped_angular_threshold_radps) &&
    odom_stopped(
    local_odom_, steady_now_sec, max_age_sec, stable_duration_sec,
    stopped_linear_threshold_mps, stopped_angular_threshold_radps);
  return evidence;
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::floor_pause(
  const double steady_now_sec,
  const double max_age_sec) const
{
  auto evidence = target_identity_evidence();
  const auto floor_key =
    std::string{"robot_floor_manager:"} + target_.transaction_id;
  const bool state_fresh =
    fresh(correction_pause_.received_steady_sec, steady_now_sec, max_age_sec);
  evidence.floor_pause_owned =
    state_fresh && contains(correction_pause_.keys, floor_key);
  evidence.correction_pause_effective =
    state_fresh && correction_pause_.active && evidence.floor_pause_owned;
  return evidence;
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::pause_handoff(
  const double steady_now_sec,
  const double max_age_sec) const
{
  auto evidence = floor_pause(steady_now_sec, max_age_sec);
  evidence.caller_pause_released =
    evidence.floor_pause_owned &&
    evidence.correction_pause_effective &&
    correction_pause_.keys.size() == 1U;
  evidence.runtime_context_invalid = true;
  return evidence;
}

bool FloorTransitionEvidenceTracker::corrections_released(
  const double steady_now_sec,
  const double max_age_sec) const
{
  return
    fresh(
    correction_pause_.received_steady_sec,
    steady_now_sec,
    max_age_sec) &&
    !correction_pause_.active &&
    correction_pause_.keys.empty();
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::target_localizer(
  const std::uint64_t expected_generation,
  const double steady_now_sec,
  const double max_age_sec) const
{
  auto evidence = target_identity_evidence();
  const bool exact =
    fresh(
    localizer_asset_received_steady_sec_,
    steady_now_sec,
    max_age_sec) &&
    localizer_asset_received_steady_sec_ > begin_steady_sec_ &&
    localizer_asset_.transaction_id == target_.transaction_id &&
    !localizer_asset_.applying &&
    localizer_asset_.success &&
    localizer_asset_.reloaded &&
    localizer_asset_.active_identity_valid &&
    localizer_asset_.active_building_id == target_.building_id &&
    localizer_asset_.active_floor_id == target_.floor_id &&
    localizer_asset_.active_map_id == target_.map_id &&
    localizer_asset_.active_asset_epoch == target_.asset_epoch &&
    localizer_asset_.active_asset_digest == target_.asset_digest &&
    localizer_asset_.localizer_generation == expected_generation &&
    localizer_asset_.localizer_generation > begin_localizer_generation_ &&
    localizer_asset_.localizer_ready;
  if (!exact) {
    evidence.asset_epoch = 0U;
    evidence.asset_digest.clear();
  }
  return evidence;
}

LocalizerApplyOutcome FloorTransitionEvidenceTracker::localizer_apply_outcome(
  const std::uint64_t baseline_generation,
  const double steady_now_sec,
  const double max_age_sec) const
{
  LocalizerApplyOutcome outcome;
  const bool current_transaction =
    fresh(
    localizer_asset_received_steady_sec_,
    steady_now_sec,
    max_age_sec) &&
    localizer_asset_received_steady_sec_ > begin_steady_sec_ &&
    localizer_asset_.transaction_id == target_.transaction_id;
  if (!current_transaction) {
    return outcome;
  }

  outcome.exact_request =
    localizer_asset_.requested_building_id == target_.building_id &&
    localizer_asset_.requested_floor_id == target_.floor_id &&
    localizer_asset_.requested_map_id == target_.map_id &&
    localizer_asset_.requested_asset_epoch == target_.asset_epoch &&
    localizer_asset_.requested_asset_digest == target_.asset_digest;
  if (!outcome.exact_request) {
    return outcome;
  }

  outcome.localizer_generation = localizer_asset_.localizer_generation;
  outcome.code = localizer_asset_.code;
  outcome.detail = localizer_asset_.detail;
  if (localizer_asset_.applying) {
    return outcome;
  }

  if (!localizer_asset_.success) {
    outcome.terminal = true;
    return outcome;
  }

  const bool exact_active_reload =
    localizer_asset_.reloaded &&
    localizer_asset_.active_identity_valid &&
    localizer_asset_.active_building_id == target_.building_id &&
    localizer_asset_.active_floor_id == target_.floor_id &&
    localizer_asset_.active_map_id == target_.map_id &&
    localizer_asset_.active_asset_epoch == target_.asset_epoch &&
    localizer_asset_.active_asset_digest == target_.asset_digest &&
    localizer_asset_.localizer_generation > baseline_generation &&
    localizer_asset_.localizer_generation > begin_localizer_generation_ &&
    localizer_asset_.localizer_ready;
  if (exact_active_reload) {
    outcome.terminal = true;
    outcome.success = true;
  }
  return outcome;
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::target_localization(
  const double steady_now_sec,
  const double max_age_sec,
  const std::uint64_t expected_localizer_generation) const
{
  auto evidence = target_identity_evidence();
  const bool valid =
    fresh(
    localization_health_received_steady_sec_,
    steady_now_sec,
    max_age_sec) &&
    localization_health_received_steady_sec_ > begin_steady_sec_ &&
    exact_target(localization_health_) &&
    localization_health_.localizer_ready &&
    localization_health_.localizer_generation > begin_localizer_generation_ &&
    (expected_localizer_generation == 0U ||
    localization_health_.localizer_generation ==
    expected_localizer_generation);
  if (!valid) {
    evidence.asset_epoch = 0U;
    evidence.asset_digest.clear();
    return evidence;
  }
  evidence.explicit_relocalization_sequence =
    localization_health_.explicit_relocalization_sequence >
    begin_explicit_relocalization_sequence_ ?
    localization_health_.explicit_relocalization_sequence : 0U;
  evidence.bridge_ready =
    localization_health_.bridge_ready &&
    localization_health_.tf_unique &&
    !localization_health_.runtime_context_valid;
  evidence.amcl_ready = localization_health_.amcl_ready;
  evidence.runtime_context_invalid =
    !localization_health_.runtime_context_valid;
  return evidence;
}

FloorTransitionEvidence FloorTransitionEvidenceTracker::fresh_costmaps(
  const double steady_now_sec,
  const double max_age_sec,
  const std::uint64_t expected_localizer_generation) const
{
  auto evidence = target_localization(
    steady_now_sec, max_age_sec, expected_localizer_generation);
  if (evidence.asset_epoch == 0U || costmaps_cleared_steady_sec_ < 0.0) {
    return evidence;
  }
  evidence.global_costmap_fresh =
    fresh(
    global_costmap_.received_steady_sec,
    steady_now_sec,
    max_age_sec) &&
    global_costmap_.received_steady_sec > costmaps_cleared_steady_sec_ &&
    global_costmap_.receive_sequence > global_costmap_clear_baseline_;
  evidence.local_costmap_fresh =
    fresh(
    local_costmap_.received_steady_sec,
    steady_now_sec,
    max_age_sec) &&
    local_costmap_.received_steady_sec > costmaps_cleared_steady_sec_ &&
    local_costmap_.receive_sequence > local_costmap_clear_baseline_;
  return evidence;
}

std::uint64_t FloorTransitionEvidenceTracker::begin_localizer_generation() const noexcept
{
  return begin_localizer_generation_;
}

std::uint64_t
FloorTransitionEvidenceTracker::begin_explicit_relocalization_sequence() const noexcept
{
  return begin_explicit_relocalization_sequence_;
}

bool FloorTransitionEvidenceTracker::fresh(
  const double received_steady_sec,
  const double now_sec,
  const double max_age_sec) const
{
  return std::isfinite(received_steady_sec) &&
         std::isfinite(now_sec) &&
         std::isfinite(max_age_sec) &&
         received_steady_sec >= 0.0 &&
         now_sec >= received_steady_sec &&
         now_sec - received_steady_sec <= max_age_sec;
}

bool FloorTransitionEvidenceTracker::odom_stopped(
  const std::deque<VelocityObservation> & observations,
  const double now_sec,
  const double max_age_sec,
  const double stable_duration_sec,
  const double linear_threshold,
  const double angular_threshold) const
{
  if (
    observations.empty() ||
    !fresh(
      observations.back().received_steady_sec,
      now_sec,
      max_age_sec))
  {
    return false;
  }
  const double required_since = now_sec - stable_duration_sec;
  bool covered_start = false;
  for (auto iterator = observations.crbegin();
    iterator != observations.crend(); ++iterator)
  {
    if (
      std::abs(iterator->linear_x) > linear_threshold ||
      std::abs(iterator->linear_y) > linear_threshold ||
      std::abs(iterator->angular_z) > angular_threshold)
    {
      break;
    }
    if (iterator->received_steady_sec <= required_since) {
      covered_start = true;
      break;
    }
  }
  return covered_start;
}

bool FloorTransitionEvidenceTracker::exact_target(
  const LocalizationHealthEvidence & health) const
{
  return health.building_id == target_.building_id &&
         health.floor_id == target_.floor_id &&
         health.map_id == target_.map_id &&
         health.asset_epoch == target_.asset_epoch &&
         health.asset_digest == target_.asset_digest;
}

FloorTransitionEvidence
FloorTransitionEvidenceTracker::target_identity_evidence() const
{
  FloorTransitionEvidence evidence;
  evidence.active_building_id = target_.building_id;
  evidence.active_floor_id = target_.floor_id;
  evidence.active_map_id = target_.map_id;
  evidence.asset_epoch = target_.asset_epoch;
  evidence.asset_digest = target_.asset_digest;
  return evidence;
}

}  // namespace robot_floor_manager
