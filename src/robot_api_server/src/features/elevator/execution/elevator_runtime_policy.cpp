#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"

#include <algorithm>
#include <cmath>

namespace robot_api_server
{
namespace
{

using robot_elevator_manager::ElevatorRuntimeFloor;
using robot_elevator_manager::ElevatorRuntimeCleanupContext;
using robot_elevator_manager::ElevatorRuntimeFloorIdentity;
using robot_elevator_manager::ElevatorRuntimePose;
using robot_elevator_manager::ElevatorCleanupDisposition;
using robot_elevator_manager::PoseRole;

bool same_floor_identity(
  const ElevatorRuntimeFloorIdentity & left,
  const ElevatorRuntimeFloorIdentity & right) noexcept
{
  return left.floor_id == right.floor_id &&
         left.map_id == right.map_id &&
         left.asset_epoch == right.asset_epoch &&
         left.asset_digest == right.asset_digest;
}

bool same_immutable_cleanup_identity(
  const ElevatorRuntimeCleanupContext & left,
  const ElevatorRuntimeCleanupContext & right) noexcept
{
  return left.transaction_id == right.transaction_id &&
         left.building_id == right.building_id &&
         same_floor_identity(left.source, right.source) &&
         same_floor_identity(left.target, right.target);
}

bool valid_confirmation_binding(
  const ElevatorRuntimeCleanupContext & context) noexcept
{
  return !context.source_outside_confirmation_recorded ||
         (!context.legacy_preflight_orphan &&
         context.disposition == ElevatorCleanupDisposition::kSourceOutside);
}

const ElevatorRuntimePose * find_pose(
  const ElevatorRuntimeFloor & floor,
  const PoseRole role)
{
  const auto found = std::find_if(
    floor.poses.cbegin(), floor.poses.cend(),
    [role](const ElevatorRuntimePose & pose) {
      return pose.role == role;
    });
  return found == floor.poses.cend() ? nullptr : &*found;
}

const ElevatorRuntimePose * find_pose_id(
  const ElevatorRuntimeFloor & floor,
  const std::string & pose_id)
{
  const auto found = std::find_if(
    floor.poses.cbegin(), floor.poses.cend(),
    [&pose_id](const ElevatorRuntimePose & pose) {
      return pose.pose_id == pose_id;
    });
  return found == floor.poses.cend() ? nullptr : &*found;
}

std::optional<double> directed_yaw(
  const ElevatorRuntimePose * from,
  const ElevatorRuntimePose * to)
{
  if (from == nullptr || to == nullptr) {
    return std::nullopt;
  }
  const auto dx = to->x - from->x;
  const auto dy = to->y - from->y;
  if (!std::isfinite(dx) || !std::isfinite(dy) || std::hypot(dx, dy) < 1.0e-6) {
    return std::nullopt;
  }
  return std::atan2(dy, dx);
}

}  // namespace

robot_elevator_manager::ElevatorRuntimeResult validate_elevator_prepared_source(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const ElevatorPreparedSourceEvidence & evidence)
{
  if (!evidence.runtime_context_confirmed || evidence.runtime_context_state != "ready") {
    return {
      false,
      "ELEVATOR_SOURCE_RUNTIME_CONTEXT_NOT_READY",
      "source runtime context must be confirmed and ready",
    };
  }
  if (
    evidence.building_id != release.building_id ||
    evidence.floor_id != release.source.floor_id ||
    evidence.map_id != release.source.map_id ||
    evidence.asset_epoch != release.source.map_asset_epoch ||
    evidence.asset_digest != release.source.map_asset_digest)
  {
    return {
      false,
      "ELEVATOR_SOURCE_ASSET_IDENTITY_MISMATCH",
      "source runtime and frozen elevator release identities differ",
    };
  }
  if (!evidence.navigation_idle || !evidence.mapping_idle || !evidence.docking_idle) {
    return {
      false,
      "ELEVATOR_SOURCE_RUNTIME_BUSY",
      std::string("navigation_idle=") +
      (evidence.navigation_idle ? "true" : "false") +
      ";mapping_idle=" + (evidence.mapping_idle ? "true" : "false") +
      ";docking_idle=" + (evidence.docking_idle ? "true" : "false"),
    };
  }
  return {true, "OK", "source runtime identity and idle state verified"};
}

ElevatorCleanupRuntimeIdentityAssessment
assess_elevator_cleanup_runtime_identity(
  const ElevatorCleanupRuntimeIdentityEvidence & evidence,
  const double now_sec,
  const double live_evidence_max_age_sec) noexcept
{
  ElevatorCleanupRuntimeIdentityAssessment assessment;
  assessment.localizer_asset_present = evidence.localizer_asset_present;
  assessment.localizer_asset_exact = evidence.localizer_asset_exact;
  assessment.localization_health_exact = evidence.localization_health_exact;
  assessment.localization_bridge_exact = evidence.localization_bridge_exact;

  const auto age =
    [now_sec](const bool present, const double received_at_sec) {
      if (
        !present || !std::isfinite(now_sec) ||
        !std::isfinite(received_at_sec) || received_at_sec > now_sec)
      {
        return -1.0;
      }
      return now_sec - received_at_sec;
    };
  assessment.localization_health_age_sec = age(
    evidence.localization_health_present,
    evidence.localization_health_received_at_sec);
  assessment.localization_bridge_age_sec = age(
    evidence.localization_bridge_present,
    evidence.localization_bridge_received_at_sec);

  const bool valid_max_age =
    std::isfinite(live_evidence_max_age_sec) &&
    live_evidence_max_age_sec > 0.0;
  assessment.localization_health_fresh =
    valid_max_age && assessment.localization_health_age_sec >= 0.0 &&
    assessment.localization_health_age_sec <= live_evidence_max_age_sec;
  assessment.localization_bridge_fresh =
    valid_max_age && assessment.localization_bridge_age_sec >= 0.0 &&
    assessment.localization_bridge_age_sec <= live_evidence_max_age_sec;
  assessment.proven =
    assessment.localizer_asset_present &&
    assessment.localizer_asset_exact &&
    assessment.localization_health_fresh &&
    assessment.localization_health_exact &&
    assessment.localization_bridge_fresh &&
    assessment.localization_bridge_exact;
  return assessment;
}

std::optional<ElevatorRuntimeFloorIdentity>
resolve_elevator_cleanup_runtime_floor(
  const ElevatorRuntimeCleanupContext & context,
  const ElevatorCleanupRuntimeFloorEvidence & evidence) noexcept
{
  if (
    !evidence.confirmed || evidence.state != "ready" ||
    evidence.building_id != context.building_id)
  {
    return std::nullopt;
  }

  const auto matches = [&evidence](const ElevatorRuntimeFloorIdentity & floor) {
      return evidence.floor_id == floor.floor_id &&
             evidence.map_id == floor.map_id &&
             evidence.asset_epoch == floor.asset_epoch &&
             evidence.asset_digest == floor.asset_digest;
    };

  const ElevatorRuntimeFloorIdentity * recorded_outside = nullptr;
  if (context.disposition == ElevatorCleanupDisposition::kSourceOutside) {
    recorded_outside = &context.source;
  } else if (context.disposition == ElevatorCleanupDisposition::kTargetOutside) {
    recorded_outside = &context.target;
  } else {
    return std::nullopt;
  }

  if (matches(*recorded_outside)) {
    return *recorded_outside;
  }

  if (context.floor_switch_action_may_have_been_submitted) {
    return std::nullopt;
  }

  if (matches(context.source)) {
    return context.source;
  }
  if (matches(context.target)) {
    return context.target;
  }
  return std::nullopt;
}

bool elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
  const ElevatorRuntimeCleanupContext & context,
  const ElevatorCleanupRuntimeFloorEvidence & evidence) noexcept
{
  if (
    context.transaction_id.empty() ||
    (context.disposition != ElevatorCleanupDisposition::kSourceOutside &&
    context.disposition != ElevatorCleanupDisposition::kTargetOutside) ||
    !evidence.confirmed || evidence.state != "ready" ||
    evidence.building_id.empty() || evidence.floor_id.empty() ||
    evidence.map_id.empty() || evidence.asset_epoch == 0U ||
    evidence.asset_digest.empty())
  {
    return false;
  }

  const auto matches = [&context, &evidence](
      const ElevatorRuntimeFloorIdentity & floor) {
      return evidence.building_id == context.building_id &&
             evidence.floor_id == floor.floor_id &&
             evidence.map_id == floor.map_id &&
             evidence.asset_epoch == floor.asset_epoch &&
             evidence.asset_digest == floor.asset_digest;
    };
  return !matches(context.source) && !matches(context.target);
}

bool elevator_cleanup_context_equal(
  const ElevatorRuntimeCleanupContext & left,
  const ElevatorRuntimeCleanupContext & right) noexcept
{
  return valid_confirmation_binding(left) &&
         valid_confirmation_binding(right) &&
         same_immutable_cleanup_identity(left, right) &&
         left.disposition == right.disposition &&
         left.legacy_preflight_orphan == right.legacy_preflight_orphan &&
         left.source_outside_confirmation_recorded ==
         right.source_outside_confirmation_recorded &&
         left.floor_switch_action_may_have_been_submitted ==
         right.floor_switch_action_may_have_been_submitted;
}

bool elevator_cleanup_context_rebind_allowed(
  const ElevatorRuntimeCleanupContext & bound,
  const ElevatorRuntimeCleanupContext & requested) noexcept
{
  if (elevator_cleanup_context_equal(bound, requested)) {
    return true;
  }
  return valid_confirmation_binding(bound) &&
         valid_confirmation_binding(requested) &&
         same_immutable_cleanup_identity(bound, requested) &&
         !bound.legacy_preflight_orphan &&
         !requested.legacy_preflight_orphan &&
         bound.floor_switch_action_may_have_been_submitted ==
         requested.floor_switch_action_may_have_been_submitted &&
         (bound.disposition == ElevatorCleanupDisposition::kRetainLock ||
         bound.disposition == ElevatorCleanupDisposition::kSourceOutside) &&
         requested.disposition == ElevatorCleanupDisposition::kSourceOutside &&
         !bound.source_outside_confirmation_recorded &&
         requested.source_outside_confirmation_recorded;
}

std::optional<ElevatorNavigationRequest>
resolve_elevator_navigation_request(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const robot_elevator_manager::ElevatorEffect & effect)
{
  const ElevatorRuntimeFloor * floor = nullptr;
  if (
    effect.floor_id == release.source.floor_id &&
    effect.map_id == release.source.map_id)
  {
    floor = &release.source;
  } else if (
    effect.floor_id == release.target.floor_id &&
    effect.map_id == release.target.map_id)
  {
    floor = &release.target;
  } else {
    return std::nullopt;
  }

  const auto * configured = find_pose_id(*floor, effect.pose_id);
  if (configured == nullptr) {
    return std::nullopt;
  }
  auto target = *configured;

  using Intent = robot_elevator_manager::ElevatorNavigationIntent;
  std::optional<double> heading;
  auto profile = ElevatorNavigationProfile::kElevatorScoped;
  switch (effect.navigation_intent) {
    case Intent::kHallCall:
      if (configured->role != PoseRole::kHallCall) {
        return std::nullopt;
      }
      profile = ElevatorNavigationProfile::kOrdinaryNav2;
      break;
    case Intent::kSourceLanding:
      if (
        release.schema_version != 2U ||
        floor != &release.source ||
        configured->role != PoseRole::kLanding)
      {
        return std::nullopt;
      }
      break;
    case Intent::kEnterCabin:
      if (
        release.schema_version != 2U ||
        floor != &release.source ||
        configured->role != PoseRole::kCabin)
      {
        return std::nullopt;
      }
      // Keep the cabin pose's commissioned final yaw. The scoped planner owns
      // the ingress transit and starts it from the live landing pose/yaw; the
      // landing-to-cabin chord must not overwrite either endpoint heading.
      profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      break;
    case Intent::kReverseEntryStaging:
      if (
        release.schema_version != 3U || floor != &release.source ||
        configured->role != PoseRole::kLanding)
      {
        return std::nullopt;
      }
      profile = ElevatorNavigationProfile::kElevatorReverseEntryStaging;
      break;
    case Intent::kReverseEnterCabin:
      if (
        release.schema_version != 3U || floor != &release.source ||
        configured->role != PoseRole::kCabin)
      {
        return std::nullopt;
      }
      profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      break;
    case Intent::kCabinPanelApproach:
      if (
        release.schema_version != 3U || floor != &release.source ||
        configured->role != PoseRole::kCabinPanel ||
        release.source.cabin_panel_side ==
        robot_elevator_manager::PanelSide::kUnknown)
      {
        return std::nullopt;
      }
      profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      break;
    case Intent::kReturnCabinCenter:
      if (
        release.schema_version != 3U || floor != &release.target ||
        configured->role != PoseRole::kCabin)
      {
        return std::nullopt;
      }
      profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      break;
    case Intent::kTargetLanding:
      if (
        floor != &release.target ||
        configured->role != PoseRole::kLanding)
      {
        return std::nullopt;
      }
      if (release.schema_version == 3U) {
        profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      } else if (release.schema_version == 2U) {
        heading = directed_yaw(
          find_pose(release.target, PoseRole::kCabin),
          find_pose(release.target, PoseRole::kLanding));
        profile = ElevatorNavigationProfile::kElevatorCabinDirect;
      } else {
        return std::nullopt;
      }
      break;
    case Intent::kNone:
      return std::nullopt;
  }
  if (
    !std::isfinite(target.x) || !std::isfinite(target.y) ||
    !std::isfinite(target.yaw))
  {
    return std::nullopt;
  }
  const bool requires_door_transit_heading =
    effect.navigation_intent == Intent::kTargetLanding &&
    release.schema_version == 2U;
  if (requires_door_transit_heading && !heading) {
    return std::nullopt;
  }
  if (heading) {
    target.yaw = *heading;
  }
  return ElevatorNavigationRequest{target, profile, effect.navigation_intent};
}

ElevatorNavigationProfile select_elevator_navigation_profile(
  const ElevatorNavigationRequest & request,
  const std::optional<ElevatorMapPose> & current_pose,
  const double scoped_max_distance_m) noexcept
{
  if (request.profile != ElevatorNavigationProfile::kOrdinaryNav2) {
    return request.profile;
  }
  if (
    request.target.role != PoseRole::kHallCall || !current_pose ||
    !std::isfinite(scoped_max_distance_m) || scoped_max_distance_m <= 0.0 ||
    !std::isfinite(current_pose->x) || !std::isfinite(current_pose->y) ||
    !std::isfinite(current_pose->yaw) ||
    !std::isfinite(request.target.x) || !std::isfinite(request.target.y) ||
    !std::isfinite(request.target.yaw))
  {
    return ElevatorNavigationProfile::kOrdinaryNav2;
  }

  const double distance = std::hypot(
    request.target.x - current_pose->x,
    request.target.y - current_pose->y);
  return std::isfinite(distance) && distance <= scoped_max_distance_m ?
         ElevatorNavigationProfile::kElevatorScoped :
         ElevatorNavigationProfile::kOrdinaryNav2;
}

std::optional<std::string> elevator_controller_id_for_profile(
  const ElevatorNavigationProfile profile,
  const PoseRole target_role)
{
  switch (profile) {
    case ElevatorNavigationProfile::kOrdinaryNav2:
      return std::nullopt;
    case ElevatorNavigationProfile::kElevatorScoped:
      return target_role == PoseRole::kHallCall ?
             std::optional<std::string>{"ElevatorHallFollowPath"} :
             std::optional<std::string>{"ElevatorFollowPath"};
    case ElevatorNavigationProfile::kElevatorReverseEntryStaging:
      return std::string{"ElevatorReverseEntryStagingFollowPath"};
    case ElevatorNavigationProfile::kElevatorReverseDocking:
      return std::string{"ElevatorReverseDockingFollowPath"};
    case ElevatorNavigationProfile::kElevatorCabinDirect:
      return std::string{"ElevatorCabinEntryDirectFollowPath"};
  }
  return std::nullopt;
}

std::optional<std::string> make_elevator_controller_session_id(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence)
{
  if (transaction_id.empty() || effect_sequence == 0U) {
    return std::nullopt;
  }
  return transaction_id + ":" + std::to_string(effect_sequence);
}

std::optional<robot_elevator_manager::ElevatorRuntimePose>
resolve_elevator_navigation_target(
  const robot_elevator_manager::FrozenElevatorRelease & release,
  const robot_elevator_manager::ElevatorEffect & effect)
{
  const auto request =
    resolve_elevator_navigation_request(release, effect);
  if (!request) {
    return std::nullopt;
  }
  return request->target;
}

void ElevatorTargetMapPoseTracker::reset(
  const double source_pose_stamp_sec) noexcept
{
  source_pose_stamp_sec_ =
    std::isfinite(source_pose_stamp_sec) && source_pose_stamp_sec > 0.0 ?
    source_pose_stamp_sec : 0.0;
  last_pose_stamp_sec_ = source_pose_stamp_sec_;
  stable_since_sec_ = -1.0;
  stable_anchor_ = ElevatorMapPose{};
  have_stable_anchor_ = false;
  ready_ = false;
  last_reason_ = "waiting for a target-map pose newer than the source-map baseline";
}

bool ElevatorTargetMapPoseTracker::observe(
  const ElevatorMapPose & pose,
  const double observed_at_sec,
  const double max_age_sec,
  const double settle_sec,
  const double translation_stability_m,
  const double yaw_stability_rad) noexcept
{
  const bool valid_options =
    std::isfinite(observed_at_sec) &&
    std::isfinite(max_age_sec) && max_age_sec > 0.0 &&
    std::isfinite(settle_sec) && settle_sec >= 0.0 &&
    std::isfinite(translation_stability_m) && translation_stability_m >= 0.0 &&
    std::isfinite(yaw_stability_rad) && yaw_stability_rad >= 0.0;
  const bool finite_pose =
    std::isfinite(pose.x) && std::isfinite(pose.y) &&
    std::isfinite(pose.yaw) && std::isfinite(pose.stamp_sec) &&
    std::isfinite(pose.age_sec);
  if (!valid_options || !finite_pose) {
    last_reason_ = "target map pose evidence is non-finite";
    return false;
  }
  if (pose.age_sec < 0.0 || pose.age_sec > max_age_sec) {
    last_reason_ = "target map pose evidence is stale";
    return false;
  }
  if (pose.stamp_sec <= source_pose_stamp_sec_ + 1.0e-9) {
    last_reason_ = "map pose did not advance beyond the source-floor TF baseline";
    return false;
  }
  if (pose.stamp_sec <= last_pose_stamp_sec_ + 1.0e-9) {
    last_reason_ = "waiting for a distinct target-map pose sample";
    return ready_;
  }

  last_pose_stamp_sec_ = pose.stamp_sec;
  if (!have_stable_anchor_) {
    stable_anchor_ = pose;
    stable_since_sec_ = observed_at_sec;
    have_stable_anchor_ = true;
    last_reason_ = "first fresh target-map pose observed; waiting for stability";
    return false;
  }

  const double translation_delta =
    std::hypot(pose.x - stable_anchor_.x, pose.y - stable_anchor_.y);
  const double yaw_delta = std::abs(std::atan2(
      std::sin(pose.yaw - stable_anchor_.yaw),
      std::cos(pose.yaw - stable_anchor_.yaw)));
  if (
    translation_delta > translation_stability_m ||
    yaw_delta > yaw_stability_rad)
  {
    stable_anchor_ = pose;
    stable_since_sec_ = observed_at_sec;
    ready_ = false;
    last_reason_ = "target map pose is still settling";
    return false;
  }

  ready_ =
    observed_at_sec >= stable_since_sec_ &&
    observed_at_sec - stable_since_sec_ >= settle_sec;
  last_reason_ = ready_ ?
    "fresh stable target-map robot pose proven" :
    "target map pose is fresh but has not met the settle duration";
  return ready_;
}

ElevatorMotionAdmissionScope motion_admission_scope_for_navigation(
  const robot_elevator_manager::ElevatorNavigationIntent intent) noexcept
{
  return intent == robot_elevator_manager::ElevatorNavigationIntent::kHallCall ?
         ElevatorMotionAdmissionScope::kHallApproach :
         ElevatorMotionAdmissionScope::kElevatorExecution;
}

bool elevator_navigation_bypasses_collision_monitor(
  const robot_elevator_manager::ElevatorNavigationIntent intent) noexcept
{
  using Intent = robot_elevator_manager::ElevatorNavigationIntent;
  return intent == Intent::kSourceLanding ||
         intent == Intent::kEnterCabin ||
         intent == Intent::kReverseEntryStaging ||
         intent == Intent::kReverseEnterCabin ||
         intent == Intent::kCabinPanelApproach ||
         intent == Intent::kReturnCabinCenter ||
         intent == Intent::kTargetLanding;
}

ElevatorMotionAdmissionAssessment assess_elevator_motion_admission(
  const ElevatorMotionAdmissionScope scope,
  const ElevatorMotionAdmissionEvidence & evidence,
  const std::uint64_t minimum_motion_allowed_generation,
  const std::uint64_t minimum_safety_status_generation,
  const std::uint64_t minimum_interlock_generation,
  const double now_sec,
  const double max_age_sec) noexcept
{
  const auto fresh_after =
    [now_sec, max_age_sec](
      const bool present,
      const std::uint64_t generation,
      const std::uint64_t minimum_generation,
      const double received_at_sec)
    {
      return
        present && generation > minimum_generation &&
        std::isfinite(now_sec) && std::isfinite(max_age_sec) &&
        std::isfinite(received_at_sec) && max_age_sec > 0.0 &&
        received_at_sec >= 0.0 && now_sec >= received_at_sec &&
        now_sec - received_at_sec <= max_age_sec;
    };

  const bool status_fresh = fresh_after(
    evidence.safety_status_present,
    evidence.safety_status_generation,
    minimum_safety_status_generation,
    evidence.safety_status_received_at_sec);
  const bool interlock_fresh = fresh_after(
    evidence.interlock_present,
    evidence.interlock_generation,
    minimum_interlock_generation,
    evidence.interlock_received_at_sec);

  const bool common_interlock_clear =
    !evidence.interlock_hold_active &&
    !evidence.interlock_motion_blocked &&
    !evidence.interlock_effective_motion_blocked;
  const bool requested_contract_clear =
    scope == ElevatorMotionAdmissionScope::kHallApproach ?
    common_interlock_clear &&
    !evidence.execution_session_engaged &&
    !evidence.execution_lease_active :
    common_interlock_clear && evidence.exact_elevator_mode_contract;
  const auto contract_block_reason = [&evidence, scope]() {
      if (evidence.interlock_hold_active) {
        return std::string{"motion_hold_active"};
      }
      if (evidence.interlock_motion_blocked) {
        return evidence.interlock_block_reason.empty() ?
               std::string{"motion_interlock_blocked"} :
               evidence.interlock_block_reason;
      }
      if (evidence.interlock_effective_motion_blocked) {
        return evidence.interlock_block_reason.empty() ?
               std::string{"effective_motion_interlock_blocked"} :
               evidence.interlock_block_reason;
      }
      if (
        scope == ElevatorMotionAdmissionScope::kHallApproach &&
        (evidence.execution_session_engaged || evidence.execution_lease_active))
      {
        return std::string{"execution_session_already_engaged"};
      }
      return std::string{"elevator_mode_contract_missing"};
    };

  if (interlock_fresh && !requested_contract_clear) {
    const auto reason = contract_block_reason();
    return {
      ElevatorMotionAdmissionKind::kHardBlocked,
      false,
      reason,
      scope == ElevatorMotionAdmissionScope::kHallApproach ?
      "owner hold release did not produce an ordinary hall-approach interlock state" :
      "owner hold release did not produce an exact elevator-mode interlock state",
    };
  }

  const bool motion_fresh = fresh_after(
    evidence.motion_allowed_present,
    evidence.motion_allowed_generation,
    minimum_motion_allowed_generation,
    evidence.motion_allowed_received_at_sec);
  if (motion_fresh && evidence.motion_allowed && interlock_fresh) {
    return {
      ElevatorMotionAdmissionKind::kAuthorized,
      true,
      "none",
      "robot_safety published a fresh motion_allowed=true sample after the "
      "requested interlock contract was proven",
    };
  }

  if (status_fresh) {
    const auto & status = evidence.safety_status;
    const bool hard_status =
      status == "ESTOP_ACTIVE" ||
      status == "MISSION_MOTION_HOLD" ||
      status == "EXECUTION_LEASE_MISSING" ||
      status == "EXECUTION_MODE_INVALID" ||
      status == "EXECUTION_SOURCE_BLOCKED" ||
      status == "LOCALIZATION_INVALID" ||
      status == "DOCKED_CONTACT_BLOCK";
    if (hard_status) {
      return {
        ElevatorMotionAdmissionKind::kHardBlocked,
        false,
        status,
        "robot_safety reports a non-watchdog safety blocker after hold release",
      };
    }
    if (
      status == "COMMAND_STALE" && interlock_fresh &&
      requested_contract_clear)
    {
      return {
        ElevatorMotionAdmissionKind::kCommandWarmup,
        true,
        "COMMAND_STALE",
        scope == ElevatorMotionAdmissionScope::kHallApproach ?
        "ordinary hall-approach contract is valid; COMMAND_STALE is expected "
        "until Nav2 publishes its first command" :
        "elevator mode contract is valid; COMMAND_STALE is expected "
        "until Nav2 publishes its first command",
      };
    }
  }

  return {
    ElevatorMotionAdmissionKind::kWaiting,
    false,
    status_fresh ? evidence.safety_status : "evidence_pending",
    "waiting for post-release safety status and interlock evidence",
  };
}

void DualOdomStopTracker::reset(
  const double linear_threshold_mps,
  const double angular_threshold_radps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  linear_threshold_mps_ =
    std::isfinite(linear_threshold_mps) && linear_threshold_mps >= 0.0 ?
    linear_threshold_mps : 0.0;
  angular_threshold_radps_ =
    std::isfinite(angular_threshold_radps) && angular_threshold_radps >= 0.0 ?
    angular_threshold_radps : 0.0;
  wheel_ = Channel{};
  local_ = Channel{};
}

void DualOdomStopTracker::observe_wheel(
  const double vx,
  const double vy,
  const double wz,
  const double received_at_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  observe(wheel_, vx, vy, wz, received_at_sec);
}

void DualOdomStopTracker::observe_local(
  const double vx,
  const double vy,
  const double wz,
  const double received_at_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  observe(local_, vx, vy, wz, received_at_sec);
}

void DualOdomStopTracker::observe(
  Channel & channel,
  const double vx,
  const double vy,
  const double wz,
  const double received_at_sec)
{
  if (
    !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
    !std::isfinite(received_at_sec))
  {
    channel = Channel{};
    return;
  }
  channel.seen = true;
  channel.received_at_sec = received_at_sec;
  channel.linear_speed_mps = std::hypot(vx, vy);
  channel.angular_speed_radps = std::abs(wz);
  if (
    channel.linear_speed_mps <= linear_threshold_mps_ &&
    channel.angular_speed_radps <= angular_threshold_radps_)
  {
    if (!channel.stable_since_sec) {
      channel.stable_since_sec = received_at_sec;
    }
  } else {
    channel.stable_since_sec.reset();
  }
}

bool DualOdomStopTracker::stopped(
  const double now_sec,
  const double max_age_sec,
  const double settle_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !std::isfinite(now_sec) || !std::isfinite(max_age_sec) ||
    !std::isfinite(settle_sec) ||
    max_age_sec <= 0.0 || settle_sec < 0.0)
  {
    return false;
  }
  const auto ready =
    [=](const Channel & channel) {
      return channel.seen &&
             channel.stable_since_sec.has_value() &&
             now_sec >= channel.received_at_sec &&
             now_sec - channel.received_at_sec <= max_age_sec &&
             channel.linear_speed_mps <= linear_threshold_mps_ &&
             channel.angular_speed_radps <= angular_threshold_radps_ &&
             now_sec >= *channel.stable_since_sec &&
             now_sec - *channel.stable_since_sec >= settle_sec;
    };
  return ready(wheel_) && ready(local_);
}

}  // namespace robot_api_server
