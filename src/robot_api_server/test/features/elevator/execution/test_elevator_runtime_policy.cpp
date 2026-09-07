#include <cmath>
#include <atomic>
#include <array>
#include <chrono>
#include <future>

#include <gtest/gtest.h>

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"

namespace
{

using robot_api_server::DualOdomStopTracker;
using robot_api_server::ElevatorCleanupRuntimeIdentityEvidence;
using robot_api_server::ElevatorCleanupRuntimeFloorEvidence;
using robot_api_server::ElevatorExecutionInterlockKind;
using robot_api_server::ElevatorMapPose;
using robot_api_server::ElevatorMotionAdmissionFence;
using robot_api_server::ElevatorMotionAdmissionEvidence;
using robot_api_server::ElevatorMotionAdmissionKind;
using robot_api_server::ElevatorMotionAdmissionScope;
using robot_api_server::ElevatorNavigationProfile;
using robot_api_server::ElevatorPreparedSourceEvidence;
using robot_api_server::ElevatorTargetMapPoseTracker;
using robot_api_server::RetainedRecoveryCancelResponse;
using robot_api_server::assess_elevator_cleanup_runtime_identity;
using robot_api_server::assess_elevator_motion_admission;
using robot_api_server::elevator_cleanup_context_equal;
using robot_api_server::elevator_cleanup_context_rebind_allowed;
using robot_api_server::elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime;
using robot_api_server::elevator_navigation_bypasses_collision_monitor;
using robot_api_server::elevator_controller_id_for_profile;
using robot_api_server::elevator_recovery_maintenance_peer_allowed;
using robot_api_server::evaluate_elevator_execution_interlock;
using robot_api_server::make_elevator_controller_session_id;
using robot_api_server::motion_admission_scope_for_navigation;
using robot_api_server::resolve_elevator_navigation_request;
using robot_api_server::resolve_elevator_cleanup_runtime_floor;
using robot_api_server::resolve_elevator_navigation_target;
using robot_api_server::select_elevator_navigation_profile;
using robot_api_server::validate_elevator_prepared_source;
using robot_elevator_manager::ElevatorEffect;
using robot_elevator_manager::ElevatorExecutionSnapshot;
using robot_elevator_manager::ElevatorNavigationIntent;
using robot_elevator_manager::ElevatorCleanupDisposition;
using robot_elevator_manager::ElevatorRuntimeCleanupContext;
using robot_elevator_manager::ElevatorRuntimeFloor;
using robot_elevator_manager::ElevatorRuntimePose;
using robot_elevator_manager::FrozenElevatorRelease;
using robot_elevator_manager::PoseRole;
using robot_elevator_manager::PanelSide;

ElevatorRuntimePose pose(
  const PoseRole role,
  const char * id,
  const double x,
  const double y,
  const double yaw)
{
  return ElevatorRuntimePose{role, id, x, y, yaw};
}

FrozenElevatorRelease release()
{
  FrozenElevatorRelease value;
  value.release_id = "elevator-config-000001-0123456789ab";
  value.generation = 1U;
  value.building_id = "B11";
  value.elevator_id = "elevator_1";
  value.source = ElevatorRuntimeFloor{
    "F1",
    "map_f1",
    6U,
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    {
      pose(PoseRole::kHallCall, "hall_f1", -2.0, 1.0, 0.25),
      pose(PoseRole::kLanding, "landing_f1", 0.0, 0.0, -1.5),
      pose(PoseRole::kCabin, "cabin_f1", 0.0, 2.0, 2.9),
    },
  };
  value.target = ElevatorRuntimeFloor{
    "F2",
    "map_f2",
    7U,
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
    {
      pose(PoseRole::kHallCall, "hall_f2", 10.0, 10.0, 0.0),
      pose(PoseRole::kLanding, "landing_f2", 2.0, 0.0, 0.0),
      pose(PoseRole::kCabin, "cabin_f2", 0.0, 0.0, 0.0),
    },
  };
  return value;
}

FrozenElevatorRelease reverse_entry_release()
{
  auto value = release();
  value.schema_version = 3U;
  value.source.poses = {
    pose(PoseRole::kHallCall, "hall_f1", -1.0, 0.0, 0.0),
    pose(PoseRole::kLanding, "landing_f1", 0.0, 0.0, 3.141592653589793),
    pose(PoseRole::kCabin, "cabin_f1", 1.0, 0.0, 3.141592653589793),
    pose(PoseRole::kCabinPanel, "panel_f1", 1.0, -0.5, 3.141592653589793),
  };
  value.source.hall_call_panel_side = PanelSide::kLeft;
  value.source.cabin_panel_side = PanelSide::kLeft;
  value.target.poses = {
    pose(PoseRole::kHallCall, "hall_f2", -1.0, 0.0, 0.0),
    pose(PoseRole::kLanding, "landing_f2", 0.0, 0.0, 3.141592653589793),
    pose(PoseRole::kCabin, "cabin_f2", 1.0, 0.0, 3.141592653589793),
    pose(PoseRole::kCabinPanel, "panel_f2", 1.0, 0.5, 3.141592653589793),
  };
  value.target.hall_call_panel_side = PanelSide::kRight;
  value.target.cabin_panel_side = PanelSide::kRight;
  return value;
}

ElevatorPreparedSourceEvidence ready_source()
{
  ElevatorPreparedSourceEvidence evidence;
  evidence.runtime_context_confirmed = true;
  evidence.runtime_context_state = "ready";
  evidence.building_id = "B11";
  evidence.floor_id = "F1";
  evidence.map_id = "map_f1";
  evidence.asset_epoch = 6U;
  evidence.asset_digest =
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  evidence.navigation_idle = true;
  evidence.mapping_idle = true;
  evidence.docking_idle = true;
  return evidence;
}

ElevatorRuntimeCleanupContext cleanup_context()
{
  ElevatorRuntimeCleanupContext context;
  context.transaction_id = "elevator-test-1";
  context.building_id = "B11";
  context.source = {
    "F1",
    "map_f1",
    6U,
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  };
  context.target = {
    "F2",
    "map_f2",
    7U,
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
  };
  return context;
}

TEST(
  ElevatorRuntimePolicy,
  TimedOutRecoveryCancelRetainsOneRequestAndConsumesItsLateResponse)
{
  using namespace std::chrono_literals;

  std::promise<int> response;
  auto response_future = response.get_future().share();
  RetainedRecoveryCancelResponse<int> pending;
  int submissions = 0;

  EXPECT_TRUE(
    pending.submit_if_absent(
      [&]() {
        ++submissions;
        return response_future;
      }));
  EXPECT_EQ(pending.wait_for(1ms), std::future_status::timeout);
  EXPECT_TRUE(pending.pending());

  EXPECT_FALSE(
    pending.submit_if_absent(
      [&]() {
        ++submissions;
        return response_future;
      }));
  EXPECT_EQ(submissions, 1);

  response.set_value(7);
  EXPECT_EQ(pending.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(pending.take_ready(), 7);
  EXPECT_FALSE(pending.pending());
}

TEST(ElevatorRuntimePolicy, CleanupContextExactReplayRemainsAllowed)
{
  const auto bound = cleanup_context();
  EXPECT_TRUE(elevator_cleanup_context_equal(bound, bound));
  EXPECT_TRUE(elevator_cleanup_context_rebind_allowed(bound, bound));

  auto changed_scope = bound;
  changed_scope.floor_switch_action_may_have_been_submitted =
    !bound.floor_switch_action_may_have_been_submitted;
  EXPECT_FALSE(elevator_cleanup_context_equal(bound, changed_scope));
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed_scope));
}

TEST(
  ElevatorRuntimePolicy,
  CleanupContextAllowsOnlyConfirmedRetainLockToSourceOutsidePromotion)
{
  const auto bound = cleanup_context();
  auto confirmed_source_outside = bound;
  confirmed_source_outside.disposition = ElevatorCleanupDisposition::kSourceOutside;
  confirmed_source_outside.source_outside_confirmation_recorded = true;

  EXPECT_FALSE(elevator_cleanup_context_equal(bound, confirmed_source_outside));
  EXPECT_TRUE(
    elevator_cleanup_context_rebind_allowed(bound, confirmed_source_outside));

  auto missing_confirmation = confirmed_source_outside;
  missing_confirmation.source_outside_confirmation_recorded = false;
  EXPECT_FALSE(
    elevator_cleanup_context_rebind_allowed(bound, missing_confirmation));

  EXPECT_FALSE(
    elevator_cleanup_context_rebind_allowed(confirmed_source_outside, bound));

  auto target_outside = confirmed_source_outside;
  target_outside.disposition = ElevatorCleanupDisposition::kTargetOutside;
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, target_outside));
}

TEST(
  ElevatorRuntimePolicy,
  PreFloorFailureMayReconcileToEitherFrozenEndpointAfterRuntimeRestart)
{
  auto context = cleanup_context();
  context.disposition = ElevatorCleanupDisposition::kSourceOutside;
  context.floor_switch_action_may_have_been_submitted = false;

  ElevatorCleanupRuntimeFloorEvidence current;
  current.confirmed = true;
  current.state = "ready";
  current.building_id = "B11";
  current.floor_id = "F2";
  current.map_id = "map_f2";
  current.asset_epoch = 7U;
  current.asset_digest =
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

  const auto resolved = resolve_elevator_cleanup_runtime_floor(context, current);
  ASSERT_TRUE(resolved.has_value());
  EXPECT_EQ(resolved->floor_id, "F2");
  EXPECT_EQ(resolved->map_id, "map_f2");
}

TEST(
  ElevatorRuntimePolicy,
  PossibleFloorSwitchSubmissionStillRequiresTheRecordedOutsideFloor)
{
  auto context = cleanup_context();
  context.disposition = ElevatorCleanupDisposition::kSourceOutside;
  context.floor_switch_action_may_have_been_submitted = true;

  ElevatorCleanupRuntimeFloorEvidence current;
  current.confirmed = true;
  current.state = "ready";
  current.building_id = "B11";
  current.floor_id = "F2";
  current.map_id = "map_f2";
  current.asset_epoch = 7U;
  current.asset_digest =
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

  EXPECT_FALSE(resolve_elevator_cleanup_runtime_floor(context, current).has_value());

  current.floor_id = "F1";
  current.map_id = "map_f1";
  current.asset_epoch = 6U;
  current.asset_digest =
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  const auto exact = resolve_elevator_cleanup_runtime_floor(context, current);
  ASSERT_TRUE(exact.has_value());
  EXPECT_EQ(exact->floor_id, "F1");
}

TEST(
  ElevatorRuntimePolicy,
  NonpersistentRestartCleanupAcceptsConfirmedUnrelatedRuntimeSupersession)
{
  auto context = cleanup_context();
  context.disposition = ElevatorCleanupDisposition::kSourceOutside;
  context.floor_switch_action_may_have_been_submitted = true;

  ElevatorCleanupRuntimeFloorEvidence current;
  current.confirmed = true;
  current.state = "ready";
  current.building_id = "B10";
  current.floor_id = "F1";
  current.map_id = "map_current_runtime";
  current.asset_epoch = 4U;
  current.asset_digest =
    "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

  // Ordinary live cleanup remains exact and must not accept this map.
  EXPECT_FALSE(resolve_elevator_cleanup_runtime_floor(context, current).has_value());
  // Restart-only, non-persistent cleanup may retire the stale transaction.
  EXPECT_TRUE(
    elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
      context, current));

  current.confirmed = false;
  EXPECT_FALSE(
    elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
      context, current));
  current.confirmed = true;
  current.asset_digest.clear();
  EXPECT_FALSE(
    elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
      context, current));
}

TEST(
  ElevatorRuntimePolicy,
  PreFloorRecoveryRejectsUnreadyOrUnrelatedRuntimeContext)
{
  auto context = cleanup_context();
  context.disposition = ElevatorCleanupDisposition::kSourceOutside;
  context.floor_switch_action_may_have_been_submitted = false;

  ElevatorCleanupRuntimeFloorEvidence current;
  current.confirmed = true;
  current.state = "ready";
  current.building_id = "B11";
  current.floor_id = "F3";
  current.map_id = "map_f3";
  current.asset_epoch = 8U;
  current.asset_digest =
    "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
  EXPECT_FALSE(resolve_elevator_cleanup_runtime_floor(context, current).has_value());

  current.floor_id = "F2";
  current.map_id = "map_f2";
  current.asset_epoch = 7U;
  current.asset_digest =
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
  current.confirmed = false;
  EXPECT_FALSE(resolve_elevator_cleanup_runtime_floor(context, current).has_value());
}

TEST(
  ElevatorRuntimePolicy,
  CleanupContextAllowsConfirmationOnAlreadyClassifiedSourceOutside)
{
  auto bound = cleanup_context();
  bound.disposition = ElevatorCleanupDisposition::kSourceOutside;

  auto confirmed = bound;
  confirmed.source_outside_confirmation_recorded = true;

  EXPECT_FALSE(elevator_cleanup_context_equal(bound, confirmed));
  EXPECT_TRUE(elevator_cleanup_context_rebind_allowed(bound, confirmed));
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(confirmed, bound));
}

TEST(ElevatorRuntimePolicy, CleanupContextPromotionRejectsIdentityOrLegacyDrift)
{
  const auto bound = cleanup_context();
  auto requested = bound;
  requested.disposition = ElevatorCleanupDisposition::kSourceOutside;
  requested.source_outside_confirmation_recorded = true;

  auto changed = requested;
  changed.transaction_id = "elevator-test-2";
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed));

  changed = requested;
  changed.building_id = "B12";
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed));

  changed = requested;
  changed.source.asset_epoch += 1U;
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed));

  changed = requested;
  changed.target.asset_digest =
    "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed));

  changed = requested;
  changed.legacy_preflight_orphan = true;
  EXPECT_FALSE(elevator_cleanup_context_rebind_allowed(bound, changed));
}

TEST(ElevatorRuntimePolicy, PreparedSourceRequiresExactReadyIdentityAndIdleRuntimes)
{
  const auto frozen = release();
  auto evidence = ready_source();

  EXPECT_TRUE(validate_elevator_prepared_source(frozen, evidence).success);

  evidence.asset_epoch += 1U;
  auto result = validate_elevator_prepared_source(frozen, evidence);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "ELEVATOR_SOURCE_ASSET_IDENTITY_MISMATCH");

  evidence = ready_source();
  evidence.navigation_idle = false;
  result = validate_elevator_prepared_source(frozen, evidence);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "ELEVATOR_SOURCE_RUNTIME_BUSY");
  EXPECT_EQ(
    result.detail,
    "navigation_idle=false;mapping_idle=true;docking_idle=true");
}

TEST(
  ElevatorRuntimePolicy,
  CleanupAcceptsStaleLatchedExactAssetWithFreshExactLiveEvidence)
{
  ElevatorCleanupRuntimeIdentityEvidence evidence;
  // A latched asset has no age field by contract, so an old receipt cannot
  // expire an unchanged exact identity.
  evidence.localizer_asset_present = true;
  evidence.localizer_asset_exact = true;
  evidence.localization_health_present = true;
  evidence.localization_health_exact = true;
  evidence.localization_health_received_at_sec = 99.5;
  evidence.localization_bridge_present = true;
  evidence.localization_bridge_exact = true;
  evidence.localization_bridge_received_at_sec = 99.75;

  const auto assessment =
    assess_elevator_cleanup_runtime_identity(evidence, 100.0, 2.0);

  EXPECT_TRUE(assessment.proven);
  EXPECT_TRUE(assessment.localization_health_fresh);
  EXPECT_TRUE(assessment.localization_bridge_fresh);
}

TEST(
  ElevatorRuntimePolicy,
  CleanupRejectsStaleOrMismatchedLiveHealthAndBridgeEvidence)
{
  ElevatorCleanupRuntimeIdentityEvidence evidence;
  evidence.localizer_asset_present = true;
  evidence.localizer_asset_exact = true;
  evidence.localization_health_present = true;
  evidence.localization_health_exact = true;
  evidence.localization_health_received_at_sec = 99.5;
  evidence.localization_bridge_present = true;
  evidence.localization_bridge_exact = true;
  evidence.localization_bridge_received_at_sec = 99.75;

  evidence.localization_health_received_at_sec = 97.0;
  auto assessment =
    assess_elevator_cleanup_runtime_identity(evidence, 100.0, 2.0);
  EXPECT_FALSE(assessment.proven);
  EXPECT_FALSE(assessment.localization_health_fresh);

  evidence.localization_health_received_at_sec = 99.5;
  evidence.localization_health_exact = false;
  assessment =
    assess_elevator_cleanup_runtime_identity(evidence, 100.0, 2.0);
  EXPECT_FALSE(assessment.proven);

  evidence.localization_health_exact = true;
  evidence.localization_bridge_exact = false;
  assessment =
    assess_elevator_cleanup_runtime_identity(evidence, 100.0, 2.0);
  EXPECT_FALSE(assessment.proven);

  evidence.localization_bridge_exact = true;
  evidence.localization_bridge_received_at_sec = 97.0;
  assessment =
    assess_elevator_cleanup_runtime_identity(evidence, 100.0, 2.0);
  EXPECT_FALSE(assessment.proven);
  EXPECT_FALSE(assessment.localization_bridge_fresh);
}

TEST(ElevatorRuntimePolicy, SourceLandingKeepsConfiguredHeading)
{
  const auto frozen = release();

  ElevatorEffect source_landing;
  source_landing.pose_id = "landing_f1";
  source_landing.floor_id = "F1";
  source_landing.map_id = "map_f1";
  source_landing.navigation_intent = ElevatorNavigationIntent::kSourceLanding;
  const auto source_target =
    resolve_elevator_navigation_target(frozen, source_landing);
  ASSERT_TRUE(source_target.has_value());
  EXPECT_NEAR(source_target->x, 0.0, 1.0e-9);
  EXPECT_NEAR(source_target->y, 0.0, 1.0e-9);
  EXPECT_NEAR(source_target->yaw, -1.5, 1.0e-9);
}

TEST(
  ElevatorRuntimePolicy,
  EnterCabinKeepsCommissionedCabinHeadingInsteadOfDoorChordOrLandingHeading)
{
  const auto frozen = release();

  ElevatorEffect enter_cabin;
  enter_cabin.pose_id = "cabin_f1";
  enter_cabin.floor_id = "F1";
  enter_cabin.map_id = "map_f1";
  enter_cabin.navigation_intent = ElevatorNavigationIntent::kEnterCabin;

  const auto request =
    resolve_elevator_navigation_request(frozen, enter_cabin);

  ASSERT_TRUE(request.has_value());
  EXPECT_NEAR(request->target.yaw, 2.9, 1.0e-9);
  EXPECT_EQ(
    request->profile,
    ElevatorNavigationProfile::kElevatorCabinDirect);
}

TEST(ElevatorRuntimePolicy, TargetLandingHeadingIsDerivedFromDoorGeometry)
{
  const auto frozen = release();

  ElevatorEffect target_landing;
  target_landing.pose_id = "landing_f2";
  target_landing.floor_id = "F2";
  target_landing.map_id = "map_f2";
  target_landing.navigation_intent = ElevatorNavigationIntent::kTargetLanding;
  const auto request =
    resolve_elevator_navigation_request(frozen, target_landing);
  ASSERT_TRUE(request.has_value());
  EXPECT_NEAR(request->target.yaw, 0.0, 1.0e-9);
  EXPECT_EQ(
    request->profile,
    ElevatorNavigationProfile::kElevatorCabinDirect);
}

TEST(ElevatorRuntimePolicy, V3ReverseEntryStagingUsesLateralFirstProfile)
{
  const auto frozen = reverse_entry_release();
  ElevatorEffect effect;
  effect.pose_id = "landing_f1";
  effect.floor_id = "F1";
  effect.map_id = "map_f1";
  effect.navigation_intent = ElevatorNavigationIntent::kReverseEntryStaging;

  const auto request = resolve_elevator_navigation_request(frozen, effect);

  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->target.role, PoseRole::kLanding);
  EXPECT_EQ(
    request->profile,
    ElevatorNavigationProfile::kElevatorReverseEntryStaging);
  EXPECT_NEAR(request->target.yaw, 3.141592653589793, 1.0e-9);
}

TEST(ElevatorRuntimePolicy, V3CabinTransitIntentsUseUncheckedDirectProfile)
{
  const auto frozen = reverse_entry_release();
  struct Case
  {
    const char * pose_id;
    const char * floor_id;
    const char * map_id;
    ElevatorNavigationIntent intent;
    PoseRole role;
  };
  const std::array<Case, 4> cases{{
    {"cabin_f1", "F1", "map_f1",
      ElevatorNavigationIntent::kReverseEnterCabin, PoseRole::kCabin},
    {"panel_f1", "F1", "map_f1",
      ElevatorNavigationIntent::kCabinPanelApproach, PoseRole::kCabinPanel},
    {"cabin_f2", "F2", "map_f2",
      ElevatorNavigationIntent::kReturnCabinCenter, PoseRole::kCabin},
    {"landing_f2", "F2", "map_f2",
      ElevatorNavigationIntent::kTargetLanding, PoseRole::kLanding},
  }};

  for (const auto & item : cases) {
    ElevatorEffect effect;
    effect.pose_id = item.pose_id;
    effect.floor_id = item.floor_id;
    effect.map_id = item.map_id;
    effect.navigation_intent = item.intent;
    const auto request =
      resolve_elevator_navigation_request(frozen, effect);
    ASSERT_TRUE(request.has_value()) << item.pose_id;
    EXPECT_EQ(request->target.role, item.role);
    EXPECT_EQ(
      request->profile,
      ElevatorNavigationProfile::kElevatorCabinDirect);
    EXPECT_NEAR(request->target.yaw, 3.141592653589793, 1.0e-9);
  }
}

TEST(ElevatorRuntimePolicy, V3IntentRejectsWrongRoleAndSchema)
{
  auto frozen = reverse_entry_release();
  ElevatorEffect effect;
  effect.pose_id = "cabin_f1";
  effect.floor_id = "F1";
  effect.map_id = "map_f1";
  effect.navigation_intent = ElevatorNavigationIntent::kCabinPanelApproach;
  EXPECT_FALSE(resolve_elevator_navigation_request(frozen, effect));

  effect.pose_id = "panel_f1";
  frozen.schema_version = 2U;
  EXPECT_FALSE(resolve_elevator_navigation_request(frozen, effect));
}

TEST(
  ElevatorRuntimePolicy,
  HallAndStagingKeepCommissionedProfilesWhileCabinTransitUsesDirectProfile)
{
  const auto frozen = release();

  ElevatorEffect hall_call;
  hall_call.pose_id = "hall_f1";
  hall_call.floor_id = "F1";
  hall_call.map_id = "map_f1";
  hall_call.navigation_intent = ElevatorNavigationIntent::kHallCall;
  const auto hall_request =
    resolve_elevator_navigation_request(frozen, hall_call);
  ASSERT_TRUE(hall_request.has_value());
  EXPECT_EQ(
    hall_request->profile,
    ElevatorNavigationProfile::kOrdinaryNav2);
  EXPECT_EQ(
    hall_request->navigation_intent,
    ElevatorNavigationIntent::kHallCall);

  ElevatorEffect source_landing;
  source_landing.pose_id = "landing_f1";
  source_landing.floor_id = "F1";
  source_landing.map_id = "map_f1";
  source_landing.navigation_intent =
    ElevatorNavigationIntent::kSourceLanding;
  const auto landing_request =
    resolve_elevator_navigation_request(frozen, source_landing);
  ASSERT_TRUE(landing_request.has_value());
  EXPECT_EQ(
    landing_request->profile,
    ElevatorNavigationProfile::kElevatorScoped);
  EXPECT_EQ(
    landing_request->navigation_intent,
    ElevatorNavigationIntent::kSourceLanding);

  ElevatorEffect enter_cabin;
  enter_cabin.pose_id = "cabin_f1";
  enter_cabin.floor_id = "F1";
  enter_cabin.map_id = "map_f1";
  enter_cabin.navigation_intent = ElevatorNavigationIntent::kEnterCabin;
  const auto cabin_request =
    resolve_elevator_navigation_request(frozen, enter_cabin);
  ASSERT_TRUE(cabin_request.has_value());
  EXPECT_EQ(
    cabin_request->profile,
    ElevatorNavigationProfile::kElevatorCabinDirect);

  ElevatorEffect target_landing;
  target_landing.pose_id = "landing_f2";
  target_landing.floor_id = "F2";
  target_landing.map_id = "map_f2";
  target_landing.navigation_intent =
    ElevatorNavigationIntent::kTargetLanding;
  const auto target_request =
    resolve_elevator_navigation_request(frozen, target_landing);
  ASSERT_TRUE(target_request.has_value());
  EXPECT_EQ(
    target_request->profile,
    ElevatorNavigationProfile::kElevatorCabinDirect);
}

TEST(
  ElevatorRuntimePolicy,
  NearbyHallCallSelectsScopedMotionInsteadOfBlockedStartupSpin)
{
  const auto frozen = release();
  ElevatorEffect hall_call;
  hall_call.pose_id = "hall_f1";
  hall_call.floor_id = "F1";
  hall_call.map_id = "map_f1";
  hall_call.navigation_intent = ElevatorNavigationIntent::kHallCall;
  const auto request =
    resolve_elevator_navigation_request(frozen, hall_call);

  ASSERT_TRUE(request.has_value());
  ASSERT_EQ(request->profile, ElevatorNavigationProfile::kOrdinaryNav2);

  // The target is one metre directly behind the robot with the same heading.
  // Ordinary RotationShim would try a near-pi startup spin; the elevator
  // profile can instead retain the heading and execute checked reverse motion.
  const ElevatorMapPose current{-1.0, 1.0, 0.25};
  EXPECT_EQ(
    select_elevator_navigation_profile(*request, current, 2.5),
    ElevatorNavigationProfile::kElevatorScoped);
}

TEST(ElevatorRuntimePolicy, FarHallCallKeepsOrdinaryNav2)
{
  const auto frozen = release();
  ElevatorEffect hall_call;
  hall_call.pose_id = "hall_f1";
  hall_call.floor_id = "F1";
  hall_call.map_id = "map_f1";
  hall_call.navigation_intent = ElevatorNavigationIntent::kHallCall;
  const auto request =
    resolve_elevator_navigation_request(frozen, hall_call);

  ASSERT_TRUE(request.has_value());
  const ElevatorMapPose current{1.0, 1.0, 0.25};
  EXPECT_EQ(
    select_elevator_navigation_profile(*request, current, 2.5),
    ElevatorNavigationProfile::kOrdinaryNav2);
}

TEST(ElevatorRuntimePolicy, MissingHallCallPoseKeepsOrdinaryNav2)
{
  const auto frozen = release();
  ElevatorEffect hall_call;
  hall_call.pose_id = "hall_f1";
  hall_call.floor_id = "F1";
  hall_call.map_id = "map_f1";
  hall_call.navigation_intent = ElevatorNavigationIntent::kHallCall;
  const auto request =
    resolve_elevator_navigation_request(frozen, hall_call);

  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(
    select_elevator_navigation_profile(*request, std::nullopt, 2.5),
    ElevatorNavigationProfile::kOrdinaryNav2);
}

TEST(ElevatorRuntimePolicy, DoorwayProfileRemainsScopedWithoutLivePose)
{
  const auto frozen = release();
  ElevatorEffect source_landing;
  source_landing.pose_id = "landing_f1";
  source_landing.floor_id = "F1";
  source_landing.map_id = "map_f1";
  source_landing.navigation_intent =
    ElevatorNavigationIntent::kSourceLanding;
  const auto request =
    resolve_elevator_navigation_request(frozen, source_landing);

  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(
    select_elevator_navigation_profile(*request, std::nullopt, 2.5),
    ElevatorNavigationProfile::kElevatorScoped);
}

TEST(ElevatorRuntimePolicy, ScopedProfilesResolveTheExactBehaviorTreeController)
{
  EXPECT_EQ(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kElevatorScoped, PoseRole::kHallCall),
    std::optional<std::string>{"ElevatorHallFollowPath"});
  EXPECT_EQ(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kElevatorScoped, PoseRole::kLanding),
    std::optional<std::string>{"ElevatorFollowPath"});
  EXPECT_EQ(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kElevatorReverseEntryStaging,
      PoseRole::kLanding),
    std::optional<std::string>{"ElevatorReverseEntryStagingFollowPath"});
  EXPECT_EQ(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kElevatorReverseDocking, PoseRole::kCabin),
    std::optional<std::string>{"ElevatorReverseDockingFollowPath"});
  EXPECT_EQ(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kElevatorCabinDirect, PoseRole::kCabin),
    std::optional<std::string>{"ElevatorCabinEntryDirectFollowPath"});
  EXPECT_FALSE(
    elevator_controller_id_for_profile(
      ElevatorNavigationProfile::kOrdinaryNav2, PoseRole::kHallCall)
    .has_value());
}

TEST(ElevatorRuntimePolicy, ControllerSessionIdentityUsesEffectNotGoalPose)
{
  EXPECT_EQ(
    make_elevator_controller_session_id("elevator-test-42", 17U),
    std::optional<std::string>{"elevator-test-42:17"});
  EXPECT_FALSE(make_elevator_controller_session_id("", 17U).has_value());
  EXPECT_FALSE(
    make_elevator_controller_session_id("elevator-test-42", 0U).has_value());
}

TEST(ElevatorRuntimePolicy, HallCallKeepsConfiguredHeadingAndRejectsWrongMap)
{
  const auto frozen = release();
  ElevatorEffect effect;
  effect.pose_id = "hall_f1";
  effect.floor_id = "F1";
  effect.map_id = "map_f1";
  effect.navigation_intent = ElevatorNavigationIntent::kHallCall;

  const auto target = resolve_elevator_navigation_target(frozen, effect);
  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->yaw, 0.25, 1.0e-9);

  effect.map_id = "map_f2";
  EXPECT_FALSE(resolve_elevator_navigation_target(frozen, effect).has_value());
}

TEST(
  ElevatorRuntimePolicy,
  DoorwayNavigationRejectsWrongRoleAndKeepsTheCommissionedTargetHeading)
{
  auto frozen = release();
  ElevatorEffect effect;
  effect.pose_id = "cabin_f1";
  effect.floor_id = "F1";
  effect.map_id = "map_f1";
  effect.navigation_intent = ElevatorNavigationIntent::kSourceLanding;
  EXPECT_FALSE(resolve_elevator_navigation_target(frozen, effect).has_value());

  effect.pose_id = "landing_f1";
  frozen.source.poses[2].x = frozen.source.poses[1].x;
  frozen.source.poses[2].y = frozen.source.poses[1].y;
  EXPECT_TRUE(resolve_elevator_navigation_target(frozen, effect).has_value());

  effect.pose_id = "cabin_f1";
  effect.navigation_intent = ElevatorNavigationIntent::kEnterCabin;
  const auto target = resolve_elevator_navigation_target(frozen, effect);
  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->yaw, 2.9, 1.0e-9);
}

TEST(ElevatorRuntimePolicy, TargetLandingTransitRejectsDegenerateGeometry)
{
  auto frozen = release();
  frozen.target.poses[1].x = frozen.target.poses[2].x;
  frozen.target.poses[1].y = frozen.target.poses[2].y;

  ElevatorEffect effect;
  effect.pose_id = "landing_f2";
  effect.floor_id = "F2";
  effect.map_id = "map_f2";
  effect.navigation_intent = ElevatorNavigationIntent::kTargetLanding;

  EXPECT_FALSE(resolve_elevator_navigation_target(frozen, effect).has_value());
}

TEST(ElevatorRuntimePolicy, StopProofRequiresFreshSettledWheelAndLocalOdom)
{
  DualOdomStopTracker tracker;
  tracker.reset(0.02, 0.03);
  tracker.observe_wheel(0.0, 0.0, 0.0, 10.0);
  tracker.observe_local(0.0, 0.0, 0.0, 10.0);

  EXPECT_FALSE(tracker.stopped(10.49, 0.20, 0.50));

  tracker.observe_wheel(0.0, 0.0, 0.0, 10.40);
  tracker.observe_local(0.0, 0.0, 0.0, 10.40);
  EXPECT_TRUE(tracker.stopped(10.51, 0.20, 0.50));

  tracker.observe_local(0.03, 0.0, 0.0, 10.52);
  EXPECT_FALSE(tracker.stopped(10.52, 0.20, 0.50));

  tracker.observe_local(0.0, 0.0, 0.0, 10.60);
  EXPECT_FALSE(tracker.stopped(11.11, 0.20, 0.50));
}

TEST(ElevatorRuntimePolicy, ResetStartsANewStopEvidenceEpoch)
{
  DualOdomStopTracker tracker;
  tracker.reset(0.02, 0.03);
  tracker.observe_wheel(0.0, 0.0, 0.0, 10.0);
  tracker.observe_local(0.0, 0.0, 0.0, 10.0);
  tracker.observe_wheel(0.0, 0.0, 0.0, 10.5);
  tracker.observe_local(0.0, 0.0, 0.0, 10.5);
  ASSERT_TRUE(tracker.stopped(10.51, 0.20, 0.50));

  tracker.reset(0.01, 0.02);
  tracker.observe_wheel(0.0, 0.0, 0.0, 10.6);
  tracker.observe_local(0.0, 0.0, 0.0, 10.6);
  EXPECT_FALSE(tracker.stopped(10.61, 0.20, 0.50));
}

TEST(ElevatorRuntimePolicy, ExecutionInterlockBlocksActiveAndRecoveryStates)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-1";
  snapshot.state = "NAVIGATING_TO_HALL_CALL";
  snapshot.terminal = false;
  snapshot.safety_hold_state_known = false;
  snapshot.safety_hold_active = false;
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kActive);

  snapshot.state = "FAILED";
  snapshot.terminal = true;
  snapshot.safety_hold_state_known = true;
  snapshot.safety_hold_active = true;
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kRecoveryRequired);

  snapshot.state = "LOCKED";
  snapshot.terminal = true;
  snapshot.safety_hold_state_known = true;
  snapshot.safety_hold_active = false;
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kRecoveryRequired);
}

TEST(ElevatorRuntimePolicy, ExecutionInterlockAllowsCleanTerminalAndMissingSnapshot)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-1";
  snapshot.state = "COMPLETE";
  snapshot.terminal = true;
  snapshot.safety_hold_state_known = true;
  snapshot.safety_hold_active = false;
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kClear);
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(std::nullopt).kind,
    ElevatorExecutionInterlockKind::kClear);
}

TEST(ElevatorRuntimePolicy, TerminalRuntimeEffectsRequireResourceReconciliation)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-runtime-effects";
  snapshot.state = "FAILED";
  snapshot.terminal = true;
  snapshot.runtime_applied = true;
  snapshot.safety_hold_state_known = true;
  snapshot.safety_hold_active = false;
  snapshot.runtime_resources_reconciled = false;

  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kRecoveryRequired);

  snapshot.runtime_resources_reconciled = true;
  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot).kind,
    ElevatorExecutionInterlockKind::kClear);
}

TEST(ElevatorRuntimePolicy, ExecutionInterlockBlocksUnreadableJournal)
{
  const auto interlock =
    evaluate_elevator_execution_interlock(std::nullopt, true);
  EXPECT_EQ(
    interlock.kind,
    ElevatorExecutionInterlockKind::kRecoveryRequired);
  EXPECT_EQ(interlock.transaction_id, "journal-unavailable");
}

TEST(ElevatorRuntimePolicy, ExecutionInterlockBlocksUnknownTerminalHoldState)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-unknown";
  snapshot.state = "FAILED";
  snapshot.terminal = true;
  snapshot.safety_hold_state_known = false;
  snapshot.safety_hold_active = false;

  const auto interlock = evaluate_elevator_execution_interlock(snapshot);

  EXPECT_EQ(
    interlock.kind,
    ElevatorExecutionInterlockKind::kRecoveryRequired);
  EXPECT_EQ(interlock.transaction_id, "tx-unknown");
}

TEST(ElevatorRuntimePolicy, NonlockingProductionPolicyAllowsEveryTerminalFailure)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-nonlocking";
  snapshot.state = "LOCKED";
  snapshot.terminal = true;
  snapshot.runtime_applied = true;
  snapshot.runtime_resources_reconciled = false;
  snapshot.safety_hold_state_known = false;
  snapshot.safety_hold_active = true;

  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot, true, false).kind,
    ElevatorExecutionInterlockKind::kClear);
}

TEST(ElevatorRuntimePolicy, NonlockingProductionPolicyDoesNotGloballyBlockOnCleanup)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.transaction_id = "tx-cleanup";
  snapshot.state = "FAILURE_CLEANUP";
  snapshot.phase = "AUTOMATIC_RESTART_RECOVERY";
  snapshot.terminal = false;
  snapshot.safety_hold_state_known = false;
  snapshot.safety_hold_active = true;

  EXPECT_EQ(
    evaluate_elevator_execution_interlock(snapshot, false, false).kind,
    ElevatorExecutionInterlockKind::kClear);
}

TEST(ElevatorRuntimePolicy, RecoveryWithoutApiTokenRequiresLoopbackMaintenancePeer)
{
  EXPECT_FALSE(elevator_recovery_maintenance_peer_allowed(false, false));
  EXPECT_TRUE(elevator_recovery_maintenance_peer_allowed(false, true));
}

TEST(ElevatorRuntimePolicy, ConfiguredApiTokenAllowsAuthenticatedRemoteRecovery)
{
  EXPECT_TRUE(elevator_recovery_maintenance_peer_allowed(true, false));
  EXPECT_TRUE(elevator_recovery_maintenance_peer_allowed(true, true));
}

TEST(ElevatorRuntimePolicy, RecoveryInvalidatesARequestWaitingAtTheMotionSubmissionSeam)
{
  using namespace std::chrono_literals;

  ElevatorMotionAdmissionFence fence;
  const auto old_epoch = fence.capture_epoch();
  std::atomic<bool> recovery_blocked{true};
  std::atomic<bool> submitted{false};

  auto recovery_guard = fence.invalidate_pending_and_lock();
  auto old_request = std::async(
    std::launch::async,
    [&]() {
      auto admission = fence.acquire_for_submission(
        old_epoch,
        [&]() {
          ElevatorExecutionSnapshot snapshot;
          snapshot.transaction_id = "tx-recovery";
          snapshot.state = recovery_blocked.load() ? "LOCKED" : "FAILED";
          snapshot.terminal = true;
          snapshot.safety_hold_state_known = true;
          snapshot.safety_hold_active = recovery_blocked.load();
          return evaluate_elevator_execution_interlock(snapshot);
        });
      if (admission.admitted()) {
        submitted.store(true);
      }
      return std::pair<bool, bool>{admission.admitted(), admission.stale()};
    });

  EXPECT_EQ(old_request.wait_for(50ms), std::future_status::timeout);
  recovery_blocked.store(false);
  recovery_guard.unlock();

  ASSERT_EQ(old_request.wait_for(1s), std::future_status::ready);
  const auto [admitted, stale] = old_request.get();
  EXPECT_FALSE(admitted);
  EXPECT_TRUE(stale);
  EXPECT_FALSE(submitted.load());

  fence.reopen_and_invalidate();
  const auto new_epoch = fence.capture_epoch();
  auto new_request = fence.acquire_for_submission(
    new_epoch,
    []() {return robot_api_server::ElevatorExecutionInterlock{};});
  EXPECT_TRUE(new_request.admitted());
  EXPECT_FALSE(new_request.stale());
}

TEST(ElevatorRuntimePolicy, ClosedFenceRejectsNewEpochUntilExplicitlyReopened)
{
  ElevatorMotionAdmissionFence fence;
  fence.close_and_invalidate();

  const auto closed_epoch = fence.capture_epoch();
  {
    auto rejected = fence.acquire_for_submission(
      closed_epoch,
      []() {return robot_api_server::ElevatorExecutionInterlock{};});
    EXPECT_FALSE(rejected.admitted());
    EXPECT_FALSE(rejected.stale());
  }

  fence.reopen_and_invalidate();
  const auto open_epoch = fence.capture_epoch();
  auto admitted = fence.acquire_for_submission(
    open_epoch,
    []() {return robot_api_server::ElevatorExecutionInterlock{};});
  EXPECT_TRUE(admitted.admitted());
  EXPECT_FALSE(admitted.stale());
}

TEST(ElevatorRuntimePolicy, UnknownDelayedSideEffectBlocksMotionAdmission)
{
  ElevatorMotionAdmissionFence fence;
  const auto epoch = fence.capture_epoch();

  auto admission = fence.acquire_for_submission(
    epoch,
    []() {
      return robot_api_server::ElevatorExecutionInterlock{
        ElevatorExecutionInterlockKind::kDelayedSideEffectUnknown,
        "",
        2U};
    });

  EXPECT_FALSE(admission.admitted());
  EXPECT_FALSE(admission.stale());
  EXPECT_TRUE(admission.interlock().delayed_side_effect_unknown());
  EXPECT_EQ(admission.interlock().delayed_side_effect_unknown_count, 2U);
}

TEST(
  ElevatorRuntimePolicy,
  TargetFloorPoseRequiresNewStableSamplesButNeverComparesCrossMapPanelCoordinates)
{
  ElevatorTargetMapPoseTracker tracker;
  tracker.reset(100.0);

  // A fresh sample from the source-map TF generation is not target-floor
  // evidence, even if its numeric pose happens to resemble a commissioned
  // waypoint on either map.
  EXPECT_FALSE(
    tracker.observe(
      ElevatorMapPose{3.741, -2.528, 3.075, 100.0, 0.01},
      10.00, 0.50, 0.20, 0.03, 0.03));

  // The target-map robot pose is intentionally nowhere near the target-map
  // cabin-panel coordinates. Readiness is about a new, fresh, stable live TF
  // generation; cabin_panel is not a cross-map localization anchor.
  EXPECT_FALSE(
    tracker.observe(
      ElevatorMapPose{-0.120, 0.610, -2.590, 101.0, 0.01},
      10.05, 0.50, 0.20, 0.03, 0.03));
  EXPECT_TRUE(
    tracker.observe(
      ElevatorMapPose{-0.112, 0.603, -2.584, 101.3, 0.01},
      10.27, 0.50, 0.20, 0.03, 0.03));
}

TEST(
  ElevatorRuntimePolicy,
  OnlyTheHallCallIntentUsesOrdinaryHallApproachAdmission)
{
  EXPECT_EQ(
    motion_admission_scope_for_navigation(ElevatorNavigationIntent::kHallCall),
    ElevatorMotionAdmissionScope::kHallApproach);

  for (const auto intent : {
      ElevatorNavigationIntent::kNone,
      ElevatorNavigationIntent::kSourceLanding,
      ElevatorNavigationIntent::kEnterCabin,
      ElevatorNavigationIntent::kReverseEntryStaging,
      ElevatorNavigationIntent::kReverseEnterCabin,
      ElevatorNavigationIntent::kCabinPanelApproach,
      ElevatorNavigationIntent::kReturnCabinCenter,
      ElevatorNavigationIntent::kTargetLanding})
  {
    EXPECT_EQ(
      motion_admission_scope_for_navigation(intent),
      ElevatorMotionAdmissionScope::kElevatorExecution);
  }
}

TEST(
  ElevatorRuntimePolicy,
  EveryPostHallCallNavigationBypassesCollisionMonitor)
{
  for (const auto intent : {
      ElevatorNavigationIntent::kSourceLanding,
      ElevatorNavigationIntent::kEnterCabin,
      ElevatorNavigationIntent::kReverseEntryStaging,
      ElevatorNavigationIntent::kReverseEnterCabin,
      ElevatorNavigationIntent::kCabinPanelApproach,
      ElevatorNavigationIntent::kReturnCabinCenter,
      ElevatorNavigationIntent::kTargetLanding})
  {
    EXPECT_TRUE(elevator_navigation_bypasses_collision_monitor(intent));
  }

  for (const auto intent : {
      ElevatorNavigationIntent::kNone,
      ElevatorNavigationIntent::kHallCall})
  {
    EXPECT_FALSE(elevator_navigation_bypasses_collision_monitor(intent));
  }
}

TEST(
  ElevatorRuntimePolicy,
  HallApproachWithoutExecutionSessionTreatsCommandStaleAsFirstCommandWarmup)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = false;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "COMMAND_STALE";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.interlock_hold_active = false;
  evidence.interlock_motion_blocked = false;
  evidence.interlock_effective_motion_blocked = false;
  evidence.execution_session_engaged = false;
  evidence.execution_lease_active = false;
  evidence.exact_elevator_mode_contract = false;

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kHallApproach,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kCommandWarmup);
  EXPECT_TRUE(assessment.ready_to_wait_for_nav2);
}

TEST(
  ElevatorRuntimePolicy,
  HallApproachRejectsARetainedOwnerHold)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = false;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "MISSION_MOTION_HOLD";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.interlock_hold_active = true;
  evidence.interlock_motion_blocked = true;
  evidence.interlock_effective_motion_blocked = true;
  evidence.interlock_block_reason = "motion_hold_active";

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kHallApproach,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kHardBlocked);
  EXPECT_FALSE(assessment.ready_to_wait_for_nav2);
  EXPECT_EQ(assessment.block_reason, "motion_hold_active");
}

TEST(
  ElevatorRuntimePolicy,
  HallApproachDoesNotBypassGlobalSafetyBlockers)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = false;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;

  for (const auto * status :
    {"ESTOP_ACTIVE", "LOCALIZATION_INVALID", "DOCKED_CONTACT_BLOCK"})
  {
    evidence.safety_status = status;
    const auto assessment = assess_elevator_motion_admission(
      ElevatorMotionAdmissionScope::kHallApproach,
      evidence, 7U, 11U, 4U, 20.15, 0.75);
    EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kHardBlocked)
      << status;
    EXPECT_FALSE(assessment.ready_to_wait_for_nav2) << status;
    EXPECT_EQ(assessment.block_reason, status);
  }
}

TEST(
  ElevatorRuntimePolicy,
  HallApproachRejectsARetainedExecutionSessionEvenWhenMotionAllowedIsTrue)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = true;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "OK";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.execution_session_engaged = true;
  evidence.execution_lease_active = true;
  evidence.exact_elevator_mode_contract = false;

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kHallApproach,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kHardBlocked);
  EXPECT_FALSE(assessment.ready_to_wait_for_nav2);
  EXPECT_EQ(assessment.block_reason, "execution_session_already_engaged");
}

TEST(
  ElevatorRuntimePolicy,
  ElevatorExecutionRejectsMissingExactModeContract)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = true;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "OK";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.execution_session_engaged = false;
  evidence.execution_lease_active = false;
  evidence.exact_elevator_mode_contract = false;

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kElevatorExecution,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kHardBlocked);
  EXPECT_FALSE(assessment.ready_to_wait_for_nav2);
  EXPECT_EQ(assessment.block_reason, "elevator_mode_contract_missing");
}

TEST(
  ElevatorRuntimePolicy,
  ExactModeContractWithoutExecutionLeaseAllowsFirstCommandWarmup)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = false;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "COMMAND_STALE";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.execution_session_engaged = false;
  evidence.execution_lease_active = false;
  evidence.exact_elevator_mode_contract = true;

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kElevatorExecution,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kCommandWarmup);
  EXPECT_TRUE(assessment.ready_to_wait_for_nav2);
}

TEST(
  ElevatorRuntimePolicy,
  RealSafetyBlockAfterHoldReleaseRemainsFailClosed)
{
  ElevatorMotionAdmissionEvidence evidence;
  evidence.motion_allowed_present = true;
  evidence.motion_allowed = false;
  evidence.motion_allowed_generation = 8U;
  evidence.motion_allowed_received_at_sec = 20.10;
  evidence.safety_status_present = true;
  evidence.safety_status = "LOCALIZATION_INVALID";
  evidence.safety_status_generation = 12U;
  evidence.safety_status_received_at_sec = 20.10;
  evidence.interlock_present = true;
  evidence.interlock_generation = 5U;
  evidence.interlock_received_at_sec = 20.10;
  evidence.execution_session_engaged = false;
  evidence.execution_lease_active = false;
  evidence.exact_elevator_mode_contract = true;

  const auto assessment = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kElevatorExecution,
    evidence, 7U, 11U, 4U, 20.15, 0.75);

  EXPECT_EQ(assessment.kind, ElevatorMotionAdmissionKind::kHardBlocked);
  EXPECT_FALSE(assessment.ready_to_wait_for_nav2);
  EXPECT_EQ(assessment.block_reason, "LOCALIZATION_INVALID");

  evidence.safety_status = "MISSION_MOTION_HOLD";
  const auto retained_hold = assess_elevator_motion_admission(
    ElevatorMotionAdmissionScope::kElevatorExecution,
    evidence, 7U, 11U, 4U, 20.15, 0.75);
  EXPECT_EQ(retained_hold.kind, ElevatorMotionAdmissionKind::kHardBlocked);
  EXPECT_FALSE(retained_hold.ready_to_wait_for_nav2);
  EXPECT_EQ(retained_hold.block_reason, "MISSION_MOTION_HOLD");
}

}  // namespace
