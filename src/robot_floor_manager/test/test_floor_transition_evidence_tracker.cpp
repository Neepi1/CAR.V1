#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "robot_floor_manager/floor_transition_evidence_tracker.hpp"

namespace robot_floor_manager
{
namespace
{

constexpr char kDigest[] =
  "sha256:0123456789abcdef0123456789abcdef"
  "0123456789abcdef0123456789abcdef";

FloorTransitionTarget target()
{
  return {"floor-live-1", "B11", "F2", "map-f2", 42U, kDigest};
}

TEST(FloorTransitionEvidenceTracker, LatchesNavTerminalWhileLiveEvidenceExpires)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.begin(target(), 100.0);
  tracker.observe_nav_graph_ready(true, 99.0);
  tracker.observe_motion_interlock(
    true, {"robot_floor_manager:floor-live-1"}, 100.1);
  tracker.observe_nav_activity(false, 100.1);
  tracker.observe_wheel_odom(0.0, 0.0, 0.0, 100.1);
  tracker.observe_local_odom(0.0, 0.0, 0.0, 100.1);
  tracker.observe_wheel_odom(0.0, 0.0, 0.0, 100.5);
  tracker.observe_local_odom(0.0, 0.0, 0.0, 100.5);

  const auto proven = tracker.preconditions(
    100.5, 0.75, 0.25, 0.02, 0.02);

  EXPECT_TRUE(proven.motion_hold_active);
  EXPECT_TRUE(proven.nav_idle);
  EXPECT_TRUE(proven.stopped);

  const auto stale = tracker.preconditions(
    101.5, 0.75, 0.25, 0.02, 0.02);
  EXPECT_FALSE(stale.motion_hold_active);
  // NavigateToPose status is event-driven rather than a heartbeat. Once the
  // latest observed action state is terminal/idle it remains idle until a new
  // active goal is observed, even when operator confirmation takes longer
  // than the freshness window used by live odometry and hold evidence.
  EXPECT_TRUE(stale.nav_idle);
  EXPECT_FALSE(stale.stopped);
}

TEST(FloorTransitionEvidenceTracker, ProvesColdStartIdleAfterStableActionServerWithoutStatus)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.begin(target(), 100.0);
  tracker.observe_motion_interlock(
    true, {"robot_floor_manager:floor-live-1"}, 100.1);
  tracker.observe_wheel_odom(0.0, 0.0, 0.0, 100.1);
  tracker.observe_local_odom(0.0, 0.0, 0.0, 100.1);
  tracker.observe_wheel_odom(0.0, 0.0, 0.0, 100.5);
  tracker.observe_local_odom(0.0, 0.0, 0.0, 100.5);

  // A freshly started NavigateToPose server does not publish an empty status
  // array until its first goal.  Its stable availability must therefore be a
  // bounded cold-start idle proof, not a permanent unknown state.
  tracker.observe_nav_graph_ready(true, 99.0);
  auto evidence = tracker.preconditions(
    100.5, 0.75, 0.25, 0.02, 0.02, 2.0);
  EXPECT_FALSE(evidence.nav_idle);

  evidence = tracker.preconditions(
    101.1, 0.75, 0.25, 0.02, 0.02, 2.0);
  EXPECT_TRUE(evidence.nav_idle);

  // Any later active status overrides the cold-start proof immediately.
  tracker.observe_nav_activity(true, 101.2);
  evidence = tracker.preconditions(
    101.2, 0.75, 0.25, 0.02, 0.02, 2.0);
  EXPECT_FALSE(evidence.nav_idle);

  tracker.observe_nav_activity(false, 101.3);
  evidence = tracker.preconditions(
    101.3, 0.75, 0.25, 0.02, 0.02, 2.0);
  EXPECT_TRUE(evidence.nav_idle);

  // Losing the action server invalidates both terminal and bootstrap proof.
  tracker.observe_nav_graph_ready(false, 101.4);
  evidence = tracker.preconditions(
    101.4, 0.75, 0.25, 0.02, 0.02, 2.0);
  EXPECT_FALSE(evidence.nav_idle);
}

TEST(FloorTransitionEvidenceTracker, ProvesPauseHandoffOnlyWhenFloorLeaseIsExclusive)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.begin(target(), 100.0);
  tracker.observe_correction_pause(
    true,
    {
      "robot_elevator_manager:floor-live-1",
      "robot_floor_manager:floor-live-1",
    },
    100.1);

  auto evidence = tracker.pause_handoff(100.1, 0.75);
  EXPECT_TRUE(evidence.floor_pause_owned);
  EXPECT_TRUE(evidence.correction_pause_effective);
  EXPECT_FALSE(evidence.caller_pause_released);

  tracker.observe_correction_pause(
    true, {"robot_floor_manager:floor-live-1"}, 100.2);
  evidence = tracker.pause_handoff(100.2, 0.75);
  EXPECT_TRUE(evidence.floor_pause_owned);
  EXPECT_TRUE(evidence.correction_pause_effective);
  EXPECT_TRUE(evidence.caller_pause_released);
}

TEST(FloorTransitionEvidenceTracker, RejectsOldOrForeignLocalizationAndCostmaps)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.observe_localization_health(
    {"B11", "F1", "map-f1", 41U, kDigest, 7U, 11U, true, true, true, true},
    99.0);
  tracker.begin(target(), 100.0);

  tracker.observe_localization_health(
    {"B11", "F2", "map-f2", 42U, kDigest, 7U, 11U, true, true, true, false},
    100.1);
  auto evidence = tracker.target_localization(100.1, 0.75);
  EXPECT_FALSE(evidence.bridge_ready);
  EXPECT_EQ(evidence.explicit_relocalization_sequence, 0U);

  tracker.observe_localization_health(
    {"B11", "F2", "map-f2", 42U, kDigest, 8U, 12U, true, true, true, false,
      false, false},
    100.2);
  evidence = tracker.target_localization(100.2, 0.75);
  EXPECT_TRUE(evidence.bridge_ready);
  EXPECT_FALSE(evidence.amcl_ready);
  EXPECT_EQ(evidence.explicit_relocalization_sequence, 12U);

  tracker.observe_localization_health(
    {"B11", "F2", "map-f2", 42U, kDigest, 8U, 12U, true, true, true, false,
      false, true},
    100.3);
  evidence = tracker.target_localization(100.3, 0.75);
  EXPECT_TRUE(evidence.amcl_ready);

  tracker.observe_global_costmap(3U, 100.2);
  tracker.observe_local_costmap(4U, 100.2);
  tracker.mark_costmaps_cleared(100.3);
  tracker.observe_global_costmap(3U, 100.4);
  tracker.observe_local_costmap(5U, 100.4);
  evidence = tracker.fresh_costmaps(100.4, 0.75);
  EXPECT_FALSE(evidence.global_costmap_fresh);
  EXPECT_TRUE(evidence.local_costmap_fresh);

  tracker.observe_global_costmap(4U, 100.5);
  evidence = tracker.fresh_costmaps(100.5, 0.75);
  EXPECT_TRUE(evidence.global_costmap_fresh);
  EXPECT_TRUE(evidence.local_costmap_fresh);
}

TEST(FloorTransitionEvidenceTracker, RequiresExactTransactionLocalizerReloadProof)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.observe_localizer_asset(
    {
      "old-transaction", false, true, true, true,
      "B11", "F1", "map-f1", 41U, kDigest, 7U, true,
    },
    99.0);
  tracker.begin(target(), 100.0);
  tracker.observe_localizer_asset(
    {
      "foreign-transaction", false, true, true, true,
      "B11", "F2", "map-f2", 42U, kDigest, 8U, true,
    },
    100.1);
  auto evidence = tracker.target_localizer(8U, 100.1, 0.75);
  EXPECT_EQ(evidence.asset_epoch, 0U);

  tracker.observe_localizer_asset(
    {
      "floor-live-1", false, true, true, true,
      "B11", "F2", "map-f2", 42U, kDigest, 8U, true,
    },
    100.2);
  evidence = tracker.target_localizer(8U, 100.2, 0.75);
  EXPECT_EQ(evidence.asset_epoch, 42U);
  EXPECT_EQ(evidence.asset_digest, kDigest);
}

TEST(FloorTransitionEvidenceTracker, ReconcilesOnlyExactTerminalLocalizerApplyEvidence)
{
  FloorTransitionEvidenceTracker tracker;
  tracker.observe_localizer_asset(
    {
      "old-transaction", false, true, true, true,
      "B11", "F1", "map-f1", 41U, kDigest, 7U, true,
    },
    99.0);
  tracker.begin(target(), 100.0);

  LocalizerAssetEvidence applying;
  applying.transaction_id = "floor-live-1";
  applying.applying = true;
  applying.requested_building_id = "B11";
  applying.requested_floor_id = "F2";
  applying.requested_map_id = "map-f2";
  applying.requested_asset_epoch = 42U;
  applying.requested_asset_digest = kDigest;
  tracker.observe_localizer_asset(applying, 100.1);

  auto outcome = tracker.localizer_apply_outcome(7U, 100.1, 0.75);
  EXPECT_TRUE(outcome.exact_request);
  EXPECT_FALSE(outcome.terminal);

  auto foreign = applying;
  foreign.applying = false;
  foreign.success = true;
  foreign.reloaded = true;
  foreign.active_identity_valid = true;
  foreign.active_building_id = "B11";
  foreign.active_floor_id = "F2";
  foreign.active_map_id = "map-f2";
  foreign.active_asset_epoch = 42U;
  foreign.active_asset_digest = kDigest;
  foreign.localizer_generation = 8U;
  foreign.localizer_ready = true;
  foreign.requested_map_id = "other-map";
  tracker.observe_localizer_asset(foreign, 100.2);
  outcome = tracker.localizer_apply_outcome(7U, 100.2, 0.75);
  EXPECT_FALSE(outcome.exact_request);
  EXPECT_FALSE(outcome.terminal);

  auto stale_generation = foreign;
  stale_generation.requested_map_id = "map-f2";
  stale_generation.localizer_generation = 7U;
  tracker.observe_localizer_asset(stale_generation, 100.3);
  outcome = tracker.localizer_apply_outcome(7U, 100.3, 0.75);
  EXPECT_TRUE(outcome.exact_request);
  EXPECT_FALSE(outcome.terminal);

  auto succeeded = stale_generation;
  succeeded.localizer_generation = 8U;
  succeeded.code = "OK";
  succeeded.detail = "target localizer loaded";
  tracker.observe_localizer_asset(succeeded, 100.4);
  outcome = tracker.localizer_apply_outcome(7U, 100.4, 0.75);
  EXPECT_TRUE(outcome.exact_request);
  EXPECT_TRUE(outcome.terminal);
  EXPECT_TRUE(outcome.success);
  EXPECT_EQ(outcome.localizer_generation, 8U);
  EXPECT_EQ(outcome.code, "OK");

  auto failed = applying;
  failed.applying = false;
  failed.code = "LOCALIZER_LOAD_TIMEOUT";
  failed.detail = "target load failed and source was restored";
  tracker.observe_localizer_asset(failed, 100.5);
  outcome = tracker.localizer_apply_outcome(7U, 100.5, 0.75);
  EXPECT_TRUE(outcome.exact_request);
  EXPECT_TRUE(outcome.terminal);
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.code, "LOCALIZER_LOAD_TIMEOUT");
}

}  // namespace
}  // namespace robot_floor_manager
