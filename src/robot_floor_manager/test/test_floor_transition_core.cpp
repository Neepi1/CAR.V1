#include <gtest/gtest.h>

#include <vector>

#include "robot_floor_manager/floor_transition_core.hpp"

namespace robot_floor_manager
{
namespace
{

constexpr std::uint64_t kExpectedAssetEpoch = 17U;
constexpr char kExpectedAssetDigest[] =
  "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

FloorTransitionRequest request()
{
  return {
    "floor-tx-1",
    "building_1",
    "F2",
    "map_f2",
    kExpectedAssetEpoch,
    kExpectedAssetDigest,
  };
}

FloorTransitionEvent success(
  const FloorTransitionOutput & output,
  const FloorTransitionRequest & target)
{
  FloorTransitionEvent event;
  event.kind = FloorTransitionEventKind::kEffectSucceeded;
  event.transaction_id = output.effect.transaction_id;
  event.effect_sequence = output.effect.sequence;
  auto & evidence = event.evidence;
  evidence.motion_hold_active = true;
  evidence.nav_idle = true;
  evidence.stopped = true;
  evidence.floor_pause_owned = true;
  evidence.correction_pause_effective = true;
  evidence.caller_pause_released = true;
  evidence.runtime_context_invalid = true;
  evidence.bridge_ready = true;
  evidence.global_costmap_fresh = true;
  evidence.local_costmap_fresh = true;
  evidence.active_building_id = target.building_id;
  evidence.active_floor_id = target.floor_id;
  evidence.active_map_id = target.map_id;
  evidence.asset_digest = target.expected_asset_digest;
  evidence.asset_epoch = target.expected_asset_epoch;
  evidence.explicit_relocalization_sequence = 9U;

  if (output.effect.kind == FloorTransitionEffectKind::kReleaseFloorPause) {
    evidence.floor_pause_owned = false;
    evidence.correction_pause_effective = false;
  }
  if (output.effect.kind == FloorTransitionEffectKind::kCommitRuntimeContext) {
    evidence.runtime_context_valid = true;
    evidence.safe_for_goal_start = true;
  }
  return event;
}

TEST(FloorTransitionCore, RunsOrderedAtomicBarrierAndCommitsOnlyAtEnd)
{
  FloorTransitionCore core;
  const auto target = request();
  auto output = core.start(target);
  ASSERT_TRUE(output.accepted);

  const std::vector<FloorTransitionEffectKind> expected{
    FloorTransitionEffectKind::kVerifyPreconditions,
    FloorTransitionEffectKind::kAcquireCorrectionPause,
    FloorTransitionEffectKind::kInvalidateRuntimeContext,
    FloorTransitionEffectKind::kReportBeginReady,
    FloorTransitionEffectKind::kVerifyPauseHandoff,
    FloorTransitionEffectKind::kLoadNavMap,
    FloorTransitionEffectKind::kLoadFilters,
    FloorTransitionEffectKind::kReloadLocalizer,
    FloorTransitionEffectKind::kReleaseFloorPause,
    FloorTransitionEffectKind::kTriggerExplicitLocalization,
    FloorTransitionEffectKind::kVerifyBridgeReady,
    FloorTransitionEffectKind::kClearCostmaps,
    FloorTransitionEffectKind::kVerifyFreshCostmaps,
    FloorTransitionEffectKind::kCommitRuntimeContext,
    FloorTransitionEffectKind::kComplete,
  };

  for (const auto kind : expected) {
    ASSERT_EQ(output.effect.kind, kind);
    EXPECT_EQ(output.effect.expected_asset_epoch, target.expected_asset_epoch);
    EXPECT_EQ(output.effect.expected_asset_digest, target.expected_asset_digest);
    const auto previous_sequence = output.effect.sequence;
    output = core.dispatch(success(output, target));
    EXPECT_TRUE(output.accepted);
    if (output.effect.kind != FloorTransitionEffectKind::kNone) {
      EXPECT_GT(output.effect.sequence, previous_sequence);
    }
    if (kind == FloorTransitionEffectKind::kInvalidateRuntimeContext) {
      EXPECT_FALSE(output.runtime_context_valid);
    }
  }

  EXPECT_EQ(core.state(), FloorTransitionState::kComplete);
  EXPECT_TRUE(core.runtime_context_valid());
  EXPECT_FALSE(core.recovery_required());
}

TEST(FloorTransitionCore, RejectsZeroEpochAndNonCanonicalDigestBeforeStarting)
{
  FloorTransitionCore core;
  auto target = request();
  target.expected_asset_epoch = 0U;

  auto output = core.start(target);

  EXPECT_FALSE(output.accepted);
  EXPECT_EQ(output.state, FloorTransitionState::kIdle);
  EXPECT_FALSE(core.active_request().has_value());

  target.expected_asset_epoch = kExpectedAssetEpoch;
  target.expected_asset_digest = "sha256:asset-f2";
  output = core.start(target);

  EXPECT_FALSE(output.accepted);
  EXPECT_EQ(output.state, FloorTransitionState::kIdle);
  EXPECT_FALSE(core.active_request().has_value());
}

TEST(FloorTransitionCore, PreconditionsFailBeforeMutationWithoutClaimingRollback)
{
  FloorTransitionCore core;
  auto output = core.start(request());
  auto event = success(output, request());
  event.evidence.motion_hold_active = false;

  output = core.dispatch(event);

  EXPECT_TRUE(output.accepted);
  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  EXPECT_FALSE(output.recovery_required);
  EXPECT_TRUE(output.runtime_context_valid);
}

TEST(FloorTransitionCore, FailureAfterInvalidationLocksInvalidContext)
{
  FloorTransitionCore core;
  const auto target = request();
  auto output = core.start(target);
  output = core.dispatch(success(output, target));
  output = core.dispatch(success(output, target));
  ASSERT_EQ(
    output.effect.kind,
    FloorTransitionEffectKind::kInvalidateRuntimeContext);
  output = core.dispatch(success(output, target));
  ASSERT_FALSE(output.runtime_context_valid);

  FloorTransitionEvent failed;
  failed.kind = FloorTransitionEventKind::kEffectFailed;
  failed.transaction_id = output.effect.transaction_id;
  failed.effect_sequence = output.effect.sequence;
  failed.detail = "asset_load_failed";
  output = core.dispatch(failed);

  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  EXPECT_TRUE(output.recovery_required);
  EXPECT_FALSE(output.runtime_context_valid);
}

TEST(FloorTransitionCore, SameTransactionStartIsIdempotentAndForeignEventIgnored)
{
  FloorTransitionCore core;
  const auto first = core.start(request());
  const auto duplicate = core.start(request());

  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.ignored);
  EXPECT_EQ(duplicate.effect.sequence, first.effect.sequence);

  auto foreign = success(first, request());
  foreign.transaction_id = "old-floor-tx";
  const auto ignored = core.dispatch(foreign);
  EXPECT_FALSE(ignored.accepted);
  EXPECT_TRUE(ignored.ignored);
  EXPECT_EQ(core.state(), FloorTransitionState::kVerifyingPreconditions);
}

TEST(FloorTransitionCore, StaleAssetEvidenceFailsClosed)
{
  FloorTransitionCore core;
  const auto target = request();
  auto output = core.start(target);
  for (int index = 0; index < 5; ++index) {
    output = core.dispatch(success(output, target));
  }
  ASSERT_EQ(output.effect.kind, FloorTransitionEffectKind::kLoadNavMap);
  auto stale = success(output, target);
  stale.evidence.asset_digest = "sha256:old";

  output = core.dispatch(stale);

  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  EXPECT_TRUE(output.recovery_required);
}

TEST(FloorTransitionCore, DifferentNonzeroAssetEpochEvidenceFailsClosed)
{
  FloorTransitionCore core;
  const auto target = request();
  auto output = core.start(target);
  for (int index = 0; index < 5; ++index) {
    output = core.dispatch(success(output, target));
  }
  ASSERT_EQ(output.effect.kind, FloorTransitionEffectKind::kLoadNavMap);
  auto wrong_epoch = success(output, target);
  wrong_epoch.evidence.asset_epoch = target.expected_asset_epoch + 1U;

  output = core.dispatch(wrong_epoch);

  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  EXPECT_EQ(output.effect.expected_asset_epoch, target.expected_asset_epoch);
  EXPECT_TRUE(output.recovery_required);
}

TEST(FloorTransitionCore, SameTransactionWithDifferentEpochIsNotIdempotent)
{
  FloorTransitionCore core;
  const auto first = core.start(request());
  auto different_epoch = request();
  ++different_epoch.expected_asset_epoch;

  const auto conflict = core.start(different_epoch);

  EXPECT_TRUE(first.accepted);
  EXPECT_FALSE(conflict.accepted);
  EXPECT_FALSE(conflict.ignored);
}

TEST(FloorTransitionCore, FailedCleanupRetainsStableCleanupIdAndRetries)
{
  FloorTransitionCore core;
  auto output = core.start(request());
  FloorTransitionEvent failure;
  failure.kind = FloorTransitionEventKind::kEffectFailed;
  failure.transaction_id = output.effect.transaction_id;
  failure.effect_sequence = output.effect.sequence;
  output = core.dispatch(failure);
  ASSERT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  const auto cleanup_id = output.effect.cleanup_id;

  failure.effect_sequence = output.effect.sequence;
  output = core.dispatch(failure);

  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kHoldAndLock);
  EXPECT_EQ(output.effect.cleanup_id, cleanup_id);
  EXPECT_EQ(core.state(), FloorTransitionState::kFailureCleanup);

  output = core.dispatch(success(output, request()));
  EXPECT_EQ(core.state(), FloorTransitionState::kFailedLocked);
  EXPECT_EQ(output.effect.kind, FloorTransitionEffectKind::kNone);
}

}  // namespace
}  // namespace robot_floor_manager
