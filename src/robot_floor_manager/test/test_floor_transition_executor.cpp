#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_floor_manager/floor_transition_executor.hpp"

namespace robot_floor_manager
{
namespace
{

constexpr char kDigest[] =
  "sha256:0123456789abcdef0123456789abcdef"
  "0123456789abcdef0123456789abcdef";

FloorTransitionRequest request()
{
  return {
    "floor-live-1",
    "B11",
    "F2",
    "map-f2",
    42U,
    kDigest,
  };
}

FloorAssetSnapshot snapshot()
{
  FloorAssetSnapshot value;
  value.building_id = "B11";
  value.floor_id = "F2";
  value.map_id = "map-f2";
  value.asset_epoch = 42U;
  value.asset_digest = kDigest;
  return value;
}

class SuccessfulRuntime : public FloorTransitionRuntimePort
{
public:
  FloorTransitionEffectResult perform(
    const FloorTransitionEffect & effect,
    const FloorAssetSnapshot &) override
  {
    observed.push_back(effect.kind);
    FloorTransitionEffectResult result;
    result.success = true;
    auto & evidence = result.evidence;
    evidence.motion_hold_active = true;
    evidence.nav_idle = true;
    evidence.stopped = true;
    evidence.floor_pause_owned = true;
    evidence.correction_pause_effective = true;
    evidence.caller_pause_released = true;
    evidence.runtime_context_invalid = true;
    evidence.bridge_ready = true;
    evidence.amcl_ready = true;
    evidence.global_costmap_fresh = true;
    evidence.local_costmap_fresh = true;
    evidence.active_building_id = "B11";
    evidence.active_floor_id = "F2";
    evidence.active_map_id = "map-f2";
    evidence.asset_epoch = 42U;
    evidence.asset_digest = kDigest;
    evidence.explicit_relocalization_sequence = 9U;
    if (effect.kind == FloorTransitionEffectKind::kReleaseFloorPause) {
      evidence.floor_pause_owned = false;
      evidence.correction_pause_effective = false;
    }
    if (effect.kind == FloorTransitionEffectKind::kCommitRuntimeContext) {
      evidence.runtime_context_valid = true;
      evidence.safe_for_goal_start = true;
    }
    return result;
  }

  std::vector<FloorTransitionEffectKind> observed;
};

class FailureAfterBeginRuntime final : public FloorTransitionRuntimePort
{
public:
  explicit FailureAfterBeginRuntime(const bool restore_source = true)
  : restore_source_(restore_source)
  {
  }

  FloorTransitionEffectResult perform(
    const FloorTransitionEffect & effect,
    const FloorAssetSnapshot &) override
  {
    FloorTransitionEffectResult result;
    auto & evidence = result.evidence;
    evidence.motion_hold_active = true;
    evidence.nav_idle = true;
    evidence.stopped = true;
    evidence.floor_pause_owned = true;
    evidence.correction_pause_effective = true;
    evidence.caller_pause_released = true;
    evidence.runtime_context_invalid = bridge_begin_established;
    evidence.active_building_id = "B11";
    evidence.active_floor_id = "F2";
    evidence.active_map_id = "map-f2";
    evidence.asset_epoch = 42U;
    evidence.asset_digest = kDigest;

    if (effect.kind == FloorTransitionEffectKind::kInvalidateRuntimeContext) {
      bridge_begin_established = true;
      evidence.runtime_context_invalid = true;
    }
    if (effect.kind == FloorTransitionEffectKind::kLoadNavMap) {
      result.failure_code = "NAV_MAP_LOAD_FAILED";
      result.detail = "target Nav map load was rejected";
      return result;
    }
    if (effect.kind == FloorTransitionEffectKind::kHoldAndLock) {
      ++cleanup_count;
      bridge_abort_called = bridge_begin_established;
      hold_active = !restore_source_;
      evidence.runtime_context_invalid = !restore_source_;
      evidence.motion_hold_active = hold_active;
      result.success = bridge_abort_called;
      result.detail = restore_source_ ?
        "source context restored and exact leases released" :
        "motion hold retained because source restore was unproven";
      return result;
    }
    result.success = true;
    return result;
  }

  bool bridge_begin_established{false};
  bool bridge_abort_called{false};
  bool hold_active{false};
  std::size_t cleanup_count{0U};

private:
  bool restore_source_{true};
};

class FailureBeforeBeginRuntime final : public FloorTransitionRuntimePort
{
public:
  FloorTransitionEffectResult perform(
    const FloorTransitionEffect & effect,
    const FloorAssetSnapshot &) override
  {
    FloorTransitionEffectResult result;
    if (effect.kind == FloorTransitionEffectKind::kVerifyPreconditions) {
      result.failure_code = "NAV_IDLE_UNPROVEN";
      result.detail = "source Nav2 idle evidence was unavailable";
      return result;
    }
    if (effect.kind == FloorTransitionEffectKind::kHoldAndLock) {
      ++cleanup_count;
      result.success = true;
      result.detail =
        "pre-mutation resources released; source context remains valid";
      return result;
    }
    result.success = true;
    return result;
  }

  std::size_t cleanup_count{0U};
};

class CancelAfterCommitRuntime final : public SuccessfulRuntime
{
public:
  FloorTransitionEffectResult perform(
    const FloorTransitionEffect & effect,
    const FloorAssetSnapshot & snapshot) override
  {
    auto result = SuccessfulRuntime::perform(effect, snapshot);
    if (effect.kind == FloorTransitionEffectKind::kCommitRuntimeContext) {
      cancel_requested = true;
    }
    return result;
  }

  bool cancel_requested{false};
};

TEST(FloorTransitionExecutor, CommitsExactTargetAfterEveryRuntimeBarrier)
{
  SuccessfulRuntime runtime;
  FloorTransitionExecutor executor(runtime);

  const auto result = executor.run(request(), snapshot());

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.state, FloorTransitionState::kComplete);
  EXPECT_EQ(result.active_building_id, "B11");
  EXPECT_EQ(result.active_floor_id, "F2");
  EXPECT_EQ(result.active_map_id, "map-f2");
  EXPECT_EQ(result.asset_epoch, 42U);
  EXPECT_EQ(result.asset_digest, kDigest);
  EXPECT_EQ(result.explicit_relocalization_sequence, 9U);
  EXPECT_TRUE(result.runtime_context_valid);
  EXPECT_FALSE(result.recovery_required);
  ASSERT_FALSE(runtime.observed.empty());
  EXPECT_EQ(
    runtime.observed.back(),
    FloorTransitionEffectKind::kComplete);
}

TEST(FloorTransitionExecutor, FailureAfterBeginRestoresSourceWithoutRecoveryLock)
{
  FailureAfterBeginRuntime runtime;
  FloorTransitionExecutor executor(runtime);

  const auto result = executor.run(request(), snapshot());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, "NAV_MAP_LOAD_FAILED");
  EXPECT_EQ(result.state, FloorTransitionState::kFailed);
  EXPECT_TRUE(result.runtime_context_valid);
  EXPECT_FALSE(result.recovery_required);
  EXPECT_TRUE(runtime.bridge_begin_established);
  EXPECT_TRUE(runtime.bridge_abort_called);
  EXPECT_FALSE(runtime.hold_active);
  EXPECT_EQ(runtime.cleanup_count, 1U);
}

TEST(FloorTransitionExecutor, UnprovenPostBeginCleanupRetainsRecoveryLock)
{
  FailureAfterBeginRuntime runtime(false);
  FloorTransitionExecutor executor(runtime);

  const auto result = executor.run(request(), snapshot());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, "NAV_MAP_LOAD_FAILED");
  EXPECT_EQ(result.state, FloorTransitionState::kFailedLocked);
  EXPECT_FALSE(result.runtime_context_valid);
  EXPECT_TRUE(result.recovery_required);
  EXPECT_TRUE(runtime.hold_active);
}

TEST(FloorTransitionExecutor, FailureBeforeBeginIsNonLockingAndKeepsSourceContext)
{
  FailureBeforeBeginRuntime runtime;
  FloorTransitionExecutor executor(runtime);

  const auto result = executor.run(request(), snapshot());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, "NAV_IDLE_UNPROVEN");
  EXPECT_EQ(result.state, FloorTransitionState::kFailed);
  EXPECT_TRUE(result.runtime_context_valid);
  EXPECT_FALSE(result.recovery_required);
  EXPECT_EQ(runtime.cleanup_count, 1U);
}

TEST(FloorTransitionExecutor, ProjectsPreBeginFailureAsNonLockingTerminalStatus)
{
  FloorTransitionExecutionResult result;
  result.state = FloorTransitionState::kFailed;
  result.failure_code = "NAV_IDLE_UNPROVEN";
  result.runtime_context_valid = true;
  result.recovery_required = false;

  const auto disposition = floor_transition_failure_disposition(result);

  EXPECT_STREQ(disposition.state, "FAILED");
  EXPECT_STREQ(disposition.stage, "FAILED");
  EXPECT_FALSE(disposition.canceled);
  EXPECT_FALSE(disposition.recovery_locked);
}

TEST(FloorTransitionExecutor, ProjectsPostBeginFailureAsRecoveryLocked)
{
  FloorTransitionExecutionResult result;
  result.state = FloorTransitionState::kFailedLocked;
  result.failure_code = "NAV_MAP_LOAD_FAILED";
  result.runtime_context_valid = false;
  result.recovery_required = true;

  const auto disposition = floor_transition_failure_disposition(result);

  EXPECT_STREQ(disposition.state, "FAILED_LOCKED");
  EXPECT_STREQ(disposition.stage, "RECOVERY_LOCKED");
  EXPECT_FALSE(disposition.canceled);
  EXPECT_TRUE(disposition.recovery_locked);
}

TEST(FloorTransitionExecutor, ProjectsPreBeginCancellationAsCanceled)
{
  FloorTransitionExecutionResult result;
  result.state = FloorTransitionState::kFailed;
  result.failure_code = "CANCELLED";
  result.runtime_context_valid = true;
  result.recovery_required = false;

  const auto disposition = floor_transition_failure_disposition(result);

  EXPECT_STREQ(disposition.state, "CANCELED");
  EXPECT_STREQ(disposition.stage, "CANCELED");
  EXPECT_TRUE(disposition.canceled);
  EXPECT_FALSE(disposition.recovery_locked);
}

TEST(FloorTransitionExecutor, PublishesStablePauseHandoffStageAfterBegin)
{
  SuccessfulRuntime runtime;
  FloorTransitionExecutor executor(runtime);
  std::vector<std::string> stages;

  const auto result = executor.run(
    request(), snapshot(), {},
    [&stages](const FloorTransitionOutput & output) {
      stages.emplace_back(floor_transition_feedback_stage(output));
    });

  ASSERT_TRUE(result.success) << result.message;
  const auto handoff = std::find(
    stages.cbegin(), stages.cend(), "CALLER_PAUSE_HANDOFF_READY");
  ASSERT_NE(handoff, stages.cend());
  ASSERT_NE(handoff, stages.cbegin());
  ASSERT_NE(handoff + 1, stages.cend());
  EXPECT_EQ(*(handoff - 1), "INVALIDATE_RUNTIME_CONTEXT");
  EXPECT_EQ(*(handoff + 1), "VERIFY_PAUSE_HANDOFF");
}

TEST(FloorTransitionExecutor, KeepsPauseHandoffReadyWhileWaitingForCallerRelease)
{
  SuccessfulRuntime runtime;
  FloorTransitionExecutor executor(runtime);
  std::vector<std::pair<FloorTransitionState, bool>> handoff_states;

  const auto result = executor.run(
    request(), snapshot(), {},
    [&handoff_states](const FloorTransitionOutput & output) {
      if (
        output.state == FloorTransitionState::kReportingBeginReady ||
        output.state == FloorTransitionState::kVerifyingPauseHandoff)
      {
        handoff_states.emplace_back(
          output.state,
          floor_transition_caller_pause_handoff_ready(output));
      }
    });

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(handoff_states.size(), 2U);
  EXPECT_EQ(
    handoff_states[0].first,
    FloorTransitionState::kReportingBeginReady);
  EXPECT_TRUE(handoff_states[0].second);
  EXPECT_EQ(
    handoff_states[1].first,
    FloorTransitionState::kVerifyingPauseHandoff);
  EXPECT_TRUE(handoff_states[1].second);
}

TEST(FloorTransitionExecutor, RejectsSnapshotMismatchBeforeRuntimeSideEffects)
{
  SuccessfulRuntime runtime;
  FloorTransitionExecutor executor(runtime);
  auto stale = snapshot();
  stale.asset_epoch += 1U;

  const auto result = executor.run(request(), stale);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, "ASSET_IDENTITY_MISMATCH");
  EXPECT_TRUE(result.runtime_context_valid);
  EXPECT_FALSE(result.recovery_required);
  EXPECT_TRUE(runtime.observed.empty());
}

TEST(FloorTransitionExecutor, IgnoresLateCancelAfterDurableCommitBarrier)
{
  CancelAfterCommitRuntime runtime;
  FloorTransitionExecutor executor(runtime);

  const auto result = executor.run(
    request(), snapshot(),
    [&runtime]() {return runtime.cancel_requested;});

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.state, FloorTransitionState::kComplete);
  ASSERT_FALSE(runtime.observed.empty());
  EXPECT_EQ(runtime.observed.back(), FloorTransitionEffectKind::kComplete);
}

}  // namespace
}  // namespace robot_floor_manager
