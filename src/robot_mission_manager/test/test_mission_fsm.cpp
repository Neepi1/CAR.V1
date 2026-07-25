#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_mission_manager/mission_fsm.hpp"

namespace
{

using robot_mission_manager::EffectCompletion;
using robot_mission_manager::EffectKind;
using robot_mission_manager::MissionFsm;
using robot_mission_manager::MissionRequest;
using robot_mission_manager::MissionState;

constexpr std::uint64_t kExpectedAssetEpoch = 7U;
const std::string kExpectedAssetDigest =
  "sha256:0123456789abcdef0123456789abcdef"
  "0123456789abcdef0123456789abcdef";

MissionRequest request(
  const std::string & mission_id,
  const std::string & source_floor,
  const std::string & target_floor,
  const std::string & target_pose)
{
  MissionRequest value;
  value.mission_id = mission_id;
  value.building_id = "building_1";
  value.expected_source_floor_id = source_floor;
  value.expected_source_map_id = source_floor + "_map";
  value.target_floor_id = target_floor;
  value.target_map_id = target_floor + "_map";
  value.target_pose_id = target_pose;
  value.preferred_elevator_id = "elevator_west";
  value.expected_asset_epoch = kExpectedAssetEpoch;
  value.expected_asset_digest = kExpectedAssetDigest;
  return value;
}

EffectCompletion elevator_success(
  const std::string & transaction_id,
  const std::string & floor_id = "F2",
  const std::string & map_id = "F2_map")
{
  EffectCompletion completion;
  completion.transaction_id = transaction_id;
  completion.success = true;
  completion.message = "elevator complete";
  completion.final_building_id = "building_1";
  completion.final_floor_id = floor_id;
  completion.final_map_id = map_id;
  completion.final_zone = "TARGET_HALL";
  completion.safety_hold_active = false;
  completion.final_asset_epoch = kExpectedAssetEpoch;
  completion.final_asset_digest = kExpectedAssetDigest;
  completion.explicit_relocalization_sequence = 11U;
  completion.runtime_context_valid = true;
  return completion;
}

TEST(MissionFsm, SameFloorMissionEmitsExactlyOneNavTarget)
{
  MissionFsm fsm("test-instance");

  const auto start = fsm.start(request("mission-1", "F1", "F1", "dropoff-7"));

  ASSERT_TRUE(start.accepted);
  ASSERT_TRUE(start.effect.has_value());
  EXPECT_EQ(start.effect->kind, EffectKind::kNavTarget);
  EXPECT_EQ(start.effect->mission_id, "mission-1");
  EXPECT_EQ(start.effect->building_id, "building_1");
  EXPECT_EQ(start.effect->target_floor_id, "F1");
  EXPECT_EQ(start.effect->target_map_id, "F1_map");
  EXPECT_EQ(start.effect->target_pose_id, "dropoff-7");
  EXPECT_EQ(start.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(start.effect->expected_asset_digest, kExpectedAssetDigest);
  EXPECT_NE(start.effect->transaction_id, "");
  EXPECT_EQ(fsm.state(), MissionState::kNavigatingToTarget);
  EXPECT_TRUE(fsm.has_active_effect());

  const auto duplicate = fsm.start(request("mission-2", "F1", "F1", "dropoff-8"));
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_FALSE(duplicate.effect.has_value());
  EXPECT_EQ(fsm.active_effect()->transaction_id, start.effect->transaction_id);

  const auto completed = fsm.complete_effect(
    EffectCompletion{start.effect->transaction_id, true, "goal reached"});
  EXPECT_TRUE(completed.accepted);
  EXPECT_FALSE(completed.effect.has_value());
  EXPECT_EQ(fsm.state(), MissionState::kSucceeded);
  EXPECT_FALSE(fsm.has_active_effect());
}

TEST(MissionFsm, CrossFloorMissionWaitsForElevatorBeforeNavTarget)
{
  MissionFsm fsm("test-instance");

  const auto start = fsm.start(request("mission-2", "F1", "F3", "room-301"));

  ASSERT_TRUE(start.accepted);
  ASSERT_TRUE(start.effect.has_value());
  EXPECT_EQ(start.effect->kind, EffectKind::kElevatorTask);
  EXPECT_EQ(start.effect->building_id, "building_1");
  EXPECT_EQ(start.effect->expected_source_floor_id, "F1");
  EXPECT_EQ(start.effect->expected_source_map_id, "F1_map");
  EXPECT_EQ(start.effect->target_floor_id, "F3");
  EXPECT_EQ(start.effect->target_map_id, "F3_map");
  EXPECT_EQ(start.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(start.effect->expected_asset_digest, kExpectedAssetDigest);
  EXPECT_EQ(start.effect->target_pose_id, "");
  EXPECT_EQ(start.effect->preferred_elevator_id, "elevator_west");
  EXPECT_EQ(fsm.state(), MissionState::kUsingElevator);

  const std::string elevator_transaction = start.effect->transaction_id;
  const auto elevator_done = fsm.complete_effect(
    elevator_success(elevator_transaction, "F3", "F3_map"));

  ASSERT_TRUE(elevator_done.accepted);
  ASSERT_TRUE(elevator_done.effect.has_value());
  EXPECT_EQ(elevator_done.effect->kind, EffectKind::kNavTarget);
  EXPECT_EQ(elevator_done.effect->target_floor_id, "F3");
  EXPECT_EQ(elevator_done.effect->target_map_id, "F3_map");
  EXPECT_EQ(elevator_done.effect->target_pose_id, "room-301");
  EXPECT_EQ(elevator_done.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(elevator_done.effect->expected_asset_digest, kExpectedAssetDigest);
  EXPECT_NE(elevator_done.effect->transaction_id, elevator_transaction);
  EXPECT_EQ(fsm.state(), MissionState::kNavigatingToTarget);
  EXPECT_TRUE(fsm.has_active_effect());
}

TEST(MissionFsm, FreezesTargetAssetIdentityAtMissionStart)
{
  MissionFsm fsm("test-instance");
  auto mutable_request =
    request("mission-frozen-identity", "F1", "F2", "room-201");

  const auto start = fsm.start(mutable_request);
  ASSERT_TRUE(start.accepted);
  ASSERT_TRUE(start.effect.has_value());

  mutable_request.expected_asset_epoch = kExpectedAssetEpoch + 1U;
  mutable_request.expected_asset_digest =
    "sha256:1123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";

  ASSERT_TRUE(fsm.active_effect().has_value());
  EXPECT_EQ(fsm.active_effect()->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(fsm.active_effect()->expected_asset_digest, kExpectedAssetDigest);

  const auto elevator_done = fsm.complete_effect(
    elevator_success(start.effect->transaction_id));

  ASSERT_TRUE(elevator_done.accepted);
  ASSERT_TRUE(elevator_done.effect.has_value());
  EXPECT_EQ(elevator_done.effect->kind, EffectKind::kNavTarget);
  EXPECT_EQ(elevator_done.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(elevator_done.effect->expected_asset_digest, kExpectedAssetDigest);
}

TEST(MissionFsm, MaintainsAtMostOneActiveEffectAcrossTransitions)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-3", "F1", "F2", "lab"));
  ASSERT_TRUE(start.effect.has_value());
  ASSERT_TRUE(fsm.active_effect().has_value());

  const auto elevator_done = fsm.complete_effect(
    elevator_success(start.effect->transaction_id));

  ASSERT_TRUE(elevator_done.effect.has_value());
  ASSERT_TRUE(fsm.active_effect().has_value());
  EXPECT_EQ(
    fsm.active_effect()->transaction_id,
    elevator_done.effect->transaction_id);
  EXPECT_NE(
    fsm.active_effect()->transaction_id,
    start.effect->transaction_id);
}

TEST(MissionFsm, FailedElevatorNeverEmitsFinalNavTarget)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-3b", "F1", "F2", "lab"));
  ASSERT_TRUE(start.effect.has_value());
  ASSERT_EQ(start.effect->kind, EffectKind::kElevatorTask);

  const auto failed = fsm.complete_effect(
    EffectCompletion{
      start.effect->transaction_id, false, "elevator transaction failed"});

  ASSERT_TRUE(failed.effect.has_value());
  EXPECT_EQ(failed.effect->kind, EffectKind::kHoldAndCancel);
  EXPECT_EQ(
    failed.effect->cancel_transaction_id,
    start.effect->transaction_id);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  EXPECT_NE(failed.effect->kind, EffectKind::kNavTarget);
}

TEST(MissionFsm, ElevatorSuccessRequiresExactFrozenTargetIdentityAndReleasedHold)
{
  std::vector<EffectCompletion> invalid_completions;
  auto wrong_building = elevator_success("placeholder");
  wrong_building.final_building_id = "building_2";
  invalid_completions.push_back(wrong_building);
  auto wrong_floor = elevator_success("placeholder");
  wrong_floor.final_floor_id = "F9";
  invalid_completions.push_back(wrong_floor);
  auto wrong_map = elevator_success("placeholder");
  wrong_map.final_map_id = "stale_map";
  invalid_completions.push_back(wrong_map);
  auto wrong_zone = elevator_success("placeholder");
  wrong_zone.final_zone = "CABIN";
  invalid_completions.push_back(wrong_zone);
  auto hold_active = elevator_success("placeholder");
  hold_active.safety_hold_active = true;
  invalid_completions.push_back(hold_active);
  auto no_epoch = elevator_success("placeholder");
  no_epoch.final_asset_epoch = 0U;
  invalid_completions.push_back(no_epoch);
  auto wrong_epoch = elevator_success("placeholder");
  wrong_epoch.final_asset_epoch = kExpectedAssetEpoch + 1U;
  invalid_completions.push_back(wrong_epoch);
  auto wrong_digest = elevator_success("placeholder");
  wrong_digest.final_asset_digest =
    "sha256:1123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  invalid_completions.push_back(wrong_digest);
  auto short_digest = elevator_success("placeholder");
  short_digest.final_asset_digest = "sha256:0123456789abcdef";
  invalid_completions.push_back(short_digest);
  auto uppercase_digest = elevator_success("placeholder");
  uppercase_digest.final_asset_digest =
    "sha256:0123456789ABCDEF0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  invalid_completions.push_back(uppercase_digest);
  auto no_relocalization = elevator_success("placeholder");
  no_relocalization.explicit_relocalization_sequence = 0U;
  invalid_completions.push_back(no_relocalization);
  auto stale_context = elevator_success("placeholder");
  stale_context.runtime_context_valid = false;
  invalid_completions.push_back(stale_context);
  auto residual_lease = elevator_success("placeholder");
  residual_lease.residual_execution_lease = true;
  invalid_completions.push_back(residual_lease);

  for (const auto & completion : invalid_completions) {
    MissionFsm fsm("test-instance");
    const auto start = fsm.start(request("mission-context", "F1", "F2", "lab"));
    ASSERT_TRUE(start.effect.has_value());
    auto candidate = completion;
    candidate.transaction_id = start.effect->transaction_id;

    const auto result = fsm.complete_effect(candidate);

    ASSERT_TRUE(result.effect.has_value());
    EXPECT_EQ(result.effect->kind, EffectKind::kHoldAndCancel);
    EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  }
}

TEST(MissionFsm, RejectsMissingOrNonCanonicalExpectedTargetIdentity)
{
  std::vector<MissionRequest> invalid_requests;

  auto zero_epoch = request("mission-zero-epoch", "F1", "F2", "lab");
  zero_epoch.expected_asset_epoch = 0U;
  invalid_requests.push_back(zero_epoch);

  auto empty_digest = request("mission-empty-digest", "F1", "F2", "lab");
  empty_digest.expected_asset_digest.clear();
  invalid_requests.push_back(empty_digest);

  auto short_digest = request("mission-short-digest", "F1", "F2", "lab");
  short_digest.expected_asset_digest = "sha256:0123456789abcdef";
  invalid_requests.push_back(short_digest);

  auto uppercase_digest = request("mission-uppercase-digest", "F1", "F1", "lab");
  uppercase_digest.expected_asset_digest =
    "sha256:0123456789ABCDEF0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  invalid_requests.push_back(uppercase_digest);

  for (const auto & invalid_request : invalid_requests) {
    MissionFsm fsm("test-instance");

    const auto result = fsm.start(invalid_request);

    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.effect.has_value());
    EXPECT_EQ(fsm.state(), MissionState::kIdle);
  }
}

TEST(MissionFsm, IgnoresEventsFromOldTransactions)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-4", "F1", "F2", "lab"));
  ASSERT_TRUE(start.effect.has_value());
  const auto elevator_done = fsm.complete_effect(
    elevator_success(start.effect->transaction_id));
  ASSERT_TRUE(elevator_done.effect.has_value());

  const auto stale = fsm.complete_effect(
    EffectCompletion{start.effect->transaction_id, false, "late abort"});

  EXPECT_FALSE(stale.accepted);
  EXPECT_TRUE(stale.ignored);
  EXPECT_FALSE(stale.effect.has_value());
  EXPECT_EQ(fsm.state(), MissionState::kNavigatingToTarget);
  ASSERT_TRUE(fsm.active_effect().has_value());
  EXPECT_EQ(
    fsm.active_effect()->transaction_id,
    elevator_done.effect->transaction_id);
}

TEST(MissionFsm, AbortFirstEmitsHoldAndCancelAndLocksFailure)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-5", "F1", "F2", "lab"));
  ASSERT_TRUE(start.effect.has_value());

  const auto aborted = fsm.abort("operator abort");

  ASSERT_TRUE(aborted.accepted);
  ASSERT_TRUE(aborted.effect.has_value());
  EXPECT_EQ(aborted.effect->kind, EffectKind::kHoldAndCancel);
  EXPECT_NE(aborted.effect->transaction_id, start.effect->transaction_id);
  EXPECT_EQ(
    aborted.effect->cancel_transaction_id,
    start.effect->transaction_id);
  EXPECT_EQ(aborted.effect->building_id, "building_1");
  EXPECT_EQ(aborted.effect->target_floor_id, "F2");
  EXPECT_EQ(aborted.effect->target_map_id, "F2_map");
  EXPECT_EQ(aborted.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(aborted.effect->expected_asset_digest, kExpectedAssetDigest);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  ASSERT_TRUE(fsm.active_effect().has_value());
  EXPECT_EQ(
    fsm.active_effect()->transaction_id,
    aborted.effect->transaction_id);

  const auto late_success = fsm.complete_effect(
    EffectCompletion{start.effect->transaction_id, true, "late success"});
  EXPECT_TRUE(late_success.ignored);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);

  const auto restart = fsm.start(request("mission-6", "F1", "F1", "home"));
  EXPECT_FALSE(restart.accepted);
  EXPECT_FALSE(restart.effect.has_value());
}

TEST(MissionFsm, CancelWithoutActiveEffectStillEmitsHoldAndCancelAndLocksFailure)
{
  MissionFsm fsm("test-instance");

  const auto cancelled = fsm.cancel("mission cancel");

  ASSERT_TRUE(cancelled.accepted);
  ASSERT_TRUE(cancelled.effect.has_value());
  EXPECT_EQ(cancelled.effect->kind, EffectKind::kHoldAndCancel);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  EXPECT_TRUE(fsm.has_active_effect());
}

TEST(MissionFsm, FailedEffectFirstEmitsHoldAndCancelAndLocksFailure)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-7", "F1", "F1", "dock"));
  ASSERT_TRUE(start.effect.has_value());

  const auto failed = fsm.complete_effect(
    EffectCompletion{start.effect->transaction_id, false, "Nav2 aborted"});

  ASSERT_TRUE(failed.accepted);
  ASSERT_TRUE(failed.effect.has_value());
  EXPECT_EQ(failed.effect->kind, EffectKind::kHoldAndCancel);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  EXPECT_EQ(fsm.failure_reason(), "Nav2 aborted");
}

TEST(MissionFsm, RejectsInvalidRequestsWithoutProducingAnEffect)
{
  MissionFsm fsm("test-instance");

  const auto missing_pose = fsm.start(request("mission-8", "F1", "F1", ""));

  EXPECT_FALSE(missing_pose.accepted);
  EXPECT_FALSE(missing_pose.effect.has_value());
  EXPECT_EQ(fsm.state(), MissionState::kIdle);
  EXPECT_FALSE(fsm.has_active_effect());
}

TEST(MissionFsm, RejectsUnsafeAssetIdentifiers)
{
  MissionFsm fsm("test-instance");
  auto unsafe = request("mission-9", "F1", "F2", "room-201");
  unsafe.target_map_id = "../F2";

  const auto result = fsm.start(unsafe);

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.effect.has_value());
  EXPECT_EQ(fsm.state(), MissionState::kIdle);

  unsafe = request("mission-10", "F1", "F2", "room-201");
  unsafe.building_id = ".";
  const auto dot = fsm.start(unsafe);
  EXPECT_FALSE(dot.accepted);
  EXPECT_EQ(fsm.state(), MissionState::kIdle);
}

TEST(MissionFsm, RejectsSameFloorMapChangeInsteadOfMisroutingToElevator)
{
  MissionFsm fsm("test-instance");
  auto map_change = request("mission-map-change", "F1", "F1", "room-201");
  map_change.target_map_id = "different_map";

  const auto result = fsm.start(map_change);

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.effect.has_value());
  EXPECT_EQ(fsm.state(), MissionState::kIdle);
}

TEST(MissionFsm, HoldAndCancelFailureRemainsActiveAndRetries)
{
  MissionFsm fsm("test-instance");
  const auto start = fsm.start(request("mission-retry", "F1", "F2", "lab"));
  ASSERT_TRUE(start.effect.has_value());
  const auto aborted = fsm.abort("operator abort");
  ASSERT_TRUE(aborted.effect.has_value());

  const auto retry = fsm.complete_effect(
    EffectCompletion{
      aborted.effect->transaction_id,
      false,
      "safety service unavailable",
    });

  ASSERT_TRUE(retry.accepted);
  ASSERT_TRUE(retry.effect.has_value());
  EXPECT_EQ(retry.effect->kind, EffectKind::kHoldAndCancel);
  EXPECT_NE(retry.effect->transaction_id, aborted.effect->transaction_id);
  EXPECT_FALSE(retry.effect->cleanup_id.empty());
  EXPECT_EQ(retry.effect->cleanup_id, aborted.effect->cleanup_id);
  EXPECT_EQ(retry.effect->expected_asset_epoch, kExpectedAssetEpoch);
  EXPECT_EQ(retry.effect->expected_asset_digest, kExpectedAssetDigest);
  EXPECT_EQ(fsm.state(), MissionState::kFailureLocked);
  EXPECT_TRUE(fsm.has_active_effect());
}

TEST(MissionFsm, InstanceIdFencesTransactionsAcrossProcessRuns)
{
  MissionFsm first("boot-a");
  MissionFsm second("boot-b");

  const auto first_effect =
    first.start(request("mission-stable", "F1", "F1", "dock"));
  const auto second_effect =
    second.start(request("mission-stable", "F1", "F1", "dock"));

  ASSERT_TRUE(first_effect.effect.has_value());
  ASSERT_TRUE(second_effect.effect.has_value());
  EXPECT_NE(
    first_effect.effect->transaction_id,
    second_effect.effect->transaction_id);
  EXPECT_LE(first_effect.effect->transaction_id.size(), 128U);
}

TEST(MissionFsm, RejectsOverlongInstanceAndMissionIdentifiers)
{
  EXPECT_THROW(MissionFsm(std::string(33U, 'a')), std::invalid_argument);

  MissionFsm fsm("test-instance");
  auto overlong = request(std::string(65U, 'm'), "F1", "F1", "dock");
  const auto result = fsm.start(overlong);

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.effect.has_value());
}

}  // namespace
