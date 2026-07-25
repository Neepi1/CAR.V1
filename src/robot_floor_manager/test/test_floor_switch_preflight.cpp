#include <gtest/gtest.h>

#include "robot_floor_manager/floor_switch_preflight.hpp"

namespace robot_floor_manager
{
namespace
{

FloorSwitchPreflightInput valid_input()
{
  FloorSwitchPreflightInput input;
  input.request.transaction_id = "elevator-tx-17";
  input.request.building_id = "B10";
  input.request.floor_id = "F2";
  input.request.map_id = "map_f2";
  input.request.expected_asset_epoch = 17U;
  input.request.expected_asset_digest =
    "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  return input;
}

TEST(FloorSwitchPreflight, RejectsUnsafeIdentifiersBeforeAnyCapabilityCheck)
{
  auto input = valid_input();
  input.request.map_id = "../current";
  input.live_floor_switch_enabled = true;
  input.exact_asset_identity_proven = true;
  input.localizer_reload_proven = true;

  const auto decision = evaluate_floor_switch_preflight(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kInvalidGoal);
}

TEST(FloorSwitchPreflight, RequiresCanonicalSha256Digest)
{
  auto input = valid_input();
  input.request.expected_asset_digest = "sha256:asset-f2";

  const auto decision = evaluate_floor_switch_preflight(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kInvalidGoal);
}

TEST(FloorSwitchPreflight, RequiresNonzeroExpectedAssetEpoch)
{
  auto input = valid_input();
  input.request.expected_asset_epoch = 0U;
  input.live_floor_switch_enabled = true;
  input.exact_asset_identity_proven = true;
  input.localizer_reload_proven = true;

  const auto decision = evaluate_floor_switch_preflight(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kInvalidGoal);
  EXPECT_FALSE(decision.mutation_allowed);
}

TEST(FloorSwitchPreflight, ProductionDefaultFailsBeforeAssetMutation)
{
  auto input = valid_input();

  const auto decision = evaluate_floor_switch_preflight(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kLiveSwitchDisabled);
  EXPECT_FALSE(decision.mutation_allowed);
}

TEST(FloorSwitchPreflight, MissingAssetIdentityAndReloadProofRemainDistinct)
{
  auto input = valid_input();
  input.live_floor_switch_enabled = true;

  auto decision = evaluate_floor_switch_preflight(input);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kAssetBundleInvalid);
  EXPECT_FALSE(decision.mutation_allowed);

  input.exact_asset_identity_proven = true;
  decision = evaluate_floor_switch_preflight(input);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kLocalizerReloadUnproven);
  EXPECT_FALSE(decision.mutation_allowed);
}

TEST(FloorSwitchPreflight, ReadyMeansEveryRequiredCapabilityWasProven)
{
  auto input = valid_input();
  input.live_floor_switch_enabled = true;
  input.exact_asset_identity_proven = true;
  input.localizer_reload_proven = true;

  const auto decision = evaluate_floor_switch_preflight(input);

  EXPECT_TRUE(decision.ready);
  EXPECT_EQ(decision.failure_code, FloorSwitchFailureCode::kNone);
  EXPECT_FALSE(decision.mutation_allowed)
    << "the current preflight-only adapter must never grant mutation";
}

}  // namespace
}  // namespace robot_floor_manager
