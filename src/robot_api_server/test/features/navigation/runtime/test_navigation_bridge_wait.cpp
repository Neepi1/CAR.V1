#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

navigation::BridgeReadinessSnapshot ready_bridge()
{
  navigation::BridgeReadinessSnapshot bridge;
  bridge.available = true;
  bridge.has_map_to_odom = true;
  bridge.map_to_odom_publisher_owner = "robot_localization_bridge";
  bridge.safe_for_goal_start = true;
  return bridge;
}

class FakeBridgeWaitRuntime final : public navigation::NavigationBridgeWaitRuntimePort
{
public:
  TimePoint bridge_wait_now() const override
  {
    return now;
  }

  void bridge_wait_sleep_for(const std::chrono::milliseconds duration) override
  {
    now += duration;
    ++sleep_calls;
  }

  bool bridge_wait_cancel_requested(
    const std::uint64_t job_id,
    std::string & detail) override
  {
    cancel_job_ids.push_back(job_id);
    detail = cancel_detail;
    return cancel;
  }

  void publish_bridge_wait_zero() override
  {
    ++zero_publish_count;
  }

  navigation::BridgeReadinessSnapshot bridge_wait_snapshot() override
  {
    ++snapshot_calls;
    if (snapshots.empty()) {
      return {};
    }
    const std::size_t index =
      std::min(snapshot_calls - 1U, snapshots.size() - 1U);
    return snapshots[index];
  }

  void request_bridge_wait_nomotion_update(
    const navigation::BridgeReadinessSnapshot & bridge,
    std::string & detail) override
  {
    ++nomotion_request_count;
    nomotion_snapshot = bridge;
    detail = nomotion_detail;
  }

  TimePoint now{};
  std::vector<navigation::BridgeReadinessSnapshot> snapshots;
  bool cancel{false};
  std::string cancel_detail{"cancel requested by app"};
  std::string nomotion_detail{"AMCL no-motion update requested for final_verify"};
  std::size_t snapshot_calls{0U};
  int sleep_calls{0};
  int zero_publish_count{0};
  int nomotion_request_count{0};
  std::vector<std::uint64_t> cancel_job_ids;
  navigation::BridgeReadinessSnapshot nomotion_snapshot;
};

navigation::NavigationBridgeWaitConfig enabled_config()
{
  navigation::NavigationBridgeWaitConfig config;
  config.final_verify_enabled = true;
  config.final_verify_wait_enabled = true;
  config.final_verify_timeout = std::chrono::milliseconds(200);
  config.final_verify_sample_period = std::chrono::milliseconds(100);
  config.final_yaw_wait_enabled = true;
  config.final_yaw_timeout = std::chrono::milliseconds(200);
  config.final_yaw_sample_period = std::chrono::milliseconds(100);
  return config;
}

}  // namespace

TEST(NavigationBridgeReadiness, PreservesGoalAndFinalVerifyDifference)
{
  auto bridge = ready_bridge();
  bridge.correction_active = true;

  const auto goal = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kGoalStart, "final yaw alignment");
  EXPECT_TRUE(goal.safe);
  EXPECT_EQ(goal.detail, "bridge safe_for_goal_start=true before final yaw alignment");

  const auto verify = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kFinalPoseVerify,
    "post-Nav2 final verification");
  EXPECT_FALSE(verify.safe);
  EXPECT_NE(verify.detail.find("correction_active=true"), std::string::npos);
}

TEST(NavigationBridgeReadiness, PreservesStaticStandbyTolerance)
{
  auto bridge = ready_bridge();
  bridge.localization_degraded = true;
  bridge.amcl_degraded_reason = "AMCL_NOT_TRACKING";
  bridge.amcl_input_enabled = true;
  bridge.amcl_seeded = true;
  bridge.amcl_static_standby = true;
  bridge.amcl_not_moving_no_update_ok = true;
  bridge.amcl_tracking_ready = true;
  bridge.amcl_correction_pending = false;

  const auto accepted = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kFinalPoseVerify, "verify");
  EXPECT_TRUE(accepted.safe);

  bridge.amcl_correction_pending = true;
  const auto rejected = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kFinalPoseVerify, "verify");
  EXPECT_FALSE(rejected.safe);
  EXPECT_EQ(rejected.detail, "LOCALIZATION_DEGRADED: AMCL_NOT_TRACKING");
}

TEST(NavigationBridgeReadiness, RejectsNoncanonicalOwnerAndCorrectionPause)
{
  auto bridge = ready_bridge();
  bridge.map_to_odom_publisher_owner = "unexpected_tf_owner";
  const auto wrong_owner = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kGoalStart, "navigation state");
  EXPECT_FALSE(wrong_owner.safe);
  EXPECT_EQ(
    wrong_owner.detail,
    "LOCALIZATION_DEGRADED: bridge map->odom not ready before navigation state");

  bridge.map_to_odom_publisher_owner = "robot_localization_bridge";
  bridge.map_odom_correction_paused = true;
  bridge.correction_pause_reason = "floor_switch";
  const auto paused = navigation::evaluate_bridge_readiness(
    bridge, navigation::BridgeReadinessPurpose::kGoalStart, "navigation state");
  EXPECT_FALSE(paused.safe);
  EXPECT_NE(paused.detail.find("LOCALIZATION_TRANSITION_ACTIVE"), std::string::npos);
  EXPECT_NE(paused.detail.find("pause_reason=floor_switch"), std::string::npos);
}

TEST(NavigationBridgeWait, DisabledFinalVerifyHasNoRuntimeSideEffects)
{
  auto config = enabled_config();
  config.final_verify_wait_enabled = false;
  navigation::NavigationBridgeWait wait(config);
  FakeBridgeWaitRuntime runtime;

  const auto result = wait.wait_before_final_verify(11U, runtime);

  EXPECT_FALSE(result.waited);
  EXPECT_FALSE(result.timeout);
  EXPECT_FALSE(result.canceled);
  EXPECT_EQ(result.detail, "post-Nav2 final verification bridge smoothing wait disabled");
  EXPECT_EQ(runtime.zero_publish_count, 0);
  EXPECT_EQ(runtime.snapshot_calls, 0U);
  EXPECT_EQ(runtime.nomotion_request_count, 0);
}

TEST(NavigationBridgeWait, FinalVerifyRequestsNomotionAndSettles)
{
  navigation::NavigationBridgeWait wait(enabled_config());
  FakeBridgeWaitRuntime runtime;
  auto smoothing = ready_bridge();
  smoothing.safe_for_goal_start = false;
  smoothing.remaining_translation_error_m = 0.12;
  runtime.snapshots = {smoothing, smoothing, ready_bridge()};

  const auto result = wait.wait_before_final_verify(22U, runtime);

  EXPECT_TRUE(result.waited);
  EXPECT_FALSE(result.timeout);
  EXPECT_FALSE(result.canceled);
  EXPECT_DOUBLE_EQ(result.elapsed_ms, 100.0);
  EXPECT_EQ(runtime.nomotion_request_count, 1);
  EXPECT_EQ(runtime.zero_publish_count, 3);
  EXPECT_EQ(runtime.sleep_calls, 1);
  EXPECT_NE(
    result.detail.find("bridge smoothing settled before post-Nav2 final pose verify"),
    std::string::npos);
  EXPECT_NE(result.detail.find(runtime.nomotion_detail), std::string::npos);
}

TEST(NavigationBridgeWait, FinalVerifyCancellationKeepsExistingOrdering)
{
  navigation::NavigationBridgeWait wait(enabled_config());
  FakeBridgeWaitRuntime runtime;
  runtime.snapshots = {ready_bridge()};
  runtime.cancel = true;

  const auto result = wait.wait_before_final_verify(33U, runtime);

  EXPECT_TRUE(result.waited);
  EXPECT_TRUE(result.canceled);
  EXPECT_FALSE(result.timeout);
  EXPECT_EQ(runtime.nomotion_request_count, 1);
  EXPECT_EQ(runtime.zero_publish_count, 1);
  ASSERT_EQ(runtime.cancel_job_ids.size(), 1U);
  EXPECT_EQ(runtime.cancel_job_ids.front(), 33U);
  EXPECT_NE(result.detail.find("cancel requested by app"), std::string::npos);
}

TEST(NavigationBridgeWait, FinalYawTimesOutWithoutNomotionRequest)
{
  navigation::NavigationBridgeWait wait(enabled_config());
  FakeBridgeWaitRuntime runtime;
  auto smoothing = ready_bridge();
  smoothing.safe_for_goal_start = false;
  smoothing.current_sequence = 7U;
  smoothing.target_sequence = 8U;
  runtime.snapshots = {smoothing};

  const auto result = wait.wait_before_final_yaw_align(44U, runtime);

  EXPECT_TRUE(result.waited);
  EXPECT_TRUE(result.timeout);
  EXPECT_FALSE(result.canceled);
  EXPECT_DOUBLE_EQ(result.elapsed_ms, 300.0);
  EXPECT_EQ(runtime.nomotion_request_count, 0);
  EXPECT_EQ(runtime.zero_publish_count, 3);
  EXPECT_NE(result.detail.find("timeout_ms=200"), std::string::npos);
  EXPECT_NE(result.detail.find("current_sequence=7"), std::string::npos);
}
