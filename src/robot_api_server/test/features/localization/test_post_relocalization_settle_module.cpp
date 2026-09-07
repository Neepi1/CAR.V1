#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"

namespace localization = robot_api_server::features::localization;
using namespace std::chrono_literals;

namespace
{

struct SettleHarness
{
  localization::BridgeStatusSnapshot bridge;
  localization::TfChainFreshnessSnapshot tf;
  std::chrono::steady_clock::time_point now{};
  std::uint64_t costmap_updates{4U};
  std::uint64_t costmap_drops{0U};
  std::string last_drop{"none"};
  bool static_lidar_ready{true};
  int clear_teleop_count{0};
  int teleop_zero_count{0};
  int navigation_zero_count{0};

  explicit SettleHarness(const std::uint64_t sequence = 7U)
  {
    bridge.available = true;
    bridge.map_to_odom_publisher_owner = "robot_localization_bridge";
    bridge.last_explicit_relocalization_sequence = sequence;
    bridge.publisher_decoupled_from_correction = true;
    bridge.has_map_to_odom = true;
    bridge.map_odom_state_valid = true;
    bridge.map_odom_publish_gap_ms = 10.0;
    bridge.map_odom_latest_accepted_sequence = sequence;
    bridge.map_odom_last_published_sequence = sequence;
    bridge.safe_for_goal_start = true;
    tf.have_map_to_odom = true;
    tf.have_odom_to_base = true;
    tf.have_map_pose = true;
    tf.map_to_odom_age_sec = 0.01;
    tf.odom_to_base_age_sec = 0.01;
    tf.map_pose_age_sec = 0.01;
  }

  localization::PostRelocalizationSettlePorts ports()
  {
    localization::PostRelocalizationSettlePorts result;
    result.bridge_status_snapshot = [this]() {return bridge;};
    result.tf_chain_freshness_snapshot = [this]() {return tf;};
    result.tf_chain_freshness_detail = [](
      const localization::TfChainFreshnessSnapshot &) {return "tf detail";};
    result.base_to_lidar_static_tf_ready = [this]() {return static_lidar_ready;};
    result.local_costmap_update_count = [this]() {return costmap_updates;};
    result.local_costmap_message_filter_drop_count = [this]() {return costmap_drops;};
    result.last_local_costmap_message_filter_drop_text = [this]() {return last_drop;};
    result.clear_teleop_command = [this]() {++clear_teleop_count;};
    result.publish_teleop_zero_burst = [this]() {++teleop_zero_count;};
    result.publish_navigation_zero_burst = [this]() {++navigation_zero_count;};
    result.steady_now = [this]() {return now;};
    result.wall_time_seconds = []() {return 123.5;};
    result.sleep_for = [this](const std::chrono::milliseconds duration) {now += duration;};
    return result;
  }
};

localization::PostRelocalizationSettleConfig passing_config()
{
  localization::PostRelocalizationSettleConfig config;
  config.min_ms = 0;
  config.max_ms = 100;
  config.stable_tf_samples = 1;
  config.tf_sample_period_ms = 10;
  config.required_local_costmap_updates = 0;
  config.large_correction_min_ms = 0;
  config.post_undock_min_ms = 0;
  config.post_undock_max_ms = 100;
  config.post_undock_stable_tf_samples = 1;
  config.post_undock_tf_sample_period_ms = 10;
  config.post_undock_required_local_costmap_updates = 0;
  return config;
}

}  // namespace

TEST(PostRelocalizationSettleModule, RejectsIncompleteIntegrationPorts)
{
  EXPECT_THROW(
    localization::PostRelocalizationSettleModule(
      passing_config(), localization::PostRelocalizationSettlePorts{}),
    std::invalid_argument);
}

TEST(PostRelocalizationSettleModule, PreservesSuccessfulBarrierAndStateEvidence)
{
  SettleHarness harness;
  localization::PostRelocalizationSettleModule module(
    passing_config(), harness.ports());

  const auto result = module.wait_for_settle(7U, "manual", "nav2_goal");

  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.failure_code, "NONE");
  EXPECT_EQ(result.observed_sequence, 7U);
  EXPECT_EQ(result.stable_samples, 1);
  EXPECT_NE(result.detail.find("post relocalization settle passed"), std::string::npos);
  EXPECT_EQ(harness.clear_teleop_count, 2);
  EXPECT_EQ(harness.teleop_zero_count, 2);
  EXPECT_EQ(harness.navigation_zero_count, 2);

  const auto state = module.state_snapshot();
  EXPECT_TRUE(state.required);
  EXPECT_FALSE(state.in_progress);
  EXPECT_TRUE(state.complete);
  EXPECT_EQ(state.reason, "manual");
  EXPECT_EQ(state.target_stage, "nav2_goal");
  EXPECT_DOUBLE_EQ(state.start_wall_time, 123.5);
  EXPECT_NE(module.state_json().find("\"expected_sequence\":7"), std::string::npos);
}

TEST(PostRelocalizationSettleModule, RejectsWrongMapOdomOwnerWithoutWaiting)
{
  SettleHarness harness;
  harness.bridge.map_to_odom_publisher_owner = "unexpected_owner";
  localization::PostRelocalizationSettleModule module(
    passing_config(), harness.ports());

  const auto result = module.wait_for_settle(7U, "manual", "nav2_goal");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure_code, "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER");
  EXPECT_EQ(harness.now.time_since_epoch(), 0ms);
  EXPECT_FALSE(module.state_snapshot().complete);
}

TEST(PostRelocalizationSettleModule, KeepsPostUndockCostmapShortfallWarningOnly)
{
  SettleHarness harness;
  auto config = passing_config();
  config.post_undock_max_ms = 0;
  config.post_undock_stable_tf_samples = 2;
  config.post_undock_required_local_costmap_updates = 2;
  localization::PostRelocalizationSettleModule module(config, harness.ports());

  const auto result = module.wait_for_settle(7U, "post_undock", "nav2_goal");

  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.failure_code, "NONE");
  EXPECT_NE(
    result.detail.find("post-undock warning only: local_costmap updates below required=2"),
    std::string::npos);
  EXPECT_NE(
    result.detail.find("releasing pending Nav2 goal"),
    std::string::npos);
}

TEST(PostRelocalizationSettleModule, PreservesExplicitCancellationCode)
{
  SettleHarness harness;
  localization::PostRelocalizationSettleModule module(
    passing_config(), harness.ports());

  const auto result = module.wait_for_settle(
    7U,
    "manual",
    "nav2_goal",
    [](std::string & detail) {
      detail = "cancel requested by test";
      return true;
    });

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.failure_code, "CANCELLED_BY_APP");
  EXPECT_EQ(result.detail, "cancel requested by test");
  EXPECT_EQ(module.state_snapshot().failure_reason, "CANCELLED_BY_APP");
}
