#include <gtest/gtest.h>

#include "robot_global_localization/post_reload_readiness_gate.hpp"

namespace robot_global_localization
{
namespace
{

TEST(PostReloadReadinessGate, RequiresPostReloadInputAndStableServiceAfterSettle)
{
  PostReloadReadinessConfig config;
  config.minimum_settle_sec = 1.0;
  config.timeout_sec = 5.0;
  config.input_max_age_sec = 0.5;
  config.required_consecutive_service_ready_samples = 3U;
  PostReloadReadinessGate gate(config);

  gate.arm(7U, 100U, 10.0);

  EXPECT_EQ(
    gate.observe({10.2, true, 101U, 0.01}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({10.8, false, 102U, 0.01}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({11.0, true, 102U, 0.01}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({11.1, true, 103U, 0.01}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({11.2, true, 104U, 0.01}),
    PostReloadReadinessDecision::kReady);
  EXPECT_FALSE(gate.pending());
  EXPECT_EQ(gate.localizer_generation(), 7U);
}

TEST(PostReloadReadinessGate, RejectsPreReloadOrStaleInputAndReportsTimeout)
{
  PostReloadReadinessConfig config;
  config.minimum_settle_sec = 0.5;
  config.timeout_sec = 2.0;
  config.input_max_age_sec = 0.25;
  config.required_consecutive_service_ready_samples = 2U;
  PostReloadReadinessGate gate(config);

  gate.arm(8U, 200U, 20.0);

  EXPECT_EQ(
    gate.observe({20.6, true, 200U, 0.01}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({20.7, true, 201U, 0.30}),
    PostReloadReadinessDecision::kWait);
  EXPECT_EQ(
    gate.observe({22.0, true, 201U, 0.30}),
    PostReloadReadinessDecision::kTimedOut);
  EXPECT_TRUE(gate.pending());
}

TEST(PostReloadReadinessGate, DoesNotDelayTriggersWhenNoReloadIsPending)
{
  PostReloadReadinessGate gate;

  EXPECT_EQ(
    gate.observe({1.0, false, 0U, -1.0}),
    PostReloadReadinessDecision::kReady);
  EXPECT_FALSE(gate.pending());
}

}  // namespace
}  // namespace robot_global_localization
