#include "gtest/gtest.h"
#include "robot_localization_bridge/post_isaac_refine_gate.hpp"

namespace bridge = robot_localization_bridge;

namespace
{

bridge::PostIsaacRefineGateInput ready_input()
{
  bridge::PostIsaacRefineGateInput input;
  input.enabled = true;
  input.gated_mode = true;
  input.explicit_target_settled = true;
  input.explicit_sequence = 4U;
  input.now_sec = 100.50;
  input.refine_reference_sec = 100.0;
  input.pose_stamp_sec = 100.40;
  input.received_sec = 100.50;
  input.window_sec = 10.0;
  input.min_delay_sec = 0.25;
  return input;
}

}  // namespace

TEST(PostIsaacRefineGate, RejectsQueuedPreseedPoseEvenWhenReceivedAfterSeed)
{
  auto input = ready_input();
  input.pose_stamp_sec = 99.95;
  input.received_sec = 100.001;

  EXPECT_EQ(
    bridge::evaluate_post_isaac_refine_gate(input),
    bridge::PostIsaacRefineDisposition::kPreseedPose);
}

TEST(PostIsaacRefineGate, WaitsUntilIsaacTargetHasFinishedSmoothing)
{
  auto input = ready_input();
  input.explicit_target_settled = false;

  EXPECT_EQ(
    bridge::evaluate_post_isaac_refine_gate(input),
    bridge::PostIsaacRefineDisposition::kWaitingForIsaacSettle);
}

TEST(PostIsaacRefineGate, RequiresARealPostSeedProcessingWindow)
{
  auto input = ready_input();
  input.now_sec = 100.10;
  input.pose_stamp_sec = 100.08;
  input.received_sec = 100.10;

  EXPECT_EQ(
    bridge::evaluate_post_isaac_refine_gate(input),
    bridge::PostIsaacRefineDisposition::kWaitingForMinimumDelay);
}

TEST(PostIsaacRefineGate, AcceptsFreshPostSeedPoseAfterIsaacSettles)
{
  const auto input = ready_input();

  EXPECT_EQ(
    bridge::evaluate_post_isaac_refine_gate(input),
    bridge::PostIsaacRefineDisposition::kEligible);
}

TEST(PostIsaacRefineGate, AbandonsStaticRefineWhenNavigationHasStarted)
{
  auto input = ready_input();
  input.robot_moving = true;

  EXPECT_EQ(
    bridge::evaluate_post_isaac_refine_gate(input),
    bridge::PostIsaacRefineDisposition::kAbandonBecauseMoving);
}

TEST(PostIsaacRefineGate, RequestsNomotionUpdateOnlyAfterSeedAndIsaacSettle)
{
  bridge::PostIsaacNomotionRequestGateInput input;
  input.enabled = true;
  input.refine_pending = true;
  input.explicit_target_settled = true;
  input.service_ready = true;
  input.max_requests = 4;
  input.now_sec = 100.50;
  input.seed_sec = 100.0;
  input.min_delay_sec = 0.25;
  input.request_period_sec = 0.50;

  EXPECT_TRUE(bridge::should_request_post_isaac_nomotion_update(input));

  input.explicit_target_settled = false;
  EXPECT_FALSE(bridge::should_request_post_isaac_nomotion_update(input));

  input.explicit_target_settled = true;
  input.now_sec = 100.10;
  EXPECT_FALSE(bridge::should_request_post_isaac_nomotion_update(input));
}

TEST(PostIsaacRefineGate, BoundsNomotionRetriesAndRequestRate)
{
  bridge::PostIsaacNomotionRequestGateInput input;
  input.enabled = true;
  input.refine_pending = true;
  input.explicit_target_settled = true;
  input.service_ready = true;
  input.max_requests = 4;
  input.now_sec = 101.0;
  input.seed_sec = 100.0;
  input.last_request_sec = 100.75;
  input.min_delay_sec = 0.25;
  input.request_period_sec = 0.50;

  EXPECT_FALSE(bridge::should_request_post_isaac_nomotion_update(input));

  input.now_sec = 101.30;
  EXPECT_TRUE(bridge::should_request_post_isaac_nomotion_update(input));

  input.request_count = 4;
  EXPECT_FALSE(bridge::should_request_post_isaac_nomotion_update(input));
}
