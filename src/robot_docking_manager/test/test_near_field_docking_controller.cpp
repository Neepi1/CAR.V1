#include <gtest/gtest.h>

#include <cmath>

#include "robot_docking_manager/near_field_docking_controller.hpp"

namespace
{

using robot_docking_manager::NearFieldControlConfig;
using robot_docking_manager::NearFieldDockingController;
using robot_docking_manager::NearFieldMotionMode;
using robot_docking_manager::NearFieldObservation;
using robot_docking_manager::NearFieldPhase;

NearFieldObservation observation(
  const std::uint64_t sequence,
  const double distance,
  const double lateral,
  const double yaw)
{
  return NearFieldObservation{true, sequence, distance, lateral, yaw};
}

TEST(NearFieldDockingController, InitialYawCaptureRequiresThreeDistinctAlignedObservations)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  const auto first = controller.step(observation(1U, 0.60, 0.0, 0.003));
  const auto repeated = controller.step(observation(1U, 0.60, 0.0, 0.003));
  const auto second = controller.step(observation(2U, 0.60, 0.0, 0.003));
  const auto third = controller.step(observation(3U, 0.60, 0.0, 0.003));

  EXPECT_EQ(first.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(first.yaw_stable_samples, 1);
  EXPECT_EQ(repeated.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(repeated.yaw_stable_samples, 1);
  EXPECT_EQ(second.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(second.yaw_stable_samples, 2);
  EXPECT_EQ(third.phase, NearFieldPhase::VectorApproach);
  EXPECT_EQ(third.yaw_stable_samples, 3);
}

TEST(NearFieldDockingController, CombinesForwardAndLateralInParallelMode)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  controller.step(observation(1U, 0.60, 0.08, 0.0));
  controller.step(observation(2U, 0.60, 0.08, 0.0));
  const auto decision = controller.step(observation(3U, 0.60, 0.08, 0.0));

  EXPECT_EQ(decision.phase, NearFieldPhase::VectorApproach);
  EXPECT_EQ(decision.mode, NearFieldMotionMode::Parallel);
  EXPECT_GT(decision.linear_x_mps, 0.0);
  EXPECT_GT(decision.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(decision.angular_z_radps, 0.0);
  EXPECT_LE(std::hypot(decision.linear_x_mps, decision.linear_y_mps), 0.15 + 1.0e-9);
}

TEST(NearFieldDockingController, YawHysteresisDoesNotReturnToSpinForSmallNoise)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  EXPECT_EQ(
    controller.step(observation(1U, 0.60, 0.0, 0.06)).phase,
    NearFieldPhase::YawCapture);
  EXPECT_EQ(
    controller.step(observation(2U, 0.60, 0.0, 0.005)).phase,
    NearFieldPhase::YawCapture);
  EXPECT_EQ(
    controller.step(observation(3U, 0.60, 0.0, 0.004)).phase,
    NearFieldPhase::YawCapture);
  EXPECT_EQ(
    controller.step(observation(4U, 0.60, 0.0, 0.003)).phase,
    NearFieldPhase::VectorApproach);

  const auto noisy = controller.step(observation(5U, 0.58, 0.01, 0.02));
  EXPECT_EQ(noisy.phase, NearFieldPhase::VectorApproach);
  EXPECT_EQ(noisy.mode, NearFieldMotionMode::Parallel);
  EXPECT_DOUBLE_EQ(noisy.angular_z_radps, 0.0);
}

TEST(NearFieldDockingController, LatchedYawAllowsContactHandoffInsideReentryBand)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  EXPECT_EQ(
    controller.step(observation(1U, 0.60, 0.0, 0.10)).phase,
    NearFieldPhase::YawCapture);
  controller.step(observation(2U, 0.60, 0.0, 0.005));
  controller.step(observation(3U, 0.60, 0.0, 0.004));
  EXPECT_EQ(
    controller.step(observation(4U, 0.60, 0.0, 0.003)).phase,
    NearFieldPhase::VectorApproach);

  // Reproduce the 2026-09-04 field dead zone: distance and lateral are in
  // tolerance, while yaw has drifted above the capture-exit threshold but
  // remains well below the configured recapture threshold.
  const auto handoff = controller.step(observation(5U, 0.346, -0.024, -0.0139));

  EXPECT_EQ(handoff.phase, NearFieldPhase::FinalApproach);
  EXPECT_TRUE(handoff.enter_contact_verify);
  EXPECT_FALSE(handoff.alignment_blocked);
  EXPECT_DOUBLE_EQ(handoff.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(handoff.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(handoff.angular_z_radps, 0.0);
  EXPECT_EQ(handoff.reason, "visual_alignment_complete");
}

TEST(NearFieldDockingController, UnlatchedYawInsideReentryBandCannotHandoff)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  const auto decision = controller.step(observation(1U, 0.346, -0.024, -0.0139));

  EXPECT_EQ(decision.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(decision.mode, NearFieldMotionMode::Spinning);
  EXPECT_FALSE(decision.enter_contact_verify);
  EXPECT_FALSE(decision.alignment_blocked);
  EXPECT_DOUBLE_EQ(decision.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(decision.linear_y_mps, 0.0);
  EXPECT_LT(decision.angular_z_radps, 0.0);
  EXPECT_EQ(decision.reason, "yaw_capture");
}

TEST(NearFieldDockingController, LatchedYawAtRecaptureThresholdCannotHandoff)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  controller.step(observation(1U, 0.60, 0.0, 0.0));
  controller.step(observation(2U, 0.60, 0.0, 0.0));
  EXPECT_EQ(
    controller.step(observation(3U, 0.60, 0.0, 0.0)).phase,
    NearFieldPhase::VectorApproach);

  const auto recapture = controller.step(observation(4U, 0.346, -0.024, 0.06));

  EXPECT_EQ(recapture.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(recapture.mode, NearFieldMotionMode::Spinning);
  EXPECT_FALSE(recapture.enter_contact_verify);
  EXPECT_FALSE(recapture.alignment_blocked);
  EXPECT_EQ(recapture.yaw_realignments, 1);
  EXPECT_DOUBLE_EQ(recapture.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(recapture.linear_y_mps, 0.0);
  EXPECT_GT(recapture.angular_z_radps, 0.0);
  EXPECT_EQ(recapture.reason, "yaw_capture");
}

TEST(NearFieldDockingController, ActionlessTranslationDecisionFailsClosed)
{
  NearFieldControlConfig config;
  config.max_parallel_speed_mps = 0.0;
  NearFieldDockingController controller(config);

  controller.step(observation(1U, 0.60, 0.0, 0.0));
  controller.step(observation(2U, 0.60, 0.0, 0.0));
  const auto blocked = controller.step(observation(3U, 0.60, 0.0, 0.0));

  EXPECT_EQ(blocked.phase, NearFieldPhase::AlignmentBlocked);
  EXPECT_TRUE(blocked.alignment_blocked);
  EXPECT_FALSE(blocked.enter_contact_verify);
  EXPECT_DOUBLE_EQ(blocked.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(blocked.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(blocked.angular_z_radps, 0.0);
  EXPECT_EQ(blocked.reason, "no_action_available");
}

TEST(NearFieldDockingController, LocksLateralOnlyInsideLastSixCentimeters)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  controller.step(observation(1U, 0.60, 0.0, 0.0));
  controller.step(observation(2U, 0.60, 0.0, 0.0));
  controller.step(observation(3U, 0.60, 0.0, 0.0));

  const auto before_lock = controller.step(observation(4U, 0.41, 0.02, 0.0));
  EXPECT_EQ(before_lock.phase, NearFieldPhase::FinalApproach);
  EXPECT_NE(before_lock.linear_y_mps, 0.0);

  const auto locked = controller.step(observation(5U, 0.39, 0.02, 0.0));
  EXPECT_EQ(locked.phase, NearFieldPhase::FinalApproach);
  EXPECT_DOUBLE_EQ(locked.linear_y_mps, 0.0);
  EXPECT_GT(locked.linear_x_mps, 0.0);
}

TEST(NearFieldDockingController, FinalApproachDoesNotRegressOnNoisyDistance)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  controller.step(observation(1U, 0.60, 0.0, 0.0));
  controller.step(observation(2U, 0.60, 0.0, 0.0));
  controller.step(observation(3U, 0.60, 0.0, 0.0));

  EXPECT_EQ(
    controller.step(observation(4U, 0.41, 0.04, 0.0)).phase,
    NearFieldPhase::FinalApproach);

  const auto noisy = controller.step(observation(5U, 0.52, 0.04, 0.0));
  EXPECT_EQ(noisy.phase, NearFieldPhase::FinalApproach);
  EXPECT_GT(noisy.linear_x_mps, 0.0);
  EXPECT_LE(noisy.linear_x_mps, 0.05 + 1.0e-9);
}

TEST(NearFieldDockingController, AllowsOnlyOneBoundedYawRecapture)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  controller.step(observation(1U, 0.60, 0.0, 0.0));
  controller.step(observation(2U, 0.60, 0.0, 0.0));
  EXPECT_EQ(
    controller.step(observation(3U, 0.60, 0.0, 0.0)).phase,
    NearFieldPhase::VectorApproach);
  const auto first_realign = controller.step(observation(4U, 0.58, 0.0, 0.06));
  EXPECT_EQ(first_realign.phase, NearFieldPhase::YawCapture);
  EXPECT_EQ(first_realign.yaw_realignments, 1);

  controller.step(observation(5U, 0.58, 0.0, 0.005));
  controller.step(observation(6U, 0.58, 0.0, 0.004));
  EXPECT_EQ(
    controller.step(observation(7U, 0.58, 0.0, 0.003)).phase,
    NearFieldPhase::VectorApproach);

  const auto blocked = controller.step(observation(8U, 0.56, 0.0, 0.06));
  EXPECT_EQ(blocked.phase, NearFieldPhase::AlignmentBlocked);
  EXPECT_TRUE(blocked.alignment_blocked);
  EXPECT_DOUBLE_EQ(blocked.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(blocked.linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(blocked.angular_z_radps, 0.0);
}

TEST(NearFieldDockingController, ContactBudgetScalesWithDistanceAndSpeed)
{
  NearFieldControlConfig config;
  config.contact_timeout_min_sec = 3.0;
  NearFieldDockingController controller(config);

  const double short_timeout = controller.contact_timeout_sec(0.12);
  const double long_timeout = controller.contact_timeout_sec(0.31);

  EXPECT_GT(long_timeout, short_timeout);
  EXPECT_NEAR(long_timeout, 14.0, 1.0e-9);
}

TEST(NearFieldDockingController, RetryBackoffUsesAttemptedDistanceInsteadOfFixedMaximum)
{
  NearFieldDockingController controller(NearFieldControlConfig{});

  EXPECT_NEAR(controller.retry_backoff_distance_m(0.10), 0.20, 1.0e-9);
  EXPECT_NEAR(controller.retry_backoff_distance_m(0.31), 0.39, 1.0e-9);
  EXPECT_NEAR(controller.retry_backoff_distance_m(0.80), 0.60, 1.0e-9);
}

}  // namespace
