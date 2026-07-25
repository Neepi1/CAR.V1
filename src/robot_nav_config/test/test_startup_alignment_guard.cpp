#include <limits>
#include <optional>

#include "gtest/gtest.h"
#include "robot_nav_config/startup_alignment_guard.hpp"

namespace robot_nav_config
{

TEST(StartupAlignmentGuard, HoldsWhenHeadingMeasurementIsUnavailable)
{
  StartupAlignmentGuard guard;
  EXPECT_EQ(
    guard.evaluate(std::nullopt, false, 0.45, 0.075),
    StartupAlignmentDecision::kHold);
  EXPECT_FALSE(guard.rotation_started());
}

TEST(StartupAlignmentGuard, CompletesWhenPathIsTooShortForEntrySampling)
{
  StartupAlignmentGuard guard;
  EXPECT_EQ(
    guard.evaluate(std::nullopt, true, 0.45, 0.075),
    StartupAlignmentDecision::kComplete);
}

TEST(StartupAlignmentGuard, UsesEngagementThresholdBeforeRotationStarts)
{
  StartupAlignmentGuard guard;
  EXPECT_EQ(
    guard.evaluate(0.44, false, 0.45, 0.075),
    StartupAlignmentDecision::kComplete);

  guard.reset();
  EXPECT_EQ(
    guard.evaluate(0.46, false, 0.45, 0.075),
    StartupAlignmentDecision::kRotate);
  EXPECT_TRUE(guard.rotation_started());
}

TEST(StartupAlignmentGuard, UsesDisengagementThresholdAfterRotationStarts)
{
  StartupAlignmentGuard guard;
  EXPECT_EQ(
    guard.evaluate(-1.0, false, 0.45, 0.075),
    StartupAlignmentDecision::kRotate);
  EXPECT_EQ(
    guard.evaluate(-0.10, false, 0.45, 0.075),
    StartupAlignmentDecision::kRotate);
  EXPECT_EQ(
    guard.evaluate(-0.07, false, 0.45, 0.075),
    StartupAlignmentDecision::kComplete);
}

TEST(StartupAlignmentGuard, HoldsOnNonFiniteHeadingError)
{
  StartupAlignmentGuard guard;
  EXPECT_EQ(
    guard.evaluate(
      std::numeric_limits<double>::quiet_NaN(), false, 0.45, 0.075),
    StartupAlignmentDecision::kHold);
}

}  // namespace robot_nav_config
