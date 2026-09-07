#include <gtest/gtest.h>

#include "robot_safety/navigation_speed_envelope.hpp"

using robot_safety::select_navigation_speed_envelope;

TEST(NavigationSpeedEnvelope, OrdinaryNavigationKeepsExistingCaps)
{
  const auto limits =
    select_navigation_speed_envelope(false, 0.08, 0.05, 0.40, 0.40);

  EXPECT_DOUBLE_EQ(limits.reverse_max_mps, 0.08);
  EXPECT_DOUBLE_EQ(limits.lateral_max_mps, 0.05);
}

TEST(NavigationSpeedEnvelope, ElevatorExecutionSessionSelectsDedicatedCaps)
{
  const auto limits =
    select_navigation_speed_envelope(true, 0.08, 0.05, 0.40, 0.40);

  EXPECT_DOUBLE_EQ(limits.reverse_max_mps, 0.40);
  EXPECT_DOUBLE_EQ(limits.lateral_max_mps, 0.40);
}
