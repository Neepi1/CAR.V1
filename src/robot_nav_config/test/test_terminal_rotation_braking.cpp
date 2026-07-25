#include <cmath>

#include <gtest/gtest.h>

#include "robot_nav_config/terminal_rotation_braking.hpp"

namespace robot_nav_config
{

TEST(TerminalRotationBraking, LimitsSpeedToStoppingEnvelopeNearGoal)
{
  constexpr double requested_speed = 0.60;
  constexpr double remaining_yaw = 0.054744;
  constexpr double max_angular_decel = 1.20;

  const double limited = limit_terminal_rotation_speed(
    requested_speed, remaining_yaw, max_angular_decel);
  const double expected_limit = std::sqrt(2.0 * max_angular_decel * remaining_yaw);

  EXPECT_NEAR(limited, expected_limit, 1.0e-9);
  EXPECT_LT(limited, requested_speed);
}

TEST(TerminalRotationBraking, PreservesDirection)
{
  const double limited = limit_terminal_rotation_speed(-0.60, 0.054744, 1.20);

  EXPECT_LT(limited, 0.0);
}

TEST(TerminalRotationBraking, LeavesFarGoalSpeedUnchanged)
{
  const double limited = limit_terminal_rotation_speed(0.60, 1.0, 1.20);

  EXPECT_DOUBLE_EQ(limited, 0.60);
}

}  // namespace robot_nav_config
