#include <gtest/gtest.h>

#include "robot_nav_config/elevator_speed_limit.hpp"

namespace
{

TEST(ElevatorSpeedLimit, Nav2ZeroSentinelRestoresUnrestrictedControllerScale) {
  EXPECT_DOUBLE_EQ(
    robot_nav_config::elevator_speed_limit_scale(0.0, false, 0.10), 1.0);
  EXPECT_DOUBLE_EQ(
    robot_nav_config::elevator_speed_limit_scale(0.0, true, 0.10), 1.0);
}

TEST(ElevatorSpeedLimit, AppliesMetricAndPercentageCaps) {
  EXPECT_DOUBLE_EQ(
    robot_nav_config::elevator_speed_limit_scale(0.05, false, 0.10), 0.5);
  EXPECT_DOUBLE_EQ(
    robot_nav_config::elevator_speed_limit_scale(25.0, true, 0.10), 0.25);
  EXPECT_DOUBLE_EQ(
    robot_nav_config::elevator_speed_limit_scale(2.0, false, 0.10), 1.0);
}

} // namespace
