#include <gtest/gtest.h>

#include "robot_local_state/stationary_gate.hpp"

using robot_local_state::StationaryGate;

TEST(StationaryGate, SlowNonzeroCommandNeverAllowsBiasUpdate)
{
  StationaryGate gate(true, true, 0.5, 0.5, 2.0, 1.0, false);

  for (int index = 0; index < 60; ++index) {
    const double now_sec = 10.0 + static_cast<double>(index) * 0.1;
    gate.observe_odom(now_sec, true);
    gate.observe_command(now_sec, false);
    EXPECT_FALSE(gate.confirmed(now_sec));
  }
}

TEST(StationaryGate, RequiresHoldoffAndContinuousWheelSettleAfterCommand)
{
  StationaryGate gate(true, true, 0.5, 0.5, 2.0, 1.0, false);

  gate.observe_command(10.0, false);
  gate.observe_odom(10.0, true);
  EXPECT_FALSE(gate.confirmed(10.0));

  gate.observe_odom(11.1, true);
  EXPECT_FALSE(gate.confirmed(11.1));

  gate.observe_odom(13.0, true);
  EXPECT_FALSE(gate.confirmed(13.0));

  gate.observe_odom(13.2, true);
  EXPECT_TRUE(gate.confirmed(13.2));
}

TEST(StationaryGate, AllowsStartupCalibrationWithoutIdleCommandHeartbeat)
{
  StationaryGate gate(true, true, 0.5, 0.5, 2.0, 1.0, false);

  gate.observe_odom(1.0, true);
  EXPECT_FALSE(gate.confirmed(1.0));

  gate.observe_odom(3.1, true);
  EXPECT_TRUE(gate.confirmed(3.1));
}

TEST(StationaryGate, WheelMotionResetsContinuousSettleWindow)
{
  StationaryGate gate(true, true, 0.5, 0.5, 2.0, 1.0, false);

  gate.observe_odom(1.0, true);
  EXPECT_FALSE(gate.confirmed(1.0));
  gate.observe_odom(2.0, false);
  EXPECT_FALSE(gate.confirmed(2.0));
  gate.observe_odom(2.1, true);
  EXPECT_FALSE(gate.confirmed(2.1));
  gate.observe_odom(4.2, true);
  EXPECT_TRUE(gate.confirmed(4.2));
}
