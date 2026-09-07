#include <chrono>

#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_blockage.hpp"

namespace
{

using namespace std::chrono_literals;
using robot_nav_config::ElevatorScopedBlockage;

TEST(ElevatorScopedBlockage, ZeroCyclesDoNotExtendTheDeadline) {
  ElevatorScopedBlockage blockage;
  const auto start = std::chrono::steady_clock::time_point{};

  blockage.observe_blocked(start);
  blockage.observe_zero_command();

  EXPECT_TRUE(blockage.active());
  EXPECT_FALSE(blockage.expired(start + 2999ms, 3.0));
  EXPECT_TRUE(blockage.expired(start + 3001ms, 3.0));
}

TEST(ElevatorScopedBlockage, AVerifiedNonzeroCommandClearsTheEpisode) {
  ElevatorScopedBlockage blockage;
  const auto start = std::chrono::steady_clock::time_point{};
  blockage.observe_blocked(start);

  blockage.observe_verified_nonzero_command();

  EXPECT_FALSE(blockage.active());
  EXPECT_FALSE(blockage.expired(start + 10s, 3.0));
}

} // namespace
