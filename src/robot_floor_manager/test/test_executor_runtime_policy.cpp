#include <stdexcept>

#include <gtest/gtest.h>

#include "robot_floor_manager/executor_runtime_policy.hpp"

TEST(ExecutorRuntimePolicy, RetriesOnlyTheKnownTransientActionClientRace)
{
  const std::runtime_error transient(
    "Taking data from action client but no ready event");
  const std::runtime_error unrelated("executor wait set is corrupted");

  EXPECT_EQ(
    robot_floor_manager::ExecutorRuntimeErrorDisposition::kRetry,
    robot_floor_manager::classify_executor_runtime_error(transient));
  EXPECT_EQ(
    robot_floor_manager::ExecutorRuntimeErrorDisposition::kFatal,
    robot_floor_manager::classify_executor_runtime_error(unrelated));
}
