#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/ordinary_local_path_repair_worker.hpp"

TEST(OrdinaryLocalPathRepairWorker, PreservesPlanGenerationAcrossBackgroundSearch)
{
  robot_nav_config::OrdinaryLocalPathRepairWorker worker(
    nullptr,
    [](const robot_nav_config::OrdinaryLocalPathRepairRequest &) {
      robot_nav_config::OrdinaryLocalPathRepairResult result;
      result.status = robot_nav_config::OrdinaryLocalPathRepairStatus::kPathClear;
      return result;
    });
  worker.start();

  robot_nav_config::OrdinaryLocalPathRepairWork work;
  work.plan_generation = 42U;
  work.request.costmap_frame = "odom";
  work.request.costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    20U, 20U, 0.05, -0.5, -0.5, 0U);
  ASSERT_TRUE(worker.submit(std::move(work)));

  std::optional<robot_nav_config::OrdinaryLocalPathRepairWorkResult> result;
  for (int attempt = 0; attempt < 100 && !result.has_value(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    result = worker.take_result();
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->plan_generation, 42U);
  EXPECT_EQ(
    result->repair.status,
    robot_nav_config::OrdinaryLocalPathRepairStatus::kPathClear);
  EXPECT_FALSE(worker.busy());
  worker.stop();
}

TEST(OrdinaryLocalPathRepairWorker, PreservesUnexpectedFailureForTheControllerThread)
{
  robot_nav_config::OrdinaryLocalPathRepairWorker worker(nullptr,
    [](const robot_nav_config::OrdinaryLocalPathRepairRequest &)
      -> robot_nav_config::OrdinaryLocalPathRepairResult {
        throw std::runtime_error("search input corrupt");
      });
  worker.start();
  robot_nav_config::OrdinaryLocalPathRepairWork work;
  work.plan_generation = 7;
  work.request.costmap_frame = "odom";
  work.request.costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    20U, 20U, 0.05, -0.5, -0.5, 0U);
  ASSERT_TRUE(worker.submit(std::move(work)));
  std::optional<robot_nav_config::OrdinaryLocalPathRepairWorkResult> result;
  for (int i = 0; i < 100 && !result; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    result = worker.take_result();
  }
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->error);
  EXPECT_THROW(std::rethrow_exception(result->error), std::runtime_error);
  EXPECT_EQ(result->plan_generation, 7U);
  worker.stop();
}
