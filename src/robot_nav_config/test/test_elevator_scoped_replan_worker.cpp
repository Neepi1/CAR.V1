#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_replan_worker.hpp"

namespace {

using namespace std::chrono_literals;
using robot_nav_config::ElevatorScopedPose;
using robot_nav_config::ElevatorScopedReplanRequest;
using robot_nav_config::ElevatorScopedReplanWorker;
using robot_nav_config::ElevatorScopedSearchResult;
using robot_nav_config::ElevatorScopedSearchStatus;

ElevatorScopedReplanRequest make_request(const std::size_t generation) {
  ElevatorScopedReplanRequest request;
  request.plan_generation = generation;
  request.costmap_frame = "odom";
  request.costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
      80U, 80U, 0.05, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE);
  for (const auto &coordinate : {std::pair<double, double>{0.39, 0.28},
                                 std::pair<double, double>{0.39, -0.28},
                                 std::pair<double, double>{-0.39, -0.28},
                                 std::pair<double, double>{-0.39, 0.28}}) {
    geometry_msgs::msg::Point point;
    point.x = coordinate.first;
    point.y = coordinate.second;
    request.footprint.push_back(point);
  }
  request.start = ElevatorScopedPose{-1.0, 0.0, 0.0};
  request.goal = ElevatorScopedPose{1.0, 0.0, 0.0};
  return request;
}

TEST(ElevatorScopedReplanWorker, ExecutesSearchOutsideTheSubmittingThread) {
  std::promise<void> search_started_promise;
  auto search_started = search_started_promise.get_future();
  std::promise<void> release_search_promise;
  auto release_search = release_search_promise.get_future().share();

  ElevatorScopedReplanWorker worker(
      [&search_started_promise,
       release_search](const ElevatorScopedReplanRequest &) mutable {
        search_started_promise.set_value();
        release_search.wait();
        ElevatorScopedSearchResult result;
        result.status = ElevatorScopedSearchStatus::kSuccess;
        return result;
      });
  worker.start();

  ASSERT_TRUE(worker.submit(make_request(17U)));
  ASSERT_EQ(search_started.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(worker.busy());
  EXPECT_FALSE(worker.take_result().has_value());

  release_search_promise.set_value();
  std::optional<robot_nav_config::ElevatorScopedReplanResult> result;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline && !result) {
    result = worker.take_result();
    if (!result) {
      std::this_thread::sleep_for(1ms);
    }
  }

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->plan_generation, 17U);
  EXPECT_EQ(result->costmap_frame, "odom");
  EXPECT_TRUE(result->search.succeeded());
  EXPECT_FALSE(worker.busy());
  worker.stop();
}

TEST(ElevatorScopedReplanWorker, PreservesACompletedResultUntilItIsConsumed) {
  ElevatorScopedReplanWorker worker([](const ElevatorScopedReplanRequest &) {
    ElevatorScopedSearchResult result;
    result.status = ElevatorScopedSearchStatus::kSuccess;
    return result;
  });
  worker.start();
  ASSERT_TRUE(worker.submit(make_request(21U)));

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline && worker.busy()) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_FALSE(worker.busy());
  EXPECT_FALSE(worker.submit(make_request(22U)));

  const auto first = worker.take_result();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->plan_generation, 21U);
  EXPECT_TRUE(worker.submit(make_request(22U)));
  worker.stop();
}

TEST(ElevatorScopedReplanWorker, ObstacleRecoveryBuildsFreshRouteToFinalGoal) {
  auto request = make_request(31U);
  request.start = ElevatorScopedPose{0.0, 0.0, 0.0};
  request.goal = ElevatorScopedPose{1.6, 0.0, 0.0};
  request.parameters.sampling.max_distance_m = 2.5;
  request.parameters.search_radius_m = 2.5;
  request.parameters.maximum_expansions = 60000U;
  request.parameters.maximum_search_time_sec = 1.0;
  unsigned int obstacle_x = 0U;
  unsigned int obstacle_y = 0U;
  ASSERT_TRUE(request.costmap->worldToMap(0.47, 0.22, obstacle_x, obstacle_y));
  request.costmap->setCost(obstacle_x, obstacle_y,
                           nav2_costmap_2d::LETHAL_OBSTACLE);

  ElevatorScopedReplanWorker worker;
  worker.start();
  ASSERT_TRUE(worker.submit(std::move(request)));
  std::optional<robot_nav_config::ElevatorScopedReplanResult> result;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline && !result) {
    result = worker.take_result();
    if (!result) {
      std::this_thread::sleep_for(1ms);
    }
  }

  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->search.succeeded())
      << robot_nav_config::elevator_scoped_search_status_name(
             result->search.status);
  ASSERT_FALSE(result->search.path.samples.empty());
  EXPECT_NEAR(result->search.path.samples.back().pose.x, 1.60, 0.03);
  EXPECT_NEAR(result->search.path.samples.back().pose.y, 0.00, 0.03);
  const auto start = result->search.path.samples.front().pose;
  double first_lateral_delta = 0.0;
  for (const auto &sample : result->search.path.samples) {
    if (sample.phase != robot_nav_config::ElevatorScopedMotionPhase::kYaw &&
        std::hypot(sample.pose.x - start.x, sample.pose.y - start.y) > 1.0e-6) {
      first_lateral_delta = sample.pose.y - start.y;
      break;
    }
  }
  EXPECT_LT(first_lateral_delta, 0.0);
  worker.stop();
}

} // namespace
