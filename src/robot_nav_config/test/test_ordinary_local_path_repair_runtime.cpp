#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "robot_nav_config/ordinary_local_path_repair_runtime.hpp"
#include "robot_nav_config/elevator_aware_progress_policy.hpp"

namespace
{
class RuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("repair_runtime_test");
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    map->set_parameter(rclcpp::Parameter("footprint_padding", 0.03));
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}),
      nav2_util::CallbackReturn::SUCCESS);
    map->getCostmap()->resizeMap(200U, 200U, 0.05, -5.0, -5.0);
    map->getCostmap()->resetMap(0, 0, 200, 200);
    // Physical half-envelope; Costmap2DROS adds 0.03 and repair adds 0.08 m.
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = footprint[1].x = 0.36;
    footprint[2].x = footprint[3].x = -0.36;
    footprint[0].y = footprint[3].y = 0.25;
    footprint[1].y = footprint[2].y = -0.25;
    map->setRobotFootprint(footprint);
    node->declare_parameter("FollowPath.primary_controller",
      "nav2_mppi_controller::MPPIController");
    node->declare_parameter("FollowPath.RangerClearanceCritic.enabled", true);
    node->declare_parameter("FollowPath.RangerClearanceCritic.preferred_margin", 0.05);
    node->declare_parameter("FollowPath.ordinary_local_repair_progress_topic", "");
    node->declare_parameter("FollowPath.ordinary_local_repair_path_topic", "");
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    runtime.configure(node, "FollowPath", tf, map, 0.40);
    runtime.activate();
    path.header.frame_id = "odom";
    for (int i = 0; i <= 100; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = -2.0 + i * 0.05;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    pose = path.poses.front();
    runtime.set_plan(path);
  }
  void TearDown() override
  {
    runtime.deactivate();
    // Isolated fixture: avoid cancel-before-spin teardown races in Humble's
    // Costmap2DROS internal executor. The next SetUp creates a fresh context.
    rclcpp::shutdown();
    map->on_cleanup(rclcpp_lifecycle::State{});
  }
  nav2_util::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  robot_nav_config::OrdinaryLocalPathRepairRuntime runtime;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
};
}

TEST_F(RuntimeTest, DistantBlockedPathDoesNotHoldWhileSearchingOrWaitingForRejoin)
{
  // All possible rejoin poses are obstructed, but the robot's nearby corridor is free.
  auto * grid = map->getCostmap();
  for (unsigned int x = 140; x < 200; ++x) {
    for (unsigned int y = 0; y < 200; ++y) {
      grid->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  for (int i = 0; i < 80; ++i) {
    EXPECT_FALSE(runtime.update(pose).hold_position);
    EXPECT_EQ(runtime.progress_state(),
      robot_nav_config::ElevatorScopedProgressState::kOrdinary);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(RuntimeTest, KnownMppiNoControlRetriesAndResumesWithoutConsumingProgressTimeout)
{
  robot_nav_config::ElevatorAwareProgressPolicy progress({0.03, 0.05, 12.0},
    {0.015, 0.015, 20.0});
  for (int i = 0; i < 300; ++i) {
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = runtime.compute_command(pose, []()
      -> geometry_msgs::msg::TwistStamped {
        throw std::runtime_error("Optimizer fail to compute path");
      }));
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
    EXPECT_EQ(runtime.progress_state(),
      robot_nav_config::ElevatorScopedProgressState::kOrdinaryLocalWaitClear);
    EXPECT_TRUE(progress.check({0.0, 0.0, 0.0}, i / 15.0, runtime.progress_state()));
  }
  const auto recovered = runtime.compute_command(pose, [] {
      geometry_msgs::msg::TwistStamped command;
      command.twist.linear.x = 0.4;
      return command;
    });
  EXPECT_DOUBLE_EQ(recovered.twist.linear.x, 0.4);
  EXPECT_EQ(runtime.progress_state(), robot_nav_config::ElevatorScopedProgressState::kOrdinary);
}

TEST_F(RuntimeTest, UnknownControllerFailureIsNotHiddenAsObstacleWait)
{
  EXPECT_THROW(runtime.compute_command(pose, []() -> geometry_msgs::msg::TwistStamped {
    throw std::runtime_error("TF input failed");
  }), std::runtime_error);
}

TEST_F(RuntimeTest, ActionAndLifecycleResetRetireOldWait)
{
  const auto no_control = []() -> geometry_msgs::msg::TwistStamped {
      throw std::runtime_error("Optimizer fail to compute path");
    };
  ASSERT_NO_THROW(runtime.compute_command(pose, no_control));
  runtime.set_plan(path);
  EXPECT_EQ(runtime.progress_state(), robot_nav_config::ElevatorScopedProgressState::kOrdinary);
  ASSERT_NO_THROW(runtime.compute_command(pose, no_control));
  runtime.deactivate();
  EXPECT_EQ(runtime.progress_state(), robot_nav_config::ElevatorScopedProgressState::kOrdinary);
  EXPECT_THROW(runtime.compute_command(pose, no_control), std::runtime_error);
}

TEST_F(RuntimeTest, SameErrorFromNonMppiPluginIsNotClassifiedAsMppiRetry)
{
  node->set_parameter(rclcpp::Parameter("FollowPath.primary_controller", "another_plugin"));
  robot_nav_config::OrdinaryLocalPathRepairRuntime other;
  other.configure(node, "FollowPath", tf, map, 0.4);
  other.set_plan(path);
  EXPECT_THROW(other.compute_command(pose, []() -> geometry_msgs::msg::TwistStamped {
    throw std::runtime_error("Optimizer fail to compute path");
  }), std::runtime_error);
}

TEST_F(RuntimeTest, MeasuredMppiAdapterRetainsExistingNoControlWaitAndRetry)
{
  node->set_parameter(rclcpp::Parameter("FollowPath.primary_controller",
    "robot_nav_config::RangerMPPIController"));
  robot_nav_config::OrdinaryLocalPathRepairRuntime measured;
  measured.configure(node, "FollowPath", tf, map, 0.4);
  measured.activate();
  measured.set_plan(path);
  EXPECT_NO_THROW(measured.compute_command(pose, []() -> geometry_msgs::msg::TwistStamped {
    throw std::runtime_error("Optimizer fail to compute path");
  }));
  EXPECT_EQ(measured.progress_state(),
    robot_nav_config::ElevatorScopedProgressState::kOrdinaryLocalWaitClear);
  const auto recovered = measured.compute_command(pose, [] {
    geometry_msgs::msg::TwistStamped command;
    command.twist.linear.x = 0.3;
    return command;
  });
  EXPECT_DOUBLE_EQ(recovered.twist.linear.x, 0.3);
}

TEST_F(RuntimeTest, ConfiguredPreferenceReachesWorkerAndHotReplacementWithoutHolding)
{
  const auto original_footprint = map->getRobotFootprint();
  for (unsigned int x = 94; x <= 104; ++x) {
    map->getCostmap()->setCost(x, 108, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  std::optional<nav_msgs::msg::Path> replacement;
  for (int i = 0; i < 300 && !replacement; ++i) {
    const auto update = runtime.update(pose);
    EXPECT_FALSE(update.hold_position);
    replacement = update.replacement_path;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(replacement.has_value());
  EXPECT_EQ(replacement->poses.back(), path.poses.back());
  EXPECT_EQ(original_footprint, map->getRobotFootprint());
}
