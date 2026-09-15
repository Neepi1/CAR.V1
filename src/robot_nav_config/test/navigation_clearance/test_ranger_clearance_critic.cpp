#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include "gtest/gtest.h"
#include "nav2_mppi_controller/critic_manager.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_mppi_controller/controller.hpp"

namespace
{
class ClearanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("clearance_test");
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("clearance_map");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"local_inflation_layer"}));
    map->declare_parameter("local_inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
    map->declare_parameter("local_inflation_layer.inflation_radius", 0.6);
    map->declare_parameter("local_inflation_layer.cost_scaling_factor", 6.0);
    map->set_parameter(rclcpp::Parameter("footprint",
      "[[0.36,0.25],[0.36,-0.25],[-0.36,-0.25],[-0.36,0.25]]"));
    map->set_parameter(rclcpp::Parameter("footprint_padding", 0.03));
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
    map->getCostmap()->resizeMap(200U, 200U, 0.05, -5.0, -5.0);
    map->getCostmap()->resetMap(0U, 0U, 200U, 200U);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = footprint[1].x = 0.36;
    footprint[2].x = footprint[3].x = -0.36;
    footprint[0].y = footprint[3].y = 0.25;
    footprint[1].y = footprint[2].y = -0.25;
    map->setRobotFootprint(footprint);
    std::vector<std::string> names{"ObstaclesCritic"};
    if (!std::getenv("NJRH_CLEARANCE_BASELINE")) {names.push_back("RangerClearanceCritic");}
    node->declare_parameter("FollowPath.critics", names);
    node->declare_parameter("FollowPath.ObstaclesCritic.consider_footprint", true);
    node->declare_parameter("FollowPath.ObstaclesCritic.inflation_radius", 0.6);
    node->declare_parameter("FollowPath.ObstaclesCritic.cost_scaling_factor", 6.0);
    node->declare_parameter("FollowPath.ObstaclesCritic.inflation_layer_name", "local_inflation_layer");
    handler = std::make_unique<mppi::ParametersHandler>(node);
    manager = std::make_unique<mppi::CriticManager>();
    // Real Humble manager and pluginlib, not a copied scoring predicate.
    manager->on_configure(node, "FollowPath", map, handler.get());
    state.pose.pose.orientation.w = 1.0;
    path.x = {0.0F, 4.0F};
    path.y = {0.0F, 0.0F};
    path.yaws = {0.0F, 0.0F};
    reset(3, 48);
  }
  void TearDown() override
  {
    manager.reset();
    handler.reset();
    map->on_cleanup(rclcpp_lifecycle::State{});
  }
  void reset(unsigned int batch, unsigned int steps)
  {
    trajectories.reset(batch, steps);
    state.reset(batch, steps);
    costs = xt::zeros<float>({batch});
  }
  void obstacle(double x, double y, unsigned char value = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    unsigned int mx, my;
    ASSERT_TRUE(map->getCostmap()->worldToMap(x, y, mx, my));
    map->getCostmap()->setCost(mx, my, value);
  }
  bool score()
  {
    costs.fill(0.0F);
    mppi::CriticData data{state, trajectories, path, costs, dt, false,
      nullptr, nullptr, std::nullopt, std::nullopt};
    manager->evalTrajectoriesScores(data);
    return data.fail_flag;
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::unique_ptr<mppi::ParametersHandler> handler;
  std::unique_ptr<mppi::CriticManager> manager;
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  xt::xtensor<float, 1> costs;
  float dt{1.0F / 15.0F};
};
}

TEST_F(ClearanceTest, PrefersClearPassageOverBufferIntrusion)
{
  obstacle(0.025, 0.375);
  for (std::size_t t = 0; t < 48; ++t) {
    trajectories.y(0, t) = -0.25F;  // > preference clearance
    trajectories.y(1, t) = 0.0F;    // free physical footprint, but StopZone edge
    trajectories.y(2, t) = -0.05F;  // between the two
  }
  EXPECT_FALSE(score());
  EXPECT_GT(costs(1), costs(0) + 10.0F);
  EXPECT_GT(costs(2), costs(0));
  EXPECT_LT(costs(2), costs(1));
}

TEST_F(ClearanceTest, BufferDoesNotMakeEscapeInfeasible)
{
  obstacle(0.025, 0.425);
  for (std::size_t t = 0; t < 48; ++t) {
    const float fraction = static_cast<float>(t) / 47.0F;
    trajectories.y(0, t) = -0.25F * fraction;  // escape
    trajectories.y(1, t) = 0.0F;             // stay
    trajectories.y(2, t) = 0.08F * fraction; // inward, still no physical collision
  }
  EXPECT_FALSE(score());
  EXPECT_LT(costs(0), costs(1));
  EXPECT_LT(costs(1), costs(2));
  for (const auto value : costs) {EXPECT_TRUE(std::isfinite(value));}
}

TEST_F(ClearanceTest, UsesRotatedRectangleNotCircularRadius)
{
  obstacle(0.025, 0.525);
  for (std::size_t t = 0; t < 48; ++t) {
    trajectories.yaws(1, t) = static_cast<float>(M_PI_2);
    trajectories.yaws(2, t) = static_cast<float>(-M_PI_2);
  }
  EXPECT_FALSE(score());
  EXPECT_GT(costs(1), costs(0) + 1.0F);
  EXPECT_NEAR(costs(1), costs(2), 1e-4F);
}

TEST_F(ClearanceTest, DoesNotWriteFootprintOrCostmapAndRefreshesAfterClearing)
{
  obstacle(0.025, 0.375);
  const auto footprint = map->getRobotFootprint();
  const auto * grid = map->getCostmap();
  const std::vector<unsigned char> before(grid->getCharMap(), grid->getCharMap() + 40000);
  EXPECT_FALSE(score());
  EXPECT_EQ(footprint, map->getRobotFootprint());
  EXPECT_EQ(before, std::vector<unsigned char>(grid->getCharMap(), grid->getCharMap() + 40000));
  EXPECT_GT(costs(0), 0.0F);
  obstacle(0.025, 0.375, nav2_costmap_2d::FREE_SPACE);
  EXPECT_FALSE(score());
  EXPECT_FLOAT_EQ(costs(0), 0.0F);
}

TEST_F(ClearanceTest, PreservesNearGoalAndNativePhysicalCollision)
{
  obstacle(0.025, 0.375);
  path.x(1) = 0.5F;
  EXPECT_FALSE(score());
  EXPECT_FLOAT_EQ(costs(0), 0.0F);
  for (unsigned int x = 90; x < 110; ++x) {
    for (unsigned int y = 90; y < 110; ++y) {
      map->getCostmap()->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  EXPECT_TRUE(score());  // Native ObstaclesCritic still owns failure.
}

TEST_F(ClearanceTest, BenchmarkProductionBatch)
{
  reset(1200, 48);
  for (int i = -80; i < 80; ++i) {
    obstacle(i * 0.05, 1.2);
    obstacle(i * 0.05, -1.2);
  }
  for (std::size_t b = 0; b < 1200; ++b) {
    const double turn = (static_cast<double>(b % 101) - 50.0) / 100.0;
    for (std::size_t t = 0; t < 48; ++t) {
      const double travel = 0.035 * t;
      trajectories.x(b, t) = travel;
      trajectories.y(b, t) = turn * travel * travel;
      trajectories.yaws(b, t) = std::atan(2.0 * turn * travel);
    }
  }
  std::vector<double> times;
  for (int pass = 0; pass < 25; ++pass) {
    const auto begin = std::chrono::steady_clock::now();
    score();
    times.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count());
  }
  std::sort(times.begin(), times.end());
  std::cout << "CLEARANCE_BENCH native+buffer 1200x48 median_ms=" << times[12]
    << " p96_ms=" << times[23] << " max_ms=" << times[24] << std::endl;
  // Report timing; don't turn hardware load variance into a flaky unit test.
}

TEST_F(ClearanceTest, ProductionYamlLoadsAndComputesThroughRealMppi)
{
  // No ROS goals or publishers: invoke the actual optimizer on a synthetic
  // map with the complete checked-in FollowPath parameter/critic configuration.
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE});
  auto controller_node = std::make_shared<nav2_util::LifecycleNode>("controller_server", "", options);
  auto tf = std::make_shared<tf2_ros::Buffer>(controller_node->get_clock());
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(controller_node, "goal_checker", map);
  nav2_mppi_controller::MPPIController controller;
  ASSERT_NO_THROW(controller.configure(controller_node, "FollowPath", tf, map));
  controller.activate();
  nav_msgs::msg::Path plan;
  plan.header.frame_id = map->getGlobalFrameID();
  plan.header.stamp = controller_node->now();
  for (int i = 0; i <= 60; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = 0.05 * i;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }
  controller.setPlan(plan);
  std::vector<double> times;
  geometry_msgs::msg::Twist velocity;
  for (int i = 0; i < 25; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto command = controller.computeVelocityCommands(plan.poses.front(), velocity, &checker);
    times.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count());
    EXPECT_TRUE(std::isfinite(command.twist.linear.x));
    EXPECT_GE(command.twist.linear.x, 0.0);
    velocity = command.twist;
  }
  EXPECT_GT(velocity.linear.x, 0.0);
  std::sort(times.begin(), times.end());
  std::cout << "FULL_MPPI_BENCH median_ms=" << times[12] << " p96_ms=" << times[23]
    << " max_ms=" << times[24] << std::endl;
  controller.deactivate();
  controller.cleanup();
}
