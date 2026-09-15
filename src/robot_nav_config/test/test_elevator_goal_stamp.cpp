#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "robot_nav_config/elevator_scoped_planner.hpp"

namespace
{
geometry_msgs::msg::Pose make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

class ElevatorGoalStamp : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("elevator_goal_stamp_test");
    node->declare_parameter("direct.unchecked_direct_path", true);
    node->declare_parameter("goal_checker.xy_goal_tolerance", 0.06);
    node->declare_parameter("goal_checker.yaw_goal_tolerance", 0.05);
    node->declare_parameter("goal_checker.stateful", false);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("goal_stamp_costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State{}),
      nav2_util::CallbackReturn::SUCCESS);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    planner.configure(node, "direct", tf, costmap);
    checker.initialize(node, "goal_checker", costmap);

    // Recorded 2026-09-10 panel arrival. Map->odom is constant across
    // the planning +/-0.5s window; the actual Path stamp was not captured.
    start.header.frame_id = goal.header.frame_id = "map";
    start.header.stamp = rclcpp::Time(1789048770117454400LL);
    goal.header.stamp = start.header.stamp;
    start.pose = make_pose(-2.698619, 0.204719, -0.027472);
    goal.pose = make_pose(-2.630019, 0.873010, -0.062941);
    insert_tf(1789048770117454400LL,
      -1.2284728335136852, 0.8601834708670231, 1.7773765150379275);
    insert_tf(1789048778772768896LL,
      -1.1510668794390237, 0.834198304379765, 1.7496201026487088);
  }

  void TearDown() override
  {
    planner.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State{});
  }

  void insert_tf(int64_t stamp, double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.header.stamp = rclcpp::Time(stamp);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation = make_pose(0, 0, yaw).orientation;
    ASSERT_TRUE(tf->setTransform(transform, "recorded_fixture", false));
  }

  geometry_msgs::msg::Pose transform_endpoint(geometry_msgs::msg::PoseStamped endpoint)
  {
    // Humble ControllerServer::isGoalReached uses this same transform seam.
    geometry_msgs::msg::PoseStamped transformed;
    auto tolerance = rclcpp::Duration::from_seconds(0.1);
    EXPECT_TRUE(nav_2d_utils::transformPose(tf, "odom", endpoint, transformed, tolerance));
    return transformed.pose;
  }

  nav2_util::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  robot_nav_config::ElevatorScopedPlanner planner;
  nav2_controller::SimpleGoalChecker checker;
  geometry_msgs::msg::PoseStamped start, goal;
};

TEST_F(ElevatorGoalStamp, OnlyEndpointStampChanges)
{
  const auto path = planner.createPlan(start, goal);
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_EQ(path.header, start.header);
  EXPECT_EQ(path.poses.front(), start);
  EXPECT_EQ(path.poses.back().header.frame_id, "map");
  EXPECT_EQ(path.poses.back().pose, goal.pose);
  EXPECT_EQ(path.poses.back().header.stamp, builtin_interfaces::msg::Time{});
}

TEST_F(ElevatorGoalStamp, CapturedOldGoalPassesButCurrentMapGoalMustNot)
{
  const auto path = planner.createPlan(start, goal);
  auto stale_endpoint = path.poses.back();
  stale_endpoint.header.stamp = start.header.stamp;
  const auto stale = transform_endpoint(stale_endpoint);
  const auto actual = transform_endpoint(path.poses.back());
  const auto robot = make_pose(
    0.24535228633375794, 1.3886304310056325, -1.838634210441626);
  EXPECT_NEAR(std::hypot(robot.position.x - stale.position.x,
    robot.position.y - stale.position.y), 0.058056333826, 1e-9);
  EXPECT_TRUE(checker.isGoalReached(robot, stale, geometry_msgs::msg::Twist{}));
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(robot, actual, geometry_msgs::msg::Twist{}));
  EXPECT_NEAR(std::hypot(robot.position.x - actual.position.x,
    robot.position.y - actual.position.y), 0.081887122908, 1e-9);
}

TEST_F(ElevatorGoalStamp, StillAcceptsWithinToleranceAndRejectsBadYaw)
{
  const auto endpoint = transform_endpoint(planner.createPlan(start, goal).poses.back());
  auto robot = endpoint;
  robot.position.x += 0.059;
  EXPECT_TRUE(checker.isGoalReached(robot, endpoint, geometry_msgs::msg::Twist{}));
  robot.position.x = endpoint.position.x + 0.061;
  EXPECT_FALSE(checker.isGoalReached(robot, endpoint, geometry_msgs::msg::Twist{}));
  robot = endpoint;
  // A 0.051 rad residual must still fail, even with perfect XY.
  const double yaw = 2.0 * std::atan2(endpoint.orientation.z, endpoint.orientation.w);
  robot.orientation = make_pose(0, 0, yaw + 0.051).orientation;
  EXPECT_FALSE(checker.isGoalReached(robot, endpoint, geometry_msgs::msg::Twist{}));
}
}  // namespace
