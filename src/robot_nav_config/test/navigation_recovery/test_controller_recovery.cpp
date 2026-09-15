#include <chrono>
#include <memory>
#include <thread>
#include <gtest/gtest.h>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "robot_nav_config/goal_scoped_rotation_shim_controller.hpp"
#include "robot_nav_config/elevator_aware_progress_checker.hpp"

namespace
{
class ControllerRecovery : public ::testing::Test
{
protected:
  virtual double idle_rearm_delay() const {return 100.0;}
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("controller_server");
    node->declare_parameter("controller_frequency", 15.0);
    node->declare_parameter("FollowPath.primary_controller",
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    node->declare_parameter("FollowPath.use_rotate_to_heading", false);
    node->declare_parameter("FollowPath.ordinary_local_repair_enabled", false);
    node->declare_parameter("FollowPath.ordinary_local_repair_progress_topic", "");
    node->declare_parameter("FollowPath.ordinary_local_repair_path_topic", "");
    node->declare_parameter("progress_checker.movement_time_allowance", 0.02);
    node->declare_parameter("FollowPath.same_goal_rearm_after_idle_sec", idle_rearm_delay());
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_recovery_costmap");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
    map->getCostmap()->resizeMap(200, 200, 0.05, -5.0, -5.0);
    map->getCostmap()->resetMap(0, 0, 200, 200);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    // This fixture supplies a static transform directly, without TF discovery.
    tf->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf->setTransform(transform, "isolated_test", true));
    controller.configure(node, "FollowPath", tf, map);
    controller.activate();
    progress.initialize(node, "progress_checker");
    checker.initialize(node, "goal_checker", map);
    state = robot_nav_config::navigation_recovery::for_node(node->get_node_base_interface().get());
    first.header.frame_id = "odom";
    first.header.stamp = node->now();
    for (int i = 0; i <= 40; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = first.header;
      p.pose.position.x = i * 0.05;
      p.pose.orientation.w = 1.0;
      first.poses.push_back(p);
    }
    pose = first.poses.front();
    controller.setPlan(first);
    progress.reset();
    ASSERT_TRUE(progress.check(pose));
    const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
    ASSERT_GT(command.twist.linear.x, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ASSERT_FALSE(progress.check(pose));
    next = first;
    next.header.stamp = node->now();
    // Same exact final goal; the new path starts north instead of east.
    for (int i = 0; i < 20; ++i) {
      next.poses[i].pose.position.x = 0.0;
      next.poses[i].pose.position.y = i * 0.05;
    }
  }
  void TearDown() override
  {
    controller.deactivate();
    controller.cleanup();
    map->on_cleanup(rclcpp_lifecycle::State{});
  }
  nav2_util::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  robot_nav_config::GoalScopedRotationShimController controller;
  robot_nav_config::ElevatorAwareProgressChecker progress;
  nav2_controller::SimpleGoalChecker checker;
  std::shared_ptr<robot_nav_config::navigation_recovery::RecoveryState> state;
  nav_msgs::msg::Path first, next;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(ControllerRecovery, RealProgressFailureRearmsActualShimImmediately)
{
  // Exercise the actual ROS request/response adapter as well as the real
  // progress checker -> shared evidence -> setPlan -> startup-command path.
  using Prepare = robot_nav_config::srv::PrepareOrdinaryNavigationRecovery;
  auto client = node->create_client<Prepare>("~/prepare_ordinary_navigation_recovery");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<Prepare::Request>();
  request->path = next;
  request->attempt_started = first.header.stamp;
  auto future = client->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node->get_node_base_interface(), future,
    std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->prepared);
  controller.setPlan(next);
  progress.reset();
  EXPECT_TRUE(progress.check(pose));
  const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0);
}

TEST_F(ControllerRecovery, HotUpdateWithoutRecoveryDoesNotRearmShim)
{
  controller.setPlan(next);
  progress.reset();
  const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(ControllerRecovery, InspectionReportsFailureButCannotPrepareRotation)
{
  using Prepare = robot_nav_config::srv::PrepareOrdinaryNavigationRecovery;
  auto client = node->create_client<Prepare>("~/prepare_ordinary_navigation_recovery");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<Prepare::Request>();
  request->inspect_only = true;
  request->path = next;
  request->attempt_started = first.header.stamp;
  auto future = client->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node->get_node_base_interface(), future,
    std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
  const auto result = future.get();
  EXPECT_TRUE(result->matched);
  EXPECT_TRUE(result->recoverable);
  EXPECT_FALSE(result->prepared);
  controller.setPlan(next);
  const auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

class ControllerRecoveryShortIdle : public ControllerRecovery
{
protected:
  double idle_rearm_delay() const override {return 0.001;}
};

TEST_F(ControllerRecoveryShortIdle, RepeatedRecoverySuppressesIdleStartupRearm)
{
  std::string reason;
  ASSERT_TRUE(state->prepare(next, rclcpp::Time(first.header.stamp).nanoseconds(), reason));
  controller.setPlan(next);
  progress.reset();
  ASSERT_TRUE(progress.check(pose));
  auto command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  ASSERT_GT(command.twist.angular.z, 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_FALSE(progress.check(pose));
  const auto previous = next.header.stamp;
  next.header.stamp = node->now();
  ASSERT_TRUE(state->prepare(next, rclcpp::Time(previous).nanoseconds(), reason));
  controller.setPlan(next);
  progress.reset();
  command = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(ControllerRecovery, RecoverySpinDoesNotBypassCostmapCollision)
{
  std::string reason;
  ASSERT_TRUE(state->prepare(next, rclcpp::Time(first.header.stamp).nanoseconds(), reason));
  controller.setPlan(next);
  progress.reset();
  auto grid = map->getCostmap();
  for (unsigned int y = 85; y < 115; ++y) {
    for (unsigned int x = 85; x < 115; ++x) {
      grid->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  EXPECT_THROW(controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker), std::exception);
}
}  // namespace
