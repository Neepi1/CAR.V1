#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <thread>
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/navigation/mission/navigation_goal_execution_module.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

navigation::NavigationTerminalRuntimePorts terminal_ports()
{
  navigation::NavigationTerminalRuntimePorts ports;
  ports.running = []() {return true;};
  ports.cancel_requested = [](std::uint64_t, std::string &) {return false;};
  ports.safety_hard_blocked = [](std::string &) {return false;};
  ports.verify_final_pose = [](
    const robot_api_server::StoredPose &, bool) {
      return navigation::FinalPoseCheck{};
    };
  ports.update_final_pose = [](
    std::uint64_t, const navigation::FinalPoseCheck &, const std::string &) {};
  ports.set_job_phase = [](std::uint64_t, const std::string &, const std::string &) {};
  ports.publish_motion_mode = [](const std::string &) {};
  ports.current_robot_pose = []() {return robot_api_server::RobotPoseSnapshot{};};
  ports.pose_in_frame = [](const std::string &) {
      return navigation::TerminalFramePoseSnapshot{};
    };
  ports.dock_contact_blocked = [](std::string &) {return false;};
  return ports;
}

class NavigationGoalExecutionModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const auto suffix = std::to_string(++sequence_);
    node_ = std::make_shared<rclcpp::Node>("navigation_goal_execution_test_" + suffix);

    navigation::NavigationActionRuntimeConfig action_config;
    action_config.action_name = "/test/navigation_goal_execution/action_" + suffix;
    action_config.status_topic = action_config.action_name + "/_action/status";
    navigation::NavigationActionRuntimePorts action_ports;
    action_ports.delayed_side_effect_started = [this]() {++side_effect_started_;};
    action_ports.delayed_side_effect_resolved = [this]() {++side_effect_resolved_;};
    action_ports.latch_safety_stop = [this]() {safety_latched_ = true;};
    action_runtime_ = std::make_unique<navigation::NavigationActionRuntime>(
      *node_, std::move(action_config), std::move(action_ports));

    mission_runtime_ = std::make_unique<navigation::NavigationMissionRuntime>();
    navigation::NavigationCompletionPolicyConfig completion_config;
    completion_config.map_frame = "map";
    completion_config.robot_pose_freshness_sec = 0.5;
    completion_config.position_tolerance_m = 0.06;
    completion_policy_ =
      std::make_unique<navigation::NavigationCompletionPolicy>(completion_config);

    navigation::NavigationBridgeWaitConfig bridge_config;
    bridge_config.final_verify_wait_enabled = false;
    bridge_config.final_yaw_wait_enabled = false;
    bridge_wait_ = std::make_unique<navigation::NavigationBridgeWait>(bridge_config);

    terminal_control_ = std::make_unique<navigation::NavigationTerminalControl>(
      navigation::TerminalControlConfig{});
    navigation::NavigationTerminalRuntimeConfig terminal_config;
    terminal_config.command_topic = "/test/navigation_goal_execution/cmd_" + suffix;
    terminal_config.speed_limit_topic = "/test/navigation_goal_execution/limit_" + suffix;
    terminal_config.reverse_enable_topic = "/test/navigation_goal_execution/reverse_" + suffix;
    terminal_config.mode_controller_status_topic =
      "/test/navigation_goal_execution/status_" + suffix;
    terminal_config.actual_stop_odom_topic = "/test/navigation_goal_execution/odom_" + suffix;
    terminal_config.local_costmap_topic = "/test/navigation_goal_execution/costmap_" + suffix;
    terminal_config.zero_command_count = 1;
    terminal_runtime_ = std::make_unique<navigation::NavigationTerminalRuntimeModule>(
      *node_, *terminal_control_, terminal_config, terminal_ports());

    execution_config_.action_name = "/test/navigation_goal_execution/action_" + suffix;
    execution_config_.final_yaw_wait_bridge_smoothing = false;
    execution_config_.pause_global_correction_during_final_yaw = false;
  }

  void TearDown() override
  {
    module_.reset();
    terminal_runtime_.reset();
    terminal_control_.reset();
    bridge_wait_.reset();
    completion_policy_.reset();
    mission_runtime_.reset();
    action_runtime_.reset();
    node_.reset();
  }

  navigation::NavigationGoalExecutionPorts execution_ports()
  {
    navigation::NavigationGoalExecutionPorts ports;
    ports.floor_runtime_operation_blocked = [](
      const std::string &, std::string & detail, std::string *) {
        detail.clear();
        return false;
      };
    ports.undock_before_navigation = [](
      const navigation::NavigationPreGoalDockSnapshot &, std::string & detail, bool & performed) {
        detail = "not docked";
        performed = false;
        return true;
      };
    ports.bridge_safe_for_goal_start = [](
      const std::string &, std::string & detail) {
        detail = "ready";
        return true;
      };
    ports.bridge_readiness_snapshot = [this]() {return bridge_snapshot_;};
    ports.request_amcl_nomotion_update = [this](
      const std::string &, std::string & detail, std::chrono::nanoseconds) {
        ++nomotion_request_count_;
        detail = "requested";
        return true;
      };
    ports.request_bridge_correction_pause = [](
      bool, std::string & detail, std::chrono::nanoseconds, bool) {
        detail = "ok";
        return true;
      };
    ports.robot_pose_snapshot = [this](bool, std::string & detail) {
        detail.clear();
        return robot_pose_;
      };
    ports.safety_hard_blocked = [](std::string &) {return false;};
    ports.docking_job_running = [this]() {return docking_job_running_;};
    ports.dock_contact_blocked = [](std::string &) {return false;};
    ports.cancel_active_goal = [](std::string & detail) {
        detail = "no active goal";
        return true;
      };
    ports.cancel_goal_for_handoff = [](
      const navigation::NavigationActionRuntime::GoalHandle::SharedPtr &, std::string & detail) {
        detail = "canceled";
        return true;
      };
    ports.request_goal_cancel = [this](const std::string & reason) {
        requested_cancel_reason_ = reason;
        return true;
      };
    ports.navigation_runtime_active = []() {return true;};
    ports.set_navigation_runtime_state = [this](
      bool, const std::string & state, const std::string & detail, bool healthy) {
        runtime_state_ = state;
        runtime_detail_ = detail;
        runtime_healthy_ = healthy;
      };
    ports.timestamp_now = []() {return std::string{"2026-09-03T00:00:00Z"};};
    ports.goal_stamp = []() {return builtin_interfaces::msg::Time{};};
    ports.delayed_side_effect_started = [this]() {++side_effect_started_;};
    ports.delayed_side_effect_resolved = [this]() {++side_effect_resolved_;};
    ports.warn = [this](const std::string & warning) {last_warning_ = warning;};
    return ports;
  }

  void make_module()
  {
    module_ = std::make_unique<navigation::NavigationGoalExecutionModule>(
      execution_config_,
      *action_runtime_,
      *mission_runtime_,
      *completion_policy_,
      *bridge_wait_,
      *terminal_control_,
      *terminal_runtime_,
      execution_ports());
  }

  static inline int sequence_{0};
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<navigation::NavigationActionRuntime> action_runtime_;
  std::unique_ptr<navigation::NavigationMissionRuntime> mission_runtime_;
  std::unique_ptr<navigation::NavigationCompletionPolicy> completion_policy_;
  std::unique_ptr<navigation::NavigationBridgeWait> bridge_wait_;
  std::unique_ptr<navigation::NavigationTerminalControl> terminal_control_;
  std::unique_ptr<navigation::NavigationTerminalRuntimeModule> terminal_runtime_;
  std::unique_ptr<navigation::NavigationGoalExecutionModule> module_;
  navigation::NavigationGoalExecutionConfig execution_config_;
  robot_api_server::RobotPoseSnapshot robot_pose_;
  navigation::BridgeReadinessSnapshot bridge_snapshot_;
  bool docking_job_running_{false};
  int nomotion_request_count_{0};
  int side_effect_started_{0};
  int side_effect_resolved_{0};
  bool safety_latched_{false};
  std::string requested_cancel_reason_;
  std::string runtime_state_;
  std::string runtime_detail_;
  bool runtime_healthy_{false};
  std::string last_warning_;
};

TEST_F(NavigationGoalExecutionModuleTest, RejectsIncompleteIntegrationPorts)
{
  EXPECT_THROW(
    module_ = std::make_unique<navigation::NavigationGoalExecutionModule>(
      execution_config_,
      *action_runtime_,
      *mission_runtime_,
      *completion_policy_,
      *bridge_wait_,
      *terminal_control_,
      *terminal_runtime_,
      navigation::NavigationGoalExecutionPorts{}),
    std::invalid_argument);
}

TEST_F(NavigationGoalExecutionModuleTest, EvaluatesFinalPoseThroughInjectedSnapshot)
{
  make_module();
  robot_pose_.available = true;
  robot_pose_.frame_id = "map";
  robot_pose_.x = 1.0;
  robot_pose_.y = 2.0;
  robot_pose_.yaw = 0.2;
  robot_pose_.age_sec = 0.1;

  robot_api_server::StoredPose target;
  target.x = 1.03;
  target.y = 2.0;
  target.yaw = 0.2;
  const auto check = module_->verify_navigation_final_pose(target, true);

  EXPECT_TRUE(check.pose_available);
  EXPECT_TRUE(check.position_reached);
  EXPECT_NEAR(check.distance_m, 0.03, 1e-9);
  EXPECT_NEAR(check.yaw_error_rad, 0.0, 1e-9);
}

TEST_F(NavigationGoalExecutionModuleTest, RequestsAmclOnlyWhenBridgeNeedsFreshCorrection)
{
  make_module();
  std::string detail;
  module_->request_bridge_wait_nomotion_update(bridge_snapshot_, detail);
  EXPECT_EQ(nomotion_request_count_, 0);
  EXPECT_NE(detail.find("skipped"), std::string::npos);

  bridge_snapshot_.available = true;
  bridge_snapshot_.amcl_input_enabled = true;
  bridge_snapshot_.amcl_process_ready = true;
  bridge_snapshot_.amcl_seeded = true;
  bridge_snapshot_.amcl_tracking_ready = true;
  bridge_snapshot_.amcl_correction_pending = true;
  module_->request_bridge_wait_nomotion_update(bridge_snapshot_, detail);
  EXPECT_EQ(nomotion_request_count_, 1);
  EXPECT_EQ(detail, "requested");
}

TEST_F(NavigationGoalExecutionModuleTest, PredockOwnerBlocksOrdinaryFinalYaw)
{
  make_module();
  ASSERT_TRUE(module_->try_acquire_predock_motion_owner(false));

  navigation::FinalPoseCheck initial;
  initial.yaw_error_rad = 0.2;
  const auto result = module_->run_final_yaw_align(
    1U, robot_api_server::StoredPose{}, initial);

  EXPECT_TRUE(result.blocked);
  EXPECT_EQ(result.blocked_reason, "predock_yaw_align_active");
  EXPECT_NE(result.detail.find("PREDOCK_YAW_ALIGN owns /cmd_vel_docking"), std::string::npos);
  module_->release_predock_motion_owner();
}

TEST_F(NavigationGoalExecutionModuleTest, FinishingMissionUpdatesRuntimeState)
{
  make_module();
  navigation::NavigationGoalJobStartSpec start;
  start.started_at = "2026-09-03T00:00:00Z";
  const auto job_id = mission_runtime_->start(start, [](std::uint64_t) {});
  mission_runtime_->join();

  module_->finish_navigation_goal_job(
    job_id, true, "succeeded", "goal complete", 0.01, 0.01, 1,
    true, true, false, false, false);

  const auto job = mission_runtime_->snapshot(job_id);
  ASSERT_TRUE(job.has_value());
  EXPECT_EQ(job->state, "succeeded");
  EXPECT_EQ(job->completed_at, "2026-09-03T00:00:00Z");
  EXPECT_EQ(runtime_state_, "ready");
  EXPECT_EQ(runtime_detail_, "goal complete");
  EXPECT_TRUE(runtime_healthy_);
}

TEST_F(NavigationGoalExecutionModuleTest, RecoveryStatusIsBoundToRealAcceptedAction)
{
  using Action = navigation::NavigationActionRuntime::NavigateToPose;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> accepted;
  auto server = rclcpp_action::create_server<Action>(node_, execution_config_.action_name,
    [](const auto &, const auto &) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
    [&](const auto & handle) {accepted = handle;});
  auto publisher = node_->create_publisher<std_msgs::msg::String>(
    "/navigation/ordinary_recovery_status", rclcpp::QoS(1).reliable());
  ASSERT_TRUE(action_runtime_->wait_for_action_server(std::chrono::seconds(2)));
  Action::Goal goal;
  goal.pose.header.stamp = node_->now();
  const auto stamp = rclcpp::Time(goal.pose.header.stamp).nanoseconds();
  auto future = action_runtime_->client()->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  auto handle = future.get();
  ASSERT_TRUE(handle);
  action_runtime_->track_goal(handle, "test", "", "", stamp);
  auto emit = [&](std::int64_t goal_stamp) {
    std_msgs::msg::String msg;
    msg.data = "{\"version\":1,\"goal_stamp_ns\":" + std::to_string(goal_stamp) +
      ",\"stamp_ns\":" + std::to_string(node_->now().nanoseconds()) +
      ",\"phase\":\"waiting\"}";
    publisher->publish(msg);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node_);
  };
  for (int i = 0; i < 20 && publisher->get_subscription_count() == 0; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  emit(stamp - 1);
  EXPECT_TRUE(action_runtime_->recovery_phase(handle).empty());
  emit(stamp);
  EXPECT_EQ(action_runtime_->recovery_phase(handle), "waiting");
  action_runtime_->track_goal(handle, "replacement", "", "", stamp + 1);
  emit(stamp);
  EXPECT_TRUE(action_runtime_->recovery_phase(handle).empty());
  ASSERT_TRUE(accepted);
  accepted->abort(std::make_shared<Action::Result>());
  action_runtime_->mark_terminal_proven(handle, true);
  EXPECT_TRUE(action_runtime_->recovery_phase(handle).empty());
}

}  // namespace
