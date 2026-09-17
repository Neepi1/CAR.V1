#include <gtest/gtest.h>

#include <cmath>
#include <atomic>
#include <future>
#include <memory>
#include <string>

#include "rclcpp_action/create_server.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_executor.hpp"

namespace nav = robot_api_server::features::navigation;
using robot_api_server::StoredPose;
using namespace std::chrono_literals;

namespace
{
nav::NavigationCompletionPolicyConfig completion_config()
{
  nav::NavigationCompletionPolicyConfig config;
  config.position_tolerance_m = 0.06;
  config.yaw_tolerance_rad = 0.05;
  config.robot_pose_freshness_sec = 0.5;
  config.final_verify_max_retry_count = 3;
  return config;
}

// Only the robot I/O edge is simulated. The executor, mission, pose evaluator,
// retry policy, terminal geometry and initial Action client/server are real.
class RobotPort : public nav::NavigationGoalExecutionPort
{
public:
  nav::NavigationCompletionPolicy policy{completion_config()};
  nav::NavigationTerminalControl terminal{nav::TerminalControlConfig{}};
  nav::NavigationMissionRuntime & mission;
  robot_api_server::RobotPoseSnapshot pose;
  nav::TerminalLateralCorrectionResult correction;
  nav::NavigationRepositionResult retry;
  double recovered_x{-0.04}, recovered_y{-0.02}, recovered_yaw{0.02};
  double recovered_age{0.0};
  bool stopped{true}, bridge_ready{true}, cancel_on_pose_refresh{false};
  bool retry_bridge_timeout{false}, retry_bridge_canceled{false};
  int corrections{0}, retries{0}, finishes{0};
  mutable int stop_checks{0};
  bool succeeded{false};
  std::string phase, detail;

  explicit RobotPort(nav::NavigationMissionRuntime & runtime) : mission(runtime)
  {
    pose.available = true;
    pose.frame_id = "map";
    pose.x = -0.05;
    pose.y = -0.06;
    pose.age_sec = 0.0;
    correction.attempted = true;
    correction.blocked = true;
    correction.detail = "original terminal correction divergence";
    retry.attempted = retry.succeeded = retry.nav2_succeeded = true;
    retry.nav2_result_code = static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED);
  }

  void clear_navigation_terminal_speed_limit() override {}
  void publish_navigation_terminal_speed_limit_for_goal(const StoredPose &) override {}
  void clear_navigation_terminal_reverse_permit(bool &, const std::string &) override {}
  void update_navigation_terminal_reverse_permit_for_goal(
    const StoredPose &, bool &, std::chrono::steady_clock::time_point &,
    const std::string &) override {}
  bool navigation_goal_cancel_requested(std::uint64_t id, std::string & why) override
  {return mission.cancel_requested(id, why);}
  bool floor_runtime_operation_blocked(
    const std::string &, std::string &, std::string *) const override {return false;}
  bool undock_before_navigation_if_needed(
    const nav::NavigationPreGoalDockSnapshot &, std::string &, bool &) override {return true;}
  bool bridge_safe_for_goal_start(const std::string &, std::string &) const override {return true;}
  bool send_initial_navigation_goal_to_nav2(
    std::uint64_t, const nav2_msgs::action::NavigateToPose::Goal &, const std::string &,
    const std::string &, const std::string &, GoalHandle::SharedPtr &) override {return false;}
  bool cancel_active_navigation_goal(std::string &) override {return true;}
  void publish_final_yaw_align_zero_burst() override {}
  void finish_navigation_goal_job(
    std::uint64_t id, bool success, const std::string & final_phase, const std::string & why,
    double, double, int, bool, bool, bool, bool, bool) override
  {
    ++finishes; succeeded = success; phase = final_phase; detail = why;
    nav::NavigationGoalJobFinishSpec finish;
    finish.succeeded = success; finish.phase = final_phase; finish.detail = why;
    EXPECT_TRUE(mission.finish(id, finish));
  }
  bool maybe_navigation_near_goal_stalled_handoff(
    std::uint64_t, const StoredPose &, const GoalHandle::SharedPtr &,
    std::chrono::steady_clock::time_point, std::chrono::steady_clock::time_point &,
    std::optional<double> &, std::chrono::steady_clock::time_point &,
    const std::string &, std::string &) override {return false;}
  nav::NavigationBridgeWaitResult wait_for_bridge_smoothing_before_final_verify(
    std::uint64_t) override
  {
    nav::NavigationBridgeWaitResult result;
    result.timeout = !bridge_ready || (retries > 0 && retry_bridge_timeout);
    result.canceled = retries > 0 && retry_bridge_canceled;
    result.detail = result.timeout ? "bridge still smoothing" : "bridge ready";
    return result;
  }
  nav::FinalPoseCheck verify_navigation_final_pose(const StoredPose & target, bool fresh) override
  {
    EXPECT_TRUE(fresh);
    if (retries > 0 && cancel_on_pose_refresh) {mission.request_cancel("test cancel after retry");}
    return policy.evaluate_final_pose(pose, target, fresh, "");
  }
  void update_navigation_goal_final_pose_fields(
    std::uint64_t, const nav::FinalPoseCheck &, const std::string &) override {}
  bool post_nav2_terminal_lateral_correction_allowed(
    const StoredPose & target, const nav::FinalPoseCheck & check, const std::string & mode,
    double & forward, double & lateral, std::string & why) const override
  {
    const auto gate = terminal.lateral_correction_gate(target, check, mode);
    forward = gate.forward_m; lateral = gate.lateral_m; why = gate.reason;
    return gate.allowed;
  }
  bool nav2_failed_near_goal_retry_allowed(
    std::uint64_t, const nav::FinalPoseCheck &, const std::string &, double,
    std::string &, std::string &) override {return false;}
  nav::NavigationRepositionResult run_post_nav2_final_verify_retry(
    std::uint64_t id, const StoredPose &, const std::string &, const std::string &) override
  {
    ++retries;
    mission.update(id, [](nav::NavigationGoalJob & job) {++job.final_verify_retry_count;});
    pose.x = recovered_x; pose.y = recovered_y; pose.yaw = recovered_yaw;
    pose.age_sec = recovered_age;
    return retry;
  }
  void update_navigation_goal_final_yaw_fields(
    std::uint64_t, const nav::FinalYawAlignResult &) override {}
  nav::FinalYawAlignResult run_final_yaw_align(
    std::uint64_t, const StoredPose &, const nav::FinalPoseCheck &) override {return {};}
  bool post_nav2_final_verify_acceptance_slack_allowed(
    std::uint64_t id, const nav::FinalPoseCheck & check, std::string & why) override
  {
    const auto decision = policy.final_verify_acceptance_slack(
      check, mission.snapshot(id)->final_verify_retry_count);
    why = decision.detail;
    return decision.allowed;
  }
  nav::TerminalLateralCorrectionResult run_post_nav2_terminal_lateral_correction(
    std::uint64_t, const StoredPose &, const nav::FinalPoseCheck &, const std::string &) override
  {
    ++corrections;
    if (correction.succeeded) {
      pose.x = recovered_x; pose.y = recovered_y; pose.yaw = recovered_yaw;
    }
    return correction;
  }
  bool post_nav2_final_verify_retry_allowed(
    std::uint64_t id, const nav::FinalPoseCheck & check, const std::string & mode,
    std::string & why, std::string & retry_phase) override
  {
    const auto decision = policy.final_verify_retry(
      check, mode, mission.snapshot(id)->final_verify_retry_count);
    why = decision.reason; retry_phase = decision.phase;
    return decision.allowed;
  }
  bool post_nav2_terminal_actual_stop_confirmed(std::string & why) const override
  {
    ++stop_checks;
    why = stopped ? "fresh odometry stable; mode exited" : "odom_stable=false";
    return stopped;
  }
};

class NavigationGoalExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  nav::NavigationMissionRuntime mission;
  RobotPort port{mission};
  nav::NavigationGoalExecutorConfig config;
  static inline int sequence{0};
  int prior_retries{0};
  std::string completion_mode{"pose_required"};

  void run()
  {
    auto node = std::make_shared<rclcpp::Node>("terminal_revalidation_" + std::to_string(++sequence));
    nav::NavigationActionRuntimeConfig action_config;
    action_config.action_name = "/test/terminal_revalidation/goal_" + std::to_string(sequence);
    action_config.status_topic = action_config.action_name + "/_action/status";
    nav::NavigationActionRuntimePorts ports;
    ports.delayed_side_effect_started = []() {};
    ports.delayed_side_effect_resolved = []() {};
    ports.latch_safety_stop = []() {};
    nav::NavigationActionRuntime runtime(*node, action_config, ports);
    using Action = nav2_msgs::action::NavigateToPose;
    auto server = rclcpp_action::create_server<Action>(node, action_config.action_name,
      [](const auto &, const auto &) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [](const auto & handle) {handle->succeed(std::make_shared<Action::Result>());});
    ASSERT_TRUE(runtime.client()->wait_for_action_server(2s));
    auto future = runtime.client()->async_send_goal(Action::Goal{});
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    auto handle = future.get();
    ASSERT_NE(handle, nullptr);
    nav::NavigationGoalJobStartSpec start;
    start.goal_completion_policy = completion_mode;
    const auto id = mission.start(start, [](std::uint64_t) {});
    mission.join();
    mission.update(id, [&](nav::NavigationGoalJob & job) {job.final_verify_retry_count = prior_retries;});
    config.navigation_final_yaw_tolerance_rad = 0.05;
    nav::NavigationGoalExecutor executor(config, runtime, mission, port);
    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::atomic<bool> spinning{true};
    auto worker = std::async(std::launch::async, [&]() {
      while (spinning.load()) {ros_executor.spin_once(20ms);}
    });
    try {
      executor.run(id, handle, StoredPose{});
    } catch (...) {
      spinning.store(false); ros_executor.cancel(); worker.get();
      throw;
    }
    spinning.store(false); ros_executor.cancel(); worker.get();
    EXPECT_EQ(port.finishes, 1);
    EXPECT_EQ(mission.snapshot().task_complete, port.succeeded);
    EXPECT_LE(port.corrections, 1);
    EXPECT_LE(prior_retries + port.retries, 3);
  }
};

TEST_F(NavigationGoalExecutorTest, HistoricalFailureThenRecoveryRevalidatesCurrentState)
{
  run();
  EXPECT_TRUE(port.succeeded) << port.detail;
  EXPECT_EQ(port.phase, "final_pose_verified");
  EXPECT_TRUE(mission.snapshot().final_pose_verified);
  EXPECT_EQ(port.retries, 1);
  EXPECT_EQ(port.corrections, 1);
  EXPECT_GT(port.stop_checks, 0);
  EXPECT_FALSE(port.correction.succeeded);
  EXPECT_FALSE(port.correction.settle_confirmed);
  EXPECT_NE(port.detail.find("attempted=true succeeded=false blocked=true"), std::string::npos);
  EXPECT_NE(port.detail.find("settle_confirmed=false"), std::string::npos);
  EXPECT_NE(port.detail.find("original terminal correction divergence"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, CurrentXYOverrunIsNotMaskedBySuccessfulRecovery)
{
  port.recovered_x = -0.07; port.recovered_y = 0.0;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.retries, 3);
  EXPECT_NE(port.detail.find("current_xy_not_reached"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, StrictLateralBetweenTargetAndTriggerStillFails)
{
  port.recovered_x = -0.01; port.recovered_y = -0.035; port.recovered_yaw = 0.0;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.retries, 1);
  EXPECT_EQ(port.corrections, 1);
  EXPECT_NE(port.detail.find("current_strict_lateral_not_reached"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, StrictYawBelowOrdinaryThresholdStillFails)
{
  port.recovered_yaw = 0.047;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.retries, 1);
  EXPECT_NE(port.detail.find("current_strict_yaw_not_reached"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, RecoveryWithoutActualStopCannotComplete)
{
  port.stopped = false;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_NE(port.detail.find("current_stop_unconfirmed"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, StalePoseAfterRecoveryCannotComplete)
{
  port.recovered_age = 0.51;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.phase, "failed_final_pose_verify");
  EXPECT_NE(port.detail.find("stale"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, CancellationDuringRetryIsTerminal)
{
  port.retry.canceled = true;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.phase, "canceled");
  EXPECT_EQ(port.stop_checks, 0);
}

TEST_F(NavigationGoalExecutorTest, CancellationAfterFreshPoseCannotBecomeSuccess)
{
  port.cancel_on_pose_refresh = true;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.phase, "canceled");
}

TEST_F(NavigationGoalExecutorTest, FailedRetryAtGoodPoseDoesNotEraseHistory)
{
  port.retry.nav2_succeeded = port.retry.succeeded = false;
  port.retry.nav2_result_code = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.stop_checks, 0);
  EXPECT_NE(port.detail.find("terminal_failure_without_successful_recovery"), std::string::npos);
}

TEST_F(NavigationGoalExecutorTest, TimedOutRetryAtGoodPoseDoesNotEraseHistory)
{
  port.retry.nav2_succeeded = port.retry.succeeded = false;
  port.retry.nav2_result_code = 0;
  port.retry.detail = "timed out waiting for post-Nav2 final verify retry result";
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.stop_checks, 0);
}

TEST_F(NavigationGoalExecutorTest, CanceledActionResultIsNotRecoverySuccess)
{
  port.retry.nav2_succeeded = port.retry.succeeded = false;
  port.retry.nav2_result_code = static_cast<int>(rclcpp_action::ResultCode::CANCELED);
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.stop_checks, 0);
}

TEST_F(NavigationGoalExecutorTest, RetryBudgetExhaustedDoesNotAddAttemptOrUseSlack)
{
  prior_retries = 3;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.retries, 0);
  EXPECT_EQ(port.corrections, 1);
}

TEST_F(NavigationGoalExecutorTest, SuccessfulLastPermittedRetryCanStillBeVerified)
{
  prior_retries = 2;
  run();
  EXPECT_TRUE(port.succeeded) << port.detail;
  EXPECT_EQ(port.retries, 1);
}

TEST_F(NavigationGoalExecutorTest, BridgeTimeoutAfterRecoveryCannotComplete)
{
  port.retry_bridge_timeout = true;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.retries, 1);
}

TEST_F(NavigationGoalExecutorTest, CancellationInBridgeWaitRemainsCancellation)
{
  port.retry_bridge_canceled = true;
  run();
  EXPECT_FALSE(port.succeeded);
  EXPECT_EQ(port.phase, "canceled");
}

TEST_F(NavigationGoalExecutorTest, OriginalDirectSuccessDoesNotRunCorrectionOrStopCheck)
{
  port.pose.x = -0.02; port.pose.y = 0.0;
  run();
  EXPECT_TRUE(port.succeeded);
  EXPECT_EQ(port.corrections, 0);
  EXPECT_EQ(port.retries, 0);
  EXPECT_EQ(port.stop_checks, 0);
}

TEST_F(NavigationGoalExecutorTest, OriginalSuccessfulCorrectionRemainsSuccessful)
{
  port.correction.succeeded = port.correction.settle_confirmed = true;
  run();
  EXPECT_TRUE(port.succeeded);
  EXPECT_EQ(port.corrections, 1);
  EXPECT_EQ(port.retries, 0);
  EXPECT_EQ(port.stop_checks, 0);
}

TEST_F(NavigationGoalExecutorTest, PositionOnlyNormalPathRemainsUnchanged)
{
  completion_mode = "position_only";
  port.pose.x = -0.02; port.pose.y = 0.0; port.pose.yaw = 0.5;
  run();
  EXPECT_TRUE(port.succeeded);
  EXPECT_EQ(port.corrections, 0);
  EXPECT_EQ(port.retries, 0);
}
}  // namespace
