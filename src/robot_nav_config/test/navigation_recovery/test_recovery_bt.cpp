#include <functional>
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robot_nav_config/navigation_recovery/recovery_state.hpp"
#include "robot_nav_config/navigation_recovery/recovery_loop.hpp"
#include "robot_nav_config/srv/prepare_ordinary_navigation_recovery.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>

namespace
{
TEST(RecoveryPlugin, LoadsAllProductionAdaptersAgainstInstalledHumble)
{
  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(RECOVERY_BT_PLUGIN));
  EXPECT_EQ(factory.manifests().count("OrdinaryFollowPath"), 1U);
  EXPECT_EQ(factory.manifests().count("OrdinaryFollowAborted"), 1U);
  EXPECT_EQ(factory.manifests().count("PrepareOrdinaryRecovery"), 1U);
  EXPECT_EQ(factory.manifests().count("OrdinaryPathTargetsGoal"), 1U);
  EXPECT_EQ(factory.manifests().count("OrdinaryRecoveryLoop"), 1U);
}

TEST(RecoveryPlugin, RealPathSelectorRetainsPreparedPathUntilEndpointChanges)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(RECOVERY_BT_PLUGIN);
  auto bb = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.poses.push_back(goal);
  bb->set("path", path);
  bb->set("goal", goal);
  auto tree = factory.createTreeFromText(
    R"(<root><BehaviorTree ID="Main"><OrdinaryPathTargetsGoal path="{path}" goal="{goal}"/></BehaviorTree></root>)", bb);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  goal.header.stamp.sec = 10;
  bb->set("goal", goal);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  goal.pose.position.y = 1.0;
  bb->set("goal", goal);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

class Leaf : public BT::ActionNodeBase
{
public:
  Leaf(const std::string & name, const BT::NodeConfiguration & config,
    std::function<BT::NodeStatus(BT::TreeNode &)> fn, int * halted)
  : ActionNodeBase(name, config), fn_(std::move(fn)), halted_(halted) {}
  BT::NodeStatus tick() override {return fn_(*this);}
  void halt() override
  {
    if (status() == BT::NodeStatus::RUNNING) {++*halted_;}
    setStatus(BT::NodeStatus::IDLE);
  }
private:
  std::function<BT::NodeStatus(BT::TreeNode &)> fn_;
  int * halted_;
};

TEST(RecoveryPlugin, RealAsyncInspectorPublishesExplicitWaitingWithoutPreparingMotion)
{
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  auto node = std::make_shared<rclcpp::Node>("ordinary_recovery_monitor_test");
  using Prepare = robot_nav_config::srv::PrepareOrdinaryNavigationRecovery;
  int inspections = 0, halted = 0;
  auto server = node->create_service<Prepare>(
    "/controller_server/prepare_ordinary_navigation_recovery",
    [&](const Prepare::Request::SharedPtr request, Prepare::Response::SharedPtr response) {
      EXPECT_TRUE(request->inspect_only);
      EXPECT_EQ(request->path.poses.size(), 1U);
      ++inspections;
      response->matched = true;
      response->waiting = true;
    });
  bool saw_waiting = false;
  auto subscriber = node->create_subscription<std_msgs::msg::String>(
    "/navigation/ordinary_recovery_status", rclcpp::QoS(1).reliable(),
    [&](const std_msgs::msg::String::SharedPtr msg) {
      saw_waiting = saw_waiting || msg->data.find("\"phase\":\"waiting\"") != std::string::npos;
    });
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(RECOVERY_BT_PLUGIN);
  factory.registerBuilder({BT::NodeType::ACTION, "KeepRunning", {}},
    [&](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<Leaf>(name, config,
        [](BT::TreeNode & leaf) {
          leaf.config().blackboard->set("ordinary_attempt_started",
            leaf.config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal").header.stamp);
          return BT::NodeStatus::RUNNING;
        }, &halted);
    });
  auto bb = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = node->now();
  goal.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.poses = {goal, goal};
  bb->set("node", node);
  bb->set("goal", goal);
  bb->set("path", path);
  bb->set("ordinary_attempt_started", goal.header.stamp);
  auto tree = factory.createTreeFromText(
    R"(<root><BehaviorTree ID="Main"><OrdinaryRecoveryLoop><KeepRunning/><AlwaysSuccess/></OrdinaryRecoveryLoop></BehaviorTree></root>)", bb);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!saw_waiting && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(saw_waiting);
  EXPECT_GT(inspections, 0);
  EXPECT_LE(inspections, 6);
  tree.haltTree();
  EXPECT_EQ(halted, 1);
}

class RecoveryTree : public ::testing::Test
{
protected:
  class Monitor : public robot_nav_config::navigation_recovery::RecoveryMonitor {
  public:
    explicit Monitor(RecoveryTree & owner) : owner_(owner) {}
    double now() const override {return owner_.clock;}
    void invalidate() override {}
    std::optional<robot_nav_config::navigation_recovery::RecoveryState::Observation> sample(
      const nav_msgs::msg::Path & path, const builtin_interfaces::msg::Time & started) override
    {
      if (owner_.missing_evidence) {return std::nullopt;}
      return owner_.state.inspect(path, static_cast<std::int64_t>(started.sec) * 1000000000LL,
        1000000000000LL);
    }
    void report(const geometry_msgs::msg::PoseStamped &, const std::string & phase,
      unsigned retries) override {owner_.phase = phase; owner_.retry_count = retries;}
  private:
    RecoveryTree & owner_;
  };
  void SetUp() override
  {
    state.set_active(true);
    factory.registerBuilder<robot_nav_config::navigation_recovery::OrdinaryRecoveryLoop>(
      "OrdinaryRecoveryLoop", [this](const std::string & n, const BT::NodeConfiguration & c) {
        return std::make_unique<robot_nav_config::navigation_recovery::OrdinaryRecoveryLoop>(
          n, c, std::make_unique<Monitor>(*this));
      });
    for (const auto * plugin : {"pipeline_sequence", "rate_controller"}) {
      factory.registerFromPlugin(std::string("/opt/ros/humble/lib/libnav2_") + plugin + "_bt_node.so");
    }
    add("ComputePathToPose", [&](BT::TreeNode & n) {
        ++plans;
        if (planner_fails) {return BT::NodeStatus::FAILURE;}
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp.sec = plans * 10;
        path.poses.resize(2);
        path.poses.back() = goal;
        n.setOutput("path", path);
        return BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
          BT::OutputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("planner_id")});
    add("GlobalUpdatedGoal", [](BT::TreeNode &) {return BT::NodeStatus::FAILURE;}, {});
    add("IsPathValid", [&](BT::TreeNode & n) {
        nav_msgs::msg::Path path;
        n.getInput("path", path);
        return !path.poses.empty() && path.poses.back().pose == goal.pose ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, {BT::InputPort<nav_msgs::msg::Path>("path")});
    add("OrdinaryFollowAborted", [](BT::TreeNode & n) {
        bool aborted = false;
        n.getInput("aborted", aborted);
        return aborted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, {BT::InputPort<bool>("aborted")});
    add("OrdinaryPathTargetsGoal", [](BT::TreeNode & n) {
        nav_msgs::msg::Path path;
        geometry_msgs::msg::PoseStamped target;
        n.getInput("path", path);
        n.getInput("goal", target);
        return !path.poses.empty() && path.poses.back().pose == target.pose ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, {BT::InputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")});
    add("PrepareOrdinaryRecovery", [&](BT::TreeNode & n) {
        ++prepares;
        nav_msgs::msg::Path path;
        builtin_interfaces::msg::Time started;
        n.getInput("path", path);
        n.getInput("attempt_started", started);
        std::string reason;
        return state.prepare(path, static_cast<std::int64_t>(started.sec) * 1000000000LL,
                 reason) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }, {BT::InputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<builtin_interfaces::msg::Time>("attempt_started")});
    add("OrdinaryFollowPath", [&](BT::TreeNode & n) {
        if (n.status() == BT::NodeStatus::IDLE) {
          ++follows;
          nav_msgs::msg::Path path;
          n.getInput("path", path);
          EXPECT_EQ(path.poses.back(), goal);
          if (state.observe_plan(path)) {++rearms;}
          state.reset_progress();
          state.observe_control();
          builtin_interfaces::msg::Time started;
          started.sec = plans * 10;
          n.setOutput("attempt_started", started);
        }
        n.setOutput("aborted", false);
        if (cancelled) {return BT::NodeStatus::FAILURE;}
        if (follows == 1 || twice_failed) {
          if (!unknown_failure) {
            state.observe_progress(false, (plans * 10LL + 5LL) * 1000000000LL);
          }
          n.setOutput("aborted", true);
          return BT::NodeStatus::FAILURE;
        }
        return hold_retry ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
      }, {BT::InputPort<nav_msgs::msg::Path>("path"), BT::InputPort<std::string>("controller_id"),
          BT::InputPort<std::string>("goal_checker_id"), BT::OutputPort<bool>("aborted"),
          BT::OutputPort<builtin_interfaces::msg::Time>("attempt_started")});
    goal.header.frame_id = "map";
    goal.pose.position.x = 2.0;
    goal.pose.orientation.w = 1.0;
    bb = BT::Blackboard::create();
    bb->set("goal", goal);
    tree = factory.createTreeFromFile(RECOVERY_BT_FILE, bb);
  }
  void add(const std::string & name,
    std::function<BT::NodeStatus(BT::TreeNode &)> fn, BT::PortsList ports)
  {
    factory.registerBuilder({BT::NodeType::ACTION, name, ports}, [this, fn](
        const std::string & n, const BT::NodeConfiguration & c) {
        return std::make_unique<Leaf>(n, c, fn, &halted);
      });
  }
  BT::NodeStatus tick()
  {
    BT::NodeStatus result = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 5 && result == BT::NodeStatus::RUNNING; ++i) {result = tree.tickRoot();}
    return result;
  }
  void TearDown() override
  {
    // The monitor reports through this fixture. Destroy the tree before its
    // referenced phase string/counters (declared after tree) are destroyed.
    tree.haltTree();
    tree = BT::Tree{};
  }
  robot_nav_config::navigation_recovery::RecoveryState state;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  BT::Blackboard::Ptr bb;
  geometry_msgs::msg::PoseStamped goal;
  int plans{0}, follows{0}, prepares{0}, rearms{0}, halted{0};
  bool planner_fails{false}, cancelled{false}, unknown_failure{false};
  bool twice_failed{false}, hold_retry{false};
  bool missing_evidence{false};
  double clock{0};
  std::string phase;
  unsigned retry_count{0};
};

TEST_F(RecoveryTree, KeepsOuterTaskRunningThenSucceedsAtSameGoal)
{
  hold_retry = true;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follows, 2);
  EXPECT_EQ(plans, 2);
  EXPECT_EQ(rearms, 1);
  hold_retry = false;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<geometry_msgs::msg::PoseStamped>("goal"), goal);
}

TEST_F(RecoveryTree, SecondFailureKeepsOriginalTaskWaiting)
{
  twice_failed = true;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follows, 2);
  EXPECT_EQ(prepares, 1);
  EXPECT_EQ(phase, "waiting");
  clock = 4.9;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follows, 2);
  twice_failed = false;
  clock = 5.0;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(follows, 3);
  EXPECT_EQ(rearms, 1); // no repeated startup spin at the same blockage
}

TEST_F(RecoveryTree, UnknownControllerFailureDoesNotRestartTracking)
{
  unknown_failure = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(follows, 1);
  EXPECT_EQ(rearms, 0);
}

TEST_F(RecoveryTree, CancellationIsNotRecovery)
{
  cancelled = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans, 1);
  EXPECT_EQ(prepares, 0);
}

TEST_F(RecoveryTree, HaltCancelsActiveRetry)
{
  hold_retry = true;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(halted, 1);
  EXPECT_EQ(follows, 2);
}

TEST_F(RecoveryTree, OldBlackboardAbortCannotRecoverNewPlannerFailure)
{
  bb->set("ordinary_follow_aborted", true);
  planner_fails = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans, 1);
  EXPECT_EQ(follows, 0);
  EXPECT_EQ(prepares, 0);
}

TEST_F(RecoveryTree, ExplicitOuterGoalReplacementReplansInsteadOfFollowingOldEndpoint)
{
  hold_retry = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  goal.pose.position.y = 1.0;
  bb->set("goal", goal);
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(plans, 3);
  EXPECT_EQ(prepares, 1);
  EXPECT_EQ(follows, 3);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path").poses.back(), goal);
}

TEST_F(RecoveryTree, RepeatedBlockageDoesNotBusyLoopOrExhaustTask)
{
  twice_failed = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  for (int i = 0; i < 20; ++i) {
    clock += 10.0;
    ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(follows, 22);
  EXPECT_EQ(rearms, 1);
  twice_failed = false;
  clock += 10.0;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
}

TEST_F(RecoveryTree, HaltDuringWaitNeverSendsDelayedRetry)
{
  twice_failed = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  clock = 100.0;
  EXPECT_EQ(follows, 2);
  EXPECT_EQ(phase, "idle");
}

TEST_F(RecoveryTree, RecoveryPlannerFailureIsNotObstacleWait)
{
  twice_failed = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  planner_fails = true;
  clock = 5.0;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(follows, 2);
}

TEST_F(RecoveryTree, MissingFailureEvidenceTerminatesInsteadOfRetryingBlindly)
{
  missing_evidence = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  clock = 2.1;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(follows, 1);
}

TEST_F(RecoveryTree, RealProgressRenewsRecoveryForASecondBlockageInSameTask)
{
  hold_retry = true;
  ASSERT_EQ(tick(), BT::NodeStatus::RUNNING);
  for (int i = 0; i <= 25; ++i) {
    state.observe_pose(i * 0.01, 0.0, false, 21000000000LL + i * 100000000LL);
  }
  twice_failed = true;
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  twice_failed = false;
  EXPECT_EQ(tick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follows, 3);
  EXPECT_EQ(rearms, 2);
  EXPECT_EQ(retry_count, 1U);
  hold_retry = false;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
}
}  // namespace
