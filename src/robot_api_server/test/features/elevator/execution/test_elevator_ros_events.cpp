#include <atomic>
#include <mutex>
#include <thread>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_interfaces/action/floor_switch.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "robot_api_server/features/elevator/execution/elevator_ros_executor.hpp"

using namespace std::chrono_literals;
using robot_api_server::ElevatorRosExecutor;
namespace {
// Test-only injection at the real client's executor take_data boundary, before
// consuming the pending event. No access to private queues or ready flags.
template<typename Action>
class InjectingClient : public rclcpp_action::Client<Action> {
public:
  InjectingClient(rclcpp::Node::SharedPtr node, const std::string & name)
  : rclcpp_action::Client<Action>(node->get_node_base_interface(),
      node->get_node_graph_interface(), node->get_node_logging_interface(), name) {}
  std::atomic<bool> inject{false};
  std::atomic<bool> unknown{false};
  std::atomic<int> takes{0};
  std::shared_ptr<void> take_data() override {
    ++takes;
    if (unknown.exchange(false)) {throw std::logic_error("test unknown take_data failure");}
    if (inject.exchange(false)) {
      throw std::runtime_error("Taking data from action client but no ready event");
    }
    return rclcpp_action::Client<Action>::take_data();
  }
};

template<typename Predicate>
bool eventually(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  do {
    if (predicate()) return true;
    std::this_thread::sleep_for(2ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

template<typename Action>
struct Rig {
  using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;
  std::shared_ptr<rclcpp::Context> context = std::make_shared<rclcpp::Context>();
  rclcpp::Node::SharedPtr client_node, server_node;
  std::shared_ptr<InjectingClient<Action>> client;
  typename rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ce, se;
  std::unique_ptr<ElevatorRosExecutor> cw, sw;
  std::atomic<int> goals{0}, cancels{0}, service_calls{0};
  std::mutex mutex;
  std::shared_ptr<ServerHandle> accepted;
  explicit Rig(const std::string & name) {
    context->init(0, nullptr);
    rclcpp::NodeOptions no; no.context(context);
    client_node=std::make_shared<rclcpp::Node>("elevator_test_adapter", no);
    server_node=std::make_shared<rclcpp::Node>("mock_robot_servers", no);
    client=std::make_shared<InjectingClient<Action>>(client_node, name);
    client_node->get_node_waitables_interface()->add_waitable(client, nullptr);
    server=rclcpp_action::create_server<Action>(server_node, name,
      [&](const auto &, auto) {++goals; return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;},
      [&](auto) {++cancels; return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {std::lock_guard<std::mutex> lock(mutex); accepted=handle;});
    service=server_node->create_service<std_srvs::srv::Trigger>("/mock_service",
      [&](std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        ++service_calls; response->success=true; response->message="original service";});
    service_client=client_node->create_client<std_srvs::srv::Trigger>("/mock_service");
    rclcpp::ExecutorOptions eo; eo.context=context;
    ce=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    // ExecutorOptions owns a MemoryStrategy shared_ptr: two executors must
    // not share the same wait-set bookkeeping, even on the same Context.
    rclcpp::ExecutorOptions server_options; server_options.context=context;
    se=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(server_options);
    ce->add_node(client_node); se->add_node(server_node);
    cw=std::make_unique<ElevatorRosExecutor>(*ce, context);
    sw=std::make_unique<ElevatorRosExecutor>(*se, context);
    cw->start(); sw->start();
  }
  std::shared_ptr<ServerHandle> handle() {std::lock_guard<std::mutex> lock(mutex); return accepted;}
  ~Rig() {
    cw->stop(); sw->stop();
    client_node->get_node_waitables_interface()->remove_waitable(client, nullptr);
    ce->remove_node(client_node); se->remove_node(server_node);
    context->shutdown("fixture complete");
  }
};

template<typename Action>
void retained_goal_flow(const std::string & name, bool cancel) {
  Rig<Action> r(name);
  ASSERT_TRUE(r.client->wait_for_action_server(5s));
  auto * original_client=r.client.get();
  std::atomic<int> feedback{0}, results{0}, responses{0};
  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback=[&](auto h) {EXPECT_TRUE(h); ++responses;};
  options.feedback_callback=[&](auto, auto) {++feedback;};
  options.result_callback=[&](const auto &) {++results;};
  r.client->inject=true; // pending goal response/status, not business callback
  auto sent=r.client->async_send_goal(typename Action::Goal{}, options);
  ASSERT_EQ(r.cw->wait_for(sent, 5s), std::future_status::ready);
  auto goal=sent.get(); ASSERT_TRUE(goal);
  ASSERT_TRUE(eventually([&] {return bool(r.handle());}));
  auto uuid=goal->get_goal_id();
  EXPECT_EQ(uuid, r.handle()->get_goal_id());
  r.handle()->execute(); // real status transition after the client owns its handle
  auto result=r.client->async_get_result(goal);
  const auto errors_before=r.cw->known_error_count();
  r.client->inject=true;
  r.handle()->publish_feedback(std::make_shared<typename Action::Feedback>());
  ASSERT_TRUE(eventually([&] {return feedback>0 && r.cw->known_error_count()>errors_before;}));
  EXPECT_TRUE(eventually([&] {
    return goal->get_status()==action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }));
  EXPECT_EQ(result.wait_for(0ms), std::future_status::timeout);
  ASSERT_TRUE(r.service_client->wait_for_service(5s));
  auto service=r.service_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(r.cw->wait_for(service, 5s), std::future_status::ready);
  EXPECT_TRUE(service.get()->success); EXPECT_EQ(r.service_calls, 1);
  const auto errors_before_terminal=r.cw->known_error_count();
  r.client->inject=true;
  if (cancel) {
    // The ONLY cancellation is this explicit test request.
    auto canceled=r.client->async_cancel_goal(goal);
    ASSERT_EQ(r.cw->wait_for(canceled, 5s), std::future_status::ready);
    ASSERT_EQ(canceled.get()->goals_canceling.size(), 1u);
    ASSERT_TRUE(eventually([&] {return r.handle()->is_canceling();}));
    r.handle()->canceled(std::make_shared<typename Action::Result>());
  } else {
    r.handle()->succeed(std::make_shared<typename Action::Result>());
  }
  ASSERT_EQ(r.cw->wait_for(result, 5s), std::future_status::ready);
  auto wrapped=result.get();
  EXPECT_EQ(wrapped.goal_id, uuid);
  EXPECT_EQ(wrapped.code, cancel ? rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(eventually([&] {return results==1;}));
  EXPECT_EQ(responses, 1); EXPECT_EQ(results, 1);
  EXPECT_EQ(r.client.get(), original_client); EXPECT_EQ(goal->get_goal_id(), uuid);
  EXPECT_EQ(r.goals, 1); EXPECT_EQ(r.cancels, cancel ? 1 : 0);
  EXPECT_GT(r.cw->known_error_count(), errors_before_terminal);
  EXPECT_FALSE(r.cw->failure());
  r.cw->stop(); r.cw->stop();
  EXPECT_EQ(goal->get_goal_id(), uuid);
}

TEST(ElevatorRosEvents, NavigationGoalSurvivesTakeDataExceptions) {
  retained_goal_flow<nav2_msgs::action::NavigateToPose>("/mock_nav", false);
}
TEST(ElevatorRosEvents, ExplicitCancelIsNotDuplicatedByRecovery) {
  retained_goal_flow<nav2_msgs::action::NavigateToPose>("/mock_nav_cancel", true);
}
TEST(ElevatorRosEvents, FloorSwitchGoalSurvivesTakeDataExceptions) {
  retained_goal_flow<robot_interfaces::action::FloorSwitch>("/mock_floor", false);
}
TEST(ElevatorRosEvents, FloorSwitchExplicitCancelSurvivesTakeDataExceptions) {
  retained_goal_flow<robot_interfaces::action::FloorSwitch>("/mock_floor_cancel", true);
}
TEST(ElevatorRosEvents, IdleElevatorClientConsumesForeignNavigationStatusWithoutGoals) {
  using Action=nav2_msgs::action::NavigateToPose;
  Rig<Action> r("/mock_shared_navigation");
  auto ordinary=rclcpp_action::create_client<Action>(r.client_node, "/mock_shared_navigation");
  ASSERT_TRUE(ordinary->wait_for_action_server(5s));
  std::atomic<int> feedback{0};
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback=[&](auto, auto) {++feedback;};
  r.client->inject=true;
  auto sent=ordinary->async_send_goal(Action::Goal{}, options);
  ASSERT_EQ(r.cw->wait_for(sent, 5s), std::future_status::ready);
  auto goal=sent.get(); ASSERT_TRUE(goal);
  ASSERT_TRUE(eventually([&] {return bool(r.handle());}));
  auto result=ordinary->async_get_result(goal);
  r.handle()->execute();
  for (int i=0; i<10; ++i) {
    r.handle()->publish_feedback(std::make_shared<Action::Feedback>());
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(eventually([&] {return feedback>=5 && r.cw->known_error_count()>0;}));
  r.handle()->succeed(std::make_shared<Action::Result>());
  ASSERT_EQ(r.cw->wait_for(result, 5s), std::future_status::ready);
  EXPECT_EQ(result.get().goal_id, goal->get_goal_id());
  EXPECT_EQ(r.goals, 1); EXPECT_EQ(r.cancels, 0);
  EXPECT_FALSE(r.cw->failure());
}

TEST(ElevatorRosEvents, UnknownTakeDataErrorWakesWaitWithoutInventingRemoteTerminal) {
  using Action=nav2_msgs::action::NavigateToPose;
  Rig<Action> r("/mock_unknown_failure");
  ASSERT_TRUE(r.client->wait_for_action_server(5s));
  auto sent=r.client->async_send_goal(Action::Goal{});
  ASSERT_EQ(r.cw->wait_for(sent, 5s), std::future_status::ready);
  auto goal=sent.get(); ASSERT_TRUE(goal);
  ASSERT_TRUE(eventually([&] {return bool(r.handle());}));
  r.handle()->execute();
  auto result=r.client->async_get_result(goal);
  const auto uuid=goal->get_goal_id();
  r.client->unknown=true;
  r.handle()->publish_feedback(std::make_shared<Action::Feedback>());
  EXPECT_THROW(r.cw->wait_for(result, 5s), robot_api_server::ElevatorRosExecutorUnavailable);
  EXPECT_TRUE(r.cw->failure());
  EXPECT_EQ(result.wait_for(0ms), std::future_status::timeout);
  EXPECT_EQ(goal->get_goal_id(), uuid);
  EXPECT_EQ(r.goals, 1); EXPECT_EQ(r.cancels, 0);
  r.cw->stop(); EXPECT_TRUE(r.context->is_valid());
  // Mock server cleanup only, NOT a recovery cancellation or client success.
  r.handle()->abort(std::make_shared<Action::Result>());
  EXPECT_EQ(result.wait_for(0ms), std::future_status::timeout);
}

TEST(ElevatorRosEvents, StopWakesPendingResultAndDoesNotCancelOrShutdownSharedContext) {
  using Action=nav2_msgs::action::NavigateToPose;
  Rig<Action> r("/mock_shutdown");
  ASSERT_TRUE(r.client->wait_for_action_server(5s));
  auto sent=r.client->async_send_goal(Action::Goal{});
  ASSERT_EQ(r.cw->wait_for(sent, 5s), std::future_status::ready);
  auto goal=sent.get(); ASSERT_TRUE(goal);
  ASSERT_TRUE(eventually([&] {return bool(r.handle());}));
  r.handle()->execute();
  auto result=r.client->async_get_result(goal);
  auto waiter=std::async(std::launch::async, [&] {
    try {r.cw->wait_for(result, 60s); return false;}
    catch (const robot_api_server::ElevatorRosExecutorUnavailable &) {return true;}
  });
  r.cw->stop();
  ASSERT_EQ(waiter.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(waiter.get()); EXPECT_TRUE(r.context->is_valid());
  EXPECT_EQ(r.goals, 1); EXPECT_EQ(r.cancels, 0);
  r.handle()->abort(std::make_shared<Action::Result>());
}
}  // namespace
