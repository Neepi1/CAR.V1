#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"
#include "robot_api_server/features/elevator/execution/elevator_ros_executor.hpp"

namespace robot_api_server
{
namespace
{
using namespace std::chrono_literals;

// HTTP is the external boundary. No arm port or vehicle service is contacted.
class ArmFaultTransport final : public ElevatorArmHttpTransport
{
public:
  std::atomic<bool> & trigger;
  bool recover_after_known_error;
  int ready_posts{0}, button_posts{0}, release_posts{0};
  ArmFaultTransport(std::atomic<bool> & signal, bool recover)
  : trigger(signal), recover_after_known_error(recover) {}

  ElevatorArmHttpResponse request(const std::string & method,
    const std::string & path, const std::string &, std::chrono::milliseconds) override
  {
    if (method == "POST") {
      if (path == "/api/v1/arm/ready") {
        ++ready_posts;
        return {202, R"({"task_id":"ready"})", {}};
      }
      if (path == "/api/v1/arm/release") {
        ++release_posts;
        return {202, R"({"task_id":"release"})", {}};
      }
      ++button_posts;
      return {202, R"({"task_id":"button"})", {}};
    }
    if (path == "/api/v1/tasks/ready" &&
      (!recover_after_known_error || ready_posts == 1))
    {
      trigger.store(true);
      return {200, R"({"state":"failed","error_code":"execution_failed"})", {}};
    }
    return {200, R"({"state":"succeeded"})", {}};
  }
};

TEST(ElevatorArmRuntimeFault, RealWorkerFailureInterruptsTheActiveArmRetry)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  node_options.start_parameter_services(false).start_parameter_event_publisher(false);
  auto node = std::make_shared<rclcpp::Node>("arm_fault_test", "/isolated", node_options);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  std::atomic<bool> trigger{false};
  auto timer = node->create_wall_timer(2ms, [&] {
    if (trigger.exchange(false)) {throw std::logic_error("injected adapter callback fault");}
  });
  ElevatorRosExecutor worker(executor, context);
  auto transport = std::make_shared<ArmFaultTransport>(trigger, false);
  ElevatorArmClientOptions options;
  options.request_timeout = 50ms;
  options.task_timeout = 100ms;
  options.poll_interval = 0ms;
  ElevatorArmClient arm(options, transport);
  worker.start();
  const auto start = std::chrono::steady_clock::now();
  const auto result = arm.press_floor("worker-fault", 1U, "F2",
    [&] {return std::chrono::steady_clock::now() - start > 2s;},
    [&]() -> std::optional<ElevatorArmOutcome> {
      if (!worker.unavailable()) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "ROS runtime executor is unavailable", {}};
    });
  worker.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_EQ(transport->ready_posts, 1);
  EXPECT_EQ(transport->button_posts, 0);
  EXPECT_EQ(transport->release_posts, 1);
  EXPECT_TRUE(context->is_valid());
  ASSERT_TRUE(worker.failure());
  try {std::rethrow_exception(worker.failure());}
  catch (const std::logic_error & error) {
    EXPECT_STREQ(error.what(), "injected adapter callback fault");
  }
  executor.remove_node(node);
  context->shutdown("isolated test complete");
}

TEST(ElevatorArmRuntimeFault, RecoveredKnownExecutorErrorDoesNotStopArmRetries)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  node_options.start_parameter_services(false).start_parameter_event_publisher(false);
  auto node = std::make_shared<rclcpp::Node>("arm_known_error_test", "/isolated", node_options);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  std::atomic<bool> trigger{false};
  auto timer = node->create_wall_timer(2ms, [&] {
    if (trigger.exchange(false)) {
      // Worker-classification integration, NOT a claim to reproduce the DDS queue bug.
      throw std::runtime_error("Taking data from action client but no ready event");
    }
  });
  ElevatorRosExecutor worker(executor, context);
  auto transport = std::make_shared<ArmFaultTransport>(trigger, true);
  ElevatorArmClientOptions options;
  options.request_timeout = 50ms;
  options.task_timeout = 100ms;
  options.poll_interval = 0ms;
  ElevatorArmClient arm(options, transport);
  worker.start();
  const auto start = std::chrono::steady_clock::now();
  const auto result = arm.press_hall_call("worker-recovered", 1U, "F1", "F2",
    [&] {return std::chrono::steady_clock::now() - start > 3s;},
    [&]() -> std::optional<ElevatorArmOutcome> {
      if (!worker.unavailable()) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "ROS runtime executor is unavailable", {}};
    });
  EXPECT_FALSE(worker.unavailable());
  worker.stop();
  EXPECT_TRUE(result.succeeded()) << result.code << ": " << result.detail;
  EXPECT_EQ(worker.known_error_count(), 1U);
  EXPECT_FALSE(worker.failure());
  EXPECT_EQ(transport->ready_posts, 2);
  EXPECT_EQ(transport->button_posts, 1);
  EXPECT_EQ(transport->release_posts, 1);
  executor.remove_node(node);
  context->shutdown("isolated test complete");
}
}  // namespace
}  // namespace robot_api_server
