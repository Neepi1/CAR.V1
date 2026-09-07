#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/dock_target_observation.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"

namespace docking = robot_api_server::features::docking;
using namespace std::chrono_literals;

namespace
{

class DockingRuntimeModuleTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>(
      "docking_runtime_module_test_" + std::to_string(++sequence_));
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void TearDown() override
  {
    module_.reset();
    node_.reset();
  }

  void spin_until(const std::function<bool()> & predicate)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor.remove_node(node_);
  }

  static inline int sequence_{0};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unique_ptr<docking::DockingRuntimeModule> module_;
};

docking::DockingRuntimeConfig test_config(const int sequence)
{
  const std::string suffix = std::to_string(sequence);
  docking::DockingRuntimeConfig config;
  config.status_topic = "/test/docking_status_" + suffix;
  config.observation_backend = "target_observation";
  config.target_observation_topic = "/test/dock_target_" + suffix;
  config.target_observation_source = "expected_camera";
  config.command_topic = "/cmd_vel_docking";
  config.forced_mode_topic = "/test/forced_mode_" + suffix;
  config.start_service = "/test/docking_start_" + suffix;
  config.stop_service = "/test/docking_stop_" + suffix;
  config.undock_service = "/test/docking_undock_" + suffix;
  return config;
}

TEST_F(DockingRuntimeModuleTest, FiltersTargetObservationAndForwardsStatus)
{
  const auto config = test_config(sequence_);

  std::mutex status_mutex;
  std::vector<std::string> statuses;
  docking::DockingRuntimePorts ports;
  ports.on_status = [&](const std::string & status) {
      std::lock_guard<std::mutex> lock(status_mutex);
      statuses.push_back(status);
    };
  module_ = std::make_unique<docking::DockingRuntimeModule>(
    *node_, callback_group_, config, std::move(ports));

  auto status_pub = node_->create_publisher<std_msgs::msg::String>(
    config.status_topic, rclcpp::QoS(10).transient_local());
  auto target_pub = node_->create_publisher<robot_interfaces::msg::DockTargetObservation>(
    config.target_observation_topic, rclcpp::QoS(5).reliable());
  spin_until(
    [&]() {
      return status_pub->get_subscription_count() == 1U &&
      target_pub->get_subscription_count() == 1U;
    });
  ASSERT_EQ(status_pub->get_subscription_count(), 1U);
  ASSERT_EQ(target_pub->get_subscription_count(), 1U);

  std_msgs::msg::String status;
  status.data = "aligning points=21";
  status_pub->publish(status);

  robot_interfaces::msg::DockTargetObservation observation;
  observation.source = "wrong_camera";
  observation.sensor_healthy = true;
  observation.valid = true;
  observation.reason = "must_be_ignored";
  target_pub->publish(observation);
  spin_until(
    [&]() {
      std::lock_guard<std::mutex> lock(status_mutex);
      return !statuses.empty();
    });
  EXPECT_LT(module_->observation_snapshot().age_sec, 0.0);

  observation.source = "expected_camera";
  observation.reason = "target_locked";
  target_pub->publish(observation);
  spin_until([&]() {return module_->observation_snapshot().age_sec >= 0.0;});

  const auto snapshot = module_->observation_snapshot();
  EXPECT_TRUE(snapshot.usable);
  EXPECT_EQ(snapshot.detail, "target_locked");
  std::lock_guard<std::mutex> lock(status_mutex);
  ASSERT_EQ(statuses.size(), 1U);
  EXPECT_EQ(statuses.front(), "aligning points=21");
}

TEST_F(DockingRuntimeModuleTest, PublishesMotionAdaptersAndOwnsSingleWorker)
{
  const auto config = test_config(sequence_);
  std::mutex message_mutex;
  geometry_msgs::msg::Twist received_command;
  std::string received_mode;
  bool have_command = false;
  bool have_mode = false;
  auto command_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    config.command_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(message_mutex);
      received_command = *msg;
      have_command = true;
    });
  auto mode_sub = node_->create_subscription<std_msgs::msg::String>(
    config.forced_mode_topic,
    rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(message_mutex);
      received_mode = msg->data;
      have_mode = true;
    });

  std::mutex worker_mutex;
  std::condition_variable worker_cv;
  bool worker_started = false;
  bool release_worker = false;
  std::uint64_t observed_job_id = 0U;
  docking::DockingRuntimePorts ports;
  ports.run_job = [&](const std::uint64_t job_id) {
      std::unique_lock<std::mutex> lock(worker_mutex);
      observed_job_id = job_id;
      worker_started = true;
      worker_cv.notify_all();
      worker_cv.wait(lock, [&]() {return release_worker;});
    };
  module_ = std::make_unique<docking::DockingRuntimeModule>(
    *node_, callback_group_, config, std::move(ports));

  spin_until(
    [&]() {
      return command_sub->get_publisher_count() == 1U &&
      mode_sub->get_publisher_count() == 1U;
    });
  geometry_msgs::msg::Twist command;
  command.linear.y = 0.031;
  command.angular.z = -0.17;
  module_->publish_command(command);
  module_->publish_forced_mode("side_slip");
  spin_until(
    [&]() {
      std::lock_guard<std::mutex> lock(message_mutex);
      return have_command && have_mode;
    });

  std::string launch_error;
  EXPECT_TRUE(module_->launch_worker(42U, launch_error)) << launch_error;
  {
    std::unique_lock<std::mutex> lock(worker_mutex);
    EXPECT_TRUE(worker_cv.wait_for(lock, 2s, [&]() {return worker_started;}));
  }
  std::string duplicate_error;
  EXPECT_FALSE(module_->launch_worker(43U, duplicate_error));
  EXPECT_EQ(duplicate_error, "docking worker is already joinable");
  {
    std::lock_guard<std::mutex> lock(worker_mutex);
    release_worker = true;
  }
  worker_cv.notify_all();
  module_->join_worker();

  {
    std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_TRUE(have_command);
    EXPECT_TRUE(have_mode);
    EXPECT_DOUBLE_EQ(received_command.linear.y, 0.031);
    EXPECT_DOUBLE_EQ(received_command.angular.z, -0.17);
    EXPECT_EQ(received_mode, "side_slip");
  }
  EXPECT_EQ(observed_job_id, 42U);
}

TEST_F(DockingRuntimeModuleTest, TreatsGs2AsFreshnessEvidenceOnly)
{
  auto config = test_config(sequence_);
  config.observation_backend = "gs2_scan";
  config.gs2_scan_topic = "/test/gs2_scan_" + std::to_string(sequence_);
  module_ = std::make_unique<docking::DockingRuntimeModule>(
    *node_, callback_group_, config, docking::DockingRuntimePorts{});

  const auto initial = module_->observation_snapshot();
  EXPECT_LT(initial.age_sec, 0.0);
  EXPECT_FALSE(initial.usable);
  EXPECT_EQ(initial.detail, "gs2_scan_unavailable");

  auto scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    config.gs2_scan_topic, rclcpp::SensorDataQoS());
  spin_until([&]() {return scan_pub->get_subscription_count() == 1U;});
  scan_pub->publish(sensor_msgs::msg::LaserScan{});
  spin_until([&]() {return module_->observation_snapshot().age_sec >= 0.0;});

  const auto received = module_->observation_snapshot();
  EXPECT_GE(received.age_sec, 0.0);
  EXPECT_TRUE(received.usable);
  EXPECT_EQ(received.detail, "gs2_scan_received");
}

TEST_F(DockingRuntimeModuleTest, PreservesTriggerRetryEvidenceAndUndockStatusParsing)
{
  auto config = test_config(sequence_);
  config.service_timeout_sec = 1.0;
  config.stop_service_wait_sec = 0.5;
  config.undock_charging_retry_sec = 0.5;

  auto service_node = std::make_shared<rclcpp::Node>(
    "docking_runtime_services_" + std::to_string(sequence_));
  std::atomic<int> undock_calls{0};
  auto start_service = service_node->create_service<std_srvs::srv::Trigger>(
    config.start_service,
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
      response->message = "fine docking accepted";
    });
  auto stop_service = service_node->create_service<std_srvs::srv::Trigger>(
    config.stop_service,
    [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = true;
      response->message = "docking stopped";
    });
  auto undock_service = service_node->create_service<std_srvs::srv::Trigger>(
    config.undock_service,
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const int call = ++undock_calls;
      response->success = call >= 2;
      response->message = response->success ? "undock accepted" : "undock rejected";
    });
  (void)start_service;
  (void)stop_service;
  (void)undock_service;

  std::atomic<int> side_effect_started{0};
  std::atomic<int> side_effect_resolved{0};
  std::atomic<int> retry_waits{0};
  docking::DockingRuntimePorts ports;
  ports.delayed_side_effect_started = [&]() {++side_effect_started;};
  ports.delayed_side_effect_resolved = [&]() {++side_effect_resolved;};
  ports.on_undock_retry_wait = [&](const std::string & detail) {
      EXPECT_EQ(detail, "waiting for docking manager charging state before undock");
      ++retry_waits;
    };
  module_ = std::make_unique<docking::DockingRuntimeModule>(
    *node_, callback_group_, config, std::move(ports));

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node_);
  executor.add_node(service_node);
  std::thread spin_thread([&]() {executor.spin();});

  std::string start_detail;
  const bool start_ok = module_->start_fine_docking(start_detail);
  docking::DockingUndockServiceObservation observation;
  std::string undock_detail;
  const bool undock_ok = module_->call_undock_with_charging_retry(
    undock_detail, true, &observation);
  std::string stop_detail;
  module_->stop_if_available(stop_detail);

  executor.cancel();
  spin_thread.join();
  executor.remove_node(service_node);
  executor.remove_node(node_);

  EXPECT_TRUE(start_ok);
  EXPECT_EQ(start_detail, "fine docking accepted");
  EXPECT_TRUE(undock_ok);
  EXPECT_EQ(undock_detail, "undock accepted");
  EXPECT_TRUE(observation.service_called);
  EXPECT_TRUE(observation.service_success);
  EXPECT_EQ(observation.message, "undock accepted");
  EXPECT_EQ(undock_calls.load(), 2);
  EXPECT_EQ(retry_waits.load(), 1);
  EXPECT_EQ(stop_detail, "docking stopped");
  EXPECT_EQ(side_effect_started.load(), 4);
  EXPECT_EQ(side_effect_resolved.load(), 4);

  robot_api_server::DockingJob job;
  job.docking_service_success = true;
  job.docking_service_warning = "waiting_for_status";
  module_->observe_undock_status(
    job, "undock_failed cmd_count=7 failure_reason=rear_contact_blocked detail=x");
  EXPECT_TRUE(job.undock_started_observed);
  EXPECT_EQ(job.undock_cmd_count_observed, 7);
  EXPECT_EQ(job.undock_failure_reason, "rear_contact_blocked");
  EXPECT_TRUE(job.docking_service_warning.empty());
}

}  // namespace
