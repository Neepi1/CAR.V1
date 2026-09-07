#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <csignal>
#include <filesystem>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include <sys/wait.h>
#include <unistd.h>

#include "robot_interfaces/msg/dock_target_observation.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"
#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"

namespace robot_api_server::features::docking
{

using namespace std::chrono_literals;

class DockingRuntimeModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & callback_group,
    DockingRuntimeConfig config,
    DockingRuntimePorts ports)
  : node_(node),
    logger_(node.get_logger()),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
    config_.service_timeout_sec = std::max(0.0, config_.service_timeout_sec);
    config_.stop_service_wait_sec = std::clamp(
      config_.stop_service_wait_sec, 0.0, config_.service_timeout_sec);
    config_.undock_charging_retry_sec = std::max(0.0, config_.undock_charging_retry_sec);
    if (config_.command_topic != "/cmd_vel_docking") {
      RCLCPP_WARN(
        logger_,
        "predock_yaw_align_cmd_topic=%s is not allowed; using /cmd_vel_docking",
        config_.command_topic.c_str());
      config_.command_topic = "/cmd_vel_docking";
    }
    if (config_.observation_backend != "target_observation" &&
      config_.observation_backend != "gs2_scan")
    {
      throw std::invalid_argument(
              "unsupported docking_observation_backend: " + config_.observation_backend);
    }

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    command_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      config_.command_topic, command_qos);
    if (!config_.forced_mode_topic.empty()) {
      forced_mode_pub_ = node_.create_publisher<std_msgs::msg::String>(
        config_.forced_mode_topic, rclcpp::QoS(1).transient_local());
    }

    rclcpp::SubscriptionOptions subscription_options;
    status_sub_ = node_.create_subscription<std_msgs::msg::String>(
      config_.status_topic,
      rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!shutting_down_.load(std::memory_order_acquire) && ports_.on_status) {
          ports_.on_status(msg->data);
        }
      },
      subscription_options);
    if (config_.observation_backend == "target_observation") {
      target_observation_sub_ =
        node_.create_subscription<robot_interfaces::msg::DockTargetObservation>(
        config_.target_observation_topic,
        rclcpp::QoS(5).reliable(),
        [this](const robot_interfaces::msg::DockTargetObservation::SharedPtr msg) {
          handle_target_observation(msg);
        },
        subscription_options);
    } else {
      gs2_scan_sub_ = node_.create_subscription<sensor_msgs::msg::LaserScan>(
        config_.gs2_scan_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          handle_gs2_scan(msg);
        },
        subscription_options);
    }

    start_client_ = node_.create_client<std_srvs::srv::Trigger>(
      config_.start_service, rmw_qos_profile_services_default, callback_group);
    stop_client_ = node_.create_client<std_srvs::srv::Trigger>(
      config_.stop_service, rmw_qos_profile_services_default, callback_group);
    undock_client_ = node_.create_client<std_srvs::srv::Trigger>(
      config_.undock_service, rmw_qos_profile_services_default, callback_group);
  }

  ~Impl()
  {
    shutdown();
  }

  DockingObservationSnapshot observation_snapshot() const
  {
    if (config_.observation_backend == "gs2_scan") {
      std::lock_guard<std::mutex> lock(gs2_scan_mutex_);
      if (!have_gs2_scan_) {
        return {-1.0, false, "gs2_scan_unavailable"};
      }
      return {
        age_sec(latest_gs2_scan_received_at_),
        true,
        "gs2_scan_received"};
    }

    std::lock_guard<std::mutex> lock(target_observation_mutex_);
    if (!have_target_observation_) {
      return {-1.0, false, "target_observation_unavailable"};
    }
    return {
      age_sec(latest_target_observation_received_at_),
      target_observation_usable_,
      target_observation_reason_};
  }

  const std::string & observation_backend() const
  {
    return config_.observation_backend;
  }

  const std::string & status_topic() const
  {
    return config_.status_topic;
  }

  bool ensure_manager_running(std::string & detail)
  {
    if (start_client_->wait_for_service(500ms)) {
      detail = "docking service already available";
      return true;
    }
    if (config_.manager_start_command.empty() ||
      !std::filesystem::exists(config_.manager_start_command))
    {
      detail = "docking manager start command is not available: " +
        config_.manager_start_command;
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(manager_process_mutex_);
      if (!manager_process_running_locked()) {
        const pid_t pid = ::fork();
        if (pid < 0) {
          detail = "failed to fork docking manager process";
          return false;
        }
        if (pid == 0) {
          prepare_child_process(config_.manager_log_file);
          ::execl(
            "/bin/bash", "bash", config_.manager_start_command.c_str(),
            static_cast<char *>(nullptr));
          ::_exit(127);
        }
        manager_pid_ = pid;
      }
    }

    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() < deadline) {
      if (start_client_->wait_for_service(500ms)) {
        detail = "docking manager ready; log_file=" + config_.manager_log_file;
        return true;
      }
    }
    detail = "timed out waiting for docking service; log_file=" + config_.manager_log_file;
    return false;
  }

  bool start_fine_docking(std::string & detail)
  {
    const auto observed = call_trigger_service_observed(
      start_client_, config_.start_service);
    detail = observed.message;
    return observed.service_success;
  }

  bool stop_if_available(std::string & detail)
  {
    if (!stop_client_->wait_for_service(stop_service_wait())) {
      detail = "docking stop service not available";
      return false;
    }
    const auto observed = call_trigger_service_observed(stop_client_, config_.stop_service);
    detail = observed.message;
    return observed.service_success;
  }

  bool call_undock_with_charging_retry(
    std::string & detail,
    const bool allow_charging_retry,
    DockingUndockServiceObservation * observation)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.undock_charging_retry_sec));
    while (true) {
      const auto observed = call_trigger_service_observed(
        undock_client_, config_.undock_service);
      detail = observed.message;
      if (observation != nullptr) {
        *observation = observed;
      }
      if (observed.service_success) {
        return true;
      }
      const bool retryable_rejection =
        detail.find("not docked and no charging contact") != std::string::npos ||
        detail.find("undock rejected") != std::string::npos;
      if (!allow_charging_retry || !retryable_rejection ||
        std::chrono::steady_clock::now() >= deadline)
      {
        return false;
      }
      const std::string waiting =
        "waiting for docking manager charging state before undock";
      if (ports_.on_undock_retry_wait) {
        ports_.on_undock_retry_wait(waiting);
      }
      std::this_thread::sleep_for(200ms);
    }
  }

  void observe_undock_status(
    ::robot_api_server::DockingJob & job,
    const std::string & status) const
  {
    if (status.empty()) {
      return;
    }
    job.docking_status_after_request = status;
    if (status_indicates_undock_started(status)) {
      job.undock_started_observed = true;
      if (job.docking_service_success) {
        job.docking_service_warning.clear();
      }
    }
    if (const auto count = status_int_value(status, "cmd_count")) {
      job.undock_cmd_count_observed = *count;
    }
    if (docking_status_is_undock_failed(status)) {
      job.undock_failure_reason = status_failure_reason(status);
    }
  }

  void publish_command(const geometry_msgs::msg::Twist & command)
  {
    if (command_pub_) {
      command_pub_->publish(command);
    }
  }

  void publish_forced_mode(const std::string & mode)
  {
    if (!forced_mode_pub_ || mode.empty()) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = mode;
    forced_mode_pub_->publish(msg);
  }

  bool launch_worker(const std::uint64_t job_id, std::string & error)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      error = "docking worker is already joinable";
      return false;
    }
    if (!ports_.run_job) {
      error = "docking worker execution port is unavailable";
      return false;
    }
    try {
      worker_ = std::thread(
        [this, job_id]() {
          ports_.run_job(job_id);
        });
      return true;
    } catch (const std::exception & exc) {
      error = std::string("failed to start docking worker: ") + exc.what();
      return false;
    }
  }

  void join_worker()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void shutdown()
  {
    bool expected = false;
    if (!shutting_down_.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel))
    {
      return;
    }
    join_worker();
    status_sub_.reset();
    gs2_scan_sub_.reset();
    target_observation_sub_.reset();
    start_client_.reset();
    stop_client_.reset();
    undock_client_.reset();
    command_pub_.reset();
    forced_mode_pub_.reset();
  }

private:
  static double age_sec(const std::chrono::steady_clock::time_point & received_at)
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_at).count();
  }

  std::chrono::nanoseconds service_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.service_timeout_sec));
  }

  std::chrono::nanoseconds stop_service_wait() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.stop_service_wait_sec));
  }

  bool manager_process_running_locked()
  {
    if (manager_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(manager_pid_, &status, WNOHANG);
    if (wait_result == manager_pid_) {
      manager_pid_ = -1;
      return false;
    }
    if (::kill(manager_pid_, 0) == 0 || errno == EPERM) {
      return true;
    }
    manager_pid_ = -1;
    return false;
  }

  DockingUndockServiceObservation call_trigger_service_observed(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & service_name)
  {
    if (!client->wait_for_service(service_timeout())) {
      return {false, false, "service unavailable: " + service_name};
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (ports_.delayed_side_effect_started) {
      ports_.delayed_side_effect_started();
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      return {false, false, "timed out waiting for service: " + service_name};
    }
    const auto response = future.get();
    if (ports_.delayed_side_effect_resolved) {
      ports_.delayed_side_effect_resolved();
    }
    return {true, response->success, response->message};
  }

  static std::optional<int> status_int_value(
    const std::string & status,
    const std::string & key)
  {
    const std::string needle = key + "=";
    const auto pos = status.find(needle);
    if (pos == std::string::npos) {
      return std::nullopt;
    }
    const auto start = pos + needle.size();
    std::size_t end = start;
    while (end < status.size() &&
      (std::isdigit(static_cast<unsigned char>(status[end])) ||
      status[end] == '-' || status[end] == '+'))
    {
      ++end;
    }
    if (end == start) {
      return std::nullopt;
    }
    try {
      return std::stoi(status.substr(start, end - start));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  static std::string status_failure_reason(const std::string & status)
  {
    if (!docking_status_is_undock_failed(status)) {
      return "";
    }
    const auto pos = status.find(" failure_reason=");
    if (pos != std::string::npos) {
      const auto start = pos + std::string(" failure_reason=").size();
      const auto end = status.find(' ', start);
      return status.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    }
    const auto end = status.find(' ');
    return status.substr(0, end == std::string::npos ? std::string::npos : end);
  }

  static bool status_indicates_undock_started(const std::string & status)
  {
    return docking_status_is_undocking(status) || docking_status_is_undocked(status) ||
           docking_status_is_undock_failed(status);
  }

  void handle_gs2_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    (void)msg;
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }
    std::lock_guard<std::mutex> lock(gs2_scan_mutex_);
    latest_gs2_scan_received_at_ = std::chrono::steady_clock::now();
    have_gs2_scan_ = true;
  }

  void handle_target_observation(
    const robot_interfaces::msg::DockTargetObservation::SharedPtr msg)
  {
    if (shutting_down_.load(std::memory_order_acquire) ||
      (!config_.target_observation_source.empty() &&
      msg->source != config_.target_observation_source))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(target_observation_mutex_);
    latest_target_observation_received_at_ = std::chrono::steady_clock::now();
    have_target_observation_ = true;
    target_observation_usable_ = msg->sensor_healthy && msg->valid;
    target_observation_reason_ = msg->reason;
  }

  rclcpp::Node & node_;
  rclcpp::Logger logger_;
  DockingRuntimeConfig config_;
  DockingRuntimePorts ports_;
  std::atomic<bool> shutting_down_{false};

  mutable std::mutex manager_process_mutex_;
  pid_t manager_pid_{-1};
  mutable std::mutex gs2_scan_mutex_;
  bool have_gs2_scan_{false};
  std::chrono::steady_clock::time_point latest_gs2_scan_received_at_{};
  mutable std::mutex target_observation_mutex_;
  bool have_target_observation_{false};
  bool target_observation_usable_{false};
  std::string target_observation_reason_;
  std::chrono::steady_clock::time_point latest_target_observation_received_at_{};
  std::mutex worker_mutex_;
  std::thread worker_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr forced_mode_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr gs2_scan_sub_;
  rclcpp::Subscription<robot_interfaces::msg::DockTargetObservation>::SharedPtr
    target_observation_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undock_client_;
};

DockingRuntimeModule::DockingRuntimeModule(
  rclcpp::Node & node,
  const rclcpp::CallbackGroup::SharedPtr & callback_group,
  DockingRuntimeConfig config,
  DockingRuntimePorts ports)
: impl_(std::make_unique<Impl>(
      node, callback_group, std::move(config), std::move(ports)))
{
}

DockingRuntimeModule::~DockingRuntimeModule() = default;

DockingObservationSnapshot DockingRuntimeModule::observation_snapshot() const
{
  return impl_->observation_snapshot();
}

const std::string & DockingRuntimeModule::observation_backend() const
{
  return impl_->observation_backend();
}

const std::string & DockingRuntimeModule::status_topic() const
{
  return impl_->status_topic();
}

bool DockingRuntimeModule::ensure_manager_running(std::string & detail)
{
  return impl_->ensure_manager_running(detail);
}

bool DockingRuntimeModule::start_fine_docking(std::string & detail)
{
  return impl_->start_fine_docking(detail);
}

bool DockingRuntimeModule::stop_if_available(std::string & detail)
{
  return impl_->stop_if_available(detail);
}

bool DockingRuntimeModule::call_undock_with_charging_retry(
  std::string & detail,
  const bool allow_charging_retry,
  DockingUndockServiceObservation * observation)
{
  return impl_->call_undock_with_charging_retry(
    detail, allow_charging_retry, observation);
}

void DockingRuntimeModule::observe_undock_status(
  ::robot_api_server::DockingJob & job,
  const std::string & status) const
{
  impl_->observe_undock_status(job, status);
}

void DockingRuntimeModule::publish_command(const geometry_msgs::msg::Twist & command)
{
  impl_->publish_command(command);
}

void DockingRuntimeModule::publish_forced_mode(const std::string & mode)
{
  impl_->publish_forced_mode(mode);
}

bool DockingRuntimeModule::launch_worker(
  const std::uint64_t job_id,
  std::string & error)
{
  return impl_->launch_worker(job_id, error);
}

void DockingRuntimeModule::join_worker()
{
  impl_->join_worker();
}

void DockingRuntimeModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::docking
