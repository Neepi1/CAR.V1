#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_api_server
{

struct DockingJob;

}  // namespace robot_api_server

namespace robot_api_server::features::docking
{

struct DockingUndockServiceObservation
{
  bool service_called{false};
  bool service_success{false};
  std::string message;
};

struct DockingObservationSnapshot
{
  double age_sec{-1.0};
  bool usable{false};
  std::string detail{"target_observation_unavailable"};
};

struct DockingRuntimeConfig
{
  std::string manager_start_command;
  std::string manager_log_file{"/tmp/njrh_docking_manager.log"};
  std::string start_service{"/docking/start"};
  std::string stop_service{"/docking/stop"};
  std::string undock_service{"/docking/undock"};
  std::string status_topic{"/docking/status"};
  std::string observation_backend{"target_observation"};
  std::string gs2_scan_topic{"/dock/gs2_scan"};
  std::string target_observation_topic{"/dock/target_observation"};
  std::string target_observation_source{"orbbec_336l_depth"};
  std::string command_topic{"/cmd_vel_docking"};
  std::string forced_mode_topic{"/ranger_mini3/forced_mode"};
  double service_timeout_sec{8.0};
  double stop_service_wait_sec{3.0};
  double undock_charging_retry_sec{3.0};
};

struct DockingRuntimePorts
{
  std::function<void(const std::string &)> on_status;
  std::function<void(std::uint64_t)> run_job;
  std::function<void()> delayed_side_effect_started;
  std::function<void()> delayed_side_effect_resolved;
  std::function<void(const std::string &)> on_undock_retry_wait;
};

// Owns the docking runtime boundary: manager child-process supervision, ROS
// service clients, status/sensor subscriptions, predock command publishers,
// cached observation evidence, and the single docking execution worker.
class DockingRuntimeModule
{
public:
  DockingRuntimeModule(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & callback_group,
    DockingRuntimeConfig config,
    DockingRuntimePorts ports);
  ~DockingRuntimeModule();

  DockingRuntimeModule(const DockingRuntimeModule &) = delete;
  DockingRuntimeModule & operator=(const DockingRuntimeModule &) = delete;

  DockingObservationSnapshot observation_snapshot() const;
  const std::string & observation_backend() const;
  const std::string & status_topic() const;

  bool ensure_manager_running(std::string & detail);
  bool start_fine_docking(std::string & detail);
  bool stop_if_available(std::string & detail);
  bool call_undock_with_charging_retry(
    std::string & detail,
    bool allow_charging_retry,
    DockingUndockServiceObservation * observation = nullptr);
  void observe_undock_status(
    ::robot_api_server::DockingJob & job,
    const std::string & status) const;

  void publish_command(const geometry_msgs::msg::Twist & command);
  void publish_forced_mode(const std::string & mode);

  bool launch_worker(std::uint64_t job_id, std::string & error);
  void join_worker();
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::docking
