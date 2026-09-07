#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::teleop
{

struct TeleopModuleConfig
{
  std::string cmd_topic{"/cmd_vel_api"};
  std::string reverse_enable_topic{"/ranger_mini3/teleop_allow_reverse"};
  std::string pose_topic{"/local_state/odometry"};
  double max_linear_x_mps{1.0};
  double max_angular_z_radps{0.55};
  bool allow_reverse{false};
  bool require_mapping_active{true};
  bool stop_on_charging{true};
  double watchdog_timeout_sec{0.5};
  double socket_idle_timeout_sec{5.0};
  double repeat_rate_hz{20.0};
  int subscription_max_ttl_ms{60000};
};

struct TeleopMappingSnapshot
{
  bool active{false};
  bool live_map_available{false};
  double live_map_age_sec{-1.0};
  std::uint32_t live_map_width{0U};
  std::uint32_t live_map_height{0U};
  double live_map_resolution{0.0};
  double known_area_m2{0.0};
};

struct TeleopPoseSnapshot
{
  bool available{false};
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double age_sec{-1.0};
};

struct TeleopModulePorts
{
  std::function<bool(const HttpRequest & request)> token_allowed;
  std::function<ElevatorExecutionInterlock()> elevator_interlock;
  std::function<ElevatorMotionAdmissionFence::Epoch()> capture_motion_admission_epoch;
  std::function<ElevatorMotionAdmissionFence::AdmissionGuard(
      ElevatorMotionAdmissionFence::Epoch expected_epoch)>
  acquire_motion_admission;
  std::function<bool()> charging_contact_active;
  std::function<TeleopMappingSnapshot(bool refresh_runtime)> mapping_snapshot;
  std::function<TeleopPoseSnapshot()> pose_snapshot;
  std::function<void(
      const std::string & client_id,
      const std::vector<std::string> & resources,
      std::chrono::milliseconds ttl)>
  acquire_subscriptions;
  std::function<void(
      const std::string & client_id,
      const std::vector<std::string> & resources)>
  release_subscriptions;
  std::function<bool()> runtime_running;
};

// Owns the complete API teleoperation vertical slice: WebSocket admission and
// session lifecycle, command parsing, ROS command/reverse publishers, mapping
// and charging policy, subscription leases, watchdog repetition, and stop
// publication. The composition root supplies only cross-domain observations
// and admission ports.
class TeleopModule
{
public:
  TeleopModule(
    rclcpp::Node & node,
    TeleopModuleConfig config,
    TeleopModulePorts ports);
  ~TeleopModule();

  TeleopModule(const TeleopModule &) = delete;
  TeleopModule & operator=(const TeleopModule &) = delete;

  bool handle_socket(int client_fd, const HttpRequest & request);
  void clear_command();
  void publish_zero();
  void publish_zero_burst();
  void on_charging_contact(bool active);
  bool active() const;
  bool idle() const;
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::teleop
