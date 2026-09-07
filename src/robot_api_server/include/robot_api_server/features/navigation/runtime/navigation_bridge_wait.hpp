#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace robot_api_server::features::navigation
{

struct BridgeReadinessSnapshot
{
  bool available{false};
  bool has_map_to_odom{false};
  std::string map_to_odom_publisher_owner{"unknown"};
  bool map_odom_correction_paused{false};
  std::string correction_pause_reason{"none"};
  bool map_odom_frozen_due_to_pause{false};
  bool correction_active{false};
  bool safe_for_goal_start{true};
  std::uint64_t current_sequence{0U};
  std::uint64_t target_sequence{0U};
  double remaining_translation_error_m{0.0};
  double remaining_yaw_error_rad{0.0};
  bool amcl_input_enabled{false};
  std::string amcl_degraded_reason;
  std::string amcl_status_source;
  double amcl_status_age_ms{-1.0};
  bool amcl_process_ready{false};
  bool amcl_seeded{false};
  bool amcl_nomotion_pose_received{false};
  bool amcl_static_standby{false};
  bool amcl_tracking_ready{false};
  bool amcl_correction_ready{false};
  bool amcl_correction_pending{false};
  bool amcl_not_moving_no_update_ok{false};
  bool localization_degraded{false};
};

enum class BridgeReadinessPurpose
{
  kGoalStart,
  kFinalPoseVerify,
};

struct BridgeReadinessDecision
{
  bool safe{false};
  std::string detail;
};

BridgeReadinessDecision evaluate_bridge_readiness(
  const BridgeReadinessSnapshot & bridge,
  BridgeReadinessPurpose purpose,
  const std::string & context);

struct NavigationBridgeWaitConfig
{
  bool final_verify_enabled{true};
  bool final_verify_wait_enabled{true};
  std::chrono::milliseconds final_verify_timeout{2000};
  std::chrono::milliseconds final_verify_sample_period{100};
  bool final_yaw_wait_enabled{true};
  std::chrono::milliseconds final_yaw_timeout{2000};
  std::chrono::milliseconds final_yaw_sample_period{100};
};

struct NavigationBridgeWaitResult
{
  bool waited{false};
  bool timeout{false};
  bool canceled{false};
  double elapsed_ms{0.0};
  std::string detail;
};

class NavigationBridgeWaitRuntimePort
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  virtual ~NavigationBridgeWaitRuntimePort() = default;

  virtual TimePoint bridge_wait_now() const = 0;
  virtual void bridge_wait_sleep_for(std::chrono::milliseconds duration) = 0;
  virtual bool bridge_wait_cancel_requested(
    std::uint64_t job_id,
    std::string & detail) = 0;
  virtual void publish_bridge_wait_zero() = 0;
  virtual BridgeReadinessSnapshot bridge_wait_snapshot() = 0;
  virtual void request_bridge_wait_nomotion_update(
    const BridgeReadinessSnapshot & bridge,
    std::string & detail) = 0;
};

class NavigationBridgeWait
{
public:
  explicit NavigationBridgeWait(NavigationBridgeWaitConfig config);

  NavigationBridgeWaitResult wait_before_final_verify(
    std::uint64_t job_id,
    NavigationBridgeWaitRuntimePort & runtime) const;

  NavigationBridgeWaitResult wait_before_final_yaw_align(
    std::uint64_t job_id,
    NavigationBridgeWaitRuntimePort & runtime) const;

private:
  NavigationBridgeWaitConfig config_;
};

}  // namespace robot_api_server::features::navigation
