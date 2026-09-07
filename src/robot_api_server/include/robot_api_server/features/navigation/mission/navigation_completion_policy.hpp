#pragma once

#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"

namespace robot_api_server::features::navigation
{

struct FinalPoseCheck
{
  RobotPoseSnapshot pose;
  bool pose_available{false};
  bool position_reached{false};
  double distance_m{-1.0};
  double yaw_error_rad{-1.0};
  std::string reason;
};

struct NavigationCompletionPolicyConfig
{
  std::string map_frame{"map"};
  double robot_pose_freshness_sec{0.5};
  double position_tolerance_m{0.06};
  double yaw_tolerance_rad{0.05};
  double yaw_align_trigger_rad{0.08};

  bool final_verify_enabled{true};
  bool retry_uses_same_nav2_goal{true};
  int final_verify_max_retry_count{3};
  double acceptance_slack_m{0.02};
  double xy_retry_min_error_m{0.06};
  bool yaw_retry_if_failed{true};
  double terminal_recovery_max_distance_m{0.40};

  bool nav2_failed_near_goal_retry_enabled{true};
  int nav2_failed_near_goal_retry_max_count{1};
  bool nav2_failed_near_goal_retry_requires_yaw_error{false};
};

struct NavigationRetryDecision
{
  bool allowed{false};
  std::string reason;
  std::string phase;
};

struct NavigationAcceptanceDecision
{
  bool allowed{false};
  std::string detail;
};

class NavigationCompletionPolicy
{
public:
  explicit NavigationCompletionPolicy(NavigationCompletionPolicyConfig config);

  FinalPoseCheck evaluate_final_pose(
    const RobotPoseSnapshot & pose,
    const StoredPose & target,
    bool require_fresh_pose,
    const std::string & pose_error) const;

  NavigationRetryDecision final_verify_retry(
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    int retry_count) const;

  NavigationAcceptanceDecision final_verify_acceptance_slack(
    const FinalPoseCheck & check,
    int retry_count) const;

  NavigationRetryDecision nav2_failed_near_goal_retry(
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    double yaw_align_xy_gate,
    int retry_count) const;

private:
  NavigationCompletionPolicyConfig config_;
};

}  // namespace robot_api_server::features::navigation
