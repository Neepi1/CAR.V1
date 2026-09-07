#pragma once

#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"

namespace robot_api_server::features::navigation
{

std::string normalize_navigation_goal_completion_policy(std::string value);
bool is_valid_navigation_goal_completion_policy(const std::string & policy);
std::string normalize_position_only_nav2_yaw_mode(std::string value);
bool is_valid_position_only_nav2_yaw_mode(const std::string & mode);

struct NavigationGoalPolicyConfig
{
  std::string default_completion_policy{"pose_required"};
  std::string delivery_point_completion_policy{"pose_required"};
  std::string position_only_nav2_yaw_mode{"approach_heading"};
  double position_only_approach_heading_min_distance_m{0.20};
};

struct NavigationGoalCompletionPolicyResolution
{
  std::string policy;
  std::string error;
};

struct NavigationGoalYawResolution
{
  double yaw{0.0};
  std::string source{"stored_yaw"};
};

class NavigationGoalPolicy
{
public:
  explicit NavigationGoalPolicy(NavigationGoalPolicyConfig config);

  NavigationGoalCompletionPolicyResolution resolve_completion_policy(
    const std::string & request_body,
    const StoredPose & target) const;

  NavigationGoalYawResolution resolve_nav2_goal_yaw(
    const std::string & goal_completion_policy,
    const StoredPose & target,
    const RobotPoseSnapshot & current_pose,
    const std::string & pose_error) const;

private:
  NavigationGoalPolicyConfig config_;
};

}  // namespace robot_api_server::features::navigation
