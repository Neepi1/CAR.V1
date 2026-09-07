#include "robot_api_server/features/navigation/mission/navigation_goal_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{
namespace
{

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  return value;
}

double normalize_angle(double value)
{
  constexpr double kPi = 3.14159265358979323846;
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

bool stored_pose_type_requires_yaw(const StoredPose & pose)
{
  const auto type = lower_copy(pose.type);
  return type == "pose_required" ||
         type == "delivery_pose_required" ||
         type == "precise_pose" ||
         type == "orientation_required" ||
         type == "require_yaw" ||
         type == "requires_yaw";
}

bool stored_pose_type_uses_delivery_default(const StoredPose & pose)
{
  const auto type = lower_copy(pose.type);
  return type.empty() || type == "delivery" || type == "delivery_point";
}

}  // namespace

std::string normalize_navigation_goal_completion_policy(std::string value)
{
  value = lower_copy(std::move(value));
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

bool is_valid_navigation_goal_completion_policy(const std::string & policy)
{
  return policy == "position_only" || policy == "pose_required" || policy == "dock_staging";
}

std::string normalize_position_only_nav2_yaw_mode(std::string value)
{
  value = lower_copy(std::move(value));
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

bool is_valid_position_only_nav2_yaw_mode(const std::string & mode)
{
  return mode == "stored_yaw" || mode == "current_yaw" || mode == "approach_heading";
}

NavigationGoalPolicy::NavigationGoalPolicy(NavigationGoalPolicyConfig config)
: config_(std::move(config))
{
}

NavigationGoalCompletionPolicyResolution NavigationGoalPolicy::resolve_completion_policy(
  const std::string & request_body,
  const StoredPose & target) const
{
  NavigationGoalCompletionPolicyResolution result;
  if (const auto requested = json_string_value(request_body, "goal_completion_policy")) {
    result.policy = normalize_navigation_goal_completion_policy(*requested);
    if (!is_valid_navigation_goal_completion_policy(result.policy)) {
      result.error =
        "goal_completion_policy must be position_only, pose_required, or dock_staging";
      result.policy = config_.default_completion_policy;
    }
    return result;
  }
  if (json_bool_value(request_body, "require_yaw", false) ||
    json_bool_value(request_body, "pose_required", false) ||
    stored_pose_type_requires_yaw(target))
  {
    result.policy = "pose_required";
    return result;
  }
  result.policy = stored_pose_type_uses_delivery_default(target) ?
    config_.delivery_point_completion_policy : config_.default_completion_policy;
  return result;
}

NavigationGoalYawResolution NavigationGoalPolicy::resolve_nav2_goal_yaw(
  const std::string & goal_completion_policy,
  const StoredPose & target,
  const RobotPoseSnapshot & current_pose,
  const std::string & pose_error) const
{
  NavigationGoalYawResolution result;
  result.yaw = normalize_angle(target.yaw);
  if (goal_completion_policy != "position_only" ||
    config_.position_only_nav2_yaw_mode == "stored_yaw")
  {
    return result;
  }

  if (!current_pose.available) {
    result.source = "stored_yaw_fallback_no_fresh_pose";
    if (!pose_error.empty()) {
      result.source += ":" + pose_error;
    }
    return result;
  }

  if (config_.position_only_nav2_yaw_mode == "current_yaw") {
    result.source = "current_yaw";
    result.yaw = normalize_angle(current_pose.yaw);
    return result;
  }

  const double dx = target.x - current_pose.x;
  const double dy = target.y - current_pose.y;
  if (std::hypot(dx, dy) >= config_.position_only_approach_heading_min_distance_m) {
    result.source = "approach_heading";
    result.yaw = normalize_angle(std::atan2(dy, dx));
    return result;
  }

  result.source = "current_yaw_near_target";
  result.yaw = normalize_angle(current_pose.yaw);
  return result;
}

}  // namespace robot_api_server::features::navigation
