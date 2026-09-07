#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot_nav_config
{

struct ExactGoalEndpointParameters
{
  // The Ranger lattice is 5 cm with 16 heading bins. These limits accept only
  // the final quantization residue; they must never bridge a failed search.
  double maximum_translation_correction_m{0.08};
  double maximum_yaw_correction_rad{0.21};
  double translation_sample_step_m{0.025};
  double yaw_sample_step_rad{0.05};
};

using ExactGoalPoseCheck = std::function<bool (double, double, double)>;

inline double exact_goal_normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::optional<double> exact_goal_quaternion_yaw(
  const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
    return std::nullopt;
  }
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y) /
    norm_squared;
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z) /
    norm_squared;
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
}

inline std::optional<nav_msgs::msg::Path> append_exact_goal_endpoint(
  const nav_msgs::msg::Path & searched_path,
  const geometry_msgs::msg::PoseStamped & requested_goal,
  const ExactGoalEndpointParameters & parameters,
  const ExactGoalPoseCheck & pose_is_clear)
{
  if (searched_path.poses.empty() || !pose_is_clear) {
    return std::nullopt;
  }

  const auto & snapped_endpoint = searched_path.poses.back();
  const std::string frame_id =
    !searched_path.header.frame_id.empty() ? searched_path.header.frame_id :
    (!snapped_endpoint.header.frame_id.empty() ? snapped_endpoint.header.frame_id :
    requested_goal.header.frame_id);
  if (frame_id.empty() ||
    (!snapped_endpoint.header.frame_id.empty() &&
    snapped_endpoint.header.frame_id != frame_id) ||
    (!requested_goal.header.frame_id.empty() &&
    requested_goal.header.frame_id != frame_id))
  {
    return std::nullopt;
  }

  const double from_x = snapped_endpoint.pose.position.x;
  const double from_y = snapped_endpoint.pose.position.y;
  const double to_x = requested_goal.pose.position.x;
  const double to_y = requested_goal.pose.position.y;
  const auto from_yaw = exact_goal_quaternion_yaw(snapped_endpoint.pose.orientation);
  const auto to_yaw = exact_goal_quaternion_yaw(requested_goal.pose.orientation);
  if (!std::isfinite(from_x) || !std::isfinite(from_y) ||
    !std::isfinite(to_x) || !std::isfinite(to_y) ||
    !from_yaw.has_value() || !to_yaw.has_value())
  {
    return std::nullopt;
  }

  const double dx = to_x - from_x;
  const double dy = to_y - from_y;
  const double translation = std::hypot(dx, dy);
  const double yaw_delta = exact_goal_normalize_angle(*to_yaw - *from_yaw);
  if (!std::isfinite(translation) || !std::isfinite(yaw_delta) ||
    translation > std::max(0.0, parameters.maximum_translation_correction_m) ||
    std::abs(yaw_delta) > std::max(0.0, parameters.maximum_yaw_correction_rad))
  {
    return std::nullopt;
  }

  const double translation_step = std::max(0.005, parameters.translation_sample_step_m);
  const double yaw_step = std::max(0.01, parameters.yaw_sample_step_rad);
  const std::size_t sample_count = static_cast<std::size_t>(std::max(
    {
      1.0,
      std::ceil(translation / translation_step),
      std::ceil(std::abs(yaw_delta) / yaw_step)}));
  for (std::size_t index = 1U; index <= sample_count; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(sample_count);
    if (!pose_is_clear(
        from_x + ratio * dx,
        from_y + ratio * dy,
        exact_goal_normalize_angle(*from_yaw + ratio * yaw_delta)))
    {
      return std::nullopt;
    }
  }

  nav_msgs::msg::Path result = searched_path;
  result.header.frame_id = frame_id;
  geometry_msgs::msg::PoseStamped exact_endpoint = requested_goal;
  exact_endpoint.header = result.header;

  constexpr double kPositionEpsilonM = 1.0e-6;
  constexpr double kYawEpsilonRad = 1.0e-6;
  if (translation <= kPositionEpsilonM && std::abs(yaw_delta) <= kYawEpsilonRad) {
    result.poses.back() = exact_endpoint;
  } else {
    result.poses.push_back(exact_endpoint);
  }
  return result;
}

}  // namespace robot_nav_config
