#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot_nav_config
{

struct DirectCorridorParameters
{
  double minimum_length_m{1.0};
  double goal_yaw_tolerance_rad{0.20};
  double sample_step_m{0.025};
};

using CorridorPoseCheck = std::function<bool(double, double, double)>;

inline double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline geometry_msgs::msg::Quaternion yaw_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

inline double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double sin_yaw = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline std::optional<nav_msgs::msg::Path> make_direct_corridor_path(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const DirectCorridorParameters & parameters,
  const CorridorPoseCheck & pose_is_clear)
{
  const std::string frame_id =
    !start.header.frame_id.empty() ? start.header.frame_id : goal.header.frame_id;
  if (frame_id.empty() ||
    (!start.header.frame_id.empty() && !goal.header.frame_id.empty() &&
    start.header.frame_id != goal.header.frame_id))
  {
    return std::nullopt;
  }

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance_m = std::hypot(dx, dy);
  if (!std::isfinite(distance_m) ||
    distance_m < std::max(0.0, parameters.minimum_length_m))
  {
    return std::nullopt;
  }

  const double route_yaw = std::atan2(dy, dx);
  const double goal_yaw = quaternion_yaw(goal.pose.orientation);
  if (!std::isfinite(goal_yaw) ||
    std::abs(normalize_angle(goal_yaw - route_yaw)) >
    std::max(0.0, parameters.goal_yaw_tolerance_rad))
  {
    return std::nullopt;
  }

  const double sample_step_m = std::max(0.005, parameters.sample_step_m);
  const std::size_t segment_count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(distance_m / sample_step_m)));

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = start.header.stamp;
  path.poses.reserve(segment_count + 1U);

  for (std::size_t index = 0U; index <= segment_count; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(segment_count);
    const double x = start.pose.position.x + ratio * dx;
    const double y = start.pose.position.y + ratio * dy;
    const double collision_yaw = index == segment_count ? goal_yaw : route_yaw;
    if (!pose_is_clear || !pose_is_clear(x, y, collision_yaw)) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z =
      start.pose.position.z + ratio * (goal.pose.position.z - start.pose.position.z);
    pose.pose.orientation = yaw_quaternion(collision_yaw);
    path.poses.push_back(std::move(pose));
  }

  path.poses.back().pose = goal.pose;
  path.poses.back().header.frame_id = frame_id;
  return path;
}

}  // namespace robot_nav_config
