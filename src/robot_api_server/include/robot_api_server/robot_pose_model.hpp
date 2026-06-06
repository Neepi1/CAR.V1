#pragma once

#include <optional>
#include <string>

namespace robot_api_server
{

struct RobotPoseSnapshot
{
  bool available{false};
  std::string frame_id;
  std::string child_frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double stamp_sec{0.0};
  double age_sec{-1.0};
};

struct RobotPoseMapIdentity
{
  std::optional<std::string> map_id;
  std::optional<std::string> floor_id;
  std::optional<std::string> building_id;
};

std::string no_fresh_map_robot_pose_json(
  const std::string & map_frame,
  const std::string & base_frame);

std::string no_fresh_map_robot_pose_json(
  const std::string & map_frame,
  const std::string & base_frame,
  const std::string & detail);

std::string robot_pose_json(
  const RobotPoseSnapshot & pose,
  const RobotPoseMapIdentity & map_identity);

}  // namespace robot_api_server
