#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

std::string poses_json_array(const std::vector<StoredPose> & poses);
std::vector<StoredPose> read_floor_poses(const std::filesystem::path & path);
void write_floor_poses(const std::filesystem::path & path, const std::vector<StoredPose> & poses);
std::optional<StoredPose> find_floor_pose(
  const std::filesystem::path & path,
  const std::string & pose_id);

}  // namespace robot_api_server
