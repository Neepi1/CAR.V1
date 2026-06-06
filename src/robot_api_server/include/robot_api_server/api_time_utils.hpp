#pragma once

#include <string>

namespace robot_api_server
{

std::string utc_timestamp_compact();
std::string utc_timestamp_iso8601();
double wall_time_seconds();

std::string generate_current_pose_id(const std::string & type, const std::string & name);
std::string generate_map_id(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & display_name);

}  // namespace robot_api_server
