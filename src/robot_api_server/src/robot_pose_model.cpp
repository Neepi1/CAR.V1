#include "robot_api_server/robot_pose_model.hpp"

#include <iomanip>
#include <sstream>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

std::string no_fresh_map_robot_pose_json(
  const std::string & map_frame,
  const std::string & base_frame)
{
  return std::string("{\"ok\":false,") +
    "\"error\":\"no fresh map-frame robot pose\"," +
    "\"frame_id\":" + json_string(map_frame) + "," +
    "\"child_frame_id\":" + json_string(base_frame) + "," +
    "\"age_sec\":null}";
}

std::string no_fresh_map_robot_pose_json(
  const std::string & map_frame,
  const std::string & base_frame,
  const std::string & detail)
{
  return std::string("{\"ok\":false,") +
    "\"error\":\"no fresh map-frame robot pose\"," +
    "\"detail\":" + json_string(detail) + "," +
    "\"frame_id\":" + json_string(map_frame) + "," +
    "\"child_frame_id\":" + json_string(base_frame) + "," +
    "\"age_sec\":null}";
}

std::string robot_pose_json(
  const RobotPoseSnapshot & pose,
  const RobotPoseMapIdentity & map_identity)
{
  std::ostringstream body;
  body << std::fixed << std::setprecision(6)
       << "{\"ok\":true,"
       << "\"frame_id\":" << json_string(pose.frame_id) << ","
       << "\"child_frame_id\":" << json_string(pose.child_frame_id) << ","
       << "\"x\":" << pose.x << ","
       << "\"y\":" << pose.y << ","
       << "\"yaw\":" << pose.yaw << ","
       << "\"stamp\":" << pose.stamp_sec << ","
       << "\"age_sec\":" << pose.age_sec << ","
       << "\"map_id\":";
  if (map_identity.map_id) {
    body << json_string(*map_identity.map_id);
  } else {
    body << "null";
  }
  body << ",\"floor_id\":";
  if (map_identity.floor_id) {
    body << json_string(*map_identity.floor_id);
  } else {
    body << "null";
  }
  body << ",\"building_id\":";
  if (map_identity.building_id) {
    body << json_string(*map_identity.building_id);
  } else {
    body << "null";
  }
  body << "}";
  return body.str();
}

}  // namespace robot_api_server
