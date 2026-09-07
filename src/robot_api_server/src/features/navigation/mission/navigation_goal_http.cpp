#include "robot_api_server/features/navigation/mission/navigation_goal_http.hpp"

#include <iomanip>
#include <sstream>

namespace robot_api_server::features::navigation
{

HttpResponse navigation_goal_error_response(
  const int status,
  const std::string & error,
  const bool pre_navigation_undock,
  const std::string & pre_navigation_undock_detail,
  const std::string & pre_navigation_dock_check_json,
  const std::string & code)
{
  std::ostringstream response;
  response << "{\"ok\":false,"
           << "\"accepted\":false,"
           << "\"error\":" << json_string(error) << ",";
  if (!code.empty()) {
    response << "\"code\":" << json_string(code) << ",";
  }
  response << "\"pre_navigation_undock\":"
           << (pre_navigation_undock ? "true" : "false") << ","
           << "\"pre_navigation_undock_detail\":"
           << json_string(pre_navigation_undock_detail) << ","
           << "\"pre_navigation_dock_check\":" << pre_navigation_dock_check_json << "}";
  return {status, "application/json", response.str()};
}

HttpResponse navigation_goal_accepted_response(const NavigationGoalAcceptedPayload & payload)
{
  std::ostringstream response;
  response << std::fixed << std::setprecision(6)
           << "{\"ok\":true,"
           << "\"accepted\":true,"
           << "\"navigation_goal_id\":" << payload.navigation_goal_id << ","
           << "\"action\":" << json_string(payload.action) << ","
           << "\"source\":" << json_string(payload.source) << ","
           << "\"goal_completion_policy\":"
           << json_string(payload.goal_completion_policy) << ","
           << "\"native_nav2_goal_completion\":"
           << (payload.native_nav2_goal_completion ? "true" : "false") << ","
           << "\"api_final_yaw_align_enabled\":"
           << (payload.api_final_yaw_align_enabled ? "true" : "false") << ","
           << "\"nav2_rotation_shim_enabled\":"
           << (payload.nav2_rotation_shim_enabled ? "true" : "false") << ","
           << "\"frame_id\":" << json_string(payload.frame_id) << ","
           << "\"pose_id\":" << json_string(payload.pose_id) << ",";
  if (payload.building_id) {
    response << "\"building_id\":" << json_string(*payload.building_id) << ",";
  }
  if (payload.floor_id) {
    response << "\"floor_id\":" << json_string(*payload.floor_id) << ",";
  }
  response << "\"pre_navigation_undock\":"
           << (payload.pre_navigation_undock ? "true" : "false") << ","
           << "\"pre_navigation_undock_detail\":"
           << json_string(payload.pre_navigation_undock_detail) << ","
           << "\"pre_navigation_dock_check\":"
           << payload.pre_navigation_dock_check_json << ","
           << "\"pre_navigation_relocalization_requested\":"
           << (payload.pre_navigation_relocalization_requested ? "true" : "false") << ","
           << "\"pre_navigation_relocalization_succeeded\":"
           << (payload.pre_navigation_relocalization_succeeded ? "true" : "false") << ","
           << "\"pre_navigation_relocalization_detail\":"
           << json_string(payload.pre_navigation_relocalization_detail) << ","
           << "\"navigation_normal_path_relocalization_enabled\":false,"
           << "\"force_accept_allowed_in_normal_path\":false,"
           << "\"ordinary_navigation_triggered_relocalization\":false,"
           << "\"localization_recovery_available\":true,"
           << "\"localization_recovery_required\":false,"
           << "\"goal\":{\"x\":" << payload.goal.x
           << ",\"y\":" << payload.goal.y
           << ",\"yaw\":" << payload.goal.yaw << "}}";
  return {202, "application/json", response.str()};
}

}  // namespace robot_api_server::features::navigation
