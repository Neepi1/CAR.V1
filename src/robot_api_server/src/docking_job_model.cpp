#include "robot_api_server/docking_job_model.hpp"

#include <iomanip>
#include <sstream>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

std::string docking_job_json(const DockingJob & job)
{
  std::ostringstream response;
  response << std::fixed << std::setprecision(6);
  response << "{\"id\":" << job.id << ","
           << "\"state\":" << json_string(job.state) << ","
           << "\"phase\":" << json_string(job.phase) << ","
           << "\"building_id\":" << json_string(job.building_id) << ","
           << "\"floor_id\":" << json_string(job.floor_id) << ","
           << "\"map_id\":" << json_string(job.map_id) << ","
           << "\"dock_id\":" << json_string(job.dock_id) << ","
           << "\"dock_name\":" << json_string(job.dock_name) << ","
           << "\"dock_type\":" << json_string(job.dock_type) << ","
           << "\"predock_pose_id\":" << json_string(job.predock_pose_id) << ","
           << "\"approach_source\":" << json_string(job.approach_source) << ","
           << "\"ok\":" << (job.ok ? "true" : "false") << ","
           << "\"resume_navigation\":" << (job.resume_navigation ? "true" : "false") << ","
           << "\"nav_goal_sent\":" << (job.nav_goal_sent ? "true" : "false") << ","
           << "\"nav_goal_succeeded\":" << (job.nav_goal_succeeded ? "true" : "false") << ","
           << "\"relocalization_requested\":" << (job.relocalization_requested ? "true" : "false") << ","
           << "\"relocalization_succeeded\":" << (job.relocalization_succeeded ? "true" : "false") << ","
           << "\"relocalization_detail\":" << json_string(job.relocalization_detail) << ","
           << "\"post_predock_relocalization_requested\":"
           << (job.post_predock_relocalization_requested ? "true" : "false") << ","
           << "\"post_predock_relocalization_succeeded\":"
           << (job.post_predock_relocalization_succeeded ? "true" : "false") << ","
           << "\"post_predock_relocalization_required\":"
           << (job.post_predock_relocalization_required ? "true" : "false") << ","
           << "\"post_predock_relocalization_detail\":"
           << json_string(job.post_predock_relocalization_detail) << ","
           << "\"post_fine_docking_relocalization_requested\":"
           << (job.post_fine_docking_relocalization_requested ? "true" : "false") << ","
           << "\"post_fine_docking_relocalization_succeeded\":"
           << (job.post_fine_docking_relocalization_succeeded ? "true" : "false") << ","
           << "\"post_fine_docking_relocalization_required\":"
           << (job.post_fine_docking_relocalization_required ? "true" : "false") << ","
           << "\"post_fine_docking_relocalization_detail\":"
           << json_string(job.post_fine_docking_relocalization_detail) << ","
           << "\"post_undock_relocalization_requested\":"
           << (job.post_undock_relocalization_requested ? "true" : "false") << ","
           << "\"post_undock_relocalization_succeeded\":"
           << (job.post_undock_relocalization_succeeded ? "true" : "false") << ","
           << "\"post_undock_relocalization_required\":"
           << (job.post_undock_relocalization_required ? "true" : "false") << ","
           << "\"post_undock_relocalization_detail\":"
           << json_string(job.post_undock_relocalization_detail) << ","
           << "\"active_navigation_cancel_requested\":"
           << (job.active_navigation_cancel_requested ? "true" : "false") << ","
           << "\"active_navigation_cancel_detail\":" << json_string(job.active_navigation_cancel_detail) << ","
           << "\"api_accepted\":" << (job.api_accepted ? "true" : "false") << ","
           << "\"already_running\":" << (job.already_running ? "true" : "false") << ","
           << "\"docking_service_called\":" << (job.docking_service_called ? "true" : "false") << ","
           << "\"docking_service_success\":" << (job.docking_service_success ? "true" : "false") << ","
           << "\"docking_service_message\":" << json_string(job.docking_service_message) << ","
           << "\"docking_status_at_request\":" << json_string(job.docking_status_at_request) << ","
           << "\"docking_status_after_request\":" << json_string(job.docking_status_after_request) << ","
           << "\"undock_started_observed\":" << (job.undock_started_observed ? "true" : "false") << ","
           << "\"undock_cmd_count_observed\":" << job.undock_cmd_count_observed << ","
           << "\"undock_failure_reason\":" << json_string(job.undock_failure_reason) << ","
           << "\"docking_service_warning\":" << json_string(job.docking_service_warning) << ","
           << "\"cancel_requested\":" << (job.cancel_requested ? "true" : "false") << ","
           << "\"dock_pose\":{\"x\":" << job.dock_x << ",\"y\":" << job.dock_y
           << ",\"yaw\":" << job.dock_yaw << "},"
           << "\"approach_pose\":{\"x\":" << job.approach_x << ",\"y\":" << job.approach_y
           << ",\"yaw\":" << job.approach_yaw << "},"
           << "\"approach_distance_m\":" << job.approach_distance_m << ","
           << "\"detail\":" << json_string(job.detail) << ","
           << "\"last_status\":" << json_string(job.last_status) << ","
           << "\"started_at\":" << json_string(job.started_at) << ","
           << "\"finished_at\":" << json_string(job.finished_at) << "}";
  return response.str();
}

}  // namespace robot_api_server
