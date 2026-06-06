#include "robot_api_server/navigation_cancel_job_model.hpp"

#include <sstream>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

std::string navigation_cancel_job_json(const NavigationCancelJob & job)
{
  std::ostringstream response;
  response << "{\"id\":" << job.id << ","
           << "\"state\":" << json_string(job.state) << ","
           << "\"phase\":" << json_string(job.phase) << ","
           << "\"reason\":" << json_string(job.reason) << ","
           << "\"stop_stack\":" << (job.stop_stack ? "true" : "false") << ","
           << "\"ok\":" << (job.ok ? "true" : "false") << ","
           << "\"action_available\":" << (job.action_available ? "true" : "false") << ","
           << "\"active_goal_cancel_requested\":"
           << (job.active_goal_cancel_requested ? "true" : "false") << ","
           << "\"cancel_all_requested\":" << (job.cancel_all_requested ? "true" : "false") << ","
           << "\"cancel_all_ok\":" << (job.cancel_all_ok ? "true" : "false") << ","
           << "\"navigation_stack_stopped\":" << (job.stop_stack_ok ? "true" : "false") << ","
           << "\"zero_velocity_published\":" << (job.zero_velocity_published ? "true" : "false") << ","
           << "\"detail\":" << json_string(job.detail) << ","
           << "\"cancel_all_detail\":" << json_string(job.cancel_all_detail) << ","
           << "\"stop_stack_detail\":" << json_string(job.stop_stack_detail) << ","
           << "\"started_at\":" << json_string(job.started_at) << ","
           << "\"finished_at\":" << json_string(job.finished_at) << "}";
  return response.str();
}

}  // namespace robot_api_server
