#pragma once

#include <cstdint>
#include <string>

namespace robot_api_server
{

struct NavigationCancelJob
{
  std::uint64_t id{0U};
  std::string state{"idle"};
  std::string phase{"idle"};
  std::string reason;
  std::string detail;
  std::string cancel_all_detail;
  std::string stop_stack_detail;
  std::string started_at;
  std::string finished_at;
  bool stop_stack{true};
  bool ok{true};
  bool action_available{false};
  bool active_goal_cancel_requested{false};
  bool cancel_all_requested{false};
  bool cancel_all_ok{true};
  bool stop_stack_ok{true};
  bool zero_velocity_published{false};
};

std::string navigation_cancel_job_json(const NavigationCancelJob & job);

}  // namespace robot_api_server
