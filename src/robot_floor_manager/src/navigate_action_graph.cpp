#include "robot_floor_manager/navigate_action_graph.hpp"

#include <algorithm>

namespace robot_floor_manager
{

std::vector<std::string> required_action_service_names(
  const std::string & action_name)
{
  std::string base = action_name;
  while (base.size() > 1U && base.back() == '/') {
    base.pop_back();
  }
  return {
    base + "/_action/send_goal",
    base + "/_action/get_result",
    base + "/_action/cancel_goal",
  };
}

bool navigate_action_graph_ready(
  const std::string & action_name,
  const std::set<std::string> & available_service_names,
  const std::size_t status_publisher_count)
{
  if (status_publisher_count == 0U) {
    return false;
  }
  const auto required = required_action_service_names(action_name);
  return std::all_of(
    required.begin(), required.end(),
    [&available_service_names](const std::string & service_name) {
      return available_service_names.count(service_name) != 0U;
    });
}

}  // namespace robot_floor_manager
