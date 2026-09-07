#ifndef ROBOT_FLOOR_MANAGER__NAVIGATE_ACTION_GRAPH_HPP_
#define ROBOT_FLOOR_MANAGER__NAVIGATE_ACTION_GRAPH_HPP_

#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace robot_floor_manager
{

std::vector<std::string> required_action_service_names(
  const std::string & action_name);

bool navigate_action_graph_ready(
  const std::string & action_name,
  const std::set<std::string> & available_service_names,
  std::size_t status_publisher_count);

}  // namespace robot_floor_manager

#endif  // ROBOT_FLOOR_MANAGER__NAVIGATE_ACTION_GRAPH_HPP_
