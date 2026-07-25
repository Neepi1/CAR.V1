#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_elevator_manager
{

struct ElevatorTopologyCatalog
{
  std::string building_id;
  bool mock_ports_enabled{false};
  std::vector<ElevatorTopology> elevators;
};

struct TopologyLoadResult
{
  std::optional<ElevatorTopologyCatalog> catalog;
  std::vector<std::string> errors;

  bool ok() const noexcept;
};

TopologyLoadResult parse_topology_yaml(const std::string & yaml_text);
TopologyLoadResult load_topology_file(const std::string & path);

}  // namespace robot_elevator_manager
