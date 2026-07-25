#include "robot_elevator_manager/elevator_topology_loader.hpp"

#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace robot_elevator_manager
{
namespace
{

Point2 parse_point(const YAML::Node & node, const std::string & field)
{
  if (!node.IsSequence() || node.size() != 2U) {
    throw std::runtime_error(field + " must be a two-number sequence");
  }
  return Point2{node[0].as<double>(), node[1].as<double>()};
}

PoseRole parse_role(const std::string & name)
{
  if (name == "hall_call") {
    return PoseRole::kHallCall;
  }
  if (name == "hall_wait") {
    return PoseRole::kHallWait;
  }
  if (name == "doorway") {
    return PoseRole::kDoorway;
  }
  if (name == "cabin") {
    return PoseRole::kCabin;
  }
  if (name == "exit") {
    return PoseRole::kExit;
  }
  throw std::runtime_error("unknown pose role: " + name);
}

FloorElevatorTopology parse_floor(
  const YAML::Node & node,
  const std::size_t index)
{
  const std::string prefix = "floors[" + std::to_string(index) + "]";
  if (!node.IsMap()) {
    throw std::runtime_error(prefix + " must be a map");
  }
  FloorElevatorTopology floor;
  floor.floor_id = node["floor_id"].as<std::string>();
  floor.map_id = node["map_id"].as<std::string>();

  const auto poses = node["poses"];
  if (!poses.IsMap()) {
    throw std::runtime_error(prefix + ".poses must be a map");
  }
  for (const auto & entry : poses) {
    const auto role_name = entry.first.as<std::string>();
    floor.poses.push_back(
      PoseBinding{parse_role(role_name), entry.second.as<std::string>()});
  }

  const auto threshold = node["threshold"];
  if (!threshold.IsMap()) {
    throw std::runtime_error(prefix + ".threshold must be a map");
  }
  floor.threshold.left = parse_point(threshold["left"], prefix + ".threshold.left");
  floor.threshold.right = parse_point(threshold["right"], prefix + ".threshold.right");
  floor.threshold.cabin_reference = parse_point(
    threshold["cabin_reference"], prefix + ".threshold.cabin_reference");
  floor.threshold.clearance_m = threshold["clearance_m"].as<double>();
  floor.threshold.jamb_clearance_m =
    threshold["jamb_clearance_m"].as<double>();
  return floor;
}

TopologyLoadResult parse_root(const YAML::Node & root)
{
  TopologyLoadResult result;
  try {
    if (!root.IsMap()) {
      throw std::runtime_error("topology root must be a map");
    }
    const auto schema_version = root["schema_version"].as<int>();
    if (schema_version != 1) {
      throw std::runtime_error("unsupported schema_version");
    }

    ElevatorTopologyCatalog catalog;
    catalog.building_id = root["building_id"].as<std::string>();
    catalog.mock_ports_enabled = root["mock_ports_enabled"].as<bool>();
    if (!safe_asset_id(catalog.building_id)) {
      throw std::runtime_error("building_id must be path-safe");
    }
    const auto elevators = root["elevators"];
    if (!elevators.IsSequence() || elevators.size() == 0U) {
      throw std::runtime_error("elevators must be a non-empty sequence");
    }

    std::set<std::string> elevator_ids;
    for (std::size_t elevator_index = 0; elevator_index < elevators.size();
      ++elevator_index)
    {
      const auto elevator_node = elevators[elevator_index];
      if (!elevator_node.IsMap()) {
        throw std::runtime_error(
                "elevators[" + std::to_string(elevator_index) + "] must be a map");
      }
      ElevatorTopology topology;
      topology.elevator_id = elevator_node["elevator_id"].as<std::string>();
      topology.building_id = catalog.building_id;
      if (!elevator_ids.insert(topology.elevator_id).second) {
        throw std::runtime_error("duplicate elevator_id: " + topology.elevator_id);
      }
      const auto floors = elevator_node["floors"];
      if (!floors.IsSequence()) {
        throw std::runtime_error("elevator floors must be a sequence");
      }
      for (std::size_t floor_index = 0; floor_index < floors.size(); ++floor_index) {
        topology.floors.push_back(parse_floor(floors[floor_index], floor_index));
      }
      const auto validation = validate_topology(topology);
      if (!validation.ok()) {
        std::ostringstream message;
        message << "elevator " << topology.elevator_id << " is invalid:";
        for (const auto & issue : validation.issues) {
          message << " " << issue.field << "=" << issue.message << ";";
        }
        throw std::runtime_error(message.str());
      }
      catalog.elevators.push_back(std::move(topology));
    }
    result.catalog = std::move(catalog);
  } catch (const YAML::Exception & error) {
    result.errors.push_back(std::string("YAML error: ") + error.what());
  } catch (const std::exception & error) {
    result.errors.push_back(error.what());
  }
  return result;
}

}  // namespace

bool TopologyLoadResult::ok() const noexcept
{
  return catalog.has_value() && errors.empty();
}

TopologyLoadResult parse_topology_yaml(const std::string & yaml_text)
{
  try {
    return parse_root(YAML::Load(yaml_text));
  } catch (const YAML::Exception & error) {
    TopologyLoadResult result;
    result.errors.push_back(std::string("YAML error: ") + error.what());
    return result;
  }
}

TopologyLoadResult load_topology_file(const std::string & path)
{
  TopologyLoadResult result;
  if (path.empty()) {
    result.errors.push_back("topology file path is empty");
    return result;
  }
  std::ifstream stream(path);
  if (!stream) {
    result.errors.push_back("cannot open topology file: " + path);
    return result;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return parse_topology_yaml(buffer.str());
}

}  // namespace robot_elevator_manager
