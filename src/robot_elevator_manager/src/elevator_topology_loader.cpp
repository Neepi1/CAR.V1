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

PoseRole parse_role(const std::string & name, const std::uint32_t schema_version)
{
  if (name == "hall_call") {
    return PoseRole::kHallCall;
  }
  if ((schema_version == 2U || schema_version == 3U) && name == "landing") {
    return PoseRole::kLanding;
  }
  if (schema_version == 1U && name == "hall_wait") {
    return PoseRole::kHallWait;
  }
  if (schema_version == 1U && name == "doorway") {
    return PoseRole::kDoorway;
  }
  if (name == "cabin") {
    return PoseRole::kCabin;
  }
  if (schema_version == 3U && name == "cabin_panel") {
    return PoseRole::kCabinPanel;
  }
  if (schema_version == 1U && name == "exit") {
    return PoseRole::kExit;
  }
  throw std::runtime_error(
          "unknown pose role for schema v" + std::to_string(schema_version) + ": " + name);
}

FloorElevatorTopology parse_floor(
  const YAML::Node & node,
  const std::size_t index,
  const std::uint32_t schema_version)
{
  const std::string prefix = "floors[" + std::to_string(index) + "]";
  if (!node.IsMap()) {
    throw std::runtime_error(prefix + " must be a map");
  }
  FloorElevatorTopology floor;
  floor.floor_id = node["floor_id"].as<std::string>();
  floor.map_id = node["map_id"].as<std::string>();
  if (schema_version == 3U) {
    const auto parse_panel_side = [&node, &prefix](const char * field) {
        if (!node[field]) {
          throw std::runtime_error(prefix + "." + field + " is required for schema v3");
        }
        const auto side = panel_side_from_string(node[field].as<std::string>());
        if (!side.has_value()) {
          throw std::runtime_error(
                  prefix + "." + field + " must be LEFT or RIGHT for schema v3");
        }
        return *side;
      };
    floor.hall_call_panel_side = parse_panel_side("hall_call_panel_side");
    floor.cabin_panel_side = parse_panel_side("cabin_panel_side");
  }

  const auto poses = node["poses"];
  if (!poses.IsMap()) {
    throw std::runtime_error(prefix + ".poses must be a map");
  }
  for (const auto & entry : poses) {
    const auto role_name = entry.first.as<std::string>();
    floor.poses.push_back(
      PoseBinding{parse_role(role_name, schema_version), entry.second.as<std::string>()});
  }

  const auto threshold = node["threshold"];
  if (schema_version == 1U) {
    if (!threshold.IsMap()) {
      throw std::runtime_error(prefix + ".threshold must be a map for schema v1");
    }
    DoorThreshold parsed_threshold;
    parsed_threshold.left = parse_point(threshold["left"], prefix + ".threshold.left");
    parsed_threshold.right = parse_point(threshold["right"], prefix + ".threshold.right");
    parsed_threshold.cabin_reference = parse_point(
      threshold["cabin_reference"], prefix + ".threshold.cabin_reference");
    if (threshold["clearance_m"]) {
      parsed_threshold.clearance_m = threshold["clearance_m"].as<double>();
    }
    if (threshold["jamb_clearance_m"]) {
      parsed_threshold.jamb_clearance_m =
        threshold["jamb_clearance_m"].as<double>();
    }
    floor.threshold = parsed_threshold;
  } else if (threshold) {
    throw std::runtime_error(prefix + ".threshold is forbidden for schema v2+");
  }
  return floor;
}

TopologyLoadResult parse_root(const YAML::Node & root)
{
  TopologyLoadResult result;
  try {
    if (!root.IsMap()) {
      throw std::runtime_error("topology root must be a map");
    }
    const auto schema_version = root["schema_version"].as<std::uint32_t>();
    if (schema_version != 1U && schema_version != 2U && schema_version != 3U) {
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
      topology.schema_version = schema_version;
      if (!elevator_ids.insert(topology.elevator_id).second) {
        throw std::runtime_error("duplicate elevator_id: " + topology.elevator_id);
      }
      const auto floors = elevator_node["floors"];
      if (!floors.IsSequence()) {
        throw std::runtime_error("elevator floors must be a sequence");
      }
      for (std::size_t floor_index = 0; floor_index < floors.size(); ++floor_index) {
        topology.floors.push_back(
          parse_floor(floors[floor_index], floor_index, schema_version));
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
