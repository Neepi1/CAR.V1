#include "robot_elevator_manager/elevator_topology.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <utility>

namespace robot_elevator_manager
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-6;

bool ascii_alnum(const unsigned char value) noexcept
{
  return (value >= static_cast<unsigned char>('a') &&
         value <= static_cast<unsigned char>('z')) ||
         (value >= static_cast<unsigned char>('A') &&
         value <= static_cast<unsigned char>('Z')) ||
         (value >= static_cast<unsigned char>('0') &&
         value <= static_cast<unsigned char>('9'));
}

bool safe_identifier(const std::string & value, const bool allow_colon) noexcept
{
  if (value.empty() || value == "." || value.size() > 128U ||
    value.find("..") != std::string::npos ||
    value.find('/') != std::string::npos || value.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [allow_colon](const unsigned char character) {
    return ascii_alnum(character) || character == static_cast<unsigned char>('-') ||
           character == static_cast<unsigned char>('_') ||
           character == static_cast<unsigned char>('.') ||
           (allow_colon && character == static_cast<unsigned char>(':'));
  });
}

bool finite(const Point2 & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double cross(
  const Point2 & origin,
  const Point2 & end,
  const Point2 & point) noexcept
{
  return (end.x - origin.x) * (point.y - origin.y) -
         (end.y - origin.y) * (point.x - origin.x);
}

bool valid_threshold(const DoorThreshold & threshold) noexcept
{
  if (!finite(threshold.left) || !finite(threshold.right) ||
    !finite(threshold.cabin_reference) || !std::isfinite(threshold.clearance_m) ||
    threshold.clearance_m < 0.0 || !std::isfinite(threshold.jamb_clearance_m) ||
    threshold.jamb_clearance_m < 0.0)
  {
    return false;
  }
  const double dx = threshold.right.x - threshold.left.x;
  const double dy = threshold.right.y - threshold.left.y;
  const double length = std::hypot(dx, dy);
  if (length <= kGeometryEpsilon) {
    return false;
  }
  if (2.0 * threshold.jamb_clearance_m >= length) {
    return false;
  }
  const double cabin_distance =
    std::abs(cross(threshold.left, threshold.right, threshold.cabin_reference)) / length;
  return cabin_distance > kGeometryEpsilon;
}

bool supported_pose_role(const PoseRole role, const std::uint32_t schema_version)
{
  const auto & roles = required_pose_roles(schema_version);
  return std::find(roles.begin(), roles.end(), role) != roles.end();
}

void append_issue(
  TopologyValidation & result,
  const TopologyIssueCode code,
  std::string field,
  std::string message)
{
  result.issues.push_back(TopologyIssue{code, std::move(field), std::move(message)});
}

}  // namespace

bool TopologyValidation::ok() const noexcept
{
  return issues.empty();
}

bool RouteResolution::ok() const noexcept
{
  return error == RouteError::kNone && route.has_value();
}

bool safe_asset_id(const std::string & value) noexcept
{
  return safe_identifier(value, false);
}

bool safe_pose_id(const std::string & value) noexcept
{
  return safe_identifier(value, true);
}

std::string to_string(const PoseRole role)
{
  switch (role) {
    case PoseRole::kHallCall:
      return "hall_call";
    case PoseRole::kLanding:
      return "landing";
    case PoseRole::kHallWait:
      return "hall_wait";
    case PoseRole::kDoorway:
      return "doorway";
    case PoseRole::kCabin:
      return "cabin";
    case PoseRole::kExit:
      return "exit";
    case PoseRole::kCabinPanel:
      return "cabin_panel";
  }
  return "unknown";
}

std::string to_string(const PanelSide side)
{
  switch (side) {
    case PanelSide::kLeft:
      return "LEFT";
    case PanelSide::kRight:
      return "RIGHT";
    case PanelSide::kUnknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

std::optional<PanelSide> panel_side_from_string(const std::string & value)
{
  if (value == "LEFT") {
    return PanelSide::kLeft;
  }
  if (value == "RIGHT") {
    return PanelSide::kRight;
  }
  return std::nullopt;
}

const std::vector<PoseRole> & required_pose_roles(const std::uint32_t schema_version)
{
  static const std::vector<PoseRole> legacy_roles{
    PoseRole::kHallCall,
    PoseRole::kHallWait,
    PoseRole::kDoorway,
    PoseRole::kCabin,
    PoseRole::kExit,
  };
  static const std::vector<PoseRole> current_roles{
    PoseRole::kHallCall,
    PoseRole::kLanding,
    PoseRole::kCabin,
  };
  static const std::vector<PoseRole> reverse_entry_roles{
    PoseRole::kHallCall,
    PoseRole::kLanding,
    PoseRole::kCabin,
    PoseRole::kCabinPanel,
  };
  static const std::vector<PoseRole> no_roles;
  if (schema_version == 1U) {
    return legacy_roles;
  }
  if (schema_version == 2U) {
    return current_roles;
  }
  if (schema_version == 3U) {
    return reverse_entry_roles;
  }
  return no_roles;
}

TopologyValidation validate_topology(const ElevatorTopology & topology)
{
  TopologyValidation result;
  if (!safe_asset_id(topology.elevator_id)) {
    append_issue(
      result, TopologyIssueCode::kUnsafeElevatorId, "elevator_id",
      "elevator_id must be a bounded path-safe identifier");
  }
  if (!safe_asset_id(topology.building_id)) {
    append_issue(
      result, TopologyIssueCode::kUnsafeBuildingId, "building_id",
      "building_id must be a bounded path-safe identifier");
  }
  if (
    topology.schema_version != 1U && topology.schema_version != 2U &&
    topology.schema_version != 3U)
  {
    append_issue(
      result, TopologyIssueCode::kUnsupportedSchema, "schema_version",
      "elevator topology schema_version must be 1, 2, or 3");
  }
  if (topology.floors.size() < 2U) {
    append_issue(
      result, TopologyIssueCode::kTooFewFloors, "floors",
      "an elevator topology requires at least two floors");
  }

  std::set<std::string> floor_ids;
  for (std::size_t floor_index = 0; floor_index < topology.floors.size(); ++floor_index) {
    const auto & floor = topology.floors[floor_index];
    const std::string floor_path = "floors[" + std::to_string(floor_index) + "]";
    if (!safe_asset_id(floor.floor_id)) {
      append_issue(
        result, TopologyIssueCode::kUnsafeFloorId, floor_path + ".floor_id",
        "floor_id must be a bounded path-safe identifier");
    }
    if (!safe_asset_id(floor.map_id)) {
      append_issue(
        result, TopologyIssueCode::kUnsafeMapId, floor_path + ".map_id",
        "map_id must be a bounded path-safe identifier");
    }
    if (!floor_ids.insert(floor.floor_id).second) {
      append_issue(
        result, TopologyIssueCode::kDuplicateFloor, floor_path + ".floor_id",
        "floor_id is duplicated in this elevator topology");
    }

    std::set<PoseRole> roles;
    std::set<std::string> pose_ids;
    for (std::size_t pose_index = 0; pose_index < floor.poses.size(); ++pose_index) {
      const auto & pose = floor.poses[pose_index];
      const std::string pose_path =
        floor_path + ".poses[" + std::to_string(pose_index) + "]";
      if (!safe_pose_id(pose.pose_id)) {
        append_issue(
          result, TopologyIssueCode::kUnsafePoseId, pose_path + ".pose_id",
          "pose_id must be a bounded path-safe identifier");
      }
      if (!supported_pose_role(pose.role, topology.schema_version)) {
        append_issue(
          result, TopologyIssueCode::kUnknownPoseRole, pose_path + ".role",
          "pose role is not part of the elevator topology contract");
      }
      if (!roles.insert(pose.role).second) {
        append_issue(
          result, TopologyIssueCode::kDuplicatePoseRole, pose_path + ".role",
          "each required pose role must occur exactly once per floor");
      }
      if (!pose_ids.insert(pose.pose_id).second) {
        append_issue(
          result, TopologyIssueCode::kDuplicatePoseId, pose_path + ".pose_id",
          "one pose_id cannot satisfy multiple elevator roles on the same floor");
      }
    }
    for (const auto required_role : required_pose_roles(topology.schema_version)) {
      if (roles.count(required_role) == 0U) {
        append_issue(
          result, TopologyIssueCode::kMissingPoseRole,
          floor_path + ".poses." + to_string(required_role),
          "required elevator pose role is missing");
      }
    }
    if (
      topology.schema_version == 1U &&
      (!floor.threshold || !valid_threshold(*floor.threshold)))
    {
      append_issue(
        result, TopologyIssueCode::kInvalidThreshold, floor_path + ".threshold",
        "schema v1 threshold must be finite, non-degenerate, and have a cabin reference off the line");
    }
    if (topology.schema_version >= 2U && floor.threshold) {
      append_issue(
        result, TopologyIssueCode::kInvalidThreshold, floor_path + ".threshold",
        "schema v2+ forbids legacy threshold geometry");
    }
    if (
      topology.schema_version == 3U &&
      (floor.hall_call_panel_side == PanelSide::kUnknown ||
      floor.cabin_panel_side == PanelSide::kUnknown))
    {
      append_issue(
        result, TopologyIssueCode::kMissingPanelSide,
        floor_path + ".panel_side",
        "schema v3 requires LEFT or RIGHT for hall_call_panel_side and "
        "cabin_panel_side");
    }
  }
  return result;
}

std::optional<std::string> find_pose_id(
  const FloorElevatorTopology & floor,
  const PoseRole role)
{
  const auto iterator = std::find_if(
    floor.poses.begin(), floor.poses.end(), [role](const PoseBinding & pose) {
      return pose.role == role;
    });
  if (iterator == floor.poses.end()) {
    return std::nullopt;
  }
  return iterator->pose_id;
}

RouteResolution resolve_route(
  const ElevatorTopology & topology,
  const std::string & source_floor,
  const std::string & source_map,
  const std::string & target_floor,
  const std::string & target_map)
{
  if (!validate_topology(topology).ok()) {
    return RouteResolution{
      RouteError::kInvalidTopology,
      "elevator topology is not valid",
      std::nullopt,
    };
  }
  if (
    !safe_asset_id(source_floor) || !safe_asset_id(source_map) ||
    !safe_asset_id(target_floor) || !safe_asset_id(target_map))
  {
    return RouteResolution{
      RouteError::kUnsafeFloorId,
      "source and target floor/map IDs must be path-safe",
      std::nullopt,
    };
  }
  if (source_floor == target_floor && source_map == target_map) {
    return RouteResolution{
      RouteError::kSameFloor,
      "source and target floors must be different",
      std::nullopt,
    };
  }

  const auto source = std::find_if(
    topology.floors.begin(), topology.floors.end(),
    [&source_floor, &source_map](const FloorElevatorTopology & floor) {
      return floor.floor_id == source_floor && floor.map_id == source_map;
    });
  const auto target = std::find_if(
    topology.floors.begin(), topology.floors.end(),
    [&target_floor, &target_map](const FloorElevatorTopology & floor) {
      return floor.floor_id == target_floor && floor.map_id == target_map;
    });
  if (source == topology.floors.end() || target == topology.floors.end()) {
    return RouteResolution{
      RouteError::kUnknownFloor,
      "source or target floor is not served by this elevator",
      std::nullopt,
    };
  }

  return RouteResolution{
    RouteError::kNone,
    "",
    ElevatorRoute{
      topology.elevator_id,
      topology.building_id,
      *source,
      *target,
      topology.schema_version,
    },
  };
}

}  // namespace robot_elevator_manager
