#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_elevator_manager
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct DoorThreshold
{
  Point2 left;
  Point2 right;
  Point2 cabin_reference;
  double clearance_m{0.05};
  double jamb_clearance_m{0.05};
};

enum class PoseRole
{
  kHallCall,
  kLanding,
  kHallWait,
  kDoorway,
  kCabin,
  kExit,
  kCabinPanel,
};

enum class PanelSide
{
  kUnknown,
  kLeft,
  kRight,
};

struct PoseBinding
{
  PoseRole role{PoseRole::kHallCall};
  std::string pose_id;
};

struct FloorElevatorTopology
{
  std::string floor_id;
  std::string map_id;
  std::vector<PoseBinding> poses;
  std::optional<DoorThreshold> threshold;
  PanelSide hall_call_panel_side{PanelSide::kUnknown};
  PanelSide cabin_panel_side{PanelSide::kUnknown};
};

struct ElevatorTopology
{
  std::string elevator_id;
  std::string building_id;
  std::vector<FloorElevatorTopology> floors;
  std::uint32_t schema_version{2U};
};

enum class TopologyIssueCode
{
  kUnsafeElevatorId,
  kUnsafeBuildingId,
  kUnsafeFloorId,
  kUnsafeMapId,
  kUnsafePoseId,
  kTooFewFloors,
  kDuplicateFloor,
  kMissingPoseRole,
  kDuplicatePoseRole,
  kUnknownPoseRole,
  kDuplicatePoseId,
  kInvalidThreshold,
  kMissingPanelSide,
  kUnsupportedSchema,
};

struct TopologyIssue
{
  TopologyIssueCode code{TopologyIssueCode::kUnsafeElevatorId};
  std::string field;
  std::string message;
};

struct TopologyValidation
{
  std::vector<TopologyIssue> issues;

  bool ok() const noexcept;
};

struct ElevatorRoute
{
  std::string elevator_id;
  std::string building_id;
  FloorElevatorTopology source;
  FloorElevatorTopology target;
  // Untrusted/manual construction must not silently become executable v2.
  // resolve_route() fills this only after validating the topology schema.
  std::uint32_t schema_version{0U};
};

enum class RouteError
{
  kNone,
  kInvalidTopology,
  kUnsafeFloorId,
  kSameFloor,
  kUnknownFloor,
};

struct RouteResolution
{
  RouteError error{RouteError::kNone};
  std::string message;
  std::optional<ElevatorRoute> route;

  bool ok() const noexcept;
};

bool safe_asset_id(const std::string & value) noexcept;
bool safe_pose_id(const std::string & value) noexcept;
std::string to_string(PoseRole role);
std::string to_string(PanelSide side);
std::optional<PanelSide> panel_side_from_string(const std::string & value);
const std::vector<PoseRole> & required_pose_roles(std::uint32_t schema_version = 2U);
TopologyValidation validate_topology(const ElevatorTopology & topology);
std::optional<std::string> find_pose_id(
  const FloorElevatorTopology & floor,
  PoseRole role);
RouteResolution resolve_route(
  const ElevatorTopology & topology,
  const std::string & source_floor,
  const std::string & source_map,
  const std::string & target_floor,
  const std::string & target_map);

}  // namespace robot_elevator_manager
