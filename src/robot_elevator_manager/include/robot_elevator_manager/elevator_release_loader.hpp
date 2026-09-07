#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_elevator_manager
{

enum class ElevatorReleaseLoadError
{
  kNone,
  kInvalidRequest,
  kUnsafePath,
  kReleaseNotFound,
  kInvalidMetadata,
  kInvalidContent,
  kInvalidConfiguration,
  kInvalidTopology,
  kInvalidInternalPoses,
  kLegacyReadOnly,
  kNoRoute,
};

struct ElevatorReleaseLoadRequest
{
  std::string config_root;
  std::string building_id;
  std::string source_floor_id;
  std::string source_map_id;
  std::string target_floor_id;
  std::string target_map_id;
  std::string preferred_elevator_id;
  std::string expected_release_id;
};

struct ElevatorRuntimePose
{
  PoseRole role{PoseRole::kHallCall};
  std::string pose_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ElevatorRuntimeFloor
{
  std::string floor_id;
  std::string map_id;
  std::uint64_t map_asset_epoch{0U};
  std::string map_asset_digest;
  std::vector<ElevatorRuntimePose> poses;
  PanelSide hall_call_panel_side{PanelSide::kUnknown};
  PanelSide cabin_panel_side{PanelSide::kUnknown};
};

struct FrozenElevatorRelease
{
  std::string release_id;
  std::uint64_t generation{0U};
  std::string configuration_digest;
  std::string building_id;
  std::string elevator_id;
  ElevatorRuntimeFloor source;
  ElevatorRuntimeFloor target;
  std::uint32_t schema_version{2U};
};

struct ElevatorReleaseLoadResult
{
  ElevatorReleaseLoadError error{ElevatorReleaseLoadError::kInvalidContent};
  std::string message;
  std::optional<FrozenElevatorRelease> release;

  bool ok() const noexcept;
};

ElevatorReleaseLoadResult load_elevator_release(
  const ElevatorReleaseLoadRequest & request);

const char * to_string(ElevatorReleaseLoadError error) noexcept;

}  // namespace robot_elevator_manager
