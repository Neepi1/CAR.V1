#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

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
  DoorThreshold threshold;
  std::array<ElevatorRuntimePose, 5> poses;
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
