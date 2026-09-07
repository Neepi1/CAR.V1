#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server::features::docking::configuration
{

struct DockingPredockPoseResolverConfig
{
  bool distance_check_enabled{false};
  double min_distance_m{0.50};
  double max_distance_m{1.20};
  double max_yaw_error_rad{0.80};
};

struct DockingPredockPoseResolverPorts
{
  std::function<std::optional<StoredPose>(
      const std::string &,
      const std::string &,
      const std::string &)>
  find_pose;
  std::function<std::vector<StoredPose>(
      const std::string &,
      const std::string &)>
  read_poses;
};

// Resolves the one configured/manual predock pose for a dock and applies the
// complete commissioning-time geometry sanity policy. It does not mutate map
// assets and does not command motion.
class DockingPredockPoseResolver
{
public:
  DockingPredockPoseResolver(
    DockingPredockPoseResolverConfig config,
    DockingPredockPoseResolverPorts ports);

  std::optional<StoredPose> resolve(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & dock_id,
    const StoredPose & dock_pose,
    const std::string & requested_predock_pose_id,
    std::string & source,
    std::string & error,
    int & error_status) const;

  bool validate(
    const StoredPose & dock_pose,
    const StoredPose & predock_pose,
    std::string & error) const;

private:
  static bool is_predock_pose_type(const StoredPose & pose);
  static std::vector<std::string> pose_id_candidates(const std::string & dock_id);

  DockingPredockPoseResolverConfig config_;
  DockingPredockPoseResolverPorts ports_;
};

}  // namespace robot_api_server::features::docking::configuration
