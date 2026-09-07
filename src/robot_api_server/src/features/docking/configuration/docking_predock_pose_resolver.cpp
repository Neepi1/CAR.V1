#include "robot_api_server/features/docking/configuration/docking_predock_pose_resolver.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "robot_api_server/features/localization/tf_pose_utils.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::docking::configuration
{

DockingPredockPoseResolver::DockingPredockPoseResolver(
  DockingPredockPoseResolverConfig config,
  DockingPredockPoseResolverPorts ports)
: config_(std::move(config)), ports_(std::move(ports))
{
  if (!ports_.find_pose || !ports_.read_poses) {
    throw std::invalid_argument("DockingPredockPoseResolver requires all ports");
  }
}

bool DockingPredockPoseResolver::is_predock_pose_type(const StoredPose & pose)
{
  const auto type = lower_copy(pose.type);
  return type == "dock_predock" || type == "predock" || type == "dock_approach";
}

std::vector<std::string> DockingPredockPoseResolver::pose_id_candidates(
  const std::string & dock_id)
{
  return {
    dock_id + "_predock",
    dock_id + "_pre_dock",
    dock_id + "_approach",
    "predock_" + dock_id,
    "pre_dock_" + dock_id,
    "approach_" + dock_id};
}

std::optional<StoredPose> DockingPredockPoseResolver::resolve(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & dock_id,
  const StoredPose & dock_pose,
  const std::string & requested_predock_pose_id,
  std::string & source,
  std::string & error,
  int & error_status) const
{
  source.clear();
  error.clear();
  error_status = 0;

  if (!requested_predock_pose_id.empty()) {
    if (!safe_pose_id(requested_predock_pose_id)) {
      error_status = 400;
      error = "valid predock_pose_id is required";
      return std::nullopt;
    }
    auto pose = ports_.find_pose(
      building_id, floor_id, requested_predock_pose_id);
    if (!pose) {
      error_status = 404;
      error = "predock_pose_id not found in poses.yaml: " + requested_predock_pose_id;
      return std::nullopt;
    }
    source = "manual_predock_explicit";
    return pose;
  }

  for (const auto & candidate_id : pose_id_candidates(dock_id)) {
    if (auto pose = ports_.find_pose(building_id, floor_id, candidate_id)) {
      source = "manual_predock_auto_id";
      return pose;
    }
  }

  const auto poses = ports_.read_poses(building_id, floor_id);
  std::vector<StoredPose> typed_predocks;
  std::vector<StoredPose> named_predocks;
  const auto dock_name_predock = lower_copy(dock_pose.name + "_predock");
  const auto dock_name_pre_dock = lower_copy(dock_pose.name + "_pre_dock");
  const auto dock_name_approach = lower_copy(dock_pose.name + "_approach");

  for (const auto & pose : poses) {
    if (pose.id == dock_id || !is_predock_pose_type(pose)) {
      continue;
    }
    typed_predocks.push_back(pose);
    const auto pose_id = lower_copy(pose.id);
    const auto pose_name = lower_copy(pose.name);
    bool named_match = false;
    for (const auto & candidate_id : pose_id_candidates(dock_id)) {
      const auto candidate = lower_copy(candidate_id);
      if (pose_id == candidate || pose_name == candidate) {
        named_match = true;
        break;
      }
    }
    if (pose_name == dock_name_predock || pose_name == dock_name_pre_dock ||
      pose_name == dock_name_approach)
    {
      named_match = true;
    }
    if (named_match) {
      named_predocks.push_back(pose);
    }
  }

  if (named_predocks.size() == 1U) {
    source = "manual_predock_auto_name";
    return named_predocks.front();
  }
  if (named_predocks.size() > 1U) {
    error_status = 409;
    error = "multiple matching dock predock poses found; pass predock_pose_id explicitly";
    return std::nullopt;
  }
  if (typed_predocks.size() == 1U) {
    source = "manual_predock_auto_unique_type";
    return typed_predocks.front();
  }
  if (typed_predocks.size() > 1U) {
    error_status = 409;
    error =
      "multiple dock_predock poses found; pass predock_pose_id or name one " +
      dock_id + "_predock";
    return std::nullopt;
  }
  return std::nullopt;
}

bool DockingPredockPoseResolver::validate(
  const StoredPose & dock_pose,
  const StoredPose & predock_pose,
  std::string & error) const
{
  const double distance = std::hypot(
    predock_pose.x - dock_pose.x, predock_pose.y - dock_pose.y);
  const double yaw_error = std::fabs(normalize_angle(predock_pose.yaw - dock_pose.yaw));
  const bool distance_ok = !config_.distance_check_enabled ||
    (distance >= config_.min_distance_m && distance <= config_.max_distance_m);
  const bool yaw_ok = yaw_error <= config_.max_yaw_error_rad;
  if (distance_ok && yaw_ok) {
    return true;
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "manual predock pose " << predock_pose.id
      << " failed docking sanity check: distance=" << distance;
  if (config_.distance_check_enabled) {
    out << " allowed=[" << config_.min_distance_m
        << "," << config_.max_distance_m << "]";
  } else {
    out << " distance_check=disabled";
  }
  out << " yaw_error=" << yaw_error
      << " max_yaw_error=" << config_.max_yaw_error_rad
      << "; resave the predock point with a heading aligned to the charger";
  error = out.str();
  return false;
}

}  // namespace robot_api_server::features::docking::configuration
