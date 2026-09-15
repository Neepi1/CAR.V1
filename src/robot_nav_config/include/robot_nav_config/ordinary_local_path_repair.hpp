#pragma once

#include <cstddef>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot_nav_config
{

struct OrdinaryLocalPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct OrdinaryLocalPathRepairParameters
{
  double minimum_turning_radius_m{0.81};
  double lookahead_distance_m{4.0};
  double rejoin_distance_after_blockage_m{0.80};
  double goal_tolerance_m{0.05};
  double max_planning_time_sec{0.40};
  double lookup_table_size_m{5.0};
  double analytic_expansion_max_length_m{4.05};
  int max_iterations{60000};
  int max_on_approach_iterations{500};
  unsigned int angle_quantization_bins{72U};
  bool allow_unknown{false};
};

enum class OrdinaryLocalPathRepairStatus
{
  kInvalidInput,
  kPathClear,
  kNoRejoinPose,
  kNoPath,
  kSuccess,
};

struct OrdinaryLocalPathRepairRequest
{
  std::chrono::steady_clock::time_point snapshot_time{};
  std::string costmap_frame;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  std::vector<geometry_msgs::msg::Point> footprint;
  // Optional preferred envelope; failure falls back to the commissioned
  // hard envelope above. Never applied to the shared live costmap footprint.
  std::vector<geometry_msgs::msg::Point> preferred_footprint;
  OrdinaryLocalPose start;
  nav_msgs::msg::Path reference_path;
  OrdinaryLocalPathRepairParameters parameters;
};

struct OrdinaryLocalPathRepairResult
{
  OrdinaryLocalPathRepairStatus status{OrdinaryLocalPathRepairStatus::kInvalidInput};
  nav_msgs::msg::Path path;
  std::size_t nearest_index{0U};
  std::size_t first_blocked_index{0U};
  std::size_t rejoin_index{0U};
  int iterations{0};
  std::vector<int> attempt_iterations;
  double maximum_lateral_deviation_m{0.0};
  bool used_preferred_clearance{false};
};

struct OrdinaryLocalPathInspection
{
  bool valid{false};
  bool path_clear{false};
  std::size_t nearest_index{0U};
  std::size_t first_blocked_index{0U};
};

OrdinaryLocalPathInspection inspect_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & request,
  const nav2_util::LifecycleNode::SharedPtr & node);

OrdinaryLocalPathRepairResult repair_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & request,
  const nav2_util::LifecycleNode::SharedPtr & node);

// Reattach a moving robot to a returned path using a bounded forward Dubins
// connector, checked with the same footprint on the latest snapshot.
std::optional<nav_msgs::msg::Path> attach_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & latest,
  const nav2_util::LifecycleNode::SharedPtr & node);

const char * ordinary_local_path_repair_status_name(
  OrdinaryLocalPathRepairStatus status) noexcept;

}  // namespace robot_nav_config
