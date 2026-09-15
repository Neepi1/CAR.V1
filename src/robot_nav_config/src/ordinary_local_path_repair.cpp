#include "robot_nav_config/ordinary_local_path_repair.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "tf2/utils.h"

namespace robot_nav_config
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

class BoundedHybridSearch
  : public nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeHybrid>
{
public:
  explicit BoundedHybridSearch(const nav2_smac_planner::SearchInfo & info)
  : AStarAlgorithm(nav2_smac_planner::MotionModel::DUBIN, info)
  {
    // Humble checks time only every 5000 expansions. That can consume the
    // entire short local-repair budget before the next candidate gets a turn.
    // Keep the stock algorithm, but check its existing deadline more often.
    _timing_interval = 64;
  }
};

double normalize_yaw(const double yaw)
{
  double normalized = std::fmod(yaw, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized;
}

unsigned int yaw_to_bin(const double yaw, const unsigned int bin_count)
{
  const double bin_size = kTwoPi / static_cast<double>(bin_count);
  auto bin = static_cast<unsigned int>(std::llround(normalize_yaw(yaw) / bin_size));
  if (bin >= bin_count) {
    bin -= bin_count;
  }
  return bin;
}

double pose_yaw(
  const nav_msgs::msg::Path & path,
  const std::size_t index)
{
  if (index + 1U < path.poses.size()) {
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1U].pose.position;
    const double dx = next.x - current.x;
    const double dy = next.y - current.y;
    if (std::hypot(dx, dy) > 1.0e-6) {
      return std::atan2(dy, dx);
    }
  }
  return tf2::getYaw(path.poses[index].pose.orientation);
}

bool pose_is_in_collision(
  nav2_smac_planner::GridCollisionChecker & checker,
  const nav2_costmap_2d::Costmap2D & costmap,
  const double x,
  const double y,
  const double yaw,
  const unsigned int angle_bins,
  const bool allow_unknown)
{
  const double mx = (x - costmap.getOriginX()) / costmap.getResolution();
  const double my = (y - costmap.getOriginY()) / costmap.getResolution();
  return checker.inCollision(
    static_cast<float>(mx), static_cast<float>(my),
    static_cast<float>(yaw_to_bin(yaw, angle_bins)), allow_unknown);
}

bool footprint_is_in_map(const OrdinaryLocalPathRepairRequest & request,
  const double x, const double y, const double yaw)
{
  unsigned int mx, my;
  for (const auto & point : request.footprint) {
    if (!request.costmap->worldToMap(
        x + std::cos(yaw) * point.x - std::sin(yaw) * point.y,
        y + std::sin(yaw) * point.x + std::cos(yaw) * point.y, mx, my))
    {
      return false;
    }
  }
  return true;
}

std::size_t nearest_path_index(
  const nav_msgs::msg::Path & path,
  const OrdinaryLocalPose & start)
{
  std::size_t nearest = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & point = path.poses[index].pose.position;
    const double distance = std::hypot(point.x - start.x, point.y - start.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = index;
    }
  }
  return nearest;
}

std::vector<double> cumulative_path_distances(
  const nav_msgs::msg::Path & path,
  const std::size_t start_index)
{
  std::vector<double> distances(path.poses.size(), 0.0);
  for (std::size_t index = start_index + 1U; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    distances[index] = distances[index - 1U] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return distances;
}

std::optional<std::size_t> first_blocked_path_index(
  const OrdinaryLocalPathRepairRequest & request,
  nav2_smac_planner::GridCollisionChecker & checker,
  const std::size_t nearest_index,
  const std::vector<double> & distances)
{
  const auto & path = request.reference_path;
  const auto & params = request.parameters;
  const double sample_step = std::max(0.01, request.costmap->getResolution() * 0.5);
  for (std::size_t index = nearest_index; index < path.poses.size(); ++index) {
    if (distances[index] > params.lookahead_distance_m) {
      break;
    }
    if (index == nearest_index) {
      continue;
    }

    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double length = std::hypot(dx, dy);
    const double yaw = length > 1.0e-6 ? std::atan2(dy, dx) : pose_yaw(path, index);
    const auto samples = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      if (pose_is_in_collision(
          checker, *request.costmap,
          previous.x + ratio * dx, previous.y + ratio * dy, yaw,
          params.angle_quantization_bins, params.allow_unknown))
      {
        return index;
      }
    }
  }
  return std::nullopt;
}

std::vector<std::size_t> candidate_rejoin_indices(
  const OrdinaryLocalPathRepairRequest & request,
  nav2_smac_planner::GridCollisionChecker & checker,
  const std::size_t blocked_index,
  const std::vector<double> & distances)
{
  std::vector<std::size_t> candidates;
  const double blocked_distance = distances[blocked_index];
  for (std::size_t index = blocked_index + 1U;
    index < request.reference_path.poses.size(); ++index)
  {
    // The 4 m inspection horizon is not a rejoin boundary. A blockage near
    // that horizon needs an exit farther ahead, bounded by this local map.
    const auto & candidate = request.reference_path.poses[index].pose.position;
    unsigned int mx, my;
    if (!request.costmap->worldToMap(candidate.x, candidate.y, mx, my)) {
      break;
    }
    if (distances[index] - blocked_distance <
      request.parameters.rejoin_distance_after_blockage_m)
    {
      continue;
    }
    const auto & pose = request.reference_path.poses[index];
    if (footprint_is_in_map(request, candidate.x, candidate.y,
        pose_yaw(request.reference_path, index)) && !pose_is_in_collision(
        checker, *request.costmap, pose.pose.position.x, pose.pose.position.y,
        pose_yaw(request.reference_path, index),
        request.parameters.angle_quantization_bins,
        request.parameters.allow_unknown))
    {
      candidates.push_back(index);
    }
  }
  std::reverse(candidates.begin(), candidates.end());
  // Far, middle and near exits, not three adjacent cells at the far end.
  if (candidates.size() > 3U) {
    const double middle = 0.5 * (distances[candidates.front()] +
      distances[candidates.back()]);
    const auto midpoint = std::min_element(candidates.begin(), candidates.end(),
      [&](const auto a, const auto b) {
        return std::abs(distances[a] - middle) < std::abs(distances[b] - middle);
      });
    candidates = {candidates.front(), *midpoint, candidates.back()};
  }
  return candidates;
}

nav2_smac_planner::SearchInfo make_search_info(
  const OrdinaryLocalPathRepairParameters & params,
  const double resolution)
{
  nav2_smac_planner::SearchInfo search_info{};
  search_info.minimum_turning_radius = static_cast<float>(
    params.minimum_turning_radius_m / resolution);
  search_info.non_straight_penalty = 1.05F;
  search_info.change_penalty = 0.20F;
  search_info.reverse_penalty = 2.10F;
  search_info.cost_penalty = 2.0F;
  search_info.retrospective_penalty = 0.015F;
  search_info.rotation_penalty = 5.0F;
  search_info.analytic_expansion_ratio = 3.5F;
  search_info.analytic_expansion_max_length = static_cast<float>(
    params.analytic_expansion_max_length_m / resolution);
  search_info.cache_obstacle_heuristic = false;
  search_info.allow_reverse_expansion = false;
  return search_info;
}

nav_msgs::msg::Path compose_repaired_path(
  const OrdinaryLocalPathRepairRequest & request,
  const nav2_smac_planner::NodeHybrid::CoordinateVector & reverse_path,
  const std::size_t rejoin_index)
{
  nav_msgs::msg::Path repaired;
  repaired.header = request.reference_path.header;
  repaired.header.frame_id = request.costmap_frame;
  repaired.poses.reserve(reverse_path.size() +
    request.reference_path.poses.size() - rejoin_index);

  for (auto iterator = reverse_path.rbegin(); iterator != reverse_path.rend(); ++iterator) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = repaired.header;
    pose.pose = nav2_smac_planner::getWorldCoords(iterator->x, iterator->y, request.costmap.get());
    pose.pose.orientation = nav2_smac_planner::getWorldOrientation(iterator->theta);
    repaired.poses.push_back(std::move(pose));
  }

  // Keep the untouched suffix and, critically, the exact original terminal
  // pose. The local repair changes only the blocked prefix of the path.
  for (std::size_t index = rejoin_index;
    index < request.reference_path.poses.size(); ++index)
  {
    auto pose = request.reference_path.poses[index];
    pose.header.frame_id = repaired.header.frame_id;
    repaired.poses.push_back(std::move(pose));
  }
  return repaired;
}

double maximum_lateral_deviation(
  const nav_msgs::msg::Path & path,
  const OrdinaryLocalPose & start,
  const geometry_msgs::msg::Point & goal)
{
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1.0e-9) {
    return 0.0;
  }
  double maximum = 0.0;
  for (const auto & pose : path.poses) {
    const double point_dx = pose.pose.position.x - start.x;
    const double point_dy = pose.pose.position.y - start.y;
    maximum = std::max(maximum, std::abs(dx * point_dy - dy * point_dx) / length);
  }
  return maximum;
}

}  // namespace

OrdinaryLocalPathInspection inspect_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & request,
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  OrdinaryLocalPathInspection inspection;
  const auto & params = request.parameters;
  if (!request.costmap || !node || request.costmap_frame.empty() ||
    request.reference_path.poses.size() < 2U || request.footprint.size() < 3U ||
    request.reference_path.header.frame_id != request.costmap_frame ||
    !std::isfinite(request.start.x) || !std::isfinite(request.start.y) ||
    !std::isfinite(request.start.yaw) || params.lookahead_distance_m <= 0.0 ||
    params.angle_quantization_bins < 8U)
  {
    return inspection;
  }

  nav2_smac_planner::GridCollisionChecker collision_checker(
    request.costmap.get(), params.angle_quantization_bins, node);
  collision_checker.setFootprint(request.preferred_footprint.empty() ?
    request.footprint : request.preferred_footprint, false, 0.0);
  inspection.nearest_index = nearest_path_index(request.reference_path, request.start);
  const auto distances = cumulative_path_distances(
    request.reference_path, inspection.nearest_index);
  const auto blocked = first_blocked_path_index(
    request, collision_checker, inspection.nearest_index, distances);
  inspection.valid = true;
  inspection.path_clear = !blocked.has_value();
  if (blocked.has_value()) {
    inspection.first_blocked_index = *blocked;
  }
  return inspection;
}

OrdinaryLocalPathRepairResult repair_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & request,
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  if (!request.preferred_footprint.empty()) {
    // Prefer the outer MPPI envelope, but do not turn it into a new hard
    // infeasibility condition at a tight start/rejoin. Both attempts share the
    // original expansion/time allowance (cold lookup setup remains excluded).
    auto preferred = request;
    preferred.footprint = std::move(preferred.preferred_footprint);
    preferred.preferred_footprint.clear();
    preferred.parameters.max_planning_time_sec *= 0.5;
    preferred.parameters.max_iterations /= 2;
    auto result = repair_ordinary_local_path(preferred, node);
    if (result.status == OrdinaryLocalPathRepairStatus::kSuccess ||
      result.status == OrdinaryLocalPathRepairStatus::kPathClear ||
      result.status == OrdinaryLocalPathRepairStatus::kInvalidInput)
    {
      result.used_preferred_clearance = true;
      return result;
    }
    auto fallback = request;
    fallback.preferred_footprint.clear();
    fallback.parameters.max_planning_time_sec *= 0.5;
    fallback.parameters.max_iterations -= result.iterations;
    auto fallback_result = repair_ordinary_local_path(fallback, node);
    fallback_result.iterations += result.iterations;
    fallback_result.attempt_iterations.insert(fallback_result.attempt_iterations.begin(),
      result.attempt_iterations.begin(), result.attempt_iterations.end());
    return fallback_result;
  }
  auto started = std::chrono::steady_clock::now();
  OrdinaryLocalPathRepairResult result;
  const auto & params = request.parameters;
  if (!request.costmap || !node || request.costmap_frame.empty() ||
    request.reference_path.poses.size() < 2U || request.footprint.size() < 3U ||
    request.reference_path.header.frame_id != request.costmap_frame ||
    !std::isfinite(request.start.x) || !std::isfinite(request.start.y) ||
    !std::isfinite(request.start.yaw) || params.minimum_turning_radius_m <= 0.0 ||
    params.lookahead_distance_m <= 0.0 || params.angle_quantization_bins < 8U ||
    params.max_iterations <= 0 || params.max_planning_time_sec <= 0.0)
  {
    return result;
  }

  const auto inspection = inspect_ordinary_local_path(request, node);
  if (!inspection.valid) {
    return result;
  }
  result.nearest_index = inspection.nearest_index;
  result.first_blocked_index = inspection.first_blocked_index;
  if (inspection.path_clear) {
    result.status = OrdinaryLocalPathRepairStatus::kPathClear;
    return result;
  }

  nav2_smac_planner::GridCollisionChecker collision_checker(
    request.costmap.get(), params.angle_quantization_bins, node);
  // Zero disables the inflation-layer shortcut. A detached snapshot has no
  // LayeredCostmap metadata, so every candidate must check the full rectangle.
  collision_checker.setFootprint(request.footprint, false, 0.0);
  const auto distances = cumulative_path_distances(request.reference_path, result.nearest_index);
  const auto candidates = candidate_rejoin_indices(
    request, collision_checker, result.first_blocked_index, distances);
  if (candidates.empty()) {
    result.status = OrdinaryLocalPathRepairStatus::kNoRejoinPose;
    return result;
  }

  unsigned int start_x = 0U;
  unsigned int start_y = 0U;
  if (!request.costmap->worldToMap(request.start.x, request.start.y, start_x, start_y) ||
    pose_is_in_collision(
      collision_checker, *request.costmap, request.start.x, request.start.y,
      request.start.yaw, params.angle_quantization_bins, params.allow_unknown))
  {
    result.status = OrdinaryLocalPathRepairStatus::kNoPath;
    return result;
  }

  const double resolution = request.costmap->getResolution();
  // The worker is long-lived and serial. Keep its expensive Dubins lookup
  // table across equivalent requests, but never its visited graph. A changed
  // resolution/kinematic model requires a fresh table.
  using SearchKey = std::tuple<double, double, double, double, unsigned int>;
  const SearchKey key{resolution, params.minimum_turning_radius_m,
    params.lookup_table_size_m, params.analytic_expansion_max_length_m,
    params.angle_quantization_bins};
  thread_local std::optional<SearchKey> cached_key;
  thread_local std::unique_ptr<BoundedHybridSearch> cached_search;
  const bool cold = !cached_search || cached_key != key;
  if (cold) {
    cached_search = std::make_unique<BoundedHybridSearch>(make_search_info(params, resolution));
    cached_key = key;
  }
  auto & planner = *cached_search;
  int max_iterations = params.max_iterations;
  float lookup_table_cells = static_cast<float>(
    params.lookup_table_size_m / request.costmap->getResolution());
  lookup_table_cells = std::max(3.0F, std::floor(lookup_table_cells));
  if (static_cast<int>(lookup_table_cells) % 2 == 0) {
    lookup_table_cells += 1.0F;
  }
  const auto setup_started = std::chrono::steady_clock::now();
  planner.initialize(
    params.allow_unknown, max_iterations, params.max_on_approach_iterations,
    params.max_planning_time_sec, lookup_table_cells,
    params.angle_quantization_bins);
  if (cold) {
    // One-time table setup is outside the shared expansion budget; subsequent
    // requests include all setup. It runs only on the worker, never the 15 Hz
    // command callback. Do not let cold setup starve every candidate.
    started += std::chrono::steady_clock::now() - setup_started;
  }
  constexpr std::size_t kMaximumCandidateAttempts = 3U;
  const std::size_t attempt_count = std::min(kMaximumCandidateAttempts, candidates.size());
  for (std::size_t attempt = 0U; attempt < attempt_count; ++attempt) {
    const double remaining = params.max_planning_time_sec -
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (remaining <= 0.0 || result.iterations >= params.max_iterations) {
      break;
    }
    // Reserve part of the one request budget for other independent exits.
    const auto remaining_attempts = static_cast<int>(attempt_count - attempt);
    int iteration_budget = (params.max_iterations - result.iterations) / remaining_attempts;
    planner.initialize(params.allow_unknown, iteration_budget,
      params.max_on_approach_iterations, remaining / remaining_attempts,
      lookup_table_cells, params.angle_quantization_bins);
    // Humble createPath() only clears its queue, not visited nodes. Reset the
    // graph, then both endpoints, for every candidate.
    planner.setCollisionChecker(&collision_checker);
    planner.setStart(start_x, start_y,
      yaw_to_bin(request.start.yaw, params.angle_quantization_bins));
    const std::size_t rejoin_index = candidates[attempt];
    const auto & goal_pose = request.reference_path.poses[rejoin_index];
    unsigned int goal_x = 0U;
    unsigned int goal_y = 0U;
    if (!request.costmap->worldToMap(
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_x, goal_y))
    {
      continue;
    }
    planner.setGoal(
      goal_x, goal_y,
      yaw_to_bin(pose_yaw(request.reference_path, rejoin_index),
      params.angle_quantization_bins));

    nav2_smac_planner::NodeHybrid::CoordinateVector reverse_path;
    int iterations = 0;
    bool created = false;
    try {
      created = planner.createPath(reverse_path, iterations,
        static_cast<float>(params.goal_tolerance_m / request.costmap->getResolution()));
    } catch (const std::runtime_error & error) {
      const std::string message(error.what());
      if (message != "Starting point in lethal space! Cannot create feasible plan." &&
        message != "Failed to compute path, goal is occupied with no tolerance.")
      {
        throw;
      }
      // Grid-quantized endpoints may be occupied even when the continuous
      // pose passed inspection. This is no path, not a worker/program fault.
    }
    result.iterations += iterations;
    result.attempt_iterations.push_back(iterations);
    if (!created || reverse_path.size() < 2U) {
      continue;
    }

    result.rejoin_index = rejoin_index;
    result.path = compose_repaired_path(request, reverse_path, rejoin_index);
    result.maximum_lateral_deviation_m = maximum_lateral_deviation(
      result.path, request.start,
      request.reference_path.poses.back().pose.position);
    result.status = OrdinaryLocalPathRepairStatus::kSuccess;
    return result;
  }

  result.status = OrdinaryLocalPathRepairStatus::kNoPath;
  return result;
}

std::optional<nav_msgs::msg::Path> attach_ordinary_local_path(
  const OrdinaryLocalPathRepairRequest & latest,
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  // Attachment is hard-clearance acceptance, not a second preference gate.
  // In particular a moving robot may now be inside the preferred envelope.
  auto hard_check = latest;
  hard_check.preferred_footprint.clear();
  const auto inspection = inspect_ordinary_local_path(hard_check, node);
  if (!inspection.valid || !inspection.path_clear) {return std::nullopt;}
  const auto & params = latest.parameters;
  const auto & path = latest.reference_path;
  nav2_smac_planner::GridCollisionChecker checker(
    latest.costmap.get(), params.angle_quantization_bins, node);
  checker.setFootprint(latest.footprint, false, 0.0);
  auto space = std::make_shared<ompl::base::DubinsStateSpace>(
    params.minimum_turning_radius_m, false);
  ompl::base::ScopedState<> start(space), end(space), sample(space);
  auto * s = start->as<ompl::base::SE2StateSpace::StateType>();
  s->setXY(latest.start.x, latest.start.y);
  s->setYaw(latest.start.yaw);
  const auto distances = cumulative_path_distances(path, inspection.nearest_index);
  double next_distance = 0.0;
  for (std::size_t i = inspection.nearest_index; i < path.poses.size(); ++i) {
    if (distances[i] > 2.0 * params.minimum_turning_radius_m) {break;}
    if (distances[i] + 1.0e-6 < next_distance) {continue;}
    next_distance = distances[i] + params.minimum_turning_radius_m * 0.5;
    const auto & target = path.poses[i].pose.position;
    auto * e = end->as<ompl::base::SE2StateSpace::StateType>();
    e->setXY(target.x, target.y);
    e->setYaw(tf2::getYaw(path.poses[i].pose.orientation));
    const double length = space->distance(start.get(), end.get());
    const double chord = std::hypot(target.x - latest.start.x, target.y - latest.start.y);
    // Reject loop-around connectors; ask the worker for a fresh route instead.
    if (!std::isfinite(length) || length > 2.0 * chord + latest.costmap->getResolution()) {
      continue;
    }
    const double step = std::max(0.01, latest.costmap->getResolution() * 0.5);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    nav_msgs::msg::Path attached;
    attached.header = path.header;
    bool clear = true;
    for (int j = 0; j <= samples; ++j) {
      space->interpolate(start.get(), end.get(), static_cast<double>(j) / samples, sample.get());
      const auto * p = sample->as<ompl::base::SE2StateSpace::StateType>();
      if (!footprint_is_in_map(latest, p->getX(), p->getY(), p->getYaw()) ||
        pose_is_in_collision(checker, *latest.costmap, p->getX(), p->getY(),
          p->getYaw(), params.angle_quantization_bins, params.allow_unknown))
      {
        clear = false;
        break;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p->getX();
      pose.pose.position.y = p->getY();
      pose.pose.orientation.z = std::sin(p->getYaw() * 0.5);
      pose.pose.orientation.w = std::cos(p->getYaw() * 0.5);
      attached.poses.push_back(std::move(pose));
    }
    if (clear) {
      attached.poses.insert(attached.poses.end(), path.poses.begin() + i, path.poses.end());
      return attached;
    }
  }
  return std::nullopt;
}

const char * ordinary_local_path_repair_status_name(
  const OrdinaryLocalPathRepairStatus status) noexcept
{
  switch (status) {
    case OrdinaryLocalPathRepairStatus::kInvalidInput:
      return "invalid_input";
    case OrdinaryLocalPathRepairStatus::kPathClear:
      return "path_clear";
    case OrdinaryLocalPathRepairStatus::kNoRejoinPose:
      return "no_rejoin_pose";
    case OrdinaryLocalPathRepairStatus::kNoPath:
      return "no_path";
    case OrdinaryLocalPathRepairStatus::kSuccess:
      return "success";
  }
  return "unknown";
}

}  // namespace robot_nav_config
