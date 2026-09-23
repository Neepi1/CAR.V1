#include "robot_nav_config/ordinary_local_path_repair_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config
{

void OrdinaryLocalPathRepairRuntime::configure(
  const nav2_util::LifecycleNode::SharedPtr & node,
  const std::string & parameter_prefix,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const double terminal_control_distance_m)
{
  if (!node || !tf || !costmap_ros || costmap_ros->getCostmap() == nullptr) {
    throw std::runtime_error("OrdinaryLocalPathRepairRuntime received invalid dependencies");
  }
  node_ = node;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();

  const auto key = [&parameter_prefix](const std::string & suffix) {
      return parameter_prefix + ".ordinary_local_repair_" + suffix;
    };
  nav2_util::declare_parameter_if_not_declared(
    node_, key("enabled"), rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("collision_margin_m"), rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("minimum_goal_distance_m"), rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("inspection_period_sec"), rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("persistence_sec"), rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("retry_period_sec"), rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("minimum_turning_radius_m"), rclcpp::ParameterValue(0.81));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("lookahead_distance_m"), rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("rejoin_distance_m"), rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("goal_tolerance_m"), rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("max_planning_time_sec"), rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("lookup_table_size_m"), rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("analytic_expansion_max_length_m"), rclcpp::ParameterValue(4.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("max_iterations"), rclcpp::ParameterValue(60000));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("max_on_approach_iterations"), rclcpp::ParameterValue(500));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("angle_quantization_bins"), rclcpp::ParameterValue(72));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("progress_topic"),
    rclcpp::ParameterValue(std::string(
      "/ranger_mini3/nav_elevator_scoped_progress_state")));
  nav2_util::declare_parameter_if_not_declared(
    node_, key("path_topic"),
    rclcpp::ParameterValue(std::string(
      "/ranger_mini3/ordinary_local_repair_path")));

  node_->get_parameter(key("enabled"), enabled_);
  std::string primary;
  node_->get_parameter(parameter_prefix + ".primary_controller", primary);
  mppi_primary_ = primary == "nav2_mppi_controller::MPPIController" ||
    primary == "robot_nav_config::RangerMPPIController";
  node_->get_parameter(key("collision_margin_m"), collision_margin_m_);
  // Share the MPPI preference width, not its unrelated native inflation cost.
  // No configured preference critic means legacy repair geometry.
  bool clearance_enabled = false;
  preferred_margin_m_ = 0.0;
  node_->get_parameter(parameter_prefix + ".RangerClearanceCritic.enabled", clearance_enabled);
  if (clearance_enabled) {
    node_->get_parameter(parameter_prefix + ".RangerClearanceCritic.preferred_margin",
      preferred_margin_m_);
    if (!std::isfinite(preferred_margin_m_) || preferred_margin_m_ <= 0.0) {
      throw std::invalid_argument("Ordinary repair: invalid MPPI preference width");
    }
  }
  node_->get_parameter(key("minimum_goal_distance_m"), minimum_goal_distance_m_);
  node_->get_parameter(key("inspection_period_sec"), inspection_period_sec_);
  node_->get_parameter(key("persistence_sec"), persistence_sec_);
  node_->get_parameter(key("retry_period_sec"), retry_period_sec_);
  node_->get_parameter(
    key("minimum_turning_radius_m"), parameters_.minimum_turning_radius_m);
  node_->get_parameter(key("lookahead_distance_m"), parameters_.lookahead_distance_m);
  node_->get_parameter(
    key("rejoin_distance_m"), parameters_.rejoin_distance_after_blockage_m);
  node_->get_parameter(key("goal_tolerance_m"), parameters_.goal_tolerance_m);
  node_->get_parameter(
    key("max_planning_time_sec"), parameters_.max_planning_time_sec);
  node_->get_parameter(key("lookup_table_size_m"), parameters_.lookup_table_size_m);
  node_->get_parameter(
    key("analytic_expansion_max_length_m"),
    parameters_.analytic_expansion_max_length_m);
  node_->get_parameter(key("max_iterations"), parameters_.max_iterations);
  node_->get_parameter(
    key("max_on_approach_iterations"), parameters_.max_on_approach_iterations);
  int angle_bins = 72;
  node_->get_parameter(key("angle_quantization_bins"), angle_bins);
  node_->get_parameter(key("progress_topic"), progress_topic_);
  node_->get_parameter(key("path_topic"), repair_path_topic_);

  collision_margin_m_ = std::max(0.0, collision_margin_m_);
  minimum_goal_distance_m_ = std::max(
    std::max(0.0, terminal_control_distance_m), minimum_goal_distance_m_);
  inspection_period_sec_ = std::max(0.05, inspection_period_sec_);
  persistence_sec_ = std::max(0.0, persistence_sec_);
  retry_period_sec_ = std::max(inspection_period_sec_, retry_period_sec_);
  parameters_.minimum_turning_radius_m = std::max(
    0.10, parameters_.minimum_turning_radius_m);
  parameters_.lookahead_distance_m = std::max(1.0, parameters_.lookahead_distance_m);
  parameters_.rejoin_distance_after_blockage_m = std::max(
    0.20, parameters_.rejoin_distance_after_blockage_m);
  parameters_.goal_tolerance_m = std::max(0.01, parameters_.goal_tolerance_m);
  parameters_.max_planning_time_sec = std::max(
    0.05, parameters_.max_planning_time_sec);
  parameters_.lookup_table_size_m = std::max(1.0, parameters_.lookup_table_size_m);
  parameters_.analytic_expansion_max_length_m = std::max(
    4.0 * parameters_.minimum_turning_radius_m,
    parameters_.analytic_expansion_max_length_m);
  parameters_.max_iterations = std::max(1000, parameters_.max_iterations);
  parameters_.max_on_approach_iterations = std::max(
    10, parameters_.max_on_approach_iterations);
  parameters_.angle_quantization_bins = static_cast<unsigned int>(
    std::max(16, angle_bins));

  if (!progress_topic_.empty()) {
    progress_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      progress_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }
  if (!repair_path_topic_.empty()) {
    repair_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      repair_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }
  worker_ = std::make_unique<OrdinaryLocalPathRepairWorker>(node_);

  RCLCPP_INFO(
    logger_,
    "Ordinary local path repair: enabled=%s lookahead=%.2fm persistence=%.2fs "
    "retry=%.2fs min_turning_radius=%.2fm margin=%.2fm",
    enabled_ ? "true" : "false", parameters_.lookahead_distance_m,
    persistence_sec_, retry_period_sec_, parameters_.minimum_turning_radius_m,
    collision_margin_m_);
}

void OrdinaryLocalPathRepairRuntime::activate()
{
  if (progress_pub_ && !progress_pub_->is_activated()) {
    progress_pub_->on_activate();
  }
  if (repair_path_pub_ && !repair_path_pub_->is_activated()) {
    repair_path_pub_->on_activate();
  }
  if (worker_) {
    worker_->start();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_path_ = nav_msgs::msg::Path{};
    control_waiting_ = false;
    ++plan_generation_;
    reset_state_locked();
  }
  publish_progress();
}

void OrdinaryLocalPathRepairRuntime::deactivate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_path_ = nav_msgs::msg::Path{};
    control_waiting_ = false;
    ++plan_generation_;
    reset_state_locked();
  }
  publish_progress();
  if (worker_) {
    worker_->stop();
  }
  if (progress_pub_ && progress_pub_->is_activated()) {
    progress_pub_->on_deactivate();
  }
  if (repair_path_pub_ && repair_path_pub_->is_activated()) {
    repair_path_pub_->on_deactivate();
  }
}

void OrdinaryLocalPathRepairRuntime::set_plan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  reference_path_ = path;
  control_waiting_ = false;
  ++plan_generation_;
  terminal_scope_active_ = false;
  reset_state_locked();
}

std::optional<OrdinaryLocalPathRepairRequest>
OrdinaryLocalPathRepairRuntime::make_request(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path) const
{
  if (!node_ || !tf_ || !costmap_ros_ || costmap_ros_->getCostmap() == nullptr ||
    path.poses.size() < 2U)
  {
    return std::nullopt;
  }

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  if (costmap_frame.empty() || pose.header.frame_id.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped start_pose = pose;
  if (start_pose.header.frame_id != costmap_frame) {
    geometry_msgs::msg::PoseStamped transformed;
    start_pose.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        start_pose, transformed, *tf_, costmap_frame))
    {
      return std::nullopt;
    }
    start_pose = std::move(transformed);
  }

  auto transformed_path = transform_path(path, costmap_frame);
  if (!transformed_path.has_value()) {
    return std::nullopt;
  }

  const auto & goal = transformed_path->poses.back().pose.position;
  if (std::hypot(
      goal.x - start_pose.pose.position.x,
      goal.y - start_pose.pose.position.y) < minimum_goal_distance_m_)
  {
    return std::nullopt;
  }

  auto * live_costmap = costmap_ros_->getCostmap();
  std::shared_ptr<nav2_costmap_2d::Costmap2D> snapshot;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *live_costmap->getMutex());
    snapshot = std::make_shared<nav2_costmap_2d::Costmap2D>(*live_costmap);
  }

  auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3U) {
    return std::nullopt;
  }
  nav2_costmap_2d::padFootprint(footprint, collision_margin_m_);

  OrdinaryLocalPathRepairRequest request;
  request.snapshot_time = std::chrono::steady_clock::now();
  request.costmap_frame = costmap_frame;
  request.costmap = std::move(snapshot);
  request.footprint = std::move(footprint);
  if (preferred_margin_m_ > 0.0) {
    request.preferred_footprint = request.footprint;
    nav2_costmap_2d::padFootprint(request.preferred_footprint, preferred_margin_m_);
  }
  request.start.x = start_pose.pose.position.x;
  request.start.y = start_pose.pose.position.y;
  request.start.yaw = tf2::getYaw(start_pose.pose.orientation);
  request.reference_path = std::move(*transformed_path);
  request.parameters = parameters_;
  return request;
}

std::optional<nav_msgs::msg::Path> OrdinaryLocalPathRepairRuntime::transform_path(
  const nav_msgs::msg::Path & path,
  const std::string & target_frame) const
{
  if (path.poses.empty() || path.header.frame_id.empty() || target_frame.empty()) {
    return std::nullopt;
  }
  if (path.header.frame_id == target_frame) {
    return path;
  }

  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_->lookupTransform(
      target_frame, path.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }

  nav_msgs::msg::Path transformed_path;
  transformed_path.header = path.header;
  transformed_path.header.frame_id = target_frame;
  transformed_path.poses.reserve(path.poses.size());
  for (auto pose : path.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = path.header.frame_id;
    }
    if (pose.header.frame_id != path.header.frame_id) {
      return std::nullopt;
    }
    geometry_msgs::msg::PoseStamped transformed;
    pose.header.stamp = builtin_interfaces::msg::Time{};
    tf2::doTransform(pose, transformed, frame_transform);
    transformed.header.frame_id = target_frame;
    transformed_path.poses.push_back(std::move(transformed));
  }
  return transformed_path;
}

bool OrdinaryLocalPathRepairRuntime::goal_is_in_terminal_scope(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty() || pose.header.frame_id.empty()) {
    return false;
  }
  auto goal = path.poses.back();
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = path.header.frame_id;
  }
  if (goal.header.frame_id.empty()) {
    return false;
  }
  geometry_msgs::msg::PoseStamped goal_in_pose_frame;
  if (goal.header.frame_id == pose.header.frame_id) {
    goal_in_pose_frame = goal;
  } else {
    goal.header.stamp = builtin_interfaces::msg::Time{};
    if (!nav2_util::transformPoseInTargetFrame(
        goal, goal_in_pose_frame, *tf_, pose.header.frame_id))
    {
      return false;
    }
  }
  return std::hypot(
    goal_in_pose_frame.pose.position.x - pose.pose.position.x,
    goal_in_pose_frame.pose.position.y - pose.pose.position.y) <
    minimum_goal_distance_m_;
}

OrdinaryLocalPathRepairRuntime::ApplyResult
OrdinaryLocalPathRepairRuntime::apply_result(
  const geometry_msgs::msg::PoseStamped & pose,
  const OrdinaryLocalPathRepairWorkResult & result)
{
  nav_msgs::msg::Path canonical_reference;
  std::size_t current_generation = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_generation = plan_generation_;
    canonical_reference = reference_path_;
  }
  if (result.plan_generation != current_generation || canonical_reference.poses.empty()) {
    return {};
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - result.request.snapshot_time).count();
  if (age > inspection_period_sec_ + 2.0 * parameters_.max_planning_time_sec) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "Ordinary local repair rejected: snapshot age %.3fs; requesting fresh path", age);
    return {};
  }

  auto latest_request = make_request(pose, canonical_reference);
  if (!latest_request.has_value()) {
    return {};
  }
  const auto original_inspection = inspect_ordinary_local_path(*latest_request, node_);
  if (!original_inspection.valid) {
    return {};
  }
  if (original_inspection.path_clear) {
    RCLCPP_INFO(
      logger_, "Ordinary local path repair cancelled: original path cleared before apply");
    return {ApplyStatus::kOriginalPathCleared, std::nullopt};
  }

  if (result.repair.path.header.frame_id != latest_request->costmap_frame) {
    return {};
  }
  auto candidate_request = *latest_request;
  candidate_request.reference_path = result.repair.path;
  auto attached = attach_ordinary_local_path(candidate_request, node_);
  if (!attached) {
    RCLCPP_WARN(
      logger_, "Ordinary local path repair rejected: latest footprint/forward attachment invalid");
    return {};
  }

  std::string canonical_frame = canonical_reference.header.frame_id;
  if (canonical_frame.empty()) {
    canonical_frame = canonical_reference.poses.back().header.frame_id;
  }
  auto transformed = transform_path(*attached, canonical_frame);
  if (!transformed.has_value() || transformed->poses.size() < 2U) {
    return {};
  }
  transformed->header = canonical_reference.header;
  transformed->header.frame_id = canonical_frame;
  transformed->poses.back() = canonical_reference.poses.back();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (plan_generation_ != current_generation) {
      return {};
    }
    reference_path_ = *transformed;
    ++plan_generation_;
    reset_state_locked();
    last_submit_time_ = std::chrono::steady_clock::now();
    have_last_submit_time_ = true;
  }

  RCLCPP_WARN(
    logger_,
    "Ordinary local path repaired inside active FollowPath: blocked_index=%zu "
    "rejoin_index=%zu poses=%zu iterations=%d lateral_deviation=%.3fm preferred_clearance=%s",
    result.repair.first_blocked_index, result.repair.rejoin_index,
    transformed->poses.size(), result.repair.iterations,
    result.repair.maximum_lateral_deviation_m,
    result.repair.used_preferred_clearance ? "true" : "false");
  if (repair_path_pub_ && repair_path_pub_->is_activated()) {
    repair_path_pub_->publish(*transformed);
  }
  return {ApplyStatus::kReplacementReady, std::move(transformed)};
}

OrdinaryLocalPathRepairUpdate OrdinaryLocalPathRepairRuntime::update(
  const geometry_msgs::msg::PoseStamped & pose)
{
  OrdinaryLocalPathRepairUpdate update_result;
  if (!enabled_ || !worker_ || !node_) {
    return update_result;
  }

  const auto now = std::chrono::steady_clock::now();
  nav_msgs::msg::Path scope_path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scope_path = reference_path_;
  }
  if (goal_is_in_terminal_scope(pose, scope_path)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!terminal_scope_active_) {
        terminal_scope_active_ = true;
        control_waiting_ = false;
        ++plan_generation_;
        reset_state_locked();
      }
    }
    publish_progress();
    return update_result;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    terminal_scope_active_ = false;
  }

  auto completed = worker_->take_result();
  if (completed.has_value()) {
    std::size_t current_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_generation = plan_generation_;
    }
    const char * observed_status = completed->plan_generation != current_generation ?
      "discarded_generation" : ordinary_local_path_repair_status_name(completed->repair.status);
    if (navlite_repair_log_.observe(observed_status,
        completed->repair.status != OrdinaryLocalPathRepairStatus::kPathClear))
    {
      RCLCPP_WARN(logger_,
        "NAVLITE repair status=%s result_generation=%zu current_generation=%zu "
        "snapshot_nearest=%zu snapshot_blocked=%zu snapshot_rejoin=%zu "
        "snapshot_age=%.6f hold_position=%d command_decision=false",
        observed_status, completed->plan_generation, current_generation,
        completed->repair.nearest_index, completed->repair.first_blocked_index,
        completed->repair.rejoin_index,
        std::chrono::duration<double>(now - completed->request.snapshot_time).count(),
        update_result.hold_position);
    }
    if (completed->plan_generation == current_generation) {
      if (completed->error) {std::rethrow_exception(completed->error);}
      auto status = completed->repair.status;
      if (status == OrdinaryLocalPathRepairStatus::kInvalidInput) {
        throw std::runtime_error("Ordinary local path repair received invalid search input");
      }
      if (status == OrdinaryLocalPathRepairStatus::kPathClear) {
        // An old clear snapshot updates neither the plan nor waiting evidence.
        const auto latest = make_request(pose, scope_path);
        if (!latest) {publish_progress(); return update_result;}
        const auto fresh = inspect_ordinary_local_path(*latest, node_);
        if (!fresh.valid) {
          throw std::runtime_error("Ordinary local path inspection received invalid input");
        }
        if (!fresh.path_clear) {status = OrdinaryLocalPathRepairStatus::kNoPath;}
      }
      if (status == OrdinaryLocalPathRepairStatus::kPathClear) {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = Mode::kTracking;
        blockage_observed_ = false;
      } else if (status != OrdinaryLocalPathRepairStatus::kInvalidInput) {
        bool persistent = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!blockage_observed_) {
            blockage_observed_ = true;
            blockage_since_ = now;
            mode_ = Mode::kBlockagePending;
            RCLCPP_WARN(
              logger_,
              "Ordinary local path blockage observed: status=%s blocked_index=%zu; "
              "confirming persistence before replacement",
              ordinary_local_path_repair_status_name(status),
              completed->repair.first_blocked_index);
          } else {
            persistent = std::chrono::duration<double>(now - blockage_since_).count() >=
              persistence_sec_;
            mode_ = persistent ? Mode::kReplanning : Mode::kBlockagePending;
          }
        }

        if (persistent && status == OrdinaryLocalPathRepairStatus::kSuccess) {
          const auto applied = apply_result(pose, *completed);
          if (applied.status == ApplyStatus::kReplacementReady) {
            update_result.replacement_path = applied.replacement_path;
          }
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (applied.status == ApplyStatus::kFailed) {
              mode_ = Mode::kWaitClear;
            } else {
              mode_ = Mode::kTracking;
              blockage_observed_ = false;
            }
          }
        } else if (persistent) {
          std::lock_guard<std::mutex> lock(mutex_);
          mode_ = Mode::kWaitClear;
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 2000,
            "Ordinary reference path has no local rejoin; retrying in background, "
            "MPPI still owns the current command");
        }
      }
    }
  }

  if (update_result.replacement_path.has_value()) {
    publish_progress();
    return update_result;
  }

  Mode mode;
  std::size_t generation = 0U;
  nav_msgs::msg::Path reference_path;
  bool due = false;
  const bool worker_busy = worker_->busy();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode = mode_;
    generation = plan_generation_;
    reference_path = reference_path_;
    const double period = mode == Mode::kTracking ? inspection_period_sec_ :
      (mode == Mode::kBlockagePending ? inspection_period_sec_ : retry_period_sec_);
    due = !reference_path.poses.empty() && !worker_busy &&
      (!have_last_submit_time_ ||
      std::chrono::duration<double>(now - last_submit_time_).count() >= period);
  }

  if (due) {
    auto request = make_request(pose, reference_path);
    if (request.has_value()) {
      OrdinaryLocalPathRepairWork work;
      work.plan_generation = generation;
      work.request = std::move(*request);
      if (worker_->submit(std::move(work))) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (plan_generation_ == generation) {
          last_submit_time_ = now;
          have_last_submit_time_ = true;
          if (mode_ == Mode::kWaitClear) {
            mode_ = Mode::kReplanning;
          }
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A blocked distant reference or busy worker is not a braking decision.
    // Always evaluate current MPPI trajectories and the existing safety chain.
    update_result.hold_position = false;
  }
  publish_progress();
  return update_result;
}

void OrdinaryLocalPathRepairRuntime::reset_state_locked()
{
  mode_ = Mode::kTracking;
  blockage_observed_ = false;
  have_last_submit_time_ = false;
}

geometry_msgs::msg::TwistStamped OrdinaryLocalPathRepairRuntime::compute_command(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::function<geometry_msgs::msg::TwistStamped()> & compute)
{
  geometry_msgs::msg::TwistStamped command;
  bool waiting = false;
  try {
    // Never short-circuit subsequent cycles while waiting. A new feasible
    // command resumes the same FollowPath immediately, even without a replan.
    command = compute();
  } catch (const std::runtime_error & error) {
    // Humble 1.1.19 fallback throws this exact runtime_error after fail_flag
    // exceeds its retry limit. Do not infer a physical collision from the text.
    // controller_server's PlannerException/failure_tolerance does not catch it.
    // Other plugins, TF faults, invalid plans and programming errors propagate.
    if (!enabled_ || !mppi_primary_ ||
      std::string(error.what()) != "Optimizer fail to compute path")
    {
      throw;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (reference_path_.poses.empty()) {throw;}
    }
    command.header = pose.header;
    waiting = true;
    if (navlite_control_log_.observe("exact_optimizer_failure_to_zero", true)) {
      RCLCPP_WARN(logger_,
        "NAVLITE controller event=exception_to_zero "
        "raw=\"Optimizer fail to compute path\" match=exact_runtime_error_text "
        "out=(0.000000,0.000000,0.000000) retained_previous_command=0");
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "Ordinary MPPI has no valid control: zero command, retaining FollowPath and retrying");
  }
  if (!waiting && navlite_control_log_.observe("computed_return", false)) {
    RCLCPP_WARN(logger_,
      "NAVLITE controller event=computed_return layer=outer_rotation_shim "
      "out=(%.9f,%.9f,%.9f) pure_mppi=not_proven downstream_release=unknown",
      command.twist.linear.x, command.twist.linear.y, command.twist.angular.z);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (control_waiting_ && !waiting) {
      RCLCPP_INFO(logger_, "Ordinary MPPI recovered a valid control in the same FollowPath");
    }
    control_waiting_ = waiting;
  }
  publish_progress();
  return command;
}

ElevatorScopedProgressState OrdinaryLocalPathRepairRuntime::progress_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return control_waiting_ ? ElevatorScopedProgressState::kOrdinaryLocalWaitClear :
         ElevatorScopedProgressState::kOrdinary;
}

void OrdinaryLocalPathRepairRuntime::publish_progress()
{
  if (!progress_pub_ || !progress_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::UInt8 message;
  message.data = static_cast<std::uint8_t>(progress_state());
  progress_pub_->publish(message);
}

}  // namespace robot_nav_config
