// Controller lifecycle/path/locking flow follows Nav2 Humble's controller.cpp.
// Copyright (c) 2022 Samsung Research America, Alexey Budyakov.
// Licensed under the Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0).
// Project additions: measured Ranger prediction and constrained output finalization.
// Upstream sampling, scoring and fallback implementation are linked unchanged.
#include "robot_nav_config/chassis_dynamics/mppi_controller.hpp"
#include "robot_nav_config/chassis_dynamics/motion_model.hpp"
#include "robot_nav_config/chassis_dynamics/output_filter.hpp"
#include "robot_nav_config/navlite_failure_trace.hpp"
#include "robot_nav_config/navlite_snapshot.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <deque>
#include <mutex>
#include <typeinfo>

namespace robot_nav_config
{
namespace
{
using Clock = std::chrono::steady_clock;
using Snapshot = navlite_snapshot::Frame;
int64_t source_ns(const builtin_interfaces::msg::Time & stamp) noexcept
{return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;}
std::array<double, 7> pose_values(const geometry_msgs::msg::Pose & p) noexcept
{return {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};}
std::array<double, 6> twist_values(const geometry_msgs::msg::Twist & v) noexcept
{return {v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z};}

class RangerOptimizer : public mppi::Optimizer
{
public:
  // Observations of the existing calls, not an additional retry policy.
  const char * trace_stage{"not_entered"};
  unsigned int trace_passes{0}, trace_passes_entered_failed{0};
  int trace_fail_flag{-1};
  void begin_trace()
  {
    trace_stage = "not_entered";
    trace_passes = trace_passes_entered_failed = 0;
    trace_fail_flag = -1;
  }
  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * checker, Snapshot * snapshot = nullptr)
  {
    // Humble evalControl flow, with sequence finalization kept before selection
    // and horizon shift. Sampling, scoring and no-control fallback stay native.
    trace_stage = "prepare";
    prepare(pose, speed, plan, checker);
    if (snapshot) {
      snapshot->model_dt = settings_.model_dt;
      const auto & c = settings_.constraints;
      snapshot->constraints = {c.vx_min, c.vx_max, c.vy, c.wz};
      snapshot->command_offset = settings_.shift_control_sequence ? 1 : 0;
      snapshot->min_turning_radius = model_ ? model_->getMinTurningRadius() : 0.0;
    }
    bool retry;
    do {
      ++trace_passes;
      trace_fail_flag = critics_data_.fail_flag;
      if (critics_data_.fail_flag) {++trace_passes_entered_failed;}
      trace_stage = "optimize";
      optimize();
      trace_fail_flag = critics_data_.fail_flag;
      trace_stage = "fallback";
      retry = fallback(critics_data_.fail_flag);
    } while (retry);
    trace_stage = "output_filter";
    chassis_dynamics::filter_control_sequence(
      control_sequence_, control_history_, settings_, *motion_model_);
    trace_stage = "command_selection";
    auto command = getControlFromSequenceAsTwist(plan.header.stamp);
    if (snapshot) {
      snapshot->sequence_original_count = control_sequence_.vx.size();
      snapshot->sequence_count = std::min<std::size_t>(control_sequence_.vx.size(), navlite_snapshot::kSteps);
      snapshot->incomplete |= snapshot->sequence_count != snapshot->sequence_original_count;
      for (unsigned t = 0; t < snapshot->sequence_count; ++t) {
        snapshot->sequence[t] = {control_sequence_.vx(t), control_sequence_.vy(t), control_sequence_.wz(t)};
      }
      snapshot->sequence_valid = true;
    }
    trace_stage = "sequence_shift";
    if (settings_.shift_control_sequence) {shiftControlSequence();}
    return command;
  }

  void install_model(const chassis_dynamics::Parameters & parameters,
    const std::vector<double> & accel, const std::vector<double> & decel)
  {
    // Humble's parameter handler retains references into its original model.
    // Keep that object alive until the handler has been destroyed.
    original_model_ = motion_model_;
    model_ = std::make_shared<chassis_dynamics::RangerMotionModel>(parameters_handler_, name_,
      parameters, settings_.model_dt, accel[0], -decel[0], accel[2], -decel[2]);
    motion_model_ = model_;
  }
  void input(chassis_dynamics::PredictionInput data)
  {
    if (model_) {
      model_->set_input(std::move(data), settings_.model_dt, settings_.constraints.vx_min,
        settings_.constraints.vx_max, settings_.constraints.wz);
    }
  }
  xt::xtensor<float, 2> predicted_optimum()
  {
    // Native getOptimizedTrajectory integrates command velocities directly.
    // Visualize the same response prediction used for collision scoring instead.
    mppi::models::State candidate;
    candidate.reset(1, settings_.time_steps);
    candidate.pose = state_.pose;
    candidate.speed = state_.speed;
    xt::view(candidate.cvx, 0, xt::all()) = control_sequence_.vx;
    xt::view(candidate.cwz, 0, xt::all()) = control_sequence_.wz;
    updateStateVelocities(candidate);
    mppi::models::Trajectories trajectory;
    trajectory.reset(1, settings_.time_steps);
    integrateStateVelocities(trajectory, candidate);
    xt::xtensor<float, 2> result = xt::zeros<float>({settings_.time_steps, 3u});
    xt::view(result, xt::all(), 0) = xt::view(trajectory.x, 0, xt::all());
    xt::view(result, xt::all(), 1) = xt::view(trajectory.y, 0, xt::all());
    xt::view(result, xt::all(), 2) = xt::view(trajectory.yaws, 0, xt::all());
    return result;
  }
private:
  std::shared_ptr<mppi::MotionModel> original_model_;
  std::shared_ptr<chassis_dynamics::RangerMotionModel> model_;
};
}

struct RangerMPPIController::Impl
{
  struct Received {Clock::time_point time; geometry_msgs::msg::Twist command;};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  rclcpp::Clock::SharedPtr clock;
  std::unique_ptr<mppi::ParametersHandler> handler;
  RangerOptimizer optimizer;
  mppi::PathHandler path;
  mppi::TrajectoryVisualizer visualizer;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr final_sub, smoother_sub;
  std::mutex feedback_mutex;
  std::deque<Received> issued;
  std::unique_ptr<navlite_snapshot::Recorder> snapshots;
  std::array<double, 8> diagnostic_dynamics{};
  std::array<double, 4> diagnostic_smoother_limits{};
  Received smoothed{};
  bool has_smoothed{false}, visualize{false}, enabled{true};
  double reset_period{1.0};
  rclcpp::Time last_call{0, 0, RCL_ROS_TIME};
  rclcpp::Logger logger{rclcpp::get_logger("RangerMPPIController")};
  NavliteFailureTrace failure_trace;
  std::uint64_t compute_sequence{0};

  void log_failure(const char * stage, const char * type, const std::string & raw)
  {
    const std::string reason = std::string(stage) + ":" + type + ":" + raw;
    if (const char * event = failure_trace.failure(reason)) {
      RCLCPP_WARN(logger,
        "NAVLITE mppi event=%s episode=%llu compute_seq=%llu consecutive_failures=%llu "
        "elapsed_sec=%.6f start_steady_ns=%lld stage=%s exception_type=%s raw=%s "
        "fail_flag=%d optimize_passes=%u passes_entered_failed=%u command_returned=0",
        event, static_cast<unsigned long long>(failure_trace.episode()),
        static_cast<unsigned long long>(compute_sequence),
        static_cast<unsigned long long>(failure_trace.failures()), failure_trace.elapsed(),
        static_cast<long long>(failure_trace.started_ns()), stage, type, navlite_quote(raw).c_str(),
        optimizer.trace_fail_flag, optimizer.trace_passes, optimizer.trace_passes_entered_failed);
    }
  }
  void interrupt_trace(const char * reason)
  {
    if (failure_trace.pending()) {
      RCLCPP_WARN(logger,
        "NAVLITE mppi event=episode_interrupted reason=%s episode=%llu "
        "consecutive_failures=%llu recovery_proven=0",
        reason, static_cast<unsigned long long>(failure_trace.episode()),
        static_cast<unsigned long long>(failure_trace.failures()));
    }
    failure_trace.interrupt();
  }

  chassis_dynamics::PredictionInput feedback(Snapshot * snapshot = nullptr)
  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(feedback_mutex);
    chassis_dynamics::PredictionInput input;
    if (snapshot) {
      snapshot->model_input_monotonic_ns = navlite_snapshot::Recorder::monotonic_ns();
      snapshot->smoother_age_sec = has_smoothed ? std::chrono::duration<double>(now - smoothed.time).count() : -1.0;
      snapshot->issued_age_sec = issued.empty() ? -1.0 : std::chrono::duration<double>(now - issued.back().time).count();
    }
    // Missing/stale telemetry changes initialization only, never adds a stop gate.
    if (!issued.empty() && std::chrono::duration<double>(now - issued.back().time).count() < 0.25) {
      for (const auto & record : issued) {
        input.issued.push_back({std::chrono::duration<double>(record.time - now).count(),
          record.command.linear.x, record.command.angular.z});
      }
    }
    if (has_smoothed && std::chrono::duration<double>(now - smoothed.time).count() < 0.25) {
      input.smoother_valid = true;
      input.smoother_linear = smoothed.command.linear.x;
      input.smoother_angular = smoothed.command.angular.z;
    }
    if (snapshot) {
      snapshot->smoother_valid = input.smoother_valid;
      snapshot->smoother_linear = input.smoother_linear;
      snapshot->smoother_angular = input.smoother_angular;
      snapshot->history_count = std::min(input.issued.size(), navlite_snapshot::kHistory);
      snapshot->incomplete |= snapshot->history_count != input.issued.size();
      for (unsigned n = 0; n < snapshot->history_count; ++n) {
        const auto & c = input.issued[n]; snapshot->issued[n] = {c.time, c.linear, c.angular};
      }
    }
    return input;
  }
};

RangerMPPIController::RangerMPPIController() : impl_(std::make_unique<Impl>()) {}
RangerMPPIController::~RangerMPPIController() = default;

void RangerMPPIController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto & i = *impl_;
  auto node = parent.lock();
  i.logger = node->get_logger();
  i.costmap = costmap;
  i.clock = node->get_clock();
  i.last_call = i.clock->now();
  i.handler = std::make_unique<mppi::ParametersHandler>(parent);
  auto get = i.handler->getParamGetter(name);
  get(i.visualize, "visualize", false);
  get(i.reset_period, "reset_period", 1.0);
  get(i.enabled, "RangerDynamics.enabled", true, mppi::ParameterType::Static);
  chassis_dynamics::Parameters p;
  const auto read = [&](double & value, const std::string & key) {
      get(value, "RangerDynamics." + key, value, mppi::ParameterType::Static);
    };
  read(p.linear_delay, "linear_delay_sec");
  read(p.linear_tau, "linear_tau_sec");
  read(p.acceleration, "acceleration_mps2");
  read(p.deceleration, "deceleration_mps2");
  read(p.steering_delay, "steering_delay_sec");
  read(p.steering_tau, "steering_tau_sec");
  read(p.wheelbase, "wheelbase_m");
  read(p.track, "track_m");
  std::vector<double> accel{0.55, 0.20, 0.90}, decel{-0.95, -0.30, -1.10};
  get(accel, "RangerDynamics.smoother_max_accel", accel, mppi::ParameterType::Static);
  get(decel, "RangerDynamics.smoother_max_decel", decel, mppi::ParameterType::Static);
  std::string model;
  get(model, "motion_model", std::string("Ackermann"), mppi::ParameterType::Static);
  if (i.enabled && (model != "Ackermann" || accel.size() != 3 || decel.size() != 3)) {
    throw std::invalid_argument("Ranger dynamics requires Ackermann and three-axis smoother limits");
  }
  for (std::size_t axis : {0u, 2u}) {
    if (accel.size() <= axis || decel.size() <= axis || !std::isfinite(accel[axis]) ||
      !std::isfinite(decel[axis]) || accel[axis] <= 0 || decel[axis] >= 0)
    {throw std::invalid_argument("Invalid Ranger prediction smoother limits");}
  }
  i.optimizer.initialize(parent, name, costmap, i.handler.get());
  i.diagnostic_dynamics = {p.linear_delay, p.linear_tau, p.acceleration, p.deceleration,
    p.steering_delay, p.steering_tau, p.wheelbase, p.track};
  i.diagnostic_smoother_limits = {accel[0], -decel[0], accel[2], -decel[2]};
  try {i.snapshots = std::make_unique<navlite_snapshot::Recorder>();}
  catch (const std::exception & error) {
    RCLCPP_WARN(i.logger, "NAVLITE mppi event=snapshot_unavailable reason=%s", error.what());
  } catch (...) {RCLCPP_WARN(i.logger, "NAVLITE mppi event=snapshot_unavailable reason=non_std_exception");}
  if (i.enabled) {
    i.optimizer.install_model(p, accel, decel);
    i.final_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {return;}
        auto & i = *impl_;
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lock(i.feedback_mutex);
        i.issued.push_back({now, *message});
        while (i.issued.size() > 100 || (!i.issued.empty() &&
          std::chrono::duration<double>(now - i.issued.front().time).count() > 0.7))
        {i.issued.pop_front();}
      });
    i.smoother_sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {return;}
        std::lock_guard<std::mutex> lock(impl_->feedback_mutex);
        impl_->smoothed = {Clock::now(), *message};
        impl_->has_smoothed = true;
      });
  }
  i.path.initialize(parent, name, costmap, tf, i.handler.get());
  i.visualizer.on_configure(parent, name, costmap->getGlobalFrameID(), i.handler.get());
  RCLCPP_WARN(node->get_logger(),
    "NAVLITE mppi event=diagnostics_ready schema=1 plugin=RangerMPPIController "
    "normal_path=transitions_only failure_repeat_max_hz=1");
  RCLCPP_INFO(node->get_logger(),
    "Ranger MPPI dynamics enabled=%s output_constraints=post_filter "
    "calibration=CAN_20260630_v1 linear_delay=%.3f tau=%.5f "
    "accel=%.5f decel=%.5f steering_delay=%.3f steering_tau=%.5f",
    i.enabled ? "true" : "false", p.linear_delay, p.linear_tau, p.acceleration,
    p.deceleration, p.steering_delay, p.steering_tau);
}

void RangerMPPIController::cleanup()
{
  impl_->interrupt_trace("cleanup");
  impl_->final_sub.reset();
  impl_->smoother_sub.reset();
  impl_->optimizer.shutdown();
  impl_->visualizer.on_cleanup();
  impl_->handler.reset();
  std::lock_guard<std::mutex> lock(impl_->feedback_mutex);
  impl_->issued.clear();
  impl_->has_smoothed = false;
}
void RangerMPPIController::activate()
{
  impl_->visualizer.on_activate();
  impl_->handler->start();
}
void RangerMPPIController::deactivate()
{
  impl_->interrupt_trace("deactivate");
  impl_->visualizer.on_deactivate();
}

geometry_msgs::msg::TwistStamped RangerMPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * checker)
{
  auto & i = *impl_;
  ++i.compute_sequence;
  Snapshot * snapshot = i.snapshots ? i.snapshots->begin(i.compute_sequence) : nullptr;
  if (snapshot) {
    snapshot->pose = pose_values(pose.pose); snapshot->odom = twist_values(speed);
    snapshot->pose_stamp_ns = source_ns(pose.header.stamp);
    snapshot->incomplete |= navlite_snapshot::text(snapshot->pose_frame, pose.header.frame_id.c_str());
    snapshot->model_enabled = i.enabled;
    snapshot->dynamics = i.diagnostic_dynamics; snapshot->smoother_limits = i.diagnostic_smoother_limits;
  }
  const auto finish_snapshot = [&](const char * stage, const char * type, const char * reason) noexcept {
      if (!snapshot) {return;}
      snapshot->incomplete |= navlite_snapshot::text(snapshot->stage, stage);
      snapshot->incomplete |= navlite_snapshot::text(snapshot->exception_type, type);
      snapshot->incomplete |= navlite_snapshot::text(snapshot->exception, reason);
      snapshot->fail_flag = i.optimizer.trace_fail_flag;
      snapshot->optimize_passes = i.optimizer.trace_passes;
      snapshot->passes_entered_failed = i.optimizer.trace_passes_entered_failed;
      i.snapshots->finish(snapshot); snapshot = nullptr;
    };
  i.optimizer.begin_trace();
  const char * stage = "idle_reset";
  try {
  if (i.clock->now() - i.last_call > rclcpp::Duration::from_seconds(i.reset_period)) {
    i.interrupt_trace("existing_idle_reset");
    i.optimizer.reset();
  }
  i.last_call = i.clock->now();
  stage = "parameters_lock";
  std::lock_guard<std::mutex> parameters_lock(*i.handler->getLock());
  stage = "transformPath";
  auto plan = i.path.transformPath(pose);
  stage = "costmap_lock";
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*i.costmap->getCostmap()->getMutex());
  if (snapshot) {
    // No serialization or rollout here. Reuse the exact lock held by native critics.
    try {
      auto * map = i.costmap->getCostmap();
      snapshot->map_copy_monotonic_ns = navlite_snapshot::Recorder::monotonic_ns();
      snapshot->map_width = map->getSizeInCellsX(); snapshot->map_height = map->getSizeInCellsY();
      const auto cells = std::size_t(snapshot->map_width) * snapshot->map_height;
      snapshot->map_count = cells <= navlite_snapshot::kMapCells ? cells : 0;
      snapshot->incomplete |= snapshot->map_count != cells;
      if (snapshot->map_count) {std::memcpy(snapshot->map.data(), map->getCharMap(), snapshot->map_count);}
      snapshot->resolution = map->getResolution();
      snapshot->origin_x = map->getOriginX(); snapshot->origin_y = map->getOriginY();
      snapshot->incomplete |= navlite_snapshot::text(snapshot->map_frame, i.costmap->getGlobalFrameID().c_str());
      snapshot->incomplete |= navlite_snapshot::text(snapshot->base_frame, i.costmap->getBaseFrameID().c_str());
      snapshot->incomplete |= navlite_snapshot::text(snapshot->path_frame, plan.header.frame_id.c_str());
      snapshot->path_stamp_ns = source_ns(plan.header.stamp);
      snapshot->path_original_count = plan.poses.size();
      snapshot->path_count = std::min(plan.poses.size(), navlite_snapshot::kPathPoses);
      snapshot->incomplete |= snapshot->path_count != plan.poses.size();
      for (unsigned n = 0; n < snapshot->path_count; ++n) {
        snapshot->path[n] = pose_values(plan.poses[n].pose);
        snapshot->path_stamps[n] = source_ns(plan.poses[n].header.stamp);
        const auto & frame = plan.poses[n].header.frame_id;
        snapshot->incomplete |= frame.size() >= snapshot->path_frames[n].size();
        const auto length = std::min(frame.size(), snapshot->path_frames[n].size() - 1);
        std::memcpy(snapshot->path_frames[n].data(), frame.data(), length);
        snapshot->path_frames[n][length] = 0;
      }
      const auto footprint = i.costmap->getRobotFootprint();
      snapshot->footprint_count = std::min(footprint.size(), navlite_snapshot::kFootprint);
      snapshot->incomplete |= snapshot->footprint_count != footprint.size();
      for (unsigned n = 0; n < snapshot->footprint_count; ++n) {
        const auto & p = footprint[n]; snapshot->footprint[n] = {p.x, p.y, p.z};
      }
    } catch (...) {snapshot->incomplete = true;} // Diagnostic capture cannot change a control result.
  }
  stage = "feedback_input";
  i.optimizer.input(i.feedback(snapshot));
  stage = "optimizer";
  auto command = i.optimizer.evalControl(pose, speed, plan, checker, snapshot);
  if (snapshot) {snapshot->command = twist_values(command.twist);}
  stage = "visualize";
  if (i.visualize) {
    i.visualizer.add(i.optimizer.getGeneratedTrajectories(), "Candidate Trajectories");
    i.visualizer.add(i.optimizer.predicted_optimum(), "Response-predicted optimum");
    i.visualizer.visualize(std::move(plan));
  }
  const auto & v = command.twist;
  if (const char * event = i.failure_trace.success(
      v.linear.x != 0.0 || v.linear.y != 0.0 || v.angular.z != 0.0))
  {
    RCLCPP_WARN(i.logger,
      "NAVLITE mppi event=%s episode=%llu compute_seq=%llu consecutive_failures=%llu "
      "elapsed_sec=%.6f start_steady_ns=%lld command_returned=1 "
      "out=(%.9f,%.9f,%.9f) downstream_release=unknown",
      event, static_cast<unsigned long long>(i.failure_trace.episode()),
      static_cast<unsigned long long>(i.compute_sequence),
      static_cast<unsigned long long>(i.failure_trace.failures()), i.failure_trace.elapsed(),
      static_cast<long long>(i.failure_trace.started_ns()), v.linear.x, v.linear.y, v.angular.z);
  }
  if (snapshot) {snapshot->command_returned = true;}
  finish_snapshot("return", "", "");
  return command;
  } catch (const std::exception & error) {
    finish_snapshot(std::strcmp(stage, "optimizer") == 0 ? i.optimizer.trace_stage : stage,
      typeid(error).name(), error.what());
    i.log_failure(std::string(stage) == "optimizer" ? i.optimizer.trace_stage : stage,
      typeid(error).name(), error.what());
    throw;
  } catch (...) {
    finish_snapshot(stage, "non_std_exception", "unavailable");
    i.log_failure(stage, "non_std_exception", "unavailable");
    throw;
  }
}
void RangerMPPIController::setPlan(const nav_msgs::msg::Path & path) {impl_->path.setPath(path);}
void RangerMPPIController::setSpeedLimit(const double & limit, const bool & percentage)
{impl_->optimizer.setSpeedLimit(limit, percentage);}
}

PLUGINLIB_EXPORT_CLASS(robot_nav_config::RangerMPPIController, nav2_core::Controller)
