// Controller lifecycle/path/locking flow follows Nav2 Humble's controller.cpp.
// Copyright (c) 2022 Samsung Research America, Alexey Budyakov.
// Licensed under the Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0).
// Project additions: measured Ranger prediction and constrained output finalization.
// Upstream sampling, scoring and fallback implementation are linked unchanged.
#include "robot_nav_config/chassis_dynamics/mppi_controller.hpp"
#include "robot_nav_config/chassis_dynamics/motion_model.hpp"
#include "robot_nav_config/chassis_dynamics/output_filter.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <deque>
#include <mutex>

namespace robot_nav_config
{
namespace
{
using Clock = std::chrono::steady_clock;
class RangerOptimizer : public mppi::Optimizer
{
public:
  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * checker)
  {
    // Humble evalControl flow, with sequence finalization kept before selection
    // and horizon shift. Sampling, scoring and no-control fallback stay native.
    prepare(pose, speed, plan, checker);
    do {optimize();} while (fallback(critics_data_.fail_flag));
    chassis_dynamics::filter_control_sequence(
      control_sequence_, control_history_, settings_, *motion_model_);
    auto command = getControlFromSequenceAsTwist(plan.header.stamp);
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
  Received smoothed{};
  bool has_smoothed{false}, visualize{false}, enabled{true};
  double reset_period{1.0};
  rclcpp::Time last_call{0, 0, RCL_ROS_TIME};

  chassis_dynamics::PredictionInput feedback()
  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(feedback_mutex);
    chassis_dynamics::PredictionInput input;
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
  RCLCPP_INFO(node->get_logger(),
    "Ranger MPPI dynamics enabled=%s output_constraints=post_filter "
    "calibration=CAN_20260630_v1 linear_delay=%.3f tau=%.5f "
    "accel=%.5f decel=%.5f steering_delay=%.3f steering_tau=%.5f",
    i.enabled ? "true" : "false", p.linear_delay, p.linear_tau, p.acceleration,
    p.deceleration, p.steering_delay, p.steering_tau);
}

void RangerMPPIController::cleanup()
{
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
void RangerMPPIController::deactivate() {impl_->visualizer.on_deactivate();}

geometry_msgs::msg::TwistStamped RangerMPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * checker)
{
  auto & i = *impl_;
  if (i.clock->now() - i.last_call > rclcpp::Duration::from_seconds(i.reset_period)) {i.optimizer.reset();}
  i.last_call = i.clock->now();
  std::lock_guard<std::mutex> parameters_lock(*i.handler->getLock());
  auto plan = i.path.transformPath(pose);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*i.costmap->getCostmap()->getMutex());
  i.optimizer.input(i.feedback());
  auto command = i.optimizer.evalControl(pose, speed, plan, checker);
  if (i.visualize) {
    i.visualizer.add(i.optimizer.getGeneratedTrajectories(), "Candidate Trajectories");
    i.visualizer.add(i.optimizer.predicted_optimum(), "Response-predicted optimum");
    i.visualizer.visualize(std::move(plan));
  }
  return command;
}
void RangerMPPIController::setPlan(const nav_msgs::msg::Path & path) {impl_->path.setPath(path);}
void RangerMPPIController::setSpeedLimit(const double & limit, const bool & percentage)
{impl_->optimizer.setSpeedLimit(limit, percentage);}
}

PLUGINLIB_EXPORT_CLASS(robot_nav_config::RangerMPPIController, nav2_core::Controller)
