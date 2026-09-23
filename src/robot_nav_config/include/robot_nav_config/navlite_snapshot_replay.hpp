#pragma once
#include "robot_nav_config/navlite_snapshot.hpp"
#include "robot_nav_config/chassis_dynamics/motion_model.hpp"
#include "nav2_mppi_controller/optimizer.hpp"

namespace robot_nav_config::navlite_snapshot
{
struct ReplayResult {mppi::models::State state; mppi::models::Trajectories trajectory;};
// Reuses the installed Humble integration routine, without initialize(), ROS
// entities, parameter reads, noise generation, candidate scoring or control calls.
class ReplayIntegrator : public mppi::Optimizer
{
public:
  mppi::models::Trajectories integrate(const mppi::models::State & state, float dt)
  {
    settings_.model_dt = dt;
    // Integration only asks whether the model is holonomic. Ranger is Ackermann.
    motion_model_ = std::make_shared<mppi::DiffDriveMotionModel>();
    mppi::models::Trajectories result; result.reset(1, state.vx.shape()[1]);
    integrateStateVelocities(result, state); return result;
  }
};

inline ReplayResult replay(const Frame & f)
{
  if (f.incomplete || !f.model_enabled || !f.sequence_valid || !f.command_returned ||
    f.sequence_count < 2 || f.sequence_count > kSteps || f.sequence_count != f.sequence_original_count ||
    f.model_dt <= 0 || f.model_dt > 0.1 || !std::isfinite(f.model_dt) ||
    f.min_turning_radius <= 0 || !std::isfinite(f.min_turning_radius))
  {throw std::invalid_argument("snapshot incomplete or unsupported model/sequence");}
  chassis_dynamics::Parameters p;
  p.linear_delay=f.dynamics[0]; p.linear_tau=f.dynamics[1]; p.acceleration=f.dynamics[2];
  p.deceleration=f.dynamics[3]; p.steering_delay=f.dynamics[4]; p.steering_tau=f.dynamics[5];
  p.wheelbase=f.dynamics[6]; p.track=f.dynamics[7]; p.validate();
  ReplayResult result; auto & state=result.state; state.reset(1, f.sequence_count);
  auto & pose=state.pose.pose;
  pose.position.x=f.pose[0]; pose.position.y=f.pose[1]; pose.position.z=f.pose[2];
  pose.orientation.x=f.pose[3]; pose.orientation.y=f.pose[4]; pose.orientation.z=f.pose[5]; pose.orientation.w=f.pose[6];
  state.speed.linear.x=f.odom[0]; state.speed.linear.y=f.odom[1]; state.speed.linear.z=f.odom[2];
  state.speed.angular.x=f.odom[3]; state.speed.angular.y=f.odom[4]; state.speed.angular.z=f.odom[5];
  // Native updateInitialStateVelocities stores measured speed in float tensors.
  state.vx(0,0)=state.speed.linear.x; state.wz(0,0)=state.speed.angular.z;
  for (unsigned t=0;t<f.sequence_count;++t) {
    state.cvx(0,t)=f.sequence[t][0]; state.cvy(0,t)=f.sequence[t][1]; state.cwz(0,t)=f.sequence[t][2];
  }
  // Exact single-row adaptation of RangerMotionModel::predict. The plant and
  // smoother functions below ARE production implementations, not fitted again.
  chassis_dynamics::ResponseModel model(p);
  const double v0=state.vx(0,0), w0=state.wz(0,0);
  model.reset(v0, model.steering_for(v0,w0), v0,w0);
  for (unsigned i=0;i<f.history_count;++i) {model.remember({f.issued[i][0],f.issued[i][1],f.issued[i][2]});}
  double sv=f.smoother_valid?f.smoother_linear:v0;
  double sw=f.smoother_valid?f.smoother_angular:w0;
  for (unsigned t=1;t<f.sequence_count;++t) {
    const double target_v=std::clamp(static_cast<double>(state.cvx(0,t-1)),f.constraints[0],f.constraints[1]);
    const double bound=std::min(f.constraints[3],std::abs(target_v)/f.min_turning_radius);
    const double target_w=std::clamp(static_cast<double>(state.cwz(0,t-1)),-bound,bound);
    sv=chassis_dynamics::RangerMotionModel::smooth(sv,target_v,f.smoother_limits[0],f.smoother_limits[1],f.model_dt);
    sw=chassis_dynamics::RangerMotionModel::smooth(sw,target_w,f.smoother_limits[2],f.smoother_limits[3],f.model_dt);
    const auto prediction=model.advance(sv,sw,f.model_dt);
    state.vx(0,t)=prediction.linear; state.wz(0,t)=prediction.angular;
  }
  ReplayIntegrator native; result.trajectory=native.integrate(state, static_cast<float>(f.model_dt));
  return result;
}
}
