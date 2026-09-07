#pragma once

#include <cstdint>
#include <string>

namespace robot_docking_manager
{

enum class NearFieldPhase
{
  YawCapture,
  VectorApproach,
  FinalApproach,
  AlignmentBlocked
};

enum class NearFieldMotionMode
{
  Spinning,
  Parallel
};

struct NearFieldObservation
{
  bool valid{false};
  std::uint64_t sequence{0U};
  double distance_m{0.0};
  double lateral_error_m{0.0};
  double yaw_error_rad{0.0};
};

struct NearFieldControlConfig
{
  double target_distance_m{0.34};
  double distance_tolerance_m{0.015};
  double lateral_tolerance_m{0.030};
  double yaw_exit_tolerance_rad{0.008726646259971648};
  double yaw_reentry_threshold_rad{0.05235987755982989};
  int yaw_stable_samples{3};
  int max_yaw_realignments{1};

  double k_forward{0.45};
  double k_lateral{0.70};
  double k_yaw{0.70};
  double lateral_command_sign{1.0};
  double lateral_deadband_m{0.005};
  double yaw_deadband_rad{0.004363323129985824};

  double min_forward_speed_mps{0.025};
  double max_forward_speed_mps{0.15};
  double min_lateral_speed_mps{0.025};
  double max_lateral_speed_mps{0.04};
  double min_yaw_speed_radps{0.05};
  double max_yaw_speed_radps{0.12};
  double max_parallel_speed_mps{0.15};
  double final_approach_window_m{0.10};
  double final_lateral_lock_distance_m{0.06};
  double final_forward_speed_mps{0.05};

  double contact_cruise_speed_mps{0.05};
  double contact_final_zone_m{0.06};
  double contact_final_speed_mps{0.02};
  double contact_timeout_safety_factor{1.5};
  double contact_timeout_margin_sec{2.0};
  double contact_timeout_min_sec{3.0};

  double retry_backoff_min_distance_m{0.20};
  double retry_backoff_clearance_margin_m{0.08};
  double retry_backoff_max_distance_m{0.60};
};

struct NearFieldDecision
{
  NearFieldPhase phase{NearFieldPhase::YawCapture};
  NearFieldMotionMode mode{NearFieldMotionMode::Spinning};
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_radps{0.0};
  bool enter_contact_verify{false};
  bool alignment_blocked{false};
  int yaw_realignments{0};
  int yaw_stable_samples{0};
  std::string reason;
};

class NearFieldDockingController
{
public:
  explicit NearFieldDockingController(NearFieldControlConfig config);

  void reset();
  NearFieldDecision step(const NearFieldObservation & observation);

  double contact_timeout_sec(double contact_distance_m) const;
  double retry_backoff_distance_m(double attempted_contact_distance_m) const;

private:
  NearFieldControlConfig config_;
  NearFieldPhase phase_{NearFieldPhase::YawCapture};
  NearFieldPhase resume_phase_{NearFieldPhase::VectorApproach};
  int yaw_stable_samples_{0};
  int yaw_realignments_{0};
  std::uint64_t last_yaw_sequence_{0U};
  bool have_last_yaw_sequence_{false};
};

}  // namespace robot_docking_manager
