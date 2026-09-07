#pragma once

#include <string>

#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"

namespace robot_api_server::features::docking::predock_alignment
{

struct PredockAlignmentConfig
{
  double pose_max_distance_m{0.30};
  double handoff_max_yaw_rad{0.35};
  double yaw_tolerance_rad{0.0698};
  double yaw_hard_fail_rad{0.80};
  double lateral_target_m{0.03};
  double lateral_max_correction_m{0.25};
  double lateral_yaw_slack_rad{0.02};
  double forward_capture_min_m{-0.40};
  double forward_capture_max_m{0.55};
};

struct PredockPoseVerification
{
  RobotPoseSnapshot pose;
  bool pose_available{false};
  bool xy_ok{false};
  bool base_yaw_ok{false};
  bool contact_yaw_ok{false};
  bool contact_frame_available{false};
  double distance_m{-1.0};
  double expected_base_yaw{0.0};
  double expected_contact_yaw{0.0};
  double current_base_yaw{0.0};
  double current_contact_yaw{0.0};
  double base_yaw_error_rad{0.0};
  double contact_yaw_error_rad{0.0};
  double forward_m{0.0};
  double lateral_m{0.0};
  double lateral_abs_m{0.0};
  std::string detail;
};

struct PredockYawAlignResult
{
  bool attempted{false};
  bool succeeded{false};
  bool canceled{false};
  bool blocked{false};
  bool actual_spin_required{false};
  std::string failure_code{"PREDOCK_YAW_ALIGN_TIMEOUT"};
  std::string detail;
  double duration_sec{-1.0};
  double initial_error_rad{-1.0};
  double final_error_rad{-1.0};
  double observed_yaw_motion_rad{0.0};
};

struct PredockLateralAlignResult
{
  bool attempted{false};
  bool succeeded{false};
  bool canceled{false};
  bool blocked{false};
  std::string failure_code{"PREDOCK_LATERAL_ALIGN_TIMEOUT"};
  std::string detail;
  double duration_sec{-1.0};
  double initial_error_m{0.0};
  double final_error_m{0.0};
  double observed_lateral_motion_m{0.0};
};

class PredockAlignmentPolicy
{
public:
  explicit PredockAlignmentPolicy(PredockAlignmentConfig config);

  double normalize_yaw_error(double yaw_error) const;
  double expected_staging_yaw(const DockingJob & job) const;
  double predock_yaw_error(double current_base_yaw, double expected_base_yaw) const;
  double contact_yaw_error(double current_contact_yaw, double expected_contact_yaw) const;

  PredockPoseVerification evaluate_pose(
    const DockingJob & job,
    const RobotPoseSnapshot & pose,
    const std::string & pose_error) const;

  bool pose_inside_handoff_window(const PredockPoseVerification & check) const;
  bool pose_inside_xy_handoff_window(const PredockPoseVerification & check) const;
  bool pose_allows_staging_recovery(const PredockPoseVerification & check) const;
  bool yaw_target_met(const PredockPoseVerification & check) const;
  bool yaw_angles_met(const PredockPoseVerification & check) const;
  bool lateral_target_met(const PredockPoseVerification & check) const;
  bool lateral_error_target_met(const PredockPoseVerification & check) const;
  bool staging_target_met(const PredockPoseVerification & check) const;
  bool forward_capture_window_met(const PredockPoseVerification & check) const;
  bool lateral_capture_allowed(const PredockPoseVerification & check) const;
  double lateral_align_yaw_gate_rad() const;
  bool lateral_align_yaw_gate_met(const PredockPoseVerification & check) const;

  void apply_pose_verification(
    DockingJob & job,
    const PredockPoseVerification & check) const;
  void apply_yaw_align_result(DockingJob & job, const PredockYawAlignResult & result) const;
  void apply_lateral_align_result(
    DockingJob & job,
    const PredockLateralAlignResult & result) const;
  void apply_fine_entry_check(
    DockingJob & job,
    const PredockPoseVerification & check,
    bool ok,
    const std::string & failure_code,
    const std::string & detail) const;

private:
  PredockAlignmentConfig config_;
};

}  // namespace robot_api_server::features::docking::predock_alignment
