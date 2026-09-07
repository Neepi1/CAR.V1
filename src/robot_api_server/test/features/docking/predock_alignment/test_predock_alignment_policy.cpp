#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "robot_api_server/features/docking/predock_alignment/predock_alignment_policy.hpp"

namespace predock = robot_api_server::features::docking::predock_alignment;

namespace
{

constexpr double kPi = 3.14159265358979323846;

predock::PredockAlignmentConfig test_config()
{
  predock::PredockAlignmentConfig config;
  config.pose_max_distance_m = 0.25;
  config.handoff_max_yaw_rad = 0.10;
  config.yaw_tolerance_rad = 0.05;
  config.yaw_hard_fail_rad = 0.50;
  config.lateral_target_m = 0.03;
  config.lateral_max_correction_m = 0.25;
  config.lateral_yaw_slack_rad = 0.02;
  config.forward_capture_min_m = -0.40;
  config.forward_capture_max_m = 0.55;
  return config;
}

}  // namespace

TEST(PredockAlignmentPolicy, NormalizesStagingAndSignedYawErrorsAcrossPi)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.approach_yaw = kPi + 0.10;

  EXPECT_NEAR(policy.expected_staging_yaw(job), -kPi + 0.10, 1e-12);
  EXPECT_NEAR(policy.predock_yaw_error(kPi - 0.02, -kPi + 0.03), 0.05, 1e-12);
  EXPECT_NEAR(policy.contact_yaw_error(-kPi + 0.03, kPi - 0.02), -0.05, 1e-12);
}

TEST(PredockAlignmentPolicy, EvaluatesPoseInApproachCoordinatesWithoutChangingThresholds)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.approach_x = 1.0;
  job.approach_y = 2.0;
  job.approach_yaw = 0.0;
  job.dock_yaw = kPi;
  job.contact_frame = "charge_contact_link";
  job.reverse_yaw_offset_applied = true;

  robot_api_server::RobotPoseSnapshot pose;
  pose.available = true;
  pose.x = 1.2;
  pose.y = 1.9;
  pose.yaw = 0.04;

  const auto check = policy.evaluate_pose(job, pose, "unused");

  EXPECT_TRUE(check.pose_available);
  EXPECT_TRUE(check.xy_ok);
  EXPECT_TRUE(check.base_yaw_ok);
  EXPECT_TRUE(check.contact_yaw_ok);
  EXPECT_FALSE(check.contact_frame_available);
  EXPECT_NEAR(check.distance_m, std::hypot(0.2, -0.1), 1e-12);
  EXPECT_NEAR(check.forward_m, 0.2, 1e-12);
  EXPECT_NEAR(check.lateral_m, -0.1, 1e-12);
  EXPECT_NEAR(check.lateral_abs_m, 0.1, 1e-12);
  EXPECT_NEAR(check.base_yaw_error_rad, 0.04, 1e-12);
  EXPECT_NEAR(check.contact_yaw_error_rad, 0.04, 1e-12);
  EXPECT_NE(check.detail.find("forward_capture_min=-0.400"), std::string::npos);
  EXPECT_NE(check.detail.find("reverse_yaw_offset_applied=true"), std::string::npos);
  EXPECT_NE(
    check.detail.find("contact_yaw_source=base_link_aligned_charge_contact_link"),
    std::string::npos);
}

TEST(PredockAlignmentPolicy, PreservesUnavailablePoseFailureDetail)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.approach_yaw = 0.7;
  robot_api_server::RobotPoseSnapshot pose;

  const auto check = policy.evaluate_pose(job, pose, "TF is stale");

  EXPECT_FALSE(check.pose_available);
  EXPECT_NEAR(check.expected_base_yaw, 0.7, 1e-12);
  EXPECT_EQ(
    check.detail,
    "no fresh map-frame pose for predock approach check: TF is stale");
}

TEST(PredockAlignmentPolicy, KeepsHandoffAndCaptureBoundariesInclusive)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  predock::PredockPoseVerification check;
  check.pose_available = true;
  check.xy_ok = true;
  check.forward_m = -0.40;
  check.lateral_abs_m = 0.25;
  check.base_yaw_error_rad = 0.10;
  check.contact_yaw_error_rad = 0.10;

  EXPECT_TRUE(policy.pose_inside_handoff_window(check));
  EXPECT_TRUE(policy.forward_capture_window_met(check));
  EXPECT_TRUE(policy.lateral_capture_allowed(check));
  EXPECT_TRUE(policy.pose_allows_staging_recovery(check));

  check.forward_m = 0.550001;
  EXPECT_FALSE(policy.forward_capture_window_met(check));
  EXPECT_FALSE(policy.lateral_capture_allowed(check));

  check.forward_m = 0.55;
  check.lateral_abs_m = 0.250001;
  EXPECT_FALSE(policy.lateral_capture_allowed(check));
}

TEST(PredockAlignmentPolicy, KeepsYawAndStagingTargetsIndependentFromXyFlag)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  predock::PredockPoseVerification check;
  check.pose_available = true;
  check.xy_ok = false;
  check.base_yaw_ok = true;
  check.contact_yaw_ok = true;
  check.forward_m = 0.55;
  check.lateral_abs_m = 0.03;
  check.base_yaw_error_rad = 0.07;
  check.contact_yaw_error_rad = 0.07;

  EXPECT_FALSE(policy.yaw_target_met(check));
  EXPECT_TRUE(policy.yaw_angles_met(check));
  EXPECT_TRUE(policy.lateral_target_met(check));
  EXPECT_TRUE(policy.lateral_error_target_met(check));
  EXPECT_TRUE(policy.staging_target_met(check));
  EXPECT_NEAR(policy.lateral_align_yaw_gate_rad(), 0.07, 1e-12);
  EXPECT_TRUE(policy.lateral_align_yaw_gate_met(check));

  check.contact_yaw_error_rad = 0.070001;
  EXPECT_FALSE(policy.lateral_align_yaw_gate_met(check));
}

TEST(PredockAlignmentPolicy, AppliesPoseVerificationUsingExistingNav2ProofRule)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.nav_goal_succeeded = true;
  predock::PredockPoseVerification check;
  check.pose_available = true;
  check.xy_ok = true;
  check.base_yaw_ok = true;
  check.contact_yaw_ok = true;
  check.distance_m = 0.12;
  check.forward_m = 0.04;
  check.lateral_m = -0.02;
  check.lateral_abs_m = 0.02;
  check.detail = "verified";

  policy.apply_pose_verification(job, check);

  EXPECT_TRUE(job.predock_pose_verified);
  EXPECT_TRUE(job.predock_yaw_verified_by_nav2);
  EXPECT_DOUBLE_EQ(job.predock_distance_m, 0.12);
  EXPECT_DOUBLE_EQ(job.predock_lateral_m, -0.02);
  EXPECT_EQ(job.detail, "verified");

  job.predock_yaw_align_attempted = true;
  policy.apply_pose_verification(job, check);
  EXPECT_FALSE(job.predock_yaw_verified_by_nav2);
}

TEST(PredockAlignmentPolicy, AppliesYawFailureAndClearsNav2ProofAfterAttempt)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.predock_yaw_verified_by_nav2 = true;
  predock::PredockYawAlignResult result;
  result.attempted = true;
  result.succeeded = false;
  result.failure_code = "PREDOCK_YAW_HARD_FAIL";
  result.detail = "yaw outside window";
  result.initial_error_rad = 0.8;
  result.final_error_rad = 0.7;
  result.duration_sec = 1.2;
  result.observed_yaw_motion_rad = 0.1;

  policy.apply_yaw_align_result(job, result);

  EXPECT_TRUE(job.predock_yaw_align_attempted);
  EXPECT_FALSE(job.predock_yaw_align_succeeded);
  EXPECT_FALSE(job.predock_yaw_aligned);
  EXPECT_FALSE(job.predock_yaw_verified_by_nav2);
  EXPECT_EQ(job.predock_yaw_align_failure_code, "PREDOCK_YAW_HARD_FAIL");
  EXPECT_EQ(job.last_error_code, "PREDOCK_YAW_HARD_FAIL");
  EXPECT_EQ(job.last_error_detail, "yaw outside window");
  EXPECT_EQ(job.detail, "yaw outside window");
}

TEST(PredockAlignmentPolicy, PreservesCumulativeLateralAttemptState)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  job.predock_lateral_align_attempted = true;
  predock::PredockLateralAlignResult result;
  result.attempted = false;
  result.succeeded = true;
  result.failure_code = "NONE";
  result.detail = "already aligned";
  result.initial_error_m = 0.02;
  result.final_error_m = 0.02;

  policy.apply_lateral_align_result(job, result);

  EXPECT_TRUE(job.predock_lateral_align_attempted);
  EXPECT_TRUE(job.predock_lateral_align_succeeded);
  EXPECT_TRUE(job.predock_lateral_aligned);
  EXPECT_TRUE(job.predock_lateral_align_failure_code.empty());
  EXPECT_EQ(job.detail, "already aligned");
}

TEST(PredockAlignmentPolicy, AppliesFineEntryFailureWithoutChangingFailureCodeRules)
{
  const predock::PredockAlignmentPolicy policy(test_config());
  robot_api_server::DockingJob job;
  predock::PredockPoseVerification check;
  check.distance_m = 0.2;
  check.lateral_abs_m = 0.08;
  check.base_yaw_error_rad = 0.04;
  check.contact_yaw_error_rad = 0.05;

  policy.apply_fine_entry_check(
    job, check, false, "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE", "lateral too large");

  EXPECT_TRUE(job.fine_entry_checked);
  EXPECT_FALSE(job.fine_entry_ok);
  EXPECT_DOUBLE_EQ(job.fine_entry_distance_m, 0.2);
  EXPECT_DOUBLE_EQ(job.fine_entry_lateral_m, 0.08);
  EXPECT_EQ(job.fine_entry_failure_code, "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE");
  EXPECT_EQ(job.last_error_code, "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE");
  EXPECT_EQ(job.detail, "lateral too large");
}
