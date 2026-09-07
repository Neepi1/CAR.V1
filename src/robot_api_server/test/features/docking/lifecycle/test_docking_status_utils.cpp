#include <gtest/gtest.h>

#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"

namespace robot_api_server
{
namespace
{

TEST(DockingStatusUtils, ContactStoppingDiagnosticsAreNotTerminalFailures)
{
  EXPECT_FALSE(docking_status_is_failure(
      "contact_stopping brake_confirmed=false bms_reason=current_above_threshold "
      "contact_stop_feedback_timeout=false"));
  EXPECT_FALSE(docking_status_is_failure(
      "contact_stopping brake_confirmed=false bms_reason=current_above_threshold "
      "contact_stop_feedback_timeout=true"));
  EXPECT_FALSE(docking_status_is_failure(
      "contact_stopping failure_deferred=true reason=contact_verify_timeout"));
}

TEST(DockingStatusUtils, DiagnosticTimeoutFieldsDoNotOverrideStateCode)
{
  EXPECT_FALSE(docking_status_is_failure(
      "undocking waiting_first_motion phase=waiting_first_motion "
      "motion_start_timeout_s=6.0"));
  EXPECT_FALSE(docking_status_is_failure(
      "undocked phase=succeeded failure_reason=none motion_start_timeout_s=6.0"));
  EXPECT_FALSE(docking_status_is_failure(
      "aligning points=42 x=0.340 y=0.004 yaw_deg=0.2 timeout=false"));
}

TEST(DockingStatusUtils, ExplicitFailureStateCodesRemainFailures)
{
  EXPECT_TRUE(docking_status_is_failure("contact_verify_timeout"));
  EXPECT_TRUE(docking_status_is_failure("contact_verify_failed_distance_limit"));
  EXPECT_TRUE(docking_status_is_failure(
      "contact_retry_backoff_failed_timeout phase=failed"));
  EXPECT_TRUE(docking_status_is_failure(
      "undock_failed_motion_start_timeout phase=failed"));
  EXPECT_TRUE(docking_status_is_failure("dock_feature_not_found"));
  EXPECT_TRUE(docking_status_is_failure("request_rejected reason=unsafe"));
  EXPECT_TRUE(docking_status_is_failure("failed reason=unknown"));
  EXPECT_TRUE(docking_status_is_failure("timeout reason=no_response"));
}

TEST(DockingStatusUtils, LeadingStateCodeOwnsClassification)
{
  EXPECT_TRUE(docking_status_is_success(
      "  docked_charging_detected brake_confirmed=true"));
  EXPECT_TRUE(docking_status_is_undocking(
      "undocking active failure_reason=none motion_start_timeout_s=6.0"));
  EXPECT_TRUE(docking_status_is_undocked(
      "undocked phase=succeeded failure_reason=none"));
  EXPECT_TRUE(docking_status_is_undock_failed(
      "undock_failed_no_progress phase=failed"));
  EXPECT_TRUE(docking_status_is_stopped("stopped by service"));

  EXPECT_FALSE(docking_status_is_success(
      "contact_stopping inferred_state=docked"));
  EXPECT_FALSE(docking_status_is_stopped(
      "contact_stopping wheel_stopped=false"));
}

}  // namespace
}  // namespace robot_api_server
