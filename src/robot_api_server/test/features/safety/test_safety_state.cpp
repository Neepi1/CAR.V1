#include <gtest/gtest.h>

#include "robot_api_server/features/safety/safety_state.hpp"

namespace safety = robot_api_server::features::safety;

TEST(SafetyState, UnknownEvidenceDefersToFinalRobotSafetyArbitration)
{
  const safety::SafetyStateSnapshot snapshot{};

  const auto admission = safety::evaluate_motion_allowed(snapshot);
  EXPECT_TRUE(admission.allowed);
  EXPECT_EQ(admission.detail, "motion_allowed unavailable");

  const auto hard_block = safety::evaluate_hard_block(snapshot);
  EXPECT_FALSE(hard_block.blocked);
  EXPECT_EQ(
    hard_block.detail,
    "motion_allowed unavailable; relying on robot_safety arbitration");
}

TEST(SafetyState, FreshAllowedEvidenceAdmitsMotion)
{
  const safety::SafetyStateSnapshot snapshot{"OK", true, true};

  const auto admission = safety::evaluate_motion_allowed(snapshot);
  EXPECT_TRUE(admission.allowed);
  EXPECT_EQ(admission.detail, "motion allowed");

  const auto hard_block = safety::evaluate_hard_block(snapshot);
  EXPECT_FALSE(hard_block.blocked);
  EXPECT_EQ(hard_block.detail, "motion allowed");
}

TEST(SafetyState, CommandStaleRemainsFirstCommandWarmupInsteadOfHardBlock)
{
  const safety::SafetyStateSnapshot snapshot{"COMMAND_STALE", false, true};

  const auto admission = safety::evaluate_motion_allowed(snapshot);
  EXPECT_FALSE(admission.allowed);
  EXPECT_EQ(admission.detail, "motion not allowed");

  const auto hard_block = safety::evaluate_hard_block(snapshot);
  EXPECT_FALSE(hard_block.blocked);
  EXPECT_EQ(
    hard_block.detail,
    "motion_allowed false because command stream is stale; "
    "final yaw command may refresh robot_safety");
}

TEST(SafetyState, ExplicitRobotSafetyDenialIsAHardBlock)
{
  const safety::SafetyStateSnapshot snapshot{
    "DOCKED_CONTACT_BLOCK", false, true};

  const auto hard_block = safety::evaluate_hard_block(snapshot);
  EXPECT_TRUE(hard_block.blocked);
  EXPECT_EQ(
    hard_block.detail,
    "motion not allowed by robot_safety: DOCKED_CONTACT_BLOCK");
  EXPECT_EQ(
    safety::normal_motion_blocked_reason(snapshot),
    "DOCKED_CONTACT_BLOCK");
}

TEST(SafetyState, JsonContractsRemainBackwardCompatible)
{
  const safety::SafetyStateSnapshot snapshot{"ESTOP_ACTIVE", false, true};

  EXPECT_EQ(
    safety::safety_state_json(snapshot),
    "{\"status\":\"ESTOP_ACTIVE\",\"motion_allowed\":false,"
    "\"motion_allowed_valid\":true}");
  EXPECT_EQ(
    safety::safety_estop_response_json(true),
    "{\"ok\":true,\"estop\":true}");
  EXPECT_EQ(
    safety::safety_estop_response_json(false),
    "{\"ok\":true,\"estop\":false}");
}
