#include <gtest/gtest.h>

#include "robot_safety/elevator_entry_collision_bypass_policy.hpp"

namespace
{

using robot_safety::ElevatorEntryCollisionBypassContext;
using robot_safety::ElevatorEntryCollisionBypassPermit;
using robot_safety::elevator_entry_collision_bypass_authorized;

ElevatorEntryCollisionBypassContext valid_context()
{
  ElevatorEntryCollisionBypassContext context;
  context.hold_clear = true;
  context.operating_mode_contract_valid = true;
  context.operating_mode_owner = "robot_elevator_manager";
  context.operating_mode_mission_id = "elevator_elevator-test-42";
  context.operating_mode = "DOORWAY";
  return context;
}

TEST(
  ElevatorEntryCollisionBypassPolicy,
  AuthorizesPostCallStagingInElevatorWait)
{
  const ElevatorEntryCollisionBypassPermit permit{"elevator-test-42", 100.0};
  auto context = valid_context();
  context.operating_mode = "ELEVATOR_WAIT";
  EXPECT_TRUE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));
}

TEST(
  ElevatorEntryCollisionBypassPolicy,
  ElevatorWaitStillRequiresFreshMatchingPermitAndClearHold)
{
  const ElevatorEntryCollisionBypassPermit permit{"elevator-test-42", 100.0};
  auto context = valid_context();
  context.operating_mode = "ELEVATOR_WAIT";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      ElevatorEntryCollisionBypassPermit{}, context, 100.2, 0.75));
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.8, 0.75));

  context.operating_mode_mission_id = "elevator_another-transaction";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));
  context.operating_mode_mission_id = "elevator_elevator-test-42";
  context.operating_mode_owner = "robot_mission_manager";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));
  context.operating_mode_owner = "robot_elevator_manager";
  context.operating_mode_contract_valid = false;
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));
  context.operating_mode_contract_valid = true;
  context.hold_clear = false;
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));
}

TEST(
  ElevatorEntryCollisionBypassPolicy,
  RejectsOrdinaryNavigationAndStationaryRideModesEvenWithPermit)
{
  const ElevatorEntryCollisionBypassPermit permit{"elevator-test-42", 100.0};
  auto context = valid_context();
  for (const auto * mode : {"NORMAL", "ELEVATOR_RIDE", "RECOVERY", ""}) {
    SCOPED_TRACE(mode);
    context.operating_mode = mode;
    EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
        permit, context, 100.2, 0.75));
  }
}

TEST(
  ElevatorEntryCollisionBypassPolicy,
  AuthorizesOnlyFreshExactDoorwayTransaction)
{
  const ElevatorEntryCollisionBypassPermit permit{
    "elevator-test-42", 100.0};
  EXPECT_TRUE(elevator_entry_collision_bypass_authorized(
      permit, valid_context(), 100.2, 0.75));

  auto context = valid_context();
  context.operating_mode_mission_id = "elevator_another-transaction";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  context = valid_context();
  context.operating_mode = "ELEVATOR_RIDE";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  context = valid_context();
  context.hold_clear = false;
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  context = valid_context();
  context.operating_mode_owner = "robot_mission_manager";
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, valid_context(), 100.8, 0.75));
}

TEST(
  ElevatorEntryCollisionBypassPolicy,
  FailsClosedOnMissingOrInvalidRuntimeEvidence)
{
  const ElevatorEntryCollisionBypassPermit permit{
    "elevator-test-42", 100.0};

  auto context = valid_context();
  context.operating_mode_contract_valid = false;
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  context = valid_context();
  context.operating_mode_mission_id.clear();
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  context = valid_context();
  context.operating_mode_owner.clear();
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, context, 100.2, 0.75));

  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      ElevatorEntryCollisionBypassPermit{}, valid_context(), 100.2, 0.75));
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, valid_context(), 99.9, 0.75));
  EXPECT_FALSE(elevator_entry_collision_bypass_authorized(
      permit, valid_context(), 100.2, 0.0));
}

}  // namespace
