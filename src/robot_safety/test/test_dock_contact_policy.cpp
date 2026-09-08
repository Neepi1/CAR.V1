#include <gtest/gtest.h>

#include "robot_safety/dock_contact_policy.hpp"

namespace robot_safety
{

TEST(DockContactPolicy, StrongSuccessfulDockLatchSurvivesFreshNoContact)
{
  const auto evidence = parse_persistent_dock_evidence(
    R"({"latched_docked":true,"source":"docking_job"})");

  EXPECT_TRUE(evidence.strong);
  EXPECT_TRUE(dock_latch_blocks_normal_motion(evidence, true, false));
  EXPECT_TRUE(bms_docking_interlock_is_active(false, false, evidence));
}

TEST(DockContactPolicy, WeakLegacyBmsLatchMayBeContradictedByFreshNoContact)
{
  const auto evidence = parse_persistent_dock_evidence(
    R"({"latched_docked":true,"source":"bms"})");

  EXPECT_FALSE(evidence.strong);
  EXPECT_FALSE(dock_latch_blocks_normal_motion(evidence, true, false));
}

TEST(DockContactPolicy, UnscopedLiveContactDoesNotCreateMemoryLatch)
{
  const PersistentDockEvidence no_evidence;

  EXPECT_FALSE(should_latch_bms_docking_interlock(
      true, false, false, no_evidence));
  EXPECT_TRUE(should_latch_bms_docking_interlock(
      true, true, false, no_evidence));
}

TEST(DockContactPolicy, ClearedLatchDoesNotBlockEvenWhenSourceWasStrong)
{
  const auto evidence = parse_persistent_dock_evidence(
    R"({"latched_docked":false,"source":"docking_job"})");

  EXPECT_FALSE(evidence.strong);
  EXPECT_FALSE(dock_latch_blocks_normal_motion(evidence, false, false));
  EXPECT_FALSE(bms_docking_interlock_is_active(false, false, evidence));
}

TEST(DockContactPolicy, ReconcileAllowsOnlyProvenRemoteUndock)
{
  DockInterlockReconcileContext context;
  context.memory_latched = true;
  context.outside_dock_zone_proven = true;
  context.battery_sample_fresh = true;
  context.live_bms_contact = false;
  context.no_contact_duration_sec = 5.0;
  context.required_no_contact_duration_sec = 3.0;
  context.docking_status_indicates_docked = false;
  context.reverse_permit_active = false;
  context.fresh_docking_command = false;

  const auto decision = evaluate_dock_interlock_reconcile(context);

  EXPECT_TRUE(decision.allowed);
  EXPECT_EQ(decision.code, "OK");
}

TEST(DockContactPolicy, ReconcileRejectsEveryUnsafeCondition)
{
  DockInterlockReconcileContext safe;
  safe.memory_latched = true;
  safe.outside_dock_zone_proven = true;
  safe.battery_sample_fresh = true;
  safe.no_contact_duration_sec = 5.0;
  safe.required_no_contact_duration_sec = 3.0;

  auto context = safe;
  context.outside_dock_zone_proven = false;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "OUTSIDE_DOCK_ZONE_NOT_PROVEN");

  context = safe;
  context.battery_sample_fresh = false;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "BMS_STATE_NOT_FRESH");

  context = safe;
  context.live_bms_contact = true;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "BMS_CONTACT_ACTIVE");

  context = safe;
  context.no_contact_duration_sec = 2.9;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "BMS_NO_CONTACT_NOT_STABLE");

  context = safe;
  context.docking_status_indicates_docked = true;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "DOCKING_STATUS_DOCKED");

  context = safe;
  context.reverse_permit_active = true;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "UNDOCK_REVERSE_ACTIVE");

  context = safe;
  context.fresh_docking_command = true;
  EXPECT_EQ(evaluate_dock_interlock_reconcile(context).code, "DOCKING_COMMAND_ACTIVE");
}

}  // namespace robot_safety
