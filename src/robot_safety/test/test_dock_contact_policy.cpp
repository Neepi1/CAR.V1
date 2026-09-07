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

}  // namespace robot_safety
