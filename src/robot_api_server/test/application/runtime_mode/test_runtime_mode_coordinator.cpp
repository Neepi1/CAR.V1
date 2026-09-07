#include <gtest/gtest.h>

#include <string>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"

namespace runtime_mode = robot_api_server::application::runtime_mode;

TEST(RuntimeModeCoordinator, StartsIdleAndHealthy)
{
  const runtime_mode::RuntimeModeCoordinator coordinator;

  const auto snapshot = coordinator.snapshot();

  EXPECT_EQ(snapshot.mode, "IDLE");
  EXPECT_EQ(snapshot.state, "idle");
  EXPECT_EQ(snapshot.mapping_state, "stopped");
  EXPECT_EQ(snapshot.navigation_state, "stopped");
  EXPECT_EQ(snapshot.docking_state, "stopped");
  EXPECT_TRUE(snapshot.healthy);
}

TEST(RuntimeModeCoordinator, MappingOwnsTheWholeRuntimeWhenActivated)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_navigation(true, "running");
  coordinator.set_docking(true, "approaching");

  coordinator.set_mapping(true, "running", "mapping ready");

  const auto snapshot = coordinator.snapshot();
  EXPECT_EQ(snapshot.mode, "MAPPING_2D");
  EXPECT_EQ(snapshot.state, "running");
  EXPECT_TRUE(snapshot.mapping_active);
  EXPECT_FALSE(snapshot.navigation_active);
  EXPECT_FALSE(snapshot.docking_active);
  EXPECT_EQ(snapshot.navigation_state, "stopped");
  EXPECT_EQ(snapshot.docking_state, "stopped");
  EXPECT_EQ(snapshot.message, "mapping ready");
}

TEST(RuntimeModeCoordinator, NavigationAndDockingOnlyStopMapping)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_mapping(true, "running");
  coordinator.set_navigation(true, "active");

  auto snapshot = coordinator.snapshot();
  EXPECT_FALSE(snapshot.mapping_active);
  EXPECT_TRUE(snapshot.navigation_active);
  EXPECT_EQ(snapshot.mode, "NAVIGATION");

  coordinator.set_docking(true, "approaching");
  snapshot = coordinator.snapshot();
  EXPECT_FALSE(snapshot.mapping_active);
  EXPECT_TRUE(snapshot.navigation_active);
  EXPECT_TRUE(snapshot.docking_active);
  EXPECT_EQ(snapshot.mode, "DOCKING");
  EXPECT_EQ(snapshot.state, "approaching");
}

TEST(RuntimeModeCoordinator, UnhealthyStateHasHighestSnapshotPriority)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_docking(true, "FINE_ALIGN", "sensor unavailable", false);

  const auto snapshot = coordinator.snapshot();

  EXPECT_EQ(snapshot.mode, "ERROR");
  EXPECT_EQ(snapshot.state, "error");
  EXPECT_FALSE(snapshot.healthy);
  EXPECT_EQ(snapshot.message, "sensor unavailable");
}

TEST(RuntimeModeCoordinator, ClassifiesFineDockingAndRecoveryProfiles)
{
  runtime_mode::RuntimeModeSnapshot snapshot;
  snapshot.docking_active = true;
  snapshot.docking_state = "FINE_DOCKING_ENTRY_CHECK";
  EXPECT_EQ(runtime_mode::active_runtime_profile(snapshot), "docking_fine");

  snapshot.docking_active = false;
  snapshot.docking_state = "stopped";
  snapshot.navigation_state = "relocalizing";
  EXPECT_EQ(runtime_mode::active_runtime_profile(snapshot), "recovery");

  snapshot.navigation_state = "running";
  snapshot.docking_state = "RELOCALIZE_FOR_DOCK";
  EXPECT_EQ(runtime_mode::active_runtime_profile(snapshot), "recovery");

  snapshot.docking_state = "approaching";
  EXPECT_EQ(runtime_mode::active_runtime_profile(snapshot), "normal");
}

TEST(RuntimeModeCoordinator, TransitionOwnerIsExclusiveAndOwnerScoped)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  std::string conflict_owner;

  EXPECT_TRUE(coordinator.try_begin_transition("mapping_start", conflict_owner));
  EXPECT_TRUE(conflict_owner.empty());
  EXPECT_EQ(coordinator.transition_owner(), "mapping_start");

  EXPECT_FALSE(coordinator.try_begin_transition("navigation_start", conflict_owner));
  EXPECT_EQ(conflict_owner, "mapping_start");

  coordinator.finish_transition("navigation_start");
  EXPECT_EQ(coordinator.transition_owner(), "mapping_start");

  coordinator.finish_transition("mapping_start");
  EXPECT_TRUE(coordinator.transition_owner().empty());
}

TEST(RuntimeModeCoordinator, DockingAdmissionUpdatesIdentityAtomically)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_mapping(true, "running", "mapping");
  coordinator.set_docking_status("old status");

  coordinator.accept_docking("dock-7", "docking accepted");

  const auto snapshot = coordinator.snapshot();
  EXPECT_TRUE(snapshot.docking_active);
  EXPECT_EQ(snapshot.docking_state, "accepted");
  EXPECT_EQ(snapshot.docking_dock_id, "dock-7");
  EXPECT_TRUE(snapshot.docking_status.empty());
  EXPECT_FALSE(snapshot.mapping_active);
  EXPECT_EQ(snapshot.mapping_state, "stopped");
  EXPECT_TRUE(snapshot.healthy);
  EXPECT_EQ(snapshot.message, "docking accepted");
}

TEST(RuntimeModeCoordinator, DockingFinishOnlyStopsNavigationAfterContactSuccess)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_navigation(true, "running");
  coordinator.accept_docking("dock-7");

  coordinator.finish_docking("failed", "contact not proven");
  auto snapshot = coordinator.snapshot();
  EXPECT_FALSE(snapshot.docking_active);
  EXPECT_TRUE(snapshot.navigation_active);
  EXPECT_EQ(snapshot.docking_state, "failed");
  EXPECT_EQ(snapshot.docking_status, "contact not proven");

  coordinator.accept_docking("dock-7");
  coordinator.finish_docking("charging", "charging confirmed");
  snapshot = coordinator.snapshot();
  EXPECT_FALSE(snapshot.docking_active);
  EXPECT_FALSE(snapshot.navigation_active);
  EXPECT_EQ(snapshot.navigation_state, "stopped");
  EXPECT_EQ(snapshot.docking_state, "charging");
  EXPECT_EQ(snapshot.docking_status, "charging confirmed");
}

TEST(RuntimeModeCoordinator, DockingIdentityMayUpdateWithoutLosingCurrentMessage)
{
  runtime_mode::RuntimeModeCoordinator coordinator;
  coordinator.set_docking(true, "undocking", "auto undock accepted");

  coordinator.set_docking_identity("dock-2");
  auto snapshot = coordinator.snapshot();
  EXPECT_EQ(snapshot.docking_dock_id, "dock-2");
  EXPECT_EQ(snapshot.message, "auto undock accepted");

  coordinator.set_docking_identity("dock-3", "manual undock accepted");
  snapshot = coordinator.snapshot();
  EXPECT_EQ(snapshot.docking_dock_id, "dock-3");
  EXPECT_EQ(snapshot.message, "manual undock accepted");
}
