#pragma once

#include <string>

namespace robot_safety
{

struct PersistentDockEvidence
{
  bool latched_docked{false};
  bool strong{false};
  std::string source{"none"};
  std::string dock_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
};

struct DockInterlockReconcileContext
{
  bool memory_latched{false};
  bool outside_dock_zone_proven{false};
  bool battery_sample_fresh{false};
  bool live_bms_contact{false};
  double no_contact_duration_sec{0.0};
  double required_no_contact_duration_sec{3.0};
  bool docking_status_indicates_docked{false};
  bool reverse_permit_active{false};
  bool fresh_docking_command{false};
};

struct DockInterlockReconcileDecision
{
  bool allowed{false};
  std::string code{"INVALID_REQUEST"};
  std::string detail;
};

PersistentDockEvidence parse_persistent_dock_evidence(const std::string & json);

bool dock_latch_blocks_normal_motion(
  const PersistentDockEvidence & evidence,
  bool fresh_bms_no_contact,
  bool docking_status_indicates_docked);

bool should_latch_bms_docking_interlock(
  bool live_contact,
  bool fresh_docking_command,
  bool docking_status_context,
  const PersistentDockEvidence & evidence);

bool bms_docking_interlock_is_active(
  bool in_memory_latch,
  bool live_contact,
  const PersistentDockEvidence & evidence);

DockInterlockReconcileDecision evaluate_dock_interlock_reconcile(
  const DockInterlockReconcileContext & context);

}  // namespace robot_safety
