#pragma once

#include <string>

namespace robot_safety
{

struct PersistentDockEvidence
{
  bool latched_docked{false};
  bool strong{false};
  std::string source{"none"};
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

}  // namespace robot_safety
