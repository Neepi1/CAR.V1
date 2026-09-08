#include "robot_safety/dock_contact_policy.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace robot_safety
{
namespace
{

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool json_true(const std::string & json, const std::string & key)
{
  const auto marker = "\"" + key + "\"";
  const auto key_pos = json.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const auto colon = json.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const auto value = json.find_first_not_of(" \t\r\n", colon + 1U);
  return value != std::string::npos && json.compare(value, 4U, "true") == 0;
}

std::string json_string(const std::string & json, const std::string & key)
{
  const auto marker = "\"" + key + "\"";
  const auto key_pos = json.find(marker);
  if (key_pos == std::string::npos) {
    return "none";
  }
  const auto colon = json.find(':', key_pos + marker.size());
  const auto begin = colon == std::string::npos ? std::string::npos : json.find('"', colon + 1U);
  const auto end = begin == std::string::npos ? std::string::npos : json.find('"', begin + 1U);
  if (begin == std::string::npos || end == std::string::npos) {
    return "none";
  }
  return json.substr(begin + 1U, end - begin - 1U);
}

bool strong_source(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "charging_session" ||
         normalized == "docking_manager_bms" ||
         normalized == "docking_job" ||
         normalized == "docking_status" ||
         normalized == "docking_manager" ||
         normalized == "manual" ||
         normalized == "manual_confirm";
}

}  // namespace

PersistentDockEvidence parse_persistent_dock_evidence(const std::string & json)
{
  PersistentDockEvidence evidence;
  evidence.latched_docked =
    json_true(json, "latched_docked") || json_true(json, "docked");
  evidence.source = json_string(json, "source");
  evidence.dock_id = json_string(json, "dock_id");
  evidence.building_id = json_string(json, "building_id");
  evidence.floor_id = json_string(json, "floor_id");
  evidence.map_id = json_string(json, "map_id");
  if (evidence.dock_id == "none") {
    evidence.dock_id.clear();
  }
  if (evidence.building_id == "none") {
    evidence.building_id.clear();
  }
  if (evidence.floor_id == "none") {
    evidence.floor_id.clear();
  }
  if (evidence.map_id == "none") {
    evidence.map_id.clear();
  }
  evidence.strong = evidence.latched_docked && strong_source(evidence.source);
  return evidence;
}

bool dock_latch_blocks_normal_motion(
  const PersistentDockEvidence & evidence,
  const bool fresh_bms_no_contact,
  const bool docking_status_indicates_docked)
{
  if (!evidence.latched_docked) {
    return false;
  }
  if (evidence.strong) {
    return true;
  }
  return !(fresh_bms_no_contact && !docking_status_indicates_docked);
}

bool should_latch_bms_docking_interlock(
  const bool live_contact,
  const bool fresh_docking_command,
  const bool docking_status_context,
  const PersistentDockEvidence & evidence)
{
  return live_contact &&
         (fresh_docking_command || docking_status_context || evidence.strong);
}

bool bms_docking_interlock_is_active(
  const bool in_memory_latch,
  const bool live_contact,
  const PersistentDockEvidence & evidence)
{
  return in_memory_latch || live_contact || evidence.strong;
}

DockInterlockReconcileDecision evaluate_dock_interlock_reconcile(
  const DockInterlockReconcileContext & context)
{
  if (!context.outside_dock_zone_proven) {
    return {false, "OUTSIDE_DOCK_ZONE_NOT_PROVEN", "robot is not proven outside the dock zone"};
  }
  if (!context.battery_sample_fresh) {
    return {false, "BMS_STATE_NOT_FRESH", "fresh BMS feedback is required"};
  }
  if (context.live_bms_contact) {
    return {false, "BMS_CONTACT_ACTIVE", "charging contact is still active"};
  }
  if (context.no_contact_duration_sec < context.required_no_contact_duration_sec) {
    return {
      false,
      "BMS_NO_CONTACT_NOT_STABLE",
      "BMS no-contact evidence has not remained stable long enough"};
  }
  if (context.docking_status_indicates_docked) {
    return {false, "DOCKING_STATUS_DOCKED", "docking runtime still reports docked"};
  }
  if (context.reverse_permit_active) {
    return {false, "UNDOCK_REVERSE_ACTIVE", "controlled undock reverse permit is active"};
  }
  if (context.fresh_docking_command) {
    return {false, "DOCKING_COMMAND_ACTIVE", "a fresh docking command is still active"};
  }
  if (!context.memory_latched) {
    return {true, "ALREADY_CLEAR", "BMS docking memory interlock is already clear"};
  }
  return {true, "OK", "stale BMS docking memory interlock may be reconciled"};
}

}  // namespace robot_safety
