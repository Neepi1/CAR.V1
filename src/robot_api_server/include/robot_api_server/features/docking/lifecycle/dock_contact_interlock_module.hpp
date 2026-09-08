#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/power/bms_contact.hpp"

namespace robot_api_server::features::docking
{

struct DockContactLatchSnapshot
{
  bool valid{false};
  bool docked{false};
  bool latched_docked{false};
  std::string source{"none"};
  std::string reason{"not_latched"};
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::string dock_id;
  std::string latched_at;
  std::string last_confirmed_at;
  std::string cleared_at;
  std::string clear_reason;
  std::string note;
  std::string updated_at;
  double age_sec{-1.0};
  bool source_bms{false};
  bool source_charging_session{false};
  bool stale{false};
  bool contradicted_by_live_state{false};
};

struct DockSafetyInterlockSnapshot
{
  bool available{false};
  bool fresh{false};
  double age_sec{-1.0};
  bool enabled{true};
  bool memory_latched{false};
  bool active{false};
  bool battery_sample_fresh{false};
  bool live_bms_contact{false};
  double no_contact_duration_sec{0.0};
  bool persistent_dock_latched{false};
  bool persistent_dock_strong{false};
  bool reverse_session_seen{false};
  bool reverse_permit_active{false};
  std::string dock_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::string state{"UNAVAILABLE"};
  std::string reason{"state_not_received"};
};

struct DockZoneSnapshot
{
  std::string state{"UNKNOWN"};
  std::string reason{"not_evaluated"};
  std::string dock_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  double distance_m{-1.0};
};

struct PreNavigationDockCheck
{
  application::runtime_mode::RuntimeModeSnapshot runtime;
  BmsChargingContactSnapshot bms;
  DockContactLatchSnapshot dock_latch;
  DockSafetyInterlockSnapshot safety_interlock;
  DockZoneSnapshot dock_zone;
  bool runtime_state_docked{false};
  bool runtime_state_charging{false};
  bool runtime_state_undocking{false};
  bool docking_status_indicates_docked{false};
  bool docking_status_indicates_charging{false};
  bool docking_status_indicates_undocking{false};
  bool dock_latch_indicates_docked{false};
  bool dock_contact_latch_present{false};
  bool dock_contact_latch_latched_docked{false};
  std::string dock_contact_latch_source;
  std::string dock_contact_latch_reason;
  double dock_contact_latch_age_sec{-1.0};
  bool dock_contact_latch_stale{false};
  bool dock_contact_latch_contradicted_by_live_state{false};
  bool dock_contact_latch_auto_cleared{false};
  std::string dock_contact_latch_clear_reason;
  std::string dock_contact_latch_source_strength{"none"};
  bool charging_session_latched{false};
  double charging_session_age_sec{-1.0};
  std::string charging_session_last_confirmed_at;
  bool full_charge_idle_on_dock{false};
  bool live_docking_state_undocked{false};
  bool live_bms_charging_contact_stable{false};
  bool strong_live_docked{false};
  bool latch_valid_for_auto_undock{false};
  bool inferred_docked{false};
  bool final_is_docked_or_charging{false};
  bool final_auto_undock_required{false};
  bool docking_active_not_docked_block{false};
  bool can_auto_undock{false};
  bool safety_interlock_state_block{false};
  bool safety_interlock_memory_latched{false};
  bool clear_stale_safety_interlock_required{false};
  bool pre_navigation_recovery_required{false};
  std::string pre_navigation_recovery_action{"NONE"};
  std::string pre_navigation_block_reason;
  std::string resolved_dock_id;
  std::string docked_state_class{"UNKNOWN"};
  std::vector<std::string> docked_evidence;
  std::vector<std::string> docked_warnings;
  std::string dock_occupancy_state{"UNKNOWN"};
  std::vector<std::string> dock_occupancy_evidence;
  std::string dock_occupancy_reason{"no_dock_evidence"};
  std::string auto_undock_reason{"not_docked"};
};

struct PreNavigationDockCheckContext
{
  std::string request_source;
  std::string pose_id;
  std::string building_id;
  std::string floor_id;
  std::string frame_id{"map"};
  bool direct_pose{false};
};

struct DockContactInterlockConfig
{
  std::filesystem::path latch_file;
  double bms_ttl_sec{300.0};
  double bms_clear_no_contact_sec{3.0};
  bool allow_bms_stale_auto_undock{false};
  bool clear_when_live_undocked_no_contact{true};
  double max_age_warn_sec{600.0};
  double charging_current_min_a{0.10};
  double full_soc_threshold_pct{99.0};
  std::string docking_status_topic{"/docking/status"};
  bool require_safety_interlock_state{true};
  std::string safety_interlock_state_topic{"/safety/dock_interlock_state"};
  std::string safety_interlock_reconcile_service{"/safety/reconcile_dock_interlock"};
  double safety_interlock_state_max_age_sec{1.0};
  double safety_interlock_reconcile_timeout_sec{3.0};
  double dock_zone_pose_max_age_sec{0.5};
  double dock_zone_near_radius_m{1.0};
  double dock_zone_clear_radius_m{1.5};
};

struct DockContactInterlockPorts
{
  std::function<application::runtime_mode::RuntimeModeSnapshot()> runtime_snapshot;
  std::function<BmsChargingContactSnapshot()> bms_snapshot;
  std::function<DockSafetyInterlockSnapshot()> safety_interlock_snapshot;
  std::function<DockZoneSnapshot(
      const DockContactLatchSnapshot &,
      const DockSafetyInterlockSnapshot &,
      const application::runtime_mode::RuntimeModeSnapshot &)> dock_zone_snapshot;
  std::function<bool()> navigation_goal_running;
  std::function<double()> wall_time_seconds;
  std::function<std::string()> timestamp_now;
  std::function<void(const std::string &)> warn;
};

// Owns persistent dock-contact memory and the complete evidence-to-occupancy
// decision. It observes state only; controlled undock execution remains a
// separate docking lifecycle transaction.
class DockContactInterlockModule
{
public:
  DockContactInterlockModule(
    DockContactInterlockConfig config,
    DockContactInterlockPorts ports);
  ~DockContactInterlockModule();

  DockContactInterlockModule(const DockContactInterlockModule &) = delete;
  DockContactInterlockModule & operator=(const DockContactInterlockModule &) = delete;

  DockContactLatchSnapshot read_latch() const;
  void update_latch(
    bool docked,
    const std::string & source,
    const std::string & reason,
    const std::string & dock_id,
    const std::string & building_id = "",
    const std::string & floor_id = "",
    const std::string & map_id = "",
    const std::string & note = "");
  void on_bms_contact_evidence(
    const BatteryContactEvaluation & charging_contact,
    bool contact_stable,
    double stable_duration_sec);

  PreNavigationDockCheck snapshot();
  std::string bms_snapshot_json(const BmsChargingContactSnapshot & bms) const;
  std::string latch_snapshot_json(const DockContactLatchSnapshot & latch) const;
  std::string pre_navigation_check_json(
    const PreNavigationDockCheck & check,
    const PreNavigationDockCheckContext & context) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::docking
