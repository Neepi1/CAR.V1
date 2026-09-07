#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/maps/poses/poses_io.hpp"
#include "robot_api_server/features/power/bms_contact.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::docking
{

struct DockingHttpOccupancySnapshot
{
  bool runtime_state_undocking{false};
  bool docking_status_indicates_undocking{false};
  bool dock_latch_indicates_docked{false};
  bool inferred_docked{false};
  bool final_is_docked_or_charging{false};
  bool can_auto_undock{false};
  bool charging_session_latched{false};
  bool full_charge_idle_on_dock{false};
  double charging_session_age_sec{-1.0};
  std::string charging_session_last_confirmed_at;
  std::string dock_occupancy_state{"UNKNOWN"};
  std::vector<std::string> dock_occupancy_evidence;
  std::string dock_occupancy_reason{"no_dock_evidence"};
  std::string auto_undock_reason{"not_docked"};
  std::string json{"{}"};
};

struct DockingHttpConfig
{
  double pre_dock_distance_m{0.65};
  std::string default_dock_profile_id{"ranger_mini3_default"};
  std::string default_dock_profile_type{"charging_dock"};
  std::string default_approach_direction{"reverse"};
  std::string default_contact_frame{"base_link"};
  std::string default_sensor_frame{"docking_sensor_link"};
  int max_retries{2};
};

struct DockingHttpPorts
{
  std::function<std::optional<HttpResponse>(const std::string &)>
  floor_runtime_interlock_response;
  std::function<bool()> mapping_start_job_running;
  std::function<std::optional<RuntimeMapContext>()> read_runtime_map_context;
  std::function<std::optional<StoredPose>(
      const std::string &,
      const std::string &,
      const std::string &,
      const StoredPose &,
      const std::string &,
      std::string &,
      std::string &,
      int &)>
  resolve_predock_pose;
  std::function<bool(const StoredPose &, const StoredPose &, std::string &)>
  validate_predock_pose;
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_teleop_zero_burst;
  // Called with docking_start_mutex held to retire a conflicting docking
  // owner before an explicit controlled-undock transaction is created.
  std::function<bool(std::string &)> prepare_controlled_undock;
  std::function<void()> join_docking_worker;
  std::function<bool(std::uint64_t, std::string &)> launch_docking_worker;
  std::function<bool(std::string &)> cancel_active_navigation_goal;
  std::function<bool(std::string &)> stop_docking_if_available;
  std::function<bool(
      std::uint64_t,
      bool,
      const std::string &,
      std::string &)>
  set_global_correction_paused;
  std::function<void(
      std::uint64_t,
      bool,
      const std::string &,
      const std::string &)>
  finish_docking_job;
  std::function<void(
      bool,
      const std::string &,
      const std::string &,
      const std::string &,
      const std::string &,
      const std::string &,
      const std::string &,
      const std::string &)>
  update_dock_contact_latch;
  std::function<std::string()> dock_contact_latch_json;
  std::function<BmsChargingContactSnapshot()> bms_charging_contact_snapshot;
  std::function<DockingHttpOccupancySnapshot()> docking_occupancy_snapshot;
  std::function<bool(std::string &)> ensure_docking_manager_running;
  std::function<bool(
      std::string &,
      bool,
      DockingUndockServiceObservation *)>
  call_undock_service_with_charging_retry;
  std::function<void(DockingJob &, const std::string &)>
  record_undock_status_observation;
};

// Owns the complete API-facing docking HTTP transaction surface. The shared
// DockingJob storage remains injected during the lifecycle migration so the
// executor, status callback and HTTP endpoints observe one unchanged state.
class DockingHttpModule
{
public:
  DockingHttpModule(
    features::maps::MapsModule & maps_module,
    features::elevator::ElevatorModule & elevator_module,
    application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
    std::mutex & docking_start_mutex,
    DockingJobStore & job_store,
    DockingHttpConfig config,
    DockingHttpPorts ports);
  ~DockingHttpModule();

  DockingHttpModule(const DockingHttpModule &) = delete;
  DockingHttpModule & operator=(const DockingHttpModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::docking
