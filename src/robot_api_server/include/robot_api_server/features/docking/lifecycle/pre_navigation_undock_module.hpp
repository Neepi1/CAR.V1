#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"

namespace robot_api_server::features::docking
{

struct PreNavigationUndockRequest
{
  bool auto_undock_required{false};
  bool pre_navigation_blocked{false};
  bool docking_active_not_docked_block{false};
  bool runtime_state_undocking{false};
  bool docking_status_indicates_undocking{false};
  bool charging_contact_at_gate{false};
  std::string runtime_docking_state;
  std::string auto_undock_reason{"not_docked"};
  std::string recovery_action{"NONE"};
  std::string pre_navigation_block_reason;
  std::string resolved_dock_id;
  std::string reconcile_evidence;
};

struct PreNavigationUndockServiceObservation
{
  bool service_called{false};
  bool service_success{false};
  std::string message;
};

struct PreNavigationUndockConfig
{
  bool relocalize_after_success{true};
  double relocalize_wait_sec{8.0};
  double auto_undock_timeout_sec{28.0};
  std::chrono::milliseconds poll_period{100};
};

struct PreNavigationUndockPorts
{
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_zero_motion;
  // Called with docking_start_mutex held. It cancels and terminalizes any
  // non-undock docking owner before controlled undock is submitted.
  std::function<bool(std::string &)> prepare_controlled_undock;
  std::function<bool(const std::string &, std::string &)> release_stale_fine_pause;
  std::function<void()> join_docking_worker;
  std::function<application::runtime_mode::RuntimeModeSnapshot()> runtime_snapshot;
  std::function<bool(const std::string &, const std::string &, std::string &)>
  reconcile_stale_interlock;
  std::function<bool(std::string &)> ensure_manager_running;
  std::function<bool(
      std::string &,
      bool,
      PreNavigationUndockServiceObservation *)>
  call_undock_with_charging_retry;
  std::function<void(::robot_api_server::DockingJob &, const std::string &)>
  observe_undock_status;
  std::function<void(bool, const std::string &, const std::string &)>
  set_docking_runtime_state;
  std::function<void(const std::string &)> set_docking_identity;
  std::function<std::string()> timestamp_now;
  std::function<std::chrono::steady_clock::time_point()> monotonic_now;
  std::function<void(std::chrono::milliseconds)> sleep_for;
};

// Owns the complete navigation-before-undock transaction. The caller supplies
// an already classified dock-occupancy decision and receives one synchronous
// admission result: Nav2 may only start after undock (and configured
// post-undock localization readiness) is proven.
class PreNavigationUndockModule
{
public:
  PreNavigationUndockModule(
    std::mutex & docking_start_mutex,
    DockingJobStore & docking_job_store,
    PreNavigationUndockConfig config,
    PreNavigationUndockPorts ports);

  bool run_if_needed(
    const PreNavigationUndockRequest & request,
    std::string & detail,
    bool & undock_performed);

private:
  bool start(
    std::string & detail,
    bool charging_contact_at_gate,
    const std::string & resolved_dock_id);
  bool wait_for_completion(std::string & detail);

  std::mutex & docking_start_mutex_;
  DockingJobStore & docking_job_store_;
  PreNavigationUndockConfig config_;
  PreNavigationUndockPorts ports_;
};

}  // namespace robot_api_server::features::docking
