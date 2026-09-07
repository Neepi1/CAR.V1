#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/logger.hpp"

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_executor.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"

namespace robot_api_server::features::docking
{

struct DockingStatusConfig
{
  bool relocalize_after_fine_docking{false};
  bool relocalize_after_fine_docking_required{false};
  bool undock_relocalize_after_success{true};
  double docking_relocalize_wait_sec{15.0};
  double undock_relocalize_wait_sec{15.0};
  std::string observation_backend{"gs2_scan"};
};

struct DockingLocalizationVisibility
{
  bool amcl_ready{false};
  bool localization_degraded{false};
  bool using_triggered_baseline_only{false};
};

struct DockingStatusPorts
{
  std::function<void(
      bool,
      const std::string &,
      const std::string &,
      const std::string &)>
  update_dock_contact_latch;
  // Called with docking_job_mutex held; returns whether correction pause must
  // be released after leaving the critical section.
  std::function<bool(bool, const std::string &, const std::string &)>
  finish_docking_job_locked;
  std::function<bool(
      std::uint64_t,
      bool,
      const std::string &,
      std::string &)>
  set_global_correction_paused;
  std::function<bool(
      const std::string &,
      std::string &,
      double,
      std::uint64_t *)>
  trigger_localization_and_wait_for_result;
  std::function<DockingRelocalizationSettleResult(
      std::uint64_t,
      const std::string &,
      const std::string &,
      const std::function<bool(std::string &)> &)>
  wait_for_relocalization_settle;
  std::function<bool(const std::string &, std::string &)>
  release_stale_docking_fine_pause;
  std::function<DockingLocalizationVisibility()> localization_visibility;
  std::function<bool(std::function<void()>)> post_deferred_work;
  std::function<void(DockingJob &, const std::string &)>
  record_undock_status_observation;
};

// Owns the complete /docking/status lifecycle transaction, including task
// terminalization, deferred correction-pause release, and post-fine/post-
// undock relocalization workers. It shares the one canonical DockingJob store
// while that store is migrated out of the composition root.
class DockingStatusModule
{
public:
  DockingStatusModule(
    rclcpp::Logger logger,
    application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
    DockingJobStore & job_store,
    DockingStatusConfig config,
    DockingStatusPorts ports);
  ~DockingStatusModule();

  DockingStatusModule(const DockingStatusModule &) = delete;
  DockingStatusModule & operator=(const DockingStatusModule &) = delete;

  void handle_status(const std::string & status);
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::docking
