#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"

namespace robot_api_server::features::docking
{

struct DockingJobStorePorts
{
  std::function<void(
      bool,
      const std::string &,
      const std::string &,
      const std::string &)>
  update_dock_contact_latch;
  std::function<void(const std::string &, const std::string &)> finish_runtime;
  std::function<std::string()> timestamp_now;
  std::function<bool(
      std::uint64_t,
      bool,
      const std::string &,
      std::string &)>
  set_global_correction_paused;
};

// The one canonical App-visible docking task store. HTTP admission, status
// callbacks and the executor share this object; no module owns a copied job.
// Methods ending in `_locked` require mutex() to be held by the caller.
class DockingJobStore
{
public:
  explicit DockingJobStore(DockingJobStorePorts ports);

  DockingJobStore(const DockingJobStore &) = delete;
  DockingJobStore & operator=(const DockingJobStore &) = delete;

  std::mutex & mutex();
  DockingJob & job_unsafe();
  const DockingJob & job_unsafe() const;
  std::uint64_t & sequence_unsafe();

  DockingJob snapshot() const;
  std::string job_json_locked() const;
  std::string post_undock_settle_json_locked() const;

  void set_phase(std::uint64_t job_id, const std::string & phase);
  bool cancel_requested(std::uint64_t job_id) const;
  void mark_navigation_goal_sent(std::uint64_t job_id);

  // Returns whether the caller must release the correction pause after
  // leaving the critical section.
  bool finish_locked(
    bool ok,
    const std::string & final_state,
    const std::string & detail);
  void finish(
    std::uint64_t job_id,
    bool ok,
    const std::string & final_state,
    const std::string & detail);
  void finish_with_code(
    std::uint64_t job_id,
    const std::string & code,
    const std::string & detail);

private:
  DockingJobStorePorts ports_;
  mutable std::mutex mutex_;
  DockingJob job_;
  std::uint64_t sequence_{0U};
};

}  // namespace robot_api_server::features::docking
