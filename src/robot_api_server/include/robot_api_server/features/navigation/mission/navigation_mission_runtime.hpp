#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "robot_api_server/features/navigation/mission/navigation_goal_job.hpp"

namespace robot_api_server::features::navigation
{

using NavigationMissionWorker = std::function<void(std::uint64_t)>;

// Owns ordinary-navigation job identity, mutable progress, cancellation and
// worker lifetime. Mission business code can update one matching job through
// the guarded mutation seam without owning its mutex or thread.
class NavigationMissionRuntime
{
public:
  NavigationMissionRuntime();
  ~NavigationMissionRuntime();

  NavigationMissionRuntime(const NavigationMissionRuntime &) = delete;
  NavigationMissionRuntime & operator=(const NavigationMissionRuntime &) = delete;

  std::uint64_t start(
    NavigationGoalJobStartSpec start_spec,
    NavigationMissionWorker worker);
  void shutdown();
  void join();

  bool running() const;
  std::string json() const;
  NavigationGoalJob snapshot() const;
  std::optional<NavigationGoalJob> snapshot(std::uint64_t job_id) const;

  bool update(
    std::uint64_t job_id,
    const std::function<void(NavigationGoalJob &)> & mutation);
  bool update_running(
    std::uint64_t job_id,
    const std::function<void(NavigationGoalJob &)> & mutation);
  bool request_cancel(const std::string & reason);
  bool cancel_requested(std::uint64_t job_id, std::string & detail) const;
  bool finish(
    std::uint64_t job_id,
    const NavigationGoalJobFinishSpec & finish_spec);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
