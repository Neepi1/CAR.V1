#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "robot_api_server/features/navigation/runtime/navigation_cancel_job_model.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationCancelStartSpec
{
  std::string reason;
  bool stop_stack{false};
  std::string started_at;
  bool zero_velocity_published{false};
};

struct NavigationCancelFinishSpec
{
  bool ok{false};
  bool action_available{false};
  bool active_goal_cancel_requested{false};
  bool cancel_all_requested{false};
  bool cancel_all_ok{false};
  bool stop_stack_ok{false};
  std::string detail;
  std::string cancel_all_detail;
  std::string stop_stack_detail;
  std::string finished_at;
};

using NavigationCancelWorker =
  std::function<void(std::uint64_t job_id, bool stop_stack)>;

// Owns the asynchronous cancel/stop job identity, progress and worker. The
// actual Nav2 cancellation proof and process stop remain in runtime adapters.
class NavigationCancelRuntime
{
public:
  NavigationCancelRuntime();
  ~NavigationCancelRuntime();

  NavigationCancelRuntime(const NavigationCancelRuntime &) = delete;
  NavigationCancelRuntime & operator=(const NavigationCancelRuntime &) = delete;

  std::uint64_t start(
    const NavigationCancelStartSpec & start_spec,
    NavigationCancelWorker worker);
  void join();

  bool running() const;
  std::string json() const;
  NavigationCancelJob snapshot() const;
  bool update_running(
    std::uint64_t job_id,
    const std::function<void(NavigationCancelJob &)> & mutation);
  bool finish(
    std::uint64_t job_id,
    const NavigationCancelFinishSpec & finish_spec);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
