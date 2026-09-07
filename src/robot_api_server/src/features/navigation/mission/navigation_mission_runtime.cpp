#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"

#include <stdexcept>
#include <utility>

namespace robot_api_server::features::navigation
{

class NavigationMissionRuntime::Impl
{
public:
  ~Impl()
  {
    shutdown();
  }

  std::uint64_t start(
    NavigationGoalJobStartSpec start_spec,
    NavigationMissionWorker worker)
  {
    if (!worker) {
      throw std::invalid_argument("navigation mission worker is required");
    }
    join_worker();

    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      if (job_.state == "running") {
        throw std::logic_error("navigation mission already running");
      }
      job_id = ++job_sequence_;
      start_spec.id = job_id;
      job_ = make_navigation_goal_job(start_spec);
    }
    try {
      worker_ = std::thread(
        [worker = std::move(worker), job_id]() mutable {
          worker(job_id);
        });
    } catch (...) {
      NavigationGoalJobFinishSpec finish;
      finish.succeeded = false;
      finish.phase = "worker_start_failed";
      finish.detail = "failed to start navigation mission worker";
      finish.completed_at = start_spec.started_at;
      (void)this->finish(job_id, finish);
      throw;
    }
    return job_id;
  }

  void shutdown()
  {
    request_cancel("navigation mission runtime shutting down");
    join_worker();
  }

  void join()
  {
    join_worker();
  }

  bool running() const
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    return job_.state == "running";
  }

  std::string json() const
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    return navigation_goal_job_json(job_);
  }

  NavigationGoalJob snapshot() const
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    return job_;
  }

  std::optional<NavigationGoalJob> snapshot(const std::uint64_t job_id) const
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.id != job_id) {
      return std::nullopt;
    }
    return job_;
  }

  bool update(
    const std::uint64_t job_id,
    const std::function<void(NavigationGoalJob &)> & mutation)
  {
    if (!mutation) {
      return false;
    }
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.id != job_id) {
      return false;
    }
    mutation(job_);
    return true;
  }

  bool update_running(
    const std::uint64_t job_id,
    const std::function<void(NavigationGoalJob &)> & mutation)
  {
    if (!mutation) {
      return false;
    }
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.id != job_id || job_.state != "running") {
      return false;
    }
    mutation(job_);
    return true;
  }

  bool request_cancel(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.state != "running") {
      return false;
    }
    job_.cancel_requested = true;
    job_.cancel_reason = reason;
    return true;
  }

  bool cancel_requested(
    const std::uint64_t job_id,
    std::string & detail) const
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.id != job_id) {
      detail = "navigation goal job was superseded";
      return true;
    }
    if (!job_.cancel_requested) {
      return false;
    }
    detail = job_.cancel_reason.empty() ?
      "navigation goal cancel requested" : job_.cancel_reason;
    return true;
  }

  bool finish(
    const std::uint64_t job_id,
    const NavigationGoalJobFinishSpec & finish_spec)
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (job_.id != job_id) {
      return false;
    }
    apply_navigation_goal_job_finish(job_, finish_spec);
    return true;
  }

private:
  void join_worker()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  mutable std::mutex job_mutex_;
  NavigationGoalJob job_;
  std::uint64_t job_sequence_{0U};
  std::thread worker_;
};

NavigationMissionRuntime::NavigationMissionRuntime()
: impl_(std::make_unique<Impl>())
{
}

NavigationMissionRuntime::~NavigationMissionRuntime() = default;

std::uint64_t NavigationMissionRuntime::start(
  NavigationGoalJobStartSpec start_spec,
  NavigationMissionWorker worker)
{
  return impl_->start(std::move(start_spec), std::move(worker));
}

void NavigationMissionRuntime::shutdown()
{
  impl_->shutdown();
}

void NavigationMissionRuntime::join()
{
  impl_->join();
}

bool NavigationMissionRuntime::running() const
{
  return impl_->running();
}

std::string NavigationMissionRuntime::json() const
{
  return impl_->json();
}

NavigationGoalJob NavigationMissionRuntime::snapshot() const
{
  return impl_->snapshot();
}

std::optional<NavigationGoalJob> NavigationMissionRuntime::snapshot(
  const std::uint64_t job_id) const
{
  return impl_->snapshot(job_id);
}

bool NavigationMissionRuntime::update(
  const std::uint64_t job_id,
  const std::function<void(NavigationGoalJob &)> & mutation)
{
  return impl_->update(job_id, mutation);
}

bool NavigationMissionRuntime::update_running(
  const std::uint64_t job_id,
  const std::function<void(NavigationGoalJob &)> & mutation)
{
  return impl_->update_running(job_id, mutation);
}

bool NavigationMissionRuntime::request_cancel(const std::string & reason)
{
  return impl_->request_cancel(reason);
}

bool NavigationMissionRuntime::cancel_requested(
  const std::uint64_t job_id,
  std::string & detail) const
{
  return impl_->cancel_requested(job_id, detail);
}

bool NavigationMissionRuntime::finish(
  const std::uint64_t job_id,
  const NavigationGoalJobFinishSpec & finish_spec)
{
  return impl_->finish(job_id, finish_spec);
}

}  // namespace robot_api_server::features::navigation
