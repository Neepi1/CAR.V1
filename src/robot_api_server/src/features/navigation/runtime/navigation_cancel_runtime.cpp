#include "robot_api_server/features/navigation/runtime/navigation_cancel_runtime.hpp"

#include <stdexcept>
#include <mutex>
#include <thread>
#include <utility>

namespace robot_api_server::features::navigation
{

class NavigationCancelRuntime::Impl
{
public:
  ~Impl()
  {
    join();
  }

  std::uint64_t start(
    const NavigationCancelStartSpec & start_spec,
    NavigationCancelWorker worker)
  {
    if (!worker) {
      throw std::invalid_argument("navigation cancel worker is required");
    }
    join();
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (job_.state == "running") {
        throw std::logic_error("navigation cancel job already running");
      }
      job_id = ++sequence_;
      job_ = NavigationCancelJob{};
      job_.id = job_id;
      job_.state = "running";
      job_.phase = "accepted";
      job_.reason = start_spec.reason;
      job_.stop_stack = start_spec.stop_stack;
      job_.started_at = start_spec.started_at;
      job_.zero_velocity_published = start_spec.zero_velocity_published;
    }
    try {
      worker_ = std::thread(
        [worker = std::move(worker), job_id, stop_stack = start_spec.stop_stack]() mutable {
          worker(job_id, stop_stack);
        });
    } catch (...) {
      NavigationCancelFinishSpec finish_spec;
      finish_spec.ok = false;
      finish_spec.detail = "failed to start navigation cancel worker";
      finish_spec.finished_at = start_spec.started_at;
      (void)finish(job_id, finish_spec);
      throw;
    }
    return job_id;
  }

  void join()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool running() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return job_.state == "running";
  }

  std::string json() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return navigation_cancel_job_json(job_);
  }

  NavigationCancelJob snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return job_;
  }

  bool update_running(
    const std::uint64_t job_id,
    const std::function<void(NavigationCancelJob &)> & mutation)
  {
    if (!mutation) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (job_.id != job_id || job_.state != "running") {
      return false;
    }
    mutation(job_);
    return true;
  }

  bool finish(
    const std::uint64_t job_id,
    const NavigationCancelFinishSpec & finish_spec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job_.id != job_id) {
      return false;
    }
    job_.state = finish_spec.ok ? "succeeded" : "failed";
    job_.phase = "finished";
    job_.ok = finish_spec.ok;
    job_.action_available = finish_spec.action_available;
    job_.active_goal_cancel_requested =
      finish_spec.active_goal_cancel_requested;
    job_.cancel_all_requested = finish_spec.cancel_all_requested;
    job_.cancel_all_ok = finish_spec.cancel_all_ok;
    job_.stop_stack_ok = finish_spec.stop_stack_ok;
    job_.detail = finish_spec.detail;
    job_.cancel_all_detail = finish_spec.cancel_all_detail;
    job_.stop_stack_detail = finish_spec.stop_stack_detail;
    job_.finished_at = finish_spec.finished_at;
    return true;
  }

private:
  mutable std::mutex mutex_;
  NavigationCancelJob job_;
  std::uint64_t sequence_{0U};
  std::thread worker_;
};

NavigationCancelRuntime::NavigationCancelRuntime()
: impl_(std::make_unique<Impl>())
{
}

NavigationCancelRuntime::~NavigationCancelRuntime() = default;

std::uint64_t NavigationCancelRuntime::start(
  const NavigationCancelStartSpec & start_spec,
  NavigationCancelWorker worker)
{
  return impl_->start(start_spec, std::move(worker));
}

void NavigationCancelRuntime::join()
{
  impl_->join();
}

bool NavigationCancelRuntime::running() const
{
  return impl_->running();
}

std::string NavigationCancelRuntime::json() const
{
  return impl_->json();
}

NavigationCancelJob NavigationCancelRuntime::snapshot() const
{
  return impl_->snapshot();
}

bool NavigationCancelRuntime::update_running(
  const std::uint64_t job_id,
  const std::function<void(NavigationCancelJob &)> & mutation)
{
  return impl_->update_running(job_id, mutation);
}

bool NavigationCancelRuntime::finish(
  const std::uint64_t job_id,
  const NavigationCancelFinishSpec & finish_spec)
{
  return impl_->finish(job_id, finish_spec);
}

}  // namespace robot_api_server::features::navigation
