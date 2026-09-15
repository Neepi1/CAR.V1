#include "robot_nav_config/ordinary_local_path_repair_worker.hpp"

#include <utility>

namespace robot_nav_config
{

OrdinaryLocalPathRepairWorker::OrdinaryLocalPathRepairWorker(
  nav2_util::LifecycleNode::SharedPtr node,
  RepairExecutor executor)
: node_(std::move(node)), executor_(std::move(executor))
{
  if (!executor_) {
    executor_ = [this](const OrdinaryLocalPathRepairRequest & request) {
        return repair_ordinary_local_path(request, node_);
      };
  }
}

OrdinaryLocalPathRepairWorker::~OrdinaryLocalPathRepairWorker()
{
  stop();
}

void OrdinaryLocalPathRepairWorker::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_.joinable()) {
    return;
  }
  stop_requested_ = false;
  running_ = false;
  pending_work_.reset();
  completed_result_.reset();
  thread_ = std::thread(&OrdinaryLocalPathRepairWorker::run, this);
}

void OrdinaryLocalPathRepairWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!thread_.joinable()) {
      stop_requested_ = false;
      running_ = false;
      pending_work_.reset();
      completed_result_.reset();
      return;
    }
    stop_requested_ = true;
    pending_work_.reset();
  }
  condition_.notify_all();
  thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  stop_requested_ = false;
  running_ = false;
  completed_result_.reset();
}

bool OrdinaryLocalPathRepairWorker::submit(OrdinaryLocalPathRepairWork work)
{
  if (!work.request.costmap || work.request.costmap_frame.empty()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!thread_.joinable() || stop_requested_ || running_ || pending_work_ ||
      completed_result_)
    {
      return false;
    }
    pending_work_ = std::move(work);
  }
  condition_.notify_one();
  return true;
}

std::optional<OrdinaryLocalPathRepairWorkResult>
OrdinaryLocalPathRepairWorker::take_result()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(completed_result_);
  completed_result_.reset();
  return result;
}

bool OrdinaryLocalPathRepairWorker::busy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_ || pending_work_.has_value();
}

void OrdinaryLocalPathRepairWorker::run()
{
  while (true) {
    OrdinaryLocalPathRepairWork work;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
          return stop_requested_ || pending_work_.has_value();
        });
      if (stop_requested_) {
        return;
      }
      work = std::move(*pending_work_);
      pending_work_.reset();
      running_ = true;
    }

    OrdinaryLocalPathRepairWorkResult result;
    result.plan_generation = work.plan_generation;
    result.request = std::move(work.request);
    try {
      result.repair = executor_(result.request);
    } catch (...) {
      result.error = std::current_exception();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      if (stop_requested_) {
        return;
      }
      completed_result_ = std::move(result);
    }
  }
}

}  // namespace robot_nav_config
