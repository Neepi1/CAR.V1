#include "robot_nav_config/elevator_scoped_replan_worker.hpp"

#include <utility>

namespace robot_nav_config {

ElevatorScopedReplanWorker::ElevatorScopedReplanWorker(SearchExecutor executor)
    : executor_(std::move(executor)) {
  if (!executor_) {
    executor_ = [](const ElevatorScopedReplanRequest &request) {
      if (!request.costmap) {
        return ElevatorScopedSearchResult{};
      }
      // Both localization changes and live obstacle invalidation rebuild the
      // complete remaining route from the current pose to the canonical
      // final goal.  The result is a replaceable route revision, not a
      // splice that must return to an already-invalidated historical path.
      return search_elevator_scoped_path(*request.costmap, request.footprint,
                                         request.start, request.goal,
                                         request.parameters);
    };
  }
}

ElevatorScopedReplanWorker::~ElevatorScopedReplanWorker() { stop(); }

void ElevatorScopedReplanWorker::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (thread_.joinable()) {
    return;
  }
  stop_requested_ = false;
  running_ = false;
  pending_request_.reset();
  completed_result_.reset();
  thread_ = std::thread(&ElevatorScopedReplanWorker::run, this);
}

void ElevatorScopedReplanWorker::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!thread_.joinable()) {
      stop_requested_ = false;
      running_ = false;
      pending_request_.reset();
      completed_result_.reset();
      return;
    }
    stop_requested_ = true;
    pending_request_.reset();
  }
  condition_.notify_all();
  thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  stop_requested_ = false;
  running_ = false;
  completed_result_.reset();
}

bool ElevatorScopedReplanWorker::submit(ElevatorScopedReplanRequest request) {
  if (!request.costmap || request.costmap_frame.empty()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!thread_.joinable() || stop_requested_ || running_ ||
        pending_request_ || completed_result_) {
      return false;
    }
    pending_request_ = std::move(request);
  }
  condition_.notify_one();
  return true;
}

std::optional<ElevatorScopedReplanResult>
ElevatorScopedReplanWorker::take_result() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(completed_result_);
  completed_result_.reset();
  return result;
}

bool ElevatorScopedReplanWorker::busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_ || pending_request_.has_value();
}

void ElevatorScopedReplanWorker::run() {
  while (true) {
    ElevatorScopedReplanRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
        return stop_requested_ || pending_request_.has_value();
      });
      if (stop_requested_) {
        return;
      }
      request = std::move(*pending_request_);
      pending_request_.reset();
      running_ = true;
    }

    ElevatorScopedReplanResult result;
    result.plan_generation = request.plan_generation;
    result.request_signature = request.request_signature;
    result.costmap_frame = request.costmap_frame;
    result.goal = request.goal;
    try {
      result.search = executor_(request);
    } catch (...) {
      result.search = ElevatorScopedSearchResult{};
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

} // namespace robot_nav_config
