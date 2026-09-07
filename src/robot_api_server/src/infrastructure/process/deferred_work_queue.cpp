#include "robot_api_server/infrastructure/process/deferred_work_queue.hpp"

#include <exception>
#include <iostream>
#include <utility>

namespace robot_api_server
{

DeferredWorkQueue::DeferredWorkQueue()
: worker_([this]() {run();})
{
}

DeferredWorkQueue::~DeferredWorkQueue()
{
  shutdown();
}

bool DeferredWorkQueue::post(std::function<void()> work)
{
  if (!work) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return false;
    }
    work_.push_back(std::move(work));
  }
  condition_.notify_one();
  return true;
}

void DeferredWorkQueue::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void DeferredWorkQueue::run()
{
  while (true) {
    std::function<void()> next;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {return stopping_ || !work_.empty();});
      if (stopping_ && work_.empty()) {
        return;
      }
      next = std::move(work_.front());
      work_.pop_front();
    }
    try {
      next();
    } catch (const std::exception & error) {
      std::cerr << "deferred work failed: " << error.what() << std::endl;
    } catch (...) {
      std::cerr << "deferred work failed with an unknown exception" << std::endl;
    }
  }
}

}  // namespace robot_api_server
