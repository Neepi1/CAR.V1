#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace robot_api_server
{

class DeferredWorkQueue
{
public:
  DeferredWorkQueue();
  ~DeferredWorkQueue();

  DeferredWorkQueue(const DeferredWorkQueue &) = delete;
  DeferredWorkQueue & operator=(const DeferredWorkQueue &) = delete;

  bool post(std::function<void()> work);
  void shutdown();

private:
  void run();

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::function<void()>> work_;
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace robot_api_server
