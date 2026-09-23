#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include "robot_nav_config/navlite_failure_trace.hpp"

int main()
{
  using Trace = robot_nav_config::NavliteFailureTrace;
  const auto t = Trace::Clock::time_point{};
  Trace trace;
  assert(trace.success(false, t) == nullptr);
  assert(std::strcmp(trace.failure("raw error", t), "failure_start") == 0);
  for (int i = 1; i < 1000; ++i) {
    assert(trace.failure("raw error", t+std::chrono::milliseconds(i)) == nullptr);
  }
  assert(trace.failures() == 1000);
  assert(std::strcmp(trace.failure("raw error", t+std::chrono::seconds(1)),
    "failure_continues") == 0);
  assert(std::strcmp(trace.failure("different raw", t+std::chrono::milliseconds(1001)),
    "failure_reason_changed") == 0);
  assert(std::strcmp(trace.success(false, t+std::chrono::seconds(2)),
    "calculation_recovered_zero") == 0);
  assert(trace.failures() == 1002 && trace.elapsed() == 2.0);
  assert(trace.success(false, t+std::chrono::seconds(3)) == nullptr);
  assert(std::strcmp(trace.success(true, t+std::chrono::seconds(4)),
    "first_nonzero_after_calculation_recovery") == 0);
  assert(trace.success(true, t+std::chrono::seconds(5)) == nullptr);
  assert(std::strcmp(trace.failure("new", t+std::chrono::seconds(6)), "failure_start") == 0);
  assert(trace.episode() == 2 && trace.failures() == 1);
  trace.interrupt();
  assert(trace.success(true, t+std::chrono::seconds(7)) == nullptr);
  assert(robot_nav_config::navlite_quote("a\n\"\\b") == "\"a\\u000a\\\"\\\\b\"");
  std::cout << "PASS failure count/duration/rate/reason/zero recovery/nonzero recovery/interruption/raw\n";
}
