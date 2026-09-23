#include "ranger_base/navlite_chassis_trace.hpp"

#include <cassert>
#include <ctime>
#include <iostream>

using westonrobot::NavliteChassisTrace;

int main() {
  NavliteChassisTrace trace;
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::time_point(std::chrono::seconds(100));
  assert(trace.Event(start).find("input_known=0") != std::string::npos);
  assert(trace.Event(start + std::chrono::milliseconds(20)).empty());
  trace.Command(0.3, 0, 0, start);
  trace.Submitted(0.3, 0.1, 0, "motion_command", start);
  auto event = trace.Event(start);
  assert(event.find("input_vx=0.300000") != std::string::npos);
  assert(event.find("sdk_linear=0.300000") != std::string::npos);
  assert(event.find("can_tx_confirmed=unknown") != std::string::npos);
  assert(trace.Event(start + std::chrono::milliseconds(30)).empty());
  // Tiny variations cannot turn normal output into a 50 Hz log stream.
  for (int i = 1; i < 50; ++i) {
    trace.Command(0.3 + (i % 2 ? 1e-5 : -1e-5), 0, 0,
                  start + std::chrono::milliseconds(i * 10));
    trace.Submitted(0.3, 0.1, 0, "motion_command", start);
    assert(trace.Event(start + std::chrono::milliseconds(i * 10)).empty());
  }
  trace.Command(0, 0, 0, start + std::chrono::milliseconds(500));
  trace.Submitted(0, 0, 0, "motion_command", start);
  event = trace.Event(start + std::chrono::milliseconds(500));
  assert(event.find("input_state=near_zero") != std::string::npos);
  assert(event.find("sdk_state=near_zero") != std::string::npos);
  assert(event.find("input_exact_zero=1") != std::string::npos);
  // A gap is not a zero command and does not submit a command.
  event = trace.Event(start + std::chrono::milliseconds(1100));
  assert(event.find("input_gap=1") != std::string::npos);
  assert(event.find("sdk_submit_seq=51") != std::string::npos);
  trace.Command(0.2, 0, 0, start + std::chrono::milliseconds(1200));
  trace.Submitted(0, 0, 0, "mode_switch_hold", start);
  trace.mode_state = "waiting_ack";
  trace.mode_changing = true;
  event = trace.Event(start + std::chrono::milliseconds(1200));
  assert(event.find("reason=mode_switch_hold") != std::string::npos);
  assert(event.find("mode_changing=1") != std::string::npos);
  assert(event.find("input_state=nonzero") != std::string::npos);
  assert(event.find("sdk_state=near_zero") != std::string::npos);
  // Group receipt time is not a per-motion CAN timestamp/sequence.
  trace.Feedback(0.2, 0, 0.1, {{1, 2, 3, 4}}, {{.1, .2, .3, .4}},
                 start, start, start + std::chrono::milliseconds(1300));
  trace.Odometry(.198, 0, 0, 123456, start);
  event = trace.Event(start + std::chrono::milliseconds(2300));
  assert(event.find("motion_can_age_sec=unknown") != std::string::npos);
  assert(event.find("motion_can_seq=unknown") != std::string::npos);
  assert(event.find("core_group_age_sec=2.300000") != std::string::npos);
  assert(event.find("odom_stamp_kind=driver_publish_time") != std::string::npos);
  assert(event.find("odom_vx=0.198000") != std::string::npos);
  trace.Command(-.2, 0, 0, start + std::chrono::milliseconds(2400));
  trace.Submitted(-.2, 0, 0, "motion_command", start);
  event = trace.Event(start + std::chrono::milliseconds(2400));
  assert(event.find("sdk_linear=-0.200000") != std::string::npos);
  std::cout << "navlite chassis trace tests passed\n";
  // Formatting included, ROS/file logging excluded. Simulates 50 Hz input;
  // establishes a bounded retained count, not a real-vehicle CPU guarantee.
  NavliteChassisTrace bench;
  bench.mode_state = "stable";
  const auto cpu_start = std::clock();
  std::size_t retained = 0;
  constexpr int samples = 50000;
  for (int i = 0; i < samples; ++i) {
    const auto tick = start + std::chrono::milliseconds(i * 20);
    bench.Command(.3, 0, 0, tick);
    bench.Submitted(.3, .1, 0, "motion_command", tick);
    if (!bench.Event(tick).empty()) ++retained;
  }
  const double cpu_sec = double(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  assert(retained == 1000);
  std::cout << "trace_benchmark observations=" << samples << " retained=" << retained
            << " simulated_seconds=1000 cpu_sec=" << cpu_sec
            << " cpu_us_per_observation=" << cpu_sec * 1e6 / samples << '\n';
}
