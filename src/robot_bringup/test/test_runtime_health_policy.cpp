#include "robot_bringup/runtime_health_policy.hpp"
#include <cassert>
#include <iostream>

using robot_bringup::health::OdomWatch;

int main() {
  OdomWatch watch;
  watch.reset(0.0);
  assert(!watch.fault(2.99));
  assert(watch.fault(3.0));
  assert(watch.observe(3.0, 100.0, 99.98));
  assert(!watch.fault(5.99));
  assert(watch.fault(6.0));
  assert(!watch.observe(4.0, 101.0, 99.98));  // replay cannot extend grace
  assert(watch.fault(6.0));
  assert(!watch.observe(7.0, 104.0, 90.0));
  assert(!watch.observe(7.0, 104.0, 105.0));
  assert(watch.observe(7.0, 104.0, 103.98));
  assert(!watch.fault(7.0));
  for (int i = 1; i <= 20; ++i) {
    assert(watch.observe(7.0 + i, 104.0 + i, 103.98 + i));
    assert(!watch.fault(7.0 + i));
  }
  watch.reset(28.0);  // observer clock epoch changed; do not blame the producer
  assert(!watch.fault(28.0));
  assert(watch.observe(29.0, 50.0, 49.98));
  std::cout << "PASS: 1Hz healthy input, 3s grace, replay, stale/future stamps, recovery\n";
}
