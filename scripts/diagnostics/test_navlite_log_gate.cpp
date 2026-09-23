// No ROS initialization or robot endpoints. Compile with both package include directories.
#include <cassert>
#include <chrono>
#include <iostream>
#include "robot_nav_config/navlite_log_gate.hpp"
#include "robot_safety/navlite_log_gate.hpp"

template<class Gate> void verify()
{
  Gate gate;
  const auto t = std::chrono::steady_clock::time_point{};
  assert(gate.observe("pass", false, t));
  assert(!gate.observe("pass", false, t + std::chrono::seconds(10)));
  assert(gate.observe("zero:reason_a", true, t + std::chrono::seconds(11)));
  for (int i = 1; i < 1000; ++i) {
    assert(!gate.observe("zero:reason_a", true,
      t + std::chrono::seconds(11) + std::chrono::milliseconds(i)));
  }
  assert(gate.observe("zero:reason_a", true, t + std::chrono::seconds(12)));
  assert(gate.observe("zero:reason_b", true, t + std::chrono::milliseconds(12001)));
  assert(gate.observe("pass", false, t + std::chrono::milliseconds(12002)));
  assert(!gate.observe("pass", false, t + std::chrono::seconds(20)));
  assert(gate.observe("suppressed_zero", true, t + std::chrono::seconds(21)));
  // Clock reversal cannot create a persistent abnormal busy log loop.
  assert(!gate.observe("suppressed_zero", true, t));
}
int main()
{
  verify<robot_nav_config::NavliteLogGate>();
  verify<robot_safety::NavliteLogGate>();
  std::cout << "PASS package-local gates: change, release, persistent <=1Hz, steady clock\n";
}
