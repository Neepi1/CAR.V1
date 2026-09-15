#include <algorithm>
#include <chrono>
#include <iostream>
#include <ctime>
#include "imu_rotation_reference.hpp"
#include "robot_local_state/imu_rotation_cache.hpp"

// No ROS initialization, network, hardware or wall-time pass threshold.
// This measures only arithmetic; it is not a whole-node CPU claim.
volatile double imu_benchmark_sink = 0.0;
template<class Operation>
double measure(Operation operation, int scenario)
{
  sensor_msgs::msg::Imu input;
  input.angular_velocity_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
  input.linear_acceleration_covariance = {0.04,0,0,0,0.04,0,0,0,0.04};
  input.linear_acceleration.z = 9.81;
  geometry_msgs::msg::Quaternion q;
  q.x = 0.1; q.y = 0.2; q.z = -0.3; q.w = 0.9;
  timespec start{}, end{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  constexpr int count = 500000;
  for (int i = 0; i < count; ++i) {
    input.angular_velocity.x = i * 1.0e-6;
    if (scenario == 1) {input.angular_velocity_covariance[0] = 0.01 + (i % 2) * 0.001;}
    if (scenario == 2) {q.z = -0.3 + (i % 2) * 0.001;}
    // Each real callback receives externally supplied data. Prevent the benchmark
    // compiler from folding invariant inputs across callback boundaries.
#if defined(__GNUC__)
    asm volatile("" : "+m"(input), "+m"(q) : : "memory");
#endif
    auto output = input;
    operation(output, q);
#if defined(__GNUC__)
    // Observe the full result, not only the checksum fields below, so neither
    // implementation can drop unused covariance/vector calculations.
    asm volatile("" : : "m"(output) : "memory");
#endif
    imu_benchmark_sink = output.angular_velocity.x + output.angular_velocity_covariance[0] +
      output.linear_acceleration_covariance[8];
  }
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
  return ((end.tv_sec - start.tv_sec) * 1.0e9 + end.tv_nsec - start.tv_nsec) / count;
}
int main()
{
  for (int scenario : {0, 1, 2}) {
    for (int repeat = 0; repeat < 3; ++repeat) {
      robot_local_state::ImuRotationCache cache;
      const auto legacy = measure(imu_rotation_reference::apply, scenario);
      const auto candidate = measure([&](auto & imu, const auto & q) {cache.apply(imu, q);}, scenario);
      std::cout << "scenario=" << scenario << " legacy_ns=" << legacy <<
        " candidate_ns=" << candidate << " ratio=" << candidate / legacy << '\n';
    }
  }
}
