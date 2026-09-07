#pragma once

#include <cstddef>
#include <cstdint>

namespace robot_global_localization
{

struct PostReloadReadinessConfig
{
  double minimum_settle_sec{1.0};
  double timeout_sec{8.0};
  double input_max_age_sec{0.5};
  std::size_t required_consecutive_service_ready_samples{3U};
};

struct PostReloadReadinessObservation
{
  double steady_now_sec{0.0};
  bool trigger_service_ready{false};
  std::uint64_t localizer_input_sequence{0U};
  double localizer_input_age_sec{-1.0};
};

enum class PostReloadReadinessDecision
{
  kWait,
  kReady,
  kTimedOut,
};

// Converts asynchronous component/service/input observations into one
// generation-fenced readiness decision. It performs no ROS calls and owns no
// timers, so both the runtime adapter and tests use the same ordering rules.
class PostReloadReadinessGate
{
public:
  explicit PostReloadReadinessGate(PostReloadReadinessConfig config = {});

  void arm(
    std::uint64_t localizer_generation,
    std::uint64_t localizer_input_sequence,
    double steady_now_sec);

  PostReloadReadinessDecision observe(
    const PostReloadReadinessObservation & observation);

  bool pending() const noexcept;
  std::uint64_t localizer_generation() const noexcept;

private:
  PostReloadReadinessConfig config_;
  bool pending_{false};
  std::uint64_t localizer_generation_{0U};
  std::uint64_t input_sequence_baseline_{0U};
  double armed_steady_sec_{0.0};
  std::size_t consecutive_service_ready_samples_{0U};
};

}  // namespace robot_global_localization
