#include "robot_global_localization/post_reload_readiness_gate.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot_global_localization
{

PostReloadReadinessGate::PostReloadReadinessGate(
  PostReloadReadinessConfig config)
: config_(std::move(config))
{
  config_.minimum_settle_sec = std::max(0.0, config_.minimum_settle_sec);
  config_.timeout_sec = std::max(config_.minimum_settle_sec, config_.timeout_sec);
  config_.input_max_age_sec = std::max(0.0, config_.input_max_age_sec);
  config_.required_consecutive_service_ready_samples = std::max(
    std::size_t{1U}, config_.required_consecutive_service_ready_samples);
}

void PostReloadReadinessGate::arm(
  const std::uint64_t localizer_generation,
  const std::uint64_t localizer_input_sequence,
  const double steady_now_sec)
{
  pending_ = true;
  localizer_generation_ = localizer_generation;
  input_sequence_baseline_ = localizer_input_sequence;
  armed_steady_sec_ = steady_now_sec;
  consecutive_service_ready_samples_ = 0U;
}

PostReloadReadinessDecision PostReloadReadinessGate::observe(
  const PostReloadReadinessObservation & observation)
{
  if (!pending_) {
    return PostReloadReadinessDecision::kReady;
  }

  const bool time_valid =
    std::isfinite(observation.steady_now_sec) &&
    std::isfinite(armed_steady_sec_) &&
    observation.steady_now_sec >= armed_steady_sec_;
  if (!time_valid) {
    consecutive_service_ready_samples_ = 0U;
    return PostReloadReadinessDecision::kWait;
  }

  if (observation.trigger_service_ready) {
    ++consecutive_service_ready_samples_;
  } else {
    consecutive_service_ready_samples_ = 0U;
  }

  const double elapsed_sec = observation.steady_now_sec - armed_steady_sec_;
  const bool settled = elapsed_sec >= config_.minimum_settle_sec;
  const bool service_stable =
    consecutive_service_ready_samples_ >=
    config_.required_consecutive_service_ready_samples;
  const bool fresh_post_reload_input =
    observation.localizer_input_sequence > input_sequence_baseline_ &&
    std::isfinite(observation.localizer_input_age_sec) &&
    observation.localizer_input_age_sec >= 0.0 &&
    observation.localizer_input_age_sec <= config_.input_max_age_sec;

  if (settled && service_stable && fresh_post_reload_input) {
    pending_ = false;
    return PostReloadReadinessDecision::kReady;
  }
  if (elapsed_sec >= config_.timeout_sec) {
    return PostReloadReadinessDecision::kTimedOut;
  }
  return PostReloadReadinessDecision::kWait;
}

bool PostReloadReadinessGate::pending() const noexcept
{
  return pending_;
}

std::uint64_t PostReloadReadinessGate::localizer_generation() const noexcept
{
  return localizer_generation_;
}

}  // namespace robot_global_localization
