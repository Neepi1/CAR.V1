#pragma once

#include <cstdint>

namespace robot_localization_bridge
{

enum class PostIsaacRefineDisposition
{
  kNotApplicable,
  kWaitingForIsaacSettle,
  kWaitingForMinimumDelay,
  kPreseedPose,
  kAbandonBecauseMoving,
  kEligible,
};

struct PostIsaacRefineGateInput
{
  bool enabled{false};
  bool gated_mode{false};
  bool explicit_target_settled{false};
  bool require_stationary{true};
  bool robot_moving{false};
  std::uint64_t explicit_sequence{0U};
  std::uint64_t refined_sequence{0U};
  std::uint64_t abandoned_sequence{0U};
  double now_sec{0.0};
  double refine_reference_sec{0.0};
  double pose_stamp_sec{0.0};
  double received_sec{0.0};
  double window_sec{0.0};
  double min_delay_sec{0.0};
  double min_pose_stamp_delta_sec{0.0};
};

struct PostIsaacNomotionRequestGateInput
{
  bool enabled{false};
  bool refine_pending{false};
  bool explicit_target_settled{false};
  bool require_stationary{true};
  bool robot_moving{false};
  bool service_ready{false};
  int request_count{0};
  int max_requests{0};
  double now_sec{0.0};
  double seed_sec{0.0};
  double last_request_sec{0.0};
  double min_delay_sec{0.0};
  double request_period_sec{0.0};
};

inline PostIsaacRefineDisposition evaluate_post_isaac_refine_gate(
  const PostIsaacRefineGateInput & input)
{
  if (
    !input.enabled || !input.gated_mode || input.explicit_sequence == 0U ||
    input.refined_sequence == input.explicit_sequence ||
    input.abandoned_sequence == input.explicit_sequence ||
    input.refine_reference_sec <= 0.0)
  {
    return PostIsaacRefineDisposition::kNotApplicable;
  }

  const double elapsed_sec = input.now_sec - input.refine_reference_sec;
  if (elapsed_sec < 0.0 || elapsed_sec > input.window_sec) {
    return PostIsaacRefineDisposition::kNotApplicable;
  }
  if (!input.explicit_target_settled) {
    return PostIsaacRefineDisposition::kWaitingForIsaacSettle;
  }
  if (input.require_stationary && input.robot_moving) {
    return PostIsaacRefineDisposition::kAbandonBecauseMoving;
  }
  if (elapsed_sec < input.min_delay_sec) {
    return PostIsaacRefineDisposition::kWaitingForMinimumDelay;
  }
  if (
    input.received_sec < input.refine_reference_sec ||
    input.pose_stamp_sec <=
    input.refine_reference_sec + input.min_pose_stamp_delta_sec)
  {
    return PostIsaacRefineDisposition::kPreseedPose;
  }
  return PostIsaacRefineDisposition::kEligible;
}

inline bool should_request_post_isaac_nomotion_update(
  const PostIsaacNomotionRequestGateInput & input)
{
  if (
    !input.enabled || !input.refine_pending || !input.explicit_target_settled ||
    !input.service_ready || input.seed_sec <= 0.0 ||
    input.request_count >= input.max_requests)
  {
    return false;
  }
  if (input.require_stationary && input.robot_moving) {
    return false;
  }
  if (input.now_sec - input.seed_sec < input.min_delay_sec) {
    return false;
  }
  return input.last_request_sec <= 0.0 ||
         input.now_sec - input.last_request_sec >= input.request_period_sec;
}

}  // namespace robot_localization_bridge
