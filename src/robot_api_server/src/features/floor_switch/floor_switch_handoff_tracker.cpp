#include "robot_api_server/features/floor_switch/floor_switch_handoff_tracker.hpp"

#include <utility>

namespace robot_api_server
{

void FloorSwitchHandoffTracker::reset(
  const std::string & transaction_id,
  const std::uint64_t submission_generation)
{
  state_ = {};
  state_.active = true;
  state_.transaction_id = transaction_id;
  state_.submission_generation = submission_generation;
}

void FloorSwitchHandoffTracker::clear()
{
  state_ = {};
}

bool FloorSwitchHandoffTracker::observe_feedback(
  const std::string & transaction_id,
  const std::uint64_t submission_generation,
  const std::uint64_t stage_sequence,
  const bool caller_pause_handoff_ready)
{
  if (
    !state_.active || state_.terminal ||
    transaction_id != state_.transaction_id ||
    submission_generation != state_.submission_generation ||
    stage_sequence < state_.accepted_stage_sequence)
  {
    return false;
  }
  state_.accepted_stage_sequence = stage_sequence;
  state_.ready = state_.ready || caller_pause_handoff_ready;
  return true;
}

bool FloorSwitchHandoffTracker::observe_terminal(
  const std::string & transaction_id,
  const std::uint64_t submission_generation)
{
  if (
    !state_.active || transaction_id != state_.transaction_id ||
    submission_generation != state_.submission_generation)
  {
    return false;
  }
  state_.terminal = true;
  state_.terminal_before_ready = !state_.ready;
  return true;
}

FloorSwitchHandoffSnapshot FloorSwitchHandoffTracker::snapshot() const
{
  return state_;
}

}  // namespace robot_api_server
