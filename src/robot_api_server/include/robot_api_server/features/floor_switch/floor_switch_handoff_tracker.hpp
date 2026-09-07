#pragma once

#include <cstdint>
#include <string>

namespace robot_api_server
{

struct FloorSwitchHandoffSnapshot
{
  bool active{false};
  bool ready{false};
  bool terminal{false};
  bool terminal_before_ready{false};
  std::uint64_t submission_generation{0U};
  std::uint64_t accepted_stage_sequence{0U};
  std::string transaction_id;
};

// Tracks the caller/floor-manager correction-pause handoff for one exact
// FloorSwitch submission. The class deliberately has no ROS dependency so the
// ordering contract can be exercised deterministically.
class FloorSwitchHandoffTracker
{
public:
  void reset(
    const std::string & transaction_id,
    std::uint64_t submission_generation);
  void clear();

  bool observe_feedback(
    const std::string & transaction_id,
    std::uint64_t submission_generation,
    std::uint64_t stage_sequence,
    bool caller_pause_handoff_ready);
  bool observe_terminal(
    const std::string & transaction_id,
    std::uint64_t submission_generation);

  FloorSwitchHandoffSnapshot snapshot() const;

private:
  FloorSwitchHandoffSnapshot state_;
};

}  // namespace robot_api_server
