#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_localization_bridge
{

enum class PauseOperation
{
  kAcquire,
  kRelease,
};

enum class PauseDecisionCode
{
  kOk,
  kInvalidRequest,
  kConflict,
  kNotOwner,
  kStaleCommand,
};

struct PauseCommand
{
  PauseOperation operation{PauseOperation::kAcquire};
  std::string owner;
  std::string transaction_id;
  std::string reason;
  std::uint64_t command_sequence{0U};
};

struct CorrectionPauseSnapshot
{
  std::uint64_t generation{0U};
  bool paused{false};
  std::vector<std::string> lease_keys;
  std::string transition_reason{"startup"};
};

struct PauseDecision
{
  bool accepted{false};
  bool changed{false};
  PauseDecisionCode code{PauseDecisionCode::kInvalidRequest};
  std::string message;
  std::uint64_t applied_sequence{0U};
  CorrectionPauseSnapshot state;
};

class CorrectionPauseArbiter
{
public:
  PauseDecision apply(const PauseCommand & command);
  CorrectionPauseSnapshot snapshot() const;

private:
  struct Record
  {
    std::string owner;
    std::string transaction_id;
    std::string reason;
  };

  std::vector<Record> records_;
  // Tombstones are intentionally retained after release so a delayed acquire
  // from the same owner/transaction cannot resurrect a pause lease.
  std::unordered_map<std::string, std::uint64_t> last_command_sequences_;
  std::uint64_t generation_{0U};
  std::string transition_reason_{"startup"};
};

}  // namespace robot_localization_bridge
