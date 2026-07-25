#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace robot_safety
{

enum class InterlockDecisionCode
{
  kOk,
  kInvalidRequest,
  kConflict,
  kNotOwner,
  kStaleLease,
};

enum class HoldOperation
{
  kAcquire,
  kRelease,
};

enum class ExecutionOperation
{
  kSet,
  kRelease,
};

struct HoldCommand
{
  HoldOperation operation{HoldOperation::kAcquire};
  std::string owner;
  std::string transaction_id;
  std::string reason;
};

struct ExecutionCommand
{
  ExecutionOperation operation{ExecutionOperation::kSet};
  std::string owner;
  std::string mission_id;
  std::string transaction_id;
  std::string lease_id;
  double lease_duration_sec{0.0};
  bool recovery{false};
};

struct MotionInterlockSnapshot
{
  std::uint64_t generation{0U};
  bool motion_blocked{false};
  std::vector<std::string> hold_keys;
  bool execution_session_engaged{false};
  bool execution_lease_active{false};
  std::string execution_owner;
  std::string execution_mission_id;
  std::string execution_transaction_id;
  std::string execution_lease_id;
  double execution_lease_remaining_sec{0.0};
  std::string transition_reason{"startup"};
};

struct InterlockDecision
{
  bool accepted{false};
  bool changed{false};
  InterlockDecisionCode code{InterlockDecisionCode::kInvalidRequest};
  std::string message;
  MotionInterlockSnapshot state;
};

class MotionInterlockArbiter
{
public:
  explicit MotionInterlockArbiter(
    double min_execution_lease_sec = 0.20,
    double max_execution_lease_sec = 5.0,
    std::string recovery_owner = "robot_mission_manager");

  InterlockDecision apply_hold(const HoldCommand & command, double now_sec);
  InterlockDecision apply_execution(const ExecutionCommand & command, double now_sec);
  std::optional<MotionInterlockSnapshot> expire(double now_sec);
  MotionInterlockSnapshot snapshot(double now_sec) const;
  bool motion_permitted(double now_sec) const;

private:
  struct HoldRecord
  {
    std::string owner;
    std::string transaction_id;
    std::string reason;
  };

  bool execution_tuple_matches(const ExecutionCommand & command) const;
  bool lease_is_retired(const std::string & lease_id) const;
  void retire_execution_lease();

  double min_execution_lease_sec_;
  double max_execution_lease_sec_;
  std::string recovery_owner_;
  std::vector<HoldRecord> holds_;
  std::unordered_set<std::string> retired_execution_lease_ids_;
  std::uint64_t generation_{0U};
  bool execution_session_engaged_{false};
  bool execution_lease_active_{false};
  std::string execution_owner_;
  std::string execution_mission_id_;
  std::string execution_transaction_id_;
  std::string execution_lease_id_;
  double execution_expires_at_sec_{0.0};
  std::string transition_reason_{"startup"};
};

}  // namespace robot_safety
