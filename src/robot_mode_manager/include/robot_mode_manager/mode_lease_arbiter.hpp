#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>

namespace robot_mode_manager
{

enum class OperatingMode
{
  kNormal,
  kRamp,
  kElevatorWait,
  kElevatorRide,
  kDoorway,
  kRecovery,
};

enum class ModeOperation
{
  kSet,
  kRelease,
};

enum class ModeDecisionCode
{
  kOk,
  kInvalidRequest,
  kUnsupportedMode,
  kLeaseConflict,
  kStaleLease,
  kNotOwner,
};

struct ModeCommand
{
  ModeOperation operation{ModeOperation::kSet};
  std::string mode;
  std::string owner;
  std::string mission_id;
  std::string lease_id;
  double lease_duration_sec{0.0};
};

struct ModeSnapshot
{
  OperatingMode mode{OperatingMode::kNormal};
  std::string owner;
  std::string mission_id;
  std::string lease_id;
  double lease_remaining_sec{0.0};
  bool lease_active{false};
  std::uint64_t generation{0U};
  std::string transition_reason{"startup"};
};

struct ModeDecision
{
  bool accepted{false};
  bool changed{false};
  ModeDecisionCode code{ModeDecisionCode::kInvalidRequest};
  std::string message;
  ModeSnapshot state;
};

class ModeLeaseArbiter
{
public:
  explicit ModeLeaseArbiter(
    double min_lease_duration_sec = 0.20,
    double max_lease_duration_sec = 30.0,
    std::string recovery_owner = "robot_mission_manager");

  ModeDecision apply(const ModeCommand & command, double now_sec);
  std::optional<ModeSnapshot> expire(double now_sec);
  ModeSnapshot snapshot(double now_sec) const;

private:
  bool is_retired(const std::string & lease_id) const;
  void retire_active_lease();
  void enter_normal(const std::string & reason);

  double min_lease_duration_sec_;
  double max_lease_duration_sec_;
  std::string recovery_owner_;
  ModeSnapshot state_;
  double lease_expires_at_sec_{0.0};
  std::unordered_set<std::string> retired_lease_ids_;
};

std::string to_string(OperatingMode mode);
bool parse_mode(const std::string & value, OperatingMode & mode);

}  // namespace robot_mode_manager
