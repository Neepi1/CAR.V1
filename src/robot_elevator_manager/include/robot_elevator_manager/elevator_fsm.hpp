#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_elevator_manager
{

enum class ElevatorState
{
  kIdle,
  kNavigatingHallCall,
  kAcquiringHallHold,
  kAcquiringExecutionLease,
  kPressingCallButton,
  kSettingElevatorWaitMode,
  kReleasingHallHold,
  kNavigatingHallWait,
  kWaitingSourceDoor,
  kSettingDoorwayEntryMode,
  kNavigatingSourceDoorway,
  kEnteringCabin,
  kVerifyingInside,
  kAcquiringCabinHold,
  kPressingTargetButton,
  kPausingCorrections,
  kSettingRideMode,
  kRiding,
  kWaitingTargetDoor,
  kBeginningFloorTransition,
  kResumingCorrections,
  kSwitchingFloor,
  kVerifyingFloorReady,
  kSettingDoorwayExitMode,
  kReleasingCabinHold,
  kNavigatingTargetDoorway,
  kExitingCabin,
  kVerifyingOutside,
  kAcquiringExitHold,
  kReleasingOperatingMode,
  kReleasingExecutionLease,
  kReleasingExitHold,
  kComplete,
  kFailureCleanup,
  kLocked,
};

enum class ElevatorEventKind
{
  kEffectSucceeded,
  kEffectFailed,
  kFootprintInside,
  kFootprintOutside,
  kFootprintStraddling,
  kCancelRequested,
};

struct ElevatorEvent
{
  ElevatorEventKind kind{ElevatorEventKind::kEffectFailed};
  std::string detail;
  std::uint64_t effect_sequence{0U};
  std::string transaction_id;
};

enum class ElevatorEffectKind
{
  kNone,
  kNavigateToPose,
  kAcquireSafetyHold,
  kReleaseSafetyHold,
  kAcquireExecutionLease,
  kReleaseExecutionLease,
  kMockPressCallButton,
  kMockPressTargetButton,
  kMockWaitDoorOpen,
  kMockRide,
  kSetOperatingMode,
  kReleaseOperatingMode,
  kPauseLocalizationCorrections,
  kResumeLocalizationCorrections,
  kBeginFloorTransition,
  kSwitchFloor,
  kVerifyFloorReady,
  kVerifyFootprintInside,
  kVerifyFootprintOutside,
  kComplete,
  kHoldAndCancel,
};

struct ElevatorEffect
{
  ElevatorEffectKind kind{ElevatorEffectKind::kNone};
  std::uint64_t sequence{0U};
  std::string transaction_id;
  std::string pose_id;
  std::string floor_id;
  std::string mode;
  std::string detail;
  std::string map_id;
};

struct ElevatorFsmOutput
{
  bool accepted{false};
  bool transitioned{false};
  ElevatorState state{ElevatorState::kIdle};
  ElevatorEffect effect;
  std::string message;
};

struct ElevatorFsmOptions
{
  bool mock_ports_enabled{false};
};

class ElevatorFsm
{
public:
  explicit ElevatorFsm(ElevatorFsmOptions options = {});

  ElevatorFsmOutput start(
    const std::string & transaction_id,
    const ElevatorRoute & route);
  ElevatorFsmOutput dispatch(const ElevatorEvent & event);

  ElevatorState state() const noexcept;
  bool locked() const noexcept;

private:
  ElevatorFsmOutput advance_effect_success();
  ElevatorFsmOutput transition(ElevatorState next, ElevatorEffect effect);
  ElevatorFsmOutput lock(const std::string & reason, bool accepted);
  ElevatorFsmOutput inert(const std::string & reason) const;
  ElevatorEffect navigation_effect(PoseRole role, bool source) const;

  ElevatorFsmOptions options_;
  ElevatorState state_{ElevatorState::kIdle};
  std::optional<ElevatorRoute> route_;
  std::string transaction_id_;
  std::uint64_t next_effect_sequence_{1U};
  std::uint64_t pending_effect_sequence_{0U};
};

std::string to_string(ElevatorState state);
std::string to_string(ElevatorEffectKind effect);

}  // namespace robot_elevator_manager
