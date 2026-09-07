#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_elevator_manager/elevator_execution_module.hpp"
#include "robot_elevator_manager/elevator_release_loader.hpp"

namespace robot_api_server
{

struct ElevatorTestStartCommand
{
  std::string building_id;
  std::string elevator_id;
  std::string source_floor_id;
  std::string source_map_id;
  std::string target_floor_id;
  std::string target_map_id;
  std::string expected_release_id;
  std::string operator_id;
};

struct ElevatorTestStateQuery
{
  std::string transaction_id;
};

struct ElevatorTestConfirmCommand
{
  std::string transaction_id;
  std::string expected_state;
  std::uint64_t effect_sequence{0U};
  std::string event;
  std::string observed_floor_id;
  std::string operator_id;
};

struct ElevatorTestCancelCommand
{
  std::string transaction_id;
  std::uint64_t effect_sequence{0U};
  std::string reason;
  std::string operator_id;
};

struct ElevatorTestRecoverCommand
{
  std::string transaction_id;
  std::string expected_state;
  std::uint64_t effect_sequence{0U};
  std::string operator_id;
  std::string reason;
  bool source_outside_confirmed{false};
  std::string confirmed_floor_id;
  std::string action;
  std::string physical_zone;
  bool stationary_confirmed{false};
  bool door_zone_clear_confirmed{false};
};

struct ElevatorTestReply
{
  int status{500};
  std::string code{"INTERNAL_ERROR"};
  std::string body{"{}"};

  bool ok() const noexcept
  {
    return status >= 200 && status < 300;
  }
};

using ElevatorReleaseResolver = std::function<
  robot_elevator_manager::ElevatorReleaseLoadResult(
    const robot_elevator_manager::ElevatorReleaseLoadRequest &)>;

// HTTP-independent transaction boundary for the commissioning App.
//
// It pins the exact immutable release, delegates transaction semantics to the
// pure-C++ ElevatorExecutionModule, and renders the App wire contract. Motion,
// holds, modes, localization, and floor switching remain exclusively behind
// the injected ElevatorRuntimePort; the maps_root-only constructor injects a
// truthful fail-closed unavailable adapter.
class ElevatorTestModule
{
public:
  explicit ElevatorTestModule(std::filesystem::path maps_root);
  ElevatorTestModule(
    std::filesystem::path maps_root,
    robot_elevator_manager::ElevatorExecutionOptions execution_options);
  ElevatorTestModule(
    std::filesystem::path maps_root,
    ElevatorReleaseResolver release_resolver);
  ElevatorTestModule(
    std::filesystem::path maps_root,
    std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port);
  ElevatorTestModule(
    std::filesystem::path maps_root,
    std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port,
    robot_elevator_manager::ElevatorExecutionOptions execution_options);
  ElevatorTestModule(
    std::filesystem::path maps_root,
    ElevatorReleaseResolver release_resolver,
    std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port);
  ~ElevatorTestModule();

  ElevatorTestModule(const ElevatorTestModule &) = delete;
  ElevatorTestModule & operator=(const ElevatorTestModule &) = delete;
  ElevatorTestModule(ElevatorTestModule &&) noexcept;
  ElevatorTestModule & operator=(ElevatorTestModule &&) noexcept;

  ElevatorTestReply start(const ElevatorTestStartCommand & command);
  ElevatorTestReply state(const ElevatorTestStateQuery & query) const;
  ElevatorTestReply confirm(const ElevatorTestConfirmCommand & command);
  ElevatorTestReply cancel(const ElevatorTestCancelCommand & command);
  ElevatorTestReply recover(const ElevatorTestRecoverCommand & command);
  std::optional<robot_elevator_manager::ElevatorExecutionSnapshot>
  current_snapshot() const;
  std::optional<std::string> active_transaction() const;
  bool journal_recovery_required() const;
  bool persistent_recovery_lock_enabled() const noexcept;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace robot_api_server
