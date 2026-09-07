#include "robot_api_server/features/elevator/execution/elevator_test_module.hpp"

#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <utility>

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

std::string json_escape_local(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

std::string json_string_local(const std::string & value)
{
  return "\"" + json_escape_local(value) + "\"";
}

ElevatorTestReply basic_error(
  const int status,
  const std::string & code,
  const std::string & detail,
  const std::string & transaction_id = "")
{
  std::ostringstream body;
  body << "{\"ok\":false,"
       << "\"code\":" << json_string_local(code) << ","
       << "\"detail\":" << json_string_local(detail);
  if (!transaction_id.empty()) {
    body << ",\"transaction_id\":" << json_string_local(transaction_id);
  }
  body << "}";
  return ElevatorTestReply{status, code, body.str()};
}

int http_status(const robot_elevator_manager::ElevatorExecutionReplyKind kind)
{
  using Kind = robot_elevator_manager::ElevatorExecutionReplyKind;
  switch (kind) {
    case Kind::kAccepted:
      return 200;
    case Kind::kInvalid:
      return 400;
    case Kind::kConflict:
      return 409;
    case Kind::kStorageFailure:
      return 503;
  }
  return 500;
}

std::string release_error_code(
  const robot_elevator_manager::ElevatorReleaseLoadError error)
{
  return std::string("ELEVATOR_RELEASE_") +
         robot_elevator_manager::to_string(error);
}

bool exact_request(
  const ElevatorTestStartCommand & command,
  const robot_elevator_manager::ElevatorExecutionSnapshot & snapshot)
{
  return
    command.building_id == snapshot.building_id &&
    command.elevator_id == snapshot.elevator_id &&
    command.source_floor_id == snapshot.source_floor_id &&
    command.source_map_id == snapshot.source_map_id &&
    command.target_floor_id == snapshot.target_floor_id &&
    command.target_map_id == snapshot.target_map_id &&
    command.expected_release_id == snapshot.pinned_release_id &&
    command.operator_id == snapshot.operator_id;
}

void append_records_json(
  std::ostringstream & body,
  const std::vector<robot_elevator_manager::ElevatorExecutionEventRecord> & records)
{
  body << "[";
  for (std::size_t index = 0U; index < records.size(); ++index) {
    if (index > 0U) {
      body << ",";
    }
    const auto & record = records[index];
    body << "{"
         << "\"sequence\":" << record.sequence << ","
         << "\"code\":" << json_string_local(record.code) << ","
         << "\"message\":" << json_string_local(record.detail) << ","
         << "\"detail\":" << json_string_local(record.detail) << ","
         << "\"timestamp\":" << json_string_local(record.timestamp)
         << "}";
  }
  body << "]";
}

ElevatorTestReply snapshot_reply(
  const robot_elevator_manager::ElevatorExecutionSnapshot & snapshot,
  const std::string & code,
  const std::string & detail,
  const bool replayed = false)
{
  const bool successful_business_state =
    snapshot.state != "FAILED" && snapshot.state != "LOCKED";
  const auto interlock =
    evaluate_elevator_execution_interlock(snapshot);
  const auto recovery_actions =
    robot_elevator_manager::allowed_recovery_actions(snapshot);
  std::ostringstream body;
  body << "{\"ok\":true,"
       << "\"accepted\":" << (successful_business_state ? "true" : "false") << ","
       << "\"code\":" << json_string_local(code) << ","
       << "\"transaction_id\":" << json_string_local(snapshot.transaction_id) << ","
       << "\"state\":" << json_string_local(snapshot.state) << ","
       << "\"phase\":" << json_string_local(snapshot.phase) << ","
       << "\"effect_sequence\":" << snapshot.effect_sequence << ","
       << "\"awaiting_confirmation\":"
       << (snapshot.awaiting_confirmation ? "true" : "false") << ","
       << "\"expected_confirmation\":";
  if (snapshot.expected_confirmation.empty()) {
    body << "null";
  } else {
    body << json_string_local(snapshot.expected_confirmation);
  }
  body << ",\"current_floor_id\":" << json_string_local(snapshot.current_floor_id)
       << ",\"current_map_id\":" << json_string_local(snapshot.current_map_id)
       << ",\"physical_zone\":"
       << json_string_local(to_string(snapshot.physical_zone))
       << ",\"interrupted_state\":"
       << (snapshot.interrupted_state.empty() ?
       "null" : json_string_local(snapshot.interrupted_state))
       << ",\"interrupted_expected_confirmation\":"
       << (snapshot.interrupted_expected_confirmation.empty() ?
       "null" :
       json_string_local(snapshot.interrupted_expected_confirmation))
       << ",\"safety_hold_state_known\":"
       << (snapshot.safety_hold_state_known ? "true" : "false")
       << ",\"safety_hold_active\":"
       << (snapshot.safety_hold_active ? "true" : "false")
       << ",\"dual_odom_stop_proven\":"
       << (snapshot.dual_odom_stop_proven ? "true" : "false")
       << ",\"recovery_required\":"
       << (interlock.recovery_required() ? "true" : "false")
       << ",\"runtime_capable\":"
       << (snapshot.runtime_capable ? "true" : "false")
       << ",\"runtime_applied\":"
       << (snapshot.runtime_applied ? "true" : "false")
       << ",\"runtime_resources_reconciled\":"
       << (snapshot.runtime_resources_reconciled ? "true" : "false")
       << ",\"motion_authorized\":"
       << (snapshot.motion_authorized ? "true" : "false")
       << ",\"floor_switch_capable\":"
       << (snapshot.floor_switch_capable ? "true" : "false")
       << ",\"terminal\":" << (snapshot.terminal ? "true" : "false")
       << ",\"task_complete\":"
       << (snapshot.state == "COMPLETE" ? "true" : "false")
       << ",\"api_fallback_active\":false"
       << ",\"runtime_config_pinned\":true"
       << ",\"replayed\":" << (replayed ? "true" : "false")
       << ",\"pinned_release_id\":"
       << json_string_local(snapshot.pinned_release_id)
       << ",\"pinned_release_generation\":"
       << snapshot.pinned_release_generation
       << ",\"building_id\":" << json_string_local(snapshot.building_id)
       << ",\"elevator_id\":" << json_string_local(snapshot.elevator_id)
       << ",\"source_floor_id\":" << json_string_local(snapshot.source_floor_id)
       << ",\"source_map_id\":" << json_string_local(snapshot.source_map_id)
       << ",\"source_asset_epoch\":" << snapshot.source_asset_epoch
       << ",\"source_asset_digest\":"
       << json_string_local(snapshot.source_asset_digest)
       << ",\"target_floor_id\":" << json_string_local(snapshot.target_floor_id)
       << ",\"target_map_id\":" << json_string_local(snapshot.target_map_id)
       << ",\"target_asset_epoch\":" << snapshot.target_asset_epoch
       << ",\"target_asset_digest\":"
       << json_string_local(snapshot.target_asset_digest)
       << ",\"operator_id\":" << json_string_local(snapshot.operator_id)
       << ",\"created_at\":" << json_string_local(snapshot.created_at)
       << ",\"updated_at\":" << json_string_local(snapshot.updated_at)
       << ",\"failure_origin_state\":"
       << (snapshot.failure_origin_state.empty() ?
       "null" : json_string_local(snapshot.failure_origin_state))
       << ",\"failure_origin_effect_kind\":"
       << json_string_local(snapshot.failure_origin_effect_kind)
       << ",\"cleanup_disposition\":"
       << json_string_local(to_string(snapshot.cleanup_disposition))
       << ",\"allowed_recovery_actions\":[";
  for (std::size_t index = 0U; index < recovery_actions.size(); ++index) {
    if (index > 0U) {
      body << ",";
    }
    body << json_string_local(recovery_actions[index]);
  }
  body << "]"
       << ",\"failure_code\":";
  if (snapshot.failure_code.empty()) {
    body << "null";
  } else {
    body << json_string_local(snapshot.failure_code);
  }
  body << ",\"message\":" << json_string_local(detail)
       << ",\"detail\":" << json_string_local(
    snapshot.detail.empty() ? detail : snapshot.detail)
       << ",\"events\":";
  append_records_json(body, snapshot.events);
  body << ",\"errors\":";
  append_records_json(body, snapshot.errors);
  body << "}";
  return ElevatorTestReply{200, code, body.str()};
}

ElevatorTestReply execution_reply(
  const robot_elevator_manager::ElevatorExecutionReply & result)
{
  if (result.kind == robot_elevator_manager::ElevatorExecutionReplyKind::kAccepted &&
    result.snapshot)
  {
    return snapshot_reply(*result.snapshot, result.code, result.detail);
  }
  std::string wire_code = result.code;
  if (wire_code == "ELEVATOR_EXECUTION_TRANSACTION_NOT_FOUND") {
    wire_code = "ELEVATOR_TEST_TRANSACTION_NOT_FOUND";
  } else if (wire_code == "ELEVATOR_EXECUTION_TERMINAL_STATE") {
    wire_code = "ELEVATOR_TEST_TERMINAL_STATE";
  }
  const auto transaction_id =
    result.snapshot ? result.snapshot->transaction_id : std::string{};
  return basic_error(
    http_status(result.kind), wire_code, result.detail, transaction_id);
}

class UnavailableElevatorRuntimePort final :
  public robot_elevator_manager::ElevatorRuntimePort
{
public:
  robot_elevator_manager::ElevatorRuntimeCapabilities capabilities()
  const noexcept override
  {
    return {};
  }

  robot_elevator_manager::ElevatorRuntimeResult prepare(
    const std::string &,
    const robot_elevator_manager::FrozenElevatorRelease &) override
  {
    return unavailable();
  }

  robot_elevator_manager::ElevatorRuntimeResult apply(
    const robot_elevator_manager::ElevatorRuntimeEffect &) override
  {
    return unavailable();
  }

  void request_cancel(const std::string &) noexcept override
  {
  }

  robot_elevator_manager::ElevatorRuntimeResult heartbeat(
    const std::string &) override
  {
    return unavailable();
  }

  robot_elevator_manager::ElevatorRuntimeResult poll_health(
    const std::string &) override
  {
    return unavailable();
  }

private:
  static robot_elevator_manager::ElevatorRuntimeResult unavailable()
  {
    return {
      false,
      "ELEVATOR_RUNTIME_ADAPTER_NOT_DEPLOYED",
      "live elevator runtime adapter is not deployed",
    };
  }
};

}  // namespace

class ElevatorTestModule::Implementation
{
public:
  Implementation(
    fs::path maps_root,
    ElevatorReleaseResolver release_resolver,
    std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port,
    robot_elevator_manager::ElevatorExecutionOptions execution_options = {})
  : maps_root_(std::move(maps_root)),
    release_resolver_(std::move(release_resolver))
  {
    if (!release_resolver_) {
      release_resolver_ = robot_elevator_manager::load_elevator_release;
    }
    if (!runtime_port) {
      runtime_port = std::make_shared<UnavailableElevatorRuntimePort>();
    }
    execution_ =
      std::make_unique<robot_elevator_manager::ElevatorExecutionModule>(
      maps_root_ / ".elevator_test" / "active.yaml",
      std::move(runtime_port), execution_options);
  }

  ElevatorTestReply start(const ElevatorTestStartCommand & command)
  {
    std::lock_guard<std::mutex> lock(start_mutex_);
    if (
      !robot_elevator_manager::safe_asset_id(command.building_id) ||
      !robot_elevator_manager::safe_asset_id(command.elevator_id) ||
      !robot_elevator_manager::safe_asset_id(command.source_floor_id) ||
      !robot_elevator_manager::safe_asset_id(command.source_map_id) ||
      !robot_elevator_manager::safe_asset_id(command.target_floor_id) ||
      !robot_elevator_manager::safe_asset_id(command.target_map_id) ||
      !robot_elevator_manager::safe_asset_id(command.expected_release_id) ||
      !robot_elevator_manager::safe_asset_id(command.operator_id))
    {
      return basic_error(
        400,
        "INVALID_ELEVATOR_TEST_REQUEST",
        "all identifiers, expected_release_id, and operator_id must be path-safe");
    }
    if (
      command.source_floor_id == command.target_floor_id &&
      command.source_map_id == command.target_map_id)
    {
      return basic_error(
        400,
        "INVALID_ELEVATOR_TEST_ROUTE",
        "source and target floor/map must be different");
    }

    if (const auto retained = execution_->current_snapshot()) {
      if (!retained->terminal) {
        if (exact_request(command, *retained)) {
          return snapshot_reply(
            *retained,
            "ELEVATOR_EXECUTION_REPLAYED",
            "existing exact active transaction returned",
            true);
        }
        return basic_error(
          409,
          "ELEVATOR_EXECUTION_ACTIVE",
          "another elevator transaction is still active",
          retained->transaction_id);
      }
    }

    robot_elevator_manager::ElevatorReleaseLoadRequest request;
    request.config_root =
      (maps_root_ / command.building_id / ".elevator_config").string();
    request.building_id = command.building_id;
    request.source_floor_id = command.source_floor_id;
    request.source_map_id = command.source_map_id;
    request.target_floor_id = command.target_floor_id;
    request.target_map_id = command.target_map_id;
    request.preferred_elevator_id = command.elevator_id;
    request.expected_release_id = command.expected_release_id;
    const auto loaded = release_resolver_(request);
    if (!loaded.ok() || !loaded.release) {
      return basic_error(
        422,
        release_error_code(loaded.error),
        loaded.message.empty() ?
        "elevator release could not be pinned" : loaded.message);
    }

    const auto & release = *loaded.release;
    if (
      release.release_id != command.expected_release_id ||
      release.building_id != command.building_id ||
      release.elevator_id != command.elevator_id ||
      release.source.floor_id != command.source_floor_id ||
      release.source.map_id != command.source_map_id ||
      release.target.floor_id != command.target_floor_id ||
      release.target.map_id != command.target_map_id)
    {
      return basic_error(
        409,
        "ELEVATOR_RELEASE_IDENTITY_MISMATCH",
        "pinned elevator release does not match the exact requested route");
    }

    const auto transaction_id = next_transaction_id();
    return execution_reply(
      execution_->start(
        robot_elevator_manager::ElevatorExecutionStart{
          transaction_id,
          command.operator_id,
          release,
        }));
  }

  ElevatorTestReply state(const ElevatorTestStateQuery & query) const
  {
    return execution_reply(execution_->snapshot(query.transaction_id));
  }

  ElevatorTestReply confirm(const ElevatorTestConfirmCommand & command)
  {
    return execution_reply(
      execution_->confirm(
        robot_elevator_manager::ElevatorExecutionConfirmation{
          command.transaction_id,
          command.expected_state,
          command.effect_sequence,
          command.event,
          command.observed_floor_id,
          command.operator_id,
        }));
  }

  ElevatorTestReply cancel(const ElevatorTestCancelCommand & command)
  {
    return execution_reply(
      execution_->cancel(
        robot_elevator_manager::ElevatorExecutionCancellation{
          command.transaction_id,
          command.effect_sequence,
          command.reason,
          command.operator_id,
        }));
  }

  ElevatorTestReply recover(const ElevatorTestRecoverCommand & command)
  {
    return execution_reply(
      execution_->recover(
        robot_elevator_manager::ElevatorExecutionRecovery{
          command.transaction_id,
          command.expected_state,
          command.effect_sequence,
          command.operator_id,
          command.reason,
          command.source_outside_confirmed,
          command.confirmed_floor_id,
          command.action,
          command.physical_zone,
          command.stationary_confirmed,
          command.door_zone_clear_confirmed,
        }));
  }

  std::optional<robot_elevator_manager::ElevatorExecutionSnapshot>
  current_snapshot() const
  {
    return execution_->current_snapshot();
  }

  std::optional<std::string> active_transaction() const
  {
    return execution_->active_transaction();
  }

  bool journal_recovery_required() const
  {
    return execution_->journal_recovery_required();
  }

  bool persistent_recovery_lock_enabled() const noexcept
  {
    return execution_->persistent_recovery_lock_enabled();
  }

private:
  std::string next_transaction_id()
  {
    const auto wall_milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto monotonic_ticks =
      std::chrono::steady_clock::now().time_since_epoch().count();
    return "elevator-test-" + std::to_string(wall_milliseconds) + "-" +
           std::to_string(monotonic_ticks) + "-" +
           std::to_string(++transaction_sequence_);
  }

  fs::path maps_root_;
  ElevatorReleaseResolver release_resolver_;
  std::unique_ptr<robot_elevator_manager::ElevatorExecutionModule> execution_;
  std::mutex start_mutex_;
  std::uint64_t transaction_sequence_{0U};
};

ElevatorTestModule::ElevatorTestModule(fs::path maps_root)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      ElevatorReleaseResolver{},
      std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort>{}))
{
}

ElevatorTestModule::ElevatorTestModule(
  fs::path maps_root,
  robot_elevator_manager::ElevatorExecutionOptions execution_options)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      ElevatorReleaseResolver{},
      std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort>{},
      execution_options))
{
}

ElevatorTestModule::ElevatorTestModule(
  fs::path maps_root,
  ElevatorReleaseResolver release_resolver)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      std::move(release_resolver),
      std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort>{}))
{
}

ElevatorTestModule::ElevatorTestModule(
  fs::path maps_root,
  std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      ElevatorReleaseResolver{},
      std::move(runtime_port)))
{
}

ElevatorTestModule::ElevatorTestModule(
  fs::path maps_root,
  std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port,
  robot_elevator_manager::ElevatorExecutionOptions execution_options)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      ElevatorReleaseResolver{},
      std::move(runtime_port),
      execution_options))
{
}

ElevatorTestModule::ElevatorTestModule(
  fs::path maps_root,
  ElevatorReleaseResolver release_resolver,
  std::shared_ptr<robot_elevator_manager::ElevatorRuntimePort> runtime_port)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root),
      std::move(release_resolver),
      std::move(runtime_port)))
{
}

ElevatorTestModule::~ElevatorTestModule() = default;

ElevatorTestModule::ElevatorTestModule(ElevatorTestModule &&) noexcept = default;

ElevatorTestModule & ElevatorTestModule::operator=(
  ElevatorTestModule &&) noexcept = default;

ElevatorTestReply ElevatorTestModule::start(
  const ElevatorTestStartCommand & command)
{
  return implementation_->start(command);
}

ElevatorTestReply ElevatorTestModule::state(
  const ElevatorTestStateQuery & query) const
{
  return implementation_->state(query);
}

ElevatorTestReply ElevatorTestModule::confirm(
  const ElevatorTestConfirmCommand & command)
{
  return implementation_->confirm(command);
}

ElevatorTestReply ElevatorTestModule::cancel(
  const ElevatorTestCancelCommand & command)
{
  return implementation_->cancel(command);
}

ElevatorTestReply ElevatorTestModule::recover(
  const ElevatorTestRecoverCommand & command)
{
  return implementation_->recover(command);
}

std::optional<robot_elevator_manager::ElevatorExecutionSnapshot>
ElevatorTestModule::current_snapshot() const
{
  return implementation_->current_snapshot();
}

std::optional<std::string> ElevatorTestModule::active_transaction() const
{
  return implementation_->active_transaction();
}

bool ElevatorTestModule::journal_recovery_required() const
{
  return implementation_->journal_recovery_required();
}

bool ElevatorTestModule::persistent_recovery_lock_enabled() const noexcept
{
  return implementation_->persistent_recovery_lock_enabled();
}

}  // namespace robot_api_server
