#include "robot_elevator_manager/elevator_execution_module.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <system_error>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace robot_elevator_manager
{
namespace
{

namespace fs = std::filesystem;

constexpr char kRuntimeUnavailable[] = "ELEVATOR_RUNTIME_ADAPTER_NOT_DEPLOYED";
constexpr char kRestartLocked[] = "ELEVATOR_EXECUTION_RESTART_RECOVERY_REQUIRED";
constexpr char kRestartCleanupUnproven[] =
  "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN";
constexpr char kJournalFailure[] = "ELEVATOR_EXECUTION_JOURNAL_WRITE_FAILED";
constexpr char kCleanupUnproven[] = "ELEVATOR_FAILURE_CLEANUP_UNPROVEN";
constexpr std::size_t kMaximumRecords = 256U;
constexpr std::size_t kMaximumCleanupAttempts = 3U;
// The API process starts before the resident Nav2 action servers on the field
// runtime.  A restart recovery must therefore outlive the normal boot window
// without weakening the three-attempt budget for non-transient proof failures.
// Each production endpoint attempt is bounded by endpoint_timeout_sec, so 40
// attempts gives the 240-second systemd startup window ample margin while
// remaining finite and fail-closed.
constexpr std::size_t kMaximumRestartStartupAttempts = 40U;
// Keep the outer v1 envelope readable by the immediately previous binary,
// but version the snapshot semantics separately. UNKNOWN hold state is
// projected as active in that envelope so an old reader also fails closed.
constexpr std::uint32_t kJournalEnvelopeSchemaVersion = 1U;
constexpr std::uint32_t kLegacySnapshotSchemaVersion = 1U;
constexpr std::uint32_t kPreviousSnapshotSchemaVersion = 2U;
constexpr std::uint32_t kSnapshotSchemaVersion = 3U;
constexpr std::uintmax_t kMaximumJournalBytes = 4U * 1024U * 1024U;

ElevatorRuntimeResult success_result()
{
  return ElevatorRuntimeResult{true, "OK", ""};
}

ElevatorRuntimeResult take_result(
  std::vector<ElevatorRuntimeResult> & results)
{
  if (results.empty()) {
    return success_result();
  }
  auto result = std::move(results.front());
  results.erase(results.begin());
  return result;
}

bool restart_recovery_startup_transient(const std::string & code)
{
  return
    code == "ELEVATOR_SAFETY_HOLD_SERVICE_UNAVAILABLE" ||
    code == "ELEVATOR_RECOVERY_HOLD_RELEASE_SERVICE_UNAVAILABLE" ||
    code == "ELEVATOR_EXECUTION_LEASE_SERVICE_UNAVAILABLE" ||
    code == "ELEVATOR_MODE_SERVICE_UNAVAILABLE" ||
    code == "ELEVATOR_CORRECTION_PAUSE_SERVICE_UNAVAILABLE" ||
    code == "ELEVATOR_CORRECTION_PAUSE_TIMEOUT" ||
    code == "ELEVATOR_OPERATING_MODE_TIMEOUT" ||
    code == "ELEVATOR_CLEANUP_RUNTIME_CONTEXT_UNPROVEN" ||
    code == "ELEVATOR_NAV2_ACTION_UNAVAILABLE" ||
    code == "ELEVATOR_FLOOR_SWITCH_ACTION_UNAVAILABLE" ||
    code == "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_PENDING" ||
    code == "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_PENDING" ||
    code == "ELEVATOR_RESTART_ACTION_TERMINAL_UNPROVEN" ||
    code == "ELEVATOR_RESTART_SAFETY_HOLD_UNPROVEN" ||
    code == "ELEVATOR_ODOM_STOP_UNPROVEN";
}

std::string timestamp_utc()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc {};
#ifdef _WIN32
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

bool canonical_asset_digest(const std::string & value)
{
  if (value.size() != 71U || value.rfind("sha256:", 0U) != 0U) {
    return false;
  }
  return std::all_of(
    value.begin() + 7, value.end(),
    [](const unsigned char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

bool bounded_reason(const std::string & value)
{
  return value.size() <= 256U;
}

void append_identity_value(
  std::ostringstream & output,
  const std::string & value)
{
  output << value.size() << ":" << value << "\n";
}

std::string frozen_release_identity(const FrozenElevatorRelease & release)
{
  std::ostringstream output;
  append_identity_value(output, release.release_id);
  output << release.generation << "\n";
  append_identity_value(output, release.configuration_digest);
  append_identity_value(output, release.building_id);
  append_identity_value(output, release.elevator_id);
  // Preserve the historical v2 replay identity byte-for-byte so an upgrade
  // can recover an already journaled three-point transaction.  V3 appends
  // the schema and panel-side contract because both affect physical motion.
  if (release.schema_version >= 3U) {
    output << release.schema_version << "\n";
  }
  for (const auto * floor : {&release.source, &release.target}) {
    append_identity_value(output, floor->floor_id);
    append_identity_value(output, floor->map_id);
    output << floor->map_asset_epoch << "\n";
    append_identity_value(output, floor->map_asset_digest);
    if (release.schema_version >= 3U) {
      output << static_cast<int>(floor->hall_call_panel_side) << "\n"
             << static_cast<int>(floor->cabin_panel_side) << "\n";
    }
    for (const auto & pose : floor->poses) {
      output << static_cast<int>(pose.role) << "\n";
      append_identity_value(output, pose.pose_id);
      output << std::hexfloat << pose.x << "\n"
             << pose.y << "\n"
             << pose.yaw << "\n"
             << std::defaultfloat;
    }
  }
  return output.str();
}

bool terminal_state(const std::string & state)
{
  return state == "COMPLETE" || state == "FAILED" ||
         state == "LOCKED" || state == "CANCELLED";
}

std::optional<ElevatorCleanupDisposition> cleanup_disposition_from_string(
  const std::string & value)
{
  if (value == "SOURCE_OUTSIDE") {
    return ElevatorCleanupDisposition::kSourceOutside;
  }
  if (value == "TARGET_OUTSIDE") {
    return ElevatorCleanupDisposition::kTargetOutside;
  }
  if (value == "RETAIN_LOCK") {
    return ElevatorCleanupDisposition::kRetainLock;
  }
  return std::nullopt;
}

std::optional<ElevatorPhysicalZone> physical_zone_from_string(
  const std::string & value)
{
  if (value == "UNKNOWN") {
    return ElevatorPhysicalZone::kUnknown;
  }
  if (value == "SOURCE_OUTSIDE") {
    return ElevatorPhysicalZone::kSourceOutside;
  }
  if (value == "DOORWAY") {
    return ElevatorPhysicalZone::kDoorway;
  }
  if (value == "CABIN") {
    return ElevatorPhysicalZone::kCabin;
  }
  if (value == "TARGET_OUTSIDE") {
    return ElevatorPhysicalZone::kTargetOutside;
  }
  return std::nullopt;
}

std::optional<std::string> confirmation_for(
  const ElevatorFsmOutput & output,
  const bool automatic_button_control)
{
  switch (output.effect.kind) {
    case ElevatorEffectKind::kMockPressCallButton:
      if (automatic_button_control) {
        return std::nullopt;
      }
      return "CALL_BUTTON_PRESSED";
    case ElevatorEffectKind::kMockPressTargetButton:
      if (automatic_button_control) {
        return std::nullopt;
      }
      return "TARGET_BUTTON_PRESSED";
    case ElevatorEffectKind::kMockRide:
      return "TARGET_FLOOR_ARRIVED";
    case ElevatorEffectKind::kMockWaitDoorOpen:
      if (output.state == ElevatorState::kWaitingSourceDoor) {
        return "SOURCE_DOOR_OPEN";
      }
      if (output.state == ElevatorState::kWaitingTargetDoor) {
        return "TARGET_DOOR_OPEN";
      }
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

std::string expected_observed_floor(
  const std::string & confirmation,
  const ElevatorExecutionSnapshot & snapshot)
{
  if (
    confirmation == "TARGET_FLOOR_ARRIVED" ||
    confirmation == "TARGET_DOOR_OPEN")
  {
    return snapshot.target_floor_id;
  }
  return snapshot.source_floor_id;
}

FloorElevatorTopology topology_floor(const ElevatorRuntimeFloor & floor)
{
  FloorElevatorTopology result;
  result.floor_id = floor.floor_id;
  result.map_id = floor.map_id;
  result.hall_call_panel_side = floor.hall_call_panel_side;
  result.cabin_panel_side = floor.cabin_panel_side;
  for (const auto & pose : floor.poses) {
    result.poses.push_back(PoseBinding{pose.role, pose.pose_id});
  }
  return result;
}

ElevatorRoute route_from(const FrozenElevatorRelease & release)
{
  return ElevatorRoute{
    release.elevator_id,
    release.building_id,
    topology_floor(release.source),
    topology_floor(release.target),
    release.schema_version,
  };
}

std::optional<ElevatorRuntimePose> find_runtime_pose(
  const FrozenElevatorRelease & release,
  const std::string & pose_id)
{
  for (const auto * floor : {&release.source, &release.target}) {
    for (const auto & pose : floor->poses) {
      if (pose.pose_id == pose_id) {
        return pose;
      }
    }
  }
  return std::nullopt;
}

const ElevatorRuntimeFloor * effect_floor(
  const FrozenElevatorRelease & release,
  const ElevatorEffect & effect)
{
  if (effect.floor_id == release.source.floor_id) {
    return &release.source;
  }
  if (effect.floor_id == release.target.floor_id) {
    return &release.target;
  }
  if (effect.map_id == release.source.map_id) {
    return &release.source;
  }
  if (effect.map_id == release.target.map_id) {
    return &release.target;
  }
  return nullptr;
}

ElevatorRuntimeEffect enrich_effect(
  const FrozenElevatorRelease & release,
  const ElevatorEffect & effect)
{
  ElevatorRuntimeEffect result;
  result.effect = effect;
  result.building_id = release.building_id;
  result.elevator_id = release.elevator_id;
  if (const auto * floor = effect_floor(release, effect)) {
    result.floor_id = floor->floor_id;
    result.map_id = floor->map_id;
    result.asset_epoch = floor->map_asset_epoch;
    result.asset_digest = floor->map_asset_digest;
  } else {
    result.floor_id = effect.floor_id;
    result.map_id = effect.map_id;
  }
  if (!effect.pose_id.empty()) {
    result.target_pose = find_runtime_pose(release, effect.pose_id);
  }
  return result;
}

bool has_durable_source_outside_confirmation(
  const ElevatorExecutionSnapshot & snapshot)
{
  if (
    snapshot.cleanup_disposition != ElevatorCleanupDisposition::kSourceOutside ||
    snapshot.physical_zone != ElevatorPhysicalZone::kSourceOutside ||
    snapshot.current_floor_id != snapshot.source_floor_id)
  {
    return false;
  }
  return std::any_of(
    snapshot.events.cbegin(), snapshot.events.cend(),
    [&snapshot](const ElevatorExecutionEventRecord & event) {
      const auto expected_detail_prefix =
        "operator=" + snapshot.operator_id +
        ";floor=" + snapshot.source_floor_id + ";reason=";
      return event.sequence > 0U &&
             event.sequence <= snapshot.effect_sequence &&
             event.code == "ON_SITE_SOURCE_OUTSIDE_CONFIRMED" &&
             event.detail.rfind(expected_detail_prefix, 0U) == 0U;
    });
}

ElevatorRuntimeCleanupContext cleanup_context_from(
  const ElevatorExecutionSnapshot & snapshot)
{
  ElevatorRuntimeCleanupContext context;
  context.transaction_id = snapshot.transaction_id;
  context.building_id = snapshot.building_id;
  context.disposition = snapshot.cleanup_disposition;
  context.source = {
    snapshot.source_floor_id,
    snapshot.source_map_id,
    snapshot.source_asset_epoch,
    snapshot.source_asset_digest,
  };
  context.target = {
    snapshot.target_floor_id,
    snapshot.target_map_id,
    snapshot.target_asset_epoch,
    snapshot.target_asset_digest,
  };
  context.legacy_preflight_orphan =
    !snapshot.runtime_applied &&
    snapshot.cleanup_disposition ==
    ElevatorCleanupDisposition::kSourceOutside;
  context.source_outside_confirmation_recorded =
    has_durable_source_outside_confirmation(snapshot);
  context.floor_switch_action_may_have_been_submitted =
    snapshot_may_have_submitted_floor_switch_action(snapshot);
  return context;
}

bool valid_release(const FrozenElevatorRelease & release)
{
  if (
    !safe_asset_id(release.release_id) ||
    !safe_asset_id(release.building_id) ||
    !safe_asset_id(release.elevator_id) ||
    release.configuration_digest.empty() ||
    release.configuration_digest.size() > 128U ||
    !safe_asset_id(release.source.floor_id) ||
    !safe_asset_id(release.source.map_id) ||
    !safe_asset_id(release.target.floor_id) ||
    !safe_asset_id(release.target.map_id) ||
    release.generation == 0U ||
    release.source.map_asset_epoch == 0U ||
    release.target.map_asset_epoch == 0U ||
    !canonical_asset_digest(release.source.map_asset_digest) ||
    !canonical_asset_digest(release.target.map_asset_digest) ||
    release.source.floor_id == release.target.floor_id)
  {
    return false;
  }
  for (const auto * floor : {&release.source, &release.target}) {
    for (const auto & pose : floor->poses) {
      if (
        !safe_pose_id(pose.pose_id) ||
        !std::isfinite(pose.x) ||
        !std::isfinite(pose.y) ||
        !std::isfinite(pose.yaw))
      {
        return false;
      }
    }
  }
  return validate_topology(
    ElevatorTopology{
      release.elevator_id,
      release.building_id,
      {topology_floor(release.source), topology_floor(release.target)},
      release.schema_version,
    }).ok();
}

void append_record(
  std::vector<ElevatorExecutionEventRecord> & records,
  const std::uint64_t sequence,
  std::string code,
  std::string detail)
{
  records.push_back(
    ElevatorExecutionEventRecord{
      sequence,
      std::move(code),
      std::move(detail),
      timestamp_utc(),
    });
  if (records.size() > kMaximumRecords) {
    records.erase(records.begin(), records.begin() + (records.size() - kMaximumRecords));
  }
}

YAML::Node record_node(const ElevatorExecutionEventRecord & record)
{
  YAML::Node node;
  node["sequence"] = record.sequence;
  node["code"] = record.code;
  node["detail"] = record.detail;
  node["timestamp"] = record.timestamp;
  return node;
}

YAML::Node snapshot_node(const ElevatorExecutionSnapshot & value)
{
  YAML::Node node;
  node["transaction_id"] = value.transaction_id;
  node["state"] = value.state;
  node["phase"] = value.phase;
  node["effect_sequence"] = value.effect_sequence;
  node["awaiting_confirmation"] = value.awaiting_confirmation;
  node["expected_confirmation"] = value.expected_confirmation;
  node["current_floor_id"] = value.current_floor_id;
  node["current_map_id"] = value.current_map_id;
  node["physical_zone"] = to_string(value.physical_zone);
  node["interrupted_state"] = value.interrupted_state;
  node["interrupted_expected_confirmation"] =
    value.interrupted_expected_confirmation;
  node["safety_hold_state_known"] = value.safety_hold_state_known;
  node["safety_hold_active"] =
    value.safety_hold_state_known ? value.safety_hold_active : true;
  node["dual_odom_stop_proven"] = value.dual_odom_stop_proven;
  node["runtime_capable"] = value.runtime_capable;
  node["runtime_applied"] = value.runtime_applied;
  node["runtime_resources_reconciled"] =
    value.runtime_resources_reconciled;
  node["motion_authorized"] = value.motion_authorized;
  node["floor_switch_capable"] = value.floor_switch_capable;
  node["terminal"] = value.terminal;
  node["pinned_release_id"] = value.pinned_release_id;
  node["frozen_release_identity"] = value.frozen_release_identity;
  node["pinned_release_generation"] = value.pinned_release_generation;
  node["building_id"] = value.building_id;
  node["elevator_id"] = value.elevator_id;
  node["source_floor_id"] = value.source_floor_id;
  node["source_map_id"] = value.source_map_id;
  node["source_asset_epoch"] = value.source_asset_epoch;
  node["source_asset_digest"] = value.source_asset_digest;
  node["target_floor_id"] = value.target_floor_id;
  node["target_map_id"] = value.target_map_id;
  node["target_asset_epoch"] = value.target_asset_epoch;
  node["target_asset_digest"] = value.target_asset_digest;
  node["operator_id"] = value.operator_id;
  node["created_at"] = value.created_at;
  node["updated_at"] = value.updated_at;
  node["failure_code"] = value.failure_code;
  node["detail"] = value.detail;
  node["failure_origin_state"] = value.failure_origin_state;
  node["failure_origin_effect_kind"] = value.failure_origin_effect_kind;
  node["cleanup_disposition"] = to_string(value.cleanup_disposition);
  for (const auto & record : value.events) {
    node["events"].push_back(record_node(record));
  }
  for (const auto & record : value.errors) {
    node["errors"].push_back(record_node(record));
  }
  return node;
}

template<typename T>
T scalar_or(const YAML::Node & node, const char * key, const T & fallback)
{
  try {
    const auto value = node[key];
    return value && value.IsScalar() ? value.as<T>() : fallback;
  } catch (const YAML::Exception &) {
    return fallback;
  }
}

template<typename T>
bool required_scalar(
  const YAML::Node & node,
  const char * key,
  T & output)
{
  try {
    const auto value = node[key];
    if (!value || !value.IsScalar()) {
      return false;
    }
    output = value.as<T>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

bool required_boolean(
  const YAML::Node & node,
  const char * key,
  bool & output)
{
  const auto value = node[key];
  if (!value || !value.IsScalar()) {
    return false;
  }
  const auto text = value.Scalar();
  if (text == "true") {
    output = true;
    return true;
  }
  if (text == "false") {
    output = false;
    return true;
  }
  return false;
}

std::vector<ElevatorExecutionEventRecord> parse_records(const YAML::Node & node)
{
  std::vector<ElevatorExecutionEventRecord> records;
  if (!node || !node.IsSequence()) {
    return records;
  }
  for (const auto & item : node) {
    if (!item.IsMap()) {
      continue;
    }
    records.push_back(
      ElevatorExecutionEventRecord{
        scalar_or<std::uint64_t>(item, "sequence", 0U),
        scalar_or<std::string>(item, "code", ""),
        scalar_or<std::string>(item, "detail", ""),
        scalar_or<std::string>(item, "timestamp", ""),
      });
  }
  return records;
}

bool record_matches(
  const ElevatorExecutionEventRecord & record,
  const std::uint64_t sequence,
  const char * code)
{
  return record.sequence == sequence && record.code == code &&
         !record.detail.empty() && !record.timestamp.empty();
}

bool deployed_preflight_orphan_history_matches(
  const ElevatorExecutionSnapshot & value)
{
  if (value.events.size() != 5U || value.errors.size() != 5U) {
    return false;
  }
  const bool events_match =
    record_matches(value.events[0], 0U, "RELEASE_PINNED") &&
    record_matches(value.events[1], 1U, "RUNTIME_PREPARE_INTENT") &&
    record_matches(value.events[2], 2U, "FAILURE_CLEANUP_INTENT") &&
    record_matches(value.events[3], 2U, "FAILURE_CLEANUP_RETRY_INTENT") &&
    record_matches(value.events[4], 2U, "FAILURE_CLEANUP_RETRY_INTENT") &&
    value.events[0].detail ==
    "exact immutable release and route identity pinned" &&
    value.events[1].detail ==
    "frozen release prepare intent journaled before adapter invocation" &&
    value.events[2].detail ==
    "navigation, mapping, and docking must be idle before elevator execution" &&
    value.events[3].detail ==
    "attempt=2;safety_hold_proven=false;dual_odom_stop_proven=false" &&
    value.events[4].detail ==
    "attempt=3;safety_hold_proven=false;dual_odom_stop_proven=false";
  const bool errors_match =
    record_matches(value.errors[0], 1U, "ELEVATOR_SOURCE_RUNTIME_BUSY") &&
    record_matches(value.errors[1], 2U, "ELEVATOR_RUNTIME_NOT_PREPARED") &&
    record_matches(value.errors[2], 2U, "ELEVATOR_RUNTIME_NOT_PREPARED") &&
    record_matches(value.errors[3], 2U, "ELEVATOR_RUNTIME_NOT_PREPARED") &&
    record_matches(
      value.errors[4], 2U, "ELEVATOR_FAILURE_CLEANUP_UNPROVEN") &&
    value.errors[0].detail ==
    "navigation, mapping, and docking must be idle before elevator execution" &&
    value.errors[1].detail == "no frozen elevator release is bound" &&
    value.errors[2].detail == "no frozen elevator release is bound" &&
    value.errors[3].detail == "no frozen elevator release is bound" &&
    value.errors[4].detail == value.detail;
  return events_match && errors_match;
}

bool deployed_clean_complete_history_matches(
  const ElevatorExecutionSnapshot & value)
{
  const bool historical_execution_lease_shape =
    value.effect_sequence == 28U;
  const bool no_execution_lease_shape = value.effect_sequence == 26U;
  if (
    (!historical_execution_lease_shape && !no_execution_lease_shape) ||
    value.events.size() != 2U * value.effect_sequence + 4U ||
    !value.errors.empty())
  {
    return false;
  }
  std::size_t index = 0U;
  const auto consume =
    [&value, &index](const std::uint64_t sequence, const char * code) {
      if (
        index >= value.events.size() ||
        !record_matches(value.events[index], sequence, code))
      {
        return false;
      }
      ++index;
      return true;
    };
  if (
    !consume(0U, "RELEASE_PINNED") ||
    !consume(1U, "RUNTIME_PREPARE_INTENT") ||
    !consume(0U, "RUNTIME_PREPARED"))
  {
    return false;
  }
  for (
    std::uint64_t sequence = 1U;
    sequence <= value.effect_sequence;
    ++sequence)
  {
    const bool manual_confirmation = historical_execution_lease_shape ?
      (sequence == 4U || sequence == 8U || sequence == 12U ||
      sequence == 15U || sequence == 16U) :
      (sequence == 3U || sequence == 7U || sequence == 11U ||
      sequence == 14U || sequence == 15U);
    if (
      !consume(
        sequence,
        manual_confirmation ?
        "OPERATOR_CONFIRMATION_REQUIRED" : "RUNTIME_EFFECT_INTENT") ||
      !consume(
        sequence,
        manual_confirmation ?
        "OPERATOR_CONFIRMATION_ACCEPTED" : "RUNTIME_EFFECT_SUCCEEDED"))
    {
      return false;
    }
  }
  return consume(value.effect_sequence, "ELEVATOR_EXECUTION_COMPLETE") &&
         index == value.events.size();
}

bool deployed_legacy_identity_matches(
  const ElevatorExecutionSnapshot & value)
{
  return safe_asset_id(value.pinned_release_id) &&
         !value.frozen_release_identity.empty() &&
         value.pinned_release_generation > 0U &&
         safe_asset_id(value.building_id) &&
         safe_asset_id(value.elevator_id) &&
         safe_asset_id(value.source_floor_id) &&
         safe_asset_id(value.source_map_id) &&
         value.source_asset_epoch > 0U &&
         canonical_asset_digest(value.source_asset_digest) &&
         safe_asset_id(value.target_floor_id) &&
         safe_asset_id(value.target_map_id) &&
         value.target_asset_epoch > 0U &&
         canonical_asset_digest(value.target_asset_digest) &&
         value.source_floor_id != value.target_floor_id &&
         safe_asset_id(value.operator_id) &&
         !value.created_at.empty() && !value.updated_at.empty();
}

bool deployed_preflight_orphan_shape(
  const ElevatorExecutionSnapshot & value,
  const std::uint32_t snapshot_schema_version)
{
  return (
    snapshot_schema_version == kLegacySnapshotSchemaVersion ||
    snapshot_schema_version == kPreviousSnapshotSchemaVersion) &&
         value.state == "LOCKED" && value.phase == "LOCKED" &&
         value.terminal && value.effect_sequence == 2U &&
         !value.awaiting_confirmation &&
         value.expected_confirmation.empty() &&
         !value.safety_hold_state_known &&
         !value.dual_odom_stop_proven &&
         value.runtime_capable &&
         !value.runtime_applied &&
         !value.runtime_resources_reconciled &&
         !value.motion_authorized &&
         value.floor_switch_capable &&
         value.current_floor_id == value.source_floor_id &&
         value.current_map_id == value.source_map_id &&
         value.failure_code == "ELEVATOR_FAILURE_CLEANUP_UNPROVEN" &&
         value.detail ==
         "failure cleanup remained unproven after 3 attempts; "
         "original_failure=ELEVATOR_SOURCE_RUNTIME_BUSY; "
         "last_error=no frozen elevator release is bound" &&
         value.failure_origin_state.empty() &&
         value.failure_origin_effect_kind == "NONE" &&
         deployed_legacy_identity_matches(value) &&
         deployed_preflight_orphan_history_matches(value);
}

bool deployed_clean_complete_shape(
  const ElevatorExecutionSnapshot & value,
  const std::uint32_t snapshot_schema_version)
{
  const bool legacy_hold_absence =
    !value.safety_hold_active &&
    (
    snapshot_schema_version == kLegacySnapshotSchemaVersion ||
    value.safety_hold_state_known);
  return (
    snapshot_schema_version == kLegacySnapshotSchemaVersion ||
    snapshot_schema_version == kPreviousSnapshotSchemaVersion) &&
         value.state == "COMPLETE" && value.phase == "COMPLETE" &&
         value.terminal &&
         (value.effect_sequence == 28U || value.effect_sequence == 26U) &&
         !value.awaiting_confirmation &&
         value.expected_confirmation.empty() &&
         legacy_hold_absence &&
         !value.dual_odom_stop_proven &&
         value.runtime_capable &&
         value.runtime_applied &&
         !value.runtime_resources_reconciled &&
         !value.motion_authorized &&
         value.floor_switch_capable &&
         value.current_floor_id == value.target_floor_id &&
         value.current_map_id == value.target_map_id &&
         value.failure_code.empty() &&
         value.detail ==
         "all automatic effects and five confirmations completed" &&
         value.failure_origin_state.empty() &&
         value.failure_origin_effect_kind == "NONE" &&
         deployed_legacy_identity_matches(value) &&
         deployed_clean_complete_history_matches(value);
}

struct AuditedInterruptionCheckpoint
{
  std::string state;
  std::string expected_confirmation;
  ElevatorPhysicalZone physical_zone{ElevatorPhysicalZone::kUnknown};
};

std::optional<AuditedInterruptionCheckpoint>
infer_interruption_checkpoint_from_audit(
  const ElevatorExecutionSnapshot & value)
{
  for (auto event = value.events.crbegin(); event != value.events.crend(); ++event) {
    if (event->code != "OPERATOR_CONFIRMATION_REQUIRED") {
      continue;
    }
    if (event->detail == "CALL_BUTTON_PRESSED") {
      return AuditedInterruptionCheckpoint{
        "PRESSING_CALL_BUTTON", event->detail,
        ElevatorPhysicalZone::kSourceOutside};
    }
    if (event->detail == "SOURCE_DOOR_OPEN") {
      return AuditedInterruptionCheckpoint{
        "WAITING_SOURCE_DOOR", event->detail,
        ElevatorPhysicalZone::kSourceOutside};
    }
    if (event->detail == "TARGET_BUTTON_PRESSED") {
      return AuditedInterruptionCheckpoint{
        "PRESSING_TARGET_BUTTON", event->detail,
        ElevatorPhysicalZone::kCabin};
    }
    if (event->detail == "TARGET_FLOOR_ARRIVED") {
      return AuditedInterruptionCheckpoint{
        "RIDING", event->detail, ElevatorPhysicalZone::kCabin};
    }
    if (event->detail == "TARGET_DOOR_OPEN") {
      return AuditedInterruptionCheckpoint{
        "WAITING_TARGET_DOOR", event->detail,
        ElevatorPhysicalZone::kCabin};
    }
  }
  return std::nullopt;
}

std::optional<ElevatorExecutionSnapshot> parse_snapshot(
  const YAML::Node & node,
  const std::uint32_t snapshot_schema_version)
{
  if (!node || !node.IsMap()) {
    return std::nullopt;
  }
  ElevatorExecutionSnapshot value;
  value.transaction_id = scalar_or<std::string>(node, "transaction_id", "");
  value.state = scalar_or<std::string>(node, "state", "");
  value.phase = scalar_or<std::string>(node, "phase", "");
  value.effect_sequence = scalar_or<std::uint64_t>(node, "effect_sequence", 0U);
  value.awaiting_confirmation =
    scalar_or<bool>(node, "awaiting_confirmation", false);
  value.expected_confirmation =
    scalar_or<std::string>(node, "expected_confirmation", "");
  value.current_floor_id =
    scalar_or<std::string>(node, "current_floor_id", "");
  value.current_map_id = scalar_or<std::string>(node, "current_map_id", "");
  value.physical_zone = physical_zone_from_string(
    scalar_or<std::string>(node, "physical_zone", "UNKNOWN")).value_or(
    ElevatorPhysicalZone::kUnknown);
  value.interrupted_state =
    scalar_or<std::string>(node, "interrupted_state", "");
  value.interrupted_expected_confirmation = scalar_or<std::string>(
    node, "interrupted_expected_confirmation", "");
  value.safety_hold_state_known =
    scalar_or<bool>(node, "safety_hold_state_known", false);
  value.safety_hold_active =
    scalar_or<bool>(node, "safety_hold_active", false);
  value.dual_odom_stop_proven =
    scalar_or<bool>(node, "dual_odom_stop_proven", false);
  value.runtime_capable = scalar_or<bool>(node, "runtime_capable", false);
  // A missing legacy field is UNKNOWN, not proof that execution never
  // started. Conservatively make such a journal ineligible for automatic
  // preflight-orphan recovery.
  value.runtime_applied = scalar_or<bool>(node, "runtime_applied", true);
  value.runtime_resources_reconciled =
    scalar_or<bool>(node, "runtime_resources_reconciled", false);
  value.motion_authorized =
    scalar_or<bool>(node, "motion_authorized", false);
  value.floor_switch_capable =
    scalar_or<bool>(node, "floor_switch_capable", false);
  value.terminal = scalar_or<bool>(node, "terminal", false);
  value.pinned_release_id =
    scalar_or<std::string>(node, "pinned_release_id", "");
  value.frozen_release_identity =
    scalar_or<std::string>(node, "frozen_release_identity", "");
  value.pinned_release_generation =
    scalar_or<std::uint64_t>(node, "pinned_release_generation", 0U);
  value.building_id = scalar_or<std::string>(node, "building_id", "");
  value.elevator_id = scalar_or<std::string>(node, "elevator_id", "");
  value.source_floor_id =
    scalar_or<std::string>(node, "source_floor_id", "");
  value.source_map_id = scalar_or<std::string>(node, "source_map_id", "");
  value.source_asset_epoch =
    scalar_or<std::uint64_t>(node, "source_asset_epoch", 0U);
  value.source_asset_digest =
    scalar_or<std::string>(node, "source_asset_digest", "");
  value.target_floor_id =
    scalar_or<std::string>(node, "target_floor_id", "");
  value.target_map_id = scalar_or<std::string>(node, "target_map_id", "");
  value.target_asset_epoch =
    scalar_or<std::uint64_t>(node, "target_asset_epoch", 0U);
  value.target_asset_digest =
    scalar_or<std::string>(node, "target_asset_digest", "");
  value.operator_id = scalar_or<std::string>(node, "operator_id", "");
  value.created_at = scalar_or<std::string>(node, "created_at", "");
  value.updated_at = scalar_or<std::string>(node, "updated_at", "");
  value.failure_code = scalar_or<std::string>(node, "failure_code", "");
  value.detail = scalar_or<std::string>(node, "detail", "");
  value.failure_origin_state =
    scalar_or<std::string>(node, "failure_origin_state", "");
  value.failure_origin_effect_kind =
    scalar_or<std::string>(node, "failure_origin_effect_kind", "NONE");
  value.events = parse_records(node["events"]);
  value.errors = parse_records(node["errors"]);
  // Deployed journals created before the durable interruption fields existed
  // can still carry a cryptographically covered operator-confirmation audit.
  // Recover only the last exact, whitelisted checkpoint and never overwrite
  // explicit fields written by the current schema.
  if (
    (value.state == "LOCKED" || value.state == "FAILURE_CLEANUP") &&
    (value.interrupted_state.empty() ||
    value.interrupted_expected_confirmation.empty() ||
    value.physical_zone == ElevatorPhysicalZone::kUnknown))
  {
    if (const auto inferred = infer_interruption_checkpoint_from_audit(value)) {
      if (value.interrupted_state.empty()) {
        value.interrupted_state = inferred->state;
      }
      if (value.interrupted_expected_confirmation.empty()) {
        value.interrupted_expected_confirmation =
          inferred->expected_confirmation;
      }
      if (value.physical_zone == ElevatorPhysicalZone::kUnknown) {
        value.physical_zone = inferred->physical_zone;
      }
    }
  }
  const auto parsed_disposition = cleanup_disposition_from_string(
    scalar_or<std::string>(node, "cleanup_disposition", ""));
  if (snapshot_schema_version == kSnapshotSchemaVersion) {
    if (!parsed_disposition) {
      return std::nullopt;
    }
    value.cleanup_disposition = *parsed_disposition;
  } else if (
    snapshot_schema_version == kLegacySnapshotSchemaVersion ||
    snapshot_schema_version == kPreviousSnapshotSchemaVersion)
  {
    // Only the exact deployed preflight-orphan shape is safe to migrate to
    // explicit outside recovery. This includes the field-observed v1 journal
    // whose prepare never bound a release and whose cleanup consequently
    // failed with "no frozen elevator release is bound". Ignore any
    // cleanup_disposition field a legacy journal happens to contain: those
    // schemas never authenticated that field, so trusting it would let a
    // forged SOURCE_OUTSIDE/TARGET_OUTSIDE value bypass the whitelist. Every
    // other legacy shape remains physically ambiguous and retains the lock.
    if (deployed_preflight_orphan_shape(
        value, snapshot_schema_version))
    {
      value.cleanup_disposition =
        ElevatorCleanupDisposition::kSourceOutside;
      value.failure_origin_state = "RUNTIME_PREPARING";
    } else if (deployed_clean_complete_shape(
        value, snapshot_schema_version))
    {
      // Old COMPLETE was emitted only after all owned resources and the hold
      // were released. Preserve that historical safe terminal instead of
      // introducing a permanent recovery interlock during schema migration.
      value.cleanup_disposition =
        ElevatorCleanupDisposition::kTargetOutside;
      value.failure_origin_state.clear();
      value.safety_hold_state_known = true;
      value.safety_hold_active = false;
      value.runtime_resources_reconciled = true;
    } else {
      value.cleanup_disposition =
        ElevatorCleanupDisposition::kRetainLock;
      value.failure_origin_state = "LEGACY_UNKNOWN";
      value.runtime_resources_reconciled = false;
    }
  } else {
    return std::nullopt;
  }
  if (snapshot_schema_version == kSnapshotSchemaVersion) {
    if (
      !required_scalar(node, "transaction_id", value.transaction_id) ||
      !required_scalar(node, "state", value.state) ||
      !required_scalar(node, "effect_sequence", value.effect_sequence) ||
      !required_boolean(
        node, "safety_hold_state_known", value.safety_hold_state_known) ||
      !required_boolean(node, "safety_hold_active", value.safety_hold_active) ||
      !required_boolean(
        node, "dual_odom_stop_proven", value.dual_odom_stop_proven) ||
      !required_boolean(node, "runtime_capable", value.runtime_capable) ||
      !required_boolean(node, "runtime_applied", value.runtime_applied) ||
      !required_boolean(
        node, "runtime_resources_reconciled",
        value.runtime_resources_reconciled) ||
      !required_boolean(node, "motion_authorized", value.motion_authorized) ||
      !required_boolean(
        node, "floor_switch_capable", value.floor_switch_capable) ||
      !required_boolean(node, "terminal", value.terminal) ||
      !required_scalar(
        node, "failure_origin_state", value.failure_origin_state) ||
      !required_scalar(
        node, "failure_origin_effect_kind",
        value.failure_origin_effect_kind) ||
      !parsed_disposition)
    {
      return std::nullopt;
    }
    const bool state_is_terminal = terminal_state(value.state);
    if (
      value.effect_sequence == 0U ||
      state_is_terminal != value.terminal ||
      (!value.safety_hold_state_known && !value.safety_hold_active) ||
      (value.terminal && value.motion_authorized))
    {
      return std::nullopt;
    }
  }
  if (!safe_asset_id(value.transaction_id) || value.state.empty()) {
    return std::nullopt;
  }
  return value;
}

enum class JournalWriteOutcome
{
  kCommitted,
  kNotCommitted,
  // The target rename completed, but parent-directory durability could not be
  // proven. Callers must assume either the old or the new journal may survive.
  kCommitUncertain,
};

bool normalize_journal_path(
  const fs::path & path,
  fs::path & absolute_path,
  std::string & filename,
  std::string & error)
{
  if (path.empty()) {
    error = "journal path is empty";
    return false;
  }
  std::error_code filesystem_error;
  absolute_path = fs::absolute(path, filesystem_error).lexically_normal();
  if (filesystem_error || !absolute_path.is_absolute()) {
    error = "cannot resolve absolute journal path";
    if (filesystem_error) {
      error += ": " + filesystem_error.message();
    }
    return false;
  }
  const auto leaf = absolute_path.filename();
  if (leaf.empty() || leaf == "." || leaf == "..") {
    error = "journal path does not name a file";
    return false;
  }
  filename = leaf.string();
  return true;
}

#ifndef _WIN32
enum class SecureDirectoryOpenOutcome
{
  kOpened,
  kNotFound,
  kError,
};

std::string errno_message(const std::string & action, const int error_number)
{
  return action + ": " +
         std::error_code(error_number, std::generic_category()).message();
}

SecureDirectoryOpenOutcome open_real_directory(
  const fs::path & absolute_directory,
  const bool create_missing,
  int & directory_fd,
  std::string & error)
{
  directory_fd = -1;
  if (!absolute_directory.is_absolute()) {
    error = "journal directory is not absolute";
    return SecureDirectoryOpenOutcome::kError;
  }

  int current_fd =
    ::open("/", O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  if (current_fd < 0) {
    error = errno_message("cannot open filesystem root", errno);
    return SecureDirectoryOpenOutcome::kError;
  }

  for (const auto & component : absolute_directory.relative_path()) {
    const auto name = component.string();
    if (name.empty() || name == ".") {
      continue;
    }
    if (name == "..") {
      error = "journal directory contains a parent traversal";
      (void)::close(current_fd);
      return SecureDirectoryOpenOutcome::kError;
    }

    bool entry_was_missing = false;
    int next_fd = ::openat(
      current_fd, name.c_str(),
      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (next_fd < 0 && errno == ENOENT) {
      if (!create_missing) {
        if (::close(current_fd) != 0) {
          error = errno_message(
            "journal directory lookup close failed", errno);
          return SecureDirectoryOpenOutcome::kError;
        }
        return SecureDirectoryOpenOutcome::kNotFound;
      }
      entry_was_missing = true;
      if (::mkdirat(current_fd, name.c_str(), 0700) != 0 && errno != EEXIST) {
        const int mkdir_error = errno;
        (void)::close(current_fd);
        error = errno_message(
          "cannot create journal directory component " + name, mkdir_error);
        return SecureDirectoryOpenOutcome::kError;
      }
      next_fd = ::openat(
        current_fd, name.c_str(),
        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    }
    if (next_fd < 0) {
      const int open_error = errno;
      (void)::close(current_fd);
      error = errno_message(
        "journal directory component is not a real directory: " + name,
        open_error);
      return SecureDirectoryOpenOutcome::kError;
    }

    struct stat directory_status {};
    const bool status_ok = ::fstat(next_fd, &directory_status) == 0;
    const int status_error = status_ok ? ENOTDIR : errno;
    if (!status_ok || !S_ISDIR(directory_status.st_mode)) {
      (void)::close(next_fd);
      (void)::close(current_fd);
      error = errno_message(
        "cannot prove journal directory component: " + name,
        status_error == 0 ? ENOTDIR : status_error);
      return SecureDirectoryOpenOutcome::kError;
    }

    if (entry_was_missing) {
      if (::fsync(next_fd) != 0 || ::fsync(current_fd) != 0) {
        const int sync_error = errno;
        (void)::close(next_fd);
        (void)::close(current_fd);
        error = errno_message(
          "cannot durably create journal directory component: " + name,
          sync_error);
        return SecureDirectoryOpenOutcome::kError;
      }
    }
    if (::close(current_fd) != 0) {
      const int close_error = errno;
      (void)::close(next_fd);
      error = errno_message(
        "journal directory traversal close failed", close_error);
      return SecureDirectoryOpenOutcome::kError;
    }
    current_fd = next_fd;
  }

  directory_fd = current_fd;
  return SecureDirectoryOpenOutcome::kOpened;
}

bool same_real_directory(
  const fs::path & absolute_directory,
  const int expected_fd,
  std::string & error)
{
  struct stat expected_status {};
  if (::fstat(expected_fd, &expected_status) != 0) {
    error = errno_message(
      "cannot inspect opened journal directory", errno);
    return false;
  }

  int current_fd = -1;
  const auto outcome =
    open_real_directory(absolute_directory, false, current_fd, error);
  if (outcome != SecureDirectoryOpenOutcome::kOpened) {
    if (outcome == SecureDirectoryOpenOutcome::kNotFound) {
      error = "journal directory disappeared during operation";
    }
    return false;
  }

  struct stat current_status {};
  const bool status_ok = ::fstat(current_fd, &current_status) == 0;
  const int status_error = errno;
  const bool close_ok = ::close(current_fd) == 0;
  const int close_error = errno;
  if (!status_ok) {
    error = errno_message(
      "cannot revalidate journal directory", status_error);
    return false;
  }
  if (!close_ok) {
    error = errno_message(
      "journal directory revalidation close failed", close_error);
    return false;
  }
  if (
    expected_status.st_dev != current_status.st_dev ||
    expected_status.st_ino != current_status.st_ino)
  {
    error = "journal directory identity changed during operation";
    return false;
  }
  return true;
}
#endif

JournalWriteOutcome atomic_write_yaml(
  const fs::path & path,
  const ElevatorExecutionSnapshot & snapshot,
  std::string & error)
{
  fs::path absolute_path;
  std::string filename;
  if (!normalize_journal_path(path, absolute_path, filename, error)) {
    return JournalWriteOutcome::kNotCommitted;
  }
  YAML::Node root;
  root["schema_version"] = kJournalEnvelopeSchemaVersion;
  root["snapshot_schema_version"] = kSnapshotSchemaVersion;
  root["snapshot"] = snapshot_node(snapshot);
  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    error = emitter.GetLastError();
    return JournalWriteOutcome::kNotCommitted;
  }
  std::string payload(emitter.c_str(), emitter.size());
  payload.push_back('\n');
  if (payload.size() > kMaximumJournalBytes) {
    error = "serialized journal exceeds size limit";
    return JournalWriteOutcome::kNotCommitted;
  }

  const auto nonce =
    std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temporary_name =
    filename + ".tmp." + std::to_string(nonce);

#ifdef _WIN32
  std::error_code filesystem_error;
  const auto journal_directory = absolute_path.parent_path();
  fs::create_directories(journal_directory, filesystem_error);
  if (filesystem_error) {
    error = "cannot create journal directory: " + filesystem_error.message();
    return JournalWriteOutcome::kNotCommitted;
  }
  filesystem_error.clear();
  const auto target_status =
    fs::symlink_status(absolute_path, filesystem_error);
  if (
    filesystem_error &&
    target_status.type() != fs::file_type::not_found)
  {
    error = "cannot inspect journal target: " + filesystem_error.message();
    return JournalWriteOutcome::kNotCommitted;
  }
  if (
    target_status.type() != fs::file_type::not_found &&
    (fs::is_symlink(target_status) || !fs::is_regular_file(target_status)))
  {
    error = "journal target is not a regular file";
    return JournalWriteOutcome::kNotCommitted;
  }
  const auto temporary = journal_directory / temporary_name;
  HANDLE journal_handle = CreateFileW(
    temporary.wstring().c_str(),
    GENERIC_WRITE,
    0,
    nullptr,
    CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH,
    nullptr);
  if (journal_handle == INVALID_HANDLE_VALUE) {
    error = "cannot exclusively create temporary journal";
    return JournalWriteOutcome::kNotCommitted;
  }
  bool write_ok = true;
  std::size_t offset = 0U;
  while (offset < payload.size()) {
    const auto remaining = payload.size() - offset;
    const DWORD requested = static_cast<DWORD>(
      std::min<std::size_t>(remaining, std::numeric_limits<DWORD>::max()));
    DWORD written = 0U;
    if (
      !WriteFile(
        journal_handle, payload.data() + offset, requested, &written,
        nullptr) ||
      written == 0U)
    {
      write_ok = false;
      break;
    }
    offset += written;
  }
  if (write_ok && !FlushFileBuffers(journal_handle)) {
    write_ok = false;
  }
  if (!CloseHandle(journal_handle)) {
    write_ok = false;
  }
  if (!write_ok) {
    error = "cannot durably write temporary journal";
    fs::remove(temporary, filesystem_error);
    return JournalWriteOutcome::kNotCommitted;
  }
  if (!MoveFileExW(
      temporary.wstring().c_str(), absolute_path.wstring().c_str(),
      MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
  {
    error = "cannot atomically replace journal";
    fs::remove(temporary, filesystem_error);
    return JournalWriteOutcome::kNotCommitted;
  }
  return JournalWriteOutcome::kCommitted;
#else
  const auto journal_directory = absolute_path.parent_path();
  int directory_fd = -1;
  const auto directory_outcome =
    open_real_directory(journal_directory, true, directory_fd, error);
  if (directory_outcome != SecureDirectoryOpenOutcome::kOpened) {
    if (directory_outcome == SecureDirectoryOpenOutcome::kNotFound) {
      error = "journal directory could not be created";
    }
    return JournalWriteOutcome::kNotCommitted;
  }

  struct stat existing_status {};
  if (
    ::fstatat(
      directory_fd, filename.c_str(), &existing_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    if (errno != ENOENT) {
      const int inspect_error = errno;
      (void)::close(directory_fd);
      error = errno_message("cannot inspect journal target", inspect_error);
      return JournalWriteOutcome::kNotCommitted;
    }
  } else if (!S_ISREG(existing_status.st_mode)) {
    (void)::close(directory_fd);
    error = "journal target is not a regular file";
    return JournalWriteOutcome::kNotCommitted;
  }

  int journal_fd = ::openat(
    directory_fd, temporary_name.c_str(),
    O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
    0600);
  if (journal_fd < 0) {
    const int open_error = errno;
    (void)::close(directory_fd);
    error = errno_message(
      "cannot exclusively create temporary journal", open_error);
    return JournalWriteOutcome::kNotCommitted;
  }

  struct stat written_status {};
  bool write_ok = true;
  std::size_t offset = 0U;
  while (offset < payload.size()) {
    const ssize_t written =
      ::write(journal_fd, payload.data() + offset, payload.size() - offset);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      write_ok = false;
      break;
    }
    offset += static_cast<std::size_t>(written);
  }
  if (write_ok && ::fsync(journal_fd) != 0) {
    write_ok = false;
  }
  if (write_ok && ::fstat(journal_fd, &written_status) != 0) {
    write_ok = false;
  }
  if (::close(journal_fd) != 0) {
    write_ok = false;
  }
  journal_fd = -1;
  if (!write_ok) {
    (void)::unlinkat(directory_fd, temporary_name.c_str(), 0);
    (void)::close(directory_fd);
    error = "cannot durably write temporary journal";
    return JournalWriteOutcome::kNotCommitted;
  }
  if (
    ::renameat(
      directory_fd, temporary_name.c_str(),
      directory_fd, filename.c_str()) != 0)
  {
    const int rename_error = errno;
    (void)::unlinkat(directory_fd, temporary_name.c_str(), 0);
    (void)::close(directory_fd);
    error = errno_message("cannot atomically replace journal", rename_error);
    return JournalWriteOutcome::kNotCommitted;
  }
  if (::fsync(directory_fd) != 0) {
    const int sync_error = errno;
    (void)::close(directory_fd);
    error = errno_message(
      "journal replacement completed but parent directory durable flush failed",
      sync_error);
    return JournalWriteOutcome::kCommitUncertain;
  }

  struct stat installed_status {};
  if (
    ::fstatat(
      directory_fd, filename.c_str(), &installed_status,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    !S_ISREG(installed_status.st_mode) ||
    installed_status.st_dev != written_status.st_dev ||
    installed_status.st_ino != written_status.st_ino)
  {
    (void)::close(directory_fd);
    error =
      "journal replacement completed but target identity changed before "
      "commit verification";
    return JournalWriteOutcome::kCommitUncertain;
  }
  if (!same_real_directory(journal_directory, directory_fd, error)) {
    (void)::close(directory_fd);
    error =
      "journal replacement completed but configured directory identity "
      "could not be revalidated: " + error;
    return JournalWriteOutcome::kCommitUncertain;
  }
  if (::close(directory_fd) != 0) {
    error =
      "journal replacement completed but parent directory close failed";
    return JournalWriteOutcome::kCommitUncertain;
  }
  return JournalWriteOutcome::kCommitted;
#endif
}

enum class JournalReadOutcome
{
  kLoaded,
  kNotFound,
  kError,
};

JournalReadOutcome read_journal_payload(
  const fs::path & path,
  std::string & payload,
  std::string & error)
{
  fs::path absolute_path;
  std::string filename;
  if (!normalize_journal_path(path, absolute_path, filename, error)) {
    return JournalReadOutcome::kError;
  }
  const auto journal_directory = absolute_path.parent_path();

#ifdef _WIN32
  std::error_code filesystem_error;
  const auto parent_status =
    fs::symlink_status(journal_directory, filesystem_error);
  if (
    parent_status.type() == fs::file_type::not_found &&
    (!filesystem_error ||
    filesystem_error == std::errc::no_such_file_or_directory))
  {
    return JournalReadOutcome::kNotFound;
  }
  if (filesystem_error) {
    error = "cannot inspect journal parent: " + filesystem_error.message();
    return JournalReadOutcome::kError;
  }
  if (fs::is_symlink(parent_status) || !fs::is_directory(parent_status)) {
    error = "journal parent is not a real directory";
    return JournalReadOutcome::kError;
  }

  filesystem_error.clear();
  const auto target_status =
    fs::symlink_status(absolute_path, filesystem_error);
  if (
    target_status.type() == fs::file_type::not_found &&
    (!filesystem_error ||
    filesystem_error == std::errc::no_such_file_or_directory))
  {
    return JournalReadOutcome::kNotFound;
  }
  if (filesystem_error) {
    error = "cannot inspect journal target: " + filesystem_error.message();
    return JournalReadOutcome::kError;
  }
  if (fs::is_symlink(target_status) || !fs::is_regular_file(target_status)) {
    error = "journal target is not a regular file";
    return JournalReadOutcome::kError;
  }
  const auto size = fs::file_size(absolute_path, filesystem_error);
  if (filesystem_error) {
    error = "cannot inspect journal size: " + filesystem_error.message();
    return JournalReadOutcome::kError;
  }
  if (size > kMaximumJournalBytes) {
    error = "journal exceeds size limit";
    return JournalReadOutcome::kError;
  }
  std::ifstream input(absolute_path, std::ios::binary);
  if (!input.good()) {
    error = "cannot open journal for reading";
    return JournalReadOutcome::kError;
  }
  payload.assign(
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>());
  if (input.bad()) {
    error = "cannot read journal";
    return JournalReadOutcome::kError;
  }
  if (payload.size() > kMaximumJournalBytes) {
    error = "journal exceeds size limit";
    return JournalReadOutcome::kError;
  }
  return JournalReadOutcome::kLoaded;
#else
  int directory_fd = -1;
  const auto directory_outcome =
    open_real_directory(journal_directory, false, directory_fd, error);
  if (directory_outcome == SecureDirectoryOpenOutcome::kNotFound) {
    return JournalReadOutcome::kNotFound;
  }
  if (directory_outcome != SecureDirectoryOpenOutcome::kOpened) {
    return JournalReadOutcome::kError;
  }

  struct stat preopen_status {};
  if (
    ::fstatat(
      directory_fd, filename.c_str(), &preopen_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    const int inspect_error = errno;
    const bool close_ok = ::close(directory_fd) == 0;
    if (!close_ok) {
      error = errno_message(
        "journal directory close failed after target inspection", errno);
      return JournalReadOutcome::kError;
    }
    if (inspect_error == ENOENT) {
      return JournalReadOutcome::kNotFound;
    }
    error = errno_message("cannot inspect journal target", inspect_error);
    return JournalReadOutcome::kError;
  }
  if (!S_ISREG(preopen_status.st_mode)) {
    (void)::close(directory_fd);
    error = "journal target is not a regular file";
    return JournalReadOutcome::kError;
  }
  if (
    preopen_status.st_size < 0 ||
    static_cast<std::uintmax_t>(preopen_status.st_size) >
    kMaximumJournalBytes)
  {
    (void)::close(directory_fd);
    error = "journal exceeds size limit";
    return JournalReadOutcome::kError;
  }

  const int journal_fd = ::openat(
    directory_fd, filename.c_str(),
    O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  if (journal_fd < 0) {
    const int open_error = errno;
    const bool close_ok = ::close(directory_fd) == 0;
    if (!close_ok) {
      error = errno_message(
        "journal directory close failed after target lookup", errno);
      return JournalReadOutcome::kError;
    }
    if (open_error == ENOENT) {
      return JournalReadOutcome::kNotFound;
    }
    error = errno_message("cannot open journal target", open_error);
    return JournalReadOutcome::kError;
  }

  struct stat initial_status {};
  if (::fstat(journal_fd, &initial_status) != 0) {
    const int status_error = errno;
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = errno_message("cannot inspect opened journal", status_error);
    return JournalReadOutcome::kError;
  }
  if (!S_ISREG(initial_status.st_mode)) {
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = "journal target is not a regular file";
    return JournalReadOutcome::kError;
  }
  if (
    initial_status.st_size < 0 ||
    static_cast<std::uintmax_t>(initial_status.st_size) >
    kMaximumJournalBytes ||
    initial_status.st_dev != preopen_status.st_dev ||
    initial_status.st_ino != preopen_status.st_ino)
  {
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error =
      initial_status.st_size < 0 ||
      static_cast<std::uintmax_t>(initial_status.st_size) >
      kMaximumJournalBytes ?
      "journal exceeds size limit" :
      "journal identity changed while it was opened";
    return JournalReadOutcome::kError;
  }

  payload.clear();
  payload.reserve(static_cast<std::size_t>(initial_status.st_size));
  char buffer[8192];
  bool read_ok = true;
  while (payload.size() <= kMaximumJournalBytes) {
    const auto remaining =
      static_cast<std::size_t>(kMaximumJournalBytes + 1U - payload.size());
    const auto requested = std::min<std::size_t>(sizeof(buffer), remaining);
    const ssize_t count = ::read(journal_fd, buffer, requested);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      read_ok = false;
      break;
    }
    if (count == 0) {
      break;
    }
    payload.append(buffer, static_cast<std::size_t>(count));
  }
  if (!read_ok) {
    const int read_error = errno;
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = errno_message("cannot read journal", read_error);
    return JournalReadOutcome::kError;
  }
  if (payload.size() > kMaximumJournalBytes) {
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = "journal exceeds size limit";
    return JournalReadOutcome::kError;
  }

  struct stat final_status {};
  struct stat path_status {};
  const bool file_stable =
    ::fstat(journal_fd, &final_status) == 0 &&
    S_ISREG(final_status.st_mode) &&
    final_status.st_dev == initial_status.st_dev &&
    final_status.st_ino == initial_status.st_ino &&
    final_status.st_size == initial_status.st_size &&
    static_cast<std::uintmax_t>(final_status.st_size) == payload.size();
  const bool path_stable =
    ::fstatat(
      directory_fd, filename.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) == 0 &&
    S_ISREG(path_status.st_mode) &&
    path_status.st_dev == initial_status.st_dev &&
    path_status.st_ino == initial_status.st_ino;
  if (!file_stable || !path_stable) {
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = "journal identity or size changed while it was read";
    return JournalReadOutcome::kError;
  }
  if (!same_real_directory(journal_directory, directory_fd, error)) {
    (void)::close(journal_fd);
    (void)::close(directory_fd);
    error = "journal directory changed while it was read: " + error;
    return JournalReadOutcome::kError;
  }
  if (::close(journal_fd) != 0) {
    const int close_error = errno;
    (void)::close(directory_fd);
    error = errno_message("journal close failed", close_error);
    return JournalReadOutcome::kError;
  }
  if (::close(directory_fd) != 0) {
    error = errno_message("journal directory close failed", errno);
    return JournalReadOutcome::kError;
  }
  return JournalReadOutcome::kLoaded;
#endif
}

fs::path recovery_audit_path(
  const fs::path & journal_path,
  const ElevatorExecutionSnapshot & snapshot,
  const std::string & checkpoint)
{
  const fs::path journal_directory =
    journal_path.parent_path().empty() ? fs::path(".") :
    journal_path.parent_path();
  return journal_directory / "history" /
         (snapshot.transaction_id + "." +
         std::to_string(snapshot.effect_sequence) + "." +
         checkpoint + ".yaml");
}

bool recovery_complete_checkpoint(const std::string & checkpoint)
{
  return checkpoint == "recovery_complete" ||
         checkpoint == "cleanup_complete";
}

bool recovery_complete_snapshots_equivalent(
  const ElevatorExecutionSnapshot & left,
  const ElevatorExecutionSnapshot & right)
{
  const auto safe_complete = [](const ElevatorExecutionSnapshot & value) {
      return value.terminal && !value.awaiting_confirmation &&
             value.expected_confirmation.empty() &&
             value.safety_hold_state_known && !value.safety_hold_active &&
             value.dual_odom_stop_proven &&
             value.runtime_resources_reconciled &&
             !value.motion_authorized &&
             (value.state == "FAILED" || value.state == "CANCELLED");
    };
  return safe_complete(left) && safe_complete(right) &&
         left.transaction_id == right.transaction_id &&
         left.state == right.state &&
         left.phase == right.phase &&
         left.effect_sequence == right.effect_sequence &&
         left.current_floor_id == right.current_floor_id &&
         left.current_map_id == right.current_map_id &&
         left.runtime_capable == right.runtime_capable &&
         left.runtime_applied == right.runtime_applied &&
         left.floor_switch_capable == right.floor_switch_capable &&
         left.pinned_release_id == right.pinned_release_id &&
         left.frozen_release_identity == right.frozen_release_identity &&
         left.pinned_release_generation == right.pinned_release_generation &&
         left.building_id == right.building_id &&
         left.elevator_id == right.elevator_id &&
         left.source_floor_id == right.source_floor_id &&
         left.source_map_id == right.source_map_id &&
         left.source_asset_epoch == right.source_asset_epoch &&
         left.source_asset_digest == right.source_asset_digest &&
         left.target_floor_id == right.target_floor_id &&
         left.target_map_id == right.target_map_id &&
         left.target_asset_epoch == right.target_asset_epoch &&
         left.target_asset_digest == right.target_asset_digest &&
         left.operator_id == right.operator_id &&
         left.created_at == right.created_at &&
         left.failure_code == right.failure_code &&
         left.failure_origin_state == right.failure_origin_state &&
         left.failure_origin_effect_kind == right.failure_origin_effect_kind &&
         left.cleanup_disposition == right.cleanup_disposition;
}

bool recovery_audit_semantics_match(
  const std::string & existing,
  const std::string & checkpoint,
  const ElevatorExecutionSnapshot & snapshot,
  std::string & error)
{
  try {
    const auto existing_root = YAML::Load(existing);
    const auto audit_snapshot_schema = scalar_or<std::uint32_t>(
      existing_root, "snapshot_schema_version",
      kLegacySnapshotSchemaVersion);
    const auto existing_snapshot =
      parse_snapshot(
      existing_root["snapshot"], audit_snapshot_schema);
    const bool snapshot_matches =
      existing_snapshot &&
      (
      YAML::Dump(snapshot_node(*existing_snapshot)) ==
      YAML::Dump(snapshot_node(snapshot)) ||
      (recovery_complete_checkpoint(checkpoint) &&
      recovery_complete_snapshots_equivalent(*existing_snapshot, snapshot)));
    if (
      scalar_or<std::uint32_t>(
        existing_root, "schema_version", 0U) != 1U ||
      scalar_or<std::string>(
        existing_root, "record_type", "") !=
      "elevator_explicit_recovery_audit" ||
      scalar_or<std::string>(
        existing_root, "checkpoint", "") != checkpoint ||
      scalar_or<std::string>(
        existing_root, "transaction_id", "") !=
      snapshot.transaction_id ||
      scalar_or<std::uint64_t>(
        existing_root, "effect_sequence", 0U) !=
      snapshot.effect_sequence ||
      !snapshot_matches)
    {
      error = "immutable recovery audit semantics conflict with checkpoint";
      return false;
    }
    return true;
  } catch (const YAML::Exception &) {
    error = "immutable recovery audit is not valid YAML";
    return false;
  }
}

#ifndef _WIN32
bool verify_existing_recovery_audit_at(
  const int history_fd,
  const std::string & filename,
  const std::string & payload,
  const std::string & checkpoint,
  const ElevatorExecutionSnapshot & snapshot,
  std::string & error)
{
  const bool semantic_complete =
    recovery_complete_checkpoint(checkpoint);
  struct stat path_status {};
  if (
    ::fstatat(
      history_fd, filename.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    error = errno_message(
      "cannot inspect immutable recovery audit", errno);
    return false;
  }
  if (
    !S_ISREG(path_status.st_mode) ||
    path_status.st_nlink != 1 ||
    path_status.st_size < 0 ||
    static_cast<std::uintmax_t>(path_status.st_size) >
    kMaximumJournalBytes ||
    (!semantic_complete &&
    static_cast<std::uintmax_t>(path_status.st_size) != payload.size()))
  {
    error = "immutable recovery audit target has unsafe file identity";
    return false;
  }

  const int existing_fd = ::openat(
    history_fd, filename.c_str(),
    O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  if (existing_fd < 0) {
    error = errno_message(
      "cannot open immutable recovery audit", errno);
    return false;
  }
  struct stat opened_status {};
  if (
    ::fstat(existing_fd, &opened_status) != 0 ||
    !S_ISREG(opened_status.st_mode) ||
    opened_status.st_nlink != 1 ||
    opened_status.st_dev != path_status.st_dev ||
    opened_status.st_ino != path_status.st_ino)
  {
    (void)::close(existing_fd);
    error = "immutable recovery audit identity changed while it was opened";
    return false;
  }

  std::string existing;
  existing.reserve(static_cast<std::size_t>(path_status.st_size));
  char buffer[8192];
  while (existing.size() <= kMaximumJournalBytes) {
    const auto remaining =
      static_cast<std::size_t>(
      kMaximumJournalBytes + 1U - existing.size());
    const auto requested = std::min<std::size_t>(sizeof(buffer), remaining);
    const ssize_t count = ::read(existing_fd, buffer, requested);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      const int read_error = errno;
      (void)::close(existing_fd);
      error = errno_message(
        "cannot read immutable recovery audit", read_error);
      return false;
    }
    if (count == 0) {
      break;
    }
    existing.append(buffer, static_cast<std::size_t>(count));
  }
  if (
    existing.size() > kMaximumJournalBytes ||
    (!semantic_complete && existing != payload) ||
    ::fsync(existing_fd) != 0)
  {
    (void)::close(existing_fd);
    error = "immutable recovery audit content conflicts with checkpoint";
    return false;
  }

  struct stat final_status {};
  struct stat final_path_status {};
  const bool identity_stable =
    ::fstat(existing_fd, &final_status) == 0 &&
    ::fstatat(
      history_fd, filename.c_str(), &final_path_status,
      AT_SYMLINK_NOFOLLOW) == 0 &&
    S_ISREG(final_status.st_mode) &&
    final_status.st_nlink == 1 &&
    final_status.st_dev == opened_status.st_dev &&
    final_status.st_ino == opened_status.st_ino &&
    final_status.st_size == opened_status.st_size &&
    final_status.st_size >= 0 &&
    static_cast<std::uintmax_t>(final_status.st_size) == existing.size() &&
    final_path_status.st_dev == opened_status.st_dev &&
    final_path_status.st_ino == opened_status.st_ino &&
    final_path_status.st_size == opened_status.st_size;
  if (::close(existing_fd) != 0 || !identity_stable) {
    error = "immutable recovery audit identity changed while it was verified";
    return false;
  }
  return recovery_audit_semantics_match(
    existing, checkpoint, snapshot, error);
}

bool flush_and_revalidate_recovery_audit_directories(
  const fs::path & history_directory,
  const int history_fd,
  std::string & error)
{
  struct stat history_status {};
  if (::fstat(history_fd, &history_status) != 0) {
    error = errno_message(
      "cannot inspect recovery audit directory", errno);
    return false;
  }

  const auto journal_directory = history_directory.parent_path();
  int journal_directory_fd = -1;
  const auto parent_outcome =
    open_real_directory(
    journal_directory, false, journal_directory_fd, error);
  if (parent_outcome != SecureDirectoryOpenOutcome::kOpened) {
    if (parent_outcome == SecureDirectoryOpenOutcome::kNotFound) {
      error = "recovery audit parent directory disappeared";
    }
    return false;
  }

  struct stat history_path_status {};
  if (
    ::fstatat(
      journal_directory_fd,
      history_directory.filename().c_str(),
      &history_path_status,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    !S_ISDIR(history_path_status.st_mode) ||
    history_path_status.st_dev != history_status.st_dev ||
    history_path_status.st_ino != history_status.st_ino)
  {
    (void)::close(journal_directory_fd);
    error = "recovery audit directory identity changed before durable flush";
    return false;
  }
  if (::fsync(history_fd) != 0 || ::fsync(journal_directory_fd) != 0) {
    const int sync_error = errno;
    (void)::close(journal_directory_fd);
    error = errno_message(
      "cannot durably flush recovery audit directories", sync_error);
    return false;
  }
  if (
    !same_real_directory(history_directory, history_fd, error) ||
    !same_real_directory(
      journal_directory, journal_directory_fd, error))
  {
    (void)::close(journal_directory_fd);
    return false;
  }
  if (::close(journal_directory_fd) != 0) {
    error = errno_message(
      "cannot close recovery audit parent directory", errno);
    return false;
  }
  return true;
}

bool immutable_write_recovery_audit_posix(
  const fs::path & history_directory,
  const std::string & filename,
  const std::string & payload,
  const std::string & checkpoint,
  const ElevatorExecutionSnapshot & snapshot,
  std::string & error)
{
  int history_fd = -1;
  const auto history_outcome =
    open_real_directory(history_directory, true, history_fd, error);
  if (history_outcome != SecureDirectoryOpenOutcome::kOpened) {
    if (history_outcome == SecureDirectoryOpenOutcome::kNotFound) {
      error = "recovery audit directory could not be created";
    }
    return false;
  }

  struct stat target_status {};
  if (
    ::fstatat(
      history_fd, filename.c_str(), &target_status,
      AT_SYMLINK_NOFOLLOW) == 0)
  {
    const bool verified = verify_existing_recovery_audit_at(
      history_fd, filename, payload, checkpoint, snapshot, error);
    const bool directories_durable =
      verified &&
      flush_and_revalidate_recovery_audit_directories(
      history_directory, history_fd, error);
    const bool close_ok = ::close(history_fd) == 0;
    if (!close_ok && directories_durable) {
      error = errno_message(
        "cannot close recovery audit directory", errno);
    }
    return directories_durable && close_ok;
  }
  if (errno != ENOENT) {
    const int inspect_error = errno;
    (void)::close(history_fd);
    error = errno_message(
      "cannot inspect immutable recovery audit target", inspect_error);
    return false;
  }

  const auto nonce =
    std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temporary_name =
    filename + ".tmp." + std::to_string(nonce);
  int audit_fd = ::openat(
    history_fd, temporary_name.c_str(),
    O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
    0600);
  if (audit_fd < 0) {
    const int open_error = errno;
    (void)::close(history_fd);
    error = errno_message(
      "cannot exclusively create temporary recovery audit", open_error);
    return false;
  }

  struct stat written_status {};
  bool write_ok = true;
  std::size_t offset = 0U;
  while (offset < payload.size()) {
    const ssize_t written =
      ::write(audit_fd, payload.data() + offset, payload.size() - offset);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      write_ok = false;
      break;
    }
    offset += static_cast<std::size_t>(written);
  }
  if (write_ok && ::fsync(audit_fd) != 0) {
    write_ok = false;
  }
  if (write_ok && ::fstat(audit_fd, &written_status) != 0) {
    write_ok = false;
  }
  if (::close(audit_fd) != 0) {
    write_ok = false;
  }
  if (!write_ok) {
    (void)::unlinkat(history_fd, temporary_name.c_str(), 0);
    (void)::close(history_fd);
    error = "cannot durably write temporary recovery audit";
    return false;
  }

  bool audit_installed = false;
  bool target_raced_into_place = false;
  bool temporary_still_exists = true;
#if defined(SYS_renameat2) && defined(RENAME_NOREPLACE)
  if (
    ::syscall(
      SYS_renameat2,
      history_fd,
      temporary_name.c_str(),
      history_fd,
      filename.c_str(),
      RENAME_NOREPLACE) == 0)
  {
    audit_installed = true;
    temporary_still_exists = false;
  } else if (errno == EEXIST) {
    target_raced_into_place = true;
  } else if (errno != ENOSYS && errno != EINVAL) {
    const int rename_error = errno;
    (void)::unlinkat(history_fd, temporary_name.c_str(), 0);
    (void)::close(history_fd);
    error = errno_message(
      "cannot atomically install immutable recovery audit", rename_error);
    return false;
  }
#endif
  if (!audit_installed && !target_raced_into_place) {
    if (
      ::linkat(
        history_fd, temporary_name.c_str(),
        history_fd, filename.c_str(), 0) == 0)
    {
      audit_installed = true;
    } else if (errno == EEXIST) {
      target_raced_into_place = true;
    } else {
      const int link_error = errno;
      (void)::unlinkat(history_fd, temporary_name.c_str(), 0);
      (void)::close(history_fd);
      error = errno_message(
        "cannot install immutable recovery audit", link_error);
      return false;
    }
  }
  if (
    temporary_still_exists &&
    ::unlinkat(history_fd, temporary_name.c_str(), 0) != 0)
  {
    (void)::close(history_fd);
    error = errno_message(
      "cannot remove recovery audit installation link", errno);
    return false;
  }

  bool target_verified = false;
  if (target_raced_into_place) {
    target_verified = verify_existing_recovery_audit_at(
      history_fd, filename, payload, checkpoint, snapshot, error);
  } else {
    struct stat installed_status {};
    target_verified =
      ::fstatat(
      history_fd, filename.c_str(), &installed_status,
      AT_SYMLINK_NOFOLLOW) == 0 &&
      S_ISREG(installed_status.st_mode) &&
      installed_status.st_nlink == 1 &&
      installed_status.st_dev == written_status.st_dev &&
      installed_status.st_ino == written_status.st_ino;
    if (!target_verified) {
      error = "installed recovery audit identity could not be proven";
    }
  }
  const bool directories_durable =
    target_verified &&
    flush_and_revalidate_recovery_audit_directories(
    history_directory, history_fd, error);
  const bool close_ok = ::close(history_fd) == 0;
  if (!close_ok && directories_durable) {
    error = errno_message(
      "cannot close recovery audit directory", errno);
  }
  return directories_durable && close_ok;
}
#endif

bool immutable_write_recovery_audit(
  const fs::path & journal_path,
  const std::string & checkpoint,
  const ElevatorExecutionSnapshot & snapshot,
  std::string & error)
{
  if (
    checkpoint != "recovery_release_pending" &&
    checkpoint != "recovery_complete" &&
    checkpoint != "cleanup_release_pending" &&
    checkpoint != "cleanup_complete")
  {
    error = "unsupported recovery audit checkpoint";
    return false;
  }
  if (
    !safe_asset_id(snapshot.transaction_id) ||
    snapshot.effect_sequence == 0U ||
    snapshot.updated_at.empty())
  {
    error = "recovery audit identity is invalid";
    return false;
  }

  const fs::path target =
    recovery_audit_path(journal_path, snapshot, checkpoint);
  const fs::path history_directory = target.parent_path();
  YAML::Node root;
  root["schema_version"] = 1U;
  root["snapshot_schema_version"] = kSnapshotSchemaVersion;
  root["record_type"] = "elevator_explicit_recovery_audit";
  root["checkpoint"] = checkpoint;
  // Keep the representation deterministic so a crash/retry can accept only
  // the exact immutable record that this checkpoint would have created.
  root["recorded_at"] = snapshot.updated_at;
  root["transaction_id"] = snapshot.transaction_id;
  root["effect_sequence"] = snapshot.effect_sequence;
  root["snapshot"] = snapshot_node(snapshot);
  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    error = emitter.GetLastError();
    return false;
  }
  std::string payload(emitter.c_str(), emitter.size());
  payload.push_back('\n');
  if (payload.size() > kMaximumJournalBytes) {
    error = "serialized recovery audit exceeds size limit";
    return false;
  }

#ifndef _WIN32
  return immutable_write_recovery_audit_posix(
    history_directory, target.filename().string(), payload,
    checkpoint, snapshot, error);
#else
  std::error_code filesystem_error;
  const auto history_status =
    fs::symlink_status(history_directory, filesystem_error);
  if (
    history_status.type() == fs::file_type::not_found &&
    (!filesystem_error ||
    filesystem_error == std::errc::no_such_file_or_directory))
  {
    filesystem_error.clear();
    fs::create_directory(history_directory, filesystem_error);
    if (filesystem_error) {
      error =
        "cannot create recovery audit directory: " +
        filesystem_error.message();
      return false;
    }
  } else if (
    filesystem_error || fs::is_symlink(history_status) ||
    !fs::is_directory(history_status))
  {
    error = "recovery audit directory is not a real directory";
    return false;
  }

  filesystem_error.clear();
  const auto target_status = fs::symlink_status(target, filesystem_error);
  if (
    filesystem_error &&
    target_status.type() != fs::file_type::not_found)
  {
    error =
      "cannot inspect immutable recovery audit target: " +
      filesystem_error.message();
    return false;
  }
  if (target_status.type() != fs::file_type::not_found) {
    if (
      fs::is_symlink(target_status) ||
      !fs::is_regular_file(target_status))
    {
      error = "immutable recovery audit target is not a regular file";
      return false;
    }
    filesystem_error.clear();
    const auto existing_size = fs::file_size(target, filesystem_error);
    if (
      filesystem_error ||
      existing_size > kMaximumJournalBytes ||
      (!recovery_complete_checkpoint(checkpoint) &&
      existing_size != payload.size()))
    {
      error = "immutable recovery audit target has unsafe file identity";
      return false;
    }
    filesystem_error.clear();
    if (
      fs::hard_link_count(target, filesystem_error) != 1U ||
      filesystem_error)
    {
      error = "immutable recovery audit target has unsafe file identity";
      return false;
    }
    std::ifstream existing_input(target, std::ios::binary);
    const std::string existing{
      std::istreambuf_iterator<char>(existing_input),
      std::istreambuf_iterator<char>()};
    if (
      existing_input.bad() ||
      (!recovery_complete_checkpoint(checkpoint) && existing != payload) ||
      !recovery_audit_semantics_match(
        existing, checkpoint, snapshot, error))
    {
      if (error.empty()) {
        error = "immutable recovery audit content conflicts with checkpoint";
      }
      return false;
    }
    return true;
  }

  const auto nonce =
    std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path temporary =
    history_directory /
    (target.filename().string() + ".tmp." + std::to_string(nonce));
  HANDLE audit_handle = CreateFileW(
    temporary.wstring().c_str(),
    GENERIC_WRITE,
    0,
    nullptr,
    CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH,
    nullptr);
  if (audit_handle == INVALID_HANDLE_VALUE) {
    error = "cannot exclusively create temporary recovery audit";
    return false;
  }
  bool write_ok = true;
  std::size_t offset = 0U;
  while (offset < payload.size()) {
    const auto remaining = payload.size() - offset;
    const DWORD requested = static_cast<DWORD>(
      std::min<std::size_t>(remaining, std::numeric_limits<DWORD>::max()));
    DWORD written = 0U;
    if (
      !WriteFile(
        audit_handle, payload.data() + offset, requested, &written, nullptr) ||
      written == 0U)
    {
      write_ok = false;
      break;
    }
    offset += written;
  }
  if (write_ok && !FlushFileBuffers(audit_handle)) {
    write_ok = false;
  }
  if (!CloseHandle(audit_handle)) {
    write_ok = false;
  }
  if (!write_ok) {
    error = "cannot durably write temporary recovery audit";
    fs::remove(temporary, filesystem_error);
    return false;
  }
  if (!MoveFileExW(
      temporary.wstring().c_str(), target.wstring().c_str(),
      MOVEFILE_WRITE_THROUGH))
  {
    error = "cannot install immutable recovery audit";
    fs::remove(temporary, filesystem_error);
    return false;
  }
#endif
  return true;
}

ElevatorExecutionReply reply(
  const ElevatorExecutionReplyKind kind,
  std::string code,
  std::string detail,
  const std::optional<ElevatorExecutionSnapshot> & snapshot = std::nullopt)
{
  return ElevatorExecutionReply{
    kind,
    std::move(code),
    std::move(detail),
    snapshot,
  };
}

}  // namespace

bool snapshot_may_have_submitted_floor_switch_action(
  const ElevatorExecutionSnapshot & snapshot) noexcept
{
  const auto floor_action_intent =
    [](const ElevatorExecutionEventRecord & event) {
      return event.code == "RUNTIME_EFFECT_INTENT" &&
             (event.detail == "BEGIN_FLOOR_TRANSITION" ||
             event.detail == "SWITCH_FLOOR");
    };
  if (
    std::any_of(
      snapshot.events.cbegin(), snapshot.events.cend(),
      [&snapshot, &floor_action_intent](
        const ElevatorExecutionEventRecord & event) {
        return event.sequence <= snapshot.effect_sequence &&
               floor_action_intent(event);
      }))
  {
    return true;
  }

  if (
    snapshot.failure_origin_effect_kind == "BEGIN_FLOOR_TRANSITION" ||
    snapshot.failure_origin_effect_kind == "SWITCH_FLOOR" ||
    snapshot.failure_origin_effect_kind == "VERIFY_FLOOR_READY")
  {
    return true;
  }

  const auto & checkpoint = snapshot.failure_origin_state.empty() ?
    snapshot.interrupted_state : snapshot.failure_origin_state;
  static const std::vector<std::string> pre_floor_action_states{
    "RUNTIME_PREPARING",
    "NAVIGATING_HALL_CALL",
    "ACQUIRING_HALL_HOLD",
    "ACQUIRING_EXECUTION_LEASE",
    "PRESSING_CALL_BUTTON",
    "SETTING_ELEVATOR_WAIT_MODE",
    "RELEASING_HALL_HOLD",
    "NAVIGATING_SOURCE_LANDING",
    "WAITING_SOURCE_DOOR",
    "SETTING_DOORWAY_ENTRY_MODE",
    "ENTERING_CABIN",
    "NAVIGATING_CABIN_PANEL",
    "ACQUIRING_CABIN_HOLD",
    "PRESSING_TARGET_BUTTON",
    "PAUSING_CORRECTIONS",
    "SETTING_RIDE_MODE",
    "RIDING",
    "WAITING_TARGET_DOOR",
  };
  return std::find(
    pre_floor_action_states.cbegin(), pre_floor_action_states.cend(),
    checkpoint) == pre_floor_action_states.cend();
}

ElevatorCleanupDisposition cleanup_disposition_for(
  const ElevatorState state) noexcept
{
  switch (state) {
    case ElevatorState::kIdle:
    case ElevatorState::kNavigatingHallCall:
    case ElevatorState::kAcquiringHallHold:
    case ElevatorState::kAcquiringExecutionLease:
    case ElevatorState::kPressingCallButton:
    case ElevatorState::kSettingElevatorWaitMode:
    case ElevatorState::kReleasingHallHold:
    case ElevatorState::kNavigatingSourceLanding:
    case ElevatorState::kWaitingSourceDoor:
    case ElevatorState::kSettingDoorwayEntryMode:
    case ElevatorState::kEnteringCabin:
    case ElevatorState::kNavigatingCabinPanel:
    case ElevatorState::kAcquiringCabinHold:
    case ElevatorState::kPressingTargetButton:
    case ElevatorState::kPausingCorrections:
    case ElevatorState::kSettingRideMode:
    case ElevatorState::kRiding:
    case ElevatorState::kWaitingTargetDoor:
    case ElevatorState::kBeginningFloorTransition:
    case ElevatorState::kResumingCorrections:
    case ElevatorState::kSwitchingFloor:
      // Until SwitchFloor succeeds, the active runtime identity is still the
      // source floor.  Ordinary execution/cancellation failures are cleaned
      // against that identity and terminate as FAILED/CANCELLED; they do not
      // create a permanent elevator lock.
      return ElevatorCleanupDisposition::kSourceOutside;
    case ElevatorState::kVerifyingFloorReady:
    case ElevatorState::kSettingDoorwayExitMode:
    case ElevatorState::kReturningToCabinCenter:
    case ElevatorState::kReleasingCabinHold:
    case ElevatorState::kNavigatingTargetLanding:
    case ElevatorState::kAcquiringExitHold:
    case ElevatorState::kReleasingOperatingMode:
    case ElevatorState::kReleasingExecutionLease:
    case ElevatorState::kReleasingExitHold:
    case ElevatorState::kComplete:
      // A successful SwitchFloor effect updates the runtime identity before
      // these states are entered, so cleanup is performed on the target.
      return ElevatorCleanupDisposition::kTargetOutside;
    case ElevatorState::kFailureCleanup:
    case ElevatorState::kLocked:
      return ElevatorCleanupDisposition::kRetainLock;
  }
  return ElevatorCleanupDisposition::kRetainLock;
}

std::string to_string(const ElevatorCleanupDisposition disposition)
{
  switch (disposition) {
    case ElevatorCleanupDisposition::kSourceOutside:
      return "SOURCE_OUTSIDE";
    case ElevatorCleanupDisposition::kTargetOutside:
      return "TARGET_OUTSIDE";
    case ElevatorCleanupDisposition::kRetainLock:
      return "RETAIN_LOCK";
  }
  return "RETAIN_LOCK";
}

ElevatorPhysicalZone physical_zone_for(const ElevatorState state) noexcept
{
  switch (state) {
    case ElevatorState::kIdle:
    case ElevatorState::kNavigatingHallCall:
    case ElevatorState::kAcquiringHallHold:
    case ElevatorState::kAcquiringExecutionLease:
    case ElevatorState::kPressingCallButton:
    case ElevatorState::kSettingElevatorWaitMode:
    case ElevatorState::kReleasingHallHold:
    case ElevatorState::kNavigatingSourceLanding:
    case ElevatorState::kWaitingSourceDoor:
    case ElevatorState::kSettingDoorwayEntryMode:
      return ElevatorPhysicalZone::kSourceOutside;
    case ElevatorState::kEnteringCabin:
    case ElevatorState::kNavigatingTargetLanding:
      return ElevatorPhysicalZone::kDoorway;
    case ElevatorState::kNavigatingCabinPanel:
    case ElevatorState::kAcquiringCabinHold:
    case ElevatorState::kPressingTargetButton:
    case ElevatorState::kPausingCorrections:
    case ElevatorState::kSettingRideMode:
    case ElevatorState::kRiding:
    case ElevatorState::kWaitingTargetDoor:
    case ElevatorState::kBeginningFloorTransition:
    case ElevatorState::kResumingCorrections:
    case ElevatorState::kSwitchingFloor:
    case ElevatorState::kVerifyingFloorReady:
    case ElevatorState::kSettingDoorwayExitMode:
    case ElevatorState::kReturningToCabinCenter:
    case ElevatorState::kReleasingCabinHold:
      return ElevatorPhysicalZone::kCabin;
    case ElevatorState::kAcquiringExitHold:
    case ElevatorState::kReleasingOperatingMode:
    case ElevatorState::kReleasingExecutionLease:
    case ElevatorState::kReleasingExitHold:
    case ElevatorState::kComplete:
      return ElevatorPhysicalZone::kTargetOutside;
    case ElevatorState::kFailureCleanup:
    case ElevatorState::kLocked:
      return ElevatorPhysicalZone::kUnknown;
  }
  return ElevatorPhysicalZone::kUnknown;
}

std::string to_string(const ElevatorPhysicalZone zone)
{
  switch (zone) {
    case ElevatorPhysicalZone::kUnknown:
      return "UNKNOWN";
    case ElevatorPhysicalZone::kSourceOutside:
      return "SOURCE_OUTSIDE";
    case ElevatorPhysicalZone::kDoorway:
      return "DOORWAY";
    case ElevatorPhysicalZone::kCabin:
      return "CABIN";
    case ElevatorPhysicalZone::kTargetOutside:
      return "TARGET_OUTSIDE";
  }
  return "UNKNOWN";
}

std::vector<std::string> allowed_recovery_actions(
  const ElevatorExecutionSnapshot & snapshot)
{
  if (
    snapshot.state != "LOCKED" || !snapshot.terminal ||
    !snapshot.safety_hold_state_known || !snapshot.safety_hold_active ||
    snapshot.source_floor_id.empty() ||
    snapshot.current_floor_id != snapshot.source_floor_id)
  {
    return {};
  }
  if (has_durable_source_outside_confirmation(snapshot)) {
    return {"RETRY_SAFETY_VERIFICATION"};
  }
  return {"CONFIRM_SOURCE_OUTSIDE_AND_RELEASE"};
}

InMemoryElevatorRuntimePort::InMemoryElevatorRuntimePort(
  const ElevatorRuntimeCapabilities capabilities)
: capabilities_(capabilities)
{
}

ElevatorRuntimeCapabilities
InMemoryElevatorRuntimePort::capabilities() const noexcept
{
  return capabilities_;
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::prepare(
  const std::string &,
  const FrozenElevatorRelease & release)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto result = take_result(queued_prepare_results_);
  if (result.success) {
    prepared_release_ = release;
  }
  return result;
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::apply(
  const ElevatorRuntimeEffect & effect)
{
  std::lock_guard<std::mutex> lock(mutex_);
  applied_effects_.push_back(effect);
  auto & queued = queued_results_[effect.effect.kind];
  if (!queued.empty()) {
    return take_result(queued);
  }
  auto result = success_result();
  // The deterministic test adapter explicitly models the safety evidence a
  // production adapter must return. Tests can still queue a successful result
  // with either proof bit false to verify that the module never invents it.
  if (
    effect.effect.kind == ElevatorEffectKind::kNavigateToPose ||
    effect.effect.kind == ElevatorEffectKind::kHoldAndCancel)
  {
    result.safety_hold_proven = true;
    result.dual_odom_stop_proven = true;
    if (effect.effect.kind == ElevatorEffectKind::kHoldAndCancel) {
      result.runtime_resources_reconciled =
        effect.cleanup_disposition !=
        ElevatorCleanupDisposition::kRetainLock;
    }
  } else if (effect.effect.kind == ElevatorEffectKind::kAcquireSafetyHold) {
    result.safety_hold_proven = true;
  } else if (effect.effect.kind == ElevatorEffectKind::kReleaseSafetyHold) {
    result.safety_hold_absence_proven = true;
  }
  return result;
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::recover_locked(
  const ElevatorRuntimeCleanupContext & context)
{
  request_cancel(context.transaction_id);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queued_recovery_results_.empty()) {
    return take_result(queued_recovery_results_);
  }
  auto result = ElevatorRuntimeResult{
    true,
    "OK",
    "restart recovery retained hold and proved dual odometry stopped",
    true,
    true,
  };
  result.runtime_resources_reconciled =
    context.disposition != ElevatorCleanupDisposition::kRetainLock;
  return result;
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::finalize_recovery(
  const ElevatorRuntimeCleanupContext &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queued_recovery_finalize_results_.empty()) {
    return take_result(queued_recovery_finalize_results_);
  }
  prepared_release_.reset();
  auto result = success_result();
  result.dual_odom_stop_proven = true;
  result.safety_hold_absence_proven = true;
  result.runtime_resources_reconciled = true;
  result.detail =
    "recovery owner hold absence and dual odometry stop proven";
  return result;
}

void InMemoryElevatorRuntimePort::request_cancel(
  const std::string & transaction_id) noexcept
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    cancellation_requests_.push_back(transaction_id);
  } catch (...) {
    // The non-blocking cancellation seam is noexcept by contract.
  }
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::heartbeat(const std::string &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return take_result(queued_heartbeat_results_);
}

ElevatorRuntimeResult InMemoryElevatorRuntimePort::poll_health(const std::string &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return take_result(queued_health_results_);
}

void InMemoryElevatorRuntimePort::queue_result(
  const ElevatorEffectKind kind,
  ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_results_[kind].push_back(std::move(result));
}

void InMemoryElevatorRuntimePort::queue_prepare_result(ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_prepare_results_.push_back(std::move(result));
}

void InMemoryElevatorRuntimePort::queue_recovery_result(
  ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_recovery_results_.push_back(std::move(result));
}

void InMemoryElevatorRuntimePort::queue_recovery_finalize_result(
  ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_recovery_finalize_results_.push_back(std::move(result));
}

void InMemoryElevatorRuntimePort::queue_heartbeat_result(ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_heartbeat_results_.push_back(std::move(result));
}

void InMemoryElevatorRuntimePort::queue_health_result(ElevatorRuntimeResult result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queued_health_results_.push_back(std::move(result));
}

std::vector<ElevatorRuntimeEffect>
InMemoryElevatorRuntimePort::applied_effects() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return applied_effects_;
}

std::optional<FrozenElevatorRelease>
InMemoryElevatorRuntimePort::prepared_release() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return prepared_release_;
}

std::vector<std::string>
InMemoryElevatorRuntimePort::cancellation_requests() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cancellation_requests_;
}

class ElevatorExecutionModule::Implementation
{
public:
  enum class RecoveryStage
  {
    kNone,
    kVerify,
    kRebindThenFinalize,
    kFinalize,
  };

  Implementation(
    fs::path journal_path,
    std::shared_ptr<ElevatorRuntimePort> runtime_port,
    ElevatorExecutionOptions options)
  : journal_path_(std::move(journal_path)),
    runtime_port_(std::move(runtime_port)),
    options_(options),
    fsm_(ElevatorFsmOptions{true})
  {
    load_journal();
    if (restart_recovery_pending_) {
      work_pending_ = true;
    }
    worker_ = std::thread([this]() {worker_loop();});
    condition_.notify_one();
  }

  ~Implementation()
  {
    std::string active_transaction;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot_ && !snapshot_->terminal) {
        active_transaction = snapshot_->transaction_id;
      }
      stop_ = true;
      condition_.notify_all();
    }
    if (runtime_port_ && !active_transaction.empty()) {
      runtime_port_->request_cancel(active_transaction);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  ElevatorExecutionReply start(const ElevatorExecutionStart & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      !safe_asset_id(command.transaction_id) ||
      !safe_asset_id(command.operator_id) ||
      !valid_release(command.release))
    {
      return reply(
        ElevatorExecutionReplyKind::kInvalid,
        "INVALID_ELEVATOR_EXECUTION_START",
        "transaction/operator/release identity is invalid");
    }
    if (!initialization_error_.empty()) {
      return reply(
        ElevatorExecutionReplyKind::kStorageFailure,
        "ELEVATOR_EXECUTION_JOURNAL_UNAVAILABLE",
        initialization_error_,
        snapshot_);
    }
    if (snapshot_) {
      if (snapshot_->transaction_id == command.transaction_id) {
        const bool same =
          snapshot_->operator_id == command.operator_id &&
          snapshot_->frozen_release_identity ==
          frozen_release_identity(command.release);
        return same ?
          reply(
          ElevatorExecutionReplyKind::kAccepted,
          "ELEVATOR_EXECUTION_REPLAYED",
          "existing transaction snapshot returned",
          snapshot_) :
          reply(
          ElevatorExecutionReplyKind::kConflict,
          "ELEVATOR_EXECUTION_TRANSACTION_ID_REUSED",
          "transaction_id identifies a different frozen request",
          snapshot_);
      }
      if (!snapshot_->terminal) {
        return reply(
          ElevatorExecutionReplyKind::kConflict,
          "ELEVATOR_EXECUTION_ACTIVE",
          "another elevator transaction is still active",
          snapshot_);
      }
      if (
        options_.persistent_recovery_lock_enabled &&
        (snapshot_->state == "LOCKED" ||
        !snapshot_->safety_hold_state_known ||
        snapshot_->safety_hold_active ||
        (snapshot_->runtime_applied &&
        !snapshot_->runtime_resources_reconciled)))
      {
        return reply(
          ElevatorExecutionReplyKind::kConflict,
          "ELEVATOR_EXECUTION_RECOVERY_REQUIRED",
          "the retained terminal transaction still owns recovery state; "
          "reconcile it before starting another transaction",
          snapshot_);
      }
    }

    const auto capabilities = runtime_port_ ?
      runtime_port_->capabilities() : ElevatorRuntimeCapabilities{};
    automatic_button_control_ = capabilities.automatic_button_control;
    frozen_release_ = command.release;
    fsm_ = ElevatorFsm(ElevatorFsmOptions{true});
    pending_effect_.reset();
    preparing_ = false;
    work_pending_ = false;
    in_flight_sequence_ = 0U;
    cleanup_attempt_count_ = 0U;
    cancellation_requested_ = false;
    runtime_side_effects_may_exist_ = false;
    original_failure_code_.clear();
    original_failure_detail_.clear();

    ElevatorExecutionSnapshot next;
    next.transaction_id = command.transaction_id;
    next.state = "RUNTIME_PREPARING";
    next.phase = "RUNTIME_PREPARING";
    next.effect_sequence = 1U;
    next.runtime_capable = capabilities.runtime_capable;
    next.cleanup_disposition = ElevatorCleanupDisposition::kSourceOutside;
    next.motion_authorized = capabilities.motion_authorized;
    next.floor_switch_capable = capabilities.floor_switch_capable;
    next.pinned_release_id = command.release.release_id;
    next.frozen_release_identity =
      frozen_release_identity(command.release);
    next.pinned_release_generation = command.release.generation;
    next.building_id = command.release.building_id;
    next.elevator_id = command.release.elevator_id;
    next.source_floor_id = command.release.source.floor_id;
    next.source_map_id = command.release.source.map_id;
    next.source_asset_epoch = command.release.source.map_asset_epoch;
    next.source_asset_digest = command.release.source.map_asset_digest;
    next.target_floor_id = command.release.target.floor_id;
    next.target_map_id = command.release.target.map_id;
    next.target_asset_epoch = command.release.target.map_asset_epoch;
    next.target_asset_digest = command.release.target.map_asset_digest;
    next.current_floor_id = next.source_floor_id;
    next.current_map_id = next.source_map_id;
    next.physical_zone = ElevatorPhysicalZone::kSourceOutside;
    // No runtime port has been invoked yet. This is authoritative absence,
    // and prevents a failed first journal write from creating an untracked
    // hold through storage-failure cleanup.
    next.safety_hold_state_known = true;
    next.safety_hold_active = false;
    next.operator_id = command.operator_id;
    next.created_at = timestamp_utc();
    next.updated_at = next.created_at;
    append_record(
      next.events, 0U, "RELEASE_PINNED",
      "exact immutable release and route identity pinned");
    snapshot_ = std::move(next);

    if (
      !capabilities.runtime_capable ||
      !capabilities.motion_authorized ||
      !capabilities.floor_switch_capable)
    {
      snapshot_->state = "FAILED";
      snapshot_->phase = "RUNTIME_PREFLIGHT";
      snapshot_->terminal = true;
      snapshot_->motion_authorized = false;
      snapshot_->safety_hold_state_known = true;
      snapshot_->safety_hold_active = false;
      snapshot_->failure_code = kRuntimeUnavailable;
      snapshot_->detail =
        "runtime adapter cannot prove motion and atomic floor-switch capability";
      append_record(
        snapshot_->errors, 0U, snapshot_->failure_code, snapshot_->detail);
      snapshot_->updated_at = timestamp_utc();
      if (!persist_locked()) {
        return storage_failure_locked();
      }
      return reply(
        ElevatorExecutionReplyKind::kAccepted,
        snapshot_->failure_code,
        snapshot_->detail,
        snapshot_);
    }

    append_record(
      snapshot_->events, snapshot_->effect_sequence, "RUNTIME_PREPARE_INTENT",
      "frozen release prepare intent journaled before adapter invocation");
    if (!persist_locked()) {
      return storage_failure_locked();
    }
    preparing_ = true;
    work_pending_ = true;
    condition_.notify_one();
    return reply(
      ElevatorExecutionReplyKind::kAccepted,
      "ELEVATOR_EXECUTION_STARTED",
      "runtime preparation queued",
      snapshot_);
  }

  ElevatorExecutionReply confirm(const ElevatorExecutionConfirmation & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      !safe_asset_id(command.transaction_id) ||
      !safe_asset_id(command.operator_id) ||
      !safe_asset_id(command.observed_floor_id) ||
      command.expected_state.empty() ||
      command.expected_state.size() > 64U ||
      command.event.empty() ||
      command.event.size() > 64U ||
      command.effect_sequence == 0U)
    {
      return reply(
        ElevatorExecutionReplyKind::kInvalid,
        "INVALID_ELEVATOR_EXECUTION_CONFIRMATION",
        "confirmation fields are missing or invalid");
    }
    auto conflict = matching_active_locked(command.transaction_id);
    if (conflict) {
      return *conflict;
    }
    if (!snapshot_->awaiting_confirmation) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_NOT_AWAITING_CONFIRMATION",
        "the current automatic phase has no operator gate",
        snapshot_);
    }
    const auto expected_floor =
      expected_observed_floor(snapshot_->expected_confirmation, *snapshot_);
    if (
      command.expected_state != snapshot_->state ||
      command.effect_sequence != snapshot_->effect_sequence ||
      command.event != snapshot_->expected_confirmation ||
      command.observed_floor_id != expected_floor ||
      command.operator_id != snapshot_->operator_id)
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_CONFIRMATION_MISMATCH",
        "state/sequence/event/observed_floor/operator must match the current gate",
        snapshot_);
    }

    append_record(
      snapshot_->events,
      snapshot_->effect_sequence,
      "OPERATOR_CONFIRMATION_ACCEPTED",
      command.event + ";observed_floor=" + command.observed_floor_id +
      ";operator=" + command.operator_id);
    snapshot_->awaiting_confirmation = false;
    snapshot_->expected_confirmation.clear();
    const auto output = fsm_.dispatch(
      ElevatorEvent{
        ElevatorEventKind::kEffectSucceeded,
        command.event,
        command.effect_sequence,
        command.transaction_id,
      });
    if (!output.accepted) {
      begin_failure_locked(
        "ELEVATOR_FSM_CONFIRMATION_REJECTED", output.message, false);
    } else {
      accept_output_locked(output);
    }
    if (!persist_locked()) {
      return storage_failure_locked();
    }
    condition_.notify_one();
    return reply(
      ElevatorExecutionReplyKind::kAccepted,
      "ELEVATOR_EXECUTION_CONFIRMATION_ACCEPTED",
      "confirmation accepted and next automatic phase queued",
      snapshot_);
  }

  ElevatorExecutionReply cancel(const ElevatorExecutionCancellation & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      !safe_asset_id(command.transaction_id) ||
      !safe_asset_id(command.operator_id) ||
      command.effect_sequence == 0U ||
      !bounded_reason(command.reason))
    {
      return reply(
        ElevatorExecutionReplyKind::kInvalid,
        "INVALID_ELEVATOR_EXECUTION_CANCELLATION",
        "cancellation fields are missing or invalid");
    }
    if (!snapshot_ || snapshot_->transaction_id != command.transaction_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_TRANSACTION_NOT_FOUND",
        "transaction is not retained",
        snapshot_);
    }
    if (command.operator_id != snapshot_->operator_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_OPERATOR_MISMATCH",
        "only the starting operator may cancel this transaction",
        snapshot_);
    }
    if (snapshot_->terminal) {
      return reply(
        ElevatorExecutionReplyKind::kAccepted,
        "ELEVATOR_EXECUTION_TERMINAL",
        "terminal cancellation is idempotent",
        snapshot_);
    }
    if (command.effect_sequence != snapshot_->effect_sequence) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_EFFECT_SEQUENCE_MISMATCH",
        "cancellation sequence is stale",
        snapshot_);
    }
    if (snapshot_->state == "FAILURE_CLEANUP" && pending_effect_) {
      append_record(
        snapshot_->events,
        snapshot_->effect_sequence,
        "FAILURE_CLEANUP_ALREADY_AUTOMATIC",
        "bounded cleanup retry is already owned by the execution worker");
    } else {
      begin_failure_locked(
        "ELEVATOR_EXECUTION_CANCELLED",
        command.reason.empty() ? "operator_cancelled" : command.reason,
        true);
    }
    if (!persist_locked()) {
      return storage_failure_locked();
    }
    runtime_port_->request_cancel(command.transaction_id);
    condition_.notify_one();
    return reply(
      ElevatorExecutionReplyKind::kAccepted,
      "ELEVATOR_EXECUTION_CANCELLATION_ACCEPTED",
      "failure cleanup queued",
      snapshot_);
  }

  ElevatorExecutionReply recover(const ElevatorExecutionRecovery & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool confirm_source_outside_action =
      command.action == "CONFIRM_SOURCE_OUTSIDE_AND_RELEASE";
    const bool retry_safety_verification_action =
      command.action == "RETRY_SAFETY_VERIFICATION";
    const bool legacy_action = command.action.empty();
    const bool has_source_outside_confirmation_fields =
      command.source_outside_confirmed || !command.confirmed_floor_id.empty() ||
      !command.physical_zone.empty() || command.stationary_confirmed ||
      command.door_zone_clear_confirmed;
    const bool action_shape_valid =
      (legacy_action && command.physical_zone.empty() &&
      !command.stationary_confirmed && !command.door_zone_clear_confirmed) ||
      (confirm_source_outside_action && command.source_outside_confirmed &&
      command.physical_zone == "SOURCE_OUTSIDE" &&
      command.stationary_confirmed && command.door_zone_clear_confirmed) ||
      (retry_safety_verification_action &&
      !has_source_outside_confirmation_fields);
    if (
      !safe_asset_id(command.transaction_id) ||
      !safe_asset_id(command.operator_id) ||
      command.expected_state != "LOCKED" ||
      command.effect_sequence == 0U ||
      command.reason.empty() ||
      !bounded_reason(command.reason) ||
      (command.source_outside_confirmed &&
      !safe_asset_id(command.confirmed_floor_id)) ||
      (!command.source_outside_confirmed &&
      !command.confirmed_floor_id.empty()) ||
      !action_shape_valid)
    {
      return reply(
        ElevatorExecutionReplyKind::kInvalid,
        "INVALID_ELEVATOR_EXECUTION_RECOVERY",
        "transaction/state/sequence/operator/reason or recovery action "
        "evidence is missing or invalid",
        snapshot_);
    }
    if (!initialization_error_.empty()) {
      return reply(
        ElevatorExecutionReplyKind::kStorageFailure,
        "ELEVATOR_EXECUTION_JOURNAL_UNAVAILABLE",
        initialization_error_,
        snapshot_);
    }
    if (!snapshot_ || snapshot_->transaction_id != command.transaction_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_TRANSACTION_NOT_FOUND",
        "transaction is not retained",
        snapshot_);
    }
    if (command.operator_id != snapshot_->operator_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_OPERATOR_MISMATCH",
        "only the starting operator may recover this transaction",
        snapshot_);
    }
    if (
      snapshot_->state != command.expected_state ||
      !snapshot_->terminal ||
      snapshot_->effect_sequence != command.effect_sequence)
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_SNAPSHOT_MISMATCH",
        "state and effect_sequence must match the retained lock",
        snapshot_);
    }
    if (
      restart_recovery_pending_ ||
      recovery_stage_ != RecoveryStage::kNone)
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_BUSY",
        "runtime recovery evidence is still being reconciled",
        snapshot_);
    }
    const bool retained_cabin_entry_navigation_lock =
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kRetainLock &&
      snapshot_->failure_origin_state == "ENTERING_CABIN" &&
      snapshot_->failure_origin_effect_kind == "NAVIGATE_TO_POSE";
    const auto server_actions = allowed_recovery_actions(*snapshot_);
    const auto action_allowed = [&server_actions](const std::string & action) {
        return std::find(
          server_actions.cbegin(), server_actions.cend(), action) !=
               server_actions.cend();
      };
    if (
      (confirm_source_outside_action &&
      !action_allowed("CONFIRM_SOURCE_OUTSIDE_AND_RELEASE")) ||
      (retry_safety_verification_action &&
      !action_allowed("RETRY_SAFETY_VERIFICATION")))
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_ACTION_NOT_ALLOWED",
        "the requested recovery action is not allowed by the retained "
        "server state",
        snapshot_);
    }
    if (
      legacy_action &&
      has_source_outside_confirmation_fields &&
      !retained_cabin_entry_navigation_lock)
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_SOURCE_OUTSIDE_CONFIRMATION_NOT_APPLICABLE",
        "source-outside confirmation is allowed only for a retained "
        "ENTERING_CABIN navigation failure",
        snapshot_);
    }
    if (
      command.source_outside_confirmed &&
      (command.confirmed_floor_id != snapshot_->source_floor_id ||
      snapshot_->current_floor_id != snapshot_->source_floor_id))
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_FLOOR_MISMATCH",
        "the confirmed floor and retained current floor must both match the "
        "source floor",
        snapshot_);
    }
    if (
      command.source_outside_confirmed &&
      (!snapshot_->safety_hold_state_known ||
      !snapshot_->safety_hold_active))
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_HOLD_NOT_PROVEN",
        "the exact transaction owner hold must be known active before "
        "source-outside recovery can begin",
        snapshot_);
    }
    const bool field_confirmed_source_outside =
      ((legacy_action && retained_cabin_entry_navigation_lock) ||
      confirm_source_outside_action) &&
      command.source_outside_confirmed &&
      command.confirmed_floor_id == snapshot_->source_floor_id &&
      snapshot_->current_floor_id == snapshot_->source_floor_id;
    const bool retry_recorded_source_outside =
      retry_safety_verification_action &&
      has_durable_source_outside_confirmation(*snapshot_);
    const bool legacy_preflight_orphan =
      !snapshot_->runtime_applied &&
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kSourceOutside;
    const bool safe_outside_cleanup =
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kSourceOutside ||
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kTargetOutside ||
      field_confirmed_source_outside || retry_recorded_source_outside;
    if (
      !legacy_preflight_orphan &&
      (!safe_outside_cleanup ||
      (snapshot_->runtime_resources_reconciled &&
      !field_confirmed_source_outside &&
      !retry_recorded_source_outside)))
    {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_RECOVERY_REQUIRES_SERVICE",
        snapshot_->cleanup_disposition ==
        ElevatorCleanupDisposition::kRetainLock ?
        "the failure origin may be in the cabin, doorway, or floor "
        "transition; on-site service is required" :
        "runtime resources are already reconciled; a release-pending journal "
        "is resumed automatically and cannot be restarted by the operator",
        snapshot_);
    }
    if (snapshot_->effect_sequence == std::numeric_limits<std::uint64_t>::max()) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_SEQUENCE_EXHAUSTED",
        "recovery effect sequence is exhausted",
        snapshot_);
    }

    ++snapshot_->effect_sequence;
    if (field_confirmed_source_outside) {
      snapshot_->cleanup_disposition =
        ElevatorCleanupDisposition::kSourceOutside;
      snapshot_->physical_zone = ElevatorPhysicalZone::kSourceOutside;
      append_record(
        snapshot_->events,
        snapshot_->effect_sequence,
        "ON_SITE_SOURCE_OUTSIDE_CONFIRMED",
        "operator=" + command.operator_id +
        ";floor=" + command.confirmed_floor_id +
        ";reason=" + command.reason);
    }
    snapshot_->phase = "RECOVERY_VERIFYING";
    snapshot_->terminal = true;
    snapshot_->motion_authorized = false;
    append_record(
      snapshot_->events,
      snapshot_->effect_sequence,
      "EXPLICIT_RECOVERY_INTENT",
      "operator=" + command.operator_id + ";reason=" + command.reason);
    if (!persist_locked()) {
      return storage_failure_locked();
    }
    recovery_stage_ = RecoveryStage::kVerify;
    work_pending_ = true;
    condition_.notify_one();
    return reply(
      ElevatorExecutionReplyKind::kAccepted,
      "ELEVATOR_EXECUTION_RECOVERY_ACCEPTED",
      "recovery verification queued",
      snapshot_);
  }

  ElevatorExecutionReply snapshot(const std::string & transaction_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!safe_asset_id(transaction_id)) {
      return reply(
        ElevatorExecutionReplyKind::kInvalid,
        "INVALID_ELEVATOR_EXECUTION_TRANSACTION",
        "a path-safe transaction_id is required");
    }
    if (!snapshot_ || snapshot_->transaction_id != transaction_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_TRANSACTION_NOT_FOUND",
        "transaction is not retained");
    }
    return reply(
      ElevatorExecutionReplyKind::kAccepted,
      "ELEVATOR_EXECUTION_STATE",
      "transaction snapshot returned",
      snapshot_);
  }

  std::optional<ElevatorExecutionSnapshot> current_snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

  std::optional<std::string> active_transaction() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!snapshot_ || snapshot_->terminal) {
      return std::nullopt;
    }
    return snapshot_->transaction_id;
  }

  bool journal_recovery_required() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return options_.persistent_recovery_lock_enabled &&
           !initialization_error_.empty();
  }

  bool persistent_recovery_lock_enabled() const noexcept
  {
    return options_.persistent_recovery_lock_enabled;
  }

private:
  std::optional<ElevatorExecutionReply> matching_active_locked(
    const std::string & transaction_id) const
  {
    if (!snapshot_ || snapshot_->transaction_id != transaction_id) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_TRANSACTION_NOT_FOUND",
        "transaction is not retained",
        snapshot_);
    }
    if (snapshot_->terminal) {
      return reply(
        ElevatorExecutionReplyKind::kConflict,
        "ELEVATOR_EXECUTION_TERMINAL_STATE",
        "terminal transaction cannot accept confirmation",
        snapshot_);
    }
    return std::nullopt;
  }

  void accept_output_locked(const ElevatorFsmOutput & output)
  {
    if (!snapshot_ || !frozen_release_) {
      return;
    }
    snapshot_->state = to_string(output.state);
    snapshot_->phase = snapshot_->state;
    snapshot_->physical_zone = physical_zone_for(output.state);
    snapshot_->effect_sequence = output.effect.sequence;
    snapshot_->updated_at = timestamp_utc();
    pending_effect_.reset();
    work_pending_ = false;
    if (const auto gate = confirmation_for(output, automatic_button_control_)) {
      snapshot_->awaiting_confirmation = true;
      snapshot_->expected_confirmation = *gate;
      append_record(
        snapshot_->events,
        output.effect.sequence,
        "OPERATOR_CONFIRMATION_REQUIRED",
        *gate);
      return;
    }
    snapshot_->awaiting_confirmation = false;
    snapshot_->expected_confirmation.clear();
    if (
      output.state == ElevatorState::kComplete ||
      output.effect.kind == ElevatorEffectKind::kComplete)
    {
      // Completion is itself a journaled runtime effect. The runtime port
      // must retire its bound transaction before the HTTP state is allowed to
      // report COMPLETE; otherwise the next ride conflicts with a port that
      // still believes the previous transaction is active.
      snapshot_->state = "FINALIZING";
      snapshot_->phase = "FINALIZING";
      snapshot_->terminal = false;
      snapshot_->motion_authorized = false;
      pending_effect_ = enrich_effect(*frozen_release_, output.effect);
      append_record(
        snapshot_->events,
        output.effect.sequence,
        "RUNTIME_EFFECT_INTENT",
        to_string(output.effect.kind));
      work_pending_ = true;
      return;
    }
    pending_effect_ = enrich_effect(*frozen_release_, output.effect);
    append_record(
      snapshot_->events,
      output.effect.sequence,
      "RUNTIME_EFFECT_INTENT",
      to_string(output.effect.kind));
    work_pending_ = true;
  }

  void begin_failure_locked(
    std::string code,
    std::string detail,
    const bool cancellation,
    const std::optional<ElevatorCleanupDisposition> proven_disposition =
    std::nullopt)
  {
    if (!snapshot_ || !frozen_release_) {
      return;
    }
    if (snapshot_->failure_origin_state.empty()) {
      snapshot_->failure_origin_state = snapshot_->state;
      snapshot_->failure_origin_effect_kind = preparing_ ?
        "RUNTIME_PREPARE" :
        (pending_effect_ ?
        to_string(pending_effect_->effect.kind) : "NONE");
      snapshot_->cleanup_disposition = proven_disposition.value_or(
        cleanup_disposition_for(fsm_.state()));
      if (
        !options_.persistent_recovery_lock_enabled &&
        snapshot_->cleanup_disposition ==
        ElevatorCleanupDisposition::kRetainLock)
      {
        snapshot_->cleanup_disposition =
          snapshot_->current_floor_id == snapshot_->target_floor_id &&
          snapshot_->current_map_id == snapshot_->target_map_id ?
          ElevatorCleanupDisposition::kTargetOutside :
          ElevatorCleanupDisposition::kSourceOutside;
      }
      snapshot_->runtime_resources_reconciled = false;
    }
    if (!cancellation && original_failure_code_.empty()) {
      original_failure_code_ = code;
      original_failure_detail_ = detail;
    }
    if (cancellation) {
      cancellation_requested_ = true;
      if (original_failure_code_.empty()) {
        original_failure_code_ = "ELEVATOR_EXECUTION_CANCELLED";
        original_failure_detail_ = detail;
      }
    }
    snapshot_->awaiting_confirmation = false;
    snapshot_->expected_confirmation.clear();
    snapshot_->state = "FAILURE_CLEANUP";
    snapshot_->phase = "FAILURE_CLEANUP";
    snapshot_->terminal = false;
    snapshot_->motion_authorized = false;
    snapshot_->failure_code = original_failure_code_;
    snapshot_->detail = original_failure_detail_;
    preparing_ = false;
    cleanup_attempt_count_ = 0U;
    append_record(
      snapshot_->errors,
      snapshot_->effect_sequence,
      code,
      detail);

    ElevatorEffect cleanup;
    if (
      fsm_.state() != ElevatorState::kIdle &&
      fsm_.state() != ElevatorState::kFailureCleanup &&
      fsm_.state() != ElevatorState::kLocked)
    {
      const auto output = fsm_.dispatch(
        ElevatorEvent{
          cancellation ?
          ElevatorEventKind::kCancelRequested :
          ElevatorEventKind::kEffectFailed,
          detail,
          snapshot_->effect_sequence,
          snapshot_->transaction_id,
        });
      cleanup = output.effect;
    }
    if (cleanup.kind != ElevatorEffectKind::kHoldAndCancel) {
      cleanup.kind = ElevatorEffectKind::kHoldAndCancel;
      cleanup.sequence = snapshot_->effect_sequence + 1U;
      cleanup.transaction_id = snapshot_->transaction_id;
      cleanup.detail = detail;
    }
    if (
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kSourceOutside)
    {
      cleanup.floor_id = snapshot_->source_floor_id;
      cleanup.map_id = snapshot_->source_map_id;
    } else if (
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kTargetOutside)
    {
      cleanup.floor_id = snapshot_->target_floor_id;
      cleanup.map_id = snapshot_->target_map_id;
    } else {
      cleanup.floor_id = snapshot_->current_floor_id;
      cleanup.map_id = snapshot_->current_map_id;
    }
    snapshot_->effect_sequence = cleanup.sequence;
    pending_effect_ = enrich_effect(*frozen_release_, cleanup);
    pending_effect_->cleanup_disposition = snapshot_->cleanup_disposition;
    append_record(
      snapshot_->events,
      cleanup.sequence,
      "FAILURE_CLEANUP_INTENT",
      detail);
    work_pending_ = true;
    snapshot_->updated_at = timestamp_utc();
  }

  void update_success_evidence_locked(
    const ElevatorRuntimeEffect & effect,
    const ElevatorRuntimeResult & result)
  {
    if (!snapshot_) {
      return;
    }
    switch (effect.effect.kind) {
      case ElevatorEffectKind::kNavigateToPose:
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
        break;
      case ElevatorEffectKind::kAcquireSafetyHold:
        update_safety_hold_evidence_locked(result);
        if (result.dual_odom_stop_proven) {
          snapshot_->dual_odom_stop_proven = true;
        }
        break;
      case ElevatorEffectKind::kReleaseSafetyHold:
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = false;
        break;
      case ElevatorEffectKind::kSwitchFloor:
        snapshot_->current_floor_id = snapshot_->target_floor_id;
        snapshot_->current_map_id = snapshot_->target_map_id;
        break;
      default:
        break;
    }
  }

  void update_safety_hold_evidence_locked(
    const ElevatorRuntimeResult & result)
  {
    if (!snapshot_) {
      return;
    }
    const bool active_proven = result.safety_hold_proven;
    const bool absent_proven = result.safety_hold_absence_proven;
    if (active_proven == absent_proven) {
      // Neither bit is UNKNOWN; both bits are contradictory and therefore
      // also UNKNOWN. Preserve the last observed value, but mark it
      // non-authoritative so every consumer remains fail-closed.
      snapshot_->safety_hold_state_known = false;
      return;
    }
    snapshot_->safety_hold_state_known = true;
    snapshot_->safety_hold_active = active_proven;
  }

  void handle_prepare_result_locked(const ElevatorRuntimeResult & result)
  {
    preparing_ = false;
    if (!snapshot_ || snapshot_->terminal) {
      return;
    }
    if (!result.success) {
      if (
        result.failure_disposition ==
        ElevatorRuntimeFailureDisposition::kRecoveryRequiredBeforeEffects)
      {
        runtime_side_effects_may_exist_ = false;
        const auto code =
          result.code.empty() ? "ELEVATOR_RUNTIME_RECOVERY_REQUIRED" :
          result.code;
        const auto detail =
          result.detail.empty() ?
          "pre-existing runtime state requires maintenance recovery" :
          result.detail;
        snapshot_->state = options_.persistent_recovery_lock_enabled ?
          "LOCKED" : "FAILED";
        snapshot_->phase = options_.persistent_recovery_lock_enabled ?
          "LOCKED" : "RUNTIME_PREFLIGHT";
        snapshot_->terminal = true;
        snapshot_->awaiting_confirmation = false;
        snapshot_->expected_confirmation.clear();
        snapshot_->runtime_applied = false;
        snapshot_->motion_authorized = false;
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
        snapshot_->failure_code = code;
        snapshot_->detail = detail;
        pending_effect_.reset();
        work_pending_ = false;
        append_record(
          snapshot_->errors, snapshot_->effect_sequence, code, detail);
        append_record(
          snapshot_->events,
          snapshot_->effect_sequence,
          options_.persistent_recovery_lock_enabled ?
          "RUNTIME_PREPARE_RECOVERY_REQUIRED" :
          "RUNTIME_PREPARE_FAILED_NONLOCKING",
          options_.persistent_recovery_lock_enabled ?
          "no new transaction effect was applied; pre-existing state remains "
          "unproven" :
          "no new transaction effect was applied; failure was recorded "
          "without installing a persistent global recovery lock");
        snapshot_->updated_at = timestamp_utc();
        if (!persist_locked()) {
          lock_for_storage_failure_locked(last_storage_error_);
        }
        condition_.notify_one();
        return;
      }
      if (
        result.failure_disposition ==
        ElevatorRuntimeFailureDisposition::kRejectedBeforeEffects)
      {
        runtime_side_effects_may_exist_ = false;
        const auto code =
          result.code.empty() ? "ELEVATOR_RUNTIME_PREPARE_FAILED" : result.code;
        const auto detail =
          result.detail.empty() ? "runtime prepare failed before effects" :
          result.detail;
        snapshot_->state = "FAILED";
        snapshot_->phase = "RUNTIME_PREFLIGHT";
        snapshot_->terminal = true;
        snapshot_->awaiting_confirmation = false;
        snapshot_->expected_confirmation.clear();
        snapshot_->runtime_applied = false;
        snapshot_->motion_authorized = false;
        snapshot_->safety_hold_state_known = true;
        snapshot_->safety_hold_active = false;
        snapshot_->dual_odom_stop_proven = false;
        snapshot_->failure_code = code;
        snapshot_->detail = detail;
        pending_effect_.reset();
        work_pending_ = false;
        append_record(
          snapshot_->errors, snapshot_->effect_sequence, code, detail);
        append_record(
          snapshot_->events,
          snapshot_->effect_sequence,
          "RUNTIME_PREPARE_REJECTED",
          "runtime adapter proved that no transaction effect was applied");
        snapshot_->updated_at = timestamp_utc();
        if (!persist_locked()) {
          lock_for_storage_failure_locked(last_storage_error_);
        }
        condition_.notify_one();
        return;
      }
      // An unclassified prepare failure may have applied a partial runtime
      // effect. Treat it conservatively as applied so it can never use the
      // preflight-orphan recovery path.
      snapshot_->runtime_applied = true;
      begin_failure_locked(
        result.code.empty() ? "ELEVATOR_RUNTIME_PREPARE_FAILED" : result.code,
        result.detail.empty() ? "runtime prepare failed" : result.detail,
        false);
      if (!persist_locked()) {
        lock_for_storage_failure_locked(last_storage_error_);
      }
      condition_.notify_one();
      return;
    }
    append_record(
      snapshot_->events, 0U, "RUNTIME_PREPARED",
      "runtime adapter bound the exact frozen release");
    snapshot_->runtime_applied = true;
    const auto output = fsm_.start(
      snapshot_->transaction_id, route_from(*frozen_release_));
    if (!output.accepted) {
      begin_failure_locked(
        "ELEVATOR_FSM_START_REJECTED", output.message, false);
    } else {
      accept_output_locked(output);
    }
    if (!persist_locked()) {
      lock_for_storage_failure_locked(last_storage_error_);
    }
    condition_.notify_one();
  }

  void handle_effect_result_locked(
    const ElevatorRuntimeEffect & effect,
    const ElevatorRuntimeResult & result)
  {
    if (
      !snapshot_ || snapshot_->terminal ||
      snapshot_->transaction_id != effect.effect.transaction_id ||
      snapshot_->effect_sequence != effect.effect.sequence)
    {
      return;
    }
    const bool cleanup_effect =
      effect.effect.kind == ElevatorEffectKind::kHoldAndCancel;
    const bool retain_lock_cleanup =
      cleanup_effect &&
      snapshot_->cleanup_disposition ==
      ElevatorCleanupDisposition::kRetainLock;
    const bool cleanup_stage_proven =
      result.success && result.safety_hold_proven &&
      !result.safety_hold_absence_proven &&
      result.dual_odom_stop_proven &&
      (retain_lock_cleanup || result.runtime_resources_reconciled);
    if (result.operator_confirmation_required) {
      const std::string expected_event =
        effect.effect.kind == ElevatorEffectKind::kMockPressCallButton ?
        "CALL_BUTTON_PRESSED" :
        effect.effect.kind == ElevatorEffectKind::kMockPressTargetButton ?
        "TARGET_BUTTON_PRESSED" : std::string{};
      if (
        !result.success || !automatic_button_control_ ||
        expected_event.empty())
      {
        begin_failure_locked(
          "ELEVATOR_RUNTIME_INVALID_MANUAL_FALLBACK",
          "runtime requested an invalid automatic-button manual fallback",
          false);
      } else {
        snapshot_->awaiting_confirmation = true;
        snapshot_->expected_confirmation = expected_event;
        snapshot_->detail = result.detail;
        snapshot_->updated_at = timestamp_utc();
        pending_effect_.reset();
        work_pending_ = false;
        append_record(
          snapshot_->events,
          effect.effect.sequence,
          "AUTOMATIC_BUTTON_MANUAL_FALLBACK",
          result.code + ";" + result.detail);
        append_record(
          snapshot_->events,
          effect.effect.sequence,
          "OPERATOR_CONFIRMATION_REQUIRED",
          expected_event);
      }
      if (!persist_locked()) {
        lock_for_storage_failure_locked(last_storage_error_);
      }
      condition_.notify_one();
      return;
    }
    if (!result.success || (cleanup_effect && !cleanup_stage_proven)) {
      const auto code =
        cleanup_effect && result.success ?
        "ELEVATOR_FAILURE_CLEANUP_EVIDENCE_INCOMPLETE" :
        (result.code.empty() ? "ELEVATOR_RUNTIME_EFFECT_FAILED" : result.code);
      const auto detail =
        cleanup_effect && result.success ?
        "cleanup must prove owner hold retained, dual odometry stopped, and "
        "all runtime resources reconciled" :
        (result.detail.empty() ? to_string(effect.effect.kind) + " failed" :
        result.detail);
      if (cleanup_effect) {
        ++cleanup_attempt_count_;
        append_record(
          snapshot_->errors, effect.effect.sequence, code, detail);
        snapshot_->state = "FAILURE_CLEANUP";
        snapshot_->phase = "FAILURE_CLEANUP";
        snapshot_->terminal = false;
        snapshot_->motion_authorized = false;
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
        snapshot_->runtime_resources_reconciled =
          result.runtime_resources_reconciled;
        if (cleanup_attempt_count_ < kMaximumCleanupAttempts) {
          snapshot_->detail =
            "failure cleanup attempt " +
            std::to_string(cleanup_attempt_count_) +
            " of " + std::to_string(kMaximumCleanupAttempts) +
            " failed; retry queued: " + detail;
          append_record(
            snapshot_->events,
            effect.effect.sequence,
            "FAILURE_CLEANUP_RETRY_INTENT",
            "attempt=" + std::to_string(cleanup_attempt_count_ + 1U) +
            ";safety_hold_proven=" +
            (result.safety_hold_proven ? "true" : "false") +
            ";dual_odom_stop_proven=" +
            (result.dual_odom_stop_proven ? "true" : "false"));
          work_pending_ = true;
        } else if (options_.persistent_recovery_lock_enabled) {
          snapshot_->state = "LOCKED";
          snapshot_->phase = "LOCKED";
          snapshot_->terminal = true;
          snapshot_->failure_code = kCleanupUnproven;
          snapshot_->detail =
            "failure cleanup remained unproven after " +
            std::to_string(kMaximumCleanupAttempts) +
            " attempts; original_failure=" + original_failure_code_ +
            "; last_error=" + detail;
          pending_effect_.reset();
          work_pending_ = false;
          append_record(
            snapshot_->errors,
            effect.effect.sequence,
            snapshot_->failure_code,
            snapshot_->detail);
        } else {
          snapshot_->state = "FAILURE_CLEANUP";
          snapshot_->phase = "AUTOMATIC_FAILURE_CLEANUP";
          snapshot_->terminal = false;
          snapshot_->failure_code = original_failure_code_;
          snapshot_->detail =
            "failure cleanup has not yet been proven after " +
            std::to_string(cleanup_attempt_count_) +
            " attempts; automatic retry remains active; last_error=" + detail;
          work_pending_ = true;
          append_record(
            snapshot_->events,
            effect.effect.sequence,
            "FAILURE_CLEANUP_AUTOMATIC_RETRY",
            "persistent recovery locking is disabled; retry remains automatic");
        }
        snapshot_->updated_at = timestamp_utc();
        if (!persist_locked()) {
          lock_for_storage_failure_locked(last_storage_error_);
        }
        condition_.notify_one();
        return;
      }
      std::optional<ElevatorCleanupDisposition> proven_disposition;
      if (
        effect.effect.kind == ElevatorEffectKind::kNavigateToPose &&
        fsm_.state() == ElevatorState::kEnteringCabin &&
        result.motion_not_authorized_proven)
      {
        proven_disposition = ElevatorCleanupDisposition::kSourceOutside;
        append_record(
          snapshot_->events,
          effect.effect.sequence,
          "ENTRY_MOTION_NOT_AUTHORIZED_PROVEN",
          "robot_safety never authorized the cabin-entry navigation before "
          "Nav2 reached a known terminal result; source outside retained");
      }
      begin_failure_locked(code, detail, false, proven_disposition);
      if (!persist_locked()) {
        lock_for_storage_failure_locked(last_storage_error_);
      }
      condition_.notify_one();
      return;
    }

    append_record(
      snapshot_->events,
      effect.effect.sequence,
      "RUNTIME_EFFECT_SUCCEEDED",
      to_string(effect.effect.kind));
    update_success_evidence_locked(effect, result);
    if (effect.effect.kind == ElevatorEffectKind::kComplete) {
      snapshot_->state = "COMPLETE";
      snapshot_->phase = "COMPLETE";
      snapshot_->terminal = true;
      snapshot_->motion_authorized = false;
      snapshot_->dual_odom_stop_proven = false;
      snapshot_->runtime_resources_reconciled = true;
      snapshot_->current_floor_id = snapshot_->target_floor_id;
      snapshot_->current_map_id = snapshot_->target_map_id;
      snapshot_->failure_code.clear();
      snapshot_->detail =
        "all automatic effects and five confirmations completed";
      snapshot_->updated_at = timestamp_utc();
      pending_effect_.reset();
      work_pending_ = false;
      cleanup_attempt_count_ = 0U;
      append_record(
        snapshot_->events,
        effect.effect.sequence,
        "ELEVATOR_EXECUTION_COMPLETE",
        snapshot_->detail);
      if (!persist_locked()) {
        lock_for_storage_failure_locked(last_storage_error_);
      }
      return;
    }
    if (effect.effect.kind == ElevatorEffectKind::kHoldAndCancel) {
      update_safety_hold_evidence_locked(result);
      snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
      snapshot_->runtime_resources_reconciled =
        result.runtime_resources_reconciled;
      snapshot_->motion_authorized = false;
      snapshot_->updated_at = timestamp_utc();
      pending_effect_.reset();
      work_pending_ = false;
      cleanup_attempt_count_ = 0U;
      append_record(
        snapshot_->events,
        effect.effect.sequence,
        "FAILURE_CLEANUP_EVIDENCE",
        std::string("safety_hold_proven=") +
        (result.safety_hold_proven ? "true" : "false") +
        ";dual_odom_stop_proven=" +
        (result.dual_odom_stop_proven ? "true" : "false") +
        ";runtime_resources_reconciled=" +
        (result.runtime_resources_reconciled ? "true" : "false"));
      if (
        snapshot_->cleanup_disposition ==
        ElevatorCleanupDisposition::kRetainLock)
      {
        snapshot_->state = "LOCKED";
        snapshot_->phase = "LOCKED";
        snapshot_->terminal = true;
        snapshot_->failure_code =
          "ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED";
        snapshot_->detail =
          "failure origin " + snapshot_->failure_origin_state +
          " is not proven outside the elevator; owner hold remains retained";
        append_record(
          snapshot_->events,
          effect.effect.sequence,
          "FAILURE_CLEANUP_RETAINED_LOCK",
          snapshot_->detail);
        if (!persist_locked()) {
          lock_for_storage_failure_locked(last_storage_error_);
        }
        return;
      }
      snapshot_->state = "FAILURE_CLEANUP";
      snapshot_->phase = "CLEANUP_RELEASE_PENDING";
      snapshot_->terminal = false;
      append_record(
        snapshot_->events,
        effect.effect.sequence,
        "CLEANUP_RELEASE_PENDING",
        "outside-floor identity and all runtime resources are reconciled; "
        "owner hold remains retained");
      if (!persist_locked()) {
        lock_for_storage_failure_locked(last_storage_error_);
        return;
      }
      if (!persist_recovery_audit_locked(
          "cleanup_release_pending", *snapshot_))
      {
        const auto audit_error = last_storage_error_;
        fail_recovery_audit_locked("cleanup_release_pending", audit_error);
        return;
      }
      recovery_stage_ = RecoveryStage::kFinalize;
      work_pending_ = true;
      condition_.notify_one();
      return;
    }

    const auto output = fsm_.dispatch(
      ElevatorEvent{
        ElevatorEventKind::kEffectSucceeded,
        result.detail,
        effect.effect.sequence,
        effect.effect.transaction_id,
      });
    if (!output.accepted) {
      begin_failure_locked(
        "ELEVATOR_FSM_EFFECT_REJECTED", output.message, false);
    } else {
      accept_output_locked(output);
    }
    if (!persist_locked()) {
      lock_for_storage_failure_locked(last_storage_error_);
    }
    condition_.notify_one();
  }

  void handle_monitor_failure_locked(
    const ElevatorRuntimeResult & result,
    const char * fallback_code)
  {
    if (
      !snapshot_ || snapshot_->terminal ||
      snapshot_->state == "FAILURE_CLEANUP")
    {
      return;
    }
    begin_failure_locked(
      result.code.empty() ? fallback_code : result.code,
      result.detail.empty() ? "runtime lease/health monitoring failed" :
      result.detail,
      false);
    if (!persist_locked()) {
      lock_for_storage_failure_locked(last_storage_error_);
    }
    condition_.notify_one();
  }

  void worker_loop()
  {
    for (;;) {
      std::shared_ptr<ElevatorRuntimePort> port;
      std::string transaction_id;
      std::optional<ElevatorRuntimeEffect> effect;
      bool prepare = false;
      bool monitor = false;
      bool restart_recovery = false;
      bool automatic_retry_delay = false;
      RecoveryStage recovery_stage = RecoveryStage::kNone;
      FrozenElevatorRelease release;
      ElevatorRuntimeCleanupContext cleanup_context;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(
          lock,
          std::chrono::milliseconds(250),
          [this]() {return stop_ || work_pending_;});
        if (stop_) {
          return;
        }
        port = runtime_port_;
        if (!port || !snapshot_) {
          continue;
        }
        transaction_id = snapshot_->transaction_id;
        cleanup_context = cleanup_context_from(*snapshot_);
        if (restart_recovery_pending_) {
          runtime_side_effects_may_exist_ = true;
          restart_recovery = true;
          automatic_retry_delay =
            !options_.persistent_recovery_lock_enabled &&
            (restart_recovery_attempt_count_ > 0U ||
            restart_recovery_startup_attempt_count_ > 0U);
          work_pending_ = false;
        } else if (recovery_stage_ != RecoveryStage::kNone) {
          runtime_side_effects_may_exist_ = true;
          recovery_stage = recovery_stage_;
          automatic_retry_delay =
            !options_.persistent_recovery_lock_enabled &&
            cleanup_attempt_count_ > 0U;
          work_pending_ = false;
        } else if (snapshot_->terminal) {
          continue;
        } else if (work_pending_) {
          work_pending_ = false;
          if (preparing_) {
            runtime_side_effects_may_exist_ = true;
            prepare = true;
            release = *frozen_release_;
          } else if (pending_effect_) {
            runtime_side_effects_may_exist_ = true;
            effect = *pending_effect_;
            automatic_retry_delay =
              !options_.persistent_recovery_lock_enabled &&
              effect->effect.kind == ElevatorEffectKind::kHoldAndCancel &&
              cleanup_attempt_count_ > 0U;
            in_flight_sequence_ = effect->effect.sequence;
          }
        } else if (
          snapshot_->state != "FAILURE_CLEANUP" &&
          !snapshot_->terminal)
        {
          monitor = true;
        }
      }

      if (automatic_retry_delay) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      if (restart_recovery) {
        ElevatorRuntimeResult result;
        try {
          result = port->recover_locked(cleanup_context);
        } catch (const std::exception & exception) {
          result = {
            false,
            "ELEVATOR_RESTART_RECOVERY_EXCEPTION",
            exception.what(),
            false,
            false,
          };
        } catch (...) {
          result = {
            false,
            "ELEVATOR_RESTART_RECOVERY_EXCEPTION",
            "unknown restart recovery exception",
            false,
            false,
          };
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (
          !snapshot_ || snapshot_->transaction_id != transaction_id ||
          !restart_recovery_pending_)
        {
          continue;
        }
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven =
          result.dual_odom_stop_proven;
        snapshot_->runtime_resources_reconciled =
          result.runtime_resources_reconciled;
        const bool proven =
          result.success && snapshot_->safety_hold_state_known &&
          snapshot_->safety_hold_active &&
          result.dual_odom_stop_proven &&
          (snapshot_->cleanup_disposition ==
          ElevatorCleanupDisposition::kRetainLock ||
          result.runtime_resources_reconciled);
        if (proven) {
          restart_recovery_pending_ = false;
          const bool physically_outside =
            snapshot_->physical_zone ==
            ElevatorPhysicalZone::kSourceOutside ||
            snapshot_->physical_zone ==
            ElevatorPhysicalZone::kTargetOutside;
          const bool safe_runtime_cleanup =
            snapshot_->runtime_applied &&
            (physically_outside ||
            !options_.persistent_recovery_lock_enabled) &&
            (snapshot_->cleanup_disposition ==
            ElevatorCleanupDisposition::kSourceOutside ||
            snapshot_->cleanup_disposition ==
            ElevatorCleanupDisposition::kTargetOutside);
          if (safe_runtime_cleanup) {
            snapshot_->state = "FAILURE_CLEANUP";
            snapshot_->phase = "CLEANUP_RELEASE_PENDING";
            snapshot_->terminal = false;
            append_record(
              snapshot_->events,
              snapshot_->effect_sequence,
              "CLEANUP_RELEASE_PENDING",
              "restart re-established exact outside cleanup proof; owner hold "
              "remains retained");
            if (!persist_locked()) {
              lock_for_storage_failure_locked(last_storage_error_);
            } else if (!persist_recovery_audit_locked(
                "cleanup_release_pending", *snapshot_))
            {
              const auto audit_error = last_storage_error_;
              fail_recovery_audit_locked(
                "cleanup_release_pending", audit_error);
            } else {
              recovery_stage_ = RecoveryStage::kFinalize;
              work_pending_ = true;
            }
            condition_.notify_one();
            continue;
          }
          if (!options_.persistent_recovery_lock_enabled) {
            snapshot_->state = "FAILURE_CLEANUP";
            snapshot_->phase = "RECOVERY_RELEASE_PENDING";
            snapshot_->terminal = false;
            append_record(
              snapshot_->events,
              snapshot_->effect_sequence,
              "AUTOMATIC_RECOVERY_RELEASE_PENDING",
              "restart cleanup proof is sufficient for automatic owner-resource release");
            if (!persist_locked()) {
              lock_for_storage_failure_locked(last_storage_error_);
            } else if (!persist_recovery_audit_locked(
                "recovery_release_pending", *snapshot_))
            {
              const auto audit_error = last_storage_error_;
              fail_recovery_audit_locked(
                "recovery_release_pending", audit_error);
            } else {
              recovery_stage_ = RecoveryStage::kFinalize;
              work_pending_ = true;
            }
            condition_.notify_one();
            continue;
          }
          snapshot_->failure_code = kRestartLocked;
          snapshot_->detail =
            "restart recovery retained the owner safety hold, terminated "
            "unknown action state, and proved dual odometry stopped; "
            "explicit maintenance recovery is still required";
          append_record(
            snapshot_->events,
            snapshot_->effect_sequence,
            "RESTART_RECOVERY_PROVEN",
            snapshot_->detail);
        } else {
          const bool startup_transient =
            restart_recovery_startup_transient(result.code);
          auto & attempt_count = startup_transient ?
            restart_recovery_startup_attempt_count_ :
            restart_recovery_attempt_count_;
          const auto maximum_attempts = startup_transient ?
            kMaximumRestartStartupAttempts : kMaximumCleanupAttempts;
          ++attempt_count;
          append_record(
            snapshot_->errors,
            snapshot_->effect_sequence,
            result.code.empty() ?
            kRestartCleanupUnproven : result.code,
            result.detail.empty() ?
            "restart recovery evidence is incomplete" : result.detail);
          if (
            attempt_count < maximum_attempts ||
            !options_.persistent_recovery_lock_enabled)
          {
            work_pending_ = true;
            snapshot_->detail =
              std::string(
              !options_.persistent_recovery_lock_enabled ?
              "automatic restart recovery attempt " :
              (startup_transient ?
              "restart startup recovery attempt " :
              "restart recovery attempt ")) +
              std::to_string(attempt_count) +
              (options_.persistent_recovery_lock_enabled ?
              " of " + std::to_string(maximum_attempts) : std::string{}) +
              " was unproven; retry queued automatically";
          } else {
            restart_recovery_pending_ = false;
            snapshot_->failure_code = kRestartCleanupUnproven;
            snapshot_->detail =
              std::string(startup_transient ?
              "restart startup recovery remained unproven after " :
              "restart recovery remained unproven after ") +
              std::to_string(maximum_attempts) +
              " attempts; all state-changing APIs remain locked";
          }
        }
        snapshot_->updated_at = timestamp_utc();
        if (!persist_locked()) {
          initialization_error_ = last_storage_error_;
        }
        condition_.notify_one();
        continue;
      }

      if (
        recovery_stage == RecoveryStage::kVerify ||
        recovery_stage == RecoveryStage::kRebindThenFinalize)
      {
        const bool release_pending_rebind =
          recovery_stage == RecoveryStage::kRebindThenFinalize;
        ElevatorRuntimeResult result;
        try {
          result = port->recover_locked(cleanup_context);
        } catch (const std::exception & exception) {
          result = {
            false,
            "ELEVATOR_EXPLICIT_RECOVERY_EXCEPTION",
            exception.what(),
            false,
            false,
          };
        } catch (...) {
          result = {
            false,
            "ELEVATOR_EXPLICIT_RECOVERY_EXCEPTION",
            "unknown explicit recovery exception",
            false,
            false,
          };
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (
          !snapshot_ || snapshot_->transaction_id != transaction_id ||
          recovery_stage_ != recovery_stage)
        {
          continue;
        }
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
        snapshot_->runtime_resources_reconciled =
          result.runtime_resources_reconciled;
        if (
          result.success && snapshot_->safety_hold_state_known &&
          snapshot_->safety_hold_active &&
          result.dual_odom_stop_proven &&
          result.runtime_resources_reconciled)
        {
          if (release_pending_rebind) {
            const bool legacy_preflight_orphan =
              !snapshot_->runtime_applied &&
              snapshot_->cleanup_disposition ==
              ElevatorCleanupDisposition::kSourceOutside;
            const char * pending_checkpoint =
              legacy_preflight_orphan ?
              "recovery_release_pending" : "cleanup_release_pending";
            // The release-pending journal is committed before its immutable
            // audit. A crash in that narrow window must never let restart
            // proceed directly to the physical hold release. Verify/create
            // the audit against the exact loaded release-intent snapshot
            // before appending any restart observation.
            if (!persist_recovery_audit_locked(
                pending_checkpoint, *snapshot_))
            {
              const auto audit_error = last_storage_error_;
              recovery_stage_ = RecoveryStage::kNone;
              fail_recovery_audit_locked(
                pending_checkpoint, audit_error);
            } else {
              // Keep the durable release-intent snapshot byte-for-byte aligned
              // with its immutable pending audit until physical release. A
              // second crash can then repeat this exact proof without creating
              // an audit collision from a transient "rebound" event.
              recovery_stage_ = RecoveryStage::kFinalize;
              work_pending_ = true;
            }
            condition_.notify_one();
            continue;
          }
          const bool legacy_preflight_orphan =
            !snapshot_->runtime_applied &&
            snapshot_->cleanup_disposition ==
            ElevatorCleanupDisposition::kSourceOutside;
          snapshot_->state = "FAILURE_CLEANUP";
          snapshot_->phase = legacy_preflight_orphan ?
            "RECOVERY_RELEASE_PENDING" : "CLEANUP_RELEASE_PENDING";
          snapshot_->terminal = false;
          append_record(
            snapshot_->events,
            snapshot_->effect_sequence,
            "EXPLICIT_RECOVERY_VERIFIED",
            "owner hold retained; all runtimes idle; dual odometry stopped");
          if (!persist_locked()) {
            recovery_stage_ = RecoveryStage::kNone;
            lock_for_storage_failure_locked(last_storage_error_);
          } else if (!persist_recovery_audit_locked(
              legacy_preflight_orphan ?
              "recovery_release_pending" : "cleanup_release_pending",
              *snapshot_))
          {
            const auto audit_error = last_storage_error_;
            fail_recovery_audit_locked(
              legacy_preflight_orphan ?
              "recovery_release_pending" : "cleanup_release_pending",
              audit_error);
          } else {
            recovery_stage_ = RecoveryStage::kFinalize;
            work_pending_ = true;
          }
        } else {
          recovery_stage_ = RecoveryStage::kNone;
          snapshot_->state = "LOCKED";
          snapshot_->phase = "LOCKED";
          snapshot_->terminal = true;
          snapshot_->failure_code =
            "ELEVATOR_EXECUTION_RECOVERY_UNPROVEN";
          snapshot_->detail =
            result.detail.empty() ?
            "explicit recovery safety evidence is incomplete" :
            result.detail;
          append_record(
            snapshot_->errors,
            snapshot_->effect_sequence,
            result.code.empty() ? snapshot_->failure_code : result.code,
            snapshot_->detail);
          if (!persist_locked()) {
            lock_for_storage_failure_locked(last_storage_error_);
          }
        }
        condition_.notify_one();
        continue;
      }

      if (recovery_stage == RecoveryStage::kFinalize) {
        ElevatorRuntimeResult result;
        try {
          result = port->finalize_recovery(cleanup_context);
        } catch (const std::exception & exception) {
          result = {
            false,
            "ELEVATOR_RECOVERY_FINALIZE_EXCEPTION",
            exception.what(),
            false,
            false,
          };
        } catch (...) {
          result = {
            false,
            "ELEVATOR_RECOVERY_FINALIZE_EXCEPTION",
            "unknown recovery finalization exception",
            false,
            false,
          };
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (
          !snapshot_ || snapshot_->transaction_id != transaction_id ||
          recovery_stage_ != RecoveryStage::kFinalize)
        {
          continue;
        }
        recovery_stage_ = RecoveryStage::kNone;
        const bool cleanup_finalization =
          snapshot_->phase == "CLEANUP_RELEASE_PENDING";
        const bool on_site_source_outside_recovery =
          has_durable_source_outside_confirmation(*snapshot_);
        update_safety_hold_evidence_locked(result);
        snapshot_->dual_odom_stop_proven = result.dual_odom_stop_proven;
        snapshot_->runtime_resources_reconciled =
          result.runtime_resources_reconciled;
        if (
          result.success && snapshot_->safety_hold_state_known &&
          !snapshot_->safety_hold_active &&
          result.dual_odom_stop_proven &&
          result.runtime_resources_reconciled)
        {
          runtime_side_effects_may_exist_ = false;
          snapshot_->state =
            cleanup_finalization &&
            snapshot_->failure_code == "ELEVATOR_EXECUTION_CANCELLED" ?
            "CANCELLED" : "FAILED";
          snapshot_->phase =
            cleanup_finalization && !on_site_source_outside_recovery ?
            snapshot_->state : "RECOVERY_COMPLETE";
          snapshot_->terminal = true;
          if (!cleanup_finalization) {
            snapshot_->runtime_applied = false;
          }
          snapshot_->motion_authorized = false;
          if (!cleanup_finalization || on_site_source_outside_recovery) {
            snapshot_->failure_code = "ELEVATOR_EXECUTION_RECOVERED";
            snapshot_->detail = on_site_source_outside_recovery ?
              "on-site source-outside confirmation was independently "
              "verified; the safety lock is released and the original "
              "elevator task remains failed" :
              "legacy preflight orphan lock safely recovered; original "
              "elevator task remains failed";
          }
          append_record(
            snapshot_->events,
            snapshot_->effect_sequence,
            "EXPLICIT_RECOVERY_COMPLETE",
            "owner hold absence and dual odometry stop proven");
          snapshot_->updated_at = timestamp_utc();
          if (!persist_recovery_audit_locked(
              cleanup_finalization ? "cleanup_complete" : "recovery_complete",
              *snapshot_))
          {
            const auto audit_error = last_storage_error_;
            fail_recovery_audit_locked(
              cleanup_finalization ? "cleanup_complete" : "recovery_complete",
              audit_error);
            condition_.notify_one();
            continue;
          }
        } else if (options_.persistent_recovery_lock_enabled) {
          snapshot_->state = "LOCKED";
          snapshot_->phase = "LOCKED";
          snapshot_->terminal = true;
          snapshot_->motion_authorized = false;
          snapshot_->failure_code =
            "ELEVATOR_EXECUTION_RECOVERY_RELEASE_UNPROVEN";
          snapshot_->detail =
            result.detail.empty() ?
            "recovery hold release or stop evidence is incomplete" :
            result.detail;
          append_record(
            snapshot_->errors,
            snapshot_->effect_sequence,
            result.code.empty() ? snapshot_->failure_code : result.code,
            snapshot_->detail);
        } else {
          ++cleanup_attempt_count_;
          recovery_stage_ = RecoveryStage::kFinalize;
          work_pending_ = true;
          snapshot_->state = "FAILURE_CLEANUP";
          snapshot_->phase = "AUTOMATIC_RELEASE_RETRY";
          snapshot_->terminal = false;
          snapshot_->motion_authorized = false;
          snapshot_->failure_code = original_failure_code_.empty() ?
            "ELEVATOR_EXECUTION_FAILED" : original_failure_code_;
          snapshot_->detail =
            result.detail.empty() ?
            "automatic owner-resource release remains pending" :
            result.detail + "; automatic release retry remains active";
          append_record(
            snapshot_->errors,
            snapshot_->effect_sequence,
            result.code.empty() ?
            "ELEVATOR_AUTOMATIC_RELEASE_RETRY" : result.code,
            snapshot_->detail);
        }
        if (!persist_locked()) {
          lock_for_storage_failure_locked(last_storage_error_);
        }
        condition_.notify_one();
        continue;
      }

      if (prepare) {
        ElevatorRuntimeResult result;
        try {
          result = port->prepare(transaction_id, release);
        } catch (const std::exception & exception) {
          result = {
            false,
            "ELEVATOR_RUNTIME_PREPARE_EXCEPTION",
            exception.what(),
          };
        } catch (...) {
          result = {
            false,
            "ELEVATOR_RUNTIME_PREPARE_EXCEPTION",
            "unknown runtime prepare exception",
          };
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (
          snapshot_ && !snapshot_->terminal &&
          snapshot_->transaction_id == transaction_id && preparing_)
        {
          handle_prepare_result_locked(result);
        }
        continue;
      }
      if (effect) {
        ElevatorRuntimeResult result;
        try {
          result = port->apply(*effect);
        } catch (const std::exception & exception) {
          result = {
            false,
            "ELEVATOR_RUNTIME_EFFECT_EXCEPTION",
            exception.what(),
          };
        } catch (...) {
          result = {
            false,
            "ELEVATOR_RUNTIME_EFFECT_EXCEPTION",
            "unknown runtime effect exception",
          };
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (in_flight_sequence_ == effect->effect.sequence) {
          in_flight_sequence_ = 0U;
        }
        handle_effect_result_locked(*effect, result);
        continue;
      }
      if (monitor) {
        ElevatorRuntimeResult heartbeat;
        try {
          heartbeat = port->heartbeat(transaction_id);
        } catch (const std::exception & exception) {
          heartbeat = {
            false,
            "ELEVATOR_RUNTIME_HEARTBEAT_EXCEPTION",
            exception.what(),
          };
        } catch (...) {
          heartbeat = {
            false,
            "ELEVATOR_RUNTIME_HEARTBEAT_EXCEPTION",
            "unknown runtime heartbeat exception",
          };
        }
        if (!heartbeat.success) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (snapshot_ && snapshot_->transaction_id == transaction_id) {
            handle_monitor_failure_locked(
              heartbeat, "ELEVATOR_RUNTIME_HEARTBEAT_FAILED");
          }
          continue;
        }
        ElevatorRuntimeResult health;
        try {
          health = port->poll_health(transaction_id);
        } catch (const std::exception & exception) {
          health = {
            false,
            "ELEVATOR_RUNTIME_HEALTH_EXCEPTION",
            exception.what(),
          };
        } catch (...) {
          health = {
            false,
            "ELEVATOR_RUNTIME_HEALTH_EXCEPTION",
            "unknown runtime health exception",
          };
        }
        if (!health.success) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (snapshot_ && snapshot_->transaction_id == transaction_id) {
            handle_monitor_failure_locked(
              health, "ELEVATOR_RUNTIME_HEALTH_FAILED");
          }
        }
      }
    }
  }

  bool persist_recovery_audit_locked(
    const std::string & checkpoint,
    const ElevatorExecutionSnapshot & snapshot)
  {
    last_storage_error_.clear();
    try {
      return immutable_write_recovery_audit(
        journal_path_, checkpoint, snapshot, last_storage_error_);
    } catch (const std::exception & exception) {
      last_storage_error_ =
        std::string("recovery audit exception: ") + exception.what();
      return false;
    } catch (...) {
      last_storage_error_ = "unknown recovery audit exception";
      return false;
    }
  }

  void fail_recovery_audit_locked(
    const std::string & checkpoint,
    const std::string & storage_error)
  {
    recovery_stage_ = RecoveryStage::kNone;
    lock_for_storage_failure_locked(
      "immutable explicit recovery " + checkpoint +
      " audit persistence failed: " + storage_error);
    if (!persist_locked()) {
      initialization_error_ = last_storage_error_;
    }
  }

  bool persist_locked()
  {
    if (!snapshot_) {
      last_storage_error_ = "no transaction snapshot";
      last_storage_commit_uncertain_ = false;
      return false;
    }
    snapshot_->updated_at = timestamp_utc();
    last_storage_error_.clear();
    last_storage_commit_uncertain_ = false;
    try {
      const auto outcome =
        atomic_write_yaml(journal_path_, *snapshot_, last_storage_error_);
      last_storage_commit_uncertain_ =
        outcome == JournalWriteOutcome::kCommitUncertain;
      if (last_storage_commit_uncertain_) {
        last_storage_error_ =
          "journal commit outcome is uncertain: " + last_storage_error_;
      }
      return outcome == JournalWriteOutcome::kCommitted;
    } catch (const std::exception & exception) {
      last_storage_error_ =
        std::string("journal exception: ") + exception.what();
      return false;
    } catch (...) {
      last_storage_error_ = "unknown journal exception";
      return false;
    }
  }

  void lock_for_storage_failure_locked(const std::string & detail)
  {
    if (!snapshot_) {
      return;
    }
    // This describes the snapshot whose write just failed, not the in-memory
    // LOCKED sentinel constructed below. If the possibly-installed snapshot
    // was a clean terminal, introducing a new hold would make disk falsely
    // advertise absence after a commit-uncertain rename.
    const bool persisted_snapshot_would_be_clear_terminal =
      snapshot_->terminal &&
      snapshot_->safety_hold_state_known &&
      !snapshot_->safety_hold_active;
    const bool cancellation_needed =
      runtime_side_effects_may_exist_ ||
      snapshot_->runtime_applied ||
      (snapshot_->safety_hold_state_known &&
      snapshot_->safety_hold_active);
    snapshot_->state = options_.persistent_recovery_lock_enabled ?
      "LOCKED" : "FAILED";
    snapshot_->phase = options_.persistent_recovery_lock_enabled ?
      "LOCKED" : "JOURNAL_FAILURE";
    snapshot_->terminal = true;
    snapshot_->awaiting_confirmation = false;
    snapshot_->expected_confirmation.clear();
    snapshot_->motion_authorized = false;
    snapshot_->failure_code = kJournalFailure;
    snapshot_->detail = detail;
    pending_effect_.reset();
    work_pending_ = false;
    in_flight_sequence_ = 0U;
    cleanup_attempt_count_ = 0U;
    preparing_ = false;
    append_record(
      snapshot_->errors, snapshot_->effect_sequence,
      snapshot_->failure_code, snapshot_->detail);
    if (
      runtime_port_ && cancellation_needed &&
      !persisted_snapshot_would_be_clear_terminal)
    {
      if (
        snapshot_->safety_hold_state_known &&
        !snapshot_->safety_hold_active)
      {
        // request_cancel() may asynchronously assert/retain this
        // transaction's hold. A previously proven absence is therefore no
        // longer authoritative once that fail-closed command is issued.
        snapshot_->safety_hold_state_known = false;
      }
      runtime_port_->request_cancel(snapshot_->transaction_id);
    }
  }

  ElevatorExecutionReply storage_failure_locked()
  {
    lock_for_storage_failure_locked(last_storage_error_);
    return reply(
      ElevatorExecutionReplyKind::kStorageFailure,
      kJournalFailure,
      last_storage_error_,
      snapshot_);
  }

  void load_journal()
  {
    std::string payload;
    std::string read_error;
    const auto read_outcome =
      read_journal_payload(journal_path_, payload, read_error);
    if (read_outcome == JournalReadOutcome::kNotFound) {
      return;
    }
    if (read_outcome == JournalReadOutcome::kError) {
      initialization_error_ = read_error;
      return;
    }
    try {
      const auto root = YAML::Load(payload);
      std::uint32_t envelope_schema_version = 0U;
      std::uint32_t snapshot_schema_version = kLegacySnapshotSchemaVersion;
      const auto snapshot_schema_node = root["snapshot_schema_version"];
      const bool snapshot_schema_present =
        static_cast<bool>(snapshot_schema_node);
      const auto snapshot_node_value = root["snapshot"];
      const bool legacy_shape_valid =
        snapshot_schema_present ||
        !snapshot_node_value ||
        !snapshot_node_value["safety_hold_state_known"];
      const bool schema_valid =
        required_scalar(
        root, "schema_version", envelope_schema_version) &&
        legacy_shape_valid &&
        (!snapshot_schema_present ||
        required_scalar(
          root, "snapshot_schema_version", snapshot_schema_version));
      if (
        !schema_valid ||
        envelope_schema_version != kJournalEnvelopeSchemaVersion ||
        (snapshot_schema_version != kLegacySnapshotSchemaVersion &&
        snapshot_schema_version != kPreviousSnapshotSchemaVersion &&
        snapshot_schema_version != kSnapshotSchemaVersion))
      {
        initialization_error_ = "unsupported journal schema";
        return;
      }
      snapshot_ =
        parse_snapshot(root["snapshot"], snapshot_schema_version);
      if (!snapshot_) {
        initialization_error_ = "journal snapshot is invalid";
        return;
      }
      if (
        snapshot_schema_version == kSnapshotSchemaVersion &&
        snapshot_->state == "LOCKED" && snapshot_->terminal &&
        snapshot_->failure_code ==
        "ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED" &&
        !snapshot_->failure_origin_state.empty() &&
        snapshot_->failure_origin_effect_kind != "NONE" &&
        snapshot_->failure_origin_effect_kind != "RUNTIME_PREPARE" &&
        snapshot_->cleanup_disposition ==
        ElevatorCleanupDisposition::kRetainLock)
      {
        std::optional<ElevatorCleanupDisposition> current_disposition;
        if (
          snapshot_->current_floor_id == snapshot_->source_floor_id &&
          snapshot_->current_map_id == snapshot_->source_map_id)
        {
          current_disposition = ElevatorCleanupDisposition::kSourceOutside;
        } else if (
          snapshot_->current_floor_id == snapshot_->target_floor_id &&
          snapshot_->current_map_id == snapshot_->target_map_id)
        {
          current_disposition = ElevatorCleanupDisposition::kTargetOutside;
        }
        if (current_disposition) {
          snapshot_->cleanup_disposition = *current_disposition;
          append_record(
            snapshot_->events,
            snapshot_->effect_sequence,
            "LEGACY_RETAINED_LOCK_AUTO_CLEANUP",
            "retained elevator lock converted to ordinary cleanup using "
            "the exact journaled current floor/map identity");
        }
      }
      const bool exact_source_identity =
        snapshot_->current_floor_id == snapshot_->source_floor_id &&
        snapshot_->current_map_id == snapshot_->source_map_id;
      const bool exact_target_identity =
        snapshot_->current_floor_id == snapshot_->target_floor_id &&
        snapshot_->current_map_id == snapshot_->target_map_id;
      const bool exhausted_ordinary_restart_recovery =
        snapshot_schema_version == kSnapshotSchemaVersion &&
        snapshot_->state == "LOCKED" && snapshot_->terminal &&
        snapshot_->failure_code == kRestartCleanupUnproven &&
        !snapshot_->failure_origin_state.empty() &&
        snapshot_->failure_origin_effect_kind != "NONE" &&
        snapshot_->failure_origin_effect_kind != "RUNTIME_PREPARE" &&
        (
          (snapshot_->cleanup_disposition ==
          ElevatorCleanupDisposition::kSourceOutside &&
          exact_source_identity) ||
          (snapshot_->cleanup_disposition ==
          ElevatorCleanupDisposition::kTargetOutside &&
          exact_target_identity));
      if (exhausted_ordinary_restart_recovery) {
        // A production restart replaces the complete runtime chain, not only
        // this API process.  Therefore process-owned action goals, leases and
        // the owner hold from the previous runtime no longer exist.  Waiting
        // for the replacement endpoints here caused their staggered startup
        // to consume the finite retry budget and permanently re-lock an
        // otherwise ordinary source/target-side failure.  Finalize only the
        // exact, current-schema shape produced by that exhausted startup
        // recovery.  Ambiguous identity, RETAIN_LOCK, prepare failures,
        // corrupt journals and storage/audit failures remain fail-closed.
        snapshot_->state = "FAILED";
        snapshot_->phase = "FAILED";
        snapshot_->terminal = true;
        snapshot_->awaiting_confirmation = false;
        snapshot_->expected_confirmation.clear();
        snapshot_->motion_authorized = false;
        snapshot_->safety_hold_state_known = true;
        snapshot_->safety_hold_active = false;
        snapshot_->dual_odom_stop_proven = false;
        snapshot_->runtime_resources_reconciled = true;
        snapshot_->detail =
          "full runtime-chain cold start finalized the previously exhausted "
          "ordinary-stage recovery using the exact journaled current "
          "floor/map identity; no motion or odometry-stop proof is claimed";
        append_record(
          snapshot_->events,
          snapshot_->effect_sequence,
          "NORMAL_STAGE_LOCK_CLEARED_ON_COLD_START",
          snapshot_->detail);
        if (!persist_locked()) {
          initialization_error_ = last_storage_error_;
          return;
        }
      }
      const bool release_pending =
        !snapshot_->terminal &&
        snapshot_->runtime_resources_reconciled &&
        snapshot_->safety_hold_state_known &&
        snapshot_->safety_hold_active &&
        (snapshot_->phase == "CLEANUP_RELEASE_PENDING" ||
        snapshot_->phase == "RECOVERY_RELEASE_PENDING");
      if (release_pending) {
        // The durable release intent/audit precedes the physical hold release.
        // Re-entering finalization is safe both before and after that release:
        // the runtime adapter re-establishes the exact binding, re-proves all
        // barriers, and performs a sequenced conditional release.
        recovery_stage_ = RecoveryStage::kRebindThenFinalize;
        work_pending_ = true;
        return;
      }
      const bool journal_was_nonterminal =
        !terminal_state(snapshot_->state) || !snapshot_->terminal;
      const bool requires_runtime_recovery =
        journal_was_nonterminal ||
        snapshot_->state == "LOCKED" ||
        !snapshot_->safety_hold_state_known ||
        snapshot_->safety_hold_active ||
        (
          snapshot_->terminal &&
          snapshot_->runtime_applied &&
          !snapshot_->runtime_resources_reconciled);
      if (requires_runtime_recovery) {
        const auto previous_failure = snapshot_->failure_code;
        if (journal_was_nonterminal) {
          snapshot_->interrupted_state = snapshot_->state;
          snapshot_->interrupted_expected_confirmation =
            snapshot_->expected_confirmation;
        }
        if (
          !options_.persistent_recovery_lock_enabled &&
          snapshot_->cleanup_disposition ==
          ElevatorCleanupDisposition::kRetainLock)
        {
          snapshot_->cleanup_disposition =
            exact_target_identity ?
            ElevatorCleanupDisposition::kTargetOutside :
            ElevatorCleanupDisposition::kSourceOutside;
        }
        snapshot_->state = options_.persistent_recovery_lock_enabled ?
          "LOCKED" : "FAILURE_CLEANUP";
        snapshot_->phase = options_.persistent_recovery_lock_enabled ?
          "LOCKED" : "AUTOMATIC_RESTART_RECOVERY";
        snapshot_->terminal = options_.persistent_recovery_lock_enabled;
        snapshot_->awaiting_confirmation = false;
        snapshot_->expected_confirmation.clear();
        snapshot_->motion_authorized = false;
        snapshot_->safety_hold_state_known = false;
        snapshot_->dual_odom_stop_proven = false;
        snapshot_->failure_code = options_.persistent_recovery_lock_enabled ?
          kRestartLocked :
          (previous_failure.empty() ?
          "ELEVATOR_EXECUTION_INTERRUPTED" : previous_failure);
        snapshot_->detail =
          options_.persistent_recovery_lock_enabled ?
          (journal_was_nonterminal ?
          std::string(
          "nonterminal elevator execution cannot resume implicitly after restart") :
          std::string(
          "a retained terminal recovery state must be re-established after restart") +
          (previous_failure.empty() ?
          std::string{} : "; previous_failure=" + previous_failure)) :
          "interrupted elevator work will be canceled and released automatically; "
          "no persistent recovery lock is installed";
        restart_recovery_pending_ = true;
        restart_recovery_attempt_count_ = 0U;
        restart_recovery_startup_attempt_count_ = 0U;
        append_record(
          snapshot_->errors,
          snapshot_->effect_sequence,
          snapshot_->failure_code,
          snapshot_->detail);
        if (!persist_locked()) {
          initialization_error_ = last_storage_error_;
        }
      }
    } catch (const YAML::Exception & exception) {
      initialization_error_ =
        std::string("cannot parse journal: ") + exception.what();
    }
  }

  fs::path journal_path_;
  std::shared_ptr<ElevatorRuntimePort> runtime_port_;
  ElevatorExecutionOptions options_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  bool stop_{false};
  bool preparing_{false};
  bool work_pending_{false};
  std::uint64_t in_flight_sequence_{0U};
  std::size_t cleanup_attempt_count_{0U};
  bool cancellation_requested_{false};
  bool runtime_side_effects_may_exist_{false};
  bool restart_recovery_pending_{false};
  std::size_t restart_recovery_attempt_count_{0U};
  std::size_t restart_recovery_startup_attempt_count_{0U};
  RecoveryStage recovery_stage_{RecoveryStage::kNone};
  ElevatorFsm fsm_;
  std::optional<FrozenElevatorRelease> frozen_release_;
  std::optional<ElevatorRuntimeEffect> pending_effect_;
  bool automatic_button_control_{false};
  std::optional<ElevatorExecutionSnapshot> snapshot_;
  std::string original_failure_code_;
  std::string original_failure_detail_;
  std::string initialization_error_;
  std::string last_storage_error_;
  bool last_storage_commit_uncertain_{false};
};

ElevatorExecutionModule::ElevatorExecutionModule(
  fs::path journal_path,
  std::shared_ptr<ElevatorRuntimePort> runtime_port,
  ElevatorExecutionOptions options)
: implementation_(
    std::make_unique<Implementation>(
      std::move(journal_path), std::move(runtime_port), options))
{
}

ElevatorExecutionModule::~ElevatorExecutionModule() = default;

ElevatorExecutionReply ElevatorExecutionModule::start(
  const ElevatorExecutionStart & command)
{
  return implementation_->start(command);
}

ElevatorExecutionReply ElevatorExecutionModule::confirm(
  const ElevatorExecutionConfirmation & command)
{
  return implementation_->confirm(command);
}

ElevatorExecutionReply ElevatorExecutionModule::cancel(
  const ElevatorExecutionCancellation & command)
{
  return implementation_->cancel(command);
}

ElevatorExecutionReply ElevatorExecutionModule::recover(
  const ElevatorExecutionRecovery & command)
{
  return implementation_->recover(command);
}

ElevatorExecutionReply ElevatorExecutionModule::snapshot(
  const std::string & transaction_id) const
{
  return implementation_->snapshot(transaction_id);
}

std::optional<ElevatorExecutionSnapshot>
ElevatorExecutionModule::current_snapshot() const
{
  return implementation_->current_snapshot();
}

std::optional<std::string>
ElevatorExecutionModule::active_transaction() const
{
  return implementation_->active_transaction();
}

bool ElevatorExecutionModule::journal_recovery_required() const
{
  return implementation_->journal_recovery_required();
}

bool ElevatorExecutionModule::persistent_recovery_lock_enabled() const noexcept
{
  return implementation_->persistent_recovery_lock_enabled();
}

}  // namespace robot_elevator_manager
