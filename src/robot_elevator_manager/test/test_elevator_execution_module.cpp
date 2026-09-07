#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#ifndef _WIN32
#include <csignal>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "robot_elevator_manager/elevator_execution_module.hpp"

namespace robot_elevator_manager
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_elevator_execution_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  fs::path journal() const
  {
    return root_ / "active.yaml";
  }

  fs::path history_directory() const
  {
    return root_ / "history";
  }

  fs::path recovery_audit(
    const std::string & transaction_id,
    const std::uint64_t effect_sequence,
    const std::string & checkpoint) const
  {
    return history_directory() /
           (transaction_id + "." + std::to_string(effect_sequence) + "." +
           checkpoint + ".yaml");
  }

private:
  fs::path root_;
};

class BlockingNavigationPort final : public ElevatorRuntimePort
{
public:
  ElevatorRuntimeCapabilities capabilities() const noexcept override
  {
    return {true, true, true};
  }

  ElevatorRuntimeResult prepare(
    const std::string &,
    const FrozenElevatorRelease &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult apply(const ElevatorRuntimeEffect & effect) override
  {
    if (effect.effect.kind == ElevatorEffectKind::kHoldAndCancel) {
      auto result = ElevatorRuntimeResult{
        true,
        "OK",
        "owned goal terminal, hold retained, and stopped odometry proven",
        true,
        true,
      };
      result.runtime_resources_reconciled =
        effect.cleanup_disposition !=
        ElevatorCleanupDisposition::kRetainLock;
      return result;
    }
    if (effect.effect.kind != ElevatorEffectKind::kNavigateToPose) {
      return {true, "OK", ""};
    }
    std::unique_lock<std::mutex> lock(mutex_);
    navigation_entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return cancellation_requested_;});
    return {false, "NAVIGATION_CANCELLED", "owned goal interrupted"};
  }

  void request_cancel(const std::string &) noexcept override
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      cancellation_requested_ = true;
      condition_.notify_all();
    } catch (...) {
    }
  }

  ElevatorRuntimeResult finalize_recovery(
    const ElevatorRuntimeCleanupContext &) override
  {
    ElevatorRuntimeResult result{
      true,
      "OK",
      "owned hold released and stopped odometry proven",
      false,
      true,
    };
    result.safety_hold_absence_proven = true;
    result.runtime_resources_reconciled = true;
    return result;
  }

  ElevatorRuntimeResult heartbeat(const std::string &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult poll_health(const std::string &) override
  {
    return {true, "OK", ""};
  }

  bool wait_until_navigation_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock,
      std::chrono::seconds(2),
      [this]() {return navigation_entered_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool navigation_entered_{false};
  bool cancellation_requested_{false};
};

class RecoveryAuditObservingPort final : public ElevatorRuntimePort
{
public:
  explicit RecoveryAuditObservingPort(
    fs::path history_directory,
    const bool inject_complete_collision = false,
    const bool fail_after_pending = false)
  : history_directory_(std::move(history_directory)),
    inject_complete_collision_(inject_complete_collision),
    fail_after_pending_(fail_after_pending)
  {
  }

  ElevatorRuntimeCapabilities capabilities() const noexcept override
  {
    return {true, true, true};
  }

  ElevatorRuntimeResult prepare(
    const std::string &,
    const FrozenElevatorRelease &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult apply(const ElevatorRuntimeEffect &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult recover_locked(
    const ElevatorRuntimeCleanupContext &) override
  {
    auto result = ElevatorRuntimeResult{
      true,
      "OK",
      "owner hold and stopped odometry proven",
      true,
      true,
    };
    result.runtime_resources_reconciled = true;
    return result;
  }

  ElevatorRuntimeResult finalize_recovery(
    const ElevatorRuntimeCleanupContext & context) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++finalize_calls_;
    const std::string pending_checkpoint =
      context.legacy_preflight_orphan ?
      "recovery_release_pending" : "cleanup_release_pending";
    const std::string complete_checkpoint =
      context.legacy_preflight_orphan ?
      "recovery_complete" : "cleanup_complete";
    const auto pending = checkpoint_files(pending_checkpoint);
    pending_seen_before_finalize_ = !pending.empty();
    complete_seen_before_finalize_ =
      !checkpoint_files(complete_checkpoint).empty();
    if (!pending_seen_before_finalize_) {
      return {
        false,
        "RECOVERY_AUDIT_ORDER_VIOLATION",
        "release-pending audit was not durable before physical release",
        true,
        true,
      };
    }
    if (fail_after_pending_) {
      return {
        false,
        "RECOVERY_RELEASE_INTERRUPTED",
        "simulated process interruption after release-pending audit",
        true,
        true,
      };
    }
    if (inject_complete_collision_) {
      auto collision_name = pending.front().filename().string();
      const auto position = collision_name.find(pending_checkpoint);
      if (position != std::string::npos) {
        collision_name.replace(
          position, pending_checkpoint.size(), complete_checkpoint);
        collision_path_ = history_directory_ / collision_name;
        std::ofstream collision(*collision_path_, std::ios::binary | std::ios::trunc);
        collision << "immutable-sentinel\n";
      }
    }
    auto result = ElevatorRuntimeResult{
      true,
      "OK",
      "owner hold absence and stopped odometry proven",
      false,
      true,
      ElevatorRuntimeFailureDisposition::kCleanupRequired,
      true,
    };
    result.runtime_resources_reconciled = true;
    return result;
  }

  void request_cancel(const std::string &) noexcept override
  {
  }

  ElevatorRuntimeResult heartbeat(const std::string &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult poll_health(const std::string &) override
  {
    return {true, "OK", ""};
  }

  std::size_t finalize_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return finalize_calls_;
  }

  bool pending_seen_before_finalize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_seen_before_finalize_;
  }

  bool complete_seen_before_finalize() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return complete_seen_before_finalize_;
  }

  std::optional<fs::path> collision_path() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return collision_path_;
  }

private:
  std::vector<fs::path> checkpoint_files(
    const std::string & checkpoint) const
  {
    std::vector<fs::path> files;
    std::error_code error;
    if (!fs::is_directory(history_directory_, error)) {
      return files;
    }
    for (fs::directory_iterator iterator(history_directory_, error), end;
      !error && iterator != end; iterator.increment(error))
    {
      if (
        iterator->is_regular_file(error) &&
        iterator->path().filename().string().find(checkpoint) !=
        std::string::npos)
      {
        files.push_back(iterator->path());
      }
    }
    return files;
  }

  fs::path history_directory_;
  bool inject_complete_collision_{false};
  bool fail_after_pending_{false};
  mutable std::mutex mutex_;
  std::size_t finalize_calls_{0U};
  bool pending_seen_before_finalize_{false};
  bool complete_seen_before_finalize_{false};
  std::optional<fs::path> collision_path_;
};

class BlockingRecoveryFinalizePort final : public ElevatorRuntimePort
{
public:
  ElevatorRuntimeCapabilities capabilities() const noexcept override
  {
    return {true, true, true};
  }

  ElevatorRuntimeResult prepare(
    const std::string &,
    const FrozenElevatorRelease &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult apply(const ElevatorRuntimeEffect &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult recover_locked(
    const ElevatorRuntimeCleanupContext &) override
  {
    auto result = ElevatorRuntimeResult{
      true,
      "OK",
      "owner hold retained and stopped odometry proven",
      true,
      true,
    };
    result.runtime_resources_reconciled = true;
    return result;
  }

  ElevatorRuntimeResult finalize_recovery(
    const ElevatorRuntimeCleanupContext &) override
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      finalize_entered_ = true;
      condition_.notify_all();
      condition_.wait(lock, [this]() {return finalize_released_;});
    }
    auto result = ElevatorRuntimeResult{
      true,
      "OK",
      "owner hold absence and stopped odometry proven",
      false,
      true,
    };
    result.safety_hold_absence_proven = true;
    result.runtime_resources_reconciled = true;
    return result;
  }

  void request_cancel(const std::string &) noexcept override
  {
    release_finalize();
  }

  ElevatorRuntimeResult heartbeat(const std::string &) override
  {
    return {true, "OK", ""};
  }

  ElevatorRuntimeResult poll_health(const std::string &) override
  {
    return {true, "OK", ""};
  }

  bool wait_until_finalize_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock,
      std::chrono::seconds(2),
      [this]() {return finalize_entered_;});
  }

  void release_finalize() noexcept
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      finalize_released_ = true;
      condition_.notify_all();
    } catch (...) {
    }
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool finalize_entered_{false};
  bool finalize_released_{false};
};

ElevatorRuntimePose pose(
  const PoseRole role,
  const std::string & pose_id,
  const double x)
{
  return ElevatorRuntimePose{role, pose_id, x, 0.0, 0.0};
}

FrozenElevatorRelease release()
{
  FrozenElevatorRelease value;
  value.release_id = "elevator-config-000001-abcdef123456";
  value.generation = 1U;
  value.configuration_digest = "0123456789abcdef";
  value.building_id = "B11";
  value.elevator_id = "elevator_1";
  value.source.floor_id = "F1";
  value.source.map_id = "map_f1";
  value.source.map_asset_epoch = 11U;
  value.source.map_asset_digest =
    "sha256:1111111111111111111111111111111111111111111111111111111111111111";
  value.source.poses = {
    pose(PoseRole::kHallCall, "f1_hall_call", 0.0),
    pose(PoseRole::kLanding, "f1_landing", 1.0),
    pose(PoseRole::kCabin, "f1_cabin", 2.0),
  };
  value.target.floor_id = "F2";
  value.target.map_id = "map_f2";
  value.target.map_asset_epoch = 12U;
  value.target.map_asset_digest =
    "sha256:2222222222222222222222222222222222222222222222222222222222222222";
  value.target.poses = {
    pose(PoseRole::kHallCall, "f2_hall_call", 0.0),
    pose(PoseRole::kLanding, "f2_landing", 1.0),
    pose(PoseRole::kCabin, "f2_cabin", 2.0),
  };
  return value;
}

ElevatorExecutionStart start_command(const std::string & transaction_id = "elevator-test-1")
{
  return ElevatorExecutionStart{
    transaction_id,
    "commissioning_app",
    release(),
  };
}

ElevatorExecutionConfirmation confirmation(
  const ElevatorExecutionSnapshot & snapshot,
  const std::string & observed_floor_id)
{
  return ElevatorExecutionConfirmation{
    snapshot.transaction_id,
    snapshot.state,
    snapshot.effect_sequence,
    snapshot.expected_confirmation,
    observed_floor_id,
    snapshot.operator_id,
  };
}

const ElevatorExecutionSnapshot & require_snapshot(const ElevatorExecutionReply & reply)
{
  EXPECT_TRUE(reply.snapshot.has_value()) << reply.code << ": " << reply.detail;
  return *reply.snapshot;
}

ElevatorExecutionSnapshot wait_for_state(
  ElevatorExecutionModule & module,
  const std::string & transaction_id,
  const std::string & state)
{
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.snapshot(transaction_id);
    if (reply.snapshot && reply.snapshot->state == state) {
      return *reply.snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto final = module.snapshot(transaction_id);
  ADD_FAILURE() << "timed out waiting for " << state << "; current=" <<
    (final.snapshot ? final.snapshot->state : final.code);
  return final.snapshot.value_or(ElevatorExecutionSnapshot{});
}

ElevatorExecutionSnapshot wait_for_confirmation(
  ElevatorExecutionModule & module,
  const std::string & transaction_id,
  const std::string & state,
  const std::string & expected_confirmation)
{
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.snapshot(transaction_id);
    if (
      reply.snapshot && reply.snapshot->state == state &&
      reply.snapshot->awaiting_confirmation &&
      reply.snapshot->expected_confirmation == expected_confirmation)
    {
      return *reply.snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto final = module.snapshot(transaction_id);
  ADD_FAILURE() << "timed out waiting for confirmation " <<
    expected_confirmation << " in " << state << "; current=" <<
    (final.snapshot ? final.snapshot->state : final.code);
  return final.snapshot.value_or(ElevatorExecutionSnapshot{});
}

ElevatorExecutionReply confirm_through_target_door(
  ElevatorExecutionModule & module,
  ElevatorExecutionReply reply)
{
  auto current = wait_for_state(
    module, require_snapshot(reply).transaction_id, "PRESSING_CALL_BUTTON");
  const std::vector<std::pair<std::string, std::string>> first_four{
    {"CALL_BUTTON_PRESSED", "F1"},
    {"SOURCE_DOOR_OPEN", "F1"},
    {"TARGET_BUTTON_PRESSED", "F1"},
    {"TARGET_FLOOR_ARRIVED", "F2"},
  };
  for (const auto & [event, observed_floor] : first_four) {
    EXPECT_EQ(current.expected_confirmation, event);
    reply = module.confirm(confirmation(current, observed_floor));
    EXPECT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
    const std::string next_state =
      event == "CALL_BUTTON_PRESSED" ? "WAITING_SOURCE_DOOR" :
      event == "SOURCE_DOOR_OPEN" ? "PRESSING_TARGET_BUTTON" :
      event == "TARGET_BUTTON_PRESSED" ? "RIDING" :
      "WAITING_TARGET_DOOR";
    current = wait_for_state(
      module, require_snapshot(reply).transaction_id, next_state);
  }
  EXPECT_EQ(current.expected_confirmation, "TARGET_DOOR_OPEN");
  return module.confirm(confirmation(current, "F2"));
}

ElevatorExecutionSnapshot wait_for_terminal(
  ElevatorExecutionModule & module,
  const std::string & transaction_id)
{
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.snapshot(transaction_id);
    if (reply.snapshot && reply.snapshot->terminal) {
      return *reply.snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto final = module.snapshot(transaction_id);
  ADD_FAILURE() << "timed out waiting for terminal state; current=" <<
    (final.snapshot ? final.snapshot->state : final.code);
  return final.snapshot.value_or(ElevatorExecutionSnapshot{});
}

ElevatorExecutionSnapshot wait_for_recovery_ready(
  ElevatorExecutionModule & module,
  const std::string & transaction_id)
{
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.snapshot(transaction_id);
    if (
      reply.snapshot && reply.snapshot->state == "LOCKED" &&
      reply.snapshot->safety_hold_state_known &&
      reply.snapshot->safety_hold_active &&
      reply.snapshot->dual_odom_stop_proven)
    {
      return *reply.snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto final = module.snapshot(transaction_id);
  ADD_FAILURE() << "timed out waiting for explicit recovery readiness";
  return final.snapshot.value_or(ElevatorExecutionSnapshot{});
}

ElevatorExecutionSnapshot wait_for_failure_code(
  ElevatorExecutionModule & module,
  const std::string & transaction_id,
  const std::string & failure_code)
{
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.snapshot(transaction_id);
    if (
      reply.snapshot && reply.snapshot->state == "LOCKED" &&
      reply.snapshot->failure_code == failure_code)
    {
      return *reply.snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto final = module.snapshot(transaction_id);
  ADD_FAILURE() << "timed out waiting for failure_code=" << failure_code;
  return final.snapshot.value_or(ElevatorExecutionSnapshot{});
}

void create_deployed_v2_preflight_orphan(
  const fs::path & journal,
  const std::string & transaction_id)
{
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    port->queue_prepare_result(
      {
        false,
        "ELEVATOR_SOURCE_RUNTIME_BUSY",
        "navigation, mapping, and docking must be idle before elevator execution",
      });
    for (int attempt = 0; attempt < 3; ++attempt) {
      port->queue_result(
        ElevatorEffectKind::kHoldAndCancel,
        {
          false,
          "ELEVATOR_RUNTIME_NOT_PREPARED",
          "no frozen elevator release is bound",
        });
    }
    ElevatorExecutionModule module(journal, port);
    const auto started = module.start(start_command(transaction_id));
    EXPECT_TRUE(started.accepted()) << started.code << ": " << started.detail;
    (void)wait_for_state(module, transaction_id, "LOCKED");
  }

  // Reproduce the exact deployed v2 preflight-orphan shape instead of
  // serializing it through the current schema.
  auto root = YAML::LoadFile(journal.string());
  root["schema_version"] = 1U;
  root["snapshot_schema_version"] = 2U;
  root["snapshot"]["runtime_applied"] = false;
  root["snapshot"].remove("runtime_resources_reconciled");
  root["snapshot"].remove("failure_origin_state");
  root["snapshot"].remove("failure_origin_effect_kind");
  root["snapshot"].remove("cleanup_disposition");
  YAML::Emitter emitter;
  emitter << root;
  ASSERT_TRUE(emitter.good()) << emitter.GetLastError();
  std::ofstream output(journal, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  output << emitter.c_str() << '\n';
  output.close();
  ASSERT_TRUE(output.good());
}

std::string read_text_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>(),
  };
}

void write_yaml_file(const fs::path & path, const YAML::Node & root)
{
  YAML::Emitter emitter;
  emitter << root;
  ASSERT_TRUE(emitter.good()) << emitter.GetLastError();
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.good());
  output << emitter.c_str() << '\n';
  output.close();
  ASSERT_TRUE(output.good());
}

void create_clean_v3_terminal_journal(
  const fs::path & journal,
  const std::string & transaction_id)
{
  auto incapable = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{false, false, false});
  ElevatorExecutionModule module(journal, incapable);
  const auto result = module.start(start_command(transaction_id));
  ASSERT_TRUE(result.accepted()) << result.code << ": " << result.detail;
}

void downgrade_journal_to_v2(
  const fs::path & journal,
  const bool runtime_applied)
{
  auto root = YAML::LoadFile(journal.string());
  root["snapshot_schema_version"] = 2U;
  root["snapshot"]["runtime_applied"] = runtime_applied;
  root["snapshot"].remove("runtime_resources_reconciled");
  root["snapshot"].remove("failure_origin_state");
  root["snapshot"].remove("failure_origin_effect_kind");
  root["snapshot"].remove("cleanup_disposition");
  write_yaml_file(journal, root);
}

std::uint64_t create_source_cleanup_release_pending_journal(
  const TemporaryDirectory & temporary,
  const std::string & transaction_id,
  const bool retain_pending_audit,
  const bool retain_complete_audit = false)
{
  ElevatorExecutionSnapshot completed;
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    port->queue_result(
      ElevatorEffectKind::kNavigateToPose,
      {false, "NAVIGATION_FAILED", "hall goal aborted"});
    ElevatorExecutionModule module(temporary.journal(), port);
    const auto started = module.start(start_command(transaction_id));
    completed = wait_for_state(module, transaction_id, "FAILED");
    EXPECT_TRUE(started.accepted()) << started.code << ": " << started.detail;
  }

  const auto pending_path = temporary.recovery_audit(
    transaction_id, completed.effect_sequence, "cleanup_release_pending");
  EXPECT_TRUE(fs::is_regular_file(pending_path));
  const auto pending_root = YAML::LoadFile(pending_path.string());
  YAML::Node journal_root;
  journal_root["schema_version"] = 1U;
  journal_root["snapshot_schema_version"] = 3U;
  journal_root["snapshot"] = pending_root["snapshot"];
  write_yaml_file(temporary.journal(), journal_root);

  if (!retain_pending_audit) {
    std::error_code error;
    EXPECT_TRUE(fs::remove_all(temporary.history_directory(), error) > 0U)
      << error.message();
  } else if (!retain_complete_audit) {
    const auto complete_path = temporary.recovery_audit(
      transaction_id, completed.effect_sequence, "cleanup_complete");
    std::error_code error;
    EXPECT_TRUE(fs::remove(complete_path, error)) << error.message();
  }
  return completed.effect_sequence;
}

std::size_t count_effect(
  const std::vector<ElevatorRuntimeEffect> & effects,
  const ElevatorEffectKind kind)
{
  return static_cast<std::size_t>(std::count_if(
    effects.begin(), effects.end(),
    [kind](const ElevatorRuntimeEffect & effect) {
      return effect.effect.kind == kind;
    }));
}

TEST(ElevatorExecutionModule, EveryFsmStateHasAnExplicitCleanupDisposition)
{
  using Disposition = ElevatorCleanupDisposition;
  const std::vector<std::pair<ElevatorState, Disposition>> expected = {
    {ElevatorState::kIdle, Disposition::kSourceOutside},
    {ElevatorState::kNavigatingHallCall, Disposition::kSourceOutside},
    {ElevatorState::kAcquiringHallHold, Disposition::kSourceOutside},
    {ElevatorState::kAcquiringExecutionLease, Disposition::kSourceOutside},
    {ElevatorState::kPressingCallButton, Disposition::kSourceOutside},
    {ElevatorState::kSettingElevatorWaitMode, Disposition::kSourceOutside},
    {ElevatorState::kReleasingHallHold, Disposition::kSourceOutside},
    {ElevatorState::kNavigatingSourceLanding, Disposition::kSourceOutside},
    {ElevatorState::kWaitingSourceDoor, Disposition::kSourceOutside},
    {ElevatorState::kSettingDoorwayEntryMode, Disposition::kSourceOutside},
    {ElevatorState::kEnteringCabin, Disposition::kSourceOutside},
    {ElevatorState::kAcquiringCabinHold, Disposition::kSourceOutside},
    {ElevatorState::kPressingTargetButton, Disposition::kSourceOutside},
    {ElevatorState::kPausingCorrections, Disposition::kSourceOutside},
    {ElevatorState::kSettingRideMode, Disposition::kSourceOutside},
    {ElevatorState::kRiding, Disposition::kSourceOutside},
    {ElevatorState::kWaitingTargetDoor, Disposition::kSourceOutside},
    {ElevatorState::kBeginningFloorTransition, Disposition::kSourceOutside},
    {ElevatorState::kResumingCorrections, Disposition::kSourceOutside},
    {ElevatorState::kSwitchingFloor, Disposition::kSourceOutside},
    {ElevatorState::kVerifyingFloorReady, Disposition::kTargetOutside},
    {ElevatorState::kSettingDoorwayExitMode, Disposition::kTargetOutside},
    {ElevatorState::kReleasingCabinHold, Disposition::kTargetOutside},
    {ElevatorState::kNavigatingTargetLanding, Disposition::kTargetOutside},
    {ElevatorState::kAcquiringExitHold, Disposition::kTargetOutside},
    {ElevatorState::kReleasingOperatingMode, Disposition::kTargetOutside},
    {ElevatorState::kReleasingExecutionLease, Disposition::kTargetOutside},
    {ElevatorState::kReleasingExitHold, Disposition::kTargetOutside},
    {ElevatorState::kComplete, Disposition::kTargetOutside},
    {ElevatorState::kFailureCleanup, Disposition::kRetainLock},
    {ElevatorState::kLocked, Disposition::kRetainLock},
  };

  for (const auto & [state, disposition] : expected) {
    EXPECT_EQ(cleanup_disposition_for(state), disposition)
      << "state=" << to_string(state);
  }
}

TEST(ElevatorExecutionModule, TargetButtonGateReportsCabinPhysicalZone)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(
    start_command("elevator-test-cabin-physical-zone"));
  auto current = wait_for_state(
    module, require_snapshot(started).transaction_id,
    "PRESSING_CALL_BUTTON");
  auto confirmed = module.confirm(confirmation(current, "F1"));
  current = wait_for_state(
    module, require_snapshot(confirmed).transaction_id,
    "WAITING_SOURCE_DOOR");
  confirmed = module.confirm(confirmation(current, "F1"));
  const auto cabin = wait_for_state(
    module, require_snapshot(confirmed).transaction_id,
    "PRESSING_TARGET_BUTTON");

  EXPECT_EQ(cabin.physical_zone, ElevatorPhysicalZone::kCabin);
}

TEST(ElevatorExecutionModule, RestartLockPreservesCabinInterruptionContext)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cabin-restart";
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(temporary.journal(), port);
    const auto started = module.start(start_command(transaction_id));
    auto current = wait_for_state(
      module, require_snapshot(started).transaction_id,
      "PRESSING_CALL_BUTTON");
    auto confirmed = module.confirm(confirmation(current, "F1"));
    current = wait_for_state(
      module, require_snapshot(confirmed).transaction_id,
      "WAITING_SOURCE_DOOR");
    confirmed = module.confirm(confirmation(current, "F1"));
    ASSERT_EQ(
      wait_for_state(
        module, require_snapshot(confirmed).transaction_id,
        "PRESSING_TARGET_BUTTON").physical_zone,
      ElevatorPhysicalZone::kCabin);
  }

  auto recovery_port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 3; ++attempt) {
    recovery_port->queue_recovery_result(
      {
        false,
        "ELEVATOR_RESTART_RECOVERY_TEST_FAILURE",
        "recovery proof deliberately unavailable",
        true,
        false,
      });
  }
  ElevatorExecutionModule restarted(temporary.journal(), recovery_port);
  const auto locked = wait_for_failure_code(
    restarted, transaction_id,
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN");

  EXPECT_EQ(locked.interrupted_state, "PRESSING_TARGET_BUTTON");
  EXPECT_EQ(
    locked.interrupted_expected_confirmation,
    "TARGET_BUTTON_PRESSED");
  EXPECT_EQ(locked.physical_zone, ElevatorPhysicalZone::kCabin);
}

TEST(ElevatorExecutionModule, LegacyRestartLockInfersCabinCheckpointFromAudit)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-legacy-cabin-lock";
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(temporary.journal(), port);
    const auto started = module.start(start_command(transaction_id));
    auto current = wait_for_state(
      module, require_snapshot(started).transaction_id,
      "PRESSING_CALL_BUTTON");
    auto confirmed = module.confirm(confirmation(current, "F1"));
    current = wait_for_state(
      module, require_snapshot(confirmed).transaction_id,
      "WAITING_SOURCE_DOOR");
    confirmed = module.confirm(confirmation(current, "F1"));
    ASSERT_EQ(
      wait_for_state(
        module, require_snapshot(confirmed).transaction_id,
        "PRESSING_TARGET_BUTTON").expected_confirmation,
      "TARGET_BUTTON_PRESSED");
  }

  auto root = YAML::LoadFile(temporary.journal().string());
  auto snapshot = root["snapshot"];
  snapshot["state"] = "LOCKED";
  snapshot["phase"] = "LOCKED";
  snapshot["terminal"] = true;
  snapshot["awaiting_confirmation"] = false;
  snapshot["expected_confirmation"] = "";
  snapshot["safety_hold_state_known"] = true;
  snapshot["safety_hold_active"] = true;
  snapshot["dual_odom_stop_proven"] = false;
  snapshot["runtime_resources_reconciled"] = false;
  snapshot["motion_authorized"] = false;
  snapshot["failure_code"] =
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN";
  snapshot["failure_origin_state"] = "";
  snapshot["failure_origin_effect_kind"] = "NONE";
  snapshot["cleanup_disposition"] = "SOURCE_OUTSIDE";
  snapshot.remove("physical_zone");
  snapshot.remove("interrupted_state");
  snapshot.remove("interrupted_expected_confirmation");
  {
    std::ofstream output(temporary.journal(), std::ios::trunc);
    output << root;
  }

  auto recovery_port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 3; ++attempt) {
    recovery_port->queue_recovery_result(
      {
        false,
        "ELEVATOR_RESTART_RECOVERY_TEST_FAILURE",
        "recovery proof deliberately unavailable",
        true,
        false,
      });
  }
  ElevatorExecutionModule restarted(temporary.journal(), recovery_port);
  const auto locked = wait_for_failure_code(
    restarted, transaction_id,
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN");

  EXPECT_EQ(locked.interrupted_state, "PRESSING_TARGET_BUTTON");
  EXPECT_EQ(
    locked.interrupted_expected_confirmation,
    "TARGET_BUTTON_PRESSED");
  EXPECT_EQ(locked.physical_zone, ElevatorPhysicalZone::kCabin);
}

TEST(ElevatorExecutionModule, FailureOriginIsCapturedBeforeCleanupOverwritesState)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {false, "CLEANUP_UNPROVEN", "injected cleanup failure"});
  }
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-origin-capture"));
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  EXPECT_EQ(locked.failure_origin_state, "NAVIGATING_HALL_CALL");
  EXPECT_EQ(locked.failure_origin_effect_kind, "NAVIGATE_TO_POSE");
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_FALSE(locked.runtime_resources_reconciled);
  const auto root = YAML::LoadFile(temporary.journal().string());
  EXPECT_EQ(root["snapshot_schema_version"].as<unsigned int>(), 3U);
  EXPECT_EQ(
    root["snapshot"]["cleanup_disposition"].as<std::string>(),
      "SOURCE_OUTSIDE");
}

TEST(
  ElevatorExecutionModule,
  RecoveryRequiresFloorActionOnlyAfterDurableFloorTransitionIntent)
{
  ElevatorExecutionSnapshot snapshot;
  snapshot.effect_sequence = 1U;
  snapshot.failure_origin_state = "NAVIGATING_HALL_CALL";
  snapshot.failure_origin_effect_kind = "NAVIGATE_TO_POSE";
  snapshot.events.push_back(
    {1U, "RUNTIME_EFFECT_INTENT", "NAVIGATE_TO_POSE", ""});

  EXPECT_FALSE(snapshot_may_have_submitted_floor_switch_action(snapshot));

  snapshot.effect_sequence = 17U;
  snapshot.events.push_back(
    {17U, "RUNTIME_EFFECT_INTENT", "BEGIN_FLOOR_TRANSITION", ""});
  EXPECT_TRUE(snapshot_may_have_submitted_floor_switch_action(snapshot));

  snapshot.events.back().detail = "SWITCH_FLOOR";
  EXPECT_TRUE(snapshot_may_have_submitted_floor_switch_action(snapshot));

  snapshot.events.clear();
  snapshot.failure_origin_state.clear();
  snapshot.failure_origin_effect_kind = "NONE";
  snapshot.interrupted_state = "PRESSING_CALL_BUTTON";
  EXPECT_FALSE(snapshot_may_have_submitted_floor_switch_action(snapshot));

  snapshot.interrupted_state = "UNKNOWN_LEGACY_STATE";
  EXPECT_TRUE(snapshot_may_have_submitted_floor_switch_action(snapshot));
}

TEST(ElevatorExecutionModule, SourceOutsideCancellationReconcilesAndReleasesAutomatically)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started =
    module.start(start_command("elevator-test-safe-source-cancel"));
  const auto waiting = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");

  const auto accepted = module.cancel(
    {
      waiting.transaction_id,
      waiting.effect_sequence,
      "operator_cancelled_before_entry",
      waiting.operator_id,
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto cancelled = wait_for_state(
    module, waiting.transaction_id, "CANCELLED");

  EXPECT_TRUE(cancelled.terminal);
  EXPECT_EQ(
    cancelled.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_TRUE(cancelled.runtime_applied);
  EXPECT_TRUE(cancelled.runtime_resources_reconciled);
  EXPECT_TRUE(cancelled.safety_hold_state_known);
  EXPECT_FALSE(cancelled.safety_hold_active);
  EXPECT_TRUE(cancelled.dual_odom_stop_proven);
  EXPECT_TRUE(fs::is_regular_file(temporary.recovery_audit(
    cancelled.transaction_id,
    cancelled.effect_sequence,
    "cleanup_release_pending")));
  const auto replacement =
    module.start(start_command("elevator-test-after-safe-source-cancel"));
  EXPECT_TRUE(replacement.accepted()) << replacement.code << ": " <<
    replacement.detail;
}

TEST(ElevatorExecutionModule, CabinEntryFailureBeforeMotionAuthorizationUsesSourceOutsideCleanup)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "hall-call navigation completed", true, true});
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "source-landing navigation completed", true, true});
  ElevatorRuntimeResult entry_failure{
    false,
    "ELEVATOR_NAV2_GOAL_FAILED",
    "Nav2 reached terminal ABORTED before motion authorization",
    true,
    true,
  };
  entry_failure.motion_not_authorized_proven = true;
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose, std::move(entry_failure));
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-entry-no-motion"));
  auto current = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");
  auto reply = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
  current = wait_for_state(
    module, require_snapshot(reply).transaction_id, "WAITING_SOURCE_DOOR");
  reply = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;

  const auto terminal = wait_for_terminal(
    module, require_snapshot(reply).transaction_id);
  EXPECT_EQ(terminal.failure_origin_state, "ENTERING_CABIN");
  EXPECT_EQ(terminal.failure_origin_effect_kind, "NAVIGATE_TO_POSE");
  EXPECT_EQ(terminal.state, "FAILED");
  EXPECT_EQ(
    terminal.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_TRUE(terminal.runtime_resources_reconciled);
  EXPECT_TRUE(terminal.safety_hold_state_known);
  EXPECT_FALSE(terminal.safety_hold_active);
  EXPECT_TRUE(terminal.dual_odom_stop_proven);

  const auto replacement =
    module.start(start_command("elevator-test-after-entry-no-motion"));
  EXPECT_TRUE(replacement.accepted()) << replacement.code << ": " <<
    replacement.detail;
}

TEST(ElevatorExecutionModule, CabinEntryFailureCleansUpWithoutPersistentLock)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "hall-call navigation completed", true, true});
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "source-landing navigation completed", true, true});
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {
      false,
      "ELEVATOR_NAV2_GOAL_FAILED",
      "navigation failed after motion authorization became unknown",
      true,
      true,
    });
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-entry-motion-unknown"));
  auto current = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");
  auto reply = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
  current = wait_for_state(
    module, require_snapshot(reply).transaction_id, "WAITING_SOURCE_DOOR");
  reply = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;

  const auto terminal = wait_for_terminal(
    module, require_snapshot(reply).transaction_id);
  EXPECT_EQ(terminal.failure_origin_state, "ENTERING_CABIN");
  EXPECT_EQ(terminal.state, "FAILED");
  EXPECT_EQ(
    terminal.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_TRUE(terminal.safety_hold_state_known);
  EXPECT_FALSE(terminal.safety_hold_active);
  EXPECT_TRUE(terminal.runtime_resources_reconciled);

  const auto replacement = module.start(
    start_command("elevator-test-after-entry-failure"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(ElevatorExecutionModule, FloorTransitionFailureCleansUpAndAllowsRetry)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kBeginFloorTransition,
    {false, "FLOOR_TRANSITION_FAILED", "target transition was not established"});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-floor-transition-retain"));
  const auto accepted = confirm_through_target_door(module, started);
  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");

  EXPECT_EQ(failed.failure_origin_state, "BEGINNING_FLOOR_TRANSITION");
  EXPECT_EQ(failed.failure_origin_effect_kind, "BEGIN_FLOOR_TRANSITION");
  EXPECT_EQ(
    failed.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_EQ(failed.failure_code, "FLOOR_TRANSITION_FAILED");
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.runtime_resources_reconciled);

  const auto replacement =
    module.start(start_command("elevator-test-after-floor-transition-failure"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(ElevatorExecutionModule, TargetLandingFailureCleansUpAndAllowsRetry)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int navigation = 0; navigation < 3; ++navigation) {
    port->queue_result(
      ElevatorEffectKind::kNavigateToPose,
      {true, "OK", "navigation completed"});
  }
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "TARGET_LANDING_NAVIGATION_FAILED", "target landing goal aborted"});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-target-landing-failure"));
  const auto accepted = confirm_through_target_door(module, started);
  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");

  EXPECT_EQ(failed.failure_origin_state, "NAVIGATING_TARGET_LANDING");
  EXPECT_EQ(failed.failure_origin_effect_kind, "NAVIGATE_TO_POSE");
  EXPECT_EQ(
    failed.cleanup_disposition,
    ElevatorCleanupDisposition::kTargetOutside);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.runtime_resources_reconciled);

  const auto replacement = module.start(
    start_command("elevator-test-after-target-landing-failure"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(ElevatorExecutionModule, AcquiringExitHoldFailureUsesTargetOutsideCleanup)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int hold = 0; hold < 2; ++hold) {
    port->queue_result(
      ElevatorEffectKind::kAcquireSafetyHold,
      {true, "OK", "owner hold acquired", true});
  }
  port->queue_result(
    ElevatorEffectKind::kAcquireSafetyHold,
    {false, "EXIT_HOLD_FAILED", "target landing hold could not be acquired"});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-target-outside-cleanup"));
  const auto accepted = confirm_through_target_door(module, started);
  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");

  EXPECT_EQ(failed.failure_origin_state, "ACQUIRING_EXIT_HOLD");
  EXPECT_EQ(failed.failure_origin_effect_kind, "ACQUIRE_SAFETY_HOLD");
  EXPECT_EQ(
    failed.cleanup_disposition,
    ElevatorCleanupDisposition::kTargetOutside);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
}

TEST(ElevatorExecutionModule, AutomaticEffectsStopAtTheFirstStrictManualGate)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto reply = module.start(start_command());

  ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
  const auto snapshot = wait_for_state(
    module, require_snapshot(reply).transaction_id, "PRESSING_CALL_BUTTON");
  EXPECT_EQ(snapshot.state, "PRESSING_CALL_BUTTON");
  EXPECT_TRUE(snapshot.awaiting_confirmation);
  EXPECT_EQ(snapshot.expected_confirmation, "CALL_BUTTON_PRESSED");
  EXPECT_EQ(snapshot.current_floor_id, "F1");
  EXPECT_EQ(snapshot.current_map_id, "map_f1");
  EXPECT_TRUE(snapshot.safety_hold_state_known);
  EXPECT_TRUE(snapshot.safety_hold_active);
  EXPECT_TRUE(snapshot.dual_odom_stop_proven);
  EXPECT_TRUE(snapshot.runtime_applied);
  EXPECT_EQ(module.active_transaction(), snapshot.transaction_id);
  ASSERT_EQ(port->applied_effects().size(), 2U);
  EXPECT_EQ(
    port->applied_effects()[0].effect.kind,
    ElevatorEffectKind::kNavigateToPose);
  EXPECT_EQ(
    port->applied_effects()[1].effect.kind,
    ElevatorEffectKind::kAcquireSafetyHold);
  EXPECT_EQ(
    count_effect(
      port->applied_effects(), ElevatorEffectKind::kAcquireExecutionLease),
    0U);
  EXPECT_EQ(
    count_effect(
      port->applied_effects(), ElevatorEffectKind::kReleaseExecutionLease),
    0U);
  ASSERT_TRUE(port->applied_effects()[0].target_pose.has_value());
  EXPECT_DOUBLE_EQ(port->applied_effects()[0].target_pose->x, 0.0);
  ASSERT_TRUE(port->prepared_release().has_value());
  EXPECT_EQ(port->prepared_release()->release_id, snapshot.pinned_release_id);
}

TEST(ElevatorExecutionModule, AutomaticButtonPortKeepsOnlyPhysicalObservationsManual)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{true, true, true, true});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(start_command("elevator-test-auto-buttons"));
  ASSERT_TRUE(started.accepted()) << started.code << ": " << started.detail;

  auto current = wait_for_state(
    module, require_snapshot(started).transaction_id, "WAITING_SOURCE_DOOR");
  EXPECT_TRUE(current.awaiting_confirmation);
  EXPECT_EQ(current.expected_confirmation, "SOURCE_DOOR_OPEN");
  ASSERT_GE(port->applied_effects().size(), 3U);
  EXPECT_EQ(
    port->applied_effects()[2].effect.kind,
    ElevatorEffectKind::kMockPressCallButton);

  auto advanced = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(advanced.accepted()) << advanced.code << ": " << advanced.detail;
  current = wait_for_state(
    module, require_snapshot(advanced).transaction_id, "RIDING");
  EXPECT_TRUE(current.awaiting_confirmation);
  EXPECT_EQ(current.expected_confirmation, "TARGET_FLOOR_ARRIVED");
  const auto effects = port->applied_effects();
  EXPECT_NE(
    std::find_if(
      effects.begin(), effects.end(),
      [](const ElevatorRuntimeEffect & effect) {
        return effect.effect.kind ==
               ElevatorEffectKind::kMockPressTargetButton;
      }),
    effects.end());
}

TEST(ElevatorExecutionModule, AutomaticCallCanExplicitlyFallBackToManualConfirmation)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{true, true, true, true});
  ElevatorRuntimeResult manual_fallback{
    true,
    "ELEVATOR_CALL_MANUAL_CONFIRMATION_REQUIRED",
    "physical hall-call capability is unavailable; arm is stowed",
  };
  manual_fallback.operator_confirmation_required = true;
  port->queue_result(
    ElevatorEffectKind::kMockPressCallButton, manual_fallback);
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(
    start_command("elevator-test-auto-call-fallback"));
  ASSERT_TRUE(started.accepted()) << started.code << ": " << started.detail;
  const auto current = wait_for_confirmation(
    module, require_snapshot(started).transaction_id,
    "PRESSING_CALL_BUTTON", "CALL_BUTTON_PRESSED");

  EXPECT_TRUE(current.awaiting_confirmation);
  EXPECT_EQ(current.expected_confirmation, "CALL_BUTTON_PRESSED");
  EXPECT_NE(
    std::find_if(
      current.events.begin(), current.events.end(),
      [](const ElevatorExecutionEventRecord & event) {
        return event.code == "AUTOMATIC_BUTTON_MANUAL_FALLBACK";
      }),
    current.events.end());
  const auto confirmed = module.confirm(confirmation(current, "F1"));
  ASSERT_TRUE(confirmed.accepted())
    << confirmed.code << ": " << confirmed.detail;
  EXPECT_EQ(
    wait_for_state(
      module, current.transaction_id, "WAITING_SOURCE_DOOR").state,
    "WAITING_SOURCE_DOOR");
}

TEST(ElevatorExecutionModule, ConfirmationIdentityMustMatchEveryGateField)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command());
  const auto first = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");

  auto stale = confirmation(first, "F1");
  --stale.effect_sequence;
  EXPECT_EQ(module.confirm(stale).kind, ElevatorExecutionReplyKind::kConflict);

  auto wrong_state = confirmation(first, "F1");
  wrong_state.expected_state = "WAITING_SOURCE_DOOR";
  EXPECT_EQ(module.confirm(wrong_state).kind, ElevatorExecutionReplyKind::kConflict);

  auto wrong_event = confirmation(first, "F1");
  wrong_event.event = "SOURCE_DOOR_OPEN";
  EXPECT_EQ(module.confirm(wrong_event).kind, ElevatorExecutionReplyKind::kConflict);

  auto wrong_floor = confirmation(first, "F2");
  EXPECT_EQ(module.confirm(wrong_floor).kind, ElevatorExecutionReplyKind::kConflict);

  auto wrong_operator = confirmation(first, "F1");
  wrong_operator.operator_id = "another_operator";
  EXPECT_EQ(module.confirm(wrong_operator).kind, ElevatorExecutionReplyKind::kConflict);

  const auto advanced = module.confirm(confirmation(first, "F1"));
  ASSERT_TRUE(advanced.accepted());
  const auto source_door = wait_for_state(
    module, require_snapshot(advanced).transaction_id, "WAITING_SOURCE_DOOR");
  EXPECT_EQ(source_door.state, "WAITING_SOURCE_DOOR");
  EXPECT_EQ(source_door.expected_confirmation, "SOURCE_DOOR_OPEN");
  EXPECT_TRUE(source_door.safety_hold_active);
}

TEST(ElevatorExecutionModule, FiveConfirmationsReachCompleteInOrder)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  auto reply = module.start(start_command());
  auto current = wait_for_state(
    module, require_snapshot(reply).transaction_id, "PRESSING_CALL_BUTTON");

  const std::vector<std::pair<std::string, std::string>> expected{
    {"CALL_BUTTON_PRESSED", "F1"},
    {"SOURCE_DOOR_OPEN", "F1"},
    {"TARGET_BUTTON_PRESSED", "F1"},
    {"TARGET_FLOOR_ARRIVED", "F2"},
    {"TARGET_DOOR_OPEN", "F2"},
  };
  for (const auto & step : expected) {
    ASSERT_EQ(current.expected_confirmation, step.first);
    reply = module.confirm(confirmation(current, step.second));
    ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
    const std::string next_state =
      step.first == "CALL_BUTTON_PRESSED" ? "WAITING_SOURCE_DOOR" :
      step.first == "SOURCE_DOOR_OPEN" ? "PRESSING_TARGET_BUTTON" :
      step.first == "TARGET_BUTTON_PRESSED" ? "RIDING" :
      step.first == "TARGET_FLOOR_ARRIVED" ? "WAITING_TARGET_DOOR" :
      "COMPLETE";
    current = wait_for_state(
      module, require_snapshot(reply).transaction_id, next_state);
  }

  const auto completed = current;
  EXPECT_EQ(completed.state, "COMPLETE");
  EXPECT_TRUE(completed.terminal);
  EXPECT_FALSE(completed.awaiting_confirmation);
  EXPECT_EQ(completed.current_floor_id, "F2");
  EXPECT_EQ(completed.current_map_id, "map_f2");
  EXPECT_TRUE(completed.safety_hold_state_known);
  EXPECT_FALSE(completed.safety_hold_active);
  EXPECT_FALSE(completed.motion_authorized);
  EXPECT_FALSE(module.active_transaction().has_value());
}

TEST(ElevatorExecutionModule, ReleaseSuccessWithoutAbsenceProofRemainsUnknown)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorRuntimeResult proven_release{true, "OK", "owner hold absent"};
  proven_release.safety_hold_absence_proven = true;
  port->queue_result(
    ElevatorEffectKind::kReleaseSafetyHold, proven_release);
  port->queue_result(
    ElevatorEffectKind::kReleaseSafetyHold, proven_release);
  port->queue_result(
    ElevatorEffectKind::kReleaseSafetyHold,
    {true, "OK", "release response omitted hold-state evidence"});
  ElevatorExecutionModule module(temporary.journal(), port);
  auto reply =
    module.start(start_command("elevator-test-release-evidence"));
  auto current = wait_for_state(
    module, require_snapshot(reply).transaction_id, "PRESSING_CALL_BUTTON");

  const std::vector<std::pair<std::string, std::string>> confirmations{
    {"CALL_BUTTON_PRESSED", "F1"},
    {"SOURCE_DOOR_OPEN", "F1"},
    {"TARGET_BUTTON_PRESSED", "F1"},
    {"TARGET_FLOOR_ARRIVED", "F2"},
    {"TARGET_DOOR_OPEN", "F2"},
  };
  for (const auto & step : confirmations) {
    reply = module.confirm(confirmation(current, step.second));
    ASSERT_TRUE(reply.accepted()) << reply.code << ": " << reply.detail;
    const std::string next_state =
      step.first == "CALL_BUTTON_PRESSED" ? "WAITING_SOURCE_DOOR" :
      step.first == "SOURCE_DOOR_OPEN" ? "PRESSING_TARGET_BUTTON" :
      step.first == "TARGET_BUTTON_PRESSED" ? "RIDING" :
      step.first == "TARGET_FLOOR_ARRIVED" ? "WAITING_TARGET_DOOR" :
      "COMPLETE";
    current = wait_for_state(
      module, require_snapshot(reply).transaction_id, next_state);
  }

  EXPECT_TRUE(current.terminal);
  EXPECT_FALSE(current.safety_hold_state_known);
  EXPECT_TRUE(current.safety_hold_active);
  const auto replacement =
    module.start(start_command("elevator-test-after-unknown-release"));
  EXPECT_EQ(replacement.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(replacement.code, "ELEVATOR_EXECUTION_RECOVERY_REQUIRED");
}

TEST(ElevatorExecutionModule, CancellationRunsHoldAndCancelAndEndsCancelled)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command());
  const auto snapshot = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");

  const auto cancelled = module.cancel(
    {
      snapshot.transaction_id,
      snapshot.effect_sequence,
      "operator_cancelled",
      snapshot.operator_id,
    });

  ASSERT_TRUE(cancelled.accepted());
  const auto terminal = wait_for_state(
    module, require_snapshot(cancelled).transaction_id, "CANCELLED");
  EXPECT_EQ(terminal.state, "CANCELLED");
  EXPECT_TRUE(terminal.terminal);
  EXPECT_TRUE(terminal.safety_hold_state_known);
  EXPECT_FALSE(terminal.safety_hold_active);
  EXPECT_TRUE(terminal.dual_odom_stop_proven);
  EXPECT_TRUE(terminal.runtime_resources_reconciled);
  ASSERT_FALSE(port->applied_effects().empty());
  EXPECT_EQ(
    port->applied_effects().back().effect.kind,
    ElevatorEffectKind::kHoldAndCancel);
  ASSERT_EQ(port->cancellation_requests().size(), 1U);
  EXPECT_EQ(port->cancellation_requests().front(), snapshot.transaction_id);

  const auto replay = module.cancel(
    {
      terminal.transaction_id,
      terminal.effect_sequence,
      "operator_cancelled_again",
      terminal.operator_id,
    });
  EXPECT_TRUE(replay.accepted());
  EXPECT_EQ(require_snapshot(replay).state, "CANCELLED");
}

TEST(ElevatorExecutionModule, CancellationInterruptsABlockedAutomaticEffectImmediately)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<BlockingNavigationPort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command());
  const auto transaction_id = require_snapshot(started).transaction_id;
  ASSERT_TRUE(port->wait_until_navigation_entered());
  const auto active = require_snapshot(module.snapshot(transaction_id));
  ASSERT_EQ(active.state, "NAVIGATING_HALL_CALL");

  const auto before = std::chrono::steady_clock::now();
  const auto cancelled = module.cancel(
    {
      active.transaction_id,
      active.effect_sequence,
      "interrupt_blocked_navigation",
      active.operator_id,
    });
  const auto elapsed = std::chrono::steady_clock::now() - before;

  ASSERT_TRUE(cancelled.accepted());
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));
  EXPECT_EQ(
    wait_for_state(module, transaction_id, "CANCELLED").state,
    "CANCELLED");
}

TEST(ElevatorExecutionModule, PortFailureCompletesSafetyCleanupAsFailed)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto reply = module.start(start_command());

  ASSERT_TRUE(reply.accepted());
  const auto failed = wait_for_state(
    module, require_snapshot(reply).transaction_id, "FAILED");
  EXPECT_EQ(failed.state, "FAILED");
  EXPECT_TRUE(failed.terminal);
  EXPECT_EQ(failed.failure_code, "NAVIGATION_FAILED");
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.dual_odom_stop_proven);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  ASSERT_EQ(port->applied_effects().size(), 2U);
  EXPECT_EQ(
    port->applied_effects().back().effect.kind,
    ElevatorEffectKind::kHoldAndCancel);
}

TEST(ElevatorExecutionModule, SafeOutsideFailureReleasesAndAllowsReplacement)
{
  TemporaryDirectory temporary;
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    port->queue_result(
      ElevatorEffectKind::kNavigateToPose,
      {false, "NAVIGATION_FAILED", "hall goal aborted"});
    ElevatorExecutionModule module(temporary.journal(), port);

    const auto first = module.start(start_command("elevator-test-1"));
    const auto failed = wait_for_state(
      module, require_snapshot(first).transaction_id, "FAILED");
    ASSERT_TRUE(failed.terminal);
    ASSERT_TRUE(failed.safety_hold_state_known);
    ASSERT_FALSE(failed.safety_hold_active);
    ASSERT_TRUE(failed.runtime_resources_reconciled);

    const auto replacement = module.start(start_command("elevator-test-2"));

    EXPECT_TRUE(replacement.accepted())
      << replacement.code << ": " << replacement.detail;
    ASSERT_TRUE(replacement.snapshot.has_value());
    EXPECT_EQ(replacement.snapshot->transaction_id, "elevator-test-2");
  }
}

TEST(ElevatorExecutionModule, HeartbeatFailureDuringManualWaitFailsClosed)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_heartbeat_result(
    {false, "EXECUTION_LEASE_HEARTBEAT_FAILED", "lease renewal rejected"});
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command());

  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");

  EXPECT_EQ(
    failed.failure_code,
    "EXECUTION_LEASE_HEARTBEAT_FAILED");
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.dual_odom_stop_proven);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
}

TEST(ElevatorExecutionModule, FailedCleanupRetriesAutomaticallyThenSucceeds)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  port->queue_result(
    ElevatorEffectKind::kHoldAndCancel,
    {false, "SAFETY_HOLD_UNAVAILABLE", "hold request timed out"});
  port->queue_result(
    ElevatorEffectKind::kHoldAndCancel,
    {
      true,
      "OK",
      "hold, dual stop, and resource absence proven",
      true,
      true,
      ElevatorRuntimeFailureDisposition::kCleanupRequired,
      false,
      true,
    });
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command());
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");
  EXPECT_TRUE(failed.terminal);
  EXPECT_EQ(failed.failure_code, "NAVIGATION_FAILED");
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.dual_odom_stop_proven);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  EXPECT_EQ(
    count_effect(
      port->applied_effects(), ElevatorEffectKind::kHoldAndCancel),
    2U);
}

TEST(ElevatorExecutionModule, PreparePreflightRejectionDoesNotEnterCleanupOrLock)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_prepare_result(
    {
      false,
      "ELEVATOR_SOURCE_RUNTIME_BUSY",
      "navigation_idle=false;mapping_idle=true;docking_idle=true",
      false,
      false,
      ElevatorRuntimeFailureDisposition::kRejectedBeforeEffects,
    });
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "ELEVATOR_RUNTIME_NOT_PREPARED",
        "no frozen elevator release is bound",
      });
  }
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(start_command());
  const auto failed = wait_for_terminal(
    module, require_snapshot(started).transaction_id);

  EXPECT_EQ(failed.state, "FAILED");
  EXPECT_EQ(failed.phase, "RUNTIME_PREFLIGHT");
  EXPECT_TRUE(failed.terminal);
  EXPECT_EQ(failed.failure_code, "ELEVATOR_SOURCE_RUNTIME_BUSY");
  EXPECT_FALSE(failed.runtime_applied);
  EXPECT_FALSE(failed.motion_authorized);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_FALSE(failed.dual_odom_stop_proven);
  EXPECT_TRUE(port->applied_effects().empty());
  EXPECT_FALSE(port->prepared_release().has_value());
  EXPECT_FALSE(module.active_transaction().has_value());
}

TEST(ElevatorExecutionModule, PreparePreflightRejectionRemainsRetryableAfterRestart)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_prepare_result(
    {
      false,
      "ELEVATOR_SOURCE_RUNTIME_BUSY",
      "navigation_idle=false;mapping_idle=true;docking_idle=true",
      false,
      false,
      ElevatorRuntimeFailureDisposition::kRejectedBeforeEffects,
    });
  {
    ElevatorExecutionModule first(temporary.journal(), port);
    const auto started = first.start(start_command("elevator-test-rejected"));
    const auto failed = wait_for_terminal(
      first, require_snapshot(started).transaction_id);
    ASSERT_EQ(failed.state, "FAILED");
    ASSERT_EQ(failed.phase, "RUNTIME_PREFLIGHT");
  }

  ElevatorExecutionModule restarted(temporary.journal(), port);
  const auto retry = restarted.start(start_command("elevator-test-retry"));

  EXPECT_TRUE(retry.accepted()) << retry.code << ": " << retry.detail;
  EXPECT_NE(
    require_snapshot(retry).state,
    "LOCKED");
  EXPECT_EQ(
    restarted.active_transaction(),
    "elevator-test-retry");
}

TEST(
  ElevatorExecutionModule,
  TerminalAppliedUnreconciledJournalRestartsLockedAndCanRecover)
{
  TemporaryDirectory temporary;
  const std::string transaction_id =
    "elevator-test-terminal-unreconciled";
  create_clean_v3_terminal_journal(temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  auto snapshot = root["snapshot"];
  snapshot["state"] = "FAILED";
  snapshot["phase"] = "FAILED";
  snapshot["terminal"] = true;
  snapshot["runtime_capable"] = true;
  snapshot["runtime_applied"] = true;
  snapshot["runtime_resources_reconciled"] = false;
  snapshot["motion_authorized"] = false;
  snapshot["floor_switch_capable"] = true;
  snapshot["safety_hold_state_known"] = true;
  snapshot["safety_hold_active"] = false;
  snapshot["dual_odom_stop_proven"] = true;
  snapshot["failure_code"] = "NAVIGATION_FAILED";
  snapshot["detail"] =
    "terminal failure persisted before runtime resources were reconciled";
  snapshot["failure_origin_state"] = "NAVIGATING_SOURCE_HALL";
  snapshot["failure_origin_effect_kind"] = "NAVIGATE_TO_POSE";
  snapshot["cleanup_disposition"] = "SOURCE_OUTSIDE";
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorRuntimeResult restart_unproven;
  restart_unproven.success = false;
  restart_unproven.code = "RESTART_PROOF_NOT_READY";
  restart_unproven.detail = "fresh owner hold and stop proof unavailable";
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_recovery_result(restart_unproven);
  }

  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_failure_code(
    module, transaction_id,
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN");

  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_TRUE(locked.terminal);
  EXPECT_TRUE(locked.runtime_applied);
  EXPECT_FALSE(locked.runtime_resources_reconciled);
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);

  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto recovered = wait_for_state(module, transaction_id, "FAILED");
  EXPECT_TRUE(recovered.runtime_applied);
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
  EXPECT_TRUE(recovered.safety_hold_state_known);
  EXPECT_FALSE(recovered.safety_hold_active);
}

TEST(
  ElevatorExecutionModule,
  LegacyRetainedFloorTransitionLockIsAutomaticallyCleanedOnRestart)
{
  TemporaryDirectory temporary;
  const std::string transaction_id =
    "elevator-test-legacy-floor-transition-lock";
  create_clean_v3_terminal_journal(temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  auto snapshot = root["snapshot"];
  snapshot["state"] = "LOCKED";
  snapshot["phase"] = "LOCKED";
  snapshot["terminal"] = true;
  snapshot["runtime_capable"] = true;
  snapshot["runtime_applied"] = true;
  snapshot["runtime_resources_reconciled"] = false;
  snapshot["motion_authorized"] = false;
  snapshot["floor_switch_capable"] = true;
  snapshot["safety_hold_state_known"] = true;
  snapshot["safety_hold_active"] = true;
  snapshot["dual_odom_stop_proven"] = true;
  snapshot["failure_code"] =
    "ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED";
  snapshot["detail"] = "legacy retained floor-transition lock";
  snapshot["failure_origin_state"] = "BEGINNING_FLOOR_TRANSITION";
  snapshot["failure_origin_effect_kind"] = "BEGIN_FLOOR_TRANSITION";
  snapshot["cleanup_disposition"] = "RETAIN_LOCK";
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto failed = wait_for_state(module, transaction_id, "FAILED");

  EXPECT_EQ(
    failed.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  const auto replacement = module.start(
    start_command("elevator-test-after-legacy-floor-transition-lock"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(
  ElevatorExecutionModule,
  ExhaustedOrdinaryStageRestartRecoveryIsFinalizedOnColdStart)
{
  TemporaryDirectory temporary;
  const std::string transaction_id =
    "elevator-test-exhausted-restart-recovery";
  create_clean_v3_terminal_journal(temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  auto snapshot = root["snapshot"];
  snapshot["state"] = "LOCKED";
  snapshot["phase"] = "LOCKED";
  snapshot["terminal"] = true;
  snapshot["runtime_capable"] = true;
  snapshot["runtime_applied"] = true;
  snapshot["runtime_resources_reconciled"] = false;
  snapshot["motion_authorized"] = false;
  snapshot["floor_switch_capable"] = true;
  snapshot["safety_hold_state_known"] = false;
  snapshot["safety_hold_active"] = true;
  snapshot["dual_odom_stop_proven"] = false;
  snapshot["failure_code"] =
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN";
  snapshot["detail"] =
    "restart startup recovery remained unproven after 40 attempts; "
    "all state-changing APIs remain locked";
  snapshot["failure_origin_state"] = "BEGINNING_FLOOR_TRANSITION";
  snapshot["failure_origin_effect_kind"] = "BEGIN_FLOOR_TRANSITION";
  snapshot["cleanup_disposition"] = "SOURCE_OUTSIDE";
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_recovery_result(
      {
        false,
        "ELEVATOR_RESTART_RECOVERY_TEST_FAILURE",
        "late runtime endpoints must not be required after a full-chain restart",
      });
  }

  ElevatorExecutionModule module(temporary.journal(), port);
  const auto failed = wait_for_state(module, transaction_id, "FAILED");

  EXPECT_TRUE(failed.terminal);
  EXPECT_TRUE(failed.runtime_applied);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  EXPECT_FALSE(failed.motion_authorized);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_FALSE(failed.dual_odom_stop_proven);
  EXPECT_EQ(
    failed.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  EXPECT_EQ(
    failed.failure_code,
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN");
  EXPECT_TRUE(port->cancellation_requests().empty());

  const auto replacement = module.start(
    start_command("elevator-test-after-exhausted-restart-recovery"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(ElevatorExecutionModule, PreparePreexistingStateLocksWithoutInvalidCleanup)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_prepare_result(
    {
      false,
      "ELEVATOR_SAFETY_INTERLOCK_STATE_STALE",
      "fresh robot_safety interlock state was not observed",
      false,
      false,
      ElevatorRuntimeFailureDisposition::kRecoveryRequiredBeforeEffects,
    });
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "ELEVATOR_RUNTIME_NOT_PREPARED",
        "no frozen elevator release is bound",
      });
  }
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started =
    module.start(start_command("elevator-test-preexisting-runtime"));
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  EXPECT_TRUE(locked.terminal);
  EXPECT_FALSE(locked.runtime_applied);
  EXPECT_FALSE(locked.safety_hold_state_known);
  EXPECT_EQ(
    locked.failure_code,
    "ELEVATOR_SAFETY_INTERLOCK_STATE_STALE");
  EXPECT_TRUE(port->applied_effects().empty());
}

TEST(ElevatorExecutionModule, ThreeFailedCleanupAttemptsEndLocked)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "SAFETY_CLEANUP_UNPROVEN",
        "hold or dual odometry proof missing",
        false,
        false,
      });
  }
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(start_command());
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  EXPECT_TRUE(locked.terminal);
  EXPECT_EQ(
    locked.failure_code,
    "ELEVATOR_FAILURE_CLEANUP_UNPROVEN");
  EXPECT_FALSE(locked.safety_hold_state_known);
  EXPECT_FALSE(locked.safety_hold_active);
  EXPECT_FALSE(locked.dual_odom_stop_proven);
  EXPECT_FALSE(module.active_transaction().has_value());
  EXPECT_EQ(
    count_effect(
      port->applied_effects(), ElevatorEffectKind::kHoldAndCancel),
    3U);
}

TEST(ElevatorExecutionModule, NonlockingPolicyRetriesCleanupInsteadOfPersistingLock)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "SAFETY_CLEANUP_UNPROVEN",
        "hold or dual odometry proof missing",
        false,
        false,
      });
  }
  ElevatorExecutionModule module(
    temporary.journal(), port, ElevatorExecutionOptions{false});

  const auto started = module.start(start_command("elevator-test-nonlocking"));
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "FAILED");

  EXPECT_TRUE(failed.terminal);
  EXPECT_NE(failed.state, "LOCKED");
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(failed.runtime_resources_reconciled);
  EXPECT_FALSE(module.active_transaction().has_value());
  EXPECT_GE(
    count_effect(port->applied_effects(), ElevatorEffectKind::kHoldAndCancel),
    4U);
}

TEST(ElevatorExecutionModule, CleanupSuccessWithoutProofDoesNotInventSafetyFacts)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {true, "OK", "adapter returned success without evidence", false, false});
  }
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto started = module.start(start_command());
  const auto failed = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  EXPECT_TRUE(failed.terminal);
  EXPECT_FALSE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_FALSE(failed.dual_odom_stop_proven);
  const auto persisted = YAML::LoadFile(temporary.journal().string());
  EXPECT_EQ(
    persisted["snapshot_schema_version"].as<std::uint32_t>(), 3U);
  EXPECT_FALSE(
    persisted["snapshot"]["safety_hold_state_known"].as<bool>());
  EXPECT_TRUE(
    persisted["snapshot"]["safety_hold_active"].as<bool>())
    << "the v1 compatibility projection must keep older readers locked";

  const auto replacement =
    module.start(start_command("elevator-test-unknown-hold"));
  EXPECT_EQ(replacement.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(replacement.code, "ELEVATOR_EXECUTION_RECOVERY_REQUIRED");
}

TEST(ElevatorExecutionModule, JournalFailurePreservesLastAuthoritativeSafetyEvidence)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started = module.start(start_command("elevator-test-storage"));
  const auto active = wait_for_state(
    module, require_snapshot(started).transaction_id, "PRESSING_CALL_BUTTON");
  ASSERT_TRUE(active.safety_hold_state_known);
  ASSERT_TRUE(active.safety_hold_active);
  ASSERT_TRUE(active.dual_odom_stop_proven);

  std::error_code error;
  ASSERT_TRUE(fs::remove(temporary.journal(), error)) << error.message();
  ASSERT_TRUE(fs::remove(temporary.journal().parent_path(), error))
    << error.message();
  {
    std::ofstream blocker(temporary.journal().parent_path());
    ASSERT_TRUE(blocker.good());
    blocker << "not-a-directory";
  }

  const auto failed = module.confirm(confirmation(active, "F1"));

  EXPECT_EQ(failed.kind, ElevatorExecutionReplyKind::kStorageFailure);
  const auto & locked = require_snapshot(failed);
  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_EQ(
    locked.failure_code,
    "ELEVATOR_EXECUTION_JOURNAL_WRITE_FAILED");
  EXPECT_TRUE(locked.safety_hold_state_known);
  EXPECT_TRUE(locked.safety_hold_active);
  EXPECT_TRUE(locked.dual_odom_stop_proven);
}

TEST(ElevatorExecutionModule, FirstJournalFailureDoesNotCreateAnOrphanHold)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  std::error_code error;
  ASSERT_TRUE(fs::remove(temporary.journal().parent_path(), error))
    << error.message();
  {
    std::ofstream blocker(temporary.journal().parent_path());
    ASSERT_TRUE(blocker.good());
    blocker << "not-a-directory";
  }

  const auto failed =
    module.start(start_command("elevator-test-first-write-failure"));

  EXPECT_EQ(failed.kind, ElevatorExecutionReplyKind::kStorageFailure);
  const auto & locked = require_snapshot(failed);
  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_TRUE(locked.safety_hold_state_known);
  EXPECT_FALSE(locked.safety_hold_active);
  EXPECT_FALSE(locked.runtime_applied);
  EXPECT_TRUE(port->cancellation_requests().empty());
}

TEST(ElevatorExecutionModule, ActiveJournalUsesV3SnapshotInV1CompatibilityEnvelope)
{
  TemporaryDirectory temporary;
  auto incapable = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{false, false, false});
  {
    ElevatorExecutionModule module(temporary.journal(), incapable);
    const auto result =
      module.start(start_command("elevator-test-schema-three"));
    ASSERT_TRUE(result.accepted()) << result.code << ": " << result.detail;
  }

  const auto root = YAML::LoadFile(temporary.journal().string());
  EXPECT_EQ(root["schema_version"].as<std::uint32_t>(), 1U);
  EXPECT_EQ(root["snapshot_schema_version"].as<std::uint32_t>(), 3U);
  EXPECT_TRUE(root["snapshot"]["safety_hold_state_known"].as<bool>());
  EXPECT_FALSE(root["snapshot"]["safety_hold_active"].as<bool>());
}

TEST(ElevatorExecutionModule, V3JournalRequiresSafetyHoldActive)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-v3-missing-hold");
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"].remove("safety_hold_active");
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
  const auto reply =
    module.start(start_command("elevator-test-after-v3-missing-hold"));
  EXPECT_EQ(reply.kind, ElevatorExecutionReplyKind::kStorageFailure);
  EXPECT_NE(reply.detail.find("invalid"), std::string::npos);
}

TEST(ElevatorExecutionModule, V3JournalRejectsWrongSafetyHoldType)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-v3-wrong-hold-type");
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"]["safety_hold_active"] = "not-a-boolean";
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
}

TEST(ElevatorExecutionModule, V3JournalRejectsContradictoryUnknownHoldProjection)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-v3-contradictory-hold");
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"]["safety_hold_state_known"] = false;
  root["snapshot"]["safety_hold_active"] = false;
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
}

TEST(ElevatorExecutionModule, V3JournalRejectsTerminalStateContradiction)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-v3-terminal-contradiction");
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"]["terminal"] = false;
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
}

TEST(ElevatorExecutionModule, MalformedSnapshotSchemaVersionIsNotLegacy)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-invalid-schema-type");
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot_schema_version"] = "not-a-version";
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
}

TEST(ElevatorExecutionModule, MissingSchemaVersionCannotDowngradeAV3Snapshot)
{
  TemporaryDirectory temporary;
  create_clean_v3_terminal_journal(
    temporary.journal(), "elevator-test-missing-v3-schema");
  auto root = YAML::LoadFile(temporary.journal().string());
  root.remove("snapshot_schema_version");
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
}

TEST(ElevatorExecutionModule, DeployedV2PreflightOrphanStillUsesExplicitRecovery)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v2-preflight-orphan";
  create_deployed_v2_preflight_orphan(
    temporary.journal(), transaction_id);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_FALSE(locked.runtime_applied);
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto recovered = wait_for_state(module, transaction_id, "FAILED");
  EXPECT_EQ(recovered.phase, "RECOVERY_COMPLETE");
  EXPECT_FALSE(recovered.safety_hold_active);
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
}

TEST(ElevatorExecutionModule, V2RuntimeAppliedWithoutPhaseFieldsFailsClosed)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v2-runtime-unknown";
  create_clean_v3_terminal_journal(temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"]["state"] = "LOCKED";
  root["snapshot"]["phase"] = "LOCKED";
  root["snapshot"]["effect_sequence"] = 7U;
  root["snapshot"]["terminal"] = true;
  root["snapshot"]["runtime_applied"] = true;
  write_yaml_file(temporary.journal(), root);
  downgrade_journal_to_v2(temporary.journal(), true);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_TRUE(locked.runtime_applied);
  EXPECT_EQ(locked.failure_origin_state, "LEGACY_UNKNOWN");
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kRetainLock);
  const auto rejected = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  EXPECT_EQ(rejected.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(
    rejected.code,
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRES_SERVICE");
}

TEST(ElevatorExecutionModule, V2RuntimeAppliedFalseOutsideWhitelistFailsClosed)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v2-false-nonpreflight";
  create_deployed_v2_preflight_orphan(
    temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  root["snapshot"]["events"].push_back(
    YAML::Load(
      "{sequence: 1, code: RUNTIME_EFFECT_SUCCEEDED, "
      "detail: NAVIGATE_TO_POSE, timestamp: 2000-01-01T00:00:00Z}"));
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_FALSE(locked.runtime_applied);
  EXPECT_EQ(locked.failure_origin_state, "LEGACY_UNKNOWN");
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kRetainLock);
  const auto rejected = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  EXPECT_EQ(rejected.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(
    rejected.code,
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRES_SERVICE");
}

TEST(
  ElevatorExecutionModule,
  ForgedLegacyOutsideDispositionNeverBypassesExactWhitelist)
{
  TemporaryDirectory temporary;
  for (const auto schema_version : {1U, 2U}) {
    const auto journal =
      temporary.journal().parent_path() /
      ("legacy-v" + std::to_string(schema_version)) / "active.yaml";
    ASSERT_TRUE(fs::create_directories(journal.parent_path()));
    const std::string transaction_id =
      "elevator-test-forged-v" + std::to_string(schema_version);
    create_deployed_v2_preflight_orphan(journal, transaction_id);
    auto root = YAML::LoadFile(journal.string());
    if (schema_version == 1U) {
      root.remove("snapshot_schema_version");
      root["snapshot"].remove("safety_hold_state_known");
    }
    // This extra effect record takes the journal outside the only recoverable
    // preflight whitelist. A legacy producer never owned the v3 disposition
    // field, so a forged value must be ignored for both v1 and v2.
    root["snapshot"]["events"].push_back(
      YAML::Load(
        "{sequence: 1, code: RUNTIME_EFFECT_SUCCEEDED, "
        "detail: NAVIGATE_TO_POSE, timestamp: 2000-01-01T00:00:00Z}"));
    root["snapshot"]["cleanup_disposition"] = "SOURCE_OUTSIDE";
    write_yaml_file(journal, root);

    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(journal, port);
    const auto locked = wait_for_recovery_ready(module, transaction_id);

    EXPECT_EQ(locked.failure_origin_state, "LEGACY_UNKNOWN");
    EXPECT_EQ(
      locked.cleanup_disposition,
      ElevatorCleanupDisposition::kRetainLock);
    const auto rejected = module.recover(
      ElevatorExecutionRecovery{
        locked.transaction_id,
        locked.state,
        locked.effect_sequence,
        locked.operator_id,
        "field_verified_stationary",
      });
    EXPECT_EQ(rejected.kind, ElevatorExecutionReplyKind::kConflict);
    EXPECT_EQ(
      rejected.code,
      "ELEVATOR_EXECUTION_RECOVERY_REQUIRES_SERVICE");
  }
}

TEST(
  ElevatorExecutionModule,
  LegacyPreflightWhitelistRejectsUnknownAndOutOfOrderEvents)
{
  TemporaryDirectory temporary;
  for (const auto schema_version : {1U, 2U}) {
    for (const std::string anomaly : {"unknown_event", "out_of_order"}) {
      const auto journal =
        temporary.journal().parent_path() /
        ("legacy-v" + std::to_string(schema_version) + "-" + anomaly) /
        "active.yaml";
      const std::string transaction_id =
        "elevator-test-v" + std::to_string(schema_version) + "-" + anomaly;
      create_deployed_v2_preflight_orphan(journal, transaction_id);
      auto root = YAML::LoadFile(journal.string());
      if (schema_version == 1U) {
        root.remove("snapshot_schema_version");
        root["snapshot"].remove("safety_hold_state_known");
      }

      auto events = root["snapshot"]["events"];
      ASSERT_GE(events.size(), 3U);
      if (anomaly == "unknown_event") {
        events.push_back(
          YAML::Load(
            "{sequence: 1, code: RUNTIME_EFFECT_INTENT, "
            "detail: NAVIGATE_TO_POSE, timestamp: 2000-01-01T00:00:00Z}"));
      } else {
        YAML::Node reordered(YAML::NodeType::Sequence);
        reordered.push_back(events[0]);
        reordered.push_back(events[2]);
        reordered.push_back(events[1]);
        for (std::size_t index = 3U; index < events.size(); ++index) {
          reordered.push_back(events[index]);
        }
        root["snapshot"]["events"] = reordered;
      }
      write_yaml_file(journal, root);

      auto port = std::make_shared<InMemoryElevatorRuntimePort>();
      ElevatorExecutionModule module(journal, port);
      const auto retained = module.snapshot(transaction_id);
      ASSERT_TRUE(retained.accepted())
        << retained.code << ": " << retained.detail;
      EXPECT_EQ(
        require_snapshot(retained).cleanup_disposition,
        ElevatorCleanupDisposition::kRetainLock)
        << "schema=" << schema_version << "; anomaly=" << anomaly;
      EXPECT_EQ(require_snapshot(retained).state, "LOCKED")
        << "schema=" << schema_version << "; anomaly=" << anomaly;
    }
  }
}

TEST(
  ElevatorExecutionModule,
  LegacyCleanCompleteWithErrorsAndPendingConfirmationRetainsLock)
{
  TemporaryDirectory temporary;
  for (const auto schema_version : {1U, 2U}) {
    const auto journal =
      temporary.journal().parent_path() /
      ("legacy-complete-v" + std::to_string(schema_version)) /
      "active.yaml";
    const std::string transaction_id =
      "elevator-test-complete-v" + std::to_string(schema_version);
    {
      auto port = std::make_shared<InMemoryElevatorRuntimePort>();
      ElevatorExecutionModule module(journal, port);
      const auto started = module.start(start_command(transaction_id));
      const auto completed_reply =
        confirm_through_target_door(module, started);
      ASSERT_TRUE(completed_reply.accepted());
      (void)wait_for_state(module, transaction_id, "COMPLETE");
    }
    downgrade_journal_to_v2(journal, true);
    auto root = YAML::LoadFile(journal.string());
    if (schema_version == 1U) {
      root.remove("snapshot_schema_version");
      root["snapshot"].remove("safety_hold_state_known");
    }
    root["snapshot"]["awaiting_confirmation"] = true;
    root["snapshot"]["expected_confirmation"] = "TARGET_DOOR_OPEN";
    root["snapshot"]["errors"].push_back(
      YAML::Load(
        "{sequence: 28, code: LEGACY_COMPLETION_ANOMALY, "
        "detail: completion carried an unresolved error, "
        "timestamp: 2000-01-01T00:00:00Z}"));
    write_yaml_file(journal, root);

    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(journal, port);
    const auto retained = module.snapshot(transaction_id);
    ASSERT_TRUE(retained.accepted())
      << retained.code << ": " << retained.detail;
    EXPECT_EQ(require_snapshot(retained).state, "LOCKED")
      << "schema=" << schema_version;
    EXPECT_EQ(
      require_snapshot(retained).cleanup_disposition,
      ElevatorCleanupDisposition::kRetainLock)
      << "schema=" << schema_version;
    EXPECT_FALSE(require_snapshot(retained).runtime_resources_reconciled)
      << "schema=" << schema_version;
  }
}

TEST(ElevatorExecutionModule, FieldObservedV1PreflightOrphanRemainsRecoverable)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v1-field-shape";
  create_deployed_v2_preflight_orphan(
    temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  root.remove("snapshot_schema_version");
  root["snapshot"].remove("safety_hold_state_known");
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_FALSE(locked.runtime_applied);
  EXPECT_EQ(locked.failure_origin_state, "RUNTIME_PREPARING");
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto recovered = wait_for_state(module, transaction_id, "FAILED");
  EXPECT_EQ(recovered.phase, "RECOVERY_COMPLETE");
  EXPECT_FALSE(recovered.safety_hold_active);
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
}

TEST(ElevatorExecutionModule, V1OutsideExactPreflightWhitelistFailsClosed)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v1-nonwhitelist";
  create_deployed_v2_preflight_orphan(
    temporary.journal(), transaction_id);
  auto root = YAML::LoadFile(temporary.journal().string());
  root.remove("snapshot_schema_version");
  root["snapshot"].remove("safety_hold_state_known");
  root["snapshot"]["effect_sequence"] = 3U;
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_EQ(locked.failure_origin_state, "LEGACY_UNKNOWN");
  EXPECT_EQ(
    locked.cleanup_disposition,
    ElevatorCleanupDisposition::kRetainLock);
  const auto rejected = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  EXPECT_EQ(rejected.kind, ElevatorExecutionReplyKind::kConflict);
}

TEST(ElevatorExecutionModule, V2CleanCompleteMigratesResourceAbsenceProof)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v2-clean-complete";
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(temporary.journal(), port);
    const auto started = module.start(start_command(transaction_id));
    const auto completed_reply =
      confirm_through_target_door(module, started);
    ASSERT_TRUE(completed_reply.accepted());
    const auto completed = wait_for_state(module, transaction_id, "COMPLETE");
    ASSERT_FALSE(completed.safety_hold_active);
  }
  downgrade_journal_to_v2(temporary.journal(), true);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto migrated = module.snapshot(transaction_id);
  ASSERT_TRUE(migrated.accepted())
    << migrated.code << ": " << migrated.detail;
  EXPECT_EQ(require_snapshot(migrated).state, "COMPLETE");
  EXPECT_TRUE(require_snapshot(migrated).runtime_resources_reconciled);
  EXPECT_EQ(
    require_snapshot(migrated).cleanup_disposition,
    ElevatorCleanupDisposition::kTargetOutside);
  const auto replacement =
    module.start(start_command("elevator-test-after-v2-complete"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

TEST(ElevatorExecutionModule, V1CleanCompleteMigratesResourceAbsenceProof)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-v1-clean-complete";
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule module(temporary.journal(), port);
    const auto started = module.start(start_command(transaction_id));
    const auto completed_reply =
      confirm_through_target_door(module, started);
    ASSERT_TRUE(completed_reply.accepted());
    (void)wait_for_state(module, transaction_id, "COMPLETE");
  }
  downgrade_journal_to_v2(temporary.journal(), true);
  auto root = YAML::LoadFile(temporary.journal().string());
  root.remove("snapshot_schema_version");
  root["snapshot"].remove("safety_hold_state_known");
  write_yaml_file(temporary.journal(), root);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto migrated = module.snapshot(transaction_id);
  ASSERT_TRUE(migrated.accepted())
    << migrated.code << ": " << migrated.detail;
  EXPECT_EQ(require_snapshot(migrated).state, "COMPLETE");
  EXPECT_TRUE(require_snapshot(migrated).runtime_resources_reconciled);
  EXPECT_EQ(
    require_snapshot(migrated).cleanup_disposition,
    ElevatorCleanupDisposition::kTargetOutside);
  const auto replacement =
    module.start(start_command("elevator-test-after-v1-complete"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
}

#ifndef _WIN32
TEST(ElevatorExecutionModule, SymlinkedAncestorCannotRedirectJournal)
{
  TemporaryDirectory temporary;
  const auto real_root = temporary.journal().parent_path() / "real-root";
  const auto linked_root = temporary.journal().parent_path() / "linked-root";
  ASSERT_TRUE(fs::create_directories(real_root / "journal"));
  ASSERT_EQ(::symlink(real_root.c_str(), linked_root.c_str()), 0);

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(
    linked_root / "journal" / "active.yaml", port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
  EXPECT_TRUE(port->applied_effects().empty());
}

TEST(ElevatorExecutionModule, ParentSymlinkInsertedAfterLoadBlocksFirstWrite)
{
  TemporaryDirectory temporary;
  const auto journal =
    temporary.journal().parent_path() / "late-parent" / "active.yaml";
  const auto redirected =
    temporary.journal().parent_path() / "redirected-parent";
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(journal, port);
  ASSERT_TRUE(fs::create_directories(redirected));
  ASSERT_EQ(::symlink(redirected.c_str(), journal.parent_path().c_str()), 0);

  const auto reply =
    module.start(start_command("elevator-test-late-parent-symlink"));

  EXPECT_EQ(reply.kind, ElevatorExecutionReplyKind::kStorageFailure);
  EXPECT_TRUE(port->applied_effects().empty());
  EXPECT_FALSE(fs::exists(redirected / "active.yaml"));
}

TEST(ElevatorExecutionModule, FifoJournalFailsQuicklyWithoutOpeningReader)
{
  TemporaryDirectory temporary;
  ASSERT_EQ(::mkfifo(temporary.journal().c_str(), 0600), 0);

  ASSERT_EXIT(
    {
      ::alarm(2);
      auto port = std::make_shared<InMemoryElevatorRuntimePort>();
      ElevatorExecutionModule module(temporary.journal(), port);
      const auto reply =
        module.start(start_command("elevator-test-after-fifo"));
      const bool rejected =
        module.journal_recovery_required() &&
        reply.kind == ElevatorExecutionReplyKind::kStorageFailure &&
        reply.detail.find("regular file") != std::string::npos;
      ::_exit(rejected ? 0 : 1);
    },
    ::testing::ExitedWithCode(0), "");
}
#endif

TEST(ElevatorExecutionModule, OversizedJournalFailsBeforeYamlParsing)
{
  TemporaryDirectory temporary;
  {
    std::ofstream output(
      temporary.journal(), std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(output.good());
    const std::string block(4096U, 'x');
    for (std::size_t index = 0U; index < 1025U; ++index) {
      output.write(block.data(), static_cast<std::streamsize>(block.size()));
    }
    output.close();
    ASSERT_TRUE(output.good());
  }

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto reply =
    module.start(start_command("elevator-test-after-oversized-journal"));

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_EQ(reply.kind, ElevatorExecutionReplyKind::kStorageFailure);
  EXPECT_NE(reply.detail.find("size limit"), std::string::npos);
}

TEST(ElevatorExecutionModule, UnreadableJournalRequiresGlobalRecoveryInterlock)
{
  TemporaryDirectory temporary;
  {
    std::ofstream output(temporary.journal(), std::ios::binary);
    output << "schema_version: [not valid for this reader\n";
  }

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
  const auto start =
    module.start(start_command("elevator-test-corrupt-journal"));
  EXPECT_EQ(start.kind, ElevatorExecutionReplyKind::kStorageFailure);
}

TEST(ElevatorExecutionModule, JournalLookupErrorRequiresGlobalRecoveryInterlock)
{
  TemporaryDirectory temporary;
  const auto non_directory =
    temporary.journal().parent_path() / "not-a-directory";
  {
    std::ofstream output(non_directory, std::ios::binary);
    output << "regular file blocks journal traversal\n";
    ASSERT_TRUE(output.good());
  }

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(non_directory / "active.yaml", port);

  EXPECT_TRUE(module.journal_recovery_required());
  EXPECT_FALSE(module.current_snapshot().has_value());
  const auto start =
    module.start(start_command("elevator-test-journal-lookup-error"));
  EXPECT_EQ(start.kind, ElevatorExecutionReplyKind::kStorageFailure);
  EXPECT_EQ(start.code, "ELEVATOR_EXECUTION_JOURNAL_UNAVAILABLE");
}

TEST(ElevatorExecutionModule, CreatesNestedJournalDirectoryBeforeRuntimeEffects)
{
  TemporaryDirectory temporary;
  const auto nested_journal =
    temporary.journal().parent_path() / "new" / "nested" / "active.yaml";
  auto incapable = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{false, false, false});
  ElevatorExecutionModule module(nested_journal, incapable);

  const auto result =
    module.start(start_command("elevator-test-nested-journal"));

  ASSERT_TRUE(result.accepted()) << result.code << ": " << result.detail;
  EXPECT_TRUE(fs::is_regular_file(nested_journal));
  EXPECT_TRUE(incapable->applied_effects().empty());
}

TEST(ElevatorExecutionModule, RestartReconcilesSafeOutsideJournalAndReleases)
{
  TemporaryDirectory temporary;
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command());
    ASSERT_EQ(
      wait_for_state(
        first, require_snapshot(started).transaction_id,
        "PRESSING_CALL_BUTTON").state,
      "PRESSING_CALL_BUTTON");
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  const auto released = wait_for_state(
    recovered, "elevator-test-1", "FAILED");
  EXPECT_TRUE(released.terminal);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_TRUE(recovered_port->applied_effects().empty());
  EXPECT_EQ(
    recovered_port->cancellation_requests(),
    std::vector<std::string>({"elevator-test-1"}));

  const auto replacement = recovered.start(start_command("elevator-test-2"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
  EXPECT_EQ(require_snapshot(replacement).transaction_id, "elevator-test-2");
}

TEST(ElevatorExecutionModule, RestartInCabinRetainsHoldUntilPhysicalExitIsConfirmed)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cabin-retained";
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command(transaction_id));
    auto current = wait_for_state(
      first, transaction_id, "PRESSING_CALL_BUTTON");
    auto confirmed = first.confirm(confirmation(current, "F1"));
    current = wait_for_state(first, transaction_id, "WAITING_SOURCE_DOOR");
    confirmed = first.confirm(confirmation(current, "F1"));
    ASSERT_EQ(
      wait_for_state(first, transaction_id, "PRESSING_TARGET_BUTTON").physical_zone,
      ElevatorPhysicalZone::kCabin);
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  ElevatorExecutionSnapshot locked;
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto current = recovered.snapshot(transaction_id);
    if (current.snapshot) {
      locked = *current.snapshot;
      if (
        locked.state == "LOCKED" && locked.safety_hold_state_known &&
        locked.dual_odom_stop_proven &&
        locked.runtime_resources_reconciled)
      {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_TRUE(locked.terminal);
  EXPECT_EQ(locked.physical_zone, ElevatorPhysicalZone::kCabin);
  EXPECT_EQ(locked.interrupted_state, "PRESSING_TARGET_BUTTON");
  EXPECT_TRUE(locked.safety_hold_state_known);
  EXPECT_TRUE(locked.safety_hold_active);
  EXPECT_TRUE(locked.dual_odom_stop_proven);
  EXPECT_TRUE(locked.runtime_resources_reconciled);
  EXPECT_EQ(recovered_port->cancellation_requests().size(), 1U);
}

TEST(ElevatorExecutionModule, FieldConfirmedSourceOutsideCanReleaseCabinRestartLock)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cabin-field-recovery";
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command(transaction_id));
    auto current = wait_for_state(first, transaction_id, "PRESSING_CALL_BUTTON");
    auto confirmed = first.confirm(confirmation(current, "F1"));
    current = wait_for_state(first, transaction_id, "WAITING_SOURCE_DOOR");
    confirmed = first.confirm(confirmation(current, "F1"));
    ASSERT_EQ(
      wait_for_state(first, transaction_id, "PRESSING_TARGET_BUTTON").physical_zone,
      ElevatorPhysicalZone::kCabin);
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  ElevatorExecutionSnapshot locked;
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto current = recovered.snapshot(transaction_id);
    if (current.snapshot) {
      locked = *current.snapshot;
      if (
        locked.state == "LOCKED" && locked.safety_hold_state_known &&
        locked.safety_hold_active && locked.dual_odom_stop_proven)
      {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto actions = allowed_recovery_actions(locked);
  EXPECT_NE(
    std::find(
      actions.cbegin(), actions.cend(),
      "CONFIRM_SOURCE_OUTSIDE_AND_RELEASE"),
    actions.cend());

  const auto incomplete = recovered.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "missing_stationary_field_check",
      true,
      locked.source_floor_id,
      "CONFIRM_SOURCE_OUTSIDE_AND_RELEASE",
      "SOURCE_OUTSIDE",
      false,
      true,
    });
  EXPECT_EQ(incomplete.kind, ElevatorExecutionReplyKind::kInvalid);
  EXPECT_EQ(
    incomplete.code,
    "INVALID_ELEVATOR_EXECUTION_RECOVERY");

  const auto accepted = recovered.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_source_outside",
      true,
      locked.source_floor_id,
      "CONFIRM_SOURCE_OUTSIDE_AND_RELEASE",
      "SOURCE_OUTSIDE",
      true,
      true,
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto released = wait_for_state(recovered, transaction_id, "FAILED");

  EXPECT_EQ(released.phase, "RECOVERY_COMPLETE");
  EXPECT_EQ(released.failure_code, "ELEVATOR_EXECUTION_RECOVERED");
  EXPECT_EQ(released.physical_zone, ElevatorPhysicalZone::kSourceOutside);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_TRUE(released.runtime_resources_reconciled);
}

TEST(ElevatorExecutionModule, RestartRecoveryRetriesThenRemainsLockedWithoutInventingProof)
{
  TemporaryDirectory temporary;
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command());
    ASSERT_EQ(
      wait_for_state(
        first, require_snapshot(started).transaction_id,
        "PRESSING_CALL_BUTTON").state,
      "PRESSING_CALL_BUTTON");
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 3; ++attempt) {
    recovered_port->queue_recovery_result(
      {
        false,
        "ELEVATOR_RESTART_RECOVERY_TEST_FAILURE",
        "recovery evidence unavailable",
        false,
        false,
      });
  }
  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  ElevatorExecutionReply reply;
  for (int attempt = 0; attempt < 400; ++attempt) {
    reply = recovered.snapshot("elevator-test-1");
    if (
      reply.snapshot &&
      reply.snapshot->failure_code ==
      "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN")
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(reply.accepted());
  const auto & locked = require_snapshot(reply);
  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_TRUE(locked.terminal);
  EXPECT_FALSE(locked.safety_hold_state_known);
  EXPECT_TRUE(locked.safety_hold_active);
  EXPECT_FALSE(locked.dual_odom_stop_proven);
  EXPECT_EQ(
    locked.failure_code,
    "ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN");
  EXPECT_EQ(recovered_port->cancellation_requests().size(), 3U);
  const auto replacement = recovered.start(start_command("elevator-test-2"));
  EXPECT_EQ(replacement.kind, ElevatorExecutionReplyKind::kConflict);
}

TEST(ElevatorExecutionModule, RestartRecoveryOutlivesTransientStartupEndpointDelay)
{
  TemporaryDirectory temporary;
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command());
    ASSERT_EQ(
      wait_for_state(
        first, require_snapshot(started).transaction_id,
        "PRESSING_CALL_BUTTON").state,
      "PRESSING_CALL_BUTTON");
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 4; ++attempt) {
    auto startup_not_ready = ElevatorRuntimeResult{
      false,
      "ELEVATOR_NAV2_ACTION_UNAVAILABLE",
      "/navigate_to_pose",
      true,
      false,
    };
    recovered_port->queue_recovery_result(std::move(startup_not_ready));
  }

  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  const auto released = wait_for_state(recovered, "elevator-test-1", "FAILED");

  EXPECT_TRUE(released.terminal);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_EQ(recovered_port->cancellation_requests().size(), 5U);
}

TEST(
  ElevatorExecutionModule,
  RestartRecoveryOutlivesPendingCancelResponseDuringRuntimeStartup)
{
  TemporaryDirectory temporary;
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command());
    ASSERT_EQ(
      wait_for_state(
        first, require_snapshot(started).transaction_id,
        "PRESSING_CALL_BUTTON").state,
      "PRESSING_CALL_BUTTON");
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  for (int attempt = 0; attempt < 4; ++attempt) {
    recovered_port->queue_recovery_result(
      {
        false,
        attempt % 2 == 0 ?
        "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_PENDING" :
        "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_PENDING",
        "the single submitted cancel-all request is still awaiting its response",
        true,
        false,
      });
  }

  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  const auto released = wait_for_state(recovered, "elevator-test-1", "FAILED");

  EXPECT_TRUE(released.terminal);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_EQ(recovered_port->cancellation_requests().size(), 5U);
}

TEST(ElevatorExecutionModule, RestartRecoveryOutlivesRuntimeStartupConvergence)
{
  TemporaryDirectory temporary;
  {
    auto first_port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorExecutionModule first(temporary.journal(), first_port);
    const auto started = first.start(start_command());
    ASSERT_EQ(
      wait_for_state(
        first, require_snapshot(started).transaction_id,
        "PRESSING_CALL_BUTTON").state,
      "PRESSING_CALL_BUTTON");
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  recovered_port->queue_recovery_result(
    {false, "ELEVATOR_CLEANUP_RUNTIME_CONTEXT_UNPROVEN", "context not ready"});
  recovered_port->queue_recovery_result(
    {false, "ELEVATOR_CORRECTION_PAUSE_TIMEOUT", "pause service warming"});
  recovered_port->queue_recovery_result(
    {false, "ELEVATOR_OPERATING_MODE_TIMEOUT", "mode service warming"});
  recovered_port->queue_recovery_result(
    {false, "ELEVATOR_CLEANUP_RUNTIME_CONTEXT_UNPROVEN", "context converging"});

  ElevatorExecutionModule recovered(temporary.journal(), recovered_port);
  const auto released = wait_for_state(recovered, "elevator-test-1", "FAILED");

  EXPECT_TRUE(released.terminal);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_EQ(recovered_port->cancellation_requests().size(), 5U);
}

TEST(ElevatorExecutionModule, RestartStrictlyAcceptsMatchingCleanupPendingAudit)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cleanup-pending-match";
  const auto effect_sequence = create_source_cleanup_release_pending_journal(
    temporary, transaction_id, true);
  const auto pending_path = temporary.recovery_audit(
    transaction_id, effect_sequence, "cleanup_release_pending");
  const auto pending_before = read_text_file(pending_path);

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto released = wait_for_state(module, transaction_id, "FAILED");

  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 1U);
  EXPECT_TRUE(port->pending_seen_before_finalize());
  EXPECT_EQ(read_text_file(pending_path), pending_before);
}

TEST(ElevatorExecutionModule, RestartAcceptsSemanticallyEquivalentCleanupCompleteAudit)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cleanup-complete-replay";
  const auto effect_sequence = create_source_cleanup_release_pending_journal(
    temporary, transaction_id, true, true);
  const auto complete_path = temporary.recovery_audit(
    transaction_id, effect_sequence, "cleanup_complete");
  auto complete_root = YAML::LoadFile(complete_path.string());
  complete_root["recorded_at"] = "2000-01-01T00:00:00Z";
  complete_root["snapshot"]["updated_at"] = "2000-01-01T00:00:00Z";
  complete_root["snapshot"]["detail"] =
    "prior process released the owner hold before its journal commit";
  complete_root["snapshot"]["events"].push_back(
    YAML::Load(
      "{sequence: 2, code: PRIOR_PROCESS_COMPLETE, "
      "detail: prior completion audit, timestamp: 2000-01-01T00:00:00Z}"));
  write_yaml_file(complete_path, complete_root);
  const auto complete_before = read_text_file(complete_path);

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto released = wait_for_state(module, transaction_id, "FAILED");

  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 1U);
  EXPECT_TRUE(port->pending_seen_before_finalize());
  EXPECT_TRUE(port->complete_seen_before_finalize());
  EXPECT_EQ(read_text_file(complete_path), complete_before);
}

TEST(ElevatorExecutionModule, RestartCreatesMissingCleanupPendingAuditBeforeRelease)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cleanup-pending-missing";
  const auto effect_sequence = create_source_cleanup_release_pending_journal(
    temporary, transaction_id, false);
  const auto pending_path = temporary.recovery_audit(
    transaction_id, effect_sequence, "cleanup_release_pending");
  ASSERT_FALSE(fs::exists(pending_path));

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto released = wait_for_state(module, transaction_id, "FAILED");

  EXPECT_TRUE(released.runtime_resources_reconciled);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 1U);
  EXPECT_TRUE(port->pending_seen_before_finalize());
  EXPECT_TRUE(fs::is_regular_file(pending_path));
}

TEST(ElevatorExecutionModule, RestartRejectsConflictingCleanupPendingAuditBeforeRelease)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-cleanup-pending-conflict";
  const auto effect_sequence = create_source_cleanup_release_pending_journal(
    temporary, transaction_id, false);
  const auto pending_path = temporary.recovery_audit(
    transaction_id, effect_sequence, "cleanup_release_pending");
  ASSERT_TRUE(fs::create_directories(temporary.history_directory()));
  {
    std::ofstream collision(pending_path, std::ios::binary | std::ios::trunc);
    collision << "immutable-sentinel\n";
    ASSERT_TRUE(collision.good());
  }

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_failure_code(
    module, transaction_id, "ELEVATOR_EXECUTION_JOURNAL_WRITE_FAILED");

  EXPECT_EQ(locked.state, "LOCKED");
  EXPECT_TRUE(locked.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 0U);
  EXPECT_EQ(read_text_file(pending_path), "immutable-sentinel\n");
}

TEST(ElevatorExecutionModule, ExplicitRecoveryReleasesDeployedV2PreflightOrphanLock)
{
  TemporaryDirectory temporary;
  create_deployed_v2_preflight_orphan(
    temporary.journal(), "elevator-test-preflight-orphan");

  auto recovery_port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule recovered(temporary.journal(), recovery_port);
  ElevatorExecutionSnapshot locked;
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto state =
      recovered.snapshot("elevator-test-preflight-orphan");
    if (
      state.snapshot && state.snapshot->safety_hold_active &&
      state.snapshot->dual_odom_stop_proven)
    {
      locked = *state.snapshot;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(locked.state, "LOCKED");

  const auto wrong_operator = recovered.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      "another_operator",
      "现场确认车辆静止且周边安全",
    });
  EXPECT_EQ(wrong_operator.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(
    wrong_operator.code,
    "ELEVATOR_EXECUTION_RECOVERY_OPERATOR_MISMATCH");

  const auto accepted = recovered.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "现场确认车辆静止且周边安全",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto released = wait_for_state(
    recovered, locked.transaction_id, "FAILED");

  EXPECT_EQ(released.phase, "RECOVERY_COMPLETE");
  EXPECT_TRUE(released.terminal);
  EXPECT_FALSE(released.runtime_applied);
  EXPECT_FALSE(released.motion_authorized);
  EXPECT_TRUE(released.safety_hold_state_known);
  EXPECT_FALSE(released.safety_hold_active);
  EXPECT_TRUE(released.dual_odom_stop_proven);
  EXPECT_EQ(
    released.failure_code,
    "ELEVATOR_EXECUTION_RECOVERED");

  const auto replacement =
    recovered.start(start_command("elevator-test-after-recovery"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
  EXPECT_NE(require_snapshot(replacement).state, "LOCKED");
}

TEST(
  ElevatorExecutionModule,
  ExplicitRecoveryPersistsImmutableAuditBeforeAndAfterPhysicalRelease)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-recovery-audit";
  create_deployed_v2_preflight_orphan(temporary.journal(), transaction_id);

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);
  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto recovery_sequence = require_snapshot(accepted).effect_sequence;
  const auto released = wait_for_state(module, transaction_id, "FAILED");

  const auto pending_path = temporary.recovery_audit(
    transaction_id, recovery_sequence, "recovery_release_pending");
  const auto complete_path = temporary.recovery_audit(
    transaction_id, recovery_sequence, "recovery_complete");
  EXPECT_EQ(released.phase, "RECOVERY_COMPLETE");
  EXPECT_EQ(port->finalize_calls(), 1U);
  EXPECT_TRUE(port->pending_seen_before_finalize());
  EXPECT_FALSE(port->complete_seen_before_finalize());
  ASSERT_TRUE(fs::is_regular_file(pending_path));
  ASSERT_TRUE(fs::is_regular_file(complete_path));
  const auto pending = read_text_file(pending_path);
  const auto complete = read_text_file(complete_path);
  EXPECT_NE(
    pending.find("checkpoint: recovery_release_pending"),
    std::string::npos);
  EXPECT_NE(pending.find("phase: RECOVERY_RELEASE_PENDING"), std::string::npos);
  EXPECT_NE(
    pending.find("code: EXPLICIT_RECOVERY_VERIFIED"),
    std::string::npos);
  EXPECT_NE(complete.find("checkpoint: recovery_complete"), std::string::npos);
  EXPECT_NE(complete.find("phase: RECOVERY_COMPLETE"), std::string::npos);
  EXPECT_NE(
    complete.find("failure_code: ELEVATOR_EXECUTION_RECOVERED"),
    std::string::npos);
  EXPECT_NE(
    complete.find("code: EXPLICIT_RECOVERY_COMPLETE"),
    std::string::npos);
}

TEST(
  ElevatorExecutionModule,
  RestartFromLegacyRecoveryReleasePendingRebindsAndFinalizes)
{
  TemporaryDirectory interrupted_storage;
  TemporaryDirectory restarted_storage;
  const std::string transaction_id =
    "elevator-test-legacy-recovery-release-pending-restart";
  create_deployed_v2_preflight_orphan(
    interrupted_storage.journal(), transaction_id);
  {
    // Match the field-observed v1 orphan: no snapshot schema discriminator
    // and no authoritative hold-known bit.
    auto root = YAML::LoadFile(interrupted_storage.journal().string());
    root.remove("snapshot_schema_version");
    root["snapshot"].remove("safety_hold_state_known");
    write_yaml_file(interrupted_storage.journal(), root);
  }

  std::uint64_t recovery_sequence = 0U;
  auto interrupted_port = std::make_shared<BlockingRecoveryFinalizePort>();
  {
    ElevatorExecutionModule interrupted(
      interrupted_storage.journal(), interrupted_port);
    const auto locked =
      wait_for_recovery_ready(interrupted, transaction_id);
    const auto accepted = interrupted.recover(
      ElevatorExecutionRecovery{
        locked.transaction_id,
        locked.state,
        locked.effect_sequence,
        locked.operator_id,
        "field_verified_stationary",
      });
    ASSERT_TRUE(accepted.accepted())
      << accepted.code << ": " << accepted.detail;
    recovery_sequence = require_snapshot(accepted).effect_sequence;
    const bool finalize_entered =
      interrupted_port->wait_until_finalize_entered();
    EXPECT_TRUE(finalize_entered);

    const auto persisted =
      YAML::LoadFile(interrupted_storage.journal().string());
    const bool snapshot_present = static_cast<bool>(persisted["snapshot"]);
    EXPECT_TRUE(snapshot_present);
    if (snapshot_present) {
      EXPECT_EQ(
        persisted["snapshot"]["state"].as<std::string>(),
        "FAILURE_CLEANUP");
      EXPECT_EQ(
        persisted["snapshot"]["phase"].as<std::string>(),
        "RECOVERY_RELEASE_PENDING");
      EXPECT_FALSE(persisted["snapshot"]["terminal"].as<bool>());
      EXPECT_EQ(persisted["snapshot_schema_version"].as<unsigned int>(), 3U);
    }

    fs::create_directories(restarted_storage.journal().parent_path());
    fs::copy_file(
      interrupted_storage.journal(),
      restarted_storage.journal(),
      fs::copy_options::overwrite_existing);
    fs::create_directories(restarted_storage.history_directory());
    const auto pending_path = interrupted_storage.recovery_audit(
      transaction_id, recovery_sequence, "recovery_release_pending");
    const bool pending_exists = fs::is_regular_file(pending_path);
    EXPECT_TRUE(pending_exists);
    if (pending_exists) {
      fs::copy_file(
        pending_path,
        restarted_storage.recovery_audit(
          transaction_id, recovery_sequence, "recovery_release_pending"),
        fs::copy_options::overwrite_existing);
    }

    interrupted_port->release_finalize();
    (void)wait_for_state(interrupted, transaction_id, "FAILED");
  }

  auto restarted_port = std::make_shared<RecoveryAuditObservingPort>(
    restarted_storage.history_directory());
  ElevatorExecutionModule restarted(
    restarted_storage.journal(), restarted_port);
  const auto recovered =
    wait_for_state(restarted, transaction_id, "FAILED");

  EXPECT_EQ(recovered.phase, "RECOVERY_COMPLETE");
  EXPECT_TRUE(recovered.terminal);
  EXPECT_EQ(recovered.effect_sequence, recovery_sequence);
  EXPECT_EQ(recovered.failure_code, "ELEVATOR_EXECUTION_RECOVERED");
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
  EXPECT_TRUE(recovered.safety_hold_state_known);
  EXPECT_FALSE(recovered.safety_hold_active);
  EXPECT_TRUE(recovered.dual_odom_stop_proven);
  EXPECT_EQ(restarted_port->finalize_calls(), 1U);
  EXPECT_TRUE(restarted_port->pending_seen_before_finalize());
  EXPECT_TRUE(fs::is_regular_file(restarted_storage.recovery_audit(
      transaction_id, recovery_sequence, "recovery_complete")));
}

TEST(
  ElevatorExecutionModule,
  V2JournalMissingRuntimeAppliedRequiresOnSiteService)
{
  TemporaryDirectory temporary;
  const std::string transaction_id =
    "elevator-test-legacy-runtime-applied-unknown";
  create_deployed_v2_preflight_orphan(temporary.journal(), transaction_id);
  {
    auto root = YAML::LoadFile(temporary.journal().string());
    root["snapshot"].remove("runtime_applied");
    YAML::Emitter emitter;
    emitter << root;
    ASSERT_TRUE(emitter.good()) << emitter.GetLastError();
    std::ofstream output(
      temporary.journal(), std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(output.good());
    output << emitter.c_str() << '\n';
    output.close();
    ASSERT_TRUE(output.good());
  }

  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);

  EXPECT_TRUE(locked.runtime_applied);
  const auto recovery = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  EXPECT_EQ(recovery.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(
    recovery.code,
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRES_SERVICE");
}

TEST(
  ElevatorExecutionModule,
  RecoveryReleasePendingAuditFailurePreventsPhysicalRelease)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-pending-audit-failure";
  create_deployed_v2_preflight_orphan(temporary.journal(), transaction_id);
  {
    std::ofstream blocker(
      temporary.history_directory(), std::ios::binary | std::ios::trunc);
    blocker << "not-a-directory\n";
    ASSERT_TRUE(blocker.good());
  }

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);
  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto retained = wait_for_failure_code(
    module, transaction_id, "ELEVATOR_EXECUTION_JOURNAL_WRITE_FAILED");

  EXPECT_EQ(retained.state, "LOCKED");
  EXPECT_EQ(retained.phase, "LOCKED");
  EXPECT_TRUE(retained.terminal);
  EXPECT_TRUE(retained.safety_hold_state_known);
  EXPECT_TRUE(retained.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 0U);
  EXPECT_NE(
    retained.detail.find("recovery_release_pending"),
    std::string::npos);
  EXPECT_EQ(read_text_file(temporary.history_directory()), "not-a-directory\n");
}

TEST(
  ElevatorExecutionModule,
  RestartAfterReleasePendingAuditPreservesHistoryAndRetriesSafely)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-audit-crash-retry";
  create_deployed_v2_preflight_orphan(temporary.journal(), transaction_id);

  std::uint64_t interrupted_sequence = 0U;
  fs::path interrupted_pending_path;
  std::string interrupted_pending;
  {
    auto interrupted_port = std::make_shared<RecoveryAuditObservingPort>(
      temporary.history_directory(), false, true);
    ElevatorExecutionModule interrupted(temporary.journal(), interrupted_port);
    const auto locked =
      wait_for_recovery_ready(interrupted, transaction_id);
    const auto accepted = interrupted.recover(
      ElevatorExecutionRecovery{
        locked.transaction_id,
        locked.state,
        locked.effect_sequence,
        locked.operator_id,
        "field_verified_stationary",
      });
    ASSERT_TRUE(accepted.accepted());
    interrupted_sequence = require_snapshot(accepted).effect_sequence;
    const auto retained = wait_for_failure_code(
      interrupted, transaction_id,
      "ELEVATOR_EXECUTION_RECOVERY_RELEASE_UNPROVEN");
    EXPECT_EQ(retained.state, "LOCKED");
    interrupted_pending_path = temporary.recovery_audit(
      transaction_id, interrupted_sequence, "recovery_release_pending");
    ASSERT_TRUE(fs::is_regular_file(interrupted_pending_path));
    interrupted_pending = read_text_file(interrupted_pending_path);
    EXPECT_EQ(interrupted_port->finalize_calls(), 1U);
  }

  auto retry_port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule retried(temporary.journal(), retry_port);
  const auto relocked = wait_for_recovery_ready(retried, transaction_id);
  const auto retry = retried.recover(
    ElevatorExecutionRecovery{
      relocked.transaction_id,
      relocked.state,
      relocked.effect_sequence,
      relocked.operator_id,
      "field_verified_stationary_after_restart",
    });
  ASSERT_TRUE(retry.accepted()) << retry.code << ": " << retry.detail;
  const auto retry_sequence = require_snapshot(retry).effect_sequence;
  ASSERT_GT(retry_sequence, interrupted_sequence);
  const auto released = wait_for_state(retried, transaction_id, "FAILED");

  EXPECT_EQ(released.phase, "RECOVERY_COMPLETE");
  EXPECT_EQ(read_text_file(interrupted_pending_path), interrupted_pending);
  EXPECT_TRUE(fs::is_regular_file(temporary.recovery_audit(
      transaction_id, retry_sequence, "recovery_release_pending")));
  EXPECT_TRUE(fs::is_regular_file(temporary.recovery_audit(
      transaction_id, retry_sequence, "recovery_complete")));
}

TEST(
  ElevatorExecutionModule,
  RecoveryCompleteAuditCollisionStaysLockedAndDoesNotOverwriteHistory)
{
  TemporaryDirectory temporary;
  const std::string transaction_id = "elevator-test-complete-audit-failure";
  create_deployed_v2_preflight_orphan(temporary.journal(), transaction_id);

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory(), true);
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);
  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  const auto recovery_sequence = require_snapshot(accepted).effect_sequence;
  const auto retained = wait_for_failure_code(
    module, transaction_id, "ELEVATOR_EXECUTION_JOURNAL_WRITE_FAILED");

  EXPECT_EQ(retained.state, "LOCKED");
  EXPECT_EQ(retained.phase, "LOCKED");
  EXPECT_TRUE(retained.terminal);
  EXPECT_TRUE(retained.safety_hold_state_known);
  EXPECT_FALSE(retained.safety_hold_active);
  EXPECT_EQ(port->finalize_calls(), 1U);
  EXPECT_TRUE(port->pending_seen_before_finalize());
  EXPECT_FALSE(port->complete_seen_before_finalize());
  EXPECT_TRUE(fs::is_regular_file(temporary.recovery_audit(
      transaction_id, recovery_sequence, "recovery_release_pending")));
  const auto collision_path = port->collision_path();
  ASSERT_TRUE(collision_path.has_value());
  ASSERT_TRUE(fs::is_regular_file(*collision_path));
  EXPECT_EQ(read_text_file(*collision_path), "immutable-sentinel\n");
  EXPECT_NE(
    retained.detail.find("recovery_complete"),
    std::string::npos);
}

TEST(ElevatorExecutionModule, ExplicitRecoveryReleaseFailureRemainsLocked)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_prepare_result(
    {
      false,
      "ELEVATOR_SOURCE_RUNTIME_BUSY",
      "navigation_idle=false;mapping_idle=true;docking_idle=true",
      false,
      false,
      ElevatorRuntimeFailureDisposition::kRecoveryRequiredBeforeEffects,
    });
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "ELEVATOR_RUNTIME_NOT_PREPARED",
        "no frozen elevator release is bound",
      });
  }
  port->queue_recovery_finalize_result(
    {
      false,
      "ELEVATOR_RECOVERY_HOLD_RELEASE_UNPROVEN",
      "hold release response was unknown",
      true,
      true,
    });
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started =
    module.start(start_command("elevator-test-release-unproven"));
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "现场确认车辆静止且周边安全",
    });
  ASSERT_TRUE(accepted.accepted());
  ElevatorExecutionSnapshot retained;
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto current = module.snapshot(locked.transaction_id);
    if (
      current.snapshot &&
      current.snapshot->failure_code ==
      "ELEVATOR_EXECUTION_RECOVERY_RELEASE_UNPROVEN")
    {
      retained = *current.snapshot;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(retained.state, "LOCKED");
  EXPECT_EQ(retained.phase, "LOCKED");
  EXPECT_TRUE(retained.terminal);
  EXPECT_TRUE(retained.safety_hold_state_known);
  EXPECT_TRUE(retained.safety_hold_active);
  EXPECT_TRUE(retained.dual_odom_stop_proven);
  const auto replacement =
    module.start(start_command("elevator-test-still-blocked"));
  EXPECT_EQ(replacement.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(replacement.code, "ELEVATOR_EXECUTION_RECOVERY_REQUIRED");
}

TEST(ElevatorExecutionModule, ExplicitRecoveryReleaseTimeoutMakesHoldStateUnknown)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_prepare_result(
    {
      false,
      "ELEVATOR_SAFETY_INTERLOCK_STATE_STALE",
      "pre-existing runtime state is unknown",
      false,
      false,
      ElevatorRuntimeFailureDisposition::kRecoveryRequiredBeforeEffects,
    });
  port->queue_recovery_finalize_result(
    {
      false,
      "ELEVATOR_RECOVERY_HOLD_RELEASE_TIMEOUT",
      "motion hold service did not respond",
      false,
      true,
    });
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started =
    module.start(start_command("elevator-test-release-timeout"));
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");

  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_check_complete",
    });
  ASSERT_TRUE(accepted.accepted());
  ElevatorExecutionSnapshot retained;
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto current = module.snapshot(locked.transaction_id);
    if (
      current.snapshot &&
      current.snapshot->failure_code ==
      "ELEVATOR_EXECUTION_RECOVERY_RELEASE_UNPROVEN")
    {
      retained = *current.snapshot;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(retained.state, "LOCKED");
  EXPECT_TRUE(retained.terminal);
  EXPECT_FALSE(retained.safety_hold_state_known);
  // Preserve the last authoritative value from recovery verification, but do
  // not present it as current proof after an ambiguous release timeout.
  EXPECT_TRUE(retained.safety_hold_active);
  EXPECT_TRUE(retained.dual_odom_stop_proven);
}

TEST(ElevatorExecutionModule, ExplicitRecoveryRetriesSafeOutsideRuntimeAppliedLock)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "SAFETY_CLEANUP_UNPROVEN",
        "hold or dual odometry proof missing",
      });
  }
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto started =
    module.start(start_command("elevator-test-runtime-applied"));
  const auto locked = wait_for_state(
    module, require_snapshot(started).transaction_id, "LOCKED");
  ASSERT_TRUE(locked.runtime_applied);

  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "现场确认车辆静止且周边安全",
    });

  ASSERT_TRUE(accepted.accepted())
    << accepted.code << ": " << accepted.detail;
  const auto recovered = wait_for_state(
    module, locked.transaction_id, "FAILED");
  EXPECT_TRUE(recovered.runtime_applied);
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
  EXPECT_TRUE(recovered.safety_hold_state_known);
  EXPECT_FALSE(recovered.safety_hold_active);
  EXPECT_EQ(
    recovered.cleanup_disposition,
    ElevatorCleanupDisposition::kSourceOutside);
}

TEST(ElevatorExecutionModule, IncapableRuntimeCreatesTruthfulFailedTerminalWithoutEffects)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>(
    ElevatorRuntimeCapabilities{false, false, false});
  ElevatorExecutionModule module(temporary.journal(), port);

  const auto reply = module.start(start_command());

  ASSERT_TRUE(reply.accepted());
  const auto & failed = require_snapshot(reply);
  EXPECT_EQ(failed.state, "FAILED");
  EXPECT_TRUE(failed.terminal);
  EXPECT_EQ(
    failed.failure_code,
    "ELEVATOR_RUNTIME_ADAPTER_NOT_DEPLOYED");
  EXPECT_FALSE(failed.runtime_capable);
  EXPECT_FALSE(failed.runtime_applied);
  EXPECT_FALSE(failed.motion_authorized);
  EXPECT_FALSE(failed.floor_switch_capable);
  EXPECT_TRUE(failed.safety_hold_state_known);
  EXPECT_FALSE(failed.safety_hold_active);
  EXPECT_TRUE(port->applied_effects().empty());

  const auto replacement = module.start(start_command("elevator-test-2"));
  EXPECT_TRUE(replacement.accepted());
  EXPECT_EQ(require_snapshot(replacement).transaction_id, "elevator-test-2");
}

TEST(ElevatorExecutionModule, ActiveCannotBeReplacedButSafeCancellationCan)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto active = module.start(start_command("elevator-test-1"));
  const auto active_snapshot = wait_for_state(
    module, require_snapshot(active).transaction_id, "PRESSING_CALL_BUTTON");
  ASSERT_FALSE(active_snapshot.terminal);

  const auto conflict = module.start(start_command("elevator-test-2"));
  EXPECT_EQ(conflict.kind, ElevatorExecutionReplyKind::kConflict);

  const auto cancelled = module.cancel(
    {
      active_snapshot.transaction_id,
      active_snapshot.effect_sequence,
      "replace_after_cancel",
      active_snapshot.operator_id,
    });
  ASSERT_EQ(
    wait_for_state(
      module, require_snapshot(cancelled).transaction_id,
      "CANCELLED").state,
    "CANCELLED");

  const auto replacement = module.start(start_command("elevator-test-2"));
  EXPECT_TRUE(replacement.accepted())
    << replacement.code << ": " << replacement.detail;
  EXPECT_EQ(require_snapshot(replacement).transaction_id, "elevator-test-2");
}

TEST(
  ElevatorExecutionModule,
  StaleExplicitRecoverySequenceCannotFinalizeOrMutateAndFreshSequenceStillRecovers)
{
  TemporaryDirectory temporary;
  const std::string transaction_id =
    "elevator-test-stale-explicit-recovery";
  create_deployed_v2_preflight_orphan(
    temporary.journal(), transaction_id);

  auto port = std::make_shared<RecoveryAuditObservingPort>(
    temporary.history_directory());
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto locked = wait_for_recovery_ready(module, transaction_id);
  ASSERT_GT(locked.effect_sequence, 1U);
  const auto journal_before_stale = read_text_file(temporary.journal());
  const auto events_before_stale = locked.events;
  const auto errors_before_stale = locked.errors;

  const auto stale = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence - 1U,
      locked.operator_id,
      "field_verified_stationary",
    });

  EXPECT_EQ(stale.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(stale.code, "ELEVATOR_EXECUTION_RECOVERY_SNAPSHOT_MISMATCH");
  ASSERT_TRUE(stale.snapshot.has_value());
  EXPECT_EQ(stale.snapshot->effect_sequence, locked.effect_sequence);
  EXPECT_EQ(port->finalize_calls(), 0U);

  const auto after_stale_reply = module.snapshot(transaction_id);
  ASSERT_TRUE(after_stale_reply.snapshot.has_value());
  const auto & after_stale = *after_stale_reply.snapshot;
  EXPECT_EQ(after_stale.effect_sequence, locked.effect_sequence);
  ASSERT_EQ(after_stale.events.size(), events_before_stale.size());
  ASSERT_EQ(after_stale.errors.size(), errors_before_stale.size());
  for (std::size_t index = 0U; index < events_before_stale.size(); ++index) {
    EXPECT_EQ(
      after_stale.events[index].sequence,
      events_before_stale[index].sequence);
    EXPECT_EQ(after_stale.events[index].code, events_before_stale[index].code);
    EXPECT_EQ(
      after_stale.events[index].detail,
      events_before_stale[index].detail);
    EXPECT_EQ(
      after_stale.events[index].timestamp,
      events_before_stale[index].timestamp);
  }
  for (std::size_t index = 0U; index < errors_before_stale.size(); ++index) {
    EXPECT_EQ(
      after_stale.errors[index].sequence,
      errors_before_stale[index].sequence);
    EXPECT_EQ(after_stale.errors[index].code, errors_before_stale[index].code);
    EXPECT_EQ(
      after_stale.errors[index].detail,
      errors_before_stale[index].detail);
    EXPECT_EQ(
      after_stale.errors[index].timestamp,
      errors_before_stale[index].timestamp);
  }
  EXPECT_EQ(read_text_file(temporary.journal()), journal_before_stale);

  const auto accepted = module.recover(
    ElevatorExecutionRecovery{
      locked.transaction_id,
      locked.state,
      locked.effect_sequence,
      locked.operator_id,
      "field_verified_stationary",
    });
  ASSERT_TRUE(accepted.accepted()) << accepted.code << ": " << accepted.detail;
  EXPECT_EQ(
    require_snapshot(accepted).effect_sequence,
    locked.effect_sequence + 1U);

  const auto recovered = wait_for_state(module, transaction_id, "FAILED");
  EXPECT_EQ(recovered.phase, "RECOVERY_COMPLETE");
  EXPECT_EQ(
    recovered.failure_code,
    "ELEVATOR_EXECUTION_RECOVERED");
  EXPECT_FALSE(recovered.safety_hold_active);
  EXPECT_TRUE(recovered.runtime_resources_reconciled);
  EXPECT_EQ(port->finalize_calls(), 1U);
}

TEST(ElevatorExecutionModule, TransactionReplayRequiresTheEntireFrozenReleaseIdentity)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorExecutionModule module(temporary.journal(), port);
  const auto command = start_command("elevator-test-exact-release");
  const auto started = module.start(command);
  ASSERT_TRUE(started.accepted());

  auto drifted = command;
  drifted.release.target.map_asset_epoch += 1U;
  drifted.release.target.map_asset_digest =
    "sha256:3333333333333333333333333333333333333333333333333333333333333333";
  drifted.release.target.poses[2].x += 0.01;
  const auto replay = module.start(drifted);

  EXPECT_EQ(replay.kind, ElevatorExecutionReplyKind::kConflict);
  EXPECT_EQ(replay.code, "ELEVATOR_EXECUTION_TRANSACTION_ID_REUSED");
  ASSERT_TRUE(replay.snapshot.has_value());
  EXPECT_EQ(
    replay.snapshot->target_asset_epoch,
    command.release.target.map_asset_epoch);
}

}  // namespace
}  // namespace robot_elevator_manager
