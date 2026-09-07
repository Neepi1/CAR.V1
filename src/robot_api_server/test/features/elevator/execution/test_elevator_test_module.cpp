#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "robot_api_server/features/elevator/execution/elevator_test_module.hpp"

namespace
{

using robot_api_server::ElevatorTestCancelCommand;
using robot_api_server::ElevatorTestConfirmCommand;
using robot_api_server::ElevatorTestModule;
using robot_api_server::ElevatorTestRecoverCommand;
using robot_api_server::ElevatorTestReply;
using robot_api_server::ElevatorTestStartCommand;
using robot_api_server::ElevatorTestStateQuery;
using robot_elevator_manager::ElevatorReleaseLoadError;
using robot_elevator_manager::ElevatorReleaseLoadRequest;
using robot_elevator_manager::ElevatorReleaseLoadResult;
using robot_elevator_manager::FrozenElevatorRelease;
using robot_elevator_manager::InMemoryElevatorRuntimePort;
using robot_elevator_manager::PoseRole;

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path_ = fs::temp_directory_path() /
      ("njrh_elevator_test_adapter_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  const fs::path & path() const
  {
    return path_;
  }

private:
  fs::path path_;
};

ElevatorTestStartCommand valid_command()
{
  return ElevatorTestStartCommand{
    "B11",
    "elevator_1",
    "F1",
    "map_f1",
    "F2",
    "map_f2",
    "elevator-config-000001-abcdef123456",
    "commissioning_app",
  };
}

ElevatorReleaseLoadResult valid_release(const ElevatorReleaseLoadRequest & request)
{
  FrozenElevatorRelease release;
  release.release_id = request.expected_release_id;
  release.generation = 7U;
  release.configuration_digest = "sha256:configuration";
  release.building_id = request.building_id;
  release.elevator_id = request.preferred_elevator_id;
  release.source.floor_id = request.source_floor_id;
  release.source.map_id = request.source_map_id;
  release.source.map_asset_epoch = 11U;
  release.source.map_asset_digest =
    "sha256:1111111111111111111111111111111111111111111111111111111111111111";
  release.source.poses = {
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kHallCall, "f1_hall_call", 0.0, 0.0, 0.0},
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kLanding, "f1_landing", 1.0, 0.0, 0.0},
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kCabin, "f1_cabin", 2.0, 0.0, 0.0},
  };
  release.target.floor_id = request.target_floor_id;
  release.target.map_id = request.target_map_id;
  release.target.map_asset_epoch = 12U;
  release.target.map_asset_digest =
    "sha256:2222222222222222222222222222222222222222222222222222222222222222";
  release.target.poses = {
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kHallCall, "f2_hall_call", 0.0, 0.0, 0.0},
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kLanding, "f2_landing", 1.0, 0.0, 0.0},
    robot_elevator_manager::ElevatorRuntimePose{
      PoseRole::kCabin, "f2_cabin", 2.0, 0.0, 0.0},
  };
  return ElevatorReleaseLoadResult{
    ElevatorReleaseLoadError::kNone,
    "",
    release,
  };
}

std::string transaction_id_from(const std::string & body)
{
  const std::string prefix = "\"transaction_id\":\"";
  const auto begin = body.find(prefix);
  if (begin == std::string::npos) {
    return "";
  }
  const auto value_begin = begin + prefix.size();
  const auto end = body.find('"', value_begin);
  return end == std::string::npos ? "" : body.substr(value_begin, end - value_begin);
}

ElevatorTestReply wait_for_wire_state(
  ElevatorTestModule & module,
  const std::string & transaction_id,
  const std::string & state)
{
  const std::string needle = "\"state\":\"" + state + "\"";
  for (int attempt = 0; attempt < 400; ++attempt) {
    const auto reply = module.state(ElevatorTestStateQuery{transaction_id});
    if (reply.body.find(needle) != std::string::npos) {
      return reply;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return module.state(ElevatorTestStateQuery{transaction_id});
}

TEST(ElevatorTestModule, ValidReleaseBecomesExplicitFailedTransactionWithoutMotionAuthority)
{
  TemporaryDirectory temporary;
  ElevatorTestModule module(temporary.path(), valid_release);
  const auto started = module.start(valid_command());
  ASSERT_EQ(started.status, 200);
  const auto transaction_id = transaction_id_from(started.body);
  ASSERT_FALSE(transaction_id.empty());
  EXPECT_NE(started.body.find("\"ok\":true"), std::string::npos);
  EXPECT_NE(started.body.find("\"state\":\"FAILED\""), std::string::npos);
  EXPECT_NE(started.body.find("\"runtime_capable\":false"), std::string::npos);
  EXPECT_NE(started.body.find("\"runtime_applied\":false"), std::string::npos);
  EXPECT_NE(started.body.find("\"motion_authorized\":false"), std::string::npos);
  EXPECT_NE(started.body.find("\"floor_switch_capable\":false"), std::string::npos);
  EXPECT_NE(
    started.body.find("\"failure_code\":\"ELEVATOR_RUNTIME_ADAPTER_NOT_DEPLOYED\""),
    std::string::npos);

  const auto state = module.state(ElevatorTestStateQuery{transaction_id});
  EXPECT_EQ(state.status, 200);
  EXPECT_EQ(transaction_id_from(state.body), transaction_id);
  EXPECT_NE(state.body.find("\"awaiting_confirmation\":false"), std::string::npos);
  EXPECT_NE(
    state.body.find("\"safety_hold_state_known\":true"),
    std::string::npos);
  EXPECT_NE(state.body.find("\"safety_hold_active\":false"), std::string::npos);
  EXPECT_NE(
    state.body.find("\"dual_odom_stop_proven\":false"),
    std::string::npos);
  EXPECT_NE(state.body.find("\"recovery_required\":false"), std::string::npos);
}

TEST(ElevatorTestModule, TerminalFailureCanBeRetriedAsANewTransaction)
{
  TemporaryDirectory temporary;
  ElevatorTestModule module(temporary.path(), valid_release);
  const auto first = module.start(valid_command());
  const auto second = module.start(valid_command());
  ASSERT_EQ(first.status, 200);
  ASSERT_EQ(second.status, 200);
  EXPECT_NE(transaction_id_from(first.body), transaction_id_from(second.body));
  EXPECT_NE(second.body.find("\"replayed\":false"), std::string::npos);
}

TEST(ElevatorTestModule, CapableRuntimeIsRenderedFromTheExecutionSnapshot)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorTestModule module(temporary.path(), valid_release, port);
  const auto started = module.start(valid_command());
  ASSERT_EQ(started.status, 200);
  const auto transaction_id = transaction_id_from(started.body);
  ASSERT_FALSE(transaction_id.empty());

  const auto waiting =
    wait_for_wire_state(module, transaction_id, "PRESSING_CALL_BUTTON");

  EXPECT_NE(
    waiting.body.find("\"expected_confirmation\":\"CALL_BUTTON_PRESSED\""),
    std::string::npos);
  EXPECT_NE(waiting.body.find("\"runtime_capable\":true"), std::string::npos);
  EXPECT_NE(waiting.body.find("\"runtime_applied\":true"), std::string::npos);
  EXPECT_NE(
    waiting.body.find("\"runtime_resources_reconciled\":false"),
    std::string::npos);
  EXPECT_NE(
    waiting.body.find("\"cleanup_disposition\":\"SOURCE_OUTSIDE\""),
    std::string::npos);
  EXPECT_NE(waiting.body.find("\"motion_authorized\":true"), std::string::npos);
  EXPECT_NE(waiting.body.find("\"floor_switch_capable\":true"), std::string::npos);
  EXPECT_EQ(module.active_transaction(), transaction_id);

  const auto replayed = module.start(valid_command());
  EXPECT_EQ(replayed.status, 200);
  EXPECT_EQ(transaction_id_from(replayed.body), transaction_id);
  EXPECT_NE(replayed.body.find("\"replayed\":true"), std::string::npos);
}

TEST(ElevatorTestModule, RestartLockProjectsServerAuthorizedFieldRecovery)
{
  TemporaryDirectory temporary;
  std::string transaction_id;
  {
    auto port = std::make_shared<InMemoryElevatorRuntimePort>();
    ElevatorTestModule first(temporary.path(), valid_release, port);
    const auto started = first.start(valid_command());
    transaction_id = transaction_id_from(started.body);
    auto waiting = wait_for_wire_state(
      first, transaction_id, "PRESSING_CALL_BUTTON");
    auto snapshot = first.current_snapshot();
    ASSERT_TRUE(snapshot.has_value());
    auto accepted = first.confirm(
      ElevatorTestConfirmCommand{
        transaction_id,
        "PRESSING_CALL_BUTTON",
        snapshot->effect_sequence,
        "CALL_BUTTON_PRESSED",
        "F1",
        "commissioning_app",
      });
    ASSERT_EQ(accepted.status, 200);
    waiting = wait_for_wire_state(first, transaction_id, "WAITING_SOURCE_DOOR");
    snapshot = first.current_snapshot();
    ASSERT_TRUE(snapshot.has_value());
    accepted = first.confirm(
      ElevatorTestConfirmCommand{
        transaction_id,
        "WAITING_SOURCE_DOOR",
        snapshot->effect_sequence,
        "SOURCE_DOOR_OPEN",
        "F1",
        "commissioning_app",
      });
    ASSERT_EQ(accepted.status, 200);
    ASSERT_NE(
      wait_for_wire_state(first, transaction_id, "PRESSING_TARGET_BUTTON").body.find(
        "\"physical_zone\":\"CABIN\""),
      std::string::npos);
  }

  auto recovered_port = std::make_shared<InMemoryElevatorRuntimePort>();
  ElevatorTestModule recovered(
    temporary.path(), valid_release, recovered_port);
  ElevatorTestReply locked;
  const std::string action_needle =
    "\"allowed_recovery_actions\":[\"CONFIRM_SOURCE_OUTSIDE_AND_RELEASE\"]";
  for (int attempt = 0; attempt < 400; ++attempt) {
    locked = recovered.state(ElevatorTestStateQuery{transaction_id});
    if (locked.body.find(action_needle) != std::string::npos) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(locked.body.find(action_needle), std::string::npos);
  EXPECT_NE(locked.body.find("\"physical_zone\":\"CABIN\""), std::string::npos);
  EXPECT_NE(
    locked.body.find("\"interrupted_state\":\"PRESSING_TARGET_BUTTON\""),
    std::string::npos);
  EXPECT_NE(
    locked.body.find(
      "\"interrupted_expected_confirmation\":\"TARGET_BUTTON_PRESSED\""),
    std::string::npos);

  const auto snapshot = recovered.current_snapshot();
  ASSERT_TRUE(snapshot.has_value());
  ElevatorTestRecoverCommand command;
  command.transaction_id = transaction_id;
  command.expected_state = "LOCKED";
  command.effect_sequence = snapshot->effect_sequence;
  command.operator_id = "commissioning_app";
  command.reason = "field_verified_source_outside";
  command.source_outside_confirmed = true;
  command.confirmed_floor_id = "F1";
  command.action = "CONFIRM_SOURCE_OUTSIDE_AND_RELEASE";
  command.physical_zone = "SOURCE_OUTSIDE";
  command.stationary_confirmed = true;
  command.door_zone_clear_confirmed = true;
  const auto accepted = recovered.recover(command);
  ASSERT_EQ(accepted.status, 200) << accepted.body;
  const auto released = wait_for_wire_state(recovered, transaction_id, "FAILED");
  EXPECT_NE(
    released.body.find("\"failure_code\":\"ELEVATOR_EXECUTION_RECOVERED\""),
    std::string::npos);
}

TEST(ElevatorTestModule, UnknownTerminalHoldStateIsRenderedRecoveryRequired)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    robot_elevator_manager::ElevatorEffectKind::kNavigateToPose,
    {false, "NAVIGATION_FAILED", "hall goal aborted"});
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      robot_elevator_manager::ElevatorEffectKind::kHoldAndCancel,
      {true, "OK", "cleanup returned no hold-state proof", false, false});
  }
  ElevatorTestModule module(temporary.path(), valid_release, port);

  const auto started = module.start(valid_command());
  ASSERT_EQ(started.status, 200);
  const auto transaction_id = transaction_id_from(started.body);
  const auto locked = wait_for_wire_state(module, transaction_id, "LOCKED");

  ASSERT_NE(locked.body.find("\"state\":\"LOCKED\""), std::string::npos);
  EXPECT_NE(
    locked.body.find("\"safety_hold_state_known\":false"),
    std::string::npos);
  EXPECT_NE(
    locked.body.find("\"recovery_required\":true"),
    std::string::npos);
}

TEST(ElevatorTestModule, TerminalTransactionRejectsConfirmationAndCancelIsIdempotent)
{
  TemporaryDirectory temporary;
  ElevatorTestModule module(temporary.path(), valid_release);
  const auto started = module.start(valid_command());
  const auto transaction_id = transaction_id_from(started.body);

  ElevatorTestConfirmCommand confirmation;
  confirmation.transaction_id = transaction_id;
  confirmation.expected_state = "FAILED";
  confirmation.effect_sequence = 1U;
  confirmation.event = "CALL_BUTTON_PRESSED";
  confirmation.observed_floor_id = "F1";
  confirmation.operator_id = "commissioning_app";
  const auto confirmed = module.confirm(confirmation);
  EXPECT_EQ(confirmed.status, 409);
  EXPECT_EQ(confirmed.code, "ELEVATOR_TEST_TERMINAL_STATE");

  ElevatorTestCancelCommand cancellation;
  cancellation.transaction_id = transaction_id;
  cancellation.effect_sequence = 1U;
  cancellation.reason = "operator_cancelled";
  cancellation.operator_id = "commissioning_app";
  const auto cancelled = module.cancel(cancellation);
  EXPECT_EQ(cancelled.status, 200);
  EXPECT_NE(cancelled.body.find("\"state\":\"FAILED\""), std::string::npos);
}

TEST(ElevatorTestModule, UnknownTransactionNeverUsesNotFoundHttpStatus)
{
  TemporaryDirectory temporary;
  ElevatorTestModule module(temporary.path(), valid_release);
  const auto state = module.state(ElevatorTestStateQuery{"elevator-test-unknown"});
  EXPECT_EQ(state.status, 409);
  EXPECT_EQ(state.code, "ELEVATOR_TEST_TRANSACTION_NOT_FOUND");

  ElevatorTestCancelCommand cancellation;
  cancellation.transaction_id = "elevator-test-unknown";
  cancellation.effect_sequence = 1U;
  cancellation.reason = "operator_cancelled";
  cancellation.operator_id = "commissioning_app";
  const auto cancelled = module.cancel(cancellation);
  EXPECT_EQ(cancelled.status, 409);
  EXPECT_EQ(cancelled.code, "ELEVATOR_TEST_TRANSACTION_NOT_FOUND");
}

TEST(ElevatorTestModule, InvalidRequestDoesNotCallReleaseResolver)
{
  TemporaryDirectory temporary;
  bool called = false;
  ElevatorTestModule module(
    temporary.path(),
    [&called](const ElevatorReleaseLoadRequest &) {
      called = true;
      return ElevatorReleaseLoadResult{};
    });
  auto command = valid_command();
  command.expected_release_id.clear();
  const auto result = module.start(command);
  EXPECT_EQ(result.status, 400);
  EXPECT_FALSE(called);
}

TEST(ElevatorTestModule, ExplicitRecoveryIsProjectedToTheHttpWireContract)
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
      robot_elevator_manager::ElevatorRuntimeFailureDisposition::
      kRecoveryRequiredBeforeEffects,
    });
  for (int attempt = 0; attempt < 3; ++attempt) {
    port->queue_result(
      robot_elevator_manager::ElevatorEffectKind::kHoldAndCancel,
      {
        false,
        "ELEVATOR_RUNTIME_NOT_PREPARED",
        "no frozen elevator release is bound",
      });
  }
  ElevatorTestModule module(temporary.path(), valid_release, port);
  const auto started = module.start(valid_command());
  const auto transaction_id = transaction_id_from(started.body);
  const auto locked = wait_for_wire_state(module, transaction_id, "LOCKED");
  ASSERT_EQ(locked.status, 200);
  const auto snapshot = module.current_snapshot();
  ASSERT_TRUE(snapshot.has_value());

  ElevatorTestRecoverCommand recovery;
  recovery.transaction_id = transaction_id;
  recovery.expected_state = "LOCKED";
  recovery.effect_sequence = snapshot->effect_sequence;
  recovery.operator_id = "commissioning_app";
  recovery.reason = "现场确认车辆静止且周边安全";
  const auto accepted = module.recover(recovery);
  ASSERT_EQ(accepted.status, 200);
  EXPECT_EQ(accepted.code, "ELEVATOR_EXECUTION_RECOVERY_ACCEPTED");
  const auto recovered = wait_for_wire_state(module, transaction_id, "FAILED");

  EXPECT_NE(
    recovered.body.find("\"phase\":\"RECOVERY_COMPLETE\""),
    std::string::npos);
  EXPECT_NE(
    recovered.body.find("\"recovery_required\":false"),
    std::string::npos);
  EXPECT_NE(
    recovered.body.find("\"runtime_resources_reconciled\":true"),
    std::string::npos);
  EXPECT_NE(
    recovered.body.find("\"failure_code\":\"ELEVATOR_EXECUTION_RECOVERED\""),
    std::string::npos);
}

TEST(ElevatorTestModule, CabinEntryFailureIsProjectedAsRetryableFailure)
{
  TemporaryDirectory temporary;
  auto port = std::make_shared<InMemoryElevatorRuntimePort>();
  port->queue_result(
    robot_elevator_manager::ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "hall-call navigation completed", true, true});
  port->queue_result(
    robot_elevator_manager::ElevatorEffectKind::kNavigateToPose,
    {true, "OK", "source-landing navigation completed", true, true});
  port->queue_result(
    robot_elevator_manager::ElevatorEffectKind::kNavigateToPose,
    {
      false,
      "ELEVATOR_NAV2_GOAL_FAILED",
      "entry navigation failed with motion authorization unknown",
      true,
      true,
    });
  ElevatorTestModule module(temporary.path(), valid_release, port);

  const auto started = module.start(valid_command());
  const auto transaction_id = transaction_id_from(started.body);
  auto state = wait_for_wire_state(
    module, transaction_id, "PRESSING_CALL_BUTTON");
  ASSERT_EQ(state.status, 200);
  auto snapshot = module.current_snapshot();
  ASSERT_TRUE(snapshot.has_value());

  ElevatorTestConfirmCommand confirmation;
  confirmation.transaction_id = transaction_id;
  confirmation.expected_state = snapshot->state;
  confirmation.effect_sequence = snapshot->effect_sequence;
  confirmation.event = "CALL_BUTTON_PRESSED";
  confirmation.observed_floor_id = "F1";
  confirmation.operator_id = "commissioning_app";
  auto confirmed = module.confirm(confirmation);
  ASSERT_EQ(confirmed.status, 200) << confirmed.body;

  state = wait_for_wire_state(module, transaction_id, "WAITING_SOURCE_DOOR");
  snapshot = module.current_snapshot();
  ASSERT_TRUE(snapshot.has_value());
  confirmation.expected_state = snapshot->state;
  confirmation.effect_sequence = snapshot->effect_sequence;
  confirmation.event = "SOURCE_DOOR_OPEN";
  confirmed = module.confirm(confirmation);
  ASSERT_EQ(confirmed.status, 200) << confirmed.body;

  const auto failed = wait_for_wire_state(module, transaction_id, "FAILED");
  ASSERT_NE(
    failed.body.find("\"failure_origin_state\":\"ENTERING_CABIN\""),
    std::string::npos);
  ASSERT_NE(
    failed.body.find("\"cleanup_disposition\":\"SOURCE_OUTSIDE\""),
    std::string::npos);
  EXPECT_NE(
    failed.body.find("\"recovery_required\":false"),
    std::string::npos);
  EXPECT_NE(
    failed.body.find("\"runtime_resources_reconciled\":true"),
    std::string::npos);
  EXPECT_NE(
    failed.body.find("\"safety_hold_active\":false"),
    std::string::npos);

  const auto replacement = module.start(valid_command());
  EXPECT_EQ(replacement.status, 200) << replacement.body;
  EXPECT_NE(transaction_id_from(replacement.body), transaction_id);
}

}  // namespace
