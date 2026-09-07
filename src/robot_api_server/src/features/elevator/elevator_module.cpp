#include "robot_api_server/features/elevator/elevator_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "robot_api_server/features/elevator/configuration/elevator_configuration_module.hpp"
#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"
#include "robot_api_server/features/elevator/execution/elevator_ros_runtime_port.hpp"
#include "robot_api_server/features/elevator/execution/elevator_test_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"

namespace robot_api_server::features::elevator
{

namespace fs = std::filesystem;

namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0U) == 0U;
}

void require_ports(const ElevatorModulePorts & ports)
{
  if (!ports.floor_runtime_interlock_response ||
    !ports.runtime_idle_probe ||
    !ports.delayed_side_effect_unknown_probe ||
    !ports.current_map_pose_probe)
  {
    throw std::invalid_argument("elevator module requires every cross-domain port");
  }
}

std::string query_string_value(
  const HttpRequest & request,
  const std::string & key,
  const std::string & default_value = "")
{
  const auto it = request.query.find(key);
  return it == request.query.end() ? default_value : it->second;
}

HttpResponse configuration_response(const ElevatorConfigurationReply & reply)
{
  return {reply.status, "application/json", reply.body};
}

HttpResponse test_response(const ElevatorTestReply & reply)
{
  return {reply.status, "application/json", reply.body};
}

std::string motion_admission_failure_detail(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  if (admission.stale()) {
    return operation +
           " request predates the completed elevator admission barrier";
  }
  if (admission.interlock().delayed_side_effect_unknown()) {
    return operation + " is blocked because " +
           std::to_string(
      admission.interlock().delayed_side_effect_unknown_count) +
           " timed-out motion/runtime submission(s) have no proven outcome";
  }
  if (admission.interlock().recovery_required()) {
    return operation +
           " is blocked by retained elevator recovery state; transaction_id=" +
           admission.interlock().transaction_id;
  }
  return operation +
         " is blocked during elevator execution; transaction_id=" +
         admission.interlock().transaction_id;
}

HttpResponse make_motion_admission_failure_response(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  const std::string code =
    admission.stale() ?
    "ELEVATOR_MOTION_ADMISSION_STALE" :
    (admission.interlock().delayed_side_effect_unknown() ?
    "DELAYED_SIDE_EFFECT_UNKNOWN" :
    (admission.interlock().recovery_required() ?
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRED" :
    "ELEVATOR_EXECUTION_ACTIVE"));
  std::ostringstream body;
  body << "{\"ok\":false,\"code\":" << json_string(code)
       << ",\"transaction_id\":"
       << json_string(admission.interlock().transaction_id)
       << ",\"delayed_side_effect_unknown_count\":"
       << admission.interlock().delayed_side_effect_unknown_count
       << ",\"detail\":"
       << json_string(motion_admission_failure_detail(operation, admission))
       << "}";
  return {409, "application/json", body.str()};
}

}  // namespace

class ElevatorModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    MapCatalog & map_catalog,
    std::mutex & map_mutation_mutex,
    std::mutex & cross_asset_commit_mutex,
    ElevatorModuleConfig config,
    ElevatorModulePorts ports)
  : node_(node),
    map_catalog_(map_catalog),
    map_mutation_mutex_(map_mutation_mutex),
    cross_asset_commit_mutex_(cross_asset_commit_mutex),
    config_(std::move(config)),
    ports_(std::move(ports)),
    motion_admission_fence_(std::make_shared<ElevatorMotionAdmissionFence>())
  {
    require_ports(ports_);
    if (config_.maps_root.empty()) {
      throw std::invalid_argument("elevator module requires maps_root");
    }

    configuration_ = std::make_unique<ElevatorConfigurationModule>(
      config_.maps_root,
      [this](
        const std::string & building_id,
        const std::string & floor_id,
        const std::string & map_id)
      {
        return resolve_floor_asset(building_id, floor_id, map_id);
      });
    initialize_execution();
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch,
    const bool api_token_configured,
    const bool maintenance_peer_is_loopback)
  {
    if (request.method == "GET" && request.path == "/api/v1/elevator-config") {
      return handle_get_elevator_configuration(request);
    }
    if (request.method == "PUT" &&
      request.path == "/api/v1/elevator-config/draft")
    {
      return handle_save_elevator_configuration_draft(request.body);
    }
    if (request.method == "POST" &&
      request.path == "/api/v1/elevator-config/publish")
    {
      return handle_publish_elevator_configuration(request.body);
    }
    if (request.method == "POST" &&
      request.path == "/api/v1/elevator-config/rollback")
    {
      return handle_rollback_elevator_configuration(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/elevator-test/start") {
      return handle_start_elevator_test(request.body, motion_admission_epoch);
    }
    if (request.method == "GET" && request.path == "/api/v1/elevator-test/state") {
      return handle_elevator_test_state(request);
    }
    if (request.method == "POST" &&
      request.path == "/api/v1/elevator-test/confirm")
    {
      return handle_confirm_elevator_test(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/elevator-test/cancel") {
      return handle_cancel_elevator_test(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/elevator-test/recover") {
      if (!elevator_recovery_maintenance_peer_allowed(
          api_token_configured, maintenance_peer_is_loopback))
      {
        return HttpResponse{
          403,
          "application/json",
          error_json(
            "elevator recovery requires authenticated API access or a "
            "loopback maintenance connection")};
      }
      return handle_recover_elevator_test(request.body);
    }
    return std::nullopt;
  }

  ElevatorExecutionInterlock execution_interlock() const
  {
    return evaluate_elevator_execution_interlock(
      test_ ? test_->current_snapshot() : std::nullopt,
      test_ && test_->journal_recovery_required(),
      test_ && test_->persistent_recovery_lock_enabled());
  }

  ElevatorMotionAdmissionFence::Epoch capture_motion_admission_epoch() const
  {
    return motion_admission_fence_->capture_epoch();
  }

  ElevatorMotionAdmissionFence::AdmissionGuard acquire_motion_admission(
    const ElevatorMotionAdmissionFence::Epoch expected_epoch) const
  {
    return motion_admission_fence_->acquire_for_submission(
      expected_epoch,
      [this]() {
        const auto unknown_count = ports_.delayed_side_effect_unknown_probe();
        if (unknown_count != 0U) {
          return ElevatorExecutionInterlock{
            ElevatorExecutionInterlockKind::kDelayedSideEffectUnknown,
            "",
            unknown_count};
        }
        return execution_interlock();
      });
  }

  std::optional<HttpResponse> interlock_response(
    const HttpRequest & request) const
  {
    if (!test_ || request.method == "GET" || request.method == "OPTIONS") {
      return std::nullopt;
    }
    const auto interlock = execution_interlock();
    if (!interlock.blocked()) {
      return std::nullopt;
    }
    const bool allowed =
      request.path == "/api/v1/elevator-test/start" ||
      request.path == "/api/v1/elevator-test/confirm" ||
      request.path == "/api/v1/elevator-test/cancel" ||
      request.path == "/api/v1/elevator-test/recover" ||
      request.path == "/api/v1/safety/stop" ||
      request.path == "/api/v1/navigation/cancel" ||
      request.path == "/api/v1/navigation/stop" ||
      request.path == "/api/v1/navigation/stop_runtime" ||
      request.path == "/api/v1/docking/cancel" ||
      request.path == "/api/v1/docking/stop" ||
      request.path == "/api/v1/mapping/2d/stop" ||
      request.path == "/api/v1/mapping/stop" ||
      starts_with(request.path, "/api/v1/subscriptions/");
    if (allowed) {
      return std::nullopt;
    }

    std::ostringstream body;
    body << "{\"ok\":false,"
         << "\"code\":" << json_string(
      interlock.recovery_required() ?
      "ELEVATOR_EXECUTION_RECOVERY_REQUIRED" :
      "ELEVATOR_EXECUTION_ACTIVE") << ","
         << "\"transaction_id\":" << json_string(interlock.transaction_id) << ","
         << "\"detail\":" << json_string(
      interlock.recovery_required() ?
      "state-changing API is blocked by retained elevator recovery state" :
      "state-changing API is blocked until the elevator transaction reaches "
      "a terminal state without a retained safety hold") << "}";
    return HttpResponse{409, "application/json", body.str()};
  }

  HttpResponse query_map_reference(const MapManifest & manifest) const
  {
    if (!configuration_) {
      return {
        503,
        "application/json",
        error_json("elevator configuration module is not initialized")};
    }
    ElevatorConfigurationQuery query;
    query.building_id = manifest.building_id;
    query.floor_id = manifest.floor_id;
    query.map_id = manifest.map_id;
    return configuration_response(configuration_->query(query));
  }

private:
  friend class ElevatorModule;

  void initialize_execution()
  {
    if (!config_.runtime_adapter_enabled) {
      test_ = std::make_unique<ElevatorTestModule>(
        config_.maps_root,
        robot_elevator_manager::ElevatorExecutionOptions{false});
      RCLCPP_WARN(
        node_.get_logger(),
        "elevator runtime adapter is disabled; /elevator-test remains read-only");
      return;
    }

    try {
      std::shared_ptr<ElevatorArmClient> arm_client;
      if (config_.arm_button_control_enabled) {
        ElevatorArmClientOptions arm_options;
        arm_options.host = config_.arm_service_host;
        arm_options.port = static_cast<std::uint16_t>(config_.arm_service_port);
        arm_options.request_timeout = std::chrono::milliseconds(
          static_cast<std::int64_t>(
            std::llround(config_.arm_request_timeout_sec * 1000.0)));
        arm_options.task_timeout = std::chrono::milliseconds(
          static_cast<std::int64_t>(
            std::llround(config_.arm_task_timeout_sec * 1000.0)));
        arm_options.poll_interval = std::chrono::milliseconds(
          static_cast<std::int64_t>(
            std::llround(config_.arm_poll_interval_sec * 1000.0)));
        arm_client = std::make_shared<ElevatorArmClient>(std::move(arm_options));
      }

      ElevatorRosRuntimeOptions options;
      options.maps_root = config_.maps_root;
      options.runtime_map_context_file = config_.runtime_map_context_file;
      options.navigate_to_pose_action = config_.navigate_to_pose_action;
      options.floor_switch_action = config_.floor_switch_action;
      options.floor_switch_timeout_sec = config_.floor_switch_timeout_sec;
      options.elevator_scoped_behavior_tree = config_.elevator_scoped_behavior_tree;
      options.elevator_hall_call_scoped_behavior_tree =
        config_.elevator_hall_call_scoped_behavior_tree;
      options.elevator_reverse_entry_staging_behavior_tree =
        config_.elevator_reverse_entry_staging_behavior_tree;
      options.elevator_reverse_docking_behavior_tree =
        config_.elevator_reverse_docking_behavior_tree;
      options.elevator_cabin_entry_direct_behavior_tree =
        config_.elevator_cabin_entry_direct_behavior_tree;
      options.elevator_entry_collision_bypass_permit_topic =
        config_.elevator_entry_collision_bypass_permit_topic;
      options.elevator_entry_collision_bypass_refresh_sec =
        config_.elevator_entry_collision_bypass_refresh_sec;
      options.motion_allowed_topic = config_.motion_allowed_topic;
      options.safety_status_topic = config_.safety_status_topic;
      options.navigation_status_topic = config_.navigation_status_topic;
      options.recovery_hold_release_service =
        config_.recovery_hold_release_service;
      options.motion_admission_fence = motion_admission_fence_;
      options.persistent_recovery_lock_enabled = false;
      options.arm_button_control_enabled = config_.arm_button_control_enabled;
      options.arm_client = std::move(arm_client);
      options.runtime_idle_probe = ports_.runtime_idle_probe;
      options.delayed_side_effect_unknown_probe =
        ports_.delayed_side_effect_unknown_probe;
      options.current_map_pose_probe = ports_.current_map_pose_probe;

      runtime_port_ =
        std::make_shared<ElevatorRosRuntimePort>(std::move(options));
      test_ = std::make_unique<ElevatorTestModule>(
        config_.maps_root,
        runtime_port_,
        robot_elevator_manager::ElevatorExecutionOptions{false});
      RCLCPP_INFO(
        node_.get_logger(),
        "production elevator runtime adapter is enabled; motion remains "
        "owner-held, floor switching remains exact-identity gated, and "
        "automatic arm button control is %s",
        config_.arm_button_control_enabled ? "enabled" : "disabled");
    } catch (const std::exception & exception) {
      runtime_port_.reset();
      test_ = std::make_unique<ElevatorTestModule>(
        config_.maps_root,
        robot_elevator_manager::ElevatorExecutionOptions{false});
      RCLCPP_ERROR(
        node_.get_logger(),
        "elevator runtime adapter initialization failed closed: %s",
        exception.what());
    }
  }

  std::optional<ElevatorFloorAssetSnapshot> resolve_floor_asset(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id) const
  {
    const auto maps = map_catalog_.read_floor_map_manifests(
      building_id, floor_id, false);
    const auto match = std::find_if(
      maps.begin(), maps.end(), [&map_id](const MapManifest & manifest) {
        return manifest.map_id == map_id;
      });
    if (match == maps.end()) {
      return std::nullopt;
    }

    ElevatorFloorAssetSnapshot snapshot;
    const auto frozen_manifest = *match;
    snapshot.manifest = frozen_manifest;
    snapshot.asset_epoch = frozen_manifest.asset_epoch;
    snapshot.asset_digest = frozen_manifest.asset_digest;
    const std::vector<fs::path> required_assets{
      frozen_manifest.nav_map_yaml,
      frozen_manifest.nav_map_pgm,
      frozen_manifest.localizer_map_png,
      frozen_manifest.localizer_params_yaml,
      frozen_manifest.keepout_mask_yaml,
      frozen_manifest.keepout_mask_pgm,
      frozen_manifest.speed_mask_yaml,
      frozen_manifest.speed_mask_pgm,
      frozen_manifest.binary_mask_yaml,
      frozen_manifest.binary_mask_pgm,
      frozen_manifest.asset_report_json,
      frozen_manifest.poses_yaml,
    };
    const auto expected_root = map_catalog_.map_root_path(
      building_id, floor_id, map_id);
    const bool root_identity_matches =
      ::robot_api_server::features::maps::same_normalized_path(
      frozen_manifest.root, expected_root);
    const bool bundle_root_is_safe =
      ::robot_api_server::features::maps::safe_bundle_directory(
      frozen_manifest.root, config_.maps_root);
    const bool manifest_is_safe =
      ::robot_api_server::features::maps::safe_bundle_regular_file(
      frozen_manifest.manifest_json, frozen_manifest.root);
    const bool required_assets_safe = std::all_of(
      required_assets.begin(), required_assets.end(),
      [&frozen_manifest](const fs::path & asset) {
        return ::robot_api_server::features::maps::safe_bundle_regular_file(
          asset, frozen_manifest.root);
      });
    const bool base_assets_safe =
      root_identity_matches && bundle_root_is_safe && manifest_is_safe &&
      required_assets_safe;
    snapshot.required_assets_complete = base_assets_safe;
    if (!base_assets_safe) {
      return snapshot;
    }

    try {
      const auto verified = verify_map_asset_identity_snapshot(
        frozen_manifest, config_.maps_root);
      snapshot.manifest = verified.manifest;
      snapshot.asset_epoch = verified.manifest.asset_epoch;
      snapshot.asset_digest = verified.manifest.asset_digest;
      snapshot.map_info = read_nav_map_info_exact_content(
        verified.nav_map_yaml,
        verified.nav_map_pgm_header,
        verified.manifest.nav_map_pgm.filename().string());
      if (!snapshot.map_info) {
        snapshot.required_assets_complete = false;
      }
    } catch (const std::exception &) {
      snapshot.required_assets_complete = false;
      snapshot.map_info.reset();
    }
    return snapshot;
  }

  HttpResponse handle_get_elevator_configuration(const HttpRequest & request)
  {
    const auto building_id = query_string_value(request, "building_id");
    if (building_id.empty()) {
      return {
        400,
        "application/json",
        error_json("building_id query parameter is required")};
    }
    ElevatorConfigurationQuery query;
    query.building_id = building_id;
    query.release_id = query_string_value(request, "release_id");
    return configuration_response(configuration_->query(query));
  }

  HttpResponse handle_save_elevator_configuration_draft(const std::string & body)
  {
    if (const auto blocked =
      ports_.floor_runtime_interlock_response("elevator_config_save_draft"))
    {
      return *blocked;
    }
    std::lock_guard<std::mutex> map_guard(map_mutation_mutex_);
    std::lock_guard<std::mutex> cross_asset_guard(cross_asset_commit_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response(
        "elevator_config_save_draft_commit"))
    {
      return *blocked;
    }
    const auto document = json_object_value(body, "configuration").value_or(
      json_object_value(body, "config").value_or(body));
    const auto building_id = json_string_value(document, "building_id").value_or(
      json_string_value(body, "building_id").value_or(""));
    if (building_id.empty()) {
      return {
        400,
        "application/json",
        error_json("building_id is required in the elevator configuration")};
    }
    ElevatorConfigurationCommand command;
    command.type = ElevatorConfigurationCommandType::kSaveDraft;
    command.building_id = building_id;
    command.document = document;
    command.expected_draft_revision =
      json_string_value(body, "expected_draft_revision").value_or(
      json_string_value(body, "expected_revision").value_or(""));
    command.actor_id = json_string_value(body, "actor_id").value_or("");
    return configuration_response(configuration_->execute(command));
  }

  HttpResponse handle_publish_elevator_configuration(const std::string & body)
  {
    if (const auto blocked =
      ports_.floor_runtime_interlock_response("elevator_config_publish"))
    {
      return *blocked;
    }
    std::lock_guard<std::mutex> map_guard(map_mutation_mutex_);
    std::lock_guard<std::mutex> cross_asset_guard(cross_asset_commit_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response(
        "elevator_config_publish_commit"))
    {
      return *blocked;
    }
    ElevatorConfigurationCommand command;
    command.type = ElevatorConfigurationCommandType::kPublish;
    command.building_id = json_string_value(body, "building_id").value_or("");
    command.expected_draft_revision =
      json_string_value(body, "expected_draft_revision").value_or(
      json_string_value(body, "expected_revision").value_or(""));
    command.expected_release_id =
      json_string_value(body, "expected_release_id").value_or(
      json_string_value(body, "expected_current_release_id").value_or(""));
    command.actor_id = json_string_value(body, "actor_id").value_or("");
    return configuration_response(configuration_->execute(command));
  }

  HttpResponse handle_rollback_elevator_configuration(const std::string & body)
  {
    if (const auto blocked =
      ports_.floor_runtime_interlock_response("elevator_config_rollback"))
    {
      return *blocked;
    }
    std::lock_guard<std::mutex> map_guard(map_mutation_mutex_);
    std::lock_guard<std::mutex> cross_asset_guard(cross_asset_commit_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response(
        "elevator_config_rollback_commit"))
    {
      return *blocked;
    }
    ElevatorConfigurationCommand command;
    command.type = ElevatorConfigurationCommandType::kRollback;
    command.building_id = json_string_value(body, "building_id").value_or("");
    command.release_id = json_string_value(body, "release_id").value_or(
      json_string_value(body, "target_release_id").value_or(""));
    command.expected_release_id =
      json_string_value(body, "expected_release_id").value_or(
      json_string_value(body, "expected_current_release_id").value_or(""));
    command.actor_id = json_string_value(body, "actor_id").value_or("");
    return configuration_response(configuration_->execute(command));
  }

  HttpResponse handle_start_elevator_test(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    auto motion_admission = acquire_motion_admission(motion_admission_epoch);
    if (!motion_admission.admitted()) {
      return make_motion_admission_failure_response(
        "elevator_test_start", motion_admission);
    }
    // start() invokes prepare(), which closes and invalidates this same fence.
    motion_admission.unlock();
    ElevatorTestStartCommand command;
    command.building_id = json_string_value(body, "building_id").value_or("");
    command.elevator_id = json_string_value(body, "elevator_id").value_or("");
    command.source_floor_id =
      json_string_value(body, "source_floor_id").value_or("");
    command.source_map_id =
      json_string_value(body, "source_map_id").value_or("");
    command.target_floor_id =
      json_string_value(body, "target_floor_id").value_or("");
    command.target_map_id =
      json_string_value(body, "target_map_id").value_or("");
    command.expected_release_id =
      json_string_value(body, "expected_release_id").value_or("");
    command.operator_id = json_string_value(body, "operator_id").value_or("");
    return test_response(test_->start(command));
  }

  HttpResponse handle_elevator_test_state(const HttpRequest & request)
  {
    ElevatorTestStateQuery query;
    query.transaction_id = query_string_value(request, "transaction_id");
    return test_response(test_->state(query));
  }

  static std::uint64_t elevator_test_effect_sequence(const std::string & body)
  {
    constexpr double kMaxExactJsonInteger = 9007199254740991.0;  // 2^53 - 1
    const auto value = json_number_value(body, "effect_sequence");
    if (!value || !std::isfinite(*value) || *value < 1.0 ||
      *value > kMaxExactJsonInteger || std::floor(*value) != *value)
    {
      return 0U;
    }
    return static_cast<std::uint64_t>(*value);
  }

  HttpResponse handle_confirm_elevator_test(const std::string & body)
  {
    ElevatorTestConfirmCommand command;
    command.transaction_id =
      json_string_value(body, "transaction_id").value_or("");
    command.expected_state = json_string_value(body, "expected_state").value_or("");
    command.effect_sequence = elevator_test_effect_sequence(body);
    command.event = json_string_value(body, "event").value_or("");
    command.observed_floor_id =
      json_string_value(body, "observed_floor_id").value_or("");
    command.operator_id = json_string_value(body, "operator_id").value_or("");
    return test_response(test_->confirm(command));
  }

  HttpResponse handle_cancel_elevator_test(const std::string & body)
  {
    ElevatorTestCancelCommand command;
    command.transaction_id =
      json_string_value(body, "transaction_id").value_or("");
    command.effect_sequence = elevator_test_effect_sequence(body);
    command.reason = json_string_value(body, "reason").value_or("");
    command.operator_id = json_string_value(body, "operator_id").value_or("");
    return test_response(test_->cancel(command));
  }

  HttpResponse handle_recover_elevator_test(const std::string & body)
  {
    ElevatorTestRecoverCommand command;
    command.transaction_id =
      json_string_value(body, "transaction_id").value_or("");
    command.expected_state = json_string_value(body, "expected_state").value_or("");
    command.effect_sequence = elevator_test_effect_sequence(body);
    command.operator_id = json_string_value(body, "operator_id").value_or("");
    command.reason = json_string_value(body, "reason").value_or("");
    command.source_outside_confirmed =
      json_bool_value(body, "source_outside_confirmed", false);
    command.confirmed_floor_id =
      json_string_value(body, "confirmed_floor_id").value_or("");
    command.action = json_string_value(body, "action").value_or("");
    command.physical_zone = json_string_value(body, "physical_zone").value_or("");
    command.stationary_confirmed =
      json_bool_value(body, "stationary_confirmed", false);
    command.door_zone_clear_confirmed =
      json_bool_value(body, "door_zone_clear_confirmed", false);
    return test_response(test_->recover(command));
  }

  rclcpp::Node & node_;
  MapCatalog & map_catalog_;
  std::mutex & map_mutation_mutex_;
  std::mutex & cross_asset_commit_mutex_;
  ElevatorModuleConfig config_;
  ElevatorModulePorts ports_;
  std::shared_ptr<ElevatorMotionAdmissionFence> motion_admission_fence_;
  std::unique_ptr<ElevatorConfigurationModule> configuration_;
  std::shared_ptr<ElevatorRosRuntimePort> runtime_port_;
  std::unique_ptr<ElevatorTestModule> test_;
};

ElevatorModule::ElevatorModule(
  rclcpp::Node & node,
  MapCatalog & map_catalog,
  std::mutex & map_mutation_mutex,
  std::mutex & cross_asset_commit_mutex,
  ElevatorModuleConfig config,
  ElevatorModulePorts ports)
: impl_(std::make_unique<Impl>(
    node,
    map_catalog,
    map_mutation_mutex,
    cross_asset_commit_mutex,
    std::move(config),
    std::move(ports)))
{
}

ElevatorModule::~ElevatorModule() = default;

std::optional<HttpResponse> ElevatorModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch,
  const bool api_token_configured,
  const bool maintenance_peer_is_loopback)
{
  return impl_->handle_http(
    request,
    motion_admission_epoch,
    api_token_configured,
    maintenance_peer_is_loopback);
}

std::optional<HttpResponse> ElevatorModule::interlock_response(
  const HttpRequest & request) const
{
  return impl_->interlock_response(request);
}

ElevatorExecutionInterlock ElevatorModule::execution_interlock() const
{
  return impl_->execution_interlock();
}

ElevatorMotionAdmissionFence::Epoch
ElevatorModule::capture_motion_admission_epoch() const
{
  return impl_->capture_motion_admission_epoch();
}

ElevatorMotionAdmissionFence::AdmissionGuard
ElevatorModule::acquire_motion_admission(
  const ElevatorMotionAdmissionFence::Epoch expected_epoch) const
{
  return impl_->acquire_motion_admission(expected_epoch);
}

HttpResponse ElevatorModule::motion_admission_failure_response(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission) const
{
  return make_motion_admission_failure_response(operation, admission);
}

HttpResponse ElevatorModule::query_map_reference(
  const MapManifest & manifest) const
{
  return impl_->query_map_reference(manifest);
}

}  // namespace robot_api_server::features::elevator
