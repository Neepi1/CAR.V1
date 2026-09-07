#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "action_msgs/srv/cancel_goal.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/floor_switch.hpp"
#include "robot_interfaces/msg/floor_switch_status.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"

#include "robot_api_server/features/floor_switch/floor_switch_http_transaction.hpp"
#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server::features::floor_switch
{

using namespace std::chrono_literals;
using FloorSwitchAction = robot_interfaces::action::FloorSwitch;
using FloorSwitchGoalHandle = rclcpp_action::ClientGoalHandle<FloorSwitchAction>;
using features::maps::ExactMapAssetSourceDrift;
using features::maps::same_exact_map_asset_source;
using features::maps::same_normalized_path;

namespace
{

void require_ports(const FloorSwitchModulePorts & ports)
{
  if (!ports.map_asset_integrity_degraded ||
    !ports.runtime_snapshot ||
    !ports.validate_map_manifest_assets ||
    !ports.activate_map_manifest ||
    !ports.acquire_motion_admission ||
    !ports.mark_delayed_side_effect_unknown ||
    !ports.resolve_delayed_side_effect_unknown)
  {
    throw std::invalid_argument("floor-switch module requires every cross-domain port");
  }
}

std::string query_string_value(
  const HttpRequest & request,
  const std::string & key,
  const std::string & default_value)
{
  const auto it = request.query.find(key);
  return it == request.query.end() ? default_value : it->second;
}

std::string motion_admission_failure_detail(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  if (admission.stale()) {
    return operation + " request predates the completed elevator admission barrier";
  }
  if (admission.interlock().delayed_side_effect_unknown()) {
    return operation + " is blocked because " +
           std::to_string(admission.interlock().delayed_side_effect_unknown_count) +
           " timed-out motion/runtime submission(s) have no proven outcome";
  }
  if (admission.interlock().recovery_required()) {
    return operation +
           " is blocked by retained elevator recovery state; transaction_id=" +
           admission.interlock().transaction_id;
  }
  return operation + " is blocked during elevator execution; transaction_id=" +
         admission.interlock().transaction_id;
}

HttpResponse motion_admission_failure_response(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  const std::string code = admission.stale() ?
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

class DelayedSideEffectEvidence
{
public:
  explicit DelayedSideEffectEvidence(const FloorSwitchModulePorts & ports)
  : ports_(ports)
  {
    ports_.mark_delayed_side_effect_unknown();
  }

  DelayedSideEffectEvidence(const DelayedSideEffectEvidence &) = delete;
  DelayedSideEffectEvidence & operator=(const DelayedSideEffectEvidence &) = delete;

  void resolve() noexcept
  {
    if (!resolved_) {
      ports_.resolve_delayed_side_effect_unknown();
      resolved_ = true;
    }
  }

private:
  const FloorSwitchModulePorts & ports_;
  bool resolved_{false};
};

}  // namespace

class FloorSwitchModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapCatalog & map_catalog,
    std::mutex & map_asset_mutation_mutex,
    std::mutex & activation_commit_mutex,
    FloorSwitchModuleConfig config,
    FloorSwitchModulePorts ports)
  : node_(node),
    callback_group_(std::move(callback_group)),
    map_catalog_(map_catalog),
    map_asset_mutation_mutex_(map_asset_mutation_mutex),
    activation_commit_mutex_(activation_commit_mutex),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
    require_ports(ports_);
    if (!callback_group_) {
      throw std::invalid_argument("floor-switch module requires a ROS callback group");
    }

    legacy_client_ = node_.create_client<robot_interfaces::srv::SwitchFloor>(
      config_.legacy_service,
      rmw_qos_profile_services_default,
      callback_group_);
    live_client_ = rclcpp_action::create_client<FloorSwitchAction>(
      &node_, config_.live_action);

    const auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    transition_status_sub_ =
      node_.create_subscription<robot_interfaces::msg::FloorSwitchStatus>(
      config_.transition_status_topic,
      retained_qos,
      [this](const robot_interfaces::msg::FloorSwitchStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(interlock_mutex_);
        interlock_.observe_floor_switch_status(
          msg->transaction_id,
          msg->state,
          msg->stage,
          msg->failure_code,
          msg->detail);
      });
    localization_health_sub_ =
      node_.create_subscription<robot_interfaces::msg::LocalizationHealth>(
      config_.localization_health_topic,
      retained_qos,
      [this](const robot_interfaces::msg::LocalizationHealth::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(interlock_mutex_);
        interlock_.observe_localization_health(
          msg->transition_active,
          msg->runtime_context_valid,
          msg->detail);
      });
  }

  ~Impl()
  {
    shutdown();
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (request.method == "POST" && request.path == "/api/v1/floor-switch/start") {
      return handle_live_start(request.body, motion_admission_epoch);
    }
    if (request.method == "GET" && request.path == "/api/v1/floor-switch/state") {
      return handle_live_state(request);
    }
    if (request.method == "POST" && request.path == "/api/v1/floor-switch/cancel") {
      return handle_live_cancel(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/floors/switch") {
      return handle_offline_switch(request.body, motion_admission_epoch);
    }
    return std::nullopt;
  }

  bool interlock_enabled() const noexcept
  {
    return config_.negative_interlock_enabled;
  }

  FloorRuntimeInterlockDecision interlock_decision() const
  {
    if (!config_.negative_interlock_enabled) {
      FloorRuntimeInterlockDecision decision;
      decision.code = "FLOOR_RUNTIME_INTERLOCK_DISABLED";
      decision.detail = "floor runtime negative interlock is disabled by configuration";
      return decision;
    }
    std::lock_guard<std::mutex> lock(interlock_mutex_);
    return interlock_.decision();
  }

  bool operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code) const
  {
    const auto decision = interlock_decision();
    if (!decision.blocked) {
      detail.clear();
      if (reason_code != nullptr) {
        *reason_code = decision.code;
      }
      return false;
    }

    std::ostringstream message;
    message << operation << " is blocked by the floor runtime interlock: "
            << decision.detail;
    if (!decision.transaction_id.empty()) {
      message << " transaction_id=" << decision.transaction_id;
    }
    detail = message.str();
    if (reason_code != nullptr) {
      *reason_code = decision.code;
    }
    return true;
  }

  std::optional<HttpResponse> interlock_response(
    const std::string & operation) const
  {
    const auto decision = interlock_decision();
    if (!decision.blocked) {
      return std::nullopt;
    }

    std::ostringstream detail;
    detail << operation << " is blocked by the floor runtime interlock: "
           << decision.detail;
    if (!decision.transaction_id.empty()) {
      detail << " transaction_id=" << decision.transaction_id;
    }
    std::ostringstream body;
    body << "{\"ok\":false,"
         << "\"code\":\"FLOOR_TRANSITION_BLOCKED\","
         << "\"reason_code\":" << json_string(decision.code) << ","
         << "\"operation\":" << json_string(operation) << ","
         << "\"transaction_id\":" << json_string(decision.transaction_id) << ","
         << "\"error\":" << json_string(detail.str()) << "}";
    const int status = decision.code == "FLOOR_TRANSITION_ACTIVE" ? 409 : 503;
    return HttpResponse{status, "application/json", body.str()};
  }

  void shutdown()
  {
    if (shutdown_requested_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    const auto snapshot = transaction_.latest_snapshot();
    if (!snapshot.transaction_id.empty() && !snapshot.terminal()) {
      (void)transaction_.request_cancel(snapshot.transaction_id);
    }
    join_live_worker();
  }

private:
  static std::string live_response_json(
    const FloorSwitchHttpSnapshot & snapshot,
    const bool accepted)
  {
    std::ostringstream out;
    out << "{\"ok\":true,\"accepted\":"
        << (accepted ? "true" : "false")
        << ",\"transaction\":"
        << floor_switch_http_snapshot_json(snapshot)
        << "}";
    return out.str();
  }

  std::optional<RuntimeMapContext> read_runtime_context() const
  {
    if (config_.runtime_map_context_file.empty()) {
      return std::nullopt;
    }
    return read_runtime_map_context_file(config_.runtime_map_context_file);
  }

  void clear_runtime_context() const
  {
    if (config_.runtime_map_context_file.empty()) {
      return;
    }
    std::error_code error;
    std::filesystem::remove(config_.runtime_map_context_file, error);
  }

  bool runtime_context_matches_map(
    const MapManifest & map,
    std::string & error) const
  {
    const auto context = read_runtime_context();
    if (!context) {
      error = "no runtime map context";
      return false;
    }
    if (!context->confirmed || context->state != "ready") {
      error =
        "runtime map context is not ready: " + context->building_id + "/" +
        context->floor_id + "/" + context->map_id + " state=" + context->state;
      return false;
    }
    if (context->building_id != map.building_id ||
      context->floor_id != map.floor_id || context->map_id != map.map_id)
    {
      error =
        "runtime map changed: expected " + map.building_id + "/" + map.floor_id + "/" +
        map.map_id + " but found " + context->building_id + "/" + context->floor_id + "/" +
        context->map_id;
      return false;
    }
    return true;
  }

  std::optional<std::string> offline_runtime_busy_detail() const
  {
    const auto runtime = ports_.runtime_snapshot();
    if (runtime.navigation_active || runtime.mapping_active || runtime.docking_active) {
      return "navigation, mapping, or docking runtime is active";
    }
    if (runtime.navigation_goal_running) {
      return "a navigation goal job is still running";
    }
    if (runtime.mapping_start_job_running) {
      return "a mapping start job is still running";
    }
    if (runtime.docking_job_running) {
      return "a docking job is still running";
    }
    if (runtime.nav2_goal_active) {
      return "Nav2 still reports an active action goal";
    }
    if (runtime.navigation_process_running) {
      return "the navigation runtime process is still running";
    }
    return std::nullopt;
  }

  void join_live_worker()
  {
    std::thread worker;
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_.joinable()) {
        worker = std::move(worker_);
      }
    }
    if (worker.joinable()) {
      worker.join();
    }
  }

  void run_live_worker(
    const std::string transaction_id,
    std::shared_future<FloorSwitchGoalHandle::SharedPtr> goal_future)
  {
    FloorSwitchHttpOutcome outcome;
    const auto finish_unknown =
      [this, &transaction_id, &outcome](const std::string & detail) {
        outcome.terminal_proven = false;
        outcome.success = false;
        outcome.recovery_required = true;
        outcome.detail = detail;
        (void)transaction_.finish(transaction_id, outcome);
      };
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.live_timeout_sec));

    while (goal_future.wait_for(50ms) != std::future_status::ready) {
      if (shutdown_requested_.load(std::memory_order_acquire)) {
        finish_unknown(
          "robot_api_server stopped before floor-switch goal admission was proven");
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        finish_unknown(
          "floor-switch goal admission remained unknown until the transaction timeout");
        return;
      }
    }

    FloorSwitchGoalHandle::SharedPtr goal_handle;
    try {
      goal_handle = goal_future.get();
    } catch (const std::exception & exception) {
      outcome.detail = std::string("floor-switch goal response failed: ") + exception.what();
      outcome.failure_code = FloorSwitchAction::Result::INTERNAL_ERROR;
      (void)transaction_.finish(transaction_id, outcome);
      return;
    } catch (...) {
      outcome.detail = "floor-switch goal response failed with an unknown exception";
      outcome.failure_code = FloorSwitchAction::Result::INTERNAL_ERROR;
      (void)transaction_.finish(transaction_id, outcome);
      return;
    }
    if (!goal_handle) {
      outcome.detail = "floor manager rejected the strict floor-switch action goal";
      outcome.failure_code = FloorSwitchAction::Result::TRANSACTION_CONFLICT;
      (void)transaction_.finish(transaction_id, outcome);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_transaction_id_ = transaction_id;
      goal_handle_ = goal_handle;
    }
    (void)transaction_.observe_goal_accepted(transaction_id);

    std::shared_future<FloorSwitchGoalHandle::WrappedResult> result_future;
    try {
      result_future = live_client_->async_get_result(goal_handle);
    } catch (const std::exception & exception) {
      finish_unknown(
        std::string("failed to request the exact floor-switch result: ") + exception.what());
      return;
    }

    bool cancel_sent = false;
    bool timeout_cancel = false;
    auto cancel_terminal_deadline = deadline + config_.service_timeout;
    while (result_future.wait_for(50ms) != std::future_status::ready) {
      if (shutdown_requested_.load(std::memory_order_acquire)) {
        finish_unknown(
          "robot_api_server stopped before the exact floor-switch terminal was proven");
        return;
      }
      const bool operator_cancel = transaction_.cancel_requested(transaction_id);
      const bool timed_out = std::chrono::steady_clock::now() >= deadline;
      if ((operator_cancel || timed_out) && !cancel_sent) {
        timeout_cancel = timed_out && !operator_cancel;
        cancel_sent = true;
        try {
          auto cancel_future = live_client_->async_cancel_goal(goal_handle);
          if (cancel_future.wait_for(config_.service_timeout) != std::future_status::ready) {
            finish_unknown(
              "floor-switch cancel outcome is unknown; exact action terminal was not proven");
            return;
          }
          const auto cancel_response = cancel_future.get();
          if (cancel_response->return_code !=
            action_msgs::srv::CancelGoal::Response::ERROR_NONE)
          {
            finish_unknown(
              "floor manager did not acknowledge cancellation of the exact action goal");
            return;
          }
          cancel_terminal_deadline =
            std::chrono::steady_clock::now() + config_.service_timeout;
        } catch (const std::exception & exception) {
          finish_unknown(
            std::string("floor-switch cancellation failed: ") + exception.what());
          return;
        }
      }
      if (cancel_sent && std::chrono::steady_clock::now() >= cancel_terminal_deadline) {
        finish_unknown(
          "floor-switch cancellation was acknowledged but its exact terminal is unknown");
        return;
      }
    }

    FloorSwitchGoalHandle::WrappedResult wrapped;
    try {
      wrapped = result_future.get();
    } catch (const std::exception & exception) {
      finish_unknown(std::string("floor-switch result future failed: ") + exception.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (goal_transaction_id_ == transaction_id) {
        goal_handle_.reset();
        goal_transaction_id_.clear();
      }
    }
    if (wrapped.code == rclcpp_action::ResultCode::UNKNOWN) {
      finish_unknown("floor manager returned UNKNOWN for the strict floor-switch action");
      return;
    }

    outcome.terminal_proven = true;
    outcome.cancelled = wrapped.code == rclcpp_action::ResultCode::CANCELED;
    if (wrapped.result) {
      outcome.success =
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success;
      outcome.failure_code = wrapped.result->failure_code;
      outcome.detail = wrapped.result->message;
      outcome.active_building_id = wrapped.result->active_building_id;
      outcome.active_floor_id = wrapped.result->active_floor_id;
      outcome.active_map_id = wrapped.result->active_map_id;
      outcome.asset_epoch = wrapped.result->asset_epoch;
      outcome.asset_digest = wrapped.result->asset_digest;
      outcome.explicit_relocalization_sequence =
        wrapped.result->explicit_relocalization_sequence;
      outcome.runtime_context_valid = wrapped.result->runtime_context_valid;
      outcome.recovery_required = wrapped.result->recovery_required;
    } else {
      outcome.detail = "floor-switch action reached terminal without a result payload";
      outcome.failure_code = FloorSwitchAction::Result::INTERNAL_ERROR;
    }
    if (timeout_cancel && outcome.cancelled) {
      outcome.detail = "floor-switch transaction timed out and the exact goal was canceled";
    }
    (void)transaction_.finish(transaction_id, outcome);
  }

  HttpResponse handle_live_start(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    std::lock_guard<std::mutex> start_guard(start_mutex_);
    const auto previous = transaction_.latest_snapshot();
    if (!previous.transaction_id.empty() && previous.terminal()) {
      join_live_worker();
    }
    if (const auto blocked = interlock_response("live_floor_switch_start")) {
      return *blocked;
    }
    if (ports_.map_asset_integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json("map asset integrity is degraded; live floor switching is blocked")};
    }

    const auto building_id = json_string_value(body, "building_id").value_or("");
    const auto floor_id = json_string_value(body, "floor_id").value_or("");
    const auto map_id = json_string_value(body, "map_id").value_or("");
    if (!safe_asset_id(building_id) || !safe_asset_id(floor_id) ||
      !safe_asset_id(map_id))
    {
      return {
        400,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_INVALID_TARGET\","
        "\"detail\":\"valid building_id, floor_id, and exact map_id are required\"}"};
    }

    std::lock_guard<std::mutex> asset_guard(map_asset_mutation_mutex_);
    const auto manifest = map_catalog_.find_map_by_id(map_id);
    if (!manifest || manifest->building_id != building_id ||
      manifest->floor_id != floor_id)
    {
      return {
        404,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_MAP_NOT_FOUND\","
        "\"detail\":\"the exact map_id is not bound to the requested floor\"}"};
    }
    std::string asset_error;
    if (manifest->asset_epoch == 0U || manifest->asset_digest.empty() ||
      !ports_.validate_map_manifest_assets(*manifest, asset_error))
    {
      return {
        409,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_ASSET_IDENTITY_UNPROVEN\","
        "\"detail\":" + json_string(
          asset_error.empty() ?
          "the exact map asset epoch/digest is not proven" : asset_error) + "}"};
    }

    const FloorSwitchHttpTarget target{
      building_id,
      floor_id,
      map_id,
      manifest->asset_epoch,
      manifest->asset_digest};
    FloorSwitchHttpRuntimeContext runtime_context;
    if (const auto context = read_runtime_context()) {
      runtime_context.available = true;
      runtime_context.confirmed = context->confirmed;
      runtime_context.state = context->state;
      runtime_context.building_id = context->building_id;
      runtime_context.floor_id = context->floor_id;
      runtime_context.map_id = context->map_id;
      runtime_context.asset_epoch = context->asset_epoch;
      runtime_context.asset_digest = context->asset_digest;
      runtime_context.explicit_relocalization_sequence =
        context->explicit_relocalization_sequence;
    }
    const auto runtime_admission =
      evaluate_floor_switch_runtime_admission(target, runtime_context);
    if (!runtime_admission.permitted) {
      return {
        409,
        "application/json",
        "{\"ok\":false,\"code\":" + json_string(runtime_admission.code) +
        ",\"detail\":" + json_string(runtime_admission.detail) + "}"};
    }

    if (runtime_admission.already_active) {
      const auto transaction_id =
        "manual-floor-switch-" + utc_timestamp_compact() + "-" +
        std::to_string(sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U);
      const auto decision = transaction_.start(transaction_id, target);
      if (!decision.accepted) {
        std::ostringstream response;
        response << "{\"ok\":false,\"code\":" << json_string(decision.code)
                 << ",\"detail\":" << json_string(decision.detail)
                 << ",\"transaction\":"
                 << floor_switch_http_snapshot_json(decision.snapshot) << "}";
        return {409, "application/json", response.str()};
      }

      FloorSwitchHttpOutcome outcome;
      outcome.success = true;
      outcome.detail = runtime_admission.detail;
      outcome.active_building_id = runtime_context.building_id;
      outcome.active_floor_id = runtime_context.floor_id;
      outcome.active_map_id = runtime_context.map_id;
      outcome.asset_epoch = runtime_context.asset_epoch;
      outcome.asset_digest = runtime_context.asset_digest;
      outcome.explicit_relocalization_sequence =
        runtime_context.explicit_relocalization_sequence;
      outcome.runtime_context_valid = true;
      (void)transaction_.finish(transaction_id, outcome);
      return {
        200,
        "application/json",
        live_response_json(transaction_.latest_snapshot(), true)};
    }

    if (!live_client_->wait_for_action_server(config_.service_timeout)) {
      return {
        503,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_ACTION_UNAVAILABLE\","
        "\"detail\":" + json_string(
          "action server unavailable: " + config_.live_action) + "}"};
    }

    auto motion_admission = ports_.acquire_motion_admission(motion_admission_epoch);
    if (!motion_admission.admitted()) {
      return motion_admission_failure_response(
        "live_floor_switch_start", motion_admission);
    }

    const auto transaction_id =
      "manual-floor-switch-" + utc_timestamp_compact() + "-" +
      std::to_string(sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U);
    const auto decision = transaction_.start(transaction_id, target);
    if (!decision.accepted) {
      motion_admission.unlock();
      std::ostringstream response;
      response << "{\"ok\":false,\"code\":" << json_string(decision.code)
               << ",\"detail\":" << json_string(decision.detail)
               << ",\"transaction\":"
               << floor_switch_http_snapshot_json(decision.snapshot) << "}";
      return {409, "application/json", response.str()};
    }

    FloorSwitchAction::Goal goal;
    goal.transaction_id = transaction_id;
    goal.building_id = building_id;
    goal.floor_id = floor_id;
    goal.map_id = map_id;
    goal.expected_asset_epoch = manifest->asset_epoch;
    goal.expected_asset_digest = manifest->asset_digest;
    rclcpp_action::Client<FloorSwitchAction>::SendGoalOptions options;
    options.feedback_callback =
      [this, transaction_id](
      FloorSwitchGoalHandle::SharedPtr,
      const std::shared_ptr<const FloorSwitchAction::Feedback> feedback)
      {
        const auto feedback_transaction_id =
          feedback->transaction_id.empty() ? transaction_id : feedback->transaction_id;
        if (feedback_transaction_id != transaction_id) {
          return;
        }
        (void)transaction_.observe_feedback(
          transaction_id,
          feedback->stage,
          feedback->progress,
          feedback->detail,
          feedback->stage_sequence);
      };

    std::shared_future<FloorSwitchGoalHandle::SharedPtr> goal_future;
    try {
      goal_future = live_client_->async_send_goal(goal, options);
    } catch (const std::exception & exception) {
      motion_admission.unlock();
      FloorSwitchHttpOutcome outcome;
      outcome.failure_code = FloorSwitchAction::Result::INTERNAL_ERROR;
      outcome.detail = std::string("failed to submit strict floor-switch action: ") +
        exception.what();
      (void)transaction_.finish(transaction_id, outcome);
      return {
        503,
        "application/json",
        live_response_json(transaction_.latest_snapshot(), false)};
    }
    motion_admission.unlock();

    shutdown_requested_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> worker_lock(worker_mutex_);
      worker_ = std::thread(
        [this, transaction_id, goal_future]() mutable {
          run_live_worker(transaction_id, std::move(goal_future));
        });
    }
    return {202, "application/json", live_response_json(decision.snapshot, true)};
  }

  HttpResponse handle_live_state(const HttpRequest & request)
  {
    const auto transaction_id = query_string_value(request, "transaction_id", "");
    if (transaction_id.empty()) {
      return {
        400,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_TRANSACTION_ID_REQUIRED\","
        "\"detail\":\"transaction_id query parameter is required\"}"};
    }
    const auto snapshot = transaction_.snapshot(transaction_id);
    if (!snapshot) {
      return {
        404,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_TRANSACTION_NOT_FOUND\","
        "\"detail\":\"the requested transaction is not retained by this server process\"}"};
    }
    return {
      200,
      "application/json",
      live_response_json(*snapshot, !snapshot->terminal())};
  }

  HttpResponse handle_live_cancel(const std::string & body)
  {
    const auto transaction_id =
      json_string_value(body, "transaction_id").value_or("");
    if (transaction_id.empty()) {
      return {
        400,
        "application/json",
        "{\"ok\":false,\"code\":\"FLOOR_SWITCH_TRANSACTION_ID_REQUIRED\","
        "\"detail\":\"transaction_id is required\"}"};
    }
    const auto decision = transaction_.request_cancel(transaction_id);
    if (!decision.accepted) {
      const int status =
        decision.code == "FLOOR_SWITCH_TRANSACTION_NOT_FOUND" ? 404 : 409;
      std::ostringstream response;
      response << "{\"ok\":false,\"code\":" << json_string(decision.code)
               << ",\"detail\":" << json_string(decision.detail)
               << ",\"transaction\":"
               << floor_switch_http_snapshot_json(decision.snapshot) << "}";
      return {status, "application/json", response.str()};
    }
    return {202, "application/json", live_response_json(decision.snapshot, true)};
  }

  std::optional<MapManifest> confirmed_navigation_runtime_map_for_request(
    const std::string & body,
    const std::string & building_id,
    const std::optional<std::string> & floor_id,
    const std::optional<std::string> & map_id,
    const std::optional<std::string> & map_name) const
  {
    const auto runtime = ports_.runtime_snapshot();
    if (!runtime.navigation_active || runtime.mapping_active || runtime.docking_active ||
      !runtime.mode_transition_owner.empty() || runtime.mapping_start_job_running ||
      runtime.docking_job_running)
    {
      return std::nullopt;
    }

    const auto context = read_runtime_context();
    if (!context || !context->confirmed || context->state != "ready") {
      return std::nullopt;
    }
    const auto runtime_map = map_catalog_.find_map_by_id(context->map_id);
    if (!runtime_map || runtime_map->building_id != context->building_id ||
      runtime_map->floor_id != context->floor_id)
    {
      return std::nullopt;
    }

    bool requested_map_matches = false;
    if (map_id && !map_id->empty()) {
      requested_map_matches = *map_id == runtime_map->map_id;
      if (const auto requested_building = json_string_value(body, "building_id")) {
        requested_map_matches =
          requested_map_matches && *requested_building == runtime_map->building_id;
      }
      if (floor_id) {
        requested_map_matches =
          requested_map_matches && *floor_id == runtime_map->floor_id;
      }
    } else if (floor_id && !floor_id->empty() &&
      safe_asset_id(building_id) && safe_asset_id(*floor_id) &&
      building_id == runtime_map->building_id && *floor_id == runtime_map->floor_id)
    {
      if (map_name && !map_name->empty()) {
        std::string lookup_error;
        const auto requested_map = map_catalog_.find_floor_map_by_name(
          building_id, *floor_id, *map_name, lookup_error);
        requested_map_matches =
          lookup_error.empty() && requested_map &&
          requested_map->map_id == runtime_map->map_id;
      } else {
        const auto requested_map = map_catalog_.active_floor_map(building_id, *floor_id);
        requested_map_matches =
          requested_map && requested_map->map_id == runtime_map->map_id;
      }
    }
    if (!requested_map_matches) {
      return std::nullopt;
    }

    std::string context_error;
    std::string asset_error;
    if (!runtime_context_matches_map(*runtime_map, context_error) ||
      !ports_.validate_map_manifest_assets(*runtime_map, asset_error))
    {
      return std::nullopt;
    }
    return runtime_map;
  }

  static HttpResponse offline_runtime_busy_response(const std::string & detail)
  {
    std::ostringstream response;
    response << "{\"ok\":false,\"code\":\"FLOOR_SELECTION_RUNTIME_BUSY\","
             << "\"detail\":" << json_string(detail) << "}";
    return {409, "application/json", response.str()};
  }

  static HttpResponse runtime_map_already_selected_response(const MapManifest & map)
  {
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"state\":\"runtime_map_already_selected\","
             << "\"message\":\"requested map already matches confirmed navigation runtime; "
             << "no state changed\","
             << "\"map_id\":" << json_string(map.map_id) << ","
             << "\"display_name\":" << json_string(map.display_name) << ","
             << "\"building_id\":" << json_string(map.building_id) << ","
             << "\"floor_id\":" << json_string(map.floor_id) << ","
             << "\"resume_navigation\":false,"
             << "\"already_active\":true,"
             << "\"runtime_unchanged\":true,"
             << "\"selection_performed\":false,"
             << "\"nav_map_yaml\":" << json_string(map.nav_map_yaml.string()) << ","
             << "\"localizer_map_png\":" << json_string(map.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":"
             << json_string(map.localizer_params_yaml.string()) << "}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_offline_switch(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    std::lock_guard<std::mutex> asset_guard(map_asset_mutation_mutex_);
    if (const auto blocked = interlock_response("floor_switch")) {
      return *blocked;
    }
    if (ports_.map_asset_integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json("keepout integrity is degraded; floor switching is blocked until repaired")};
    }

    auto floor_id = json_string_value(body, "floor_id");
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const bool resume_navigation = json_bool_value(body, "resume_navigation", false);
    if (resume_navigation) {
      return {
        409,
        "application/json",
        "{\"ok\":false,\"code\":\"LIVE_FLOOR_SWITCH_DISABLED\","
        "\"error\":\"resume_navigation=true is disabled on the legacy endpoint; "
        "use POST /api/v1/floor-switch/start\"}"};
    }

    if (const auto busy = offline_runtime_busy_detail()) {
      const auto already_selected = confirmed_navigation_runtime_map_for_request(
        body, building_id, floor_id, map_id, map_name);
      if (already_selected) {
        if (const auto blocked = interlock_response("floor_switch_noop_commit")) {
          return *blocked;
        }
        const auto rechecked = confirmed_navigation_runtime_map_for_request(
          body, building_id, floor_id, map_id, map_name);
        if (rechecked && rechecked->map_id == already_selected->map_id) {
          return runtime_map_already_selected_response(*rechecked);
        }
      }
      return offline_runtime_busy_response(
        "offline map selection is allowed only after all motion runtimes stop: " + *busy);
    }

    std::optional<MapManifest> selected_map;
    if (map_id && !map_id->empty()) {
      selected_map = map_catalog_.find_map_by_id(*map_id);
      if (!selected_map) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (!safe_asset_id(building_id) || (floor_id && !safe_asset_id(*floor_id))) {
        return {
          400,
          "application/json",
          error_json("building_id/floor_id must be safe asset ids")};
      }
      if (json_string_value(body, "building_id") &&
        building_id != selected_map->building_id)
      {
        return {
          400,
          "application/json",
          error_json("map_id does not belong to requested building")};
      }
      if (floor_id && *floor_id != selected_map->floor_id) {
        return {
          400,
          "application/json",
          error_json("map_id does not belong to requested floor")};
      }
      building_id = selected_map->building_id;
      floor_id = selected_map->floor_id;
    } else {
      if (!floor_id || floor_id->empty()) {
        return {400, "application/json", error_json("floor_id is required")};
      }
      if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
        return {
          400,
          "application/json",
          error_json("building_id/floor_id must be safe asset ids")};
      }
      if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = map_catalog_.find_floor_map_by_name(
          building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {
            404,
            "application/json",
            error_json("map_name not found on requested floor: " + *map_name)};
        }
      } else {
        selected_map = map_catalog_.active_floor_map(building_id, *floor_id);
      }
    }

    if (!selected_map) {
      return {
        404,
        "application/json",
        error_json("no exact map asset is available for the requested floor")};
    }

    std::unique_ptr<MapAssetCommitTransaction> selection_transaction;
    try {
      selection_transaction =
        std::make_unique<MapAssetCommitTransaction>(config_.maps_root);
      std::string asset_error;
      if (!ports_.validate_map_manifest_assets(*selected_map, asset_error)) {
        return {409, "application/json", error_json(asset_error)};
      }
      selected_map = verify_map_asset_identity_snapshot(
        *selected_map,
        config_.maps_root,
        *selection_transaction).manifest;
    } catch (const std::exception & exception) {
      return {
        409,
        "application/json",
        error_json(
          std::string("exact floor map identity verification failed: ") +
          exception.what())};
    }

    if (!legacy_client_->wait_for_service(config_.service_timeout)) {
      return {
        503,
        "application/json",
        error_json("service unavailable: " + config_.legacy_service)};
    }
    if (const auto blocked = interlock_response("floor_switch_submit")) {
      return *blocked;
    }

    auto request = std::make_shared<robot_interfaces::srv::SwitchFloor::Request>();
    request->building_id = building_id;
    request->floor_id = *floor_id;
    request->map_id = selected_map->map_id;
    request->expected_asset_epoch = selected_map->asset_epoch;
    request->expected_asset_digest = selected_map->asset_digest;
    request->resume_navigation = resume_navigation;

    auto motion_admission = ports_.acquire_motion_admission(motion_admission_epoch);
    if (!motion_admission.admitted()) {
      return motion_admission_failure_response("floor_switch", motion_admission);
    }
    DelayedSideEffectEvidence pending_side_effect(ports_);
    auto future = legacy_client_->async_send_request(request);
    if (future.wait_for(config_.service_timeout) != std::future_status::ready) {
      return {
        503,
        "application/json",
        error_json("timed out waiting for floor switch")};
    }
    const auto response = future.get();
    pending_side_effect.resolve();

    if (response->success) {
      const bool exact_identity_confirmed =
        response->code == "OK" &&
        response->selected_building_id == selected_map->building_id &&
        response->selected_floor_id == selected_map->floor_id &&
        response->selected_map_id == selected_map->map_id &&
        response->asset_epoch == selected_map->asset_epoch &&
        response->asset_digest == selected_map->asset_digest &&
        same_normalized_path(response->nav_map_yaml, selected_map->nav_map_yaml) &&
        same_normalized_path(response->localizer_map_png, selected_map->localizer_map_png) &&
        same_normalized_path(
          response->localizer_params_yaml,
          selected_map->localizer_params_yaml);
      if (!exact_identity_confirmed) {
        return {
          409,
          "application/json",
          "{\"ok\":false,\"code\":\"FLOOR_SELECTION_IDENTITY_UNPROVEN\","
          "\"detail\":\"floor manager did not confirm the exact requested "
          "map identity and source paths\"}"};
      }

      std::lock_guard<std::mutex> activation_guard(activation_commit_mutex_);
      if (const auto blocked = interlock_response("floor_selection_commit")) {
        return *blocked;
      }
      if (const auto busy = offline_runtime_busy_detail()) {
        return offline_runtime_busy_response(
          "runtime became active while offline map selection was pending: " + *busy);
      }
      try {
        const auto commit_map = map_catalog_.find_map_by_id(selected_map->map_id);
        if (!commit_map || !same_exact_map_asset_source(*commit_map, *selected_map)) {
          return {
            409,
            "application/json",
            "{\"ok\":false,\"code\":\"FLOOR_SELECTION_SOURCE_DRIFT\","
            "\"detail\":\"map source identity changed after floor-manager preflight\"}"};
        }
        const auto commit_snapshot = verify_map_asset_identity_snapshot(
          *commit_map,
          config_.maps_root,
          *selection_transaction);
        if (!same_exact_map_asset_source(commit_snapshot.manifest, *selected_map)) {
          return {
            409,
            "application/json",
            "{\"ok\":false,\"code\":\"FLOOR_SELECTION_SOURCE_DRIFT\","
            "\"detail\":\"verified map identity changed after floor-manager preflight\"}"};
        }
        selected_map = commit_snapshot.manifest;
      } catch (const std::exception & exception) {
        std::ostringstream drift;
        drift << "{\"ok\":false,\"code\":\"FLOOR_SELECTION_SOURCE_DRIFT\","
              << "\"detail\":" << json_string(exception.what()) << "}";
        return {409, "application/json", drift.str()};
      }

      try {
        clear_runtime_context();
        ports_.activate_map_manifest(*selected_map, *selection_transaction);
      } catch (const ExactMapAssetSourceDrift & exception) {
        std::ostringstream drift;
        drift << "{\"ok\":false,\"code\":\"FLOOR_SELECTION_SOURCE_DRIFT\","
              << "\"detail\":" << json_string(exception.what()) << "}";
        return {409, "application/json", drift.str()};
      } catch (const std::exception & exception) {
        return {500, "application/json", error_json(exception.what())};
      }
    }

    std::ostringstream out;
    out << "{\"ok\":" << (response->success ? "true" : "false")
        << ",\"code\":" << json_string(response->code)
        << ",\"message\":" << json_string(response->message)
        << ",\"building_id\":" << json_string(response->selected_building_id)
        << ",\"floor_id\":" << json_string(response->selected_floor_id)
        << ",\"map_id\":" << json_string(response->selected_map_id)
        << ",\"asset_epoch\":" << response->asset_epoch
        << ",\"asset_digest\":" << json_string(response->asset_digest)
        << ",\"display_name\":" << json_string(selected_map->display_name)
        << ",\"nav_map_yaml\":" << json_string(response->nav_map_yaml)
        << ",\"localizer_map_png\":" << json_string(response->localizer_map_png)
        << ",\"localizer_params_yaml\":"
        << json_string(response->localizer_params_yaml) << "}";
    return {response->success ? 200 : 409, "application/json", out.str()};
  }

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  MapCatalog & map_catalog_;
  std::mutex & map_asset_mutation_mutex_;
  std::mutex & activation_commit_mutex_;
  FloorSwitchModuleConfig config_;
  FloorSwitchModulePorts ports_;

  mutable std::mutex interlock_mutex_;
  FloorRuntimeInterlock interlock_;
  rclcpp::Subscription<robot_interfaces::msg::FloorSwitchStatus>::SharedPtr
    transition_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizationHealth>::SharedPtr
    localization_health_sub_;
  rclcpp::Client<robot_interfaces::srv::SwitchFloor>::SharedPtr legacy_client_;
  rclcpp_action::Client<FloorSwitchAction>::SharedPtr live_client_;

  std::mutex start_mutex_;
  std::mutex worker_mutex_;
  std::mutex goal_mutex_;
  FloorSwitchHttpTransaction transaction_;
  std::atomic<std::uint64_t> sequence_{0U};
  std::atomic<bool> shutdown_requested_{false};
  std::thread worker_;
  FloorSwitchGoalHandle::SharedPtr goal_handle_;
  std::string goal_transaction_id_;
};

FloorSwitchModule::FloorSwitchModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  MapCatalog & map_catalog,
  std::mutex & map_asset_mutation_mutex,
  std::mutex & activation_commit_mutex,
  FloorSwitchModuleConfig config,
  FloorSwitchModulePorts ports)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      map_catalog,
      map_asset_mutation_mutex,
      activation_commit_mutex,
      std::move(config),
      std::move(ports)))
{
}

FloorSwitchModule::~FloorSwitchModule() = default;

std::optional<HttpResponse> FloorSwitchModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

bool FloorSwitchModule::interlock_enabled() const noexcept
{
  return impl_->interlock_enabled();
}

FloorRuntimeInterlockDecision FloorSwitchModule::interlock_decision() const
{
  return impl_->interlock_decision();
}

bool FloorSwitchModule::operation_blocked(
  const std::string & operation,
  std::string & detail,
  std::string * reason_code) const
{
  return impl_->operation_blocked(operation, detail, reason_code);
}

std::optional<HttpResponse> FloorSwitchModule::interlock_response(
  const std::string & operation) const
{
  return impl_->interlock_response(operation);
}

void FloorSwitchModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::floor_switch
