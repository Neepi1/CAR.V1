#include "robot_api_server/features/elevator/execution/elevator_ros_runtime_port.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/floor_switch.hpp"
#include "robot_interfaces/msg/correction_pause_state.hpp"
#include "robot_interfaces/msg/floor_switch_status.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/msg/localizer_asset_state.hpp"
#include "robot_interfaces/msg/motion_interlock_state.hpp"
#include "robot_interfaces/msg/operating_mode_state.hpp"
#include "robot_interfaces/srv/release_motion_hold_if_execution_idle.hpp"
#include "robot_interfaces/srv/set_correction_pause.hpp"
#include "robot_interfaces/srv/set_elevator_navigation_session.hpp"
#include "robot_interfaces/srv/set_mode.hpp"
#include "robot_interfaces/srv/set_motion_hold.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_api_server/features/elevator/execution/elevator_lease_keepalive.hpp"
#include "robot_api_server/features/elevator/execution/elevator_ros_executor.hpp"
#include "robot_api_server/features/elevator/execution/elevator_recovery_action_barrier.hpp"
#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_handoff_tracker.hpp"
#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_safety/persistent_sequence_allocator.hpp"

namespace robot_api_server
{
namespace
{

using namespace std::chrono_literals;
using RuntimeResult = robot_elevator_manager::ElevatorRuntimeResult;
using FailureDisposition =
  robot_elevator_manager::ElevatorRuntimeFailureDisposition;
using RuntimeEffect = robot_elevator_manager::ElevatorRuntimeEffect;
using CleanupContext =
  robot_elevator_manager::ElevatorRuntimeCleanupContext;
using ElevatorRuntimeFloorIdentity =
  robot_elevator_manager::ElevatorRuntimeFloorIdentity;
using CleanupDisposition =
  robot_elevator_manager::ElevatorCleanupDisposition;
using EffectKind = robot_elevator_manager::ElevatorEffectKind;
using FrozenRelease = robot_elevator_manager::FrozenElevatorRelease;
using FloorSwitch = robot_interfaces::action::FloorSwitch;
using FloorGoalHandle = rclcpp_action::ClientGoalHandle<FloorSwitch>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using ElevatorNavigationSession =
  robot_interfaces::srv::SetElevatorNavigationSession;
using RecoveryCancelResponse =
  action_msgs::srv::CancelGoal::Response::SharedPtr;

constexpr char kRuntimeOwner[] = "robot_elevator_manager";
constexpr char kFloorPauseOwner[] = "robot_floor_manager";
constexpr char kPauseHandoffStage[] = "CALLER_PAUSE_HANDOFF_READY";
constexpr char kVerifyPauseHandoffStage[] = "VERIFY_PAUSE_HANDOFF";

enum class ResourceOwnership
{
  kAbsent,
  kPresent,
  kUnknown,
};

RuntimeResult ok(const std::string & detail)
{
  return {true, "OK", detail};
}

RuntimeResult failed(const std::string & code, const std::string & detail)
{
  return {false, code, detail};
}

RuntimeResult rejected_before_effects(RuntimeResult result)
{
  result.failure_disposition = FailureDisposition::kRejectedBeforeEffects;
  return result;
}

RuntimeResult recovery_required_before_effects(RuntimeResult result)
{
  result.failure_disposition =
    FailureDisposition::kRecoveryRequiredBeforeEffects;
  return result;
}

RuntimeResult with_safety_proof(
  RuntimeResult result,
  const bool safety_hold_proven,
  const bool dual_odom_stop_proven = false)
{
  result.safety_hold_proven = safety_hold_proven;
  result.dual_odom_stop_proven = dual_odom_stop_proven;
  return result;
}

CleanupContext cleanup_context_for_release(
  const std::string & transaction_id,
  const FrozenRelease & release)
{
  CleanupContext context;
  context.transaction_id = transaction_id;
  context.building_id = release.building_id;
  context.source = {
    release.source.floor_id,
    release.source.map_id,
    release.source.map_asset_epoch,
    release.source.map_asset_digest,
  };
  context.target = {
    release.target.floor_id,
    release.target.map_id,
    release.target.map_asset_epoch,
    release.target.map_asset_digest,
  };
  context.floor_switch_action_may_have_been_submitted = false;
  return context;
}

std::optional<std::string> json_string_field(
  const std::string & json,
  const std::string & key)
{
  const auto marker = "\"" + key + "\":\"";
  const auto begin = json.find(marker);
  if (begin == std::string::npos) {
    return std::nullopt;
  }
  const auto value_begin = begin + marker.size();
  const auto end = json.find('"', value_begin);
  if (end == std::string::npos) {
    return std::nullopt;
  }
  return json.substr(value_begin, end - value_begin);
}

std::optional<bool> json_bool_field(
  const std::string & json,
  const std::string & key)
{
  const auto marker = "\"" + key + "\":";
  const auto begin = json.find(marker);
  if (begin == std::string::npos) {
    return std::nullopt;
  }
  const auto value_begin = begin + marker.size();
  if (json.compare(value_begin, 4U, "true") == 0) {
    return true;
  }
  if (json.compare(value_begin, 5U, "false") == 0) {
    return false;
  }
  return std::nullopt;
}

std::optional<std::uint64_t> json_uint64_field(
  const std::string & json,
  const std::string & key)
{
  const auto marker = "\"" + key + "\":";
  const auto begin = json.find(marker);
  if (begin == std::string::npos) {
    return std::nullopt;
  }
  const auto value_begin = begin + marker.size();
  std::uint64_t value = 0U;
  std::size_t consumed = 0U;
  try {
    value = std::stoull(json.substr(value_begin), &consumed);
  } catch (...) {
    return std::nullopt;
  }
  return consumed == 0U ? std::nullopt :
         std::optional<std::uint64_t>{value};
}

double steady_now_sec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Duration duration_message(const double seconds)
{
  builtin_interfaces::msg::Duration result;
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return result;
  }
  const auto whole = std::floor(seconds);
  result.sec = static_cast<std::int32_t>(whole);
  result.nanosec = static_cast<std::uint32_t>(
    std::floor((seconds - whole) * 1.0e9));
  return result;
}

bool contains_key(
  const std::vector<std::string> & keys,
  const std::string & key)
{
  return std::find(keys.cbegin(), keys.cend(), key) != keys.cend();
}

bool active_goal_status(const std::int8_t status)
{
  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
         status == action_msgs::msg::GoalStatus::STATUS_CANCELING;
}

ElevatorRecoveryGoalState recovery_goal_state(const std::int8_t status)
{
  if (active_goal_status(status)) {
    return ElevatorRecoveryGoalState::kActive;
  }
  if (
    status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
    status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
    status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
  {
    return ElevatorRecoveryGoalState::kTerminal;
  }
  return ElevatorRecoveryGoalState::kUnknown;
}

std::vector<ElevatorRecoveryGoalStatus> recovery_goal_statuses(
  const action_msgs::msg::GoalStatusArray & message)
{
  std::vector<ElevatorRecoveryGoalStatus> result;
  result.reserve(message.status_list.size());
  for (const auto & status : message.status_list) {
    result.push_back({
      status.goal_info.goal_id.uuid,
      recovery_goal_state(status.status),
    });
  }
  return result;
}

std::vector<ElevatorRecoveryGoalId> recovery_cancel_goal_ids(
  const action_msgs::srv::CancelGoal::Response & response)
{
  std::vector<ElevatorRecoveryGoalId> result;
  result.reserve(response.goals_canceling.size());
  for (const auto & goal : response.goals_canceling) {
    result.push_back(goal.goal_id.uuid);
  }
  return result;
}

template<typename GoalHandleT>
bool cancel_response_contains_goal(
  const action_msgs::srv::CancelGoal::Response & response,
  const std::shared_ptr<GoalHandleT> & goal_handle)
{
  if (!goal_handle) {
    return false;
  }
  const auto goal_id = goal_handle->get_goal_id();
  return std::any_of(
    response.goals_canceling.cbegin(),
    response.goals_canceling.cend(),
    [&goal_id](const action_msgs::msg::GoalInfo & info) {
      return std::equal(
        goal_id.cbegin(), goal_id.cend(), info.goal_id.uuid.cbegin());
    });
}

std::string result_code_text(const rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

class PrepareAdmissionFenceGuard
{
public:
  explicit PrepareAdmissionFenceGuard(
    std::shared_ptr<ElevatorMotionAdmissionFence> fence)
  : fence_(std::move(fence))
  {
    fence_->close_and_invalidate();
  }

  ~PrepareAdmissionFenceGuard()
  {
    if (!retain_closed_) {
      fence_->reopen_and_invalidate();
    }
  }

  void retain_closed() noexcept
  {
    retain_closed_ = true;
  }

private:
  std::shared_ptr<ElevatorMotionAdmissionFence> fence_;
  bool retain_closed_{false};
};

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> callback)
  : callback_(std::move(callback))
  {
  }

  ~ScopeExit()
  {
    if (callback_) {
      callback_();
    }
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&) = delete;
  ScopeExit & operator=(ScopeExit &&) = delete;

private:
  std::function<void()> callback_;
};

}  // namespace

class ElevatorRosRuntimePort::Implementation
{
public:
  explicit Implementation(ElevatorRosRuntimeOptions options)
  : options_(std::move(options))
  {
    if (!options_.motion_admission_fence) {
      options_.motion_admission_fence =
        std::make_shared<ElevatorMotionAdmissionFence>();
    }
    validate_options();
    hold_sequence_allocator_ =
      std::make_unique<robot_safety::PersistentSequenceAllocator>(
      options_.hold_sequence_state_file);
    node_ = std::make_shared<rclcpp::Node>("robot_elevator_runtime_adapter");
    hold_client_ = node_->create_client<robot_interfaces::srv::SetMotionHold>(
      options_.motion_hold_service);
    recovery_hold_release_client_ =
      node_->create_client<
      robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle>(
      options_.recovery_hold_release_service);
    mode_client_ = node_->create_client<robot_interfaces::srv::SetMode>(
      options_.mode_service);
    correction_client_ =
      node_->create_client<robot_interfaces::srv::SetCorrectionPause>(
      options_.correction_pause_service);
    for (const auto * controller_id : {
        "ElevatorHallFollowPath",
        "ElevatorFollowPath",
        "ElevatorReverseEntryStagingFollowPath",
        "ElevatorReverseDockingFollowPath",
        "ElevatorCabinEntryDirectFollowPath",
        "ElevatorCabinPanelFollowPath"})
    {
      const auto service_name =
        options_.elevator_controller_session_service_prefix + "/" +
        controller_id + "/execution_session";
      controller_session_clients_.emplace(
        controller_id,
        node_->create_client<ElevatorNavigationSession>(service_name));
    }
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, options_.navigate_to_pose_action);
    floor_client_ = rclcpp_action::create_client<FloorSwitch>(
      node_, options_.floor_switch_action);
    elevator_entry_collision_bypass_permit_pub_ =
      node_->create_publisher<std_msgs::msg::String>(
      options_.elevator_entry_collision_bypass_permit_topic,
      rclcpp::QoS(10).reliable());

    const auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    motion_allowed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      options_.motion_allowed_topic, state_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          have_motion_allowed_ = true;
          motion_allowed_ = message->data;
          motion_allowed_received_at_sec_ = steady_now_sec();
          ++motion_allowed_generation_;
        }
        evidence_cv_.notify_all();
      });
    safety_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      options_.safety_status_topic, state_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          have_safety_status_ = true;
          safety_status_ = message->data;
          safety_status_received_at_sec_ = steady_now_sec();
          ++safety_status_generation_;
        }
        evidence_cv_.notify_all();
      });
    interlock_sub_ =
      node_->create_subscription<robot_interfaces::msg::MotionInterlockState>(
      options_.motion_interlock_topic, state_qos,
      [this](
        const robot_interfaces::msg::MotionInterlockState::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          interlock_state_ = *message;
          have_interlock_state_ = true;
          interlock_received_at_sec_ = steady_now_sec();
          ++interlock_observation_generation_;
        }
        evidence_cv_.notify_all();
      });
    mode_sub_ =
      node_->create_subscription<robot_interfaces::msg::OperatingModeState>(
      options_.operating_mode_topic, state_qos,
      [this](
        const robot_interfaces::msg::OperatingModeState::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          operating_mode_state_ = *message;
          have_operating_mode_state_ = true;
          operating_mode_received_at_sec_ = steady_now_sec();
          ++operating_mode_observation_generation_;
        }
        evidence_cv_.notify_all();
      });
    correction_sub_ =
      node_->create_subscription<robot_interfaces::msg::CorrectionPauseState>(
      options_.correction_pause_topic, state_qos,
      [this](
        const robot_interfaces::msg::CorrectionPauseState::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          correction_pause_state_ = *message;
          have_correction_pause_state_ = true;
          correction_pause_received_at_sec_ = steady_now_sec();
          ++correction_pause_observation_generation_;
        }
        evidence_cv_.notify_all();
      });
    floor_status_sub_ =
      node_->create_subscription<robot_interfaces::msg::FloorSwitchStatus>(
      options_.floor_switch_status_topic, state_qos,
      [this](
        const robot_interfaces::msg::FloorSwitchStatus::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          floor_switch_status_ = *message;
          have_floor_switch_status_ = true;
          floor_switch_status_received_at_sec_ = steady_now_sec();
        }
        if (
          message->stage == kPauseHandoffStage ||
          message->stage == kVerifyPauseHandoffStage)
        {
          std::lock_guard<std::mutex> lock(floor_goal_mutex_);
          const auto handoff = floor_handoff_tracker_.snapshot();
          if (
            handoff.active &&
            handoff.transaction_id == message->transaction_id)
          {
            (void)floor_handoff_tracker_.observe_feedback(
              message->transaction_id,
              handoff.submission_generation,
              handoff.accepted_stage_sequence,
              true);
          }
        }
        evidence_cv_.notify_all();
        floor_goal_cv_.notify_all();
      });
    navigation_status_sub_ =
      node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      options_.navigation_status_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr message) {
        bool active = false;
        for (const auto & status : message->status_list) {
          if (active_goal_status(status.status)) {
            active = true;
            break;
          }
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          have_navigation_status_ = true;
          navigation_goal_active_ = active;
          navigation_status_received_at_sec_ = steady_now_sec();
          recovery_nav_action_barrier_.observe_post_response_status(
            recovery_goal_statuses(*message));
        }
        evidence_cv_.notify_all();
      });
    floor_action_status_sub_ =
      node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      options_.floor_switch_action_status_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          recovery_floor_action_barrier_.observe_post_response_status(
            recovery_goal_statuses(*message));
        }
        evidence_cv_.notify_all();
      });
    localizer_asset_state_sub_ =
      node_->create_subscription<robot_interfaces::msg::LocalizerAssetState>(
      options_.localizer_asset_state_topic, state_qos,
      [this](
        const robot_interfaces::msg::LocalizerAssetState::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          localizer_asset_state_ = *message;
          have_localizer_asset_state_ = true;
        }
        evidence_cv_.notify_all();
      });
    localization_health_sub_ =
      node_->create_subscription<robot_interfaces::msg::LocalizationHealth>(
      options_.localization_health_topic, state_qos,
      [this](
        const robot_interfaces::msg::LocalizationHealth::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          localization_health_ = *message;
          have_localization_health_ = true;
          localization_health_received_at_sec_ = steady_now_sec();
        }
        evidence_cv_.notify_all();
      });
    localization_bridge_status_sub_ =
      node_->create_subscription<std_msgs::msg::String>(
      options_.localization_bridge_status_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          localization_bridge_status_ = message->data;
          have_localization_bridge_status_ = true;
          localization_bridge_status_received_at_sec_ = steady_now_sec();
        }
        evidence_cv_.notify_all();
      });
    wheel_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      options_.wheel_odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        stop_tracker_.observe_wheel(
          message->twist.twist.linear.x,
          message->twist.twist.linear.y,
          message->twist.twist.angular.z,
          steady_now_sec());
        evidence_cv_.notify_all();
      });
    local_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      options_.local_odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        stop_tracker_.observe_local(
          message->twist.twist.linear.x,
          message->twist.twist.linear.y,
          message->twist.twist.angular.z,
          steady_now_sec());
        evidence_cv_.notify_all();
      });

    const auto lease_period = [](const double lease_duration_sec) {
        return std::max(
          1ms,
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(
              std::min(0.75, lease_duration_sec / 3.0))));
      };
    mode_keepalive_ = std::make_unique<ElevatorLeaseKeepalive>(
      lease_period(options_.mode_lease_sec),
      [this]() {
        const auto result = renew_mode_lease_once();
        return ElevatorLeaseKeepaliveStatus{
          result.success, result.code, result.detail};
      },
      [this](const ElevatorLeaseKeepaliveStatus & failure) {
        handle_mode_keepalive_failure(failure);
      });

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = node_->get_node_base_interface()->get_context();
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(
      executor_options);
    executor_->add_node(node_);
    ros_worker_ = std::make_unique<ElevatorRosExecutor>(
      *executor_, executor_options.context,
      [this]() {evidence_cv_.notify_all(); floor_goal_cv_.notify_all();},
      [this](uint64_t count, uint64_t consecutive) {
        RCLCPP_ERROR(node_->get_logger(),
          "ELEVATOR_ACTION_NO_READY_EVENT count=%llu consecutive=%llu "
          "node=robot_elevator_runtime_adapter; retained clients/goals; "
          "event retry is not proof of action progress",
          static_cast<unsigned long long>(count),
          static_cast<unsigned long long>(consecutive));
      });
    ros_worker_->start();
  }

  ~Implementation()
  {
    // The execution module joins its business worker before releasing this
    // shared port. Wake the remaining keepalive waits before either join.
    if (ros_worker_) {ros_worker_->request_stop();}
    if (elevator_entry_collision_bypass_permit_pub_) {
      try {publish_elevator_entry_collision_bypass_permit("");}
      catch (...) {std::fputs("elevator adapter: shutdown permit publish failed\n", stderr);}
    }
    if (mode_keepalive_) {mode_keepalive_->stop();}
    if (ros_worker_) {ros_worker_->stop();}
    if (executor_ && node_) {
      try {executor_->remove_node(node_);}
      catch (...) {std::fputs("elevator adapter: node removal failed\n", stderr);}
    }
  }

  template<typename Operation>
  RuntimeResult invoke(Operation operation)
  {
    try {
      ros_worker_->check();
      return operation();
    } catch (const ElevatorRosExecutorUnavailable & error) {
      return failed("ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", error.what());
    }
  }

  template<typename Probe>
  bool wait_endpoint(Probe probe, std::chrono::nanoseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    for (;;) {
      ros_worker_->check();
      const auto left = deadline - std::chrono::steady_clock::now();
      const auto slice = std::min(
        std::chrono::duration_cast<std::chrono::nanoseconds>(left),
        std::chrono::duration_cast<std::chrono::nanoseconds>(50ms));
      if (probe(std::max(slice, std::chrono::nanoseconds::zero()))) {
        ros_worker_->check();
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {return false;}
    }
  }

  robot_elevator_manager::ElevatorRuntimeCapabilities capabilities()
  const noexcept
  {
    robot_elevator_manager::ElevatorRuntimeCapabilities result;
    try {
      if (
        ros_worker_->unavailable() || !node_ ||
        !options_.runtime_idle_probe)
      {
        return result;
      }
      // Capabilities describe the deployed adapter, not a transient DDS graph
      // sample taken milliseconds after process startup. prepare() performs
      // bounded endpoint discovery and returns the exact missing endpoint.
      result.motion_authorized = true;
      result.floor_switch_capable = true;
      result.runtime_capable = true;
      result.automatic_button_control =
        options_.arm_button_control_enabled &&
        static_cast<bool>(options_.arm_client);
    } catch (...) {
      return {};
    }
    return result;
  }

  RuntimeResult prepare(
    const std::string & transaction_id,
    const FrozenRelease & release)
  {
    if (transaction_id.empty()) {
      return rejected_before_effects(
        failed(
          "ELEVATOR_RUNTIME_INVALID_TRANSACTION",
          "transaction_id is required"));
    }
    publish_elevator_entry_collision_bypass_permit("");
    PrepareAdmissionFenceGuard admission_fence_guard(
      options_.motion_admission_fence);
    const auto recovery_required = [this, &admission_fence_guard](
        RuntimeResult result) {
        if (options_.persistent_recovery_lock_enabled) {
          admission_fence_guard.retain_closed();
        }
        return recovery_required_before_effects(std::move(result));
      };
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      if (prepared_ && transaction_id_ != transaction_id) {
        if (!terminal_) {
          return recovery_required(
            failed(
              "ELEVATOR_RUNTIME_TRANSACTION_CONFLICT",
              "another elevator runtime transaction is bound"));
        }
        if (hold_active_) {
          return recovery_required(
            failed(
              "ELEVATOR_RUNTIME_RECOVERY_REQUIRED",
              "the previous terminal transaction intentionally retained its "
              "owner safety hold"));
        }
      }
      if (
        mode_lease_ownership_ != ResourceOwnership::kAbsent ||
        correction_pause_ownership_ != ResourceOwnership::kAbsent)
      {
        return recovery_required(
          failed(
            "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN",
            "a previous mode or correction-pause resource is "
            "present or has an unknown outcome"));
      }
    }
    if (
      recovery_nav_cancel_all_response_unknown_.load() ||
      recovery_floor_cancel_all_response_unknown_.load())
    {
      return recovery_required(
        failed(
          "ELEVATOR_RECOVERY_CANCEL_RESPONSE_UNKNOWN",
          "a previous recovery cancel-all request has no proven response; "
          "a full runtime restart is required before motion can resume"));
    }
    {
      std::lock_guard<std::mutex> lock(recovery_cancel_mutex_);
      if (
        recovery_nav_cancel_all_response_.pending() ||
        recovery_floor_cancel_all_response_.pending())
      {
        return recovery_required(
          failed(
            "ELEVATOR_RECOVERY_CANCEL_RESPONSE_PENDING",
            "a previously submitted recovery cancel-all request is still "
            "awaiting its response"));
      }
    }
    {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      if (active_nav_goal_ || nav_goal_admission_unknown_) {
        return recovery_required(
          failed(
            "ELEVATOR_NAV2_TERMINAL_UNPROVEN",
            "a previous Nav2 goal has no proven terminal result"));
      }
    }
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      if (active_floor_goal_ || floor_goal_admission_unknown_) {
        return recovery_required(
          failed(
            "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
            "a previous floor-switch goal has no proven terminal result"));
      }
    }
    const auto endpoint_result = wait_for_endpoints();
    if (!endpoint_result.success) {
      return rejected_before_effects(endpoint_result);
    }
    const auto navigation_idle_result = wait_for_navigation_idle_evidence();
    if (!navigation_idle_result.success) {
      return rejected_before_effects(navigation_idle_result);
    }
    if (!options_.delayed_side_effect_unknown_probe) {
      return rejected_before_effects(
        failed(
          "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE",
          "timed-out side-effect evidence probe is not configured"));
    }
    std::uint64_t delayed_side_effect_unknown_count = 0U;
    try {
      delayed_side_effect_unknown_count =
        options_.delayed_side_effect_unknown_probe();
    } catch (const std::exception & exception) {
      return rejected_before_effects(
        failed(
          "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE", exception.what()));
    } catch (...) {
      return rejected_before_effects(
        failed(
          "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE",
          "timed-out side-effect evidence probe threw an unknown exception"));
    }
    if (delayed_side_effect_unknown_count != 0U) {
      return rejected_before_effects(
        failed(
          "DELAYED_SIDE_EFFECT_UNKNOWN",
          std::to_string(delayed_side_effect_unknown_count) +
          " timed-out motion/runtime submission(s) remain unresolved"));
    }
    const auto recovery_hold_result =
      wait_for_no_foreign_elevator_hold(transaction_id);
    if (!recovery_hold_result.success) {
      return recovery_required(recovery_hold_result);
    }

    std::array<bool, 3> idle{};
    try {
      idle = options_.runtime_idle_probe();
    } catch (const std::exception & exception) {
      return rejected_before_effects(
        failed(
          "ELEVATOR_SOURCE_RUNTIME_PROBE_FAILED", exception.what()));
    } catch (...) {
      return rejected_before_effects(
        failed(
          "ELEVATOR_SOURCE_RUNTIME_PROBE_FAILED",
          "runtime idle probe threw an unknown exception"));
    }

    ElevatorPreparedSourceEvidence evidence;
    const auto context =
      read_runtime_map_context_file(options_.runtime_map_context_file);
    if (context) {
      evidence.runtime_context_confirmed = context->confirmed;
      evidence.runtime_context_state = context->state;
      evidence.building_id = context->building_id;
      evidence.floor_id = context->floor_id;
      evidence.map_id = context->map_id;
      evidence.asset_epoch = context->asset_epoch;
      evidence.asset_digest = context->asset_digest;
    }
    try {
      MapCatalog catalog(options_.maps_root);
      const auto verify_floor =
        [this, &catalog, &release](
        const robot_elevator_manager::ElevatorRuntimeFloor & floor,
        const std::string & label) {
          const auto manifests = catalog.read_floor_map_manifests(
            release.building_id,
            floor.floor_id, false);
          const auto found = std::find_if(
            manifests.cbegin(), manifests.cend(),
            [&floor](const MapManifest & manifest) {
              return manifest.map_id == floor.map_id;
            });
          if (found == manifests.cend()) {
            throw std::runtime_error(
                    label + " map manifest is not present in the managed catalog");
          }
          const auto snapshot =
            verify_map_asset_identity_snapshot(*found, options_.maps_root);
          if (
            snapshot.manifest.asset_epoch != floor.map_asset_epoch ||
            snapshot.manifest.asset_digest != floor.map_asset_digest)
          {
            throw std::runtime_error(
                    label + " map identity differs from the frozen release");
          }
          return snapshot.manifest;
        };
      const auto source_manifest = verify_floor(release.source, "source");
      (void)verify_floor(release.target, "target");
      (void)source_manifest;
    } catch (const std::exception & exception) {
      return rejected_before_effects(
        failed(
          "ELEVATOR_ASSET_PREFLIGHT_FAILED", exception.what()));
    }
    // wait_for_navigation_idle_evidence() already bounded both the latched
    // action status (when available) and the authoritative API runtime. A
    // status topic is allowed to be silent while idle, so do not turn that
    // successful no-status proof back into a failure here.
    evidence.navigation_idle = idle[0];
    evidence.mapping_idle = idle[1];
    evidence.docking_idle = idle[2];
    const auto verified = validate_elevator_prepared_source(release, evidence);
    if (!verified.success) {
      return rejected_before_effects(verified);
    }
    const auto live_source = validate_live_source_identity(release);
    if (!live_source.success) {
      return rejected_before_effects(live_source);
    }
    if (options_.arm_button_control_enabled) {
      if (!ElevatorArmClient::floor_button_label(release.source.floor_id)) {
        return rejected_before_effects(
          failed(
            "ELEVATOR_ARM_SOURCE_FLOOR_UNSUPPORTED",
            "source floor is not one of -2, -1, or 1 through 20"));
      }
      if (!ElevatorArmClient::floor_button_label(release.target.floor_id)) {
        return rejected_before_effects(
          failed(
            "ELEVATOR_ARM_TARGET_FLOOR_UNSUPPORTED",
            "target floor is not one of -2, -1, or 1 through 20"));
      }
      if (!ElevatorArmClient::hall_call_direction(
          release.source.floor_id, release.target.floor_id))
      {
        return rejected_before_effects(
          failed(
            "ELEVATOR_ARM_HALL_CALL_DIRECTION_UNRESOLVED",
            "source and target floors must resolve to different numeric levels"));
      }
    }

    // A previous transaction with proven-absent resources must not leave a
    // joinable mode keepalive worker or a latched renewal failure in the new
    // bind.
    mode_keepalive_->stop();
    mode_keepalive_->reset();

    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      if (prepared_ && transaction_id_ != transaction_id && !terminal_) {
        return recovery_required(
          failed(
            "ELEVATOR_RUNTIME_TRANSACTION_CONFLICT",
            "another elevator runtime transaction is bound"));
      }
      transaction_id_ = transaction_id;
      mission_id_ = "elevator_" + transaction_id;
      mode_lease_id_ = "mode_" + transaction_id;
      mode_lease_revision_ = 0U;
      release_ = release;
      cleanup_context_ = cleanup_context_for_release(transaction_id, release);
      prepared_ = true;
      terminal_ = false;
      restart_cleanup_runtime_superseded_ = false;
      cancel_requested_.store(false);
      hold_active_ = false;
      mode_lease_ownership_ = ResourceOwnership::kAbsent;
      mode_.clear();
      correction_pause_ownership_ = ResourceOwnership::kAbsent;
      floor_result_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      active_nav_goal_.reset();
      nav_goal_response_future_ = {};
      nav_result_future_ = {};
      nav_goal_admission_unknown_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      active_floor_goal_.reset();
      floor_goal_response_future_ = {};
      floor_result_future_ = {};
      floor_goal_admission_unknown_ = false;
      floor_handoff_tracker_.clear();
    }
    reset_stop_evidence();
    admission_fence_guard.retain_closed();
    return ok("exact frozen elevator release bound to ready source runtime");
  }

  RuntimeResult apply(const RuntimeEffect & effect)
  {
    const auto binding_result = validate_effect_binding(effect);
    if (!binding_result.success) {
      return binding_result;
    }
    if (
      cancel_requested_.load() &&
      effect.effect.kind != EffectKind::kAcquireSafetyHold &&
      effect.effect.kind != EffectKind::kHoldAndCancel)
    {
      return failed(
        "ELEVATOR_CANCEL_FENCE_ACTIVE",
        "cancellation forbids starting another automatic runtime effect");
    }
    const auto lease_gate = renew_before_state_changing_effect(
      effect.effect.kind);
    if (!lease_gate.success) {
      return lease_gate;
    }
    switch (effect.effect.kind) {
      case EffectKind::kNavigateToPose:
        return navigate(effect);
      case EffectKind::kAcquireSafetyHold:
        return set_hold(true, effect.effect.detail);
      case EffectKind::kReleaseSafetyHold:
        return set_hold(false, effect.effect.detail);
      case EffectKind::kAcquireExecutionLease:
      case EffectKind::kReleaseExecutionLease:
        return failed(
          "ELEVATOR_EXECUTION_LEASE_EFFECT_DISABLED",
          "elevator-test no longer creates or releases an execution lease");
      case EffectKind::kSetOperatingMode:
        return set_mode(effect.effect.mode);
      case EffectKind::kReleaseOperatingMode:
        return release_mode();
      case EffectKind::kPauseLocalizationCorrections:
        return set_correction_pause(true, effect.effect.detail);
      case EffectKind::kResumeLocalizationCorrections:
        return set_correction_pause(false, effect.effect.detail, true);
      case EffectKind::kBeginFloorTransition:
        {
          std::lock_guard<std::mutex> lock(binding_mutex_);
          cleanup_context_.floor_switch_action_may_have_been_submitted = true;
        }
        return begin_floor_switch(effect);
      case EffectKind::kSwitchFloor:
        return await_floor_switch(effect);
      case EffectKind::kVerifyFloorReady:
        return verify_floor_ready(effect);
      case EffectKind::kComplete:
      {
        {
          std::lock_guard<std::mutex> lock(binding_mutex_);
          if (
            hold_active_ ||
            mode_lease_ownership_ != ResourceOwnership::kAbsent ||
            correction_pause_ownership_ != ResourceOwnership::kAbsent)
          {
            return failed(
              "ELEVATOR_RUNTIME_RESOURCES_NOT_RELEASED",
              "completion requires hold, operating mode, "
              "and caller correction pause to be released");
          }
        }
        {
          std::lock_guard<std::mutex> lock(nav_goal_mutex_);
          if (active_nav_goal_ || nav_goal_admission_unknown_) {
            return failed(
              "ELEVATOR_NAV2_TERMINAL_UNPROVEN",
              "completion requires no active or admission-unknown Nav2 goal");
          }
        }
        {
          std::lock_guard<std::mutex> lock(floor_goal_mutex_);
          if (active_floor_goal_ || floor_goal_admission_unknown_) {
            return failed(
              "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
              "completion requires no active or admission-unknown floor goal");
          }
        }
        {
          std::lock_guard<std::mutex> lock(binding_mutex_);
          terminal_ = true;
        }
        options_.motion_admission_fence->reopen_and_invalidate();
        return ok("elevator runtime transaction completed");
      }
      case EffectKind::kHoldAndCancel:
        return hold_and_cancel(effect);
      case EffectKind::kMockPressCallButton:
        return apply_arm_button(effect, true);
      case EffectKind::kMockPressTargetButton:
        return apply_arm_button(effect, false);
      case EffectKind::kMockWaitDoorOpen:
      case EffectKind::kMockRide:
        return failed(
          "ELEVATOR_MANUAL_GATE_REACHED_RUNTIME_PORT",
          "manual confirmation effects must be consumed by the execution module");
      case EffectKind::kNone:
        return failed(
          "ELEVATOR_RUNTIME_INVALID_EFFECT", "NONE is not executable");
    }
    return failed(
      "ELEVATOR_RUNTIME_INVALID_EFFECT", "unknown elevator runtime effect");
  }

  RuntimeResult recover_locked(const CleanupContext & context)
  {
    const auto & transaction_id = context.transaction_id;
    if (transaction_id.empty()) {
      return failed(
        "ELEVATOR_RUNTIME_INVALID_TRANSACTION",
          "restart recovery requires transaction_id");
    }
    options_.motion_admission_fence->close_and_invalidate();
    ScopeExit nonpersistent_fence_release([this]() {
        if (!options_.persistent_recovery_lock_enabled) {
          options_.motion_admission_fence->reopen_and_invalidate();
        }
      });
    {
      std::lock_guard<std::mutex> submission_lock(submission_mutex_);
      std::lock_guard<std::mutex> binding_lock(binding_mutex_);
      if (prepared_ && transaction_id_ != transaction_id) {
        return failed(
          "ELEVATOR_RUNTIME_TRANSACTION_CONFLICT",
          "restart recovery cannot replace another bound transaction");
      }
      if (
        prepared_ && !cleanup_context_.transaction_id.empty() &&
        !elevator_cleanup_context_rebind_allowed(cleanup_context_, context))
      {
        return failed(
          "ELEVATOR_RECOVERY_CONTEXT_MISMATCH",
          "restart recovery context differs from the bound transaction");
      }
      if (!prepared_) {
        transaction_id_ = transaction_id;
        mission_id_ = "elevator_" + transaction_id;
        mode_lease_id_ = "mode_" + transaction_id;
        mode_lease_revision_ = 0U;
        prepared_ = true;
        terminal_ = true;
        restart_cleanup_runtime_superseded_ = false;
        hold_active_ = false;
        mode_lease_ownership_ = ResourceOwnership::kAbsent;
        mode_.clear();
        correction_pause_ownership_ = ResourceOwnership::kAbsent;
      }
      cleanup_context_ = context;
      cancel_requested_.store(true);
    }

    const auto safety_endpoints = wait_for_recovery_safety_endpoints();
    if (!safety_endpoints.success) {
      return safety_endpoints;
    }
    const auto hold_result = set_hold(true, "restart_locked_recovery");
    if (!hold_result.success) {
      return hold_result;
    }
    // Nav2 and floor actions start later than robot_safety in the resident
    // runtime.  Retain the exact owner hold before waiting for those actions so
    // an early recovery probe cannot leave the journal locked while the
    // physical safety hold is absent.
    const bool require_floor_action =
      context.floor_switch_action_may_have_been_submitted;
    const auto action_endpoints =
      wait_for_recovery_action_endpoints(require_floor_action);
    if (!action_endpoints.success) {
      return with_safety_proof(action_endpoints, true, false);
    }
    const auto correction_release = set_correction_pause(
      false, "restart_locked_recovery", false);
    if (!correction_release.success) {
      return with_safety_proof(correction_release, true, false);
    }
    const auto mode_release = release_mode();
    if (!mode_release.success) {
      return with_safety_proof(mode_release, true, false);
    }
    begin_recovery_cancel_barrier(require_floor_action);
    const auto nav_cancel = cancel_all_nav_goals_for_recovery();
    if (!nav_cancel.success) {
      return with_safety_proof(nav_cancel, true, false);
    }
    if (require_floor_action) {
      const auto floor_cancel = cancel_all_floor_goals_for_recovery();
      if (!floor_cancel.success) {
        return with_safety_proof(floor_cancel, true, false);
      }
    }
    const auto idle_result =
      wait_for_restart_action_idle(require_floor_action);
    if (!idle_result.success) {
      return with_safety_proof(idle_result, true, false);
    }
    const auto runtime_idle_result = wait_for_all_runtimes_idle();
    if (!runtime_idle_result.success) {
      return with_safety_proof(runtime_idle_result, true, false);
    }
    const auto delayed_effects = prove_no_delayed_side_effect_unknown();
    if (!delayed_effects.success) {
      return with_safety_proof(delayed_effects, true, false);
    }
    const bool safe_outside =
      context.disposition == CleanupDisposition::kSourceOutside ||
      context.disposition == CleanupDisposition::kTargetOutside;
    if (safe_outside) {
      const auto resource_result =
        wait_for_recovery_resource_absence(transaction_id);
      if (!resource_result.success) {
        return with_safety_proof(resource_result, true, false);
      }
      bool runtime_superseded = false;
      const auto runtime_identity = validate_live_cleanup_identity(
        context, !options_.persistent_recovery_lock_enabled,
        &runtime_superseded);
      if (!runtime_identity.success) {
        return with_safety_proof(runtime_identity, true, false);
      }
      {
        std::lock_guard<std::mutex> lock(binding_mutex_);
        if (prepared_ && transaction_id_ == transaction_id) {
          restart_cleanup_runtime_superseded_ = runtime_superseded;
        }
      }
    }
    reset_stop_evidence();
    const auto stop_result = wait_for_stop();
    if (!stop_result.success) {
      return with_safety_proof(stop_result, true, false);
    }
    const auto hold_still_proven = wait_for_owner_hold_evidence();
    if (!hold_still_proven.success) {
      return with_safety_proof(hold_still_proven, false, true);
    }
    auto recovered = with_safety_proof(
      ok(
        "restart recovery retained the sequenced owner hold, canceled unknown "
        "actions, and proved dual odometry stopped"),
      true, true);
    recovered.runtime_resources_reconciled = safe_outside;
    return recovered;
  }

  RuntimeResult finalize_recovery(const CleanupContext & context)
  {
    const auto & transaction_id = context.transaction_id;
    if (transaction_id.empty()) {
      return failed(
        "ELEVATOR_RUNTIME_INVALID_TRANSACTION",
        "recovery finalization requires transaction_id");
    }
    bool allow_ready_runtime_supersession = false;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      if (
        !prepared_ || transaction_id_ != transaction_id || !terminal_ ||
        !hold_active_ ||
        !elevator_cleanup_context_equal(cleanup_context_, context))
      {
        return failed(
          "ELEVATOR_RECOVERY_BINDING_UNPROVEN",
          "exact terminal recovery binding and owner hold are required");
      }
      if (
        (!context.legacy_preflight_orphan &&
        context.disposition != CleanupDisposition::kSourceOutside &&
        context.disposition != CleanupDisposition::kTargetOutside) ||
        mode_lease_ownership_ != ResourceOwnership::kAbsent ||
        correction_pause_ownership_ != ResourceOwnership::kAbsent)
      {
        return failed(
          "ELEVATOR_RECOVERY_NOT_PREFLIGHT_ORPHAN",
          "recovery finalization requires a safe outside disposition and no "
          "execution, mode, or correction-pause resource");
      }
      allow_ready_runtime_supersession =
        restart_cleanup_runtime_superseded_;
    }

    const bool require_floor_action =
      context.floor_switch_action_may_have_been_submitted;
    const auto endpoints = wait_for_recovery_endpoints(require_floor_action);
    if (!endpoints.success) {
      return with_safety_proof(endpoints, true, false);
    }
    begin_recovery_cancel_barrier(require_floor_action);
    const auto nav_cancel = cancel_all_nav_goals_for_recovery();
    if (!nav_cancel.success) {
      return with_safety_proof(nav_cancel, true, false);
    }
    if (require_floor_action) {
      const auto floor_cancel = cancel_all_floor_goals_for_recovery();
      if (!floor_cancel.success) {
        return with_safety_proof(floor_cancel, true, false);
      }
    }
    const auto action_idle =
      wait_for_restart_action_idle(require_floor_action);
    if (!action_idle.success) {
      return with_safety_proof(action_idle, true, false);
    }
    const auto runtime_idle = wait_for_all_runtimes_idle();
    if (!runtime_idle.success) {
      return with_safety_proof(runtime_idle, true, false);
    }
    const auto resource_absence =
      wait_for_recovery_resource_absence(transaction_id);
    if (!resource_absence.success) {
      return with_safety_proof(resource_absence, true, false);
    }
    const auto runtime_identity = validate_live_cleanup_identity(
      context, allow_ready_runtime_supersession);
    if (!runtime_identity.success) {
      return with_safety_proof(runtime_identity, true, false);
    }
    const auto no_foreign_hold =
      wait_for_no_foreign_elevator_hold(transaction_id);
    if (!no_foreign_hold.success) {
      return with_safety_proof(no_foreign_hold, true, false);
    }
    reset_stop_evidence();
    const auto stop_result = wait_for_stop();
    if (!stop_result.success) {
      return with_safety_proof(stop_result, true, false);
    }
    // Re-check every non-action runtime and owner resource after the bounded
    // stop observation. The action cancel barrier is intentionally last so no
    // older action-status sample can bridge the irreversible hold release.
    const auto final_runtime_idle = wait_for_all_runtimes_idle();
    if (!final_runtime_idle.success) {
      return with_safety_proof(final_runtime_idle, true, true);
    }
    std::uint64_t expected_interlock_generation = 0U;
    const auto final_resource_absence =
      wait_for_recovery_resource_absence(
      transaction_id, &expected_interlock_generation);
    if (!final_resource_absence.success) {
      return with_safety_proof(final_resource_absence, true, true);
    }
    const auto final_runtime_identity =
      validate_live_cleanup_identity(context, allow_ready_runtime_supersession);
    if (!final_runtime_identity.success) {
      return with_safety_proof(final_runtime_identity, true, true);
    }
    const auto final_no_foreign_hold =
      wait_for_no_foreign_elevator_hold(transaction_id);
    if (!final_no_foreign_hold.success) {
      return with_safety_proof(final_no_foreign_hold, true, true);
    }
    const auto hold_result = wait_for_owner_hold_evidence();
    if (!hold_result.success) {
      return with_safety_proof(hold_result, false, true);
    }
    // Invalidate every request admitted before recovery, then keep the shared
    // fence closed through the last cancel barrier and the conditional hold
    // release. A request already inside this fence completes before the final
    // cancel round; a waiting old request observes a stale epoch afterwards.
    auto motion_admission_lock =
      options_.motion_admission_fence->invalidate_pending_and_lock();
    begin_recovery_cancel_barrier(require_floor_action);
    const auto final_nav_cancel = cancel_all_nav_goals_for_recovery();
    if (!final_nav_cancel.success) {
      return with_safety_proof(final_nav_cancel, true, true);
    }
    if (require_floor_action) {
      const auto final_floor_cancel = cancel_all_floor_goals_for_recovery();
      if (!final_floor_cancel.success) {
        return with_safety_proof(final_floor_cancel, true, true);
      }
    }
    const auto final_action_idle =
      wait_for_restart_action_idle(require_floor_action);
    if (!final_action_idle.success) {
      return with_safety_proof(final_action_idle, true, true);
    }
    const auto final_delayed_effects = prove_no_delayed_side_effect_unknown();
    if (!final_delayed_effects.success) {
      return with_safety_proof(final_delayed_effects, true, true);
    }
    std::lock_guard<std::mutex> submission_lock(submission_mutex_);
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      if (
        !prepared_ || transaction_id_ != transaction_id || !terminal_ ||
        !hold_active_ ||
        !elevator_cleanup_context_equal(cleanup_context_, context) ||
        mode_lease_ownership_ != ResourceOwnership::kAbsent ||
        correction_pause_ownership_ != ResourceOwnership::kAbsent)
      {
        return with_safety_proof(
          failed(
            "ELEVATOR_RECOVERY_BINDING_CHANGED",
            "recovery binding changed before hold release"),
          true, true);
      }
    }

    auto request = std::make_shared<
      robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle::Request>();
    const auto command_sequence = next_hold_command_sequence();
    if (!command_sequence) {
      return with_safety_proof(
        failed(
          "ELEVATOR_SAFETY_HOLD_SEQUENCE_EXHAUSTED",
          "motion-hold command sequence is exhausted"),
        true, true);
    }
    request->owner = kRuntimeOwner;
    request->transaction_id = transaction_id;
    request->reason = context.legacy_preflight_orphan ?
      "explicit_preflight_orphan_recovery" :
      "phase_aware_elevator_cleanup";
    request->command_sequence = *command_sequence;
    request->expected_generation = expected_interlock_generation;
    const auto response = call_service<
      robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle>(
      recovery_hold_release_client_, request);
    if (!response) {
      return with_safety_proof(
        failed(
          "ELEVATOR_RECOVERY_HOLD_RELEASE_TIMEOUT",
          "motion hold service did not respond"),
        false, true);
    }
    const bool own_key =
      contains_key(response->state.hold_keys, hold_key(transaction_id));
    if (
      !response->success &&
      response->result_code ==
      robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle::Response::
      RESULT_STALE_COMMAND)
    {
      synchronize_hold_command_sequence(response->applied_sequence);
      return with_safety_proof(
        failed(
          "ELEVATOR_SAFETY_HOLD_SEQUENCE_STALE",
          response->message + "; requested_sequence=" +
          std::to_string(*command_sequence) + "; applied_sequence=" +
          std::to_string(response->applied_sequence)),
        own_key, true);
    }
    const bool execution_absent_in_response =
      !response->state.execution_session_engaged &&
      !response->state.execution_lease_active;
    const bool generation_advanced_exactly_once =
      expected_interlock_generation !=
      std::numeric_limits<std::uint64_t>::max() &&
      response->state.generation == expected_interlock_generation + 1U;
    if (
      !response->success ||
      response->result_code !=
      robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle::Response::
      RESULT_OK ||
      response->applied_sequence != *command_sequence ||
      own_key ||
      !execution_absent_in_response ||
      !generation_advanced_exactly_once)
    {
      return with_safety_proof(
        failed(
          "ELEVATOR_RECOVERY_HOLD_RELEASE_UNPROVEN",
          response->message + "; requested_sequence=" +
          std::to_string(*command_sequence) + "; applied_sequence=" +
          std::to_string(response->applied_sequence) +
          "; expected_generation=" +
          std::to_string(expected_interlock_generation) +
          "; response_generation=" +
          std::to_string(response->state.generation) +
          "; execution_session_engaged=" +
          std::to_string(response->state.execution_session_engaged) +
          "; execution_lease_active=" +
          std::to_string(response->state.execution_lease_active)),
        own_key, true);
    }

    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      interlock_state_ = response->state;
      have_interlock_state_ = true;
      interlock_received_at_sec_ = steady_now_sec();
    }
    evidence_cv_.notify_all();
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      hold_active_ = false;
      prepared_ = false;
      terminal_ = true;
      transaction_id_.clear();
      mission_id_.clear();
      mode_lease_id_.clear();
      mode_lease_revision_ = 0U;
      release_ = {};
      cleanup_context_ = {};
      restart_cleanup_runtime_superseded_ = false;
      mode_.clear();
      floor_result_.reset();
      cancel_requested_.store(false);
    }
    auto result = with_safety_proof(
      ok(
        "owner hold absence, runtime resource reconciliation, and dual "
        "odometry stop proven"),
      false, true);
    result.safety_hold_absence_proven = true;
    result.runtime_resources_reconciled = true;
    motion_admission_lock.unlock();
    options_.motion_admission_fence->reopen_and_invalidate();
    return result;
  }

  void request_cancel(const std::string & transaction_id) noexcept
  {
    // Linearize cancellation intent with both action submission seams. Once
    // this critical section completes, a sender that has not yet submitted
    // must observe cancel_requested_, while a sender that already submitted
    // has published either its goal handle or admission-unknown state.
    try {
      std::lock_guard<std::mutex> submission_lock(submission_mutex_);
      {
        std::lock_guard<std::mutex> lock(binding_mutex_);
        if (!prepared_ || transaction_id_ != transaction_id) {
          return;
        }
        cancel_requested_.store(true);
      }
    } catch (...) {
      return;
    }

    // Asserting the hold is safe to request immediately. Action cancellation
    // is deliberately left to formal cleanup below: it waits for the original
    // admission response, cancels the exact goal, and proves a terminal result.
    // A fire-and-forget cancel (especially cancel-all) could arrive during a
    // later transaction and cancel work that it does not own.
    try {
      async_assert_hold(transaction_id, "operator_cancel");
    } catch (...) {
    }

    try {
      evidence_cv_.notify_all();
      floor_goal_cv_.notify_all();
    } catch (...) {
      // noexcept emergency seam: formal cleanup retries synchronously.
    }
  }

  RuntimeResult heartbeat(const std::string & transaction_id)
  {
    const auto transaction_result = validate_transaction(transaction_id);
    if (!transaction_result.success) {
      return transaction_result;
    }
    if (const auto failure = mode_keepalive_failure()) {
      const auto recovered = renew_mode_lease();
      if (recovered.success) {
        return ok("operating-mode keepalive recovered");
      }
      if (owner_hold_active()) {
        return ok(
          "operating-mode renewal is deferred while the owner hold proves "
          "the robot stopped");
      }
      return failed(
        recovered.code.empty() ? failure->code : recovered.code,
        recovered.detail.empty() ? failure->detail : recovered.detail);
    }
    return ok("transaction-scoped operating mode is healthy");
  }

  RuntimeResult poll_health(const std::string & transaction_id)
  {
    const auto transaction_result = validate_transaction(transaction_id);
    if (!transaction_result.success) {
      return transaction_result;
    }
    const auto keepalive_failure = mode_keepalive_failure();
    if (keepalive_failure) {
      const auto recovered = renew_mode_lease();
      if (!recovered.success && !owner_hold_active()) {
        return failed(
          recovered.code.empty() ? keepalive_failure->code : recovered.code,
          recovered.detail.empty() ? keepalive_failure->detail :
          recovered.detail);
      }
    }
    if (ros_worker_->unavailable()) {
      return failed(
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY",
        "ROS runtime executor is unavailable");
    }
    if (cancel_requested_.load()) {
      return failed(
        "ELEVATOR_RUNTIME_CANCEL_REQUESTED",
        "elevator transaction cancellation is pending cleanup");
    }

    std::string transaction;
    std::string mission;
    std::string mode_lease;
    std::string expected_mode;
    bool expect_hold = false;
    bool expect_mode = false;
    bool expect_pause = false;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      transaction = transaction_id_;
      mission = mission_id_;
      mode_lease = mode_lease_id_;
      expected_mode = mode_;
      expect_hold = hold_active_;
      expect_mode = mode_lease_ownership_ == ResourceOwnership::kPresent;
      expect_pause =
        correction_pause_ownership_ == ResourceOwnership::kPresent;
      if (
        mode_lease_ownership_ == ResourceOwnership::kUnknown ||
        correction_pause_ownership_ == ResourceOwnership::kUnknown)
      {
        return failed(
          "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN",
          "mode or correction-pause outcome is unknown");
      }
    }

    const auto now = steady_now_sec();
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (
      expect_hold &&
      (!have_interlock_state_ ||
      now - interlock_received_at_sec_ > options_.state_evidence_max_age_sec ||
      !contains_key(interlock_state_.hold_keys, hold_key(transaction))))
    {
      return failed(
        "ELEVATOR_SAFETY_HOLD_LOST",
        "owner-scoped safety hold is missing or stale");
    }
    if (
      expect_mode && !expect_hold &&
      (!have_operating_mode_state_ ||
      now - operating_mode_received_at_sec_ >
      options_.state_evidence_max_age_sec ||
      !operating_mode_state_.lease_active ||
      operating_mode_state_.mode != expected_mode ||
      operating_mode_state_.owner != kRuntimeOwner ||
      operating_mode_state_.mission_id != mission ||
      operating_mode_state_.lease_id != mode_lease))
    {
      return failed(
        "ELEVATOR_OPERATING_MODE_LEASE_LOST",
        "owner-scoped operating mode lease is missing, stale, or mismatched");
    }
    if (
      expect_pause &&
      (!have_correction_pause_state_ ||
      now - correction_pause_received_at_sec_ >
      options_.state_evidence_max_age_sec ||
      !correction_pause_state_.paused ||
      !contains_key(
        correction_pause_state_.lease_keys,
        correction_key(kRuntimeOwner, transaction))))
    {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_LOST",
        "owner-scoped localization correction pause is missing or stale");
    }
    return ok("elevator runtime ownership and health evidence are current");
  }

private:
  void validate_options() const
  {
    const auto positive =
      [](const double value) {
        return std::isfinite(value) && value > 0.0;
      };
    if (
      options_.maps_root.empty() ||
      options_.runtime_map_context_file.empty() ||
      options_.hold_sequence_state_file.empty() ||
      !options_.hold_sequence_state_file.is_absolute() ||
      options_.elevator_scoped_behavior_tree.empty() ||
      options_.elevator_hall_call_scoped_behavior_tree.empty() ||
      options_.elevator_reverse_entry_staging_behavior_tree.empty() ||
      options_.elevator_reverse_docking_behavior_tree.empty() ||
      options_.elevator_cabin_entry_direct_behavior_tree.empty() ||
      options_.elevator_controller_session_service_prefix.empty() ||
      options_.elevator_entry_collision_bypass_permit_topic.empty() ||
      options_.recovery_hold_release_service.empty() ||
      options_.safety_status_topic.empty() ||
      options_.localization_bridge_status_topic.empty() ||
      options_.navigation_status_topic.empty() ||
      options_.floor_switch_action_status_topic.empty() ||
      !positive(options_.endpoint_timeout_sec) ||
      !positive(options_.service_timeout_sec) ||
      !positive(options_.navigation_timeout_sec) ||
      !positive(options_.floor_switch_timeout_sec) ||
      !positive(options_.stop_timeout_sec) ||
      !positive(options_.navigation_idle_stable_sec) ||
      !positive(options_.state_evidence_max_age_sec) ||
      !positive(options_.stop_feedback_max_age_sec) ||
      !positive(options_.stop_settle_sec) ||
      !positive(options_.mode_lease_sec) ||
      !positive(options_.nearby_hall_call_scoped_distance_m) ||
      options_.nearby_hall_call_scoped_distance_m > 2.5 ||
      !positive(options_.target_map_pose_freshness_sec) ||
      !positive(options_.target_map_pose_settle_sec) ||
      !positive(options_.target_map_pose_stability_translation_m) ||
      !positive(options_.target_map_pose_stability_yaw_rad) ||
      !positive(options_.elevator_entry_collision_bypass_refresh_sec))
    {
      throw std::invalid_argument("invalid elevator ROS runtime options");
    }
    if (options_.arm_button_control_enabled && !options_.arm_client) {
      throw std::invalid_argument(
              "automatic elevator button control requires an arm client");
    }
  }

  RuntimeResult apply_arm_button(
    const RuntimeEffect & effect,
    const bool hall_call)
  {
    if (!options_.arm_button_control_enabled || !options_.arm_client) {
      return failed(
        "ELEVATOR_AUTOMATIC_BUTTON_CONTROL_UNAVAILABLE",
        "an automatic button effect reached a runtime without arm control");
    }
    try {
      std::string target_floor_id;
      if (hall_call) {
        std::lock_guard<std::mutex> lock(binding_mutex_);
        target_floor_id = release_.target.floor_id;
      }
      const auto cancellation_probe = [this]() {
          return cancel_requested_.load();
        };
      const auto outcome = hall_call ?
        options_.arm_client->press_hall_call(
        effect.effect.transaction_id,
        effect.effect.sequence,
        effect.effect.floor_id,
        target_floor_id,
        cancellation_probe) :
        options_.arm_client->press_floor(
        effect.effect.transaction_id,
        effect.effect.sequence,
        effect.effect.floor_id,
        cancellation_probe);
      if (outcome.kind == ElevatorArmOutcomeKind::kFailed) {
        return failed(
          outcome.code.empty() ? "ELEVATOR_ARM_OPERATION_FAILED" : outcome.code,
          outcome.detail);
      }
      RuntimeResult result{true, outcome.code, outcome.detail};
      result.operator_confirmation_required =
        outcome.kind == ElevatorArmOutcomeKind::kManualConfirmationRequired;
      return result;
    } catch (const std::exception & exception) {
      return failed("ELEVATOR_ARM_CLIENT_EXCEPTION", exception.what());
    } catch (...) {
      return failed(
        "ELEVATOR_ARM_CLIENT_EXCEPTION",
        "arm client threw an unknown exception during button operation");
    }
  }

  std::chrono::nanoseconds timeout(const double seconds) const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  RuntimeResult wait_for_recovery_safety_endpoints()
  {
    const auto endpoint_timeout = timeout(options_.endpoint_timeout_sec);
    if (!wait_endpoint([&](auto slice) {return hold_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_SAFETY_HOLD_SERVICE_UNAVAILABLE",
        options_.motion_hold_service);
    }
    if (!wait_endpoint([&](auto slice) {return recovery_hold_release_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_RECOVERY_HOLD_RELEASE_SERVICE_UNAVAILABLE",
        options_.recovery_hold_release_service);
    }
    if (!wait_endpoint([&](auto slice) {return mode_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_MODE_SERVICE_UNAVAILABLE",
        options_.mode_service);
    }
    if (!wait_endpoint([&](auto slice) {return correction_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_SERVICE_UNAVAILABLE",
        options_.correction_pause_service);
    }
    return ok("restart recovery safety endpoints are available");
  }

  RuntimeResult wait_for_recovery_action_endpoints(
    const bool require_floor_action)
  {
    const auto endpoint_timeout = timeout(options_.endpoint_timeout_sec);
    if (!wait_endpoint([&](auto slice) {return nav_client_->wait_for_action_server(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_NAV2_ACTION_UNAVAILABLE",
        options_.navigate_to_pose_action);
    }
    if (
      require_floor_action &&
      !wait_endpoint([&](auto slice) {return floor_client_->wait_for_action_server(slice);}, endpoint_timeout))
    {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_ACTION_UNAVAILABLE",
        options_.floor_switch_action);
    }
    return ok("restart recovery action endpoints are available");
  }

  RuntimeResult wait_for_recovery_endpoints(const bool require_floor_action)
  {
    const auto safety = wait_for_recovery_safety_endpoints();
    if (!safety.success) {
      return safety;
    }
    return wait_for_recovery_action_endpoints(require_floor_action);
  }

  RuntimeResult prove_no_delayed_side_effect_unknown()
  {
    if (!options_.delayed_side_effect_unknown_probe) {
      return failed(
        "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE",
        "timed-out side-effect evidence probe is not configured");
    }
    try {
      const auto count = options_.delayed_side_effect_unknown_probe();
      return count == 0U ?
        ok("no delayed side-effect outcome remains unknown") :
        failed(
        "DELAYED_SIDE_EFFECT_UNKNOWN",
        std::to_string(count) +
        " timed-out motion/runtime submission(s) remain unresolved");
    } catch (const std::exception & exception) {
      return failed(
        "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE", exception.what());
    } catch (...) {
      return failed(
        "DELAYED_SIDE_EFFECT_EVIDENCE_UNAVAILABLE",
        "timed-out side-effect evidence probe threw an unknown exception");
    }
  }

  RuntimeResult wait_for_all_runtimes_idle()
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.navigation_idle_stable_sec);
    bool sampled = false;
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      std::array<bool, 3> idle{};
      try {
        idle = options_.runtime_idle_probe();
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_RECOVERY_RUNTIME_PROBE_FAILED", exception.what());
      } catch (...) {
        return failed(
          "ELEVATOR_RECOVERY_RUNTIME_PROBE_FAILED",
          "runtime idle probe threw an unknown exception");
      }
      sampled = true;
      if (!idle[0] || !idle[1] || !idle[2]) {
        return failed(
          "ELEVATOR_RECOVERY_RUNTIME_BUSY",
          "navigation, teleop, mapping, or docking runtime is active");
      }
      std::this_thread::sleep_for(20ms);
    }
    return sampled ?
      ok("all motion runtimes remained idle for the stability window") :
      failed(
      "ELEVATOR_RECOVERY_RUNTIME_IDLE_UNPROVEN",
      "runtime idle stability window produced no sample");
  }

  void begin_recovery_cancel_barrier(const bool require_floor_action)
  {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    recovery_nav_action_barrier_.begin_round();
    if (require_floor_action) {
      recovery_floor_action_barrier_.begin_round();
    }
  }

  RuntimeResult wait_for_recovery_resource_absence(
    const std::string & transaction_id,
    std::uint64_t * proven_interlock_generation = nullptr)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto now = steady_now_sec();
      const bool fresh_interlock =
        have_interlock_state_ &&
        now - interlock_received_at_sec_ >= 0.0 &&
        now - interlock_received_at_sec_ <=
        options_.state_evidence_max_age_sec;
      const bool fresh_mode =
        have_operating_mode_state_ &&
        now - operating_mode_received_at_sec_ >= 0.0 &&
        now - operating_mode_received_at_sec_ <=
        options_.state_evidence_max_age_sec;
      const bool fresh_pause =
        have_correction_pause_state_ &&
        now - correction_pause_received_at_sec_ >= 0.0 &&
        now - correction_pause_received_at_sec_ <=
        options_.state_evidence_max_age_sec;
      if (fresh_interlock && fresh_mode && fresh_pause) {
        const bool execution_absent =
          !interlock_state_.execution_session_engaged &&
          !interlock_state_.execution_lease_active;
        const bool own_mode_absent =
          !operating_mode_state_.lease_active ||
          operating_mode_state_.owner != kRuntimeOwner ||
          operating_mode_state_.mission_id !=
          "elevator_" + transaction_id;
        const bool own_pause_absent =
          !contains_key(
          correction_pause_state_.lease_keys,
          correction_key(kRuntimeOwner, transaction_id));
        const bool floor_pause_absent =
          !contains_key(
          correction_pause_state_.lease_keys,
          correction_key(kFloorPauseOwner, transaction_id));
        const bool floor_hold_absent =
          !contains_key(
          interlock_state_.hold_keys,
          std::string(kFloorPauseOwner) + ":" + transaction_id);
        if (
          execution_absent && own_mode_absent && own_pause_absent &&
          floor_pause_absent && floor_hold_absent)
        {
          if (proven_interlock_generation != nullptr) {
            *proven_interlock_generation = interlock_state_.generation;
          }
          return ok(
            "no execution session, elevator mode lease, API correction pause, "
            "floor handoff pause, or floor handoff hold is retained");
        }
        return failed(
          "ELEVATOR_RECOVERY_RESOURCE_RETAINED",
          "an execution session (including a failure-latched session), "
          "elevator mode lease, API correction pause, floor handoff pause, or "
          "floor handoff hold is still retained");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_RECOVERY_RESOURCE_EVIDENCE_STALE",
      "fresh execution, mode, and correction-pause evidence was not observed");
  }

  RuntimeResult cancel_all_nav_goals_for_recovery()
  {
    std::lock_guard<std::mutex> cancel_lock(recovery_cancel_mutex_);
    if (recovery_nav_cancel_all_response_unknown_.load()) {
      return failed(
        "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_UNKNOWN",
        "an earlier Nav2 recovery cancel-all request has no proven response; "
        "a full runtime restart is required");
    }
    bool request_may_have_side_effect =
      recovery_nav_cancel_all_response_.pending();
    RecoveryCancelResponse response;
    try {
      const bool submitted =
        recovery_nav_cancel_all_response_.submit_if_absent(
        [this]() {return nav_client_->async_cancel_all_goals();});
      request_may_have_side_effect = request_may_have_side_effect || submitted;
      if (
        ros_worker_->wait_for(recovery_nav_cancel_all_response_, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_PENDING",
          "the single submitted Nav2 cancel-all request is still awaiting its "
          "response; the pending response was retained and no duplicate "
          "request was sent");
      }
      response = recovery_nav_cancel_all_response_.take_ready();
    } catch (const ElevatorRosExecutorUnavailable &) {
        throw;
    } catch (const std::exception & exception) {
      recovery_nav_cancel_all_response_.abandon();
      if (request_may_have_side_effect) {
        recovery_nav_cancel_all_response_unknown_.store(true);
      }
      return failed(
        request_may_have_side_effect ?
        "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_UNKNOWN" :
        "ELEVATOR_RESTART_NAV_CANCEL_FAILED",
        request_may_have_side_effect ?
        std::string(exception.what()) +
        "; the submitted cancel-all outcome is unknown and requires a full "
        "runtime restart" :
        exception.what());
    } catch (...) {
      recovery_nav_cancel_all_response_.abandon();
      if (request_may_have_side_effect) {
        recovery_nav_cancel_all_response_unknown_.store(true);
      }
      return failed(
        request_may_have_side_effect ?
        "ELEVATOR_RESTART_NAV_CANCEL_RESPONSE_UNKNOWN" :
        "ELEVATOR_RESTART_NAV_CANCEL_FAILED",
        request_may_have_side_effect ?
        "the submitted Nav2 cancel-all outcome is unknown; a full runtime "
        "restart is required" :
        "Nav2 cancel-all threw an unknown exception before submission");
    }
    if (
      !response ||
      response->return_code !=
      action_msgs::srv::CancelGoal::Response::ERROR_NONE)
    {
      return failed(
        "ELEVATOR_RESTART_NAV_CANCEL_REJECTED",
        response ?
        "Nav2 cancel-all return_code=" +
        std::to_string(response->return_code) :
        "Nav2 returned an empty cancel-all response");
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      recovery_nav_action_barrier_.record_cancel_response(
        recovery_cancel_goal_ids(*response));
    }
    evidence_cv_.notify_all();
    return ok("Nav2 accepted restart cancel-all");
  }

  RuntimeResult cancel_all_floor_goals_for_recovery()
  {
    std::lock_guard<std::mutex> cancel_lock(recovery_cancel_mutex_);
    if (recovery_floor_cancel_all_response_unknown_.load()) {
      return failed(
        "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_UNKNOWN",
        "an earlier floor recovery cancel-all request has no proven response; "
        "a full runtime restart is required");
    }
    bool request_may_have_side_effect =
      recovery_floor_cancel_all_response_.pending();
    RecoveryCancelResponse response;
    try {
      const bool submitted =
        recovery_floor_cancel_all_response_.submit_if_absent(
        [this]() {return floor_client_->async_cancel_all_goals();});
      request_may_have_side_effect = request_may_have_side_effect || submitted;
      if (
        ros_worker_->wait_for(recovery_floor_cancel_all_response_, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_PENDING",
          "the single submitted floor-switch cancel-all request is still "
          "awaiting its response; the pending response was retained and no "
          "duplicate request was sent");
      }
      response = recovery_floor_cancel_all_response_.take_ready();
    } catch (const ElevatorRosExecutorUnavailable &) {
        throw;
    } catch (const std::exception & exception) {
      recovery_floor_cancel_all_response_.abandon();
      if (request_may_have_side_effect) {
        recovery_floor_cancel_all_response_unknown_.store(true);
      }
      return failed(
        request_may_have_side_effect ?
        "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_UNKNOWN" :
        "ELEVATOR_RESTART_FLOOR_CANCEL_FAILED",
        request_may_have_side_effect ?
        std::string(exception.what()) +
        "; the submitted cancel-all outcome is unknown and requires a full "
        "runtime restart" :
        exception.what());
    } catch (...) {
      recovery_floor_cancel_all_response_.abandon();
      if (request_may_have_side_effect) {
        recovery_floor_cancel_all_response_unknown_.store(true);
      }
      return failed(
        request_may_have_side_effect ?
        "ELEVATOR_RESTART_FLOOR_CANCEL_RESPONSE_UNKNOWN" :
        "ELEVATOR_RESTART_FLOOR_CANCEL_FAILED",
        request_may_have_side_effect ?
        "the submitted floor cancel-all outcome is unknown; a full runtime "
        "restart is required" :
        "floor cancel-all threw an unknown exception before submission");
    }
    if (
      !response ||
      response->return_code !=
      action_msgs::srv::CancelGoal::Response::ERROR_NONE)
    {
      return failed(
        "ELEVATOR_RESTART_FLOOR_CANCEL_REJECTED",
        response ?
        "floor cancel-all return_code=" +
        std::to_string(response->return_code) :
        "floor manager returned an empty cancel-all response");
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      recovery_floor_action_barrier_.record_cancel_response(
        recovery_cancel_goal_ids(*response));
    }
    evidence_cv_.notify_all();
    return ok("floor manager accepted restart cancel-all");
  }

  RuntimeResult wait_for_restart_action_idle(const bool require_floor_action)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(std::max(
        options_.service_timeout_sec,
        options_.endpoint_timeout_sec));
    std::optional<std::chrono::steady_clock::time_point> idle_since;
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto nav_state = recovery_nav_action_barrier_.state();
      const auto floor_state = require_floor_action ?
        recovery_floor_action_barrier_.state() :
        ElevatorRecoveryActionBarrierState::kIdleProven;
      if (
        nav_state == ElevatorRecoveryActionBarrierState::kUnsafe ||
        floor_state == ElevatorRecoveryActionBarrierState::kUnsafe)
      {
        return failed(
          "ELEVATOR_RESTART_ACTION_STATUS_UNSAFE",
          "Nav2: " + recovery_nav_action_barrier_.detail() +
          (require_floor_action ?
          "; floor-switch: " + recovery_floor_action_barrier_.detail() :
          "; floor-switch: not journaled for this transaction"));
      }
      const bool nav_idle =
        nav_state == ElevatorRecoveryActionBarrierState::kIdleProven;
      const bool floor_idle =
        floor_state == ElevatorRecoveryActionBarrierState::kIdleProven;
      const auto now = std::chrono::steady_clock::now();
      if (nav_idle && floor_idle) {
        if (!idle_since) {
          idle_since = now;
        }
        if (
          now - *idle_since >=
          std::chrono::duration<double>(options_.navigation_idle_stable_sec))
        {
          return ok(
            require_floor_action ?
            "both cancel-all rounds are empty or every named action goal has "
            "post-response terminal evidence" :
            "Nav2 cancel-all is empty or every named goal has post-response "
            "terminal evidence; no floor action intent was journaled");
        }
      } else {
        idle_since.reset();
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_RESTART_ACTION_TERMINAL_UNPROVEN",
      require_floor_action ?
      "cancel-all responses did not prove both action servers empty and "
      "post-response terminal evidence was incomplete" :
      "Nav2 cancel-all did not prove terminal idle evidence");
  }

  RuntimeResult wait_for_owner_hold_evidence()
  {
    std::string transaction;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      transaction = transaction_id_;
    }
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto age = steady_now_sec() - interlock_received_at_sec_;
      if (
        have_interlock_state_ && age >= 0.0 &&
        age <= options_.state_evidence_max_age_sec &&
        contains_key(interlock_state_.hold_keys, hold_key(transaction)))
      {
        return ok("fresh robot_safety evidence retains the owner hold");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_RESTART_SAFETY_HOLD_UNPROVEN",
      "fresh robot_safety evidence does not retain the restart owner hold");
  }

  RuntimeResult wait_for_endpoints()
  {
    const auto endpoint_timeout = timeout(options_.endpoint_timeout_sec);
    if (!wait_endpoint([&](auto slice) {return hold_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_SAFETY_HOLD_SERVICE_UNAVAILABLE",
        options_.motion_hold_service);
    }
    if (!wait_endpoint([&](auto slice) {return mode_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_MODE_SERVICE_UNAVAILABLE", options_.mode_service);
    }
    if (!wait_endpoint([&](auto slice) {return correction_client_->wait_for_service(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_SERVICE_UNAVAILABLE",
        options_.correction_pause_service);
    }
    if (!wait_endpoint([&](auto slice) {return nav_client_->wait_for_action_server(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_NAV2_ACTION_UNAVAILABLE",
        options_.navigate_to_pose_action);
    }
    if (!wait_endpoint([&](auto slice) {return floor_client_->wait_for_action_server(slice);}, endpoint_timeout)) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_ACTION_UNAVAILABLE",
        options_.floor_switch_action);
    }
    return ok("all elevator runtime endpoints are available");
  }

  RuntimeResult wait_for_navigation_idle_evidence()
  {
    const auto stable_deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.navigation_idle_stable_sec);
    bool observed_status = false;
    while (std::chrono::steady_clock::now() < stable_deadline) {
      ros_worker_->check();
      {
        std::unique_lock<std::mutex> lock(evidence_mutex_);
        observed_status = observed_status || have_navigation_status_;
        if (have_navigation_status_ && navigation_goal_active_) {
          return failed(
            "ELEVATOR_SOURCE_NAVIGATION_BUSY",
            "latched Nav2 action status reports an active goal");
        }
        evidence_cv_.wait_for(lock, 20ms);
      }
      try {
        const auto idle = options_.runtime_idle_probe();
        if (!idle[0]) {
          return failed(
            "ELEVATOR_SOURCE_NAVIGATION_BUSY",
            "the API runtime snapshot reports navigation or teleop active");
        }
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_SOURCE_RUNTIME_PROBE_FAILED", exception.what());
      } catch (...) {
        return failed(
          "ELEVATOR_SOURCE_RUNTIME_PROBE_FAILED",
          "runtime idle probe threw an unknown exception");
      }
    }
    return ok(observed_status ?
      "latched Nav2 status and API runtime remained idle for the bounded "
      "stability window" :
      "Nav2 published no goal status; the authoritative API goal/teleop "
      "runtime remained idle for the bounded stability window");
  }

  RuntimeResult wait_for_no_foreign_elevator_hold(
    const std::string & transaction_id)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto age = steady_now_sec() - interlock_received_at_sec_;
      if (
        have_interlock_state_ && age >= 0.0 &&
        age <= options_.state_evidence_max_age_sec)
      {
        const auto prefix = std::string(kRuntimeOwner) + ":";
        const auto own_key = hold_key(transaction_id);
        const auto foreign = std::find_if(
          interlock_state_.hold_keys.cbegin(),
          interlock_state_.hold_keys.cend(),
          [&prefix, &own_key](const std::string & key) {
            return key.rfind(prefix, 0U) == 0U && key != own_key;
          });
        if (foreign != interlock_state_.hold_keys.cend()) {
          return failed(
            "ELEVATOR_RUNTIME_RECOVERY_REQUIRED",
            "a previous elevator transaction still owns safety hold " +
            *foreign);
        }
        return ok("no previous elevator transaction hold remains");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_SAFETY_INTERLOCK_STATE_STALE",
      "fresh robot_safety interlock state was not observed");
  }

  RuntimeResult validate_live_source_identity(const FrozenRelease & release)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto now_sec = steady_now_sec();
      const auto health_age =
        now_sec - localization_health_received_at_sec_;
      const bool fresh =
        have_localizer_asset_state_ && have_localization_health_ &&
        health_age >= 0.0 &&
        health_age <= options_.state_evidence_max_age_sec;
      const auto & source = release.source;
      const bool localizer_exact =
        fresh &&
        localizer_asset_state_.success &&
        !localizer_asset_state_.applying &&
        localizer_asset_state_.active_identity_valid &&
        localizer_asset_state_.active_building_id == release.building_id &&
        localizer_asset_state_.active_floor_id == source.floor_id &&
        localizer_asset_state_.active_map_id == source.map_id &&
        localizer_asset_state_.active_asset_epoch == source.map_asset_epoch &&
        localizer_asset_state_.active_asset_digest == source.map_asset_digest &&
        localizer_asset_state_.localizer_generation > 0U &&
        localizer_asset_state_.localizer_ready;
      const bool bridge_exact =
        fresh &&
        localization_health_.building_id == release.building_id &&
        localization_health_.floor_id == source.floor_id &&
        localization_health_.map_id == source.map_id &&
        localization_health_.asset_epoch == source.map_asset_epoch &&
        localization_health_.asset_digest == source.map_asset_digest &&
        localization_health_.localizer_generation ==
        localizer_asset_state_.localizer_generation &&
        localization_health_.localizer_ready &&
        localization_health_.bridge_ready &&
        localization_health_.tf_unique &&
        localization_health_.runtime_context_valid &&
        !localization_health_.transition_active &&
        localization_health_.explicit_relocalization_sequence > 0U;
      if (localizer_exact && bridge_exact) {
        return ok(
          "live localizer and localization bridge prove the exact source "
          "asset identity");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_SOURCE_LIVE_IDENTITY_UNPROVEN",
      "live localizer/bridge evidence does not match the frozen source map");
  }

  RuntimeResult validate_live_cleanup_identity(
    const CleanupContext & context,
    const bool allow_ready_runtime_supersession = false,
    bool * const runtime_superseded_out = nullptr)
  {
    if (runtime_superseded_out != nullptr) {
      *runtime_superseded_out = false;
    }
    return check_elevator_cleanup_runtime_readiness(
      options_.persistent_recovery_lock_enabled, context.disposition,
      [this, &context, allow_ready_runtime_supersession, runtime_superseded_out]() {
        return validate_strict_cleanup_runtime_identity(
          context, allow_ready_runtime_supersession, runtime_superseded_out);
      });
  }

  RuntimeResult validate_strict_cleanup_runtime_identity(
    const CleanupContext & context,
    const bool allow_ready_runtime_supersession,
    bool * const runtime_superseded_out)
  {
    const auto runtime_context =
      read_runtime_map_context_file(options_.runtime_map_context_file);
    if (!runtime_context)
    {
      return failed(
        "ELEVATOR_CLEANUP_RUNTIME_CONTEXT_UNPROVEN",
        "runtime context is unavailable during elevator cleanup");
    }
    const ElevatorCleanupRuntimeFloorEvidence runtime_evidence{
      runtime_context->confirmed,
      runtime_context->state,
      runtime_context->building_id,
      runtime_context->floor_id,
      runtime_context->map_id,
      runtime_context->asset_epoch,
      runtime_context->asset_digest,
    };
    const auto resolved_floor =
      resolve_elevator_cleanup_runtime_floor(context, runtime_evidence);
    const bool runtime_superseded =
      !resolved_floor && allow_ready_runtime_supersession &&
      elevator_nonpersistent_restart_cleanup_superseded_by_ready_runtime(
        context, runtime_evidence);
    if (!resolved_floor && !runtime_superseded)
    {
      return failed(
        "ELEVATOR_CLEANUP_RUNTIME_CONTEXT_UNPROVEN",
        context.floor_switch_action_may_have_been_submitted ?
        "runtime context is not confirmed ready for the exact outside floor" :
        "pre-floor failure cleanup requires a confirmed frozen source or target endpoint");
    }
    const ElevatorRuntimeFloorIdentity floor = resolved_floor.value_or(
      ElevatorRuntimeFloorIdentity{
        runtime_evidence.floor_id,
        runtime_evidence.map_id,
        runtime_evidence.asset_epoch,
        runtime_evidence.asset_digest,
      });
    const auto & expected_building_id = runtime_superseded ?
      runtime_evidence.building_id : context.building_id;
    const auto & recorded_outside =
      context.disposition == CleanupDisposition::kSourceOutside ?
      context.source : context.target;
    const bool endpoint_reconciled = runtime_superseded ||
      floor.floor_id != recorded_outside.floor_id ||
      floor.map_id != recorded_outside.map_id ||
      floor.asset_epoch != recorded_outside.asset_epoch ||
      floor.asset_digest != recorded_outside.asset_digest;

    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    ElevatorCleanupRuntimeIdentityAssessment last_assessment;
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto now = steady_now_sec();
      const bool localizer_exact =
        have_localizer_asset_state_ && localizer_asset_state_.success &&
        !localizer_asset_state_.applying &&
        localizer_asset_state_.active_identity_valid &&
        localizer_asset_state_.active_building_id == expected_building_id &&
        localizer_asset_state_.active_floor_id == floor.floor_id &&
        localizer_asset_state_.active_map_id == floor.map_id &&
        localizer_asset_state_.active_asset_epoch == floor.asset_epoch &&
        localizer_asset_state_.active_asset_digest == floor.asset_digest &&
        localizer_asset_state_.localizer_generation > 0U &&
        localizer_asset_state_.localizer_ready;
      const bool health_exact =
        have_localization_health_ &&
        localization_health_.building_id == expected_building_id &&
        localization_health_.floor_id == floor.floor_id &&
        localization_health_.map_id == floor.map_id &&
        localization_health_.asset_epoch == floor.asset_epoch &&
        localization_health_.asset_digest == floor.asset_digest &&
        localization_health_.localizer_generation ==
        localizer_asset_state_.localizer_generation &&
        localization_health_.localizer_ready &&
        localization_health_.bridge_ready &&
        localization_health_.tf_unique &&
        localization_health_.runtime_context_valid &&
        !localization_health_.transition_active;
      const auto bridge_has_map =
        json_bool_field(localization_bridge_status_, "has_map_to_odom");
      const auto bridge_state_valid =
        json_bool_field(localization_bridge_status_, "map_odom_state_valid");
      const auto bridge_correction_active =
        json_bool_field(localization_bridge_status_, "correction_active");
      const auto bridge_safe =
        json_bool_field(localization_bridge_status_, "safe_for_goal_start");
      const auto bridge_transition =
        json_bool_field(localization_bridge_status_, "floor_transition_active");
      const auto bridge_context_valid =
        json_bool_field(
        localization_bridge_status_, "floor_runtime_context_valid");
      const auto bridge_failed =
        json_bool_field(localization_bridge_status_, "floor_failed_locked");
      const auto current_sequence =
        json_uint64_field(localization_bridge_status_, "current_sequence");
      const auto target_sequence =
        json_uint64_field(localization_bridge_status_, "target_sequence");
      const bool bridge_exact =
        have_localization_bridge_status_ &&
        bridge_has_map.value_or(false) &&
        bridge_state_valid.value_or(false) &&
        !bridge_correction_active.value_or(true) &&
        bridge_safe.value_or(false) &&
        !bridge_transition.value_or(true) &&
        bridge_context_valid.value_or(false) &&
        !bridge_failed.value_or(true) &&
        current_sequence && target_sequence &&
        *current_sequence == *target_sequence &&
        json_string_field(localization_bridge_status_, "floor_building_id") ==
        std::optional<std::string>{expected_building_id} &&
        json_string_field(localization_bridge_status_, "floor_id") ==
        std::optional<std::string>{floor.floor_id} &&
        json_string_field(localization_bridge_status_, "floor_map_id") ==
        std::optional<std::string>{floor.map_id} &&
        json_uint64_field(localization_bridge_status_, "floor_asset_epoch") ==
        std::optional<std::uint64_t>{floor.asset_epoch} &&
        json_string_field(localization_bridge_status_, "floor_asset_digest") ==
        std::optional<std::string>{floor.asset_digest};
      last_assessment = assess_elevator_cleanup_runtime_identity(
        ElevatorCleanupRuntimeIdentityEvidence{
          have_localizer_asset_state_,
          localizer_exact,
          have_localization_health_,
          health_exact,
          localization_health_received_at_sec_,
          have_localization_bridge_status_,
          bridge_exact,
          localization_bridge_status_received_at_sec_,
        },
        now,
        options_.state_evidence_max_age_sec);
      if (last_assessment.proven) {
        if (runtime_superseded_out != nullptr) {
          *runtime_superseded_out = runtime_superseded;
        }
        return ok(
          runtime_superseded ?
          "confirmed ready replacement runtime supersedes the stale elevator "
          "transaction; localizer, settled map->odom, unique TF, and goal-start "
          "safety are proven" : endpoint_reconciled ?
          "pre-floor failure reconciled to the current frozen elevator endpoint; "
          "localizer, settled map->odom, unique TF, and goal-start safety are proven" :
          "exact outside runtime context, localizer, settled map->odom, "
          "unique TF, and goal-start safety are proven");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    const auto boolean_text =
      [](const bool value) {return value ? "true" : "false";};
    const auto age_text =
      [](const double value) {
        return value >= 0.0 ? std::to_string(value) : std::string("unavailable");
      };
    return failed(
      "ELEVATOR_CLEANUP_RUNTIME_IDENTITY_UNPROVEN",
      std::string("outside runtime identity evidence is incomplete;") +
      "asset_present=" +
      boolean_text(last_assessment.localizer_asset_present) +
      ";health_age_sec=" +
      age_text(last_assessment.localization_health_age_sec) +
      ";bridge_age_sec=" +
      age_text(last_assessment.localization_bridge_age_sec) +
      ";localizer_exact=" +
      boolean_text(last_assessment.localizer_asset_exact) +
      ";health_exact=" +
      boolean_text(last_assessment.localization_health_exact) +
      ";bridge_exact=" +
      boolean_text(last_assessment.localization_bridge_exact) +
      ";health_fresh=" +
      boolean_text(last_assessment.localization_health_fresh) +
      ";bridge_fresh=" +
      boolean_text(last_assessment.localization_bridge_fresh));
  }

  RuntimeResult validate_transaction(const std::string & transaction_id) const
  {
    std::lock_guard<std::mutex> lock(binding_mutex_);
    if (!prepared_) {
      return failed(
        "ELEVATOR_RUNTIME_NOT_PREPARED",
        "no frozen elevator release is bound");
    }
    if (transaction_id != transaction_id_) {
      return failed(
        "ELEVATOR_RUNTIME_TRANSACTION_MISMATCH",
        "runtime transaction_id does not match the bound release");
    }
    return ok("transaction matches");
  }

  RuntimeResult validate_effect_binding(const RuntimeEffect & runtime_effect) const
  {
    const auto transaction_result =
      validate_transaction(runtime_effect.effect.transaction_id);
    if (!transaction_result.success) {
      return transaction_result;
    }
    std::lock_guard<std::mutex> lock(binding_mutex_);
    if (
      runtime_effect.building_id != release_.building_id ||
      runtime_effect.elevator_id != release_.elevator_id)
    {
      return failed(
        "ELEVATOR_RUNTIME_RELEASE_MISMATCH",
        "runtime effect building/elevator differs from frozen release");
    }
    if (!runtime_effect.floor_id.empty()) {
      const auto * floor =
        runtime_effect.floor_id == release_.source.floor_id ?
        &release_.source :
        (runtime_effect.floor_id == release_.target.floor_id ?
        &release_.target : nullptr);
      if (
        floor == nullptr ||
        runtime_effect.map_id != floor->map_id ||
        runtime_effect.asset_epoch != floor->map_asset_epoch ||
        runtime_effect.asset_digest != floor->map_asset_digest)
      {
        return failed(
          "ELEVATOR_RUNTIME_ASSET_IDENTITY_MISMATCH",
          "runtime effect floor identity differs from frozen release");
      }
    }
    return ok("runtime effect matches frozen release");
  }

  template<typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const std::shared_ptr<typename ServiceT::Request> & request)
  {
    if (!wait_endpoint([&](auto slice) {return client->wait_for_service(slice);}, timeout(options_.service_timeout_sec))) {
      return {};
    }
    auto future = client->async_send_request(request);
    if (ros_worker_->wait_for(future, timeout(options_.service_timeout_sec)) !=
      std::future_status::ready)
    {
      client->remove_pending_request(future);
      return {};
    }
    return future.get();
  }

  RuntimeResult set_elevator_controller_session(
    const std::string & controller_id,
    const std::string & session_id,
    const std::uint8_t operation,
    const std::string & terminal_reason)
  {
    const auto client = controller_session_clients_.find(controller_id);
    if (client == controller_session_clients_.end()) {
      return failed(
        "ELEVATOR_CONTROLLER_SESSION_CLIENT_MISSING",
        "no execution-session client is configured for " + controller_id);
    }
    auto request = std::make_shared<ElevatorNavigationSession::Request>();
    request->operation = operation;
    request->session_id = session_id;
    request->terminal_reason = terminal_reason;
    const auto response = call_service<ElevatorNavigationSession>(
      client->second, request);
    if (!response) {
      return failed(
        "ELEVATOR_CONTROLLER_SESSION_SERVICE_UNAVAILABLE",
        "execution-session service unavailable or timed out for " +
        controller_id);
    }
    if (!response->accepted) {
      return failed(
        "ELEVATOR_CONTROLLER_SESSION_REJECTED",
        "controller=" + controller_id + " session=" + session_id +
        " detail=" + response->detail);
    }
    return ok(
      "controller=" + controller_id + " session=" + session_id +
      " detail=" + response->detail);
  }

  std::string hold_key(const std::string & transaction_id) const
  {
    return std::string(kRuntimeOwner) + ":" + transaction_id;
  }

  std::optional<std::uint64_t> next_hold_command_sequence()
  {
    return hold_sequence_allocator_->next();
  }

  void synchronize_hold_command_sequence(const std::uint64_t observed)
  {
    hold_sequence_allocator_->synchronize(observed);
  }

  static std::string correction_key(
    const std::string & owner,
    const std::string & transaction_id)
  {
    return owner + ":" + transaction_id;
  }

  RuntimeResult set_hold(const bool acquire, const std::string & reason)
  {
    if (!acquire && cancel_requested_.load()) {
      return failed(
        "ELEVATOR_CANCEL_FENCE_ACTIVE",
        "a cancellation fence forbids releasing the owner safety hold");
    }
    std::string transaction;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      transaction = transaction_id_;
      if (!acquire && !hold_active_) {
        auto result = ok("owner safety hold already released");
        result.safety_hold_absence_proven = true;
        return result;
      }
    }
    auto request =
      std::make_shared<robot_interfaces::srv::SetMotionHold::Request>();
    const auto command_sequence = next_hold_command_sequence();
    if (!command_sequence) {
      return failed(
        "ELEVATOR_SAFETY_HOLD_SEQUENCE_EXHAUSTED",
        "motion-hold command sequence is exhausted");
    }
    request->owner = kRuntimeOwner;
    request->transaction_id = transaction;
    request->reason = reason.empty() ? "elevator_runtime" : reason;
    request->operation = acquire ?
      robot_interfaces::srv::SetMotionHold::Request::OP_ACQUIRE :
      robot_interfaces::srv::SetMotionHold::Request::OP_RELEASE;
    request->command_sequence = *command_sequence;
    const auto response =
      call_service<robot_interfaces::srv::SetMotionHold>(hold_client_, request);
    if (!response) {
      return failed(
        "ELEVATOR_SAFETY_HOLD_TIMEOUT",
        "motion hold service did not respond");
    }
    const bool own_key =
      contains_key(response->state.hold_keys, hold_key(transaction));
    if (
      !response->success &&
      response->result_code ==
      robot_interfaces::srv::SetMotionHold::Response::RESULT_STALE_COMMAND)
    {
      synchronize_hold_command_sequence(response->applied_sequence);
      return failed(
        "ELEVATOR_SAFETY_HOLD_SEQUENCE_STALE",
        response->message + "; requested_sequence=" +
        std::to_string(*command_sequence) + "; applied_sequence=" +
        std::to_string(response->applied_sequence));
    }
    if (
      !response->success ||
      response->applied_sequence != *command_sequence ||
      (acquire && !own_key) || (!acquire && own_key))
    {
      return failed(
        "ELEVATOR_SAFETY_HOLD_REJECTED",
        response->message + "; requested_sequence=" +
        std::to_string(*command_sequence) + "; applied_sequence=" +
        std::to_string(response->applied_sequence));
    }
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      hold_active_ = acquire;
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      interlock_state_ = response->state;
      have_interlock_state_ = true;
      interlock_received_at_sec_ = steady_now_sec();
      ++interlock_observation_generation_;
    }
    evidence_cv_.notify_all();
    if (!acquire && cancel_requested_.load()) {
      const auto reacquired = set_hold(true, "cancel_fence_after_release");
      return failed(
        "ELEVATOR_CANCEL_FENCE_ACTIVE",
        reacquired.success ?
        "cancellation raced with hold release; owner hold was reacquired" :
        "cancellation raced with hold release and synchronous reacquire "
        "could not be proven: " + reacquired.detail);
    }
    auto result =
      ok(acquire ? "owner safety hold acquired" : "owner safety hold released");
    result.safety_hold_proven = acquire;
    result.safety_hold_absence_proven = !acquire;
    return result;
  }

  void async_assert_hold(
    const std::string & transaction_id,
    const std::string & reason)
  {
    if (!hold_client_->service_is_ready()) {
      return;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::SetMotionHold::Request>();
    const auto command_sequence = next_hold_command_sequence();
    if (!command_sequence) {
      return;
    }
    request->owner = kRuntimeOwner;
    request->transaction_id = transaction_id;
    request->reason = reason;
    request->operation =
      robot_interfaces::srv::SetMotionHold::Request::OP_ACQUIRE;
    request->command_sequence = *command_sequence;
    (void)hold_client_->async_send_request(request);
  }

  RuntimeResult wait_for_mode_release_evidence(
    const std::uint64_t observation_generation,
    const double response_received_at_sec)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      if (
        operating_mode_observation_generation_ > observation_generation &&
        operating_mode_received_at_sec_ > response_received_at_sec &&
        !operating_mode_state_.lease_active &&
        operating_mode_state_.mode == "NORMAL")
      {
        return ok("fresh operating-mode state proves NORMAL and unowned");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_OPERATING_MODE_RELEASE_UNPROVEN",
      "no post-response operating-mode state proved NORMAL and unowned");
  }

  RuntimeResult wait_for_correction_pause_release_evidence(
    const std::string & transaction,
    const bool require_floor_handoff,
    const std::uint64_t observation_generation,
    const double response_received_at_sec)
  {
    const auto caller_key = correction_key(kRuntimeOwner, transaction);
    const auto floor_key = correction_key(kFloorPauseOwner, transaction);
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const bool caller_absent =
        !contains_key(correction_pause_state_.lease_keys, caller_key);
      const bool handoff_proven =
        !require_floor_handoff ||
        (correction_pause_state_.paused &&
        contains_key(correction_pause_state_.lease_keys, floor_key));
      if (
        correction_pause_observation_generation_ > observation_generation &&
        correction_pause_received_at_sec_ > response_received_at_sec &&
        caller_absent && handoff_proven)
      {
        return ok(
          require_floor_handoff ?
          "fresh correction state proves caller-to-floor pause handoff" :
          "fresh correction state proves caller pause absent");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      require_floor_handoff ?
      "ELEVATOR_CORRECTION_PAUSE_HANDOFF_UNPROVEN" :
      "ELEVATOR_CORRECTION_PAUSE_RELEASE_UNPROVEN",
      require_floor_handoff ?
      "no post-response correction state proved caller absent while the "
      "floor-manager pause remained" :
      "no post-response correction state proved the caller pause absent");
  }

  std::optional<RuntimeResult> mode_keepalive_failure() const
  {
    if (!mode_keepalive_) {
      return std::nullopt;
    }
    const auto failure = mode_keepalive_->last_failure();
    if (!failure) {
      return std::nullopt;
    }
    return failed(failure->code, failure->detail);
  }

  void handle_mode_keepalive_failure(
    const ElevatorLeaseKeepaliveStatus & failure) noexcept
  {
    try {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Elevator operating-mode keepalive failed: code=%s detail=%s",
        failure.code.c_str(), failure.detail.c_str());
      evidence_cv_.notify_all();
      floor_goal_cv_.notify_all();
    } catch (...) {
      // The failure is latched. The active navigation loop or transaction
      // monitor performs ordinary hold/cancel cleanup. This callback must not
      // cancel an already-held FloorSwitch action and create FAILED_LOCKED.
    }
  }

  bool owner_hold_active() const
  {
    std::lock_guard<std::mutex> lock(binding_mutex_);
    return hold_active_;
  }

  RuntimeResult renew_before_state_changing_effect(const EffectKind kind)
  {
    switch (kind) {
      case EffectKind::kNavigateToPose:
      case EffectKind::kReleaseSafetyHold:
        break;
      default:
        return ok("effect does not require a pre-effect mode renewal");
    }

    ResourceOwnership mode_ownership = ResourceOwnership::kAbsent;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mode_ownership = mode_lease_ownership_;
    }
    return mode_ownership != ResourceOwnership::kAbsent ?
      renew_mode_lease() : ok("operating mode is not active yet");
  }

  RuntimeResult set_mode(const std::string & mode)
  {
    RuntimeResult result;
    {
      std::lock_guard<std::mutex> operation_lock(mode_lease_operation_mutex_);
      result = set_mode_once(mode, false);
      if (!result.success) {
        bool rotate_absent_lease = false;
        {
          std::lock_guard<std::mutex> lock(binding_mutex_);
          rotate_absent_lease =
            mode_lease_ownership_ == ResourceOwnership::kAbsent &&
            prepared_ && !transaction_id_.empty();
          if (rotate_absent_lease) {
            ++mode_lease_revision_;
            mode_lease_id_ =
              "mode_" + transaction_id_ + "_r" +
              std::to_string(mode_lease_revision_);
          }
        }
        if (rotate_absent_lease) {
          result = set_mode_once(mode, false);
        }
      }
    }
    if (!result.success || mode_keepalive_->running()) {
      return result;
    }

    // A failed worker is deliberately recoverable. Re-applying the exact mode
    // above proves ownership again; only then do we clear the old failure and
    // start a fresh renewal worker.
    mode_keepalive_->stop();
    mode_keepalive_->reset();
    const auto started = mode_keepalive_->start();
    if (started.success) {
      return result;
    }

    RuntimeResult rollback;
    {
      std::lock_guard<std::mutex> operation_lock(mode_lease_operation_mutex_);
      rollback = release_mode_once();
    }
    return failed(
      started.code.empty() ?
      "ELEVATOR_MODE_KEEPALIVE_START_FAILED" : started.code,
      started.detail +
      (rollback.success ?
      "; operating mode rolled back" :
      "; operating-mode rollback failed: " + rollback.detail));
  }

  RuntimeResult set_mode_once(const std::string & mode, const bool renewal)
  {
    if (mode.empty() || mode == "NORMAL") {
      return failed(
        "ELEVATOR_OPERATING_MODE_INVALID",
        "elevator runtime requires a non-NORMAL leased mode");
    }
    std::string mission;
    std::string lease;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mission = mission_id_;
      lease = mode_lease_id_;
      if (
        !renewal || mode_lease_ownership_ != ResourceOwnership::kPresent ||
        mode_ != mode)
      {
        mode_lease_ownership_ = ResourceOwnership::kUnknown;
      }
    }
    auto request = std::make_shared<robot_interfaces::srv::SetMode::Request>();
    request->mode = mode;
    request->owner = kRuntimeOwner;
    request->mission_id = mission;
    request->lease_id = lease;
    request->lease_duration = duration_message(options_.mode_lease_sec);
    request->operation = robot_interfaces::srv::SetMode::Request::OP_SET;
    const auto response =
      call_service<robot_interfaces::srv::SetMode>(mode_client_, request);
    if (!response) {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mode_lease_ownership_ = ResourceOwnership::kUnknown;
      return failed(
        "ELEVATOR_OPERATING_MODE_TIMEOUT",
        "operating mode service did not respond; lease ownership is unknown");
    }
    const bool exact =
      response->state.lease_active &&
      response->state.mode == mode &&
      response->state.owner == kRuntimeOwner &&
      response->state.mission_id == mission &&
      response->state.lease_id == lease;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mode_lease_ownership_ = exact ?
        ResourceOwnership::kPresent :
        (response->state.lease_active ?
        ResourceOwnership::kUnknown : ResourceOwnership::kAbsent);
      mode_ = exact ? mode : "";
    }
    if (!response->success || !exact) {
      return failed(
        "ELEVATOR_OPERATING_MODE_REJECTED", response->message);
    }
    return ok("operating mode lease applied: " + mode);
  }

  RuntimeResult release_mode()
  {
    // Join the renewal worker before release so a late renewal cannot recreate
    // the mode after the release response.
    mode_keepalive_->stop();
    std::lock_guard<std::mutex> operation_lock(mode_lease_operation_mutex_);
    return release_mode_once();
  }

  RuntimeResult release_mode_once()
  {
    std::string mission;
    std::string lease;
    std::uint64_t observation_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mission = mission_id_;
      lease = mode_lease_id_;
      mode_lease_ownership_ = ResourceOwnership::kUnknown;
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      observation_generation = operating_mode_observation_generation_;
    }
    auto request = std::make_shared<robot_interfaces::srv::SetMode::Request>();
    request->owner = kRuntimeOwner;
    request->mission_id = mission;
    request->lease_id = lease;
    request->operation = robot_interfaces::srv::SetMode::Request::OP_RELEASE;
    const auto response =
      call_service<robot_interfaces::srv::SetMode>(mode_client_, request);
    if (!response) {
      return failed(
        "ELEVATOR_OPERATING_MODE_TIMEOUT",
        "operating mode release did not respond; exact lease remains unknown");
    }
    const auto response_received_at_sec = steady_now_sec();
    const bool absent =
      !response->state.lease_active && response->state.mode == "NORMAL";
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mode_lease_ownership_ = absent ?
        ResourceOwnership::kAbsent : ResourceOwnership::kPresent;
      if (absent) {
        mode_.clear();
      }
    }
    if (
      !response->success || !absent)
    {
      return failed(
        "ELEVATOR_OPERATING_MODE_RELEASE_REJECTED", response->message);
    }
    const auto evidence = wait_for_mode_release_evidence(
      observation_generation, response_received_at_sec);
    if (!evidence.success) {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      mode_lease_ownership_ = ResourceOwnership::kUnknown;
      return evidence;
    }
    return ok("operating mode lease released");
  }

  RuntimeResult set_correction_pause(
    const bool acquire,
    const std::string & reason,
    const bool require_floor_handoff = false)
  {
    std::string transaction;
    std::uint64_t observation_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      transaction = transaction_id_;
      correction_pause_ownership_ = ResourceOwnership::kUnknown;
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      observation_generation = correction_pause_observation_generation_;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::SetCorrectionPause::Request>();
    const auto command_sequence = next_hold_command_sequence();
    if (!command_sequence) {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_SEQUENCE_EXHAUSTED",
        "persistent correction-pause command sequence is exhausted");
    }
    request->owner = kRuntimeOwner;
    request->transaction_id = transaction;
    request->reason = reason.empty() ? "elevator_ride" : reason;
    request->operation = acquire ?
      robot_interfaces::srv::SetCorrectionPause::Request::OP_ACQUIRE :
      robot_interfaces::srv::SetCorrectionPause::Request::OP_RELEASE;
    request->command_sequence = *command_sequence;
    const auto response =
      call_service<robot_interfaces::srv::SetCorrectionPause>(
      correction_client_, request);
    if (!response) {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_TIMEOUT",
        "correction pause service did not respond; exact ownership remains "
        "unknown until a higher-sequence release is proven");
    }
    const auto response_received_at_sec = steady_now_sec();
    const auto caller_key = correction_key(kRuntimeOwner, transaction);
    const bool caller_owned =
      contains_key(response->state.lease_keys, caller_key);
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      correction_pause_ownership_ = caller_owned ?
        ResourceOwnership::kPresent : ResourceOwnership::kAbsent;
    }
    if (
      !response->success &&
      response->result_code ==
      robot_interfaces::srv::SetCorrectionPause::Response::
      RESULT_STALE_COMMAND)
    {
      synchronize_hold_command_sequence(response->applied_sequence);
    }
    if (
      !response->success ||
      response->applied_sequence != *command_sequence ||
      (acquire && !caller_owned) ||
      (!acquire && caller_owned))
    {
      return failed(
        "ELEVATOR_CORRECTION_PAUSE_REJECTED",
        response->message + "; requested_sequence=" +
        std::to_string(*command_sequence) + "; applied_sequence=" +
        std::to_string(response->applied_sequence));
    }
    if (!acquire && require_floor_handoff) {
      const auto floor_key = correction_key(kFloorPauseOwner, transaction);
      if (
        !response->state.paused ||
        !contains_key(response->state.lease_keys, floor_key))
      {
        return failed(
          "ELEVATOR_CORRECTION_PAUSE_HANDOFF_LOST",
          "floor manager did not retain the pause during caller handoff");
      }
    }
    if (!acquire) {
      const auto evidence = wait_for_correction_pause_release_evidence(
        transaction, require_floor_handoff, observation_generation,
        response_received_at_sec);
      if (!evidence.success) {
        std::lock_guard<std::mutex> lock(binding_mutex_);
        correction_pause_ownership_ = ResourceOwnership::kUnknown;
        return evidence;
      }
    }
    return ok(
      acquire ?
      "caller localization correction pause acquired" :
      (require_floor_handoff ?
      "caller correction pause handed to floor manager" :
      "caller correction pause released with stale-command fence"));
  }

  RuntimeResult renew_mode_lease_once()
  {
    std::lock_guard<std::mutex> operation_lock(mode_lease_operation_mutex_);
    bool renew_mode = false;
    std::string mode;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      renew_mode = mode_lease_ownership_ == ResourceOwnership::kPresent;
      mode = mode_;
    }
    if (!renew_mode) {
      return ok("operating-mode lease is not active");
    }
    return set_mode_once(mode, true);
  }

  RuntimeResult renew_mode_lease()
  {
    std::string mode;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      if (mode_lease_ownership_ == ResourceOwnership::kAbsent) {
        return ok("operating mode is not active");
      }
      if (mode_.empty()) {
        return failed(
          "ELEVATOR_OPERATING_MODE_OWNERSHIP_UNPROVEN",
          "operating mode cannot be renewed because its exact owner is "
          "unknown");
      }
      mode = mode_;
    }
    return set_mode(mode);
  }

  RuntimeResult wait_for_stop()
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.stop_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      if (
        stop_tracker_.stopped(
          steady_now_sec(),
          options_.stop_feedback_max_age_sec,
          options_.stop_settle_sec))
      {
        return ok("wheel and local odometry prove settled stop");
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_STOP_UNPROVEN",
      "fresh settled wheel/local odometry was not proven");
  }

  void reset_stop_evidence()
  {
    stop_tracker_.reset(
      options_.stop_linear_threshold_mps,
      options_.stop_angular_threshold_radps);
  }

  RuntimeResult cancel_nav_goal_and_wait_terminal()
  {
    std::shared_ptr<NavigateGoalHandle> goal_handle;
    std::shared_future<NavigateGoalHandle::WrappedResult> result_future;
    std::shared_future<std::shared_ptr<NavigateGoalHandle>> response_future;
    bool admission_unknown = false;
    {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      goal_handle = active_nav_goal_;
      result_future = nav_result_future_;
      response_future = nav_goal_response_future_;
      admission_unknown = nav_goal_admission_unknown_;
    }

    if (admission_unknown) {
      if (
        !response_future.valid() ||
        ros_worker_->wait_for(response_future, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          "ELEVATOR_NAV2_GOAL_ADMISSION_UNKNOWN",
          "Nav2 goal response is still unknown; terminal state cannot be "
          "proven");
      }
      try {
        goal_handle = response_future.get();
        if (goal_handle) {
          result_future = nav_client_->async_get_result(goal_handle);
        }
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_NAV2_GOAL_ADMISSION_UNKNOWN", exception.what());
      }
      {
        std::lock_guard<std::mutex> lock(nav_goal_mutex_);
        nav_goal_admission_unknown_ = false;
        if (!goal_handle) {
          nav_goal_response_future_ = {};
          nav_result_future_ = {};
          return ok("late Nav2 response proved that the goal was rejected");
        }
        active_nav_goal_ = goal_handle;
        nav_result_future_ = result_future;
      }
    }

    if (!goal_handle) {
      return ok("no elevator-owned Nav2 goal is active");
    }
    if (!result_future.valid()) {
      try {
        result_future = nav_client_->async_get_result(goal_handle);
        std::lock_guard<std::mutex> lock(nav_goal_mutex_);
        nav_result_future_ = result_future;
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_NAV2_TERMINAL_UNPROVEN", exception.what());
      }
    }

    if (result_future.wait_for(0ms) != std::future_status::ready) {
      std::string cancel_issue;
      try {
        auto cancel_future = nav_client_->async_cancel_goal(goal_handle);
        if (
          ros_worker_->wait_for(cancel_future, timeout(options_.service_timeout_sec)) ==
          std::future_status::ready)
        {
          try {
            const auto response = cancel_future.get();
            if (!response) {
              cancel_issue = "Nav2 returned an empty cancel response";
            } else if (
              response->return_code ==
              action_msgs::srv::CancelGoal::Response::ERROR_NONE &&
              !cancel_response_contains_goal(*response, goal_handle))
            {
              cancel_issue =
                "Nav2 cancel response did not name the exact elevator goal";
            } else if (
              response->return_code !=
              action_msgs::srv::CancelGoal::Response::ERROR_NONE)
            {
              cancel_issue =
                "Nav2 cancel response return_code=" +
                std::to_string(response->return_code);
            }
          } catch (const std::exception & exception) {
            cancel_issue =
              std::string("Nav2 cancel response failed: ") + exception.what();
          } catch (...) {
            cancel_issue = "Nav2 cancel response failed";
          }
        } else {
          cancel_issue = "Nav2 cancel response timed out";
        }
      } catch (const ElevatorRosExecutorUnavailable &) {
        throw;
      } catch (const std::exception & exception) {
        cancel_issue =
          std::string("Nav2 cancel request failed: ") + exception.what();
      } catch (...) {
        cancel_issue = "Nav2 cancel request failed";
      }
      if (
        ros_worker_->wait_for(result_future, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          cancel_issue.empty() ?
          "ELEVATOR_NAV2_TERMINAL_UNPROVEN" :
          "ELEVATOR_NAV2_CANCEL_NOT_ACKNOWLEDGED",
          cancel_issue.empty() ?
          "the elevator-owned Nav2 goal has no explicit terminal result" :
          cancel_issue +
          "; the exact elevator goal also has no explicit terminal result");
      }
    }

    NavigateGoalHandle::WrappedResult wrapped;
    try {
      wrapped = result_future.get();
    } catch (const std::exception & exception) {
      return failed(
        "ELEVATOR_NAV2_TERMINAL_UNPROVEN",
        std::string("Nav2 terminal result failed: ") + exception.what());
    } catch (...) {
      return failed(
        "ELEVATOR_NAV2_TERMINAL_UNPROVEN",
        "Nav2 terminal result failed");
    }
    if (wrapped.code == rclcpp_action::ResultCode::UNKNOWN) {
      return failed(
        "ELEVATOR_NAV2_TERMINAL_UNPROVEN",
        "Nav2 returned UNKNOWN rather than an explicit terminal result");
    }
    {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      if (active_nav_goal_ == goal_handle) {
        active_nav_goal_.reset();
        nav_goal_response_future_ = {};
        nav_result_future_ = {};
      }
    }
    return ok(
      "elevator-owned Nav2 goal reached terminal " +
      result_code_text(wrapped.code));
  }

  RuntimeResult cancel_floor_goal_and_wait_terminal()
  {
    std::shared_ptr<FloorGoalHandle> goal_handle;
    std::shared_future<FloorGoalHandle::WrappedResult> result_future;
    std::shared_future<std::shared_ptr<FloorGoalHandle>> response_future;
    bool admission_unknown = false;
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      goal_handle = active_floor_goal_;
      result_future = floor_result_future_;
      response_future = floor_goal_response_future_;
      admission_unknown = floor_goal_admission_unknown_;
    }

    if (admission_unknown) {
      if (
        !response_future.valid() ||
        ros_worker_->wait_for(response_future, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          "ELEVATOR_FLOOR_GOAL_ADMISSION_UNKNOWN",
          "floor-switch goal response is still unknown; terminal state "
          "cannot be proven");
      }
      try {
        goal_handle = response_future.get();
        if (goal_handle) {
          result_future = floor_client_->async_get_result(goal_handle);
        }
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_FLOOR_GOAL_ADMISSION_UNKNOWN", exception.what());
      }
      {
        std::lock_guard<std::mutex> lock(floor_goal_mutex_);
        floor_goal_admission_unknown_ = false;
        if (!goal_handle) {
          floor_goal_response_future_ = {};
          floor_result_future_ = {};
          return ok(
            "late floor-switch response proved that the goal was rejected");
        }
        active_floor_goal_ = goal_handle;
        floor_result_future_ = result_future;
      }
    }

    if (!goal_handle) {
      return ok("no elevator-owned floor-switch goal is active");
    }
    if (!result_future.valid()) {
      try {
        result_future = floor_client_->async_get_result(goal_handle);
        std::lock_guard<std::mutex> lock(floor_goal_mutex_);
        floor_result_future_ = result_future;
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN", exception.what());
      }
    }

    if (result_future.wait_for(0ms) != std::future_status::ready) {
      std::string cancel_issue;
      try {
        auto cancel_future = floor_client_->async_cancel_goal(goal_handle);
        if (
          ros_worker_->wait_for(cancel_future, timeout(options_.service_timeout_sec)) ==
          std::future_status::ready)
        {
          try {
            const auto response = cancel_future.get();
            if (!response) {
              cancel_issue = "floor manager returned an empty cancel response";
            } else if (
              response->return_code ==
              action_msgs::srv::CancelGoal::Response::ERROR_NONE &&
              !cancel_response_contains_goal(*response, goal_handle))
            {
              cancel_issue =
                "floor cancel response did not name the exact elevator goal";
            } else if (
              response->return_code !=
              action_msgs::srv::CancelGoal::Response::ERROR_NONE)
            {
              cancel_issue =
                "floor cancel response return_code=" +
                std::to_string(response->return_code);
            }
          } catch (const std::exception & exception) {
            cancel_issue =
              std::string("floor cancel response failed: ") + exception.what();
          } catch (...) {
            cancel_issue = "floor cancel response failed";
          }
        } else {
          cancel_issue = "floor cancel response timed out";
        }
      } catch (const ElevatorRosExecutorUnavailable &) {
        throw;
      } catch (const std::exception & exception) {
        cancel_issue =
          std::string("floor cancel request failed: ") + exception.what();
      } catch (...) {
        cancel_issue = "floor cancel request failed";
      }
      if (
        ros_worker_->wait_for(result_future, timeout(options_.service_timeout_sec)) !=
        std::future_status::ready)
      {
        return failed(
          cancel_issue.empty() ?
          "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN" :
          "ELEVATOR_FLOOR_SWITCH_CANCEL_NOT_ACKNOWLEDGED",
          cancel_issue.empty() ?
          "the floor-switch action has no explicit terminal result" :
          cancel_issue +
          "; the exact floor goal also has no explicit terminal result");
      }
    }

    FloorGoalHandle::WrappedResult wrapped;
    try {
      wrapped = result_future.get();
    } catch (const std::exception & exception) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
        std::string("floor terminal result failed: ") + exception.what());
    } catch (...) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
        "floor terminal result failed");
    }
    if (wrapped.code == rclcpp_action::ResultCode::UNKNOWN) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
        "floor manager returned UNKNOWN rather than a terminal result");
    }
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      if (active_floor_goal_ == goal_handle) {
        floor_result_ = wrapped;
        active_floor_goal_.reset();
        floor_goal_response_future_ = {};
      }
    }
    return ok(
      "elevator-owned floor-switch goal reached terminal " +
      result_code_text(wrapped.code));
  }

  RuntimeResult wait_for_motion_allowed(
    const ElevatorMotionAdmissionScope scope,
    const std::uint64_t minimum_generation,
    const std::uint64_t minimum_safety_status_generation,
    const std::uint64_t minimum_interlock_generation,
    const std::shared_future<NavigateGoalHandle::WrappedResult> &
    nav_result_future)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    ElevatorMotionAdmissionAssessment last_assessment;
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      if (
        nav_result_future.valid() &&
        nav_result_future.wait_for(0ms) == std::future_status::ready)
      {
        lock.unlock();
        try {
          const auto wrapped = nav_result_future.get();
          if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return ok(
              "Nav2 completed before a new motion-allowed sample was needed");
          }
          auto terminal_failure = failed(
            "ELEVATOR_NAV2_GOAL_FAILED",
            "Nav2 reached terminal " + result_code_text(wrapped.code) +
            " before robot_safety authorized motion");
          terminal_failure.motion_not_authorized_proven = true;
          return terminal_failure;
        } catch (const std::exception & exception) {
          return failed(
            "ELEVATOR_NAV2_RESULT_UNKNOWN",
            std::string("early Nav2 terminal result failed: ") +
            exception.what());
        } catch (...) {
          return failed(
            "ELEVATOR_NAV2_RESULT_UNKNOWN",
            "early Nav2 terminal result failed");
        }
      }
      std::string mission;
      std::string mode_lease;
      std::string expected_mode;
      {
        std::lock_guard<std::mutex> binding_lock(binding_mutex_);
        mission = mission_id_;
        mode_lease = mode_lease_id_;
        expected_mode = mode_;
      }
      const auto now_sec = steady_now_sec();
      const bool fresh_operating_mode =
        have_operating_mode_state_ &&
        now_sec - operating_mode_received_at_sec_ >= 0.0 &&
        now_sec - operating_mode_received_at_sec_ <=
        options_.state_evidence_max_age_sec;
      const bool exact_mode_contract =
        have_interlock_state_ &&
        !interlock_state_.hold_active &&
        !interlock_state_.motion_blocked &&
        !interlock_state_.interlock_effective_motion_blocked &&
        interlock_state_.execution_mode_contract_valid &&
        interlock_state_.normal_source_only &&
        fresh_operating_mode &&
        operating_mode_state_.lease_active &&
        operating_mode_state_.owner == kRuntimeOwner &&
        operating_mode_state_.mission_id == mission &&
        operating_mode_state_.lease_id == mode_lease &&
        operating_mode_state_.mode == expected_mode;
      ElevatorMotionAdmissionEvidence admission_evidence;
      admission_evidence.motion_allowed_present = have_motion_allowed_;
      admission_evidence.motion_allowed = motion_allowed_;
      admission_evidence.motion_allowed_generation = motion_allowed_generation_;
      admission_evidence.motion_allowed_received_at_sec =
        motion_allowed_received_at_sec_;
      admission_evidence.safety_status_present = have_safety_status_;
      admission_evidence.safety_status = safety_status_;
      admission_evidence.safety_status_generation = safety_status_generation_;
      admission_evidence.safety_status_received_at_sec =
        safety_status_received_at_sec_;
      admission_evidence.interlock_present = have_interlock_state_;
      admission_evidence.interlock_generation =
        interlock_observation_generation_;
      admission_evidence.interlock_received_at_sec = interlock_received_at_sec_;
      if (have_interlock_state_) {
        admission_evidence.interlock_hold_active = interlock_state_.hold_active;
        admission_evidence.interlock_motion_blocked =
          interlock_state_.motion_blocked;
        admission_evidence.interlock_effective_motion_blocked =
          interlock_state_.interlock_effective_motion_blocked;
        admission_evidence.execution_session_engaged =
          interlock_state_.execution_session_engaged;
        admission_evidence.execution_lease_active =
          interlock_state_.execution_lease_active;
        admission_evidence.interlock_block_reason =
          interlock_state_.interlock_effective_block_reason;
      } else {
        admission_evidence.interlock_block_reason =
          "interlock evidence unavailable";
      }
      admission_evidence.exact_elevator_mode_contract = exact_mode_contract;
      last_assessment = assess_elevator_motion_admission(
        scope,
        admission_evidence,
        minimum_generation,
        minimum_safety_status_generation,
        minimum_interlock_generation,
        steady_now_sec(),
        options_.state_evidence_max_age_sec);
      if (last_assessment.ready_to_wait_for_nav2) {
        return ok(last_assessment.detail);
      }
      if (
        last_assessment.kind ==
        ElevatorMotionAdmissionKind::kHardBlocked)
      {
        return failed(
          "ELEVATOR_SAFETY_BLOCKED_AFTER_HOLD_RELEASE",
          last_assessment.detail + "; block_reason=" +
          last_assessment.block_reason);
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    return failed(
      "ELEVATOR_MOTION_ADMISSION_UNPROVEN",
      last_assessment.detail + "; last_reason=" +
      last_assessment.block_reason);
  }

  void publish_elevator_entry_collision_bypass_permit(
    const std::string & transaction_id)
  {
    std_msgs::msg::String message;
    message.data = transaction_id;
    elevator_entry_collision_bypass_permit_pub_->publish(message);
  }

  RuntimeResult navigate(const RuntimeEffect & runtime_effect)
  {
    FrozenRelease release;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      release = release_;
    }
    const auto request =
      resolve_elevator_navigation_request(release, runtime_effect.effect);
    if (!request) {
      return failed(
        "ELEVATOR_NAVIGATION_TARGET_INVALID",
        "frozen pose identity or doorway heading is invalid");
    }
    const bool bypass_collision_monitor =
      elevator_navigation_bypasses_collision_monitor(
      request->navigation_intent);
    bool bypass_permit_active = false;
    auto next_bypass_permit_refresh = std::chrono::steady_clock::now();
    const auto clear_bypass_permit = [this, &bypass_permit_active]() {
      if (!bypass_permit_active) {
        return;
      }
      publish_elevator_entry_collision_bypass_permit("");
      bypass_permit_active = false;
    };
    ScopeExit bypass_permit_cleanup(clear_bypass_permit);
    const auto refresh_bypass_permit =
      [this, &runtime_effect, bypass_collision_monitor,
      &bypass_permit_active, &next_bypass_permit_refresh]()
      {
        if (!bypass_collision_monitor) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (bypass_permit_active && now < next_bypass_permit_refresh) {
          return;
        }
        publish_elevator_entry_collision_bypass_permit(
          runtime_effect.effect.transaction_id);
        bypass_permit_active = true;
        next_bypass_permit_refresh = now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
            options_.elevator_entry_collision_bypass_refresh_sec));
      };

    auto result = set_hold(true, "pre_navigation_goal");
    if (!result.success) {
      return result;
    }
    reset_stop_evidence();
    result = wait_for_stop();
    if (!result.success) {
      return result;
    }
    if (cancel_requested_.load()) {
      return with_safety_proof(
        failed(
          "ELEVATOR_CANCEL_FENCE_ACTIVE",
          "cancellation was requested before the Nav2 goal was submitted"),
        true, true);
    }

    std::optional<ElevatorMapPose> current_map_pose;
    if (options_.current_map_pose_probe) {
      try {
        current_map_pose = options_.current_map_pose_probe();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Elevator current map pose probe failed; keeping base navigation "
          "profile: %s",
          exception.what());
      } catch (...) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Elevator current map pose probe failed; keeping base navigation "
          "profile");
      }
    }
    const auto navigation_profile = select_elevator_navigation_profile(
      *request, current_map_pose,
      options_.nearby_hall_call_scoped_distance_m);
    if (request->target.role == robot_elevator_manager::PoseRole::kHallCall) {
      const double distance = current_map_pose ? std::hypot(
          request->target.x - current_map_pose->x,
          request->target.y - current_map_pose->y) :
        std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(
        node_->get_logger(),
        "Elevator hall-call navigation profile=%s distance=%.3fm "
        "scoped_envelope=%.2fm",
        navigation_profile == ElevatorNavigationProfile::kElevatorReverseDocking ?
        "elevator_reverse_docking" :
        (navigation_profile == ElevatorNavigationProfile::kElevatorScoped ?
        "elevator_scoped" : "ordinary_nav2"),
        distance, options_.nearby_hall_call_scoped_distance_m);
    }

    const auto controller_id = elevator_controller_id_for_profile(
      navigation_profile, request->target.role);
    const auto controller_session_id = controller_id ?
      make_elevator_controller_session_id(
      runtime_effect.effect.transaction_id,
      runtime_effect.effect.sequence) : std::nullopt;
    bool controller_session_active = false;
    std::string controller_session_terminal_reason{
      "navigate_return_before_nav2_terminal"};
    ScopeExit controller_session_cleanup(
      [this, &controller_id, &controller_session_id,
      &controller_session_active, &controller_session_terminal_reason]()
      {
        if (
          !controller_session_active || !controller_id ||
          !controller_session_id)
        {
          return;
        }
        if (ros_worker_->unavailable()) {
          RCLCPP_ERROR(node_->get_logger(),
            "Elevator controller session END unprocessed: local adapter unavailable; "
            "remote goal/session outcome remains unknown");
          return;
        }
        const auto end_result = invoke([&]() {
          return set_elevator_controller_session(
            *controller_id, *controller_session_id,
            ElevatorNavigationSession::Request::OP_END,
            controller_session_terminal_reason);
        });
        if (!end_result.success) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Elevator controller session END failed: controller=%s "
            "session=%s code=%s detail=%s; the next BEGIN remains the "
            "authoritative reset boundary",
            controller_id->c_str(), controller_session_id->c_str(),
            end_result.code.c_str(), end_result.detail.c_str());
        }
        controller_session_active = false;
      });
    if (controller_id) {
      if (!controller_session_id) {
        return failed(
          "ELEVATOR_CONTROLLER_SESSION_ID_INVALID",
          "the elevator transaction/effect sequence cannot identify the "
          "controller action attempt");
      }
      result = set_elevator_controller_session(
        *controller_id, *controller_session_id,
        ElevatorNavigationSession::Request::OP_BEGIN, "");
      if (!result.success) {
        return result;
      }
      controller_session_active = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "Elevator controller session BEGIN accepted: controller=%s "
        "session=%s",
        controller_id->c_str(), controller_session_id->c_str());
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = request->target.x;
    goal.pose.pose.position.y = request->target.y;
    goal.pose.pose.orientation.z = std::sin(request->target.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(request->target.yaw * 0.5);
    if (navigation_profile ==
      ElevatorNavigationProfile::kElevatorReverseEntryStaging)
    {
      goal.behavior_tree =
        options_.elevator_reverse_entry_staging_behavior_tree;
    } else if (
      navigation_profile == ElevatorNavigationProfile::kElevatorCabinDirect)
    {
      goal.behavior_tree = elevator_cabin_behavior_tree_for_role(
        options_.elevator_cabin_entry_direct_behavior_tree, request->target.role);
    } else if (navigation_profile ==
      ElevatorNavigationProfile::kElevatorReverseDocking)
    {
      goal.behavior_tree =
        options_.elevator_reverse_docking_behavior_tree;
    } else if (
      navigation_profile == ElevatorNavigationProfile::kElevatorScoped)
    {
      goal.behavior_tree =
        request->target.role == robot_elevator_manager::PoseRole::kHallCall ?
        options_.elevator_hall_call_scoped_behavior_tree :
        options_.elevator_scoped_behavior_tree;
    }

    std::shared_future<std::shared_ptr<NavigateGoalHandle>> send_future;
    {
      std::lock_guard<std::mutex> submission_lock(submission_mutex_);
      if (cancel_requested_.load()) {
        return with_safety_proof(
          failed(
            "ELEVATOR_CANCEL_FENCE_ACTIVE",
            "cancellation raced with Nav2 goal construction"),
          true, true);
      }
      {
        std::lock_guard<std::mutex> lock(nav_goal_mutex_);
        nav_goal_response_future_ = {};
        nav_goal_admission_unknown_ = true;
      }
      try {
        send_future = nav_client_->async_send_goal(goal);
      } catch (const std::exception & exception) {
        return with_safety_proof(
          failed(
            "ELEVATOR_NAV2_GOAL_ADMISSION_UNKNOWN",
            std::string("Nav2 goal submission threw before admission was ") +
            "proven: " + exception.what()),
          true, true);
      } catch (...) {
        return with_safety_proof(
          failed(
            "ELEVATOR_NAV2_GOAL_ADMISSION_UNKNOWN",
            "Nav2 goal submission threw before admission was proven"),
          true, true);
      }
      {
        std::lock_guard<std::mutex> lock(nav_goal_mutex_);
        nav_goal_response_future_ = send_future;
      }
    }
    if (
      ros_worker_->wait_for(send_future, timeout(options_.service_timeout_sec)) !=
      std::future_status::ready)
    {
      return with_safety_proof(
        failed(
          "ELEVATOR_NAV2_GOAL_RESPONSE_TIMEOUT",
          "Nav2 goal admission is unknown; owner hold remains active"),
        true, true);
    }
    std::shared_ptr<NavigateGoalHandle> goal_handle;
    try {
      goal_handle = send_future.get();
    } catch (const std::exception & exception) {
      return with_safety_proof(
        failed("ELEVATOR_NAV2_GOAL_RESPONSE_FAILED", exception.what()),
        true, true);
    }
    if (!goal_handle) {
      controller_session_terminal_reason = "nav2_goal_rejected";
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      nav_goal_admission_unknown_ = false;
      nav_goal_response_future_ = {};
      return failed(
        "ELEVATOR_NAV2_GOAL_REJECTED",
        "Nav2 rejected the elevator-owned goal while hold remained active");
    }
    const auto nav_result_future = nav_client_->async_get_result(goal_handle);
    {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      active_nav_goal_ = goal_handle;
      nav_result_future_ = nav_result_future;
      nav_goal_admission_unknown_ = false;
    }

    std::uint64_t motion_generation = 0U;
    std::uint64_t safety_status_generation = 0U;
    std::uint64_t interlock_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      motion_generation = motion_allowed_generation_;
      safety_status_generation = safety_status_generation_;
      interlock_generation = interlock_observation_generation_;
    }
    result = set_hold(false, "navigate_goal_accepted");
    if (!result.success) {
      controller_session_terminal_reason = "hold_release_failed";
      (void)set_hold(true, "navigation_hold_release_failed");
      (void)cancel_nav_goal_and_wait_terminal();
      return result;
    }
    refresh_bypass_permit();
    result = wait_for_motion_allowed(
      motion_admission_scope_for_navigation(request->navigation_intent),
      motion_generation,
      safety_status_generation,
      interlock_generation,
      nav_result_future);
    if (!result.success) {
      controller_session_terminal_reason = "motion_admission_failed";
      clear_bypass_permit();
      (void)set_hold(true, "navigation_motion_not_allowed");
      (void)cancel_nav_goal_and_wait_terminal();
      return result;
    }
    refresh_bypass_permit();

    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.navigation_timeout_sec);
    while (ros_worker_->wait_for(nav_result_future, 100ms) != std::future_status::ready) {
      refresh_bypass_permit();
      if (const auto mode_failure = mode_keepalive_failure()) {
        const auto recovered = renew_mode_lease();
        if (!recovered.success) {
          controller_session_terminal_reason = "mode_keepalive_failed";
          clear_bypass_permit();
          const auto hold_result = set_hold(
            true, "navigation_mode_renewal_failed");
          const auto cancel_result = cancel_nav_goal_and_wait_terminal();
          reset_stop_evidence();
          const auto stop_result = wait_for_stop();
          if (!hold_result.success) {
            return hold_result;
          }
          if (!cancel_result.success) {
            return with_safety_proof(cancel_result, true, false);
          }
          if (!stop_result.success) {
            return with_safety_proof(stop_result, true, false);
          }
          return with_safety_proof(
            failed(
              recovered.code.empty() ? mode_failure->code : recovered.code,
              recovered.detail.empty() ? mode_failure->detail :
              recovered.detail),
            true, true);
        }
      }
      if (cancel_requested_.load()) {
        controller_session_terminal_reason = "operator_cancel";
        clear_bypass_permit();
        const auto hold_result = set_hold(true, "navigation_cancel");
        const auto cancel_result = cancel_nav_goal_and_wait_terminal();
        reset_stop_evidence();
        const auto stop_result = wait_for_stop();
        if (!hold_result.success) {
          return hold_result;
        }
        if (!cancel_result.success) {
          return with_safety_proof(cancel_result, true, false);
        }
        if (!stop_result.success) {
          return with_safety_proof(stop_result, true, false);
        }
        return with_safety_proof(
          failed(
            "ELEVATOR_NAV2_CANCELED",
            "elevator-owned Nav2 goal was canceled by operator request"),
          true, true);
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        controller_session_terminal_reason = "navigation_timeout";
        clear_bypass_permit();
        const auto hold_result = set_hold(true, "navigation_timeout");
        const auto cancel_result = cancel_nav_goal_and_wait_terminal();
        reset_stop_evidence();
        const auto stop_result = wait_for_stop();
        if (!hold_result.success) {
          return hold_result;
        }
        if (!cancel_result.success) {
          return with_safety_proof(cancel_result, true, false);
        }
        if (!stop_result.success) {
          return with_safety_proof(stop_result, true, false);
        }
        return with_safety_proof(
          failed(
            "ELEVATOR_NAV2_TIMEOUT",
            "elevator-owned Nav2 goal timed out and reached a terminal state "
            "under hold"),
          true, true);
      }
    }

    NavigateGoalHandle::WrappedResult wrapped;
    try {
      wrapped = nav_result_future.get();
    } catch (const std::exception & exception) {
      controller_session_terminal_reason = "nav2_result_exception";
      clear_bypass_permit();
      (void)set_hold(true, "navigation_result_exception");
      return failed("ELEVATOR_NAV2_RESULT_UNKNOWN", exception.what());
    }
    const bool explicit_terminal =
      wrapped.code != rclcpp_action::ResultCode::UNKNOWN;
    controller_session_terminal_reason =
      "nav2_terminal_" + result_code_text(wrapped.code);
    if (explicit_terminal) {
      std::lock_guard<std::mutex> lock(nav_goal_mutex_);
      if (active_nav_goal_ == goal_handle) {
        active_nav_goal_.reset();
        nav_goal_response_future_ = {};
        nav_result_future_ = {};
      }
    }
    clear_bypass_permit();
    const auto hold_result = set_hold(true, "navigation_terminal");
    reset_stop_evidence();
    const auto stop_result = wait_for_stop();
    if (!hold_result.success) {
      return hold_result;
    }
    if (!stop_result.success) {
      return with_safety_proof(stop_result, true, false);
    }
    if (!explicit_terminal) {
      return failed(
        "ELEVATOR_NAV2_RESULT_UNKNOWN",
        "Nav2 returned UNKNOWN rather than an explicit terminal result");
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return with_safety_proof(
        failed(
          "ELEVATOR_NAV2_GOAL_FAILED",
          "Nav2 terminal result=" + result_code_text(wrapped.code)),
        true, true);
    }
    return with_safety_proof(
      ok("Nav2 goal succeeded; owner hold reacquired and dual odom settled"),
      true, true);
  }

  RuntimeResult begin_floor_switch(const RuntimeEffect & runtime_effect)
  {
    double source_pose_stamp_sec = 0.0;
    if (options_.current_map_pose_probe) {
      try {
        const auto source_pose = options_.current_map_pose_probe();
        if (source_pose && std::isfinite(source_pose->stamp_sec)) {
          source_pose_stamp_sec = source_pose->stamp_sec;
        }
      } catch (...) {
        source_pose_stamp_sec = 0.0;
      }
    }
    std::uint64_t submission_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      if (active_floor_goal_) {
        return failed(
          "ELEVATOR_FLOOR_SWITCH_ALREADY_ACTIVE",
          "a floor switch action is already active");
      }
      floor_goal_response_future_ = {};
      floor_result_future_ = {};
      floor_goal_admission_unknown_ = false;
      submission_generation = ++floor_goal_submission_generation_;
      floor_handoff_tracker_.reset(
        runtime_effect.effect.transaction_id, submission_generation);
      floor_switch_source_pose_stamp_sec_ = source_pose_stamp_sec;
    }

    FloorSwitch::Goal goal;
    goal.transaction_id = runtime_effect.effect.transaction_id;
    goal.building_id = runtime_effect.building_id;
    goal.floor_id = runtime_effect.floor_id;
    goal.map_id = runtime_effect.map_id;
    goal.expected_asset_epoch = runtime_effect.asset_epoch;
    goal.expected_asset_digest = runtime_effect.asset_digest;
    if (cancel_requested_.load()) {
      return failed(
        "ELEVATOR_CANCEL_FENCE_ACTIVE",
        "cancellation was requested before the floor-switch goal was "
        "submitted");
    }

    rclcpp_action::Client<FloorSwitch>::SendGoalOptions send_options;
    send_options.feedback_callback =
      [
        this,
        submission_generation,
        transaction_id = runtime_effect.effect.transaction_id](
        FloorGoalHandle::SharedPtr callback_goal,
        const std::shared_ptr<const FloorSwitch::Feedback> feedback)
      {
        {
          std::lock_guard<std::mutex> lock(floor_goal_mutex_);
          if (
            submission_generation != floor_goal_submission_generation_ ||
            (active_floor_goal_ && active_floor_goal_ != callback_goal) ||
            (!active_floor_goal_ && !floor_goal_admission_unknown_))
          {
            return;
          }
          const auto & feedback_transaction_id =
            feedback->transaction_id.empty() ?
            transaction_id : feedback->transaction_id;
          const bool handoff_ready =
            feedback->caller_pause_handoff_ready ||
            feedback->stage == kPauseHandoffStage ||
            feedback->stage == kVerifyPauseHandoffStage;
          (void)floor_handoff_tracker_.observe_feedback(
            feedback_transaction_id,
            submission_generation,
            feedback->stage_sequence,
            handoff_ready);
        }
        floor_goal_cv_.notify_all();
      };
    std::shared_future<std::shared_ptr<FloorGoalHandle>> send_future;
    {
      std::lock_guard<std::mutex> submission_lock(submission_mutex_);
      if (cancel_requested_.load()) {
        return failed(
          "ELEVATOR_CANCEL_FENCE_ACTIVE",
          "cancellation raced with floor-switch goal construction");
      }
      {
        std::lock_guard<std::mutex> lock(floor_goal_mutex_);
        floor_goal_admission_unknown_ = true;
        floor_goal_response_future_ = {};
      }
      try {
        send_future = floor_client_->async_send_goal(goal, send_options);
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_FLOOR_GOAL_ADMISSION_UNKNOWN",
          std::string("floor-switch submission threw before admission was ") +
          "proven: " + exception.what());
      } catch (...) {
        return failed(
          "ELEVATOR_FLOOR_GOAL_ADMISSION_UNKNOWN",
          "floor-switch submission threw before admission was proven");
      }
      {
        std::lock_guard<std::mutex> lock(floor_goal_mutex_);
        floor_goal_response_future_ = send_future;
      }
    }
    if (
      ros_worker_->wait_for(send_future, timeout(options_.service_timeout_sec)) !=
      std::future_status::ready)
    {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_RESPONSE_TIMEOUT",
        "floor-switch admission is unknown; safety hold remains active");
    }
    std::shared_ptr<FloorGoalHandle> goal_handle;
    try {
      goal_handle = send_future.get();
    } catch (const std::exception & exception) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_RESPONSE_FAILED", exception.what());
    }
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      floor_goal_admission_unknown_ = false;
      floor_goal_response_future_ = {};
      return failed(
        "ELEVATOR_FLOOR_SWITCH_REJECTED",
        "floor manager rejected the exact floor switch goal");
    }
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      active_floor_goal_ = goal_handle;
      floor_result_future_ = floor_client_->async_get_result(goal_handle);
      floor_goal_admission_unknown_ = false;
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.floor_switch_timeout_sec);
    std::unique_lock<std::mutex> lock(floor_goal_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      if (
        floor_result_future_.valid() &&
        floor_result_future_.wait_for(0ms) == std::future_status::ready)
      {
        const auto wrapped = floor_result_future_.get();
        floor_result_ = wrapped;
        (void)floor_handoff_tracker_.observe_terminal(
          runtime_effect.effect.transaction_id, submission_generation);
        return failed(
          "ELEVATOR_FLOOR_SWITCH_PREMATURE_TERMINAL",
          "floor action terminated before correction-pause handoff: " +
          result_code_text(wrapped.code));
      }
      if (floor_handoff_tracker_.snapshot().ready) {
        return ok(
          "floor manager owns pause and invalidated source runtime context");
      }
      if (cancel_requested_.load()) {
        lock.unlock();
        const auto cancel_result = cancel_floor_goal_and_wait_terminal();
        return cancel_result.success ?
          failed(
          "ELEVATOR_FLOOR_SWITCH_CANCELED",
          "floor-switch action reached terminal after operator cancel") :
          cancel_result;
      }
      floor_goal_cv_.wait_for(lock, 100ms);
    }
    lock.unlock();
    const auto cancel_result = cancel_floor_goal_and_wait_terminal();
    if (!cancel_result.success) {
      return cancel_result;
    }
    return failed(
      "ELEVATOR_FLOOR_SWITCH_HANDOFF_TIMEOUT",
      "floor manager did not prove caller pause handoff before reaching a "
      "terminal state");
  }

  RuntimeResult await_floor_switch(const RuntimeEffect & runtime_effect)
  {
    std::shared_future<FloorGoalHandle::WrappedResult> result_future;
    std::shared_ptr<FloorGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      result_future = floor_result_future_;
      goal_handle = active_floor_goal_;
    }
    if (!goal_handle || !result_future.valid()) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_NOT_STARTED",
        "floor switch action was not started by BEGIN_FLOOR_TRANSITION");
    }
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.floor_switch_timeout_sec);
    while (ros_worker_->wait_for(result_future, 100ms) != std::future_status::ready) {
      if (cancel_requested_.load()) {
        const auto cancel_result = cancel_floor_goal_and_wait_terminal();
        return cancel_result.success ?
          failed(
          "ELEVATOR_FLOOR_SWITCH_CANCELED",
          "floor-switch action reached terminal after operator cancel") :
          cancel_result;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        const auto cancel_result = cancel_floor_goal_and_wait_terminal();
        if (!cancel_result.success) {
          return cancel_result;
        }
        return failed(
          "ELEVATOR_FLOOR_SWITCH_TIMEOUT",
          "floor switch timed out and then reached an explicit terminal "
          "state");
      }
    }
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::UNKNOWN) {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_TERMINAL_UNPROVEN",
        "floor manager returned UNKNOWN rather than an explicit terminal "
        "result");
    }
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      floor_result_ = wrapped;
      active_floor_goal_.reset();
      floor_goal_response_future_ = {};
      floor_result_future_ = {};
    }
    if (
      wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped.result || !wrapped.result->success)
    {
      const auto detail = wrapped.result ?
        wrapped.result->message : result_code_text(wrapped.code);
      return failed("ELEVATOR_FLOOR_SWITCH_FAILED", detail);
    }
    if (
      wrapped.result->active_building_id != runtime_effect.building_id ||
      wrapped.result->active_floor_id != runtime_effect.floor_id ||
      wrapped.result->active_map_id != runtime_effect.map_id ||
      wrapped.result->asset_epoch != runtime_effect.asset_epoch ||
      wrapped.result->asset_digest != runtime_effect.asset_digest ||
      !wrapped.result->runtime_context_valid)
    {
      return failed(
        "ELEVATOR_FLOOR_SWITCH_RESULT_IDENTITY_MISMATCH",
        "floor action result did not prove the exact target runtime identity");
    }
    return ok("floor action committed the exact target runtime identity");
  }

  RuntimeResult verify_floor_ready(const RuntimeEffect & runtime_effect)
  {
    {
      std::lock_guard<std::mutex> lock(floor_goal_mutex_);
      if (
        !floor_result_ || !floor_result_->result ||
        floor_result_->code != rclcpp_action::ResultCode::SUCCEEDED ||
        !floor_result_->result->success)
      {
        return failed(
          "ELEVATOR_FLOOR_SWITCH_RESULT_MISSING",
          "successful floor action evidence is missing");
      }
    }
    const auto context =
      read_runtime_map_context_file(options_.runtime_map_context_file);
    if (
      !context || !context->confirmed || context->state != "ready" ||
      context->building_id != runtime_effect.building_id ||
      context->floor_id != runtime_effect.floor_id ||
      context->map_id != runtime_effect.map_id ||
      context->asset_epoch != runtime_effect.asset_epoch ||
      context->asset_digest != runtime_effect.asset_digest)
    {
      return failed(
        "ELEVATOR_TARGET_RUNTIME_CONTEXT_MISMATCH",
        "API runtime map context does not confirm the target floor/map");
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      ros_worker_->check();
      const auto & status = floor_switch_status_;
      const bool exact =
        have_floor_switch_status_ &&
        status.transaction_id == runtime_effect.effect.transaction_id &&
        status.active_building_id == runtime_effect.building_id &&
        status.active_floor_id == runtime_effect.floor_id &&
        status.active_map_id == runtime_effect.map_id &&
        status.asset_epoch == runtime_effect.asset_epoch &&
        status.asset_digest == runtime_effect.asset_digest;
      const bool ready =
        exact && status.state == "COMPLETE" && status.active_context_valid &&
        status.nav_map_ready && status.filters_ready &&
        status.localizer_ready && status.bridge_ready && status.amcl_ready &&
        status.costmaps_ready && status.nav2_ready &&
        status.failure_code == FloorSwitch::Result::NONE;
      if (ready) {
        break;
      }
      evidence_cv_.wait_for(lock, 50ms);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return failed(
        "ELEVATOR_TARGET_FLOOR_READINESS_UNPROVEN",
        "typed floor status did not prove target map, AMCL, bridge, costmaps, and Nav2 ready");
    }
    lock.unlock();

    if (!options_.current_map_pose_probe) {
      return failed(
        "ELEVATOR_TARGET_MAP_POSE_PROBE_UNAVAILABLE",
        "target floor committed but no map-frame robot pose probe is configured");
    }
    double source_pose_stamp_sec = 0.0;
    {
      std::lock_guard<std::mutex> floor_lock(floor_goal_mutex_);
      source_pose_stamp_sec = floor_switch_source_pose_stamp_sec_;
    }
    ElevatorTargetMapPoseTracker pose_tracker;
    pose_tracker.reset(source_pose_stamp_sec);
    const auto pose_deadline =
      std::chrono::steady_clock::now() +
      timeout(options_.endpoint_timeout_sec);
    while (std::chrono::steady_clock::now() < pose_deadline) {
      ros_worker_->check();
      try {
        const auto pose = options_.current_map_pose_probe();
        if (
          pose && pose_tracker.observe(
            *pose,
            steady_now_sec(),
            options_.target_map_pose_freshness_sec,
            options_.target_map_pose_settle_sec,
            options_.target_map_pose_stability_translation_m,
            options_.target_map_pose_stability_yaw_rad))
        {
          return ok(
            "target map, AMCL, bridge, costmaps, Nav2, and fresh stable map-frame robot pose are ready");
        }
      } catch (const std::exception & exception) {
        return failed(
          "ELEVATOR_TARGET_MAP_POSE_PROBE_FAILED", exception.what());
      } catch (...) {
        return failed(
          "ELEVATOR_TARGET_MAP_POSE_PROBE_FAILED",
          "target map pose probe raised an unknown exception");
      }
      std::this_thread::sleep_for(50ms);
    }
    return failed(
      "ELEVATOR_TARGET_MAP_POSE_UNPROVEN",
      pose_tracker.last_reason());
  }

  RuntimeResult hold_and_cancel(const RuntimeEffect & effect)
  {
    const auto & reason = effect.effect.detail;
    options_.motion_admission_fence->close_and_invalidate();
    ScopeExit nonpersistent_fence_release([this]() {
        if (!options_.persistent_recovery_lock_enabled) {
          options_.motion_admission_fence->reopen_and_invalidate();
        }
      });
    std::string transaction;
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      transaction = transaction_id_;
      cleanup_context_.disposition = effect.cleanup_disposition;
      cancel_requested_.store(true);
    }
    request_cancel(transaction);
    auto hold_result = set_hold(
      true, reason.empty() ? "failure_cleanup" : reason);
    if (!hold_result.success) {
      return with_safety_proof(hold_result, false, false);
    }
    const auto nav_result = cancel_nav_goal_and_wait_terminal();
    if (!nav_result.success) {
      return with_safety_proof(nav_result, true, false);
    }
    const auto floor_result = cancel_floor_goal_and_wait_terminal();
    if (!floor_result.success) {
      return with_safety_proof(floor_result, true, false);
    }
    reset_stop_evidence();
    const auto stop_result = wait_for_stop();
    if (!stop_result.success) {
      return with_safety_proof(stop_result, true, false);
    }
    const auto correction_release = set_correction_pause(
      false, reason.empty() ? "failure_cleanup" : reason, false);
    if (!correction_release.success) {
      return with_safety_proof(correction_release, true, true);
    }
    const auto mode_release = release_mode();
    if (!mode_release.success) {
      return with_safety_proof(mode_release, true, true);
    }
    const auto runtime_idle = wait_for_all_runtimes_idle();
    if (!runtime_idle.success) {
      return with_safety_proof(runtime_idle, true, true);
    }
    const auto delayed_effects = prove_no_delayed_side_effect_unknown();
    if (!delayed_effects.success) {
      return with_safety_proof(delayed_effects, true, true);
    }
    bool resources_reconciled = false;
    if (
      effect.cleanup_disposition == CleanupDisposition::kSourceOutside ||
      effect.cleanup_disposition == CleanupDisposition::kTargetOutside)
    {
      const auto resource_absence =
        wait_for_recovery_resource_absence(transaction);
      if (!resource_absence.success) {
        return with_safety_proof(resource_absence, true, true);
      }
      CleanupContext context;
      {
        std::lock_guard<std::mutex> lock(binding_mutex_);
        context = cleanup_context_;
      }
      const auto runtime_identity = validate_live_cleanup_identity(context);
      if (!runtime_identity.success) {
        return with_safety_proof(runtime_identity, true, true);
      }
      resources_reconciled = true;
    }
    reset_stop_evidence();
    const auto final_stop = wait_for_stop();
    if (!final_stop.success) {
      return with_safety_proof(final_stop, true, false);
    }
    const auto hold_evidence = wait_for_owner_hold_evidence();
    if (!hold_evidence.success) {
      return with_safety_proof(hold_evidence, false, true);
    }
    {
      std::lock_guard<std::mutex> lock(binding_mutex_);
      terminal_ = true;
    }
    auto result = with_safety_proof(
      ok("owner hold retained; owned goals terminal; dual odom stopped"),
      true, true);
    result.runtime_resources_reconciled = resources_reconciled;
    return result;
  }

  ElevatorRosRuntimeOptions options_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<ElevatorLeaseKeepalive> mode_keepalive_;
  std::atomic<bool> cancel_requested_{false};
  std::unique_ptr<robot_safety::PersistentSequenceAllocator>
  hold_sequence_allocator_;
  // Serializes the final cancellation check with action submission. Goal
  // mutexes protect the individual action state after this ordering decision.
  mutable std::mutex submission_mutex_;
  // Serializes mode SET, renewal, and RELEASE calls.
  mutable std::mutex mode_lease_operation_mutex_;

  rclcpp::Client<robot_interfaces::srv::SetMotionHold>::SharedPtr hold_client_;
  rclcpp::Client<
    robot_interfaces::srv::ReleaseMotionHoldIfExecutionIdle>::SharedPtr
    recovery_hold_release_client_;
  rclcpp::Client<robot_interfaces::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<robot_interfaces::srv::SetCorrectionPause>::SharedPtr
    correction_client_;
  std::unordered_map<
    std::string,
    rclcpp::Client<ElevatorNavigationSession>::SharedPtr>
    controller_session_clients_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<FloorSwitch>::SharedPtr floor_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    elevator_entry_collision_bypass_permit_pub_;

  mutable std::mutex binding_mutex_;
  bool prepared_{false};
  bool terminal_{false};
  bool restart_cleanup_runtime_superseded_{false};
  bool hold_active_{false};
  ResourceOwnership mode_lease_ownership_{ResourceOwnership::kAbsent};
  ResourceOwnership correction_pause_ownership_{ResourceOwnership::kAbsent};
  std::string transaction_id_;
  std::string mission_id_;
  std::string mode_lease_id_;
  std::uint64_t mode_lease_revision_{0U};
  std::string mode_;
  FrozenRelease release_;
  CleanupContext cleanup_context_;

  mutable std::mutex evidence_mutex_;
  std::condition_variable evidence_cv_;
  bool have_motion_allowed_{false};
  bool motion_allowed_{false};
  std::uint64_t motion_allowed_generation_{0U};
  double motion_allowed_received_at_sec_{0.0};
  bool have_safety_status_{false};
  std::string safety_status_;
  std::uint64_t safety_status_generation_{0U};
  double safety_status_received_at_sec_{0.0};
  bool have_interlock_state_{false};
  double interlock_received_at_sec_{0.0};
  std::uint64_t interlock_observation_generation_{0U};
  robot_interfaces::msg::MotionInterlockState interlock_state_;
  bool have_operating_mode_state_{false};
  double operating_mode_received_at_sec_{0.0};
  std::uint64_t operating_mode_observation_generation_{0U};
  robot_interfaces::msg::OperatingModeState operating_mode_state_;
  bool have_correction_pause_state_{false};
  double correction_pause_received_at_sec_{0.0};
  std::uint64_t correction_pause_observation_generation_{0U};
  robot_interfaces::msg::CorrectionPauseState correction_pause_state_;
  bool have_floor_switch_status_{false};
  double floor_switch_status_received_at_sec_{0.0};
  robot_interfaces::msg::FloorSwitchStatus floor_switch_status_;
  bool have_navigation_status_{false};
  bool navigation_goal_active_{false};
  double navigation_status_received_at_sec_{0.0};
  ElevatorRecoveryActionBarrier recovery_nav_action_barrier_;
  ElevatorRecoveryActionBarrier recovery_floor_action_barrier_;
  bool have_localizer_asset_state_{false};
  robot_interfaces::msg::LocalizerAssetState localizer_asset_state_;
  bool have_localization_health_{false};
  double localization_health_received_at_sec_{0.0};
  robot_interfaces::msg::LocalizationHealth localization_health_;
  bool have_localization_bridge_status_{false};
  double localization_bridge_status_received_at_sec_{0.0};
  std::string localization_bridge_status_;
  DualOdomStopTracker stop_tracker_;

  mutable std::mutex nav_goal_mutex_;
  std::shared_ptr<NavigateGoalHandle> active_nav_goal_;
  std::shared_future<std::shared_ptr<NavigateGoalHandle>>
    nav_goal_response_future_;
  std::shared_future<NavigateGoalHandle::WrappedResult> nav_result_future_;
  bool nav_goal_admission_unknown_{false};
  mutable std::mutex recovery_cancel_mutex_;
  RetainedRecoveryCancelResponse<RecoveryCancelResponse>
    recovery_nav_cancel_all_response_;
  std::atomic<bool> recovery_nav_cancel_all_response_unknown_{false};
  mutable std::mutex floor_goal_mutex_;
  std::condition_variable floor_goal_cv_;
  std::shared_ptr<FloorGoalHandle> active_floor_goal_;
  std::shared_future<std::shared_ptr<FloorGoalHandle>>
    floor_goal_response_future_;
  std::shared_future<FloorGoalHandle::WrappedResult> floor_result_future_;
  bool floor_goal_admission_unknown_{false};
  RetainedRecoveryCancelResponse<RecoveryCancelResponse>
    recovery_floor_cancel_all_response_;
  std::atomic<bool> recovery_floor_cancel_all_response_unknown_{false};
  std::uint64_t floor_goal_submission_generation_{0U};
  std::optional<FloorGoalHandle::WrappedResult> floor_result_;
  FloorSwitchHandoffTracker floor_handoff_tracker_;
  double floor_switch_source_pose_stamp_sec_{0.0};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_allowed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::MotionInterlockState>::SharedPtr
    interlock_sub_;
  rclcpp::Subscription<robot_interfaces::msg::OperatingModeState>::SharedPtr
    mode_sub_;
  rclcpp::Subscription<robot_interfaces::msg::CorrectionPauseState>::SharedPtr
    correction_sub_;
  rclcpp::Subscription<robot_interfaces::msg::FloorSwitchStatus>::SharedPtr
    floor_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
    navigation_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
    floor_action_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizerAssetState>::SharedPtr
    localizer_asset_state_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizationHealth>::SharedPtr
    localization_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    localization_bridge_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
  // Destroy first, including constructor unwinding: stop() may notify the
  // condition variables above even after an earlier explicit stop/join.
  std::unique_ptr<ElevatorRosExecutor> ros_worker_;
};

ElevatorRosRuntimePort::ElevatorRosRuntimePort(
  ElevatorRosRuntimeOptions options)
: implementation_(std::make_unique<Implementation>(std::move(options)))
{
}

ElevatorRosRuntimePort::~ElevatorRosRuntimePort() = default;

robot_elevator_manager::ElevatorRuntimeCapabilities
ElevatorRosRuntimePort::capabilities() const noexcept
{
  return implementation_->capabilities();
}

RuntimeResult ElevatorRosRuntimePort::prepare(
  const std::string & transaction_id,
  const FrozenRelease & release)
{
  return implementation_->invoke([&]() {return implementation_->prepare(transaction_id, release);});
}

RuntimeResult ElevatorRosRuntimePort::apply(const RuntimeEffect & effect)
{
  return implementation_->invoke([&]() {return implementation_->apply(effect);});
}

RuntimeResult ElevatorRosRuntimePort::recover_locked(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & context)
{
  return implementation_->invoke([&]() {return implementation_->recover_locked(context);});
}

RuntimeResult ElevatorRosRuntimePort::finalize_recovery(
  const robot_elevator_manager::ElevatorRuntimeCleanupContext & context)
{
  return implementation_->invoke([&]() {return implementation_->finalize_recovery(context);});
}

void ElevatorRosRuntimePort::request_cancel(
  const std::string & transaction_id) noexcept
{
  implementation_->request_cancel(transaction_id);
}

RuntimeResult ElevatorRosRuntimePort::heartbeat(
  const std::string & transaction_id)
{
  return implementation_->invoke([&]() {return implementation_->heartbeat(transaction_id);});
}

RuntimeResult ElevatorRosRuntimePort::poll_health(
  const std::string & transaction_id)
{
  return implementation_->invoke([&]() {return implementation_->poll_health(transaction_id);});
}

}  // namespace robot_api_server
