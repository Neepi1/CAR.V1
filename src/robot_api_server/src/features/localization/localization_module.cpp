#include "robot_api_server/features/localization/localization_module.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "robot_api_server/features/localization/tf_pose_utils.hpp"

namespace robot_api_server::features::localization
{

using namespace std::chrono_literals;

namespace
{

void require_ports(const LocalizationModulePorts & ports)
{
  if (!ports.operation_blocked ||
    !ports.side_effect_started ||
    !ports.side_effect_resolved ||
    !ports.run_admitted_operation ||
    !ports.wait_for_settle ||
    !ports.settle_state_json)
  {
    throw std::invalid_argument("localization module requires every cross-domain port");
  }
}

std::uint64_t json_uint64_value(
  const std::string & body,
  const std::string & key,
  const std::uint64_t fallback = 0U)
{
  const auto value = json_number_value(body, key);
  if (!value || !std::isfinite(*value) || *value < 0.0) {
    return fallback;
  }
  return static_cast<std::uint64_t>(*value);
}

// A timed-out submission may still reach its ROS server. Deliberately do not
// resolve on destruction: the process-wide admission count remains fail-closed
// until a response proves the outcome.
class PendingSideEffectEvidence
{
public:
  explicit PendingSideEffectEvidence(const LocalizationModulePorts & ports)
  : ports_(ports)
  {
    ports_.side_effect_started();
  }

  void resolve()
  {
    if (!resolved_) {
      ports_.side_effect_resolved();
      resolved_ = true;
    }
  }

private:
  const LocalizationModulePorts & ports_;
  bool resolved_{false};
};

}  // namespace

class LocalizationModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    LocalizationModuleConfig config,
    LocalizationModulePorts ports)
  : node_(node),
    callback_group_(std::move(callback_group)),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
    require_ports(ports_);
    config_.map_frame = normalized_frame_id(config_.map_frame);
    config_.odom_frame = normalized_frame_id(config_.odom_frame);
    config_.base_frame = normalized_frame_id(config_.base_frame);
    config_.static_lidar_frame = normalized_frame_id(config_.static_lidar_frame);

    localization_result_sub_ =
      node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      config_.result_topic,
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handle_localization_result(msg);
      });
    bridge_status_sub_ = node_.create_subscription<std_msgs::msg::String>(
      config_.bridge_status_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_localization_bridge_status(msg);
      });
    tf_static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
      config_.tf_static_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        handle_tf_static_message(msg);
      });

    localization_trigger_client_ =
      node_.create_client<robot_interfaces::srv::TriggerLocalization>(
      config_.trigger_service,
      rmw_qos_profile_services_default,
      callback_group_);
    bridge_correction_pause_client_ = node_.create_client<std_srvs::srv::SetBool>(
      config_.bridge_correction_pause_service,
      rmw_qos_profile_services_default,
      callback_group_);
    amcl_nomotion_update_client_ = node_.create_client<std_srvs::srv::Empty>(
      config_.amcl_nomotion_update_service,
      rmw_qos_profile_services_default,
      callback_group_);
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const std::uint64_t motion_admission_epoch)
  {
    if (request.method != "POST" || request.path != "/api/v1/localization/trigger") {
      return std::nullopt;
    }
    const std::string body = request.body;
    return ports_.run_admitted_operation(
      motion_admission_epoch,
      "manual_localization",
      [this, body]() {return handle_trigger_localization_body(body);});
  }

  void ensure_tf_subscription_active()
  {
    std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
    if (!tf_sub_) {
      tf_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
        config_.tf_topic,
        rclcpp::QoS(100),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          handle_tf_message(msg);
        });
    }
  }

  RobotPoseSnapshot current_robot_pose_snapshot() const
  {
    RobotPoseSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_pose_ || latest_pose_frame_ != config_.map_frame) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.frame_id = latest_pose_frame_;
    snapshot.child_frame_id = config_.base_frame;
    snapshot.x = latest_pose_x_;
    snapshot.y = latest_pose_y_;
    snapshot.yaw = latest_pose_yaw_;
    snapshot.stamp_sec = latest_pose_stamp_sec_;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_pose_received_at_).count();
    return snapshot;
  }

  FramePoseSnapshot latest_pose_snapshot() const
  {
    FramePoseSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_pose_) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.frame_id = latest_pose_frame_;
    snapshot.x = latest_pose_x_;
    snapshot.y = latest_pose_y_;
    snapshot.yaw = latest_pose_yaw_;
    snapshot.stamp_sec = latest_pose_stamp_sec_;
    snapshot.received_at = latest_pose_received_at_;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_pose_received_at_).count();
    return snapshot;
  }

  FramePoseSnapshot pose_in_frame(const std::string & requested_frame) const
  {
    const auto frame = normalized_frame_id(requested_frame);
    FramePoseSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (frame == config_.map_frame && have_pose_ && latest_pose_frame_ == config_.map_frame) {
      snapshot.available = true;
      snapshot.frame_id = config_.map_frame;
      snapshot.x = latest_pose_x_;
      snapshot.y = latest_pose_y_;
      snapshot.yaw = latest_pose_yaw_;
      snapshot.stamp_sec = latest_pose_stamp_sec_;
      snapshot.received_at = latest_pose_received_at_;
    } else if (frame == config_.odom_frame && have_odom_to_base_) {
      snapshot.available = true;
      snapshot.frame_id = config_.odom_frame;
      snapshot.x = latest_odom_to_base_x_;
      snapshot.y = latest_odom_to_base_y_;
      snapshot.yaw = latest_odom_to_base_yaw_;
      snapshot.stamp_sec = latest_odom_to_base_stamp_sec_;
      snapshot.received_at = latest_odom_to_base_received_at_;
    }
    if (snapshot.available) {
      snapshot.age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - snapshot.received_at).count();
    }
    return snapshot;
  }

  RobotPoseSnapshot wait_for_current_robot_pose(
    const bool require_map_frame,
    std::string & error)
  {
    ensure_tf_subscription_active();
    RobotPoseSnapshot snapshot;
    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = current_robot_pose_snapshot();
      if (snapshot.available && snapshot.age_sec <= config_.robot_pose_freshness_sec) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
    if (!snapshot.available || snapshot.age_sec > config_.robot_pose_freshness_sec) {
      error = "no fresh map-frame robot pose";
      snapshot.available = false;
      return snapshot;
    }
    (void)require_map_frame;
    return snapshot;
  }

  TfChainFreshnessSnapshot tf_chain_freshness_snapshot() const
  {
    TfChainFreshnessSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    snapshot.have_map_to_odom = have_map_to_odom_;
    snapshot.have_odom_to_base = have_odom_to_base_;
    snapshot.have_map_pose = have_pose_ && latest_pose_frame_ == config_.map_frame;
    snapshot.map_to_odom_stamp_sec = latest_map_to_odom_stamp_sec_;
    snapshot.odom_to_base_stamp_sec = latest_odom_to_base_stamp_sec_;
    if (have_map_to_odom_) {
      snapshot.map_to_odom_age_sec = std::chrono::duration<double>(
        now - latest_map_to_odom_received_at_).count();
    }
    if (have_odom_to_base_) {
      snapshot.odom_to_base_age_sec = std::chrono::duration<double>(
        now - latest_odom_to_base_received_at_).count();
    }
    if (snapshot.have_map_pose) {
      snapshot.map_pose_age_sec = std::chrono::duration<double>(
        now - latest_pose_received_at_).count();
    }
    return snapshot;
  }

  std::string tf_chain_freshness_detail(
    const TfChainFreshnessSnapshot & snapshot) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "map_to_odom=" << (snapshot.have_map_to_odom ? "yes" : "no")
        << " age=" << snapshot.map_to_odom_age_sec
        << " odom_to_base=" << (snapshot.have_odom_to_base ? "yes" : "no")
        << " age=" << snapshot.odom_to_base_age_sec
        << " map_pose=" << (snapshot.have_map_pose ? "yes" : "no")
        << " age=" << snapshot.map_pose_age_sec
        << " freshness_limit=" << config_.tf_chain_freshness_sec;
    return out.str();
  }

  bool tf_chain_is_fresh(const TfChainFreshnessSnapshot & snapshot) const
  {
    return snapshot.have_map_to_odom && snapshot.have_odom_to_base && snapshot.have_map_pose &&
           snapshot.map_to_odom_age_sec >= 0.0 &&
           snapshot.odom_to_base_age_sec >= 0.0 &&
           snapshot.map_pose_age_sec >= 0.0 &&
           snapshot.map_to_odom_age_sec <= config_.tf_chain_freshness_sec &&
           snapshot.odom_to_base_age_sec <= config_.tf_chain_freshness_sec &&
           snapshot.map_pose_age_sec <= config_.robot_pose_freshness_sec;
  }

  bool wait_for_fresh_tf_chain(const std::string & reason, std::string & detail)
  {
    ensure_tf_subscription_active();
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.tf_chain_settle_timeout_sec));
    std::string last_detail = "no TF sample yet";
    while (std::chrono::steady_clock::now() <= deadline) {
      const auto snapshot = tf_chain_freshness_snapshot();
      last_detail = tf_chain_freshness_detail(snapshot);
      if (tf_chain_is_fresh(snapshot)) {
        detail = "fresh TF chain before " + reason + ": " + last_detail;
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    detail = "timed out waiting for fresh TF chain before " + reason + ": " + last_detail;
    return false;
  }

  bool base_to_lidar_static_tf_ready() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return have_base_to_lidar_static_tf_;
  }

  LocalizationResultSnapshot localization_result_snapshot() const
  {
    LocalizationResultSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_localization_result_) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.seq = latest_localization_result_seq_;
    snapshot.frame_id = latest_localization_result_frame_;
    snapshot.x = latest_localization_result_x_;
    snapshot.y = latest_localization_result_y_;
    snapshot.yaw = latest_localization_result_yaw_;
    snapshot.stamp_sec = latest_localization_result_stamp_sec_;
    snapshot.received_at = latest_localization_result_received_at_;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_localization_result_received_at_).count();
    return snapshot;
  }

  BridgeStatusSnapshot bridge_status_snapshot() const
  {
    std::lock_guard<std::mutex> lock(bridge_status_mutex_);
    BridgeStatusSnapshot snapshot = latest_bridge_status_;
    if (snapshot.available) {
      snapshot.age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - snapshot.received_at).count();
    }
    return snapshot;
  }

  AmclRuntimeStatus read_amcl_runtime_status(const double now_sec) const
  {
    return ::robot_api_server::features::localization::read_amcl_runtime_status(
      config_.amcl_runtime_status_file,
      now_sec,
      config_.amcl_runtime_status_ttl_sec);
  }

  std::string trigger_service_name() const
  {
    return config_.trigger_service;
  }

  std::string result_topic_name() const
  {
    return config_.result_topic;
  }

  std::filesystem::path amcl_runtime_status_file() const
  {
    return config_.amcl_runtime_status_file;
  }

  bool trigger_localization_and_wait_for_result(
    const std::string & reason,
    std::string & detail,
    const double wait_timeout_sec,
    std::uint64_t * accepted_sequence)
  {
    if (ports_.operation_blocked("localization_trigger", detail)) {
      return false;
    }
    const auto trigger_started_at = std::chrono::steady_clock::now();
    if (accepted_sequence != nullptr) {
      *accepted_sequence = 0U;
    }
    const auto bridge_before = bridge_status_snapshot();
    const std::uint64_t previous_explicit_sequence =
      bridge_before.available ? bridge_before.last_explicit_relocalization_sequence : 0U;
    const double timeout_sec = wait_timeout_sec > 0.0 ?
      wait_timeout_sec : config_.default_relocalization_wait_sec;
    const auto trigger_timeout = localization_trigger_service_timeout(timeout_sec);
    if (!localization_trigger_client_->wait_for_service(trigger_timeout)) {
      detail = "service unavailable: " + config_.trigger_service;
      return false;
    }
    if (ports_.operation_blocked("localization_trigger", detail)) {
      return false;
    }

    const std::string force_accept_detail =
      "global localization wrapper owns bridge force-accept for reason=" + reason;
    PendingSideEffectEvidence pending_side_effect(ports_);
    auto request = std::make_shared<robot_interfaces::srv::TriggerLocalization::Request>();
    request->reason = reason;
    auto future = localization_trigger_client_->async_send_request(request);
    if (future.wait_for(trigger_timeout) != std::future_status::ready) {
      detail = "timed out waiting for localization trigger";
      return false;
    }

    const auto response = future.get();
    if (!response->accepted) {
      const bool predispatch_without_unknown_bridge_arm =
        response->message.find("dispatch_state=not_dispatched") != std::string::npos &&
        response->message.find("failure_code=BRIDGE_FORCE_ACCEPT_TIMEOUT") == std::string::npos &&
        response->message.find("failure_code=BRIDGE_FORCE_ACCEPT_FAILED") == std::string::npos;
      if (predispatch_without_unknown_bridge_arm) {
        pending_side_effect.resolve();
      }
      detail = response->message;
      return false;
    }

    std::string result_detail;
    LocalizationResultSnapshot accepted_snapshot;
    if (!wait_for_localization_result_after(
        trigger_started_at, timeout_sec, result_detail, &accepted_snapshot))
    {
      accepted_snapshot = localization_result_snapshot();
      if (accepted_snapshot.available &&
        accepted_snapshot.frame_id == config_.map_frame &&
        accepted_snapshot.age_sec <= config_.recent_result_max_age_sec)
      {
        detail = force_accept_detail + "; " + localization_result_recent_fallback_detail(
          response->message,
          config_.result_topic,
          accepted_snapshot);
        const bool accepted = wait_for_localization_bridge_acceptance(
          accepted_snapshot, detail);
        const auto bridge_after = bridge_status_snapshot();
        const bool explicit_sequence_advanced =
          bridge_after.available &&
          bridge_after.last_explicit_relocalization_sequence > previous_explicit_sequence;
        const bool proven = accepted && explicit_sequence_advanced;
        if (proven) {
          pending_side_effect.resolve();
        }
        if (accepted && !explicit_sequence_advanced) {
          detail += "; map->odom bridge explicit relocalization sequence did not advance";
        }
        if (proven && accepted_sequence != nullptr) {
          *accepted_sequence = bridge_after.last_explicit_relocalization_sequence;
        }
        return proven;
      }
      detail = force_accept_detail + "; " + response->message + "; " + result_detail;
      return false;
    }

    detail = force_accept_detail + "; " + response->message + "; " + result_detail;
    const bool accepted = wait_for_localization_bridge_acceptance(accepted_snapshot, detail);
    const auto bridge_after = bridge_status_snapshot();
    const bool explicit_sequence_advanced =
      bridge_after.available &&
      bridge_after.last_explicit_relocalization_sequence > previous_explicit_sequence;
    const bool proven = accepted && explicit_sequence_advanced;
    if (proven) {
      pending_side_effect.resolve();
    }
    if (accepted && !explicit_sequence_advanced) {
      detail += "; map->odom bridge explicit relocalization sequence did not advance";
    }
    if (proven && accepted_sequence != nullptr) {
      *accepted_sequence = bridge_after.last_explicit_relocalization_sequence;
    }
    return proven;
  }

  bool request_bridge_correction_pause(
    const bool paused,
    std::string & detail,
    const std::chrono::nanoseconds timeout,
    const bool enabled)
  {
    if (!enabled) {
      detail = "global correction pause disabled by config";
      return true;
    }
    if (!bridge_correction_pause_client_->wait_for_service(timeout)) {
      detail = "service unavailable: " + config_.bridge_correction_pause_service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = paused;
    PendingSideEffectEvidence pending_side_effect(ports_);
    auto future = bridge_correction_pause_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = "timeout waiting for service: " + config_.bridge_correction_pause_service;
      return false;
    }
    const auto response = future.get();
    pending_side_effect.resolve();
    detail = response->message;
    return response->success;
  }

  bool request_amcl_nomotion_update(
    const std::string & context,
    std::string & detail,
    const std::chrono::nanoseconds timeout)
  {
    if (!amcl_nomotion_update_client_->wait_for_service(timeout)) {
      detail = "AMCL no-motion update service unavailable for " + context + ": " +
        config_.amcl_nomotion_update_service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    PendingSideEffectEvidence pending_side_effect(ports_);
    auto future = amcl_nomotion_update_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = "timeout waiting for AMCL no-motion update service for " + context + ": " +
        config_.amcl_nomotion_update_service;
      return false;
    }
    (void)future.get();
    pending_side_effect.resolve();
    detail = "AMCL no-motion update requested for " + context + " via " +
      config_.amcl_nomotion_update_service;
    return true;
  }

  bool wait_for_manual_relocalization_amcl_refine(
    const std::uint64_t relocalization_sequence,
    const bool refine_requested,
    const bool refine_required,
    std::string & detail)
  {
    if (!refine_requested) {
      detail = "manual relocalization AMCL refine not requested";
      return true;
    }
    if (relocalization_sequence == 0U) {
      detail = "manual relocalization AMCL refine skipped: no accepted Isaac relocalization sequence";
      return !refine_required;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.manual_amcl_refine_timeout_sec));
    const auto request_period = std::chrono::milliseconds(
      config_.manual_amcl_refine_request_period_ms);
    const auto poll_period = std::chrono::milliseconds(config_.manual_amcl_refine_poll_ms);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_request_at = std::chrono::steady_clock::now();
    int request_count = 0;
    std::string last_request_detail = "not requested";
    std::string last_state = "not checked";

    while (std::chrono::steady_clock::now() < deadline) {
      const auto bridge = bridge_status_snapshot();
      if (!bridge.available || bridge.age_sec > 1.0) {
        std::ostringstream out;
        out << "bridge status unavailable or stale: available="
            << (bridge.available ? "true" : "false")
            << " age_sec=" << bridge.age_sec;
        last_state = out.str();
        std::this_thread::sleep_for(poll_period);
        continue;
      }

      const bool refine_accepted =
        bridge.amcl_post_isaac_refined_sequence == relocalization_sequence;
      const bool refine_fully_applied =
        refine_accepted && bridge.safe_for_goal_start && !bridge.correction_active &&
        bridge.current_sequence == bridge.target_sequence &&
        bridge.last_published_sequence >= bridge.current_sequence;
      if (refine_fully_applied) {
        std::ostringstream out;
        out << "manual relocalization AMCL refine applied sequence=" << relocalization_sequence
            << " source=" << bridge.map_odom_latest_source
            << " candidate_translation_m=" << bridge.last_candidate_correction_translation_m
            << " candidate_yaw_rad=" << bridge.last_candidate_correction_yaw_rad
            << " accepted_count=" << bridge.amcl_post_isaac_refine_accepted_count
            << " bridge_requests=" << bridge.amcl_post_isaac_refine_nomotion_request_count
            << " api_requests=" << request_count
            << " current_sequence=" << bridge.current_sequence
            << " target_sequence=" << bridge.target_sequence
            << " published_sequence=" << bridge.last_published_sequence;
        detail = out.str();
        return true;
      }

      if (!bridge.amcl_input_enabled) {
        detail = "manual relocalization AMCL refine unavailable: AMCL input disabled";
        return !refine_required;
      }
      if (!bridge.amcl_post_isaac_refine_enabled) {
        detail = "manual relocalization AMCL refine unavailable: bridge post-Isaac refine disabled";
        return !refine_required;
      }

      const auto now = std::chrono::steady_clock::now();
      const bool bridge_owns_nomotion_requests =
        bridge.amcl_post_isaac_refine_request_nomotion_update;
      if (now >= next_request_at && bridge.amcl_process_ready && bridge.amcl_seeded &&
        !bridge_owns_nomotion_requests)
      {
        std::string request_detail;
        if (request_amcl_nomotion_update(
            "manual_relocalization_amcl_refine",
            request_detail,
            service_timeout()))
        {
          ++request_count;
        }
        last_request_detail = request_detail;
        next_request_at = now + request_period;
      }

      std::ostringstream out;
      out << "sequence=" << relocalization_sequence
          << " refined_sequence=" << bridge.amcl_post_isaac_refined_sequence
          << " active=" << (bridge.amcl_post_isaac_refine_active ? "true" : "false")
          << " process_ready=" << (bridge.amcl_process_ready ? "true" : "false")
          << " seeded=" << (bridge.amcl_seeded ? "true" : "false")
          << " tracking_ready=" << (bridge.amcl_tracking_ready ? "true" : "false")
          << " pose_age_ms=" << bridge.amcl_pose_age_ms
          << " candidates=" << bridge.amcl_post_isaac_refine_candidate_count
          << " accepted=" << bridge.amcl_post_isaac_refine_accepted_count
          << " rejected=" << bridge.amcl_post_isaac_refine_rejected_count
          << " waiting=" << bridge.amcl_post_isaac_refine_waiting_count
          << " refine_accepted=" << (refine_accepted ? "true" : "false")
          << " correction_active=" << (bridge.correction_active ? "true" : "false")
          << " safe_for_goal_start=" << (bridge.safe_for_goal_start ? "true" : "false")
          << " current_sequence=" << bridge.current_sequence
          << " target_sequence=" << bridge.target_sequence
          << " published_sequence=" << bridge.last_published_sequence
          << " bridge_owns_nomotion=" << (bridge_owns_nomotion_requests ? "true" : "false")
          << " bridge_nomotion_service_ready="
          << (bridge.amcl_post_isaac_refine_nomotion_service_ready ? "true" : "false")
          << " bridge_nomotion_requests=" << bridge.amcl_post_isaac_refine_nomotion_request_count
          << " bridge_nomotion_state=" << bridge.amcl_post_isaac_refine_nomotion_state
          << " last_reject=" << bridge.amcl_last_reject_reason
          << " candidate_translation_m=" << bridge.last_candidate_correction_translation_m
          << " candidate_yaw_rad=" << bridge.last_candidate_correction_yaw_rad
          << " last_request=\"" << last_request_detail << "\"";
      last_state = out.str();
      std::this_thread::sleep_for(poll_period);
    }

    detail = "manual relocalization AMCL refine timed out: " + last_state;
    return !refine_required;
  }

private:
  std::chrono::nanoseconds service_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.service_timeout_sec));
  }

  std::chrono::nanoseconds localization_trigger_service_timeout(
    const double result_wait_timeout_sec) const
  {
    const double result_wait = result_wait_timeout_sec > 0.0 ?
      result_wait_timeout_sec : config_.default_relocalization_wait_sec;
    const double timeout_sec = std::max(
      config_.trigger_service_timeout_sec,
      config_.service_timeout_sec + result_wait);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));
  }

  bool wait_for_localization_result_after(
    const std::chrono::steady_clock::time_point & min_received_at,
    const double timeout_sec,
    std::string & detail,
    LocalizationResultSnapshot * accepted_snapshot)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));
    LocalizationResultSnapshot snapshot;
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = localization_result_snapshot();
      if (snapshot.available && snapshot.received_at >= min_received_at &&
        snapshot.frame_id == config_.map_frame)
      {
        detail = localization_result_success_detail(snapshot);
        if (accepted_snapshot != nullptr) {
          *accepted_snapshot = snapshot;
        }
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    detail = localization_result_wait_failure_detail(
      config_.result_topic, timeout_sec, snapshot);
    return false;
  }

  bool wait_for_localization_bridge_acceptance(
    const LocalizationResultSnapshot & localization,
    std::string & detail)
  {
    if (config_.bridge_acceptance_timeout_sec <= 0.0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.bridge_acceptance_timeout_sec));
    std::string last_detail = "no fresh map-frame robot pose";
    while (std::chrono::steady_clock::now() <= deadline) {
      const auto pose = current_robot_pose_snapshot();
      if (pose.available && pose.frame_id == config_.map_frame) {
        const double distance = std::hypot(pose.x - localization.x, pose.y - localization.y);
        const double yaw_error = std::fabs(normalize_angle(pose.yaw - localization.yaw));
        std::ostringstream out;
        out << std::fixed << std::setprecision(3)
            << "bridge acceptance distance=" << distance
            << " yaw_error=" << yaw_error
            << " pose_age=" << pose.age_sec;
        last_detail = out.str();
        if (distance <= config_.bridge_acceptance_max_distance_m &&
          yaw_error <= config_.bridge_acceptance_max_yaw_rad)
        {
          detail += "; " + last_detail;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    detail += "; localization_result not accepted by map->odom bridge: " + last_detail;
    return false;
  }

  HttpResponse handle_trigger_localization_body(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("robot_api_server");
    const auto wait_timeout = json_number_value(body, "wait_timeout_sec").value_or(
      config_.default_relocalization_wait_sec);
    const bool wait_for_settle = json_bool_value(body, "wait_for_settle", false);
    const bool amcl_refine_requested = json_bool_value(
      body, "amcl_refine", config_.manual_amcl_refine_enabled);
    const bool amcl_refine_required = json_bool_value(
      body, "amcl_refine_required", config_.manual_amcl_refine_required);
    std::string detail;
    std::uint64_t relocalization_sequence = 0U;
    bool ok = trigger_localization_and_wait_for_result(
      reason, detail, wait_timeout, &relocalization_sequence);
    bool amcl_refine_ok = false;
    std::string amcl_refine_detail = "not run";
    if (ok) {
      amcl_refine_ok = wait_for_manual_relocalization_amcl_refine(
        relocalization_sequence,
        amcl_refine_requested,
        amcl_refine_required,
        amcl_refine_detail);
      detail += "; " + amcl_refine_detail;
      if (!amcl_refine_ok) {
        ok = false;
      }
    } else {
      amcl_refine_detail = "not run because Isaac relocalization was not accepted";
    }

    bool settle_ok = false;
    std::string settle_detail = "not requested";
    if (ok && wait_for_settle) {
      const auto settle = ports_.wait_for_settle(
        relocalization_sequence,
        "manual_before_navigation",
        "navigation_resume");
      settle_ok = settle.ok;
      settle_detail = settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail;
      if (!settle.ok) {
        ok = false;
      }
      detail += "; " + settle_detail;
    } else if (ok) {
      detail += "; manual localization correction accepted; post-settle not requested";
    }

    std::ostringstream out;
    out << "{\"ok\":" << (ok ? "true" : "false")
        << ",\"message\":" << json_string(detail)
        << ",\"last_explicit_relocalization_sequence\":" << relocalization_sequence
        << ",\"manual_relocalization_amcl_refine_requested\":"
        << (amcl_refine_requested ? "true" : "false")
        << ",\"manual_relocalization_amcl_refine_required\":"
        << (amcl_refine_required ? "true" : "false")
        << ",\"manual_relocalization_amcl_refine_ok\":"
        << (amcl_refine_ok ? "true" : "false")
        << ",\"manual_relocalization_amcl_refine_detail\":"
        << json_string(amcl_refine_detail)
        << ",\"post_relocalization_settle_requested\":"
        << (wait_for_settle ? "true" : "false")
        << ",\"post_relocalization_settle_ok\":"
        << (settle_ok ? "true" : "false")
        << ",\"post_relocalization_settle_detail\":"
        << json_string(settle_detail)
        << ",\"post_relocalization_settle\":" << ports_.settle_state_json()
        << "}";
    return {ok ? 200 : 503, "application/json", out.str()};
  }

  void handle_localization_result(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++latest_localization_result_seq_;
    latest_localization_result_frame_ = normalized_frame_id(msg->header.frame_id);
    latest_localization_result_x_ = msg->pose.pose.position.x;
    latest_localization_result_y_ = msg->pose.pose.position.y;
    latest_localization_result_yaw_ = quaternion_yaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    latest_localization_result_stamp_sec_ = stamp_to_seconds(msg->header.stamp);
    latest_localization_result_received_at_ = std::chrono::steady_clock::now();
    have_localization_result_ = true;
  }

  void handle_localization_bridge_status(const std_msgs::msg::String::SharedPtr msg)
  {
    BridgeStatusSnapshot snapshot;
    snapshot.available = true;
    snapshot.raw = msg->data;
    snapshot.received_at = std::chrono::steady_clock::now();
    snapshot.has_map_to_odom = json_bool_value(msg->data, "has_map_to_odom", false);
    snapshot.map_to_odom_publisher_owner =
      json_string_value(msg->data, "map_to_odom_publisher_owner").value_or("unknown");
    snapshot.map_to_odom_age_ms =
      json_number_value(msg->data, "map_to_odom_age_ms").value_or(-1.0);
    snapshot.map_odom_publish_loop_hz =
      json_number_value(msg->data, "map_odom_publish_loop_hz").value_or(0.0);
    snapshot.map_odom_publish_gap_ms =
      json_number_value(msg->data, "map_odom_publish_gap_ms").value_or(-1.0);
    snapshot.map_odom_publish_gap_max_ms =
      json_number_value(msg->data, "map_odom_publish_gap_max_ms").value_or(0.0);
    snapshot.map_odom_publish_callback_duration_us =
      json_number_value(msg->data, "map_odom_publish_callback_duration_us").value_or(0.0);
    snapshot.map_odom_latest_accepted_sequence =
      json_uint64_value(msg->data, "map_odom_latest_accepted_sequence");
    snapshot.map_odom_last_published_sequence =
      json_uint64_value(msg->data, "map_odom_last_published_sequence");
    snapshot.map_odom_publish_missed_count =
      json_uint64_value(msg->data, "map_odom_publish_missed_count");
    snapshot.map_odom_latest_source =
      json_string_value(msg->data, "map_odom_latest_source").value_or("none");
    snapshot.map_odom_state_valid =
      json_bool_value(msg->data, "map_odom_state_valid", false);
    snapshot.map_odom_correction_paused =
      json_bool_value(msg->data, "map_odom_correction_paused", false);
    snapshot.correction_pause_reason =
      json_string_value(msg->data, "correction_pause_reason").value_or("none");
    snapshot.map_odom_frozen_due_to_pause =
      json_bool_value(msg->data, "map_odom_frozen_due_to_pause", false);
    snapshot.publisher_decoupled_from_correction =
      json_bool_value(msg->data, "publisher_decoupled_from_correction", false);
    snapshot.smoothing_enabled = json_bool_value(msg->data, "smoothing_enabled", false);
    snapshot.correction_active = json_bool_value(msg->data, "correction_active", false);
    snapshot.safe_for_goal_start = json_bool_value(msg->data, "safe_for_goal_start", true);
    snapshot.current_sequence = json_uint64_value(msg->data, "current_sequence");
    snapshot.target_sequence = json_uint64_value(msg->data, "target_sequence");
    snapshot.last_accepted_sequence = json_uint64_value(msg->data, "last_accepted_sequence");
    snapshot.last_published_sequence = json_uint64_value(msg->data, "last_published_sequence");
    snapshot.remaining_translation_error_m =
      json_number_value(msg->data, "remaining_translation_error_m").value_or(0.0);
    snapshot.remaining_yaw_error_rad =
      json_number_value(msg->data, "remaining_yaw_error_rad").value_or(0.0);
    snapshot.last_explicit_relocalization_sequence =
      json_uint64_value(msg->data, "last_explicit_relocalization_sequence");
    snapshot.last_explicit_relocalization_accept_time =
      json_number_value(msg->data, "last_explicit_relocalization_accept_time").value_or(0.0);
    snapshot.last_explicit_relocalization_source =
      json_string_value(msg->data, "last_explicit_relocalization_source").value_or("none");
    snapshot.last_accepted_source =
      json_string_value(msg->data, "last_accepted_source").value_or("none");
    snapshot.last_reject_reason =
      json_string_value(msg->data, "last_reject_reason").value_or("none");
    snapshot.last_accepted_correction_translation_m =
      json_number_value(msg->data, "last_accepted_correction_translation_m").value_or(0.0);
    snapshot.last_accepted_correction_yaw_rad =
      json_number_value(msg->data, "last_accepted_correction_yaw_rad").value_or(0.0);
    snapshot.last_candidate_correction_translation_m =
      json_number_value(msg->data, "last_candidate_correction_translation_m").value_or(0.0);
    snapshot.last_candidate_correction_yaw_rad =
      json_number_value(msg->data, "last_candidate_correction_yaw_rad").value_or(0.0);
    snapshot.amcl_input_enabled = json_bool_value(msg->data, "amcl_input_enabled", false);
    snapshot.amcl_ready = json_bool_value(msg->data, "amcl_ready", false);
    snapshot.amcl_degraded = json_bool_value(msg->data, "amcl_degraded", false);
    snapshot.amcl_degraded_reason =
      json_string_value(msg->data, "amcl_degraded_reason").value_or("");
    snapshot.amcl_status_source =
      json_string_value(msg->data, "amcl_status_source").value_or("");
    snapshot.amcl_status_file_stale =
      json_bool_value(msg->data, "amcl_status_file_stale", true);
    snapshot.amcl_status_age_ms =
      json_number_value(msg->data, "amcl_status_age_ms").value_or(-1.0);
    snapshot.amcl_process_ready = json_bool_value(msg->data, "amcl_process_ready", false);
    snapshot.amcl_seeded = json_bool_value(msg->data, "amcl_seeded", false);
    snapshot.amcl_seed_response_ok =
      json_bool_value(msg->data, "amcl_seed_response_ok", false);
    snapshot.amcl_nomotion_pose_received =
      json_bool_value(msg->data, "amcl_nomotion_pose_received", false);
    snapshot.amcl_static_standby =
      json_bool_value(msg->data, "amcl_static_standby", false);
    snapshot.amcl_tracking_ready =
      json_bool_value(msg->data, "amcl_tracking_ready", false);
    snapshot.amcl_correction_ready =
      json_bool_value(msg->data, "amcl_correction_ready", false);
    snapshot.amcl_correction_pending =
      json_bool_value(msg->data, "amcl_correction_pending", false);
    snapshot.amcl_not_moving_no_update_ok =
      json_bool_value(msg->data, "amcl_not_moving_no_update_ok", false);
    snapshot.amcl_scan_admission_enabled =
      json_bool_value(msg->data, "amcl_scan_admission_enabled", false);
    snapshot.amcl_scan_admission_alive =
      json_bool_value(msg->data, "amcl_scan_admission_alive", false);
    snapshot.amcl_message_filter_drop_detected =
      json_bool_value(msg->data, "amcl_message_filter_drop_detected", false);
    snapshot.amcl_scan_admission_last_error =
      json_string_value(msg->data, "amcl_scan_admission_last_error").value_or("none");
    snapshot.amcl_post_isaac_refine_enabled =
      json_bool_value(msg->data, "amcl_post_isaac_refine_enabled", false);
    snapshot.amcl_post_isaac_refine_active =
      json_bool_value(msg->data, "amcl_post_isaac_refine_active", false);
    snapshot.amcl_post_isaac_refine_request_nomotion_update =
      json_bool_value(msg->data, "amcl_post_isaac_refine_request_nomotion_update", false);
    snapshot.amcl_post_isaac_refine_nomotion_service_ready =
      json_bool_value(msg->data, "amcl_post_isaac_refine_nomotion_service_ready", false);
    snapshot.amcl_post_isaac_refined_sequence =
      json_uint64_value(msg->data, "amcl_post_isaac_refined_sequence");
    snapshot.amcl_post_isaac_refine_candidate_count =
      json_uint64_value(msg->data, "amcl_post_isaac_refine_candidate_count");
    snapshot.amcl_post_isaac_refine_accepted_count =
      json_uint64_value(msg->data, "amcl_post_isaac_refine_accepted_count");
    snapshot.amcl_post_isaac_refine_rejected_count =
      json_uint64_value(msg->data, "amcl_post_isaac_refine_rejected_count");
    snapshot.amcl_post_isaac_refine_waiting_count =
      json_uint64_value(msg->data, "amcl_post_isaac_refine_waiting_count");
    snapshot.amcl_post_isaac_refine_nomotion_request_count =
      json_uint64_value(msg->data, "amcl_post_isaac_refine_nomotion_request_count");
    snapshot.amcl_post_isaac_refine_nomotion_state =
      json_string_value(msg->data, "amcl_post_isaac_refine_nomotion_state").value_or("idle");
    snapshot.amcl_last_reject_reason =
      json_string_value(msg->data, "amcl_last_reject_reason").value_or("none");
    snapshot.amcl_pose_age_ms = json_number_value(msg->data, "amcl_pose_age_ms").value_or(-1.0);
    snapshot.localization_degraded = json_bool_value(msg->data, "localization_degraded", false);

    std::lock_guard<std::mutex> lock(bridge_status_mutex_);
    latest_bridge_status_ = std::move(snapshot);
  }

  void handle_tf_static_message(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & transform : msg->transforms) {
      const auto parent = normalized_frame_id(transform.header.frame_id);
      const auto child = normalized_frame_id(transform.child_frame_id);
      if (parent == config_.base_frame && child == config_.static_lidar_frame) {
        have_base_to_lidar_static_tf_ = true;
      }
    }
  }

  void handle_tf_message(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool saw_direct_map_to_base = false;
    for (const auto & transform : msg->transforms) {
      const auto parent = normalized_frame_id(transform.header.frame_id);
      const auto child = normalized_frame_id(transform.child_frame_id);
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      const double yaw = quaternion_yaw(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const double stamp_sec = stamp_to_seconds(transform.header.stamp);

      if (parent == config_.map_frame && child == config_.base_frame) {
        latest_pose_frame_ = config_.map_frame;
        latest_pose_x_ = x;
        latest_pose_y_ = y;
        latest_pose_yaw_ = yaw;
        latest_pose_stamp_sec_ = stamp_sec;
        latest_pose_received_at_ = now;
        have_pose_ = true;
        saw_direct_map_to_base = true;
      } else if (parent == config_.map_frame && child == config_.odom_frame) {
        latest_map_to_odom_x_ = x;
        latest_map_to_odom_y_ = y;
        latest_map_to_odom_yaw_ = yaw;
        latest_map_to_odom_stamp_sec_ = stamp_sec;
        latest_map_to_odom_received_at_ = now;
        have_map_to_odom_ = true;
      } else if (parent == config_.odom_frame && child == config_.base_frame) {
        latest_odom_to_base_x_ = x;
        latest_odom_to_base_y_ = y;
        latest_odom_to_base_yaw_ = yaw;
        latest_odom_to_base_stamp_sec_ = stamp_sec;
        latest_odom_to_base_received_at_ = now;
        have_odom_to_base_ = true;
      }
    }

    if (!saw_direct_map_to_base && have_map_to_odom_ && have_odom_to_base_) {
      const double cosine = std::cos(latest_map_to_odom_yaw_);
      const double sine = std::sin(latest_map_to_odom_yaw_);
      latest_pose_frame_ = config_.map_frame;
      latest_pose_x_ = latest_map_to_odom_x_ +
        cosine * latest_odom_to_base_x_ - sine * latest_odom_to_base_y_;
      latest_pose_y_ = latest_map_to_odom_y_ +
        sine * latest_odom_to_base_x_ + cosine * latest_odom_to_base_y_;
      latest_pose_yaw_ = normalize_angle(
        latest_map_to_odom_yaw_ + latest_odom_to_base_yaw_);
      latest_pose_stamp_sec_ = older_nonzero_stamp(
        latest_map_to_odom_stamp_sec_, latest_odom_to_base_stamp_sec_);
      latest_pose_received_at_ =
        latest_map_to_odom_received_at_ < latest_odom_to_base_received_at_ ?
        latest_map_to_odom_received_at_ : latest_odom_to_base_received_at_;
      have_pose_ = true;
    } else if (have_odom_to_base_ && !have_pose_) {
      latest_pose_frame_ = config_.odom_frame;
      latest_pose_x_ = latest_odom_to_base_x_;
      latest_pose_y_ = latest_odom_to_base_y_;
      latest_pose_yaw_ = latest_odom_to_base_yaw_;
      latest_pose_stamp_sec_ = latest_odom_to_base_stamp_sec_;
      latest_pose_received_at_ = latest_odom_to_base_received_at_;
      have_pose_ = true;
    }
  }

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  LocalizationModuleConfig config_;
  LocalizationModulePorts ports_;

  mutable std::mutex subscription_lifecycle_mutex_;
  mutable std::mutex state_mutex_;
  mutable std::mutex bridge_status_mutex_;

  std::string latest_pose_frame_;
  double latest_pose_x_{0.0};
  double latest_pose_y_{0.0};
  double latest_pose_yaw_{0.0};
  double latest_pose_stamp_sec_{0.0};
  bool have_pose_{false};
  std::chrono::steady_clock::time_point latest_pose_received_at_{};

  bool have_map_to_odom_{false};
  double latest_map_to_odom_x_{0.0};
  double latest_map_to_odom_y_{0.0};
  double latest_map_to_odom_yaw_{0.0};
  double latest_map_to_odom_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_map_to_odom_received_at_{};

  bool have_odom_to_base_{false};
  double latest_odom_to_base_x_{0.0};
  double latest_odom_to_base_y_{0.0};
  double latest_odom_to_base_yaw_{0.0};
  double latest_odom_to_base_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_odom_to_base_received_at_{};

  bool have_localization_result_{false};
  std::uint64_t latest_localization_result_seq_{0U};
  std::string latest_localization_result_frame_;
  double latest_localization_result_x_{0.0};
  double latest_localization_result_y_{0.0};
  double latest_localization_result_yaw_{0.0};
  double latest_localization_result_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_localization_result_received_at_{};

  bool have_base_to_lidar_static_tf_{false};
  BridgeStatusSnapshot latest_bridge_status_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_status_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr
    localization_trigger_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bridge_correction_pause_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr amcl_nomotion_update_client_;
};

LocalizationModule::LocalizationModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  LocalizationModuleConfig config,
  LocalizationModulePorts ports)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(config),
      std::move(ports)))
{
}

LocalizationModule::~LocalizationModule() = default;

std::optional<HttpResponse> LocalizationModule::handle_http(
  const HttpRequest & request,
  const std::uint64_t motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

void LocalizationModule::ensure_tf_subscription_active()
{
  impl_->ensure_tf_subscription_active();
}

RobotPoseSnapshot LocalizationModule::current_robot_pose_snapshot() const
{
  return impl_->current_robot_pose_snapshot();
}

FramePoseSnapshot LocalizationModule::latest_pose_snapshot() const
{
  return impl_->latest_pose_snapshot();
}

FramePoseSnapshot LocalizationModule::pose_in_frame(const std::string & frame_id) const
{
  return impl_->pose_in_frame(frame_id);
}

RobotPoseSnapshot LocalizationModule::wait_for_current_robot_pose(
  const bool require_map_frame,
  std::string & error)
{
  return impl_->wait_for_current_robot_pose(require_map_frame, error);
}

TfChainFreshnessSnapshot LocalizationModule::tf_chain_freshness_snapshot() const
{
  return impl_->tf_chain_freshness_snapshot();
}

std::string LocalizationModule::tf_chain_freshness_detail(
  const TfChainFreshnessSnapshot & snapshot) const
{
  return impl_->tf_chain_freshness_detail(snapshot);
}

bool LocalizationModule::tf_chain_is_fresh(
  const TfChainFreshnessSnapshot & snapshot) const
{
  return impl_->tf_chain_is_fresh(snapshot);
}

bool LocalizationModule::wait_for_fresh_tf_chain(
  const std::string & reason,
  std::string & detail)
{
  return impl_->wait_for_fresh_tf_chain(reason, detail);
}

bool LocalizationModule::base_to_lidar_static_tf_ready() const
{
  return impl_->base_to_lidar_static_tf_ready();
}

LocalizationResultSnapshot LocalizationModule::localization_result_snapshot() const
{
  return impl_->localization_result_snapshot();
}

BridgeStatusSnapshot LocalizationModule::bridge_status_snapshot() const
{
  return impl_->bridge_status_snapshot();
}

AmclRuntimeStatus LocalizationModule::read_amcl_runtime_status(const double now_sec) const
{
  return impl_->read_amcl_runtime_status(now_sec);
}

std::string LocalizationModule::trigger_service_name() const
{
  return impl_->trigger_service_name();
}

std::string LocalizationModule::result_topic_name() const
{
  return impl_->result_topic_name();
}

std::filesystem::path LocalizationModule::amcl_runtime_status_file() const
{
  return impl_->amcl_runtime_status_file();
}

bool LocalizationModule::trigger_localization_and_wait_for_result(
  const std::string & reason,
  std::string & detail,
  const double wait_timeout_sec,
  std::uint64_t * accepted_sequence)
{
  return impl_->trigger_localization_and_wait_for_result(
    reason, detail, wait_timeout_sec, accepted_sequence);
}

bool LocalizationModule::request_bridge_correction_pause(
  const bool paused,
  std::string & detail,
  const std::chrono::nanoseconds timeout,
  const bool enabled)
{
  return impl_->request_bridge_correction_pause(paused, detail, timeout, enabled);
}

bool LocalizationModule::request_amcl_nomotion_update(
  const std::string & context,
  std::string & detail,
  const std::chrono::nanoseconds timeout)
{
  return impl_->request_amcl_nomotion_update(context, detail, timeout);
}

bool LocalizationModule::wait_for_manual_relocalization_amcl_refine(
  const std::uint64_t relocalization_sequence,
  const bool refine_requested,
  const bool refine_required,
  std::string & detail)
{
  return impl_->wait_for_manual_relocalization_amcl_refine(
    relocalization_sequence, refine_requested, refine_required, detail);
}

}  // namespace robot_api_server::features::localization
