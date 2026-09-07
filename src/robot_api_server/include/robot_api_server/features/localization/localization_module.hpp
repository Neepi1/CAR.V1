#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/localization/amcl_runtime_status.hpp"
#include "robot_api_server/features/localization/localization_result_model.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::localization
{

struct TfChainFreshnessSnapshot
{
  bool have_map_to_odom{false};
  bool have_odom_to_base{false};
  bool have_map_pose{false};
  double map_to_odom_age_sec{-1.0};
  double odom_to_base_age_sec{-1.0};
  double map_pose_age_sec{-1.0};
  double map_to_odom_stamp_sec{0.0};
  double odom_to_base_stamp_sec{0.0};
};

struct BridgeStatusSnapshot
{
  bool available{false};
  std::string raw;
  std::chrono::steady_clock::time_point received_at{};
  double age_sec{-1.0};
  bool has_map_to_odom{false};
  std::string map_to_odom_publisher_owner{"unknown"};
  double map_to_odom_age_ms{-1.0};
  double map_odom_publish_loop_hz{0.0};
  double map_odom_publish_gap_ms{-1.0};
  double map_odom_publish_gap_max_ms{0.0};
  double map_odom_publish_callback_duration_us{0.0};
  std::uint64_t map_odom_latest_accepted_sequence{0U};
  std::uint64_t map_odom_last_published_sequence{0U};
  std::uint64_t map_odom_publish_missed_count{0U};
  std::string map_odom_latest_source{"none"};
  bool map_odom_state_valid{false};
  bool map_odom_correction_paused{false};
  std::string correction_pause_reason{"none"};
  bool map_odom_frozen_due_to_pause{false};
  bool publisher_decoupled_from_correction{false};
  bool smoothing_enabled{false};
  bool correction_active{false};
  bool safe_for_goal_start{true};
  std::uint64_t current_sequence{0U};
  std::uint64_t target_sequence{0U};
  std::uint64_t last_accepted_sequence{0U};
  std::uint64_t last_published_sequence{0U};
  double remaining_translation_error_m{0.0};
  double remaining_yaw_error_rad{0.0};
  std::uint64_t last_explicit_relocalization_sequence{0U};
  double last_explicit_relocalization_accept_time{0.0};
  std::string last_explicit_relocalization_source{"none"};
  std::string last_accepted_source{"none"};
  std::string last_reject_reason{"none"};
  double last_accepted_correction_translation_m{0.0};
  double last_accepted_correction_yaw_rad{0.0};
  double last_candidate_correction_translation_m{0.0};
  double last_candidate_correction_yaw_rad{0.0};
  bool amcl_input_enabled{false};
  bool amcl_ready{false};
  bool amcl_degraded{false};
  std::string amcl_degraded_reason;
  std::string amcl_status_source;
  bool amcl_status_file_stale{true};
  double amcl_status_age_ms{-1.0};
  bool amcl_process_ready{false};
  bool amcl_seeded{false};
  bool amcl_seed_response_ok{false};
  bool amcl_nomotion_pose_received{false};
  bool amcl_static_standby{false};
  bool amcl_tracking_ready{false};
  bool amcl_correction_ready{false};
  bool amcl_correction_pending{false};
  bool amcl_not_moving_no_update_ok{false};
  bool amcl_scan_admission_enabled{false};
  bool amcl_scan_admission_alive{false};
  bool amcl_message_filter_drop_detected{false};
  std::string amcl_scan_admission_last_error{"none"};
  bool amcl_post_isaac_refine_enabled{false};
  bool amcl_post_isaac_refine_active{false};
  bool amcl_post_isaac_refine_request_nomotion_update{false};
  bool amcl_post_isaac_refine_nomotion_service_ready{false};
  std::uint64_t amcl_post_isaac_refined_sequence{0U};
  std::uint64_t amcl_post_isaac_refine_candidate_count{0U};
  std::uint64_t amcl_post_isaac_refine_accepted_count{0U};
  std::uint64_t amcl_post_isaac_refine_rejected_count{0U};
  std::uint64_t amcl_post_isaac_refine_waiting_count{0U};
  std::uint64_t amcl_post_isaac_refine_nomotion_request_count{0U};
  std::string amcl_post_isaac_refine_nomotion_state{"idle"};
  std::string amcl_last_reject_reason{"none"};
  double amcl_pose_age_ms{-1.0};
  bool localization_degraded{false};
};

struct FramePoseSnapshot
{
  bool available{false};
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double stamp_sec{0.0};
  double age_sec{-1.0};
  std::chrono::steady_clock::time_point received_at{};
};

struct RelocalizationSettleResult
{
  bool ok{false};
  std::string failure_code;
  std::string detail;
};

struct LocalizationModuleConfig
{
  std::string trigger_service{"/global_localization/trigger"};
  std::string result_topic{"/localization_result"};
  std::string bridge_status_topic{"/localization/bridge_status"};
  std::string bridge_correction_pause_service{
    "/robot_localization_bridge/set_correction_paused"};
  std::string amcl_nomotion_update_service{"/request_nomotion_update"};
  std::filesystem::path amcl_runtime_status_file{"/tmp/njrh_amcl_runtime_status.env"};
  double amcl_runtime_status_ttl_sec{5.0};
  std::string tf_topic{"/tf"};
  std::string tf_static_topic{"/tf_static"};
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
  std::string static_lidar_frame{"lidar_level_link"};
  double tf_pose_max_age_sec{2.0};
  double robot_pose_freshness_sec{0.5};
  double tf_chain_freshness_sec{0.30};
  double tf_chain_settle_timeout_sec{2.0};
  double service_timeout_sec{8.0};
  double trigger_service_timeout_sec{15.0};
  double bridge_acceptance_timeout_sec{3.0};
  double bridge_acceptance_max_distance_m{1.0};
  double bridge_acceptance_max_yaw_rad{0.35};
  double default_relocalization_wait_sec{8.0};
  double recent_result_max_age_sec{5.0};
  bool manual_amcl_refine_enabled{true};
  bool manual_amcl_refine_required{true};
  double manual_amcl_refine_timeout_sec{4.0};
  int manual_amcl_refine_poll_ms{100};
  int manual_amcl_refine_request_period_ms{500};
};

using AdmittedLocalizationOperation = std::function<HttpResponse()>;

struct LocalizationModulePorts
{
  std::function<bool(const std::string &, std::string &)> operation_blocked;
  std::function<void()> side_effect_started;
  std::function<void()> side_effect_resolved;
  std::function<HttpResponse(
      std::uint64_t,
      const std::string &,
      const AdmittedLocalizationOperation &)> run_admitted_operation;
  std::function<RelocalizationSettleResult(
      std::uint64_t,
      const std::string &,
      const std::string &)> wait_for_settle;
  std::function<std::string()> settle_state_json;
};

// Owns the API-facing localization vertical slice and all of its ROS
// observation/control state. It consumes canonical TF but never publishes TF.
class LocalizationModule
{
public:
  LocalizationModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    LocalizationModuleConfig config,
    LocalizationModulePorts ports);
  ~LocalizationModule();

  LocalizationModule(const LocalizationModule &) = delete;
  LocalizationModule & operator=(const LocalizationModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    std::uint64_t motion_admission_epoch);

  void ensure_tf_subscription_active();
  RobotPoseSnapshot current_robot_pose_snapshot() const;
  FramePoseSnapshot latest_pose_snapshot() const;
  FramePoseSnapshot pose_in_frame(const std::string & frame_id) const;
  RobotPoseSnapshot wait_for_current_robot_pose(
    bool require_map_frame,
    std::string & error);

  TfChainFreshnessSnapshot tf_chain_freshness_snapshot() const;
  std::string tf_chain_freshness_detail(
    const TfChainFreshnessSnapshot & snapshot) const;
  bool tf_chain_is_fresh(const TfChainFreshnessSnapshot & snapshot) const;
  bool wait_for_fresh_tf_chain(const std::string & reason, std::string & detail);
  bool base_to_lidar_static_tf_ready() const;

  LocalizationResultSnapshot localization_result_snapshot() const;
  BridgeStatusSnapshot bridge_status_snapshot() const;
  AmclRuntimeStatus read_amcl_runtime_status(double now_sec) const;
  std::string trigger_service_name() const;
  std::string result_topic_name() const;
  std::filesystem::path amcl_runtime_status_file() const;

  bool trigger_localization_and_wait_for_result(
    const std::string & reason,
    std::string & detail,
    double wait_timeout_sec = -1.0,
    std::uint64_t * accepted_sequence = nullptr);
  bool request_bridge_correction_pause(
    bool paused,
    std::string & detail,
    std::chrono::nanoseconds timeout,
    bool enabled = true);
  bool request_amcl_nomotion_update(
    const std::string & context,
    std::string & detail,
    std::chrono::nanoseconds timeout);
  bool wait_for_manual_relocalization_amcl_refine(
    std::uint64_t relocalization_sequence,
    bool refine_requested,
    bool refine_required,
    std::string & detail);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::localization
