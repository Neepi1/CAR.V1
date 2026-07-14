#include <algorithm>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
{
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion quat;
  quat.z = std::sin(yaw * 0.5);
  quat.w = std::cos(yaw * 0.5);
  return quat;
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double normalize_yaw(const double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

std::string json_escape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const char c : input) {
    switch (c) {
      case '\\':
        output += "\\\\";
        break;
      case '"':
        output += "\\\"";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        output += c;
        break;
    }
  }
  return output;
}

std::string json_field_value(const std::string & json, const std::string & key)
{
  const std::string quoted_key = "\"" + key + "\"";
  const auto key_pos = json.find(quoted_key);
  if (key_pos == std::string::npos) {
    return "";
  }
  const auto colon_pos = json.find(':', key_pos + quoted_key.size());
  if (colon_pos == std::string::npos) {
    return "";
  }
  auto value_pos = colon_pos + 1;
  while (value_pos < json.size() && std::isspace(static_cast<unsigned char>(json[value_pos]))) {
    ++value_pos;
  }
  if (value_pos >= json.size()) {
    return "";
  }
  if (json[value_pos] == '"') {
    ++value_pos;
    std::string value;
    bool escaped = false;
    for (; value_pos < json.size(); ++value_pos) {
      const char c = json[value_pos];
      if (escaped) {
        value.push_back(c);
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        break;
      } else {
        value.push_back(c);
      }
    }
    return value;
  }
  auto end_pos = value_pos;
  while (end_pos < json.size() && json[end_pos] != ',' && json[end_pos] != '}') {
    ++end_pos;
  }
  auto value = json.substr(value_pos, end_pos - value_pos);
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

double json_double_field(const std::string & json, const std::string & key, const double fallback)
{
  const auto value = json_field_value(json, key);
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

std::uint64_t json_uint64_field(
  const std::string & json,
  const std::string & key,
  const std::uint64_t fallback)
{
  const auto value = json_field_value(json, key);
  if (value.empty()) {
    return fallback;
  }
  try {
    return static_cast<std::uint64_t>(std::stoull(value));
  } catch (const std::exception &) {
    return fallback;
  }
}

bool json_bool_field(const std::string & json, const std::string & key, const bool fallback)
{
  const auto value = json_field_value(json, key);
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return fallback;
}

std::string trim_copy(const std::string & input)
{
  std::size_t begin = 0U;
  while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
    ++begin;
  }
  std::size_t end = input.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1U]))) {
    --end;
  }
  return input.substr(begin, end - begin);
}

std::string unquote_env_value(std::string value)
{
  value = trim_copy(value);
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1U, value.size() - 2U);
  }
  std::string output;
  output.reserve(value.size());
  bool escaped = false;
  for (const char c : value) {
    if (escaped) {
      output.push_back(c);
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else {
      output.push_back(c);
    }
  }
  if (escaped) {
    output.push_back('\\');
  }
  return output;
}

bool env_bool_field(
  const std::unordered_map<std::string, std::string> & fields,
  const std::string & key,
  const bool fallback)
{
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  if (it->second == "true" || it->second == "True" || it->second == "1") {
    return true;
  }
  if (it->second == "false" || it->second == "False" || it->second == "0") {
    return false;
  }
  return fallback;
}

int env_int_field(
  const std::unordered_map<std::string, std::string> & fields,
  const std::string & key,
  const int fallback)
{
  const auto it = fields.find(key);
  if (it == fields.end() || it->second.empty()) {
    return fallback;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

double env_double_field(
  const std::unordered_map<std::string, std::string> & fields,
  const std::string & key,
  const double fallback)
{
  const auto it = fields.find(key);
  if (it == fields.end() || it->second.empty()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

struct MapToOdom
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CandidateCorrection
{
  MapToOdom transform;
  geometry_msgs::msg::PoseWithCovarianceStamped map_base_pose;
  double correction_translation_m{0.0};
  double correction_yaw_rad{0.0};
  double result_age_ms{-1.0};
  double gate_result_age_limit_ms{-1.0};
  double xy_covariance{-1.0};
  double yaw_covariance{-1.0};
  std::string gate_mode{"triggered"};
  std::string source{"isaac_triggered"};
  std::string reject_reason;
  bool explicit_trigger{false};
  bool valid{false};
};

struct MapOdomState
{
  MapToOdom current_transform;
  MapToOdom target_transform;
  MapToOdom transform;
  double current_z{0.0};
  double target_z{0.0};
  double z{0.0};
  std::uint64_t current_sequence{0U};
  std::uint64_t target_sequence{0U};
  std::uint64_t last_accepted_sequence{0U};
  std::uint64_t last_published_sequence{0U};
  std::uint64_t sequence{0U};
  double accepted_time_sec{0.0};
  double last_correction_accept_time{0.0};
  double last_correction_apply_time{0.0};
  std::string current_source{"none"};
  std::string target_source{"none"};
  std::string source{"none"};
  std::string last_correction_source{"none"};
  double correction_translation_m{0.0};
  double correction_yaw_rad{0.0};
  double remaining_translation_error_m{0.0};
  double remaining_yaw_error_rad{0.0};
  double last_step_translation_m{0.0};
  double last_step_yaw_rad{0.0};
  double smoothing_translation_rate_mps{0.0};
  double smoothing_yaw_rate_radps{0.0};
  double last_correction_delta_translation_m{0.0};
  double last_correction_delta_yaw_rad{0.0};
  std::string smoothing_policy{"default"};
  bool valid{false};
  bool correction_paused{false};
  bool frozen_due_to_pause{false};
  bool correction_active{false};
  bool safe_for_goal_start{true};
  bool smoothing_enabled{false};
  bool large_correction_requires_recovery{true};
};

struct AmclRuntimeStatus
{
  bool available{false};
  std::string mode;
  std::string state;
  std::string start_result;
  bool ready{false};
  bool degraded{false};
  std::string degraded_reason;
  bool process_alive{false};
  bool node_exists{false};
  bool lifecycle_active{false};
  bool scan_admission_alive{false};
  int pose_publisher_count{0};
  int scan_admission_status_publisher_count{0};
  double last_pose_age_ms{-1.0};
  bool seed_succeeded{false};
  bool seed_response_ok{false};
  bool nomotion_probe_used{false};
  bool nomotion_pose_received{false};
  int nomotion_pose_count{0};
  double nomotion_pose_header_age_ms{-1.0};
  bool process_ready{false};
  bool seeded{false};
  bool static_standby{false};
  bool tracking_ready{false};
  bool correction_ready{false};
  bool not_moving_no_update_ok{false};
  std::string stamp;
  double stamp_sec{-1.0};
  double age_ms{-1.0};
  bool stale{true};
};

}  // namespace

class LocalizationBridgeNode : public rclcpp::Node
{
public:
  LocalizationBridgeNode()
  : Node("robot_localization_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    jump_threshold_m_ = declare_parameter<double>("jump_threshold_m", 1.0);
    forced_jump_threshold_m_ = declare_parameter<double>("forced_jump_threshold_m", 20.0);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    tf_future_stamp_offset_sec_ = declare_parameter<double>("tf_future_stamp_offset_sec", 0.0);
    localization_topic_ = declare_parameter<std::string>("localization_topic", "/localization_result");
    local_odom_topic_ = declare_parameter<std::string>("local_odom_topic", "/local_state/odometry");
    health_topic_ = declare_parameter<std::string>("health_topic", "/localization/health");
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/bridge_status");
    force_accept_service_ = declare_parameter<std::string>(
      "force_accept_service", "/robot_localization_bridge/force_accept_next_localization");
    correction_pause_service_ = declare_parameter<std::string>(
      "correction_pause_service", "/robot_localization_bridge/set_correction_paused");
    two_d_mode_ = declare_parameter<bool>("two_d_mode", true);
    const bool deprecated_continuous_localization_enabled = declare_parameter<bool>(
      "continuous_localization_enabled", false);
    continuous_localization_mode_ = declare_parameter<std::string>(
      "continuous_localization_mode", "triggered");
    const double legacy_max_result_age_ms =
      declare_parameter<double>("max_result_age_ms", 5000.0);
    triggered_max_result_age_ms_ = declare_parameter<double>(
      "triggered_max_result_age_ms", legacy_max_result_age_ms);
    force_accept_min_pose_stamp_slack_sec_ = declare_parameter<double>(
      "force_accept_min_pose_stamp_slack_sec", 1.0);
    max_odom_tf_age_ms_ = declare_parameter<double>("max_odom_tf_age_ms", 100.0);
    odom_tf_lookup_timeout_ms_ = declare_parameter<double>("odom_tf_lookup_timeout_ms", 20.0);
    triggered_allow_large_correction_ = declare_parameter<bool>(
      "triggered_allow_large_correction", true);
    triggered_hard_reject_translation_m_ = declare_parameter<double>(
      "triggered_hard_reject_translation_m", forced_jump_threshold_m_);
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    amcl_runtime_status_file_ = declare_parameter<std::string>(
      "amcl_runtime_status_file", "/tmp/njrh_amcl_runtime_status.env");
    amcl_runtime_status_ttl_sec_ = declare_parameter<double>("amcl_runtime_status_ttl_sec", 5.0);
    amcl_input_enabled_ = declare_parameter<bool>("amcl_input_enabled", false);
    amcl_gate_mode_ = declare_parameter<std::string>("amcl_gate_mode", "shadow");
    amcl_max_result_age_ms_ = declare_parameter<double>("amcl_max_result_age_ms", 1000.0);
    amcl_small_correction_translation_m_ = declare_parameter<double>(
      "amcl_small_correction_translation_m", 0.07);
    amcl_small_correction_yaw_rad_ = declare_parameter<double>(
      "amcl_small_correction_yaw_rad", 0.20);
    amcl_medium_correction_translation_m_ = declare_parameter<double>(
      "amcl_medium_correction_translation_m", 0.15);
    amcl_medium_correction_yaw_rad_ = declare_parameter<double>(
      "amcl_medium_correction_yaw_rad", amcl_small_correction_yaw_rad_);
    const int legacy_large_consistency_count = declare_parameter<int>(
      "amcl_large_correction_consistency_count", 3);
    amcl_medium_correction_consistency_count_ = declare_parameter<int>(
      "amcl_medium_correction_consistency_count", legacy_large_consistency_count);
    amcl_accept_corrections_while_moving_ = declare_parameter<bool>(
      "amcl_accept_corrections_while_moving", true);
    amcl_moving_linear_speed_mps_ = declare_parameter<double>(
      "amcl_moving_linear_speed_mps", 0.02);
    amcl_moving_angular_speed_radps_ = declare_parameter<double>(
      "amcl_moving_angular_speed_radps", 0.02);
    amcl_hard_reject_translation_m_ = declare_parameter<double>(
      "amcl_hard_reject_translation_m", 0.30);
    amcl_hard_reject_yaw_rad_ = declare_parameter<double>(
      "amcl_hard_reject_yaw_rad", 0.8);
    amcl_covariance_gate_enabled_ = declare_parameter<bool>(
      "amcl_min_pose_covariance_ok", true);
    amcl_max_xy_covariance_ = declare_parameter<double>("amcl_max_xy_covariance", 1.0);
    amcl_max_yaw_covariance_ = declare_parameter<double>("amcl_max_yaw_covariance", 0.5);
    amcl_accept_after_isaac_delay_sec_ = declare_parameter<double>(
      "amcl_accept_when_isaac_recently_triggered_delay_sec", 2.0);
    amcl_post_isaac_refine_enabled_ = declare_parameter<bool>(
      "amcl_post_isaac_refine_enabled", true);
    amcl_post_isaac_refine_window_sec_ = declare_parameter<double>(
      "amcl_post_isaac_refine_window_sec", 10.0);
    amcl_post_isaac_refine_max_translation_m_ = declare_parameter<double>(
      "amcl_post_isaac_refine_max_translation_m", 0.12);
    amcl_post_isaac_refine_max_yaw_rad_ = declare_parameter<double>(
      "amcl_post_isaac_refine_max_yaw_rad", 0.10);
    amcl_post_isaac_refine_consistency_count_ = declare_parameter<int>(
      "amcl_post_isaac_refine_consistency_count", 2);
    amcl_post_isaac_refine_agreement_translation_m_ = declare_parameter<double>(
      "amcl_post_isaac_refine_agreement_translation_m", 0.08);
    amcl_post_isaac_refine_agreement_yaw_rad_ = declare_parameter<double>(
      "amcl_post_isaac_refine_agreement_yaw_rad", 0.08);
    amcl_post_isaac_refine_require_stationary_ = declare_parameter<bool>(
      "amcl_post_isaac_refine_require_stationary", true);
    amcl_initial_pose_topic_ = declare_parameter<std::string>(
      "amcl_initial_pose_topic", "/initialpose");
    amcl_initial_pose_seed_enabled_ = declare_parameter<bool>(
      "amcl_initial_pose_seed_enabled", true);
    amcl_initial_pose_xy_covariance_ = declare_parameter<double>(
      "amcl_initial_pose_xy_covariance", 0.01);
    amcl_initial_pose_yaw_covariance_ = declare_parameter<double>(
      "amcl_initial_pose_yaw_covariance", 0.0076);
    amcl_seed_service_ = declare_parameter<std::string>(
      "amcl_seed_service", "/robot_localization_bridge/seed_amcl_initial_pose");
    amcl_pose_max_age_ms_ = declare_parameter<double>("amcl_pose_max_age_ms", 1000.0);
    amcl_initial_pose_publish_repetitions_ = declare_parameter<int>(
      "amcl_initial_pose_publish_repetitions", 3);
    amcl_initial_pose_repeat_period_ms_ = declare_parameter<int>(
      "amcl_initial_pose_repeat_period_ms", 100);
    amcl_scan_admission_enabled_ = declare_parameter<bool>(
      "amcl_scan_admission_enabled", false);
    amcl_scan_admission_status_topic_ = declare_parameter<std::string>(
      "amcl_scan_admission_status_topic", "/amcl_scan_admission/status");
    status_publish_period_sec_ = declare_parameter<double>("status_publish_period_sec", 1.0);
    map_odom_publish_gap_warn_ms_ =
      declare_parameter<double>("map_odom_publish_gap_warn_ms", 100.0);
    map_odom_publish_gap_fail_ms_ =
      declare_parameter<double>("map_odom_publish_gap_fail_ms", 250.0);
    map_odom_smoothing_enabled_ = declare_parameter<bool>("map_odom_smoothing_enabled", true);
    map_odom_smoothing_publish_rate_hz_ =
      declare_parameter<double>("map_odom_smoothing_publish_rate_hz", publish_rate_hz_);
    map_odom_smoothing_translation_rate_mps_ =
      declare_parameter<double>("map_odom_smoothing_translation_rate_mps", 0.20);
    map_odom_smoothing_yaw_rate_radps_ =
      declare_parameter<double>("map_odom_smoothing_yaw_rate_radps", 0.25);
    map_odom_smoothing_snap_translation_epsilon_m_ =
      declare_parameter<double>("map_odom_smoothing_snap_translation_epsilon_m", 0.005);
    map_odom_smoothing_snap_yaw_epsilon_rad_ =
      declare_parameter<double>("map_odom_smoothing_snap_yaw_epsilon_rad", 0.005);
    explicit_relocalization_fast_smoothing_enabled_ =
      declare_parameter<bool>("explicit_relocalization_fast_smoothing_enabled", true);
    explicit_relocalization_fast_correction_translation_m_ =
      declare_parameter<double>("explicit_relocalization_fast_correction_translation_m", 1.0);
    explicit_relocalization_fast_correction_yaw_rad_ =
      declare_parameter<double>("explicit_relocalization_fast_correction_yaw_rad", 0.35);
    explicit_relocalization_fast_max_duration_sec_ =
      declare_parameter<double>("explicit_relocalization_fast_max_duration_sec", 3.0);
    map_odom_large_correction_translation_m_ =
      declare_parameter<double>("map_odom_large_correction_translation_m", 0.50);
    map_odom_large_correction_yaw_rad_ =
      declare_parameter<double>("map_odom_large_correction_yaw_rad", 0.35);
    map_odom_large_correction_requires_recovery_ =
      declare_parameter<bool>("map_odom_large_correction_requires_recovery", true);
    map_odom_online_hard_reject_translation_m_ =
      declare_parameter<double>("map_odom_online_hard_reject_translation_m", 0.80);
    map_odom_online_hard_reject_yaw_rad_ =
      declare_parameter<double>("map_odom_online_hard_reject_yaw_rad", 0.80);
    require_result_frame_match_ = declare_parameter<bool>("require_result_frame_match", true);

    if (continuous_localization_mode_ != "triggered") {
      RCLCPP_WARN(
        get_logger(),
        "unsupported continuous_localization_mode '%s'; Isaac continuous localization has been removed, falling back to triggered",
        continuous_localization_mode_.c_str());
      continuous_localization_mode_ = "triggered";
    }
    if (amcl_gate_mode_ != "shadow" && amcl_gate_mode_ != "gated") {
      RCLCPP_WARN(
        get_logger(),
        "unknown amcl_gate_mode '%s'; falling back to shadow",
        amcl_gate_mode_.c_str());
      amcl_gate_mode_ = "shadow";
    }
    if (deprecated_continuous_localization_enabled) {
      RCLCPP_WARN(
        get_logger(),
        "continuous_localization_enabled is ignored; use AMCL shadow/gated modes for continuous correction candidates");
    }
    if (forced_jump_threshold_m_ < jump_threshold_m_) {
      RCLCPP_WARN(
        get_logger(),
        "forced_jump_threshold_m %.3f is below jump_threshold_m %.3f; clamping to normal threshold",
        forced_jump_threshold_m_,
        jump_threshold_m_);
      forced_jump_threshold_m_ = jump_threshold_m_;
    }
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    tf_future_stamp_offset_sec_ = std::clamp(tf_future_stamp_offset_sec_, 0.0, 0.20);
    triggered_max_result_age_ms_ = std::max(1.0, triggered_max_result_age_ms_);
    force_accept_min_pose_stamp_slack_sec_ =
      std::max(0.0, force_accept_min_pose_stamp_slack_sec_);
    max_odom_tf_age_ms_ = std::max(1.0, max_odom_tf_age_ms_);
    odom_tf_lookup_timeout_ms_ = std::max(0.0, odom_tf_lookup_timeout_ms_);
    triggered_hard_reject_translation_m_ =
      std::max(jump_threshold_m_, triggered_hard_reject_translation_m_);
    amcl_max_result_age_ms_ = std::max(1.0, amcl_max_result_age_ms_);
    amcl_small_correction_translation_m_ =
      std::max(0.0, amcl_small_correction_translation_m_);
    amcl_small_correction_yaw_rad_ = std::max(0.0, amcl_small_correction_yaw_rad_);
    amcl_medium_correction_translation_m_ =
      std::max(amcl_small_correction_translation_m_, amcl_medium_correction_translation_m_);
    amcl_medium_correction_yaw_rad_ =
      std::max(amcl_small_correction_yaw_rad_, amcl_medium_correction_yaw_rad_);
    amcl_medium_correction_consistency_count_ =
      std::max(1, amcl_medium_correction_consistency_count_);
    amcl_moving_linear_speed_mps_ = std::max(0.0, amcl_moving_linear_speed_mps_);
    amcl_moving_angular_speed_radps_ = std::max(0.0, amcl_moving_angular_speed_radps_);
    amcl_hard_reject_translation_m_ =
      std::max(amcl_medium_correction_translation_m_, amcl_hard_reject_translation_m_);
    amcl_hard_reject_yaw_rad_ =
      std::max(amcl_small_correction_yaw_rad_, amcl_hard_reject_yaw_rad_);
    amcl_max_xy_covariance_ = std::max(0.0, amcl_max_xy_covariance_);
    amcl_max_yaw_covariance_ = std::max(0.0, amcl_max_yaw_covariance_);
    amcl_accept_after_isaac_delay_sec_ =
      std::max(0.0, amcl_accept_after_isaac_delay_sec_);
    amcl_post_isaac_refine_window_sec_ = std::max(0.0, amcl_post_isaac_refine_window_sec_);
    amcl_post_isaac_refine_max_translation_m_ =
      std::max(0.0, amcl_post_isaac_refine_max_translation_m_);
    amcl_post_isaac_refine_max_yaw_rad_ =
      std::max(0.0, amcl_post_isaac_refine_max_yaw_rad_);
    amcl_post_isaac_refine_consistency_count_ =
      std::max(1, amcl_post_isaac_refine_consistency_count_);
    amcl_post_isaac_refine_agreement_translation_m_ =
      std::max(0.0, amcl_post_isaac_refine_agreement_translation_m_);
    amcl_post_isaac_refine_agreement_yaw_rad_ =
      std::max(0.0, amcl_post_isaac_refine_agreement_yaw_rad_);
    amcl_initial_pose_xy_covariance_ = std::max(0.0, amcl_initial_pose_xy_covariance_);
    amcl_initial_pose_yaw_covariance_ =
      std::max(0.0, amcl_initial_pose_yaw_covariance_);
    amcl_pose_max_age_ms_ = std::max(1.0, amcl_pose_max_age_ms_);
    amcl_initial_pose_publish_repetitions_ =
      std::clamp(amcl_initial_pose_publish_repetitions_, 1, 5);
    amcl_initial_pose_repeat_period_ms_ =
      std::clamp(amcl_initial_pose_repeat_period_ms_, 0, 1000);
    status_publish_period_sec_ = std::max(0.2, status_publish_period_sec_);
    amcl_runtime_status_ttl_sec_ = std::max(0.0, amcl_runtime_status_ttl_sec_);
    map_odom_publish_gap_warn_ms_ = std::max(1.0, map_odom_publish_gap_warn_ms_);
    map_odom_publish_gap_fail_ms_ =
      std::max(map_odom_publish_gap_warn_ms_, map_odom_publish_gap_fail_ms_);
    map_odom_smoothing_translation_rate_mps_ =
      std::max(0.001, map_odom_smoothing_translation_rate_mps_);
    map_odom_smoothing_publish_rate_hz_ =
      std::max(1.0, map_odom_smoothing_publish_rate_hz_);
    map_odom_smoothing_yaw_rate_radps_ =
      std::max(0.001, map_odom_smoothing_yaw_rate_radps_);
    map_odom_smoothing_snap_translation_epsilon_m_ =
      std::max(0.0, map_odom_smoothing_snap_translation_epsilon_m_);
    map_odom_smoothing_snap_yaw_epsilon_rad_ =
      std::max(0.0, map_odom_smoothing_snap_yaw_epsilon_rad_);
    explicit_relocalization_fast_correction_translation_m_ =
      std::max(0.0, explicit_relocalization_fast_correction_translation_m_);
    explicit_relocalization_fast_correction_yaw_rad_ =
      std::max(0.0, explicit_relocalization_fast_correction_yaw_rad_);
    explicit_relocalization_fast_max_duration_sec_ =
      std::max(0.1, explicit_relocalization_fast_max_duration_sec_);
    map_odom_large_correction_translation_m_ =
      std::max(0.0, map_odom_large_correction_translation_m_);
    map_odom_large_correction_yaw_rad_ =
      std::max(0.0, map_odom_large_correction_yaw_rad_);
    map_odom_online_hard_reject_translation_m_ =
      std::max(map_odom_large_correction_translation_m_, map_odom_online_hard_reject_translation_m_);
    map_odom_online_hard_reject_yaw_rad_ =
      std::max(map_odom_large_correction_yaw_rad_, map_odom_online_hard_reject_yaw_rad_);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationBridgeNode::on_pose, this, std::placeholders::_1));
    if (amcl_input_enabled_) {
      amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        amcl_pose_topic_,
        rclcpp::QoS(20),
        std::bind(&LocalizationBridgeNode::on_amcl_pose, this, std::placeholders::_1));
      if (amcl_scan_admission_enabled_) {
        amcl_scan_admission_status_sub_ = create_subscription<std_msgs::msg::String>(
          amcl_scan_admission_status_topic_,
          rclcpp::QoS(10),
          std::bind(
            &LocalizationBridgeNode::on_amcl_scan_admission_status,
            this,
            std::placeholders::_1));
      }
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationBridgeNode::on_odom, this, std::placeholders::_1));
    health_pub_ = create_publisher<std_msgs::msg::Bool>(health_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    amcl_initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_initial_pose_topic_,
      rclcpp::QoS(10));
    force_accept_srv_ = create_service<std_srvs::srv::Trigger>(
      force_accept_service_,
      std::bind(
        &LocalizationBridgeNode::on_force_accept_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    correction_pause_srv_ = create_service<std_srvs::srv::SetBool>(
      correction_pause_service_,
      std::bind(
        &LocalizationBridgeNode::on_correction_pause_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    amcl_seed_srv_ = create_service<std_srvs::srv::Trigger>(
      amcl_seed_service_,
      std::bind(
        &LocalizationBridgeNode::on_amcl_seed_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    map_odom_publisher_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&LocalizationBridgeNode::on_timer, this),
      map_odom_publisher_callback_group_);
    const auto status_period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(status_publish_period_sec_ * 1000.0)));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(status_period_ms),
      std::bind(&LocalizationBridgeNode::on_status_timer, this));
    RCLCPP_INFO(
      get_logger(),
      "map->odom publisher decoupled from correction callbacks: rate=%.1fHz warn_gap=%.1fms fail_gap=%.1fms",
      publish_rate_hz_,
      map_odom_publish_gap_warn_ms_,
      map_odom_publish_gap_fail_ms_);
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const double received_sec = now().seconds();
    std::string ignored_reason;
    if (should_ignore_force_accept_pretrigger_result(*msg, received_sec, ignored_reason)) {
      last_result_header_stamp_sec_ = stamp_to_sec(msg->header.stamp);
      last_result_receive_time_sec_ = received_sec;
      last_result_age_ms_ = (received_sec - last_result_header_stamp_sec_) * 1000.0;
      last_force_accept_ignored_reason_ = ignored_reason;
      ++force_accept_ignored_pretrigger_result_count_;
      RCLCPP_WARN(
        get_logger(),
        "ignoring stale localization_result before force-accept arm: %s",
        ignored_reason.c_str());
      return;
    }
    latest_pose_ = *msg;
    has_pose_ = true;
    latest_pose_received_sec_ = received_sec;
    last_result_header_stamp_sec_ = stamp_to_sec(msg->header.stamp);
    last_result_receive_time_sec_ = latest_pose_received_sec_;
    ++localization_result_count_;
    refresh_state("pose");
  }

  bool should_ignore_force_accept_pretrigger_result(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const double received_sec,
    std::string & reason) const
  {
    if (!force_accept_next_pose_ || !force_accept_next_pose_explicit_trigger_) {
      return false;
    }
    if (force_accept_armed_sec_ <= 0.0) {
      return false;
    }
    const double pose_stamp_sec = stamp_to_sec(pose.header.stamp);
    const double min_pose_stamp_sec =
      force_accept_armed_sec_ - force_accept_min_pose_stamp_slack_sec_;
    if (pose_stamp_sec >= min_pose_stamp_sec) {
      return false;
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "pose_stamp=" << pose_stamp_sec
        << " force_accept_armed=" << force_accept_armed_sec_
        << " min_pose_stamp=" << min_pose_stamp_sec
        << " slack_sec=" << force_accept_min_pose_stamp_slack_sec_
        << " received_age_ms=" << ((received_sec - pose_stamp_sec) * 1000.0);
    reason = out.str();
    return true;
  }

  void on_amcl_scan_admission_status(const std_msgs::msg::String::SharedPtr msg)
  {
    last_amcl_scan_admission_status_received_sec_ = now().seconds();
    last_amcl_scan_admission_status_ = msg->data;
    amcl_scan_admission_hz_ = json_double_field(msg->data, "hz", amcl_scan_admission_hz_);
    amcl_scan_admission_dropped_age_count_ =
      json_uint64_field(msg->data, "dropped_age_count", amcl_scan_admission_dropped_age_count_);
    amcl_scan_admission_dropped_tf_count_ =
      json_uint64_field(msg->data, "dropped_tf_count", amcl_scan_admission_dropped_tf_count_);
    amcl_scan_frame_id_ = json_field_value(msg->data, "frame_id");
    amcl_scan_last_age_ms_ = json_double_field(msg->data, "last_age_ms", amcl_scan_last_age_ms_);
    amcl_message_filter_drop_detected_ =
      json_bool_field(msg->data, "message_filter_drop_detected", amcl_message_filter_drop_detected_);
    amcl_scan_admission_last_error_ = json_field_value(msg->data, "last_error");
  }

  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    ++amcl_pose_count_;
    last_amcl_pose_received_sec_ = now().seconds();
    last_amcl_state_ = "received";
    if (!amcl_input_enabled_) {
      last_amcl_state_ = "disabled";
      return;
    }
    if (!has_odom_) {
      last_amcl_state_ = "waiting_for_odom";
      return;
    }
    if (!has_map_to_odom_) {
      last_amcl_state_ = "waiting_for_initial_map_to_odom";
      return;
    }
    const double now_sec = now().seconds();
    if (!amcl_seed_succeeded_) {
      last_amcl_state_ = "not_seeded";
      reject_candidate("AMCL_NOT_SEEDED", "amcl_pose", false);
      return;
    }
    const double amcl_pose_age_ms = (now_sec - stamp_to_sec(msg->header.stamp)) * 1000.0;
    if (amcl_pose_age_ms > amcl_pose_max_age_ms_) {
      last_amcl_state_ = "pose_stale";
      reject_candidate("AMCL_POSE_STALE", "amcl_pose", false);
      return;
    }
    if (!amcl_scan_admission_ready(now_sec)) {
      last_amcl_state_ = "scan_admission_not_ready";
      reject_candidate(amcl_scan_admission_reject_reason(), "amcl_pose", false);
      return;
    }
    auto candidate = build_candidate(
      *msg,
      amcl_source_name(),
      amcl_source_name(),
      amcl_max_result_age_ms_,
      amcl_covariance_gate_enabled_,
      amcl_max_xy_covariance_,
      amcl_max_yaw_covariance_);
    if (amcl_post_isaac_refine_eligible(now_sec)) {
      (void)accept_post_isaac_refine_candidate(candidate, "amcl_pose");
      return;
    }
    if (amcl_recently_seeded_by_isaac(now_sec)) {
      ++amcl_suppressed_after_isaac_count_;
      last_amcl_state_ = "suppressed_after_isaac_triggered";
      last_reject_reason_ = "amcl_suppressed_after_isaac_triggered";
      last_rejected_source_ = amcl_source_name();
      return;
    }
    (void)accept_amcl_candidate(candidate, "amcl_pose");
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
    if (!has_map_to_odom_ && has_pose_) {
      refresh_state("odom");
    }
  }

  void on_timer()
  {
    publish_map_to_odom_from_state();
  }

  void on_status_timer()
  {
    refresh_state("status_timer");
    publish_status_if_due();
  }

  void publish_health(const bool ok, const std::string & reason)
  {
    std_msgs::msg::Bool msg;
    msg.data = ok;
    health_pub_->publish(msg);
    if (has_last_health_ && last_health_state_ == ok && last_health_reason_ == reason) {
      return;
    }
    has_last_health_ = true;
    last_health_state_ = ok;
    last_health_reason_ = reason;
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s", reason.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    }
  }

  void on_force_accept_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    force_accept_armed_sec_ = now().seconds();
    force_accept_next_pose_ = true;
    force_accept_next_pose_explicit_trigger_ = true;
    response->success = true;
    response->message =
      "next localization_result may update map->odom across normal jump threshold; "
      "pre-arm result stamps are ignored";
    RCLCPP_WARN(
      get_logger(),
      "force accepting next localization_result up to %.3f m map->odom jump; "
      "armed_sec=%.3f min_pose_stamp_sec=%.3f",
      forced_jump_threshold_m_,
      force_accept_armed_sec_,
      force_accept_armed_sec_ - force_accept_min_pose_stamp_slack_sec_);
  }

  void on_correction_pause_request(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    correction_paused_ = request->data;
    correction_pause_reason_ = correction_paused_ ? "docking_fine" : "none";
    update_map_odom_pause_state();
    response->success = true;
    response->message = correction_paused_ ?
      "global localization corrections are paused" :
      "global localization corrections are enabled";
    RCLCPP_WARN(
      get_logger(),
      "global localization correction pause=%s reason=%s",
      correction_paused_ ? "true" : "false",
      correction_pause_reason_.c_str());
  }

  void on_amcl_seed_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped seed_pose;
    if (!current_map_base_pose(seed_pose)) {
      response->success = false;
      response->message = "cannot seed AMCL: map->odom or local odom is not available";
      amcl_seed_requested_ = true;
      amcl_seed_succeeded_ = false;
      ++amcl_seed_attempt_count_;
      amcl_seed_source_ = "current_map_base";
      amcl_seed_last_error_ = response->message;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }
    const bool seeded = publish_amcl_initial_pose(seed_pose, "current_map_base");
    response->success = seeded;
    response->message = seeded ?
      "published AMCL /initialpose from current map->base_link" :
      "AMCL /initialpose has no visible subscribers or seed is disabled";
  }

  bool is_new_pose_stamp() const
  {
    if (!has_last_pose_stamp_used_) {
      return true;
    }
    return latest_pose_.header.stamp.sec != last_pose_stamp_used_.sec ||
           latest_pose_.header.stamp.nanosec != last_pose_stamp_used_.nanosec;
  }

  void mark_latest_pose_stamp_used()
  {
    last_pose_stamp_used_ = latest_pose_.header.stamp;
    has_last_pose_stamp_used_ = true;
  }

  bool candidate_should_retry_later(const CandidateCorrection & candidate) const
  {
    return !candidate.valid && candidate.reject_reason.rfind("tf_history_missing", 0) == 0;
  }

  bool pose_frame_matches(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) const
  {
    return !require_result_frame_match_ ||
           pose.header.frame_id.empty() ||
           pose.header.frame_id == map_frame_;
  }

  bool lookup_odom_base(
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform,
    std::string & reason)
  {
    last_tf_lookup_stamp_sec_ = stamp_to_sec(stamp);
    last_odom_tf_history_lookup_ok_ = false;
    try {
      const auto timeout = rclcpp::Duration::from_seconds(odom_tf_lookup_timeout_ms_ * 1.0e-3);
      transform = tf_buffer_.lookupTransform(odom_frame_, base_frame_, stamp, timeout);
      last_odom_tf_history_lookup_ok_ = true;
      const rclcpp::Time latest_tf_time(0, 0, get_clock()->get_clock_type());
      const auto latest_transform =
        tf_buffer_.lookupTransform(odom_frame_, base_frame_, latest_tf_time, timeout);
      const double latest_tf_age_ms =
        (now().seconds() - stamp_to_sec(latest_transform.header.stamp)) * 1000.0;
      latest_odom_tf_age_ms_ = latest_tf_age_ms;
      latest_odom_tf_fresh_ = latest_tf_age_ms <= max_odom_tf_age_ms_;
      if (latest_tf_age_ms > max_odom_tf_age_ms_) {
        reason = "odom_base_latest_tf_stale_ms=" + std::to_string(latest_tf_age_ms);
        return false;
      }
    } catch (const std::exception & exc) {
      latest_odom_tf_fresh_ = false;
      reason = std::string("tf_history_missing: odom_base_tf_unavailable: ") + exc.what();
      return false;
    }
    return true;
  }

  std::string active_gate_mode() const
  {
    return "triggered";
  }

  std::string localization_source_for_gate(const std::string & gate_mode) const
  {
    (void)gate_mode;
    return "isaac_triggered";
  }

  std::string amcl_source_name() const
  {
    return amcl_gate_mode_ == "gated" ? "amcl_gated" : "amcl_shadow";
  }

  double result_age_limit_for_gate(const std::string & gate_mode) const
  {
    (void)gate_mode;
    return triggered_max_result_age_ms_;
  }

  bool amcl_scan_admission_ready(const double now_sec) const
  {
    if (!amcl_scan_admission_enabled_) {
      return true;
    }
    if (last_amcl_scan_admission_status_received_sec_ <= 0.0) {
      return false;
    }
    if (now_sec - last_amcl_scan_admission_status_received_sec_ > 2.5) {
      return false;
    }
    if (amcl_scan_admission_hz_ <= 0.0) {
      return false;
    }
    return amcl_scan_admission_last_error_.empty() ||
           amcl_scan_admission_last_error_ == "none";
  }

  std::string amcl_scan_admission_reject_reason() const
  {
    if (amcl_scan_admission_last_error_.find("AMCL_SCAN_TF_UNAVAILABLE") != std::string::npos) {
      return "AMCL_SCAN_TF_UNAVAILABLE";
    }
    if (last_amcl_scan_admission_status_received_sec_ <= 0.0) {
      return "AMCL_SCAN_ADMISSION_STATUS_MISSING";
    }
    return "AMCL_SCAN_TF_UNAVAILABLE";
  }

  std::string normalize_amcl_reject_reason(const std::string & reason) const
  {
    if (reason.find("pose_stale") != std::string::npos) {
      return "AMCL_POSE_STALE";
    }
    if (reason.find("tf_history_missing") != std::string::npos) {
      return "AMCL_TF_LOOKUP_FAILED";
    }
    if (reason.find("odom_base_latest_tf_stale") != std::string::npos) {
      return "AMCL_TF_LOOKUP_FAILED";
    }
    if (reason.find("covariance") != std::string::npos) {
      return "AMCL_COVARIANCE_TOO_LARGE";
    }
    if (reason.find("frame_mismatch") != std::string::npos) {
      return "AMCL_TF_LOOKUP_FAILED";
    }
    return reason.empty() ? "AMCL_TF_LOOKUP_FAILED" : reason;
  }

  MapToOdom current_map_to_odom_snapshot() const
  {
    std::lock_guard<std::mutex> lock(map_odom_state_mutex_);
    if (map_odom_state_.valid) {
      return map_odom_state_.transform;
    }
    return map_to_odom_;
  }

  CandidateCorrection build_candidate(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::string & gate_mode,
    const std::string & source_label,
    const double result_age_limit_ms,
    const bool covariance_gate_enabled,
    const double max_xy_covariance,
    const double max_yaw_covariance)
  {
    CandidateCorrection candidate;
    candidate.gate_mode = gate_mode;
    candidate.source = source_label;
    candidate.explicit_trigger = force_accept_next_pose_;
    candidate.gate_result_age_limit_ms = result_age_limit_ms;
    last_gate_mode_ = candidate.gate_mode;
    last_gate_result_age_limit_ms_ = candidate.gate_result_age_limit_ms;
    const double now_sec = now().seconds();
    const double pose_stamp_sec = stamp_to_sec(pose.header.stamp);
    const double result_age_ms = (now_sec - pose_stamp_sec) * 1000.0;
    candidate.result_age_ms = result_age_ms;
    last_result_age_ms_ = result_age_ms;
    last_result_header_stamp_sec_ = pose_stamp_sec;
    last_result_receive_time_sec_ =
      source_label.rfind("amcl_", 0) == 0 ? last_amcl_pose_received_sec_ : latest_pose_received_sec_;
    candidate.xy_covariance = std::max(pose.pose.covariance[0], pose.pose.covariance[7]);
    candidate.yaw_covariance = pose.pose.covariance[35];
    last_candidate_source_ = source_label;
    if (source_label.rfind("amcl_", 0) == 0) {
      last_amcl_xy_covariance_ = candidate.xy_covariance;
      last_amcl_yaw_covariance_ = candidate.yaw_covariance;
    }
    if (result_age_ms < -50.0) {
      candidate.reject_reason = source_label + "_pose_from_future_ms=" + std::to_string(-result_age_ms);
      return candidate;
    }
    if (result_age_ms > candidate.gate_result_age_limit_ms) {
      candidate.reject_reason =
        source_label + "_pose_stale_ms=" + std::to_string(result_age_ms) +
        " gate_mode=" + candidate.gate_mode +
        " gate_limit_ms=" + std::to_string(candidate.gate_result_age_limit_ms);
      return candidate;
    }
    if (!pose_frame_matches(pose)) {
      candidate.reject_reason =
        source_label + "_frame_mismatch:" + pose.header.frame_id + "!=" + map_frame_;
      return candidate;
    }
    if (
      covariance_gate_enabled &&
      (candidate.xy_covariance > max_xy_covariance || candidate.yaw_covariance > max_yaw_covariance))
    {
      candidate.reject_reason =
        source_label + "_covariance_rejected xy=" + std::to_string(candidate.xy_covariance) +
        " yaw=" + std::to_string(candidate.yaw_covariance);
      return candidate;
    }

    geometry_msgs::msg::TransformStamped odom_base_tf;
    std::string tf_reason;
    if (!lookup_odom_base(pose.header.stamp, odom_base_tf, tf_reason)) {
      candidate.reject_reason = tf_reason;
      return candidate;
    }

    const double map_x = pose.pose.pose.position.x;
    const double map_y = pose.pose.pose.position.y;
    const double map_yaw = yaw_from_quaternion(pose.pose.pose.orientation);
    const double odom_x = odom_base_tf.transform.translation.x;
    const double odom_y = odom_base_tf.transform.translation.y;
    const double odom_yaw = yaw_from_quaternion(odom_base_tf.transform.rotation);

    const double map_to_odom_yaw = normalize_yaw(map_yaw - odom_yaw);
    const double cos_delta = std::cos(map_to_odom_yaw);
    const double sin_delta = std::sin(map_to_odom_yaw);
    const double map_to_odom_x = map_x - (cos_delta * odom_x - sin_delta * odom_y);
    const double map_to_odom_y = map_y - (sin_delta * odom_x + cos_delta * odom_y);

    candidate.transform.x = map_to_odom_x;
    candidate.transform.y = map_to_odom_y;
    candidate.transform.yaw = map_to_odom_yaw;
    candidate.map_base_pose = pose;
    if (has_map_to_odom_) {
      const auto current_map_to_odom = current_map_to_odom_snapshot();
      candidate.correction_translation_m = std::hypot(
        map_to_odom_x - current_map_to_odom.x,
        map_to_odom_y - current_map_to_odom.y);
      candidate.correction_yaw_rad =
        std::abs(normalize_yaw(map_to_odom_yaw - current_map_to_odom.yaw));
    }
    candidate.valid = true;
    return candidate;
  }

  bool candidate_is_small(
    const CandidateCorrection & candidate,
    const double translation_limit_m,
    const double yaw_limit_rad) const
  {
    return candidate.correction_translation_m <= translation_limit_m &&
           candidate.correction_yaw_rad <= yaw_limit_rad;
  }

  bool candidate_is_hard_reject(
    const CandidateCorrection & candidate,
    const double translation_limit_m,
    const double yaw_limit_rad) const
  {
    return candidate.correction_translation_m > translation_limit_m ||
           candidate.correction_yaw_rad > yaw_limit_rad;
  }

  bool amcl_candidate_agrees_with_previous_medium(const CandidateCorrection & candidate)
  {
    if (!has_last_amcl_medium_candidate_) {
      last_amcl_medium_candidate_ = candidate.transform;
      has_last_amcl_medium_candidate_ = true;
      amcl_medium_candidate_agreement_count_ = 1;
      return false;
    }
    const double dx = candidate.transform.x - last_amcl_medium_candidate_.x;
    const double dy = candidate.transform.y - last_amcl_medium_candidate_.y;
    const double dyaw =
      std::abs(normalize_yaw(candidate.transform.yaw - last_amcl_medium_candidate_.yaw));
    if (
      std::hypot(dx, dy) <= amcl_small_correction_translation_m_ &&
      dyaw <= amcl_small_correction_yaw_rad_)
    {
      ++amcl_medium_candidate_agreement_count_;
    } else {
      last_amcl_medium_candidate_ = candidate.transform;
      amcl_medium_candidate_agreement_count_ = 1;
    }
    return amcl_medium_candidate_agreement_count_ >= amcl_medium_correction_consistency_count_;
  }

  void reset_amcl_medium_consistency()
  {
    has_last_amcl_medium_candidate_ = false;
    amcl_medium_candidate_agreement_count_ = 0;
  }

  bool amcl_recently_seeded_by_isaac(const double now_sec) const
  {
    return last_isaac_triggered_accept_sec_ > 0.0 &&
           now_sec - last_isaac_triggered_accept_sec_ < amcl_accept_after_isaac_delay_sec_;
  }

  bool amcl_post_isaac_refine_eligible(const double now_sec) const
  {
    if (!amcl_post_isaac_refine_enabled_ || amcl_gate_mode_ != "gated") {
      return false;
    }
    if (last_isaac_triggered_accept_sec_ <= 0.0) {
      return false;
    }
    if (
      last_explicit_relocalization_sequence_ > 0U &&
      amcl_post_isaac_refined_sequence_ == last_explicit_relocalization_sequence_)
    {
      return false;
    }
    const double refine_reference_sec = amcl_post_isaac_refine_reference_sec();
    if (refine_reference_sec <= 0.0) {
      return false;
    }
    if (now_sec - refine_reference_sec > amcl_post_isaac_refine_window_sec_) {
      return false;
    }
    return !amcl_post_isaac_refine_require_stationary_ || !amcl_robot_moving_now();
  }

  double amcl_post_isaac_refine_reference_sec() const
  {
    if (
      last_amcl_initial_pose_seed_sec_ > 0.0 &&
      last_amcl_initial_pose_seed_sec_ >= last_isaac_triggered_accept_sec_)
    {
      return last_amcl_initial_pose_seed_sec_;
    }
    return last_isaac_triggered_accept_sec_;
  }

  bool post_isaac_refine_candidate_agrees(const CandidateCorrection & candidate)
  {
    if (!has_last_post_isaac_refine_candidate_) {
      last_post_isaac_refine_candidate_ = candidate.transform;
      has_last_post_isaac_refine_candidate_ = true;
      amcl_post_isaac_refine_agreement_count_ = 1;
      return amcl_post_isaac_refine_consistency_count_ <= 1;
    }
    const double dx = candidate.transform.x - last_post_isaac_refine_candidate_.x;
    const double dy = candidate.transform.y - last_post_isaac_refine_candidate_.y;
    const double dyaw =
      std::abs(normalize_yaw(candidate.transform.yaw - last_post_isaac_refine_candidate_.yaw));
    if (
      std::hypot(dx, dy) <= amcl_post_isaac_refine_agreement_translation_m_ &&
      dyaw <= amcl_post_isaac_refine_agreement_yaw_rad_)
    {
      ++amcl_post_isaac_refine_agreement_count_;
    } else {
      last_post_isaac_refine_candidate_ = candidate.transform;
      amcl_post_isaac_refine_agreement_count_ = 1;
    }
    return amcl_post_isaac_refine_agreement_count_ >=
           amcl_post_isaac_refine_consistency_count_;
  }

  void reset_post_isaac_refine_consistency()
  {
    has_last_post_isaac_refine_candidate_ = false;
    amcl_post_isaac_refine_agreement_count_ = 0;
  }

  void record_gate_result_count(const CandidateCorrection & candidate)
  {
    if (candidate.source == "amcl_shadow" || candidate.source == "amcl_gated") {
      ++amcl_candidate_count_;
    } else {
      ++triggered_result_count_;
    }
  }

  bool accept_candidate(CandidateCorrection & candidate, const char * source)
  {
    record_gate_result_count(candidate);
    last_candidate_correction_translation_m_ = candidate.correction_translation_m;
    last_candidate_correction_yaw_rad_ = candidate.correction_yaw_rad;
    last_gate_mode_ = candidate.gate_mode;
    last_gate_result_age_limit_ms_ = candidate.gate_result_age_limit_ms;
    last_result_age_ms_ = candidate.result_age_ms;
    last_candidate_sec_ = now().seconds();

    if (!candidate.valid) {
      if (!candidate_should_retry_later(candidate)) {
        mark_latest_pose_stamp_used();
      }
      reject_candidate(candidate.reject_reason, source);
      return false;
    }
    mark_latest_pose_stamp_used();

    if (correction_paused_) {
      reject_candidate("GLOBAL_CORRECTION_PAUSED:" + correction_pause_reason_, source, false);
      return false;
    }

    if (!has_map_to_odom_) {
      if (force_accept_next_pose_) {
        force_accept_next_pose_ = false;
        force_accept_next_pose_explicit_trigger_ = false;
        apply_candidate(candidate, source, "EXPLICIT_TRIGGERED_RELOCALIZATION");
      } else {
        apply_candidate(candidate, source, "initial_lock");
      }
      return true;
    }

    if (force_accept_next_pose_) {
      const double forced_limit = std::min(
        forced_jump_threshold_m_,
        triggered_hard_reject_translation_m_);
      if (candidate.correction_translation_m > forced_limit) {
        ++large_correction_rejected_count_;
        reject_candidate("bridge forced map->odom jump rejected", source);
        force_accept_next_pose_ = false;
        force_accept_next_pose_explicit_trigger_ = false;
        return false;
      }
      if (!triggered_allow_large_correction_ && candidate.correction_translation_m > jump_threshold_m_) {
        reject_candidate("triggered_large_correction_disabled", source);
        force_accept_next_pose_ = false;
        force_accept_next_pose_explicit_trigger_ = false;
        return false;
      }
      force_accept_next_pose_ = false;
      force_accept_next_pose_explicit_trigger_ = false;
      apply_candidate(candidate, source, "EXPLICIT_TRIGGERED_RELOCALIZATION");
      return true;
    }

    if (
      candidate.correction_translation_m > map_odom_online_hard_reject_translation_m_ ||
      candidate.correction_yaw_rad > map_odom_online_hard_reject_yaw_rad_)
    {
      ++large_correction_rejected_count_;
      reject_candidate("online_correction_requires_recovery", source);
      return false;
    }

    if (candidate.correction_translation_m > jump_threshold_m_) {
      ++large_correction_rejected_count_;
      reject_candidate("triggered_jump_over_threshold", source);
      return false;
    }
    apply_candidate(candidate, source, "triggered_correction");
    return true;
  }

  bool accept_post_isaac_refine_candidate(CandidateCorrection & candidate, const char * source)
  {
    record_gate_result_count(candidate);
    ++amcl_post_isaac_refine_candidate_count_;
    last_candidate_correction_translation_m_ = candidate.correction_translation_m;
    last_candidate_correction_yaw_rad_ = candidate.correction_yaw_rad;
    last_gate_mode_ = candidate.gate_mode;
    last_gate_result_age_limit_ms_ = candidate.gate_result_age_limit_ms;
    last_result_age_ms_ = candidate.result_age_ms;
    last_candidate_sec_ = now().seconds();
    last_candidate_source_ = candidate.source;

    if (!candidate.valid) {
      reset_post_isaac_refine_consistency();
      last_amcl_state_ = "post_isaac_refine_rejected";
      ++amcl_post_isaac_refine_rejected_count_;
      reject_candidate(normalize_amcl_reject_reason(candidate.reject_reason), source, false);
      return false;
    }

    if (correction_paused_) {
      reset_post_isaac_refine_consistency();
      last_amcl_state_ = "post_isaac_refine_paused";
      ++amcl_post_isaac_refine_rejected_count_;
      reject_candidate("GLOBAL_CORRECTION_PAUSED:" + correction_pause_reason_, source, false);
      return false;
    }

    if (candidate_is_hard_reject(
        candidate,
        amcl_post_isaac_refine_max_translation_m_,
        amcl_post_isaac_refine_max_yaw_rad_))
    {
      reset_post_isaac_refine_consistency();
      last_amcl_state_ = "post_isaac_refine_too_large";
      ++amcl_post_isaac_refine_rejected_count_;
      reject_candidate("AMCL_POST_ISAAC_REFINE_TOO_LARGE", source, false);
      return false;
    }

    if (!post_isaac_refine_candidate_agrees(candidate)) {
      last_amcl_state_ = "post_isaac_refine_waiting_for_consistency";
      ++amcl_post_isaac_refine_waiting_count_;
      reject_candidate("AMCL_POST_ISAAC_REFINE_WAITING_FOR_CONSISTENCY", source, false);
      return false;
    }

    reset_post_isaac_refine_consistency();
    last_amcl_state_ = "accepted_post_isaac_refine_correction";
    ++amcl_post_isaac_refine_accepted_count_;
    apply_candidate(candidate, source, "AMCL_POST_ISAAC_REFINE_CORRECTION");
    amcl_post_isaac_refined_sequence_ = last_explicit_relocalization_sequence_;
    return true;
  }

  bool accept_amcl_candidate(CandidateCorrection & candidate, const char * source)
  {
    record_gate_result_count(candidate);
    last_candidate_correction_translation_m_ = candidate.correction_translation_m;
    last_candidate_correction_yaw_rad_ = candidate.correction_yaw_rad;
    last_gate_mode_ = candidate.gate_mode;
    last_gate_result_age_limit_ms_ = candidate.gate_result_age_limit_ms;
    last_result_age_ms_ = candidate.result_age_ms;
    last_candidate_sec_ = now().seconds();
    last_candidate_source_ = candidate.source;

    if (!candidate.valid) {
      last_amcl_state_ = "rejected";
      reject_candidate(normalize_amcl_reject_reason(candidate.reject_reason), source, false);
      return false;
    }

    if (correction_paused_) {
      last_amcl_state_ = "paused";
      reject_candidate("GLOBAL_CORRECTION_PAUSED:" + correction_pause_reason_, source, false);
      return false;
    }

    if (amcl_gate_mode_ == "shadow") {
      ++amcl_shadow_candidate_count_;
      ++shadow_candidate_count_;
      last_amcl_state_ = "shadow_candidate";
      last_reject_reason_ = "AMCL_SHADOW_ONLY";
      last_rejected_source_ = candidate.source;
      return false;
    }

    if (!amcl_accept_corrections_while_moving_ && amcl_robot_moving_now()) {
      reset_amcl_medium_consistency();
      last_amcl_state_ = "moving_observe_only";
      reject_candidate("AMCL_ROBOT_MOVING_OBSERVE_ONLY", source, false);
      return false;
    }

    if (candidate_is_small(
        candidate,
        amcl_small_correction_translation_m_,
        amcl_small_correction_yaw_rad_))
    {
      reset_amcl_medium_consistency();
      last_amcl_state_ = "accepted_small_correction";
      apply_candidate(candidate, source, "AMCL_SMALL_CORRECTION");
      return true;
    }

    if (candidate_is_hard_reject(
        candidate,
        amcl_hard_reject_translation_m_,
        amcl_hard_reject_yaw_rad_))
    {
      reset_amcl_medium_consistency();
      last_amcl_state_ = "hard_reject";
      ++large_correction_rejected_count_;
      reject_candidate("AMCL_CORRECTION_TOO_LARGE", source, false);
      return false;
    }

    if (candidate_is_small(
        candidate,
        amcl_medium_correction_translation_m_,
        amcl_medium_correction_yaw_rad_))
    {
      if (amcl_candidate_agrees_with_previous_medium(candidate)) {
        reset_amcl_medium_consistency();
        last_amcl_state_ = "accepted_medium_consistent_correction";
        apply_candidate(candidate, source, "AMCL_MEDIUM_CONSISTENT_CORRECTION");
        return true;
      }
      last_amcl_state_ = "medium_correction_waiting_for_consistency";
      reject_candidate("AMCL_MEDIUM_CORRECTION_WAITING_FOR_CONSISTENCY", source, false);
      return false;
    }

    reset_amcl_medium_consistency();
    last_amcl_state_ = "large_consistent_requires_isaac_recovery";
    reject_candidate("AMCL_CORRECTION_TOO_LARGE", source, false);
    return false;
  }

  bool amcl_robot_moving_now() const
  {
    if (!has_odom_) {
      return false;
    }
    const double linear_speed_mps = std::hypot(
      latest_odom_.twist.twist.linear.x,
      latest_odom_.twist.twist.linear.y);
    const double angular_speed_radps = std::abs(latest_odom_.twist.twist.angular.z);
    return linear_speed_mps > amcl_moving_linear_speed_mps_ ||
      angular_speed_radps > amcl_moving_angular_speed_radps_;
  }

  void reject_candidate(
    const std::string & reason,
    const char * source,
    const bool affect_health = true)
  {
    ++rejected_result_count_;
    if (std::string(source).rfind("amcl", 0) == 0) {
      ++amcl_rejected_count_;
    }
    last_reject_reason_ = reason + " (" + source + ")";
    last_rejected_source_ = std::string(source).rfind("amcl", 0) == 0 ? amcl_source_name() : last_candidate_source_;
    if (affect_health) {
      publish_health(false, "bridge localization_result rejected: " + last_reject_reason_);
    }
  }

  double candidate_target_z(const CandidateCorrection & candidate) const
  {
    return (!two_d_mode_ && has_odom_) ?
      candidate.map_base_pose.pose.pose.position.z - latest_odom_.pose.pose.position.z : 0.0;
  }

  void refresh_map_odom_error_locked(MapOdomState & state)
  {
    const double dx = state.target_transform.x - state.current_transform.x;
    const double dy = state.target_transform.y - state.current_transform.y;
    const double dyaw =
      std::abs(normalize_yaw(state.target_transform.yaw - state.current_transform.yaw));
    state.remaining_translation_error_m = std::hypot(dx, dy);
    state.remaining_yaw_error_rad = dyaw;
    state.correction_active =
      state.remaining_translation_error_m > map_odom_smoothing_snap_translation_epsilon_m_ ||
      state.remaining_yaw_error_rad > map_odom_smoothing_snap_yaw_epsilon_rad_;
    state.safe_for_goal_start = !state.correction_active;
    if (!state.correction_active) {
      state.current_transform = state.target_transform;
      state.current_z = state.target_z;
      state.current_sequence = state.target_sequence;
      state.current_source = state.target_source;
      state.remaining_translation_error_m = 0.0;
      state.remaining_yaw_error_rad = 0.0;
    }
    state.transform = state.current_transform;
    state.z = state.current_z;
    state.sequence = state.current_sequence;
    state.source = state.target_source;
    map_to_odom_ = state.transform;
  }

  bool explicit_relocalization_uses_fast_smoothing(
    const CandidateCorrection & candidate,
    const bool initial_lock) const
  {
    if (
      initial_lock ||
      !explicit_relocalization_fast_smoothing_enabled_ ||
      !candidate.explicit_trigger ||
      candidate.source != "isaac_triggered")
    {
      return false;
    }
    return candidate.correction_translation_m >= explicit_relocalization_fast_correction_translation_m_ ||
           candidate.correction_yaw_rad >= explicit_relocalization_fast_correction_yaw_rad_;
  }

  void configure_correction_smoothing_locked(
    MapOdomState & state,
    const CandidateCorrection & candidate,
    const bool initial_lock) const
  {
    state.smoothing_policy = "default";
    state.smoothing_translation_rate_mps = map_odom_smoothing_translation_rate_mps_;
    state.smoothing_yaw_rate_radps = map_odom_smoothing_yaw_rate_radps_;

    if (!explicit_relocalization_uses_fast_smoothing(candidate, initial_lock)) {
      return;
    }

    state.smoothing_policy = "explicit_relocalization_fast";
    state.smoothing_translation_rate_mps = std::max(
      map_odom_smoothing_translation_rate_mps_,
      candidate.correction_translation_m / explicit_relocalization_fast_max_duration_sec_);
    state.smoothing_yaw_rate_radps = std::max(
      map_odom_smoothing_yaw_rate_radps_,
      candidate.correction_yaw_rad / explicit_relocalization_fast_max_duration_sec_);
  }

  void update_map_odom_state_from_candidate(
    const CandidateCorrection & candidate,
    const bool initial_lock)
  {
    std::lock_guard<std::mutex> lock(map_odom_state_mutex_);
    auto & state = map_odom_state_;
    const double now_sec = now().seconds();
    state.target_transform = candidate.transform;
    state.target_z = candidate_target_z(candidate);
    state.target_sequence = accepted_result_count_;
    state.last_accepted_sequence = accepted_result_count_;
    state.accepted_time_sec = last_accepted_sec_;
    state.last_correction_accept_time = last_accepted_sec_;
    state.target_source = candidate.source;
    state.last_correction_source = candidate.source;
    state.correction_translation_m = candidate.correction_translation_m;
    state.correction_yaw_rad = candidate.correction_yaw_rad;
    state.last_correction_delta_translation_m = candidate.correction_translation_m;
    state.last_correction_delta_yaw_rad = candidate.correction_yaw_rad;
    state.valid = true;
    state.correction_paused = correction_paused_;
    state.frozen_due_to_pause = correction_paused_;
    state.smoothing_enabled = map_odom_smoothing_enabled_;
    configure_correction_smoothing_locked(state, candidate, initial_lock);
    state.large_correction_requires_recovery = map_odom_large_correction_requires_recovery_;

    const bool snap_immediately = initial_lock || !map_odom_smoothing_enabled_;
    if (snap_immediately || !state.valid || state.current_sequence == 0U) {
      state.current_transform = state.target_transform;
      state.current_z = state.target_z;
      state.current_sequence = state.target_sequence;
      state.current_source = state.target_source;
      state.last_step_translation_m = candidate.correction_translation_m;
      state.last_step_yaw_rad = candidate.correction_yaw_rad;
      ++online_correction_snap_count_;
    } else {
      refresh_map_odom_error_locked(state);
      if (state.correction_active) {
        ++online_correction_smoothed_count_;
      } else {
        ++online_correction_snap_count_;
      }
    }
    state.last_correction_apply_time = now_sec;
    refresh_map_odom_error_locked(state);
  }

  void update_map_odom_pause_state()
  {
    std::lock_guard<std::mutex> lock(map_odom_state_mutex_);
    map_odom_state_.correction_paused = correction_paused_;
    map_odom_state_.frozen_due_to_pause = correction_paused_ && map_odom_state_.valid;
  }

  void advance_map_odom_state_locked(MapOdomState & state, const double now_sec)
  {
    if (!state.valid) {
      return;
    }
    state.smoothing_enabled = map_odom_smoothing_enabled_;
    if (state.smoothing_translation_rate_mps <= 0.0) {
      state.smoothing_translation_rate_mps = map_odom_smoothing_translation_rate_mps_;
    }
    if (state.smoothing_yaw_rate_radps <= 0.0) {
      state.smoothing_yaw_rate_radps = map_odom_smoothing_yaw_rate_radps_;
    }
    state.large_correction_requires_recovery = map_odom_large_correction_requires_recovery_;
    state.last_step_translation_m = 0.0;
    state.last_step_yaw_rad = 0.0;

    if (!map_odom_smoothing_enabled_ || state.frozen_due_to_pause) {
      refresh_map_odom_error_locked(state);
      return;
    }

    double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    if (state.last_correction_apply_time > 0.0) {
      dt = std::clamp(now_sec - state.last_correction_apply_time, 0.0, 0.25);
      if (dt <= 0.0) {
        dt = 1.0 / std::max(1.0, publish_rate_hz_);
      }
    }

    const double dx = state.target_transform.x - state.current_transform.x;
    const double dy = state.target_transform.y - state.current_transform.y;
    const double distance = std::hypot(dx, dy);
    const double max_translation_step = state.smoothing_translation_rate_mps * dt;
    if (distance <= map_odom_smoothing_snap_translation_epsilon_m_ || distance <= max_translation_step) {
      state.last_step_translation_m = distance;
      state.current_transform.x = state.target_transform.x;
      state.current_transform.y = state.target_transform.y;
    } else if (distance > 0.0) {
      const double ratio = max_translation_step / distance;
      state.current_transform.x += dx * ratio;
      state.current_transform.y += dy * ratio;
      state.last_step_translation_m = max_translation_step;
    }

    const double yaw_error = normalize_yaw(state.target_transform.yaw - state.current_transform.yaw);
    const double abs_yaw_error = std::abs(yaw_error);
    const double max_yaw_step = state.smoothing_yaw_rate_radps * dt;
    if (abs_yaw_error <= map_odom_smoothing_snap_yaw_epsilon_rad_ || abs_yaw_error <= max_yaw_step) {
      state.current_transform.yaw = state.target_transform.yaw;
      state.last_step_yaw_rad = abs_yaw_error;
    } else {
      const double yaw_step = std::copysign(max_yaw_step, yaw_error);
      state.current_transform.yaw = normalize_yaw(state.current_transform.yaw + yaw_step);
      state.last_step_yaw_rad = std::abs(yaw_step);
    }

    state.current_z = state.target_z;
    state.last_correction_apply_time = now_sec;
    refresh_map_odom_error_locked(state);
  }

  void apply_candidate(
    const CandidateCorrection & candidate,
    const char * source,
    const std::string & accept_reason)
  {
    const bool initial_lock = !has_map_to_odom_;
    mark_latest_pose_stamp_used();
    last_accepted_correction_translation_m_ = candidate.correction_translation_m;
    last_accepted_correction_yaw_rad_ = candidate.correction_yaw_rad;
    last_accepted_sec_ = now().seconds();
    last_accept_reason_ = accept_reason;
    last_reject_reason_.clear();
    last_accepted_source_ = candidate.source;
    active_correction_source_ = candidate.source;
    ++accepted_result_count_;
    if (candidate.source == "isaac_triggered" && candidate.explicit_trigger) {
      ++last_explicit_relocalization_sequence_;
      last_explicit_relocalization_accept_sec_ = last_accepted_sec_;
      last_explicit_relocalization_source_ = candidate.source;
    }
    if (candidate.source == "amcl_gated") {
      ++amcl_accepted_count_;
    }
    update_map_odom_state_from_candidate(candidate, initial_lock);
    has_map_to_odom_ = true;
    if (candidate.source == "isaac_triggered") {
      last_isaac_triggered_accept_sec_ = last_accepted_sec_;
      reset_post_isaac_refine_consistency();
      if (amcl_initial_pose_seed_enabled_) {
        (void)publish_amcl_initial_pose(candidate.map_base_pose, "isaac_triggered_accept");
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "accepted map->odom correction reason=%s source=%s translation=%.3f yaw=%.3f",
      accept_reason.c_str(),
      source,
      candidate.correction_translation_m,
      candidate.correction_yaw_rad);
  }

  void fill_amcl_initial_pose_covariance(
    geometry_msgs::msg::PoseWithCovarianceStamped & pose) const
  {
    for (auto & value : pose.pose.covariance) {
      value = 0.0;
    }
    pose.pose.covariance[0] = amcl_initial_pose_xy_covariance_;
    pose.pose.covariance[7] = amcl_initial_pose_xy_covariance_;
    pose.pose.covariance[35] = amcl_initial_pose_yaw_covariance_;
  }

  bool publish_amcl_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & seed_pose,
    const std::string & reason)
  {
    amcl_seed_requested_ = true;
    ++amcl_seed_attempt_count_;
    amcl_seed_source_ = reason;
    if (!amcl_initial_pose_seed_enabled_ || !amcl_initial_pose_pub_) {
      amcl_seed_succeeded_ = false;
      amcl_seed_last_error_ = "SEED_DISABLED";
      return false;
    }
    auto msg = seed_pose;
    msg.header.frame_id = map_frame_;
    fill_amcl_initial_pose_covariance(msg);
    last_amcl_initial_pose_subscribers_ = amcl_initial_pose_pub_->get_subscription_count();
    if (last_amcl_initial_pose_subscribers_ == 0U) {
      RCLCPP_WARN(
        get_logger(),
        "publishing AMCL initial pose reason=%s but no /initialpose subscribers are visible",
        reason.c_str());
      amcl_seed_succeeded_ = false;
      amcl_seed_last_error_ = "NO_INITIALPOSE_SUBSCRIBERS";
      return false;
    }
    for (int i = 0; i < amcl_initial_pose_publish_repetitions_; ++i) {
      amcl_initial_pose_pub_->publish(msg);
      ++amcl_initial_pose_published_count_;
      if (i + 1 < amcl_initial_pose_publish_repetitions_ && amcl_initial_pose_repeat_period_ms_ > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(amcl_initial_pose_repeat_period_ms_));
      }
    }
    last_amcl_initial_pose_seed_sec_ = now().seconds();
    last_amcl_initial_pose_reason_ = reason;
    ++amcl_initial_pose_seed_count_;
    amcl_seed_succeeded_ = true;
    amcl_seed_last_error_ = "none";
    return true;
  }

  bool current_map_base_pose(geometry_msgs::msg::PoseWithCovarianceStamped & pose) const
  {
    if (!has_map_to_odom_ || !has_odom_) {
      return false;
    }
    const auto current_map_to_odom = current_map_to_odom_snapshot();
    const double odom_x = latest_odom_.pose.pose.position.x;
    const double odom_y = latest_odom_.pose.pose.position.y;
    const double odom_yaw = yaw_from_quaternion(latest_odom_.pose.pose.orientation);
    const double cos_delta = std::cos(current_map_to_odom.yaw);
    const double sin_delta = std::sin(current_map_to_odom.yaw);
    const double map_x = current_map_to_odom.x + (cos_delta * odom_x - sin_delta * odom_y);
    const double map_y = current_map_to_odom.y + (sin_delta * odom_x + cos_delta * odom_y);
    const double map_yaw = normalize_yaw(current_map_to_odom.yaw + odom_yaw);

    pose.header = latest_odom_.header;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = map_x;
    pose.pose.pose.position.y = map_y;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = quaternion_from_yaw(map_yaw);
    fill_amcl_initial_pose_covariance(pose);
    return true;
  }

  void publish_map_to_odom_from_state()
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const double wall_sec = now().seconds();
    MapOdomState state;
    {
      std::lock_guard<std::mutex> lock(map_odom_state_mutex_);
      advance_map_odom_state_locked(map_odom_state_, wall_sec);
      state = map_odom_state_;
    }

    bool published = false;
    if (publish_tf_ && state.valid) {
      geometry_msgs::msg::TransformStamped tf;
      auto tf_stamp = now();
      if (tf_future_stamp_offset_sec_ > 0.0) {
        tf_stamp = tf_stamp + rclcpp::Duration::from_seconds(tf_future_stamp_offset_sec_);
      }
      tf.header.stamp = tf_stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = state.transform.x;
      tf.transform.translation.y = state.transform.y;
      tf.transform.translation.z = state.z;
      tf.transform.rotation = quaternion_from_yaw(state.transform.yaw);
      tf_broadcaster_->sendTransform(tf);
      published = true;
    }

    const auto callback_end = std::chrono::steady_clock::now();
    const double callback_duration_us =
      std::chrono::duration<double, std::micro>(callback_end - callback_start).count();
    std::lock_guard<std::mutex> lock(map_odom_publish_stats_mutex_);
    map_odom_publish_callback_duration_us_ = callback_duration_us;
    map_odom_state_valid_snapshot_ = state.valid;
    map_odom_latest_source_ = state.source;
    map_odom_latest_accepted_sequence_ = state.last_accepted_sequence;
    map_odom_correction_paused_snapshot_ = state.correction_paused;
    map_odom_frozen_due_to_pause_snapshot_ = state.frozen_due_to_pause;
    map_odom_smoothing_enabled_snapshot_ = state.smoothing_enabled;
    map_odom_correction_active_snapshot_ = state.correction_active;
    map_odom_safe_for_goal_start_snapshot_ = state.safe_for_goal_start;
    map_odom_current_sequence_snapshot_ = state.current_sequence;
    map_odom_target_sequence_snapshot_ = state.target_sequence;
    map_odom_last_accepted_sequence_snapshot_ = state.last_accepted_sequence;
    map_odom_current_source_snapshot_ = state.current_source;
    map_odom_target_source_snapshot_ = state.target_source;
    map_odom_remaining_translation_error_m_snapshot_ = state.remaining_translation_error_m;
    map_odom_remaining_yaw_error_rad_snapshot_ = state.remaining_yaw_error_rad;
    map_odom_last_step_translation_m_snapshot_ = state.last_step_translation_m;
    map_odom_last_step_yaw_rad_snapshot_ = state.last_step_yaw_rad;
    map_odom_active_smoothing_translation_rate_mps_snapshot_ = state.smoothing_translation_rate_mps;
    map_odom_active_smoothing_yaw_rate_radps_snapshot_ = state.smoothing_yaw_rate_radps;
    map_odom_smoothing_policy_snapshot_ = state.smoothing_policy;
    map_odom_last_correction_source_snapshot_ = state.last_correction_source;
    map_odom_last_correction_accept_time_snapshot_ = state.last_correction_accept_time;
    map_odom_last_correction_apply_time_snapshot_ = state.last_correction_apply_time;
    if (!published) {
      return;
    }
    if (map_odom_last_publish_wall_sec_ > 0.0) {
      map_odom_last_publish_gap_ms_ = std::max(
        0.0, (wall_sec - map_odom_last_publish_wall_sec_) * 1000.0);
      map_odom_publish_gap_max_ms_ =
        std::max(map_odom_publish_gap_max_ms_, map_odom_last_publish_gap_ms_);
      if (map_odom_last_publish_gap_ms_ > map_odom_publish_gap_warn_ms_) {
        ++map_odom_publish_missed_count_;
      }
      if (map_odom_last_publish_gap_ms_ > map_odom_publish_gap_fail_ms_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "map->odom publish gap %.1f ms exceeds fail threshold %.1f ms",
          map_odom_last_publish_gap_ms_,
          map_odom_publish_gap_fail_ms_);
      }
    }
    map_odom_last_publish_wall_sec_ = wall_sec;
    map_odom_last_published_sequence_ = state.current_sequence;
    map_odom_last_published_sequence_snapshot_ = state.current_sequence;
    ++map_odom_publish_count_;
  }

  void refresh_state(const char * source)
  {
    if (!has_odom_) {
      publish_health(false, std::string("bridge waiting for odom (") + source + ")");
      return;
    }

    const double now_sec = now().seconds();
    const double odom_sec = stamp_to_sec(latest_odom_.header.stamp);
    if (now_sec - odom_sec > timeout_sec_) {
      publish_health(false, std::string("bridge odom timeout (") + source + ")");
      return;
    }

    bool update_from_pose = false;
    if (has_pose_) {
      const double pose_received_sec = latest_pose_received_sec_;
      if (!has_map_to_odom_) {
        if (now_sec - pose_received_sec > timeout_sec_) {
          publish_health(false, std::string("bridge localization_result timeout before initial lock (") + source + ")");
          return;
        }
        update_from_pose = true;
      } else if (is_new_pose_stamp() && now_sec - pose_received_sec <= timeout_sec_) {
        update_from_pose = true;
      }
    } else if (!has_map_to_odom_) {
      publish_health(false, std::string("bridge waiting for localization_result (") + source + ")");
      return;
    }

    if (update_from_pose) {
      const auto gate_mode = active_gate_mode();
      auto candidate = build_candidate(
        latest_pose_,
        gate_mode,
        localization_source_for_gate(gate_mode),
        result_age_limit_for_gate(gate_mode),
        false,
        0.0,
        0.0);
      (void)accept_candidate(candidate, source);
    }

    if (!has_map_to_odom_) {
      publish_health(false, std::string("bridge has no map->odom solution (") + source + ")");
      return;
    }

    publish_health(true, std::string("bridge map->odom active (") + source + ")");
  }

  double rate_since_last_status(
    const std::uint64_t current,
    std::uint64_t & previous,
    const double elapsed_sec)
  {
    const auto delta = current - previous;
    previous = current;
    return elapsed_sec > 0.0 ? static_cast<double>(delta) / elapsed_sec : 0.0;
  }

  AmclRuntimeStatus read_amcl_runtime_status_file() const
  {
    AmclRuntimeStatus status;
    if (amcl_runtime_status_file_.empty()) {
      return status;
    }
    std::ifstream input(amcl_runtime_status_file_);
    if (!input.good()) {
      return status;
    }

    std::unordered_map<std::string, std::string> fields;
    std::string line;
    while (std::getline(input, line)) {
      line = trim_copy(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      const auto equal_pos = line.find('=');
      if (equal_pos == std::string::npos) {
        continue;
      }
      const auto key = trim_copy(line.substr(0U, equal_pos));
      const auto value = unquote_env_value(line.substr(equal_pos + 1U));
      if (!key.empty()) {
        fields[key] = value;
      }
    }

    status.available = true;
    status.mode = fields["AMCL_MODE"];
    status.state = fields["AMCL_STATE"];
    status.start_result = fields["AMCL_START_RESULT"];
    status.ready = env_bool_field(fields, "AMCL_READY", false);
    status.degraded = env_bool_field(fields, "AMCL_DEGRADED", false);
    status.degraded_reason = fields["AMCL_FAILURE_REASON"];
    status.process_alive = env_bool_field(fields, "AMCL_PID_ALIVE", false);
    status.node_exists = env_bool_field(fields, "AMCL_NODE_EXISTS", false);
    status.lifecycle_active = env_bool_field(fields, "AMCL_LIFECYCLE_ACTIVE", false);
    status.scan_admission_alive = env_bool_field(fields, "SCAN_ADMISSION_ALIVE", false);
    status.pose_publisher_count = env_int_field(fields, "AMCL_POSE_PUBLISHER_COUNT", 0);
    status.scan_admission_status_publisher_count =
      env_int_field(fields, "SCAN_ADMISSION_STATUS_PUBLISHER_COUNT", 0);
    status.last_pose_age_ms = env_double_field(
      fields, "AMCL_POSE_LAST_RECEIVE_AGE_MS",
      env_double_field(fields, "AMCL_LAST_POSE_AGE_MS", -1.0));
    status.seed_succeeded = env_bool_field(fields, "AMCL_SEED_SUCCEEDED", false);
    status.seed_response_ok = env_bool_field(fields, "AMCL_SEED_RESPONSE_OK", false);
    status.nomotion_probe_used = env_bool_field(fields, "AMCL_NOMOTION_PROBE_USED", false);
    status.nomotion_pose_received =
      env_bool_field(fields, "AMCL_NOMOTION_POSE_RECEIVED", false);
    status.nomotion_pose_count = env_int_field(fields, "AMCL_NOMOTION_POSE_COUNT", 0);
    status.nomotion_pose_header_age_ms =
      env_double_field(fields, "AMCL_NOMOTION_POSE_HEADER_AGE_MS", -1.0);
    status.process_ready = env_bool_field(
      fields, "AMCL_PROCESS_READY",
      status.process_alive && status.node_exists && status.lifecycle_active);
    status.seeded = env_bool_field(
      fields, "AMCL_SEEDED",
      status.seed_succeeded || status.seed_response_ok);
    status.static_standby = env_bool_field(fields, "AMCL_STATIC_STANDBY", false);
    status.tracking_ready = env_bool_field(fields, "AMCL_TRACKING_READY", false);
    status.correction_ready = env_bool_field(fields, "AMCL_CORRECTION_READY", false);
    status.not_moving_no_update_ok =
      env_bool_field(fields, "AMCL_NOT_MOVING_NO_UPDATE_OK", false);
    status.stamp_sec = env_double_field(fields, "AMCL_STATUS_STAMP_SEC", -1.0);
    status.stamp = fields["TIMESTAMP"];
    if (status.stamp_sec > 0.0 && amcl_runtime_status_ttl_sec_ >= 0.0) {
      status.age_ms = std::max(0.0, (now().seconds() - status.stamp_sec) * 1000.0);
      status.stale = status.age_ms > amcl_runtime_status_ttl_sec_ * 1000.0;
    } else {
      status.age_ms = -1.0;
      status.stale = true;
    }
    return status;
  }

  void publish_status_if_due()
  {
    const double now_sec = now().seconds();
    if (last_status_publish_sec_ > 0.0 && now_sec - last_status_publish_sec_ < status_publish_period_sec_) {
      return;
    }
    const double elapsed = last_status_publish_sec_ > 0.0 ?
      now_sec - last_status_publish_sec_ : status_publish_period_sec_;
    last_status_publish_sec_ = now_sec;
    const double result_hz =
      rate_since_last_status(localization_result_count_, previous_localization_result_count_, elapsed);
    const double accepted_hz =
      rate_since_last_status(accepted_result_count_, previous_accepted_result_count_, elapsed);
    const double amcl_pose_hz =
      rate_since_last_status(amcl_pose_count_, previous_amcl_pose_count_, elapsed);
    double map_odom_publish_loop_hz = 0.0;
    double map_odom_publish_gap_ms = -1.0;
    double map_odom_publish_gap_max_ms = 0.0;
    double map_odom_publish_callback_duration_us = 0.0;
    std::uint64_t map_odom_latest_accepted_sequence = 0U;
    std::uint64_t map_odom_last_published_sequence = 0U;
    std::uint64_t map_odom_publish_missed_count = 0U;
    std::uint64_t map_odom_current_sequence = 0U;
    std::uint64_t map_odom_target_sequence = 0U;
    std::uint64_t map_odom_last_accepted_sequence = 0U;
    std::string map_odom_latest_source{"none"};
    std::string map_odom_current_source{"none"};
    std::string map_odom_target_source{"none"};
    std::string map_odom_last_correction_source{"none"};
    bool map_odom_state_valid = false;
    bool map_odom_correction_paused = false;
    bool map_odom_frozen_due_to_pause = false;
    bool map_odom_smoothing_enabled = false;
    bool map_odom_correction_active = false;
    bool map_odom_safe_for_goal_start = true;
    double map_odom_remaining_translation_error_m = 0.0;
    double map_odom_remaining_yaw_error_rad = 0.0;
    double map_odom_last_step_translation_m = 0.0;
    double map_odom_last_step_yaw_rad = 0.0;
    double map_odom_active_smoothing_translation_rate_mps = map_odom_smoothing_translation_rate_mps_;
    double map_odom_active_smoothing_yaw_rate_radps = map_odom_smoothing_yaw_rate_radps_;
    std::string map_odom_smoothing_policy{"default"};
    double map_odom_last_correction_accept_time = 0.0;
    double map_odom_last_correction_apply_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(map_odom_publish_stats_mutex_);
      const auto publish_delta = map_odom_publish_count_ - previous_map_odom_publish_count_;
      previous_map_odom_publish_count_ = map_odom_publish_count_;
      map_odom_publish_loop_hz =
        elapsed > 0.0 ? static_cast<double>(publish_delta) / elapsed : 0.0;
      map_odom_publish_gap_ms = map_odom_last_publish_gap_ms_;
      map_odom_publish_gap_max_ms = map_odom_publish_gap_max_ms_;
      map_odom_publish_callback_duration_us = map_odom_publish_callback_duration_us_;
      map_odom_latest_accepted_sequence = map_odom_latest_accepted_sequence_;
      map_odom_last_published_sequence = map_odom_last_published_sequence_;
      map_odom_publish_missed_count = map_odom_publish_missed_count_;
      map_odom_current_sequence = map_odom_current_sequence_snapshot_;
      map_odom_target_sequence = map_odom_target_sequence_snapshot_;
      map_odom_last_accepted_sequence = map_odom_last_accepted_sequence_snapshot_;
      map_odom_latest_source = map_odom_latest_source_;
      map_odom_current_source = map_odom_current_source_snapshot_;
      map_odom_target_source = map_odom_target_source_snapshot_;
      map_odom_last_correction_source = map_odom_last_correction_source_snapshot_;
      map_odom_state_valid = map_odom_state_valid_snapshot_;
      map_odom_correction_paused = map_odom_correction_paused_snapshot_;
      map_odom_frozen_due_to_pause = map_odom_frozen_due_to_pause_snapshot_;
      map_odom_smoothing_enabled = map_odom_smoothing_enabled_snapshot_;
      map_odom_correction_active = map_odom_correction_active_snapshot_;
      map_odom_safe_for_goal_start = map_odom_safe_for_goal_start_snapshot_;
      map_odom_remaining_translation_error_m = map_odom_remaining_translation_error_m_snapshot_;
      map_odom_remaining_yaw_error_rad = map_odom_remaining_yaw_error_rad_snapshot_;
      map_odom_last_step_translation_m = map_odom_last_step_translation_m_snapshot_;
      map_odom_last_step_yaw_rad = map_odom_last_step_yaw_rad_snapshot_;
      map_odom_active_smoothing_translation_rate_mps =
        map_odom_active_smoothing_translation_rate_mps_snapshot_;
      map_odom_active_smoothing_yaw_rate_radps = map_odom_active_smoothing_yaw_rate_radps_snapshot_;
      map_odom_smoothing_policy = map_odom_smoothing_policy_snapshot_;
      map_odom_last_correction_accept_time = map_odom_last_correction_accept_time_snapshot_;
      map_odom_last_correction_apply_time = map_odom_last_correction_apply_time_snapshot_;
    }

    const double map_to_odom_age_ms = last_accepted_sec_ > 0.0 ?
      (now_sec - last_accepted_sec_) * 1000.0 : -1.0;
    const double last_amcl_pose_age_ms = last_amcl_pose_received_sec_ > 0.0 ?
      (now_sec - last_amcl_pose_received_sec_) * 1000.0 : -1.0;
    const double amcl_initial_pose_age_ms = last_amcl_initial_pose_seed_sec_ > 0.0 ?
      (now_sec - last_amcl_initial_pose_seed_sec_) * 1000.0 : -1.0;
    const auto amcl_runtime_status = read_amcl_runtime_status_file();
    const std::size_t graph_amcl_pose_publisher_count = count_publishers(amcl_pose_topic_);
    const std::size_t graph_scan_admission_status_publisher_count =
      count_publishers(amcl_scan_admission_status_topic_);
    const bool amcl_runtime_status_authoritative =
      amcl_runtime_status.available && !amcl_runtime_status.stale;
    const std::string amcl_status_source = !amcl_runtime_status.available ?
      std::string("live") :
      (amcl_runtime_status_authoritative ? std::string("file") : std::string("stale_file_ignored"));
    const bool amcl_process_alive = amcl_runtime_status_authoritative ?
      amcl_runtime_status.process_alive :
      graph_amcl_pose_publisher_count > 0U;
    const bool amcl_lifecycle_active = amcl_runtime_status_authoritative ?
      amcl_runtime_status.lifecycle_active :
      graph_amcl_pose_publisher_count > 0U;
    const bool amcl_node_exists = amcl_runtime_status_authoritative ?
      amcl_runtime_status.node_exists :
      graph_amcl_pose_publisher_count > 0U;
    const bool amcl_scan_admission_alive = amcl_runtime_status_authoritative ?
      amcl_runtime_status.scan_admission_alive :
      graph_scan_admission_status_publisher_count > 0U;
    const bool amcl_pose_publisher_available =
      graph_amcl_pose_publisher_count > 0U &&
      (!amcl_runtime_status_authoritative || amcl_runtime_status.pose_publisher_count > 0);
    const bool amcl_scan_status_publisher_available =
      !amcl_scan_admission_enabled_ ||
      (graph_scan_admission_status_publisher_count > 0U &&
      (!amcl_runtime_status_authoritative ||
      amcl_runtime_status.scan_admission_status_publisher_count > 0));
    const bool amcl_scan_ready = amcl_scan_admission_ready(now_sec) && amcl_scan_status_publisher_available;
    const bool amcl_pose_seen = last_amcl_pose_age_ms >= 0.0 && amcl_pose_publisher_available;
    const bool amcl_pose_fresh =
      amcl_pose_seen && last_amcl_pose_age_ms <= amcl_pose_max_age_ms_;
    const double amcl_linear_speed_mps = has_odom_ ?
      std::hypot(
        latest_odom_.twist.twist.linear.x,
        latest_odom_.twist.twist.linear.y) : 0.0;
    const double amcl_angular_speed_rps = has_odom_ ?
      std::abs(latest_odom_.twist.twist.angular.z) : 0.0;
    const bool amcl_robot_moving = amcl_robot_moving_now();
    const bool amcl_runtime_static_standby =
      amcl_runtime_status_authoritative &&
      (amcl_runtime_status.static_standby ||
      amcl_runtime_status.not_moving_no_update_ok ||
      (amcl_runtime_status.tracking_ready && !amcl_runtime_status.correction_ready));
    const bool amcl_not_moving_no_update_ok =
      amcl_input_enabled_ && !amcl_robot_moving &&
      ((amcl_pose_seen && !amcl_pose_fresh) || amcl_runtime_status.not_moving_no_update_ok);
    const bool amcl_process_ready =
      amcl_input_enabled_ &&
      amcl_process_alive &&
      amcl_node_exists &&
      amcl_lifecycle_active &&
      amcl_pose_publisher_available;
    const bool amcl_seeded =
      amcl_input_enabled_ &&
      (amcl_seed_succeeded_ ||
      amcl_pose_seen ||
      amcl_runtime_status.seeded ||
      amcl_runtime_status.seed_succeeded ||
      amcl_runtime_status.seed_response_ok);
    const bool amcl_runtime_waiting_seed =
      amcl_runtime_status_authoritative &&
      amcl_runtime_status.start_result == "waiting_seed";
    const bool amcl_runtime_waiting_seed_resolved =
      amcl_runtime_waiting_seed &&
      amcl_process_ready &&
      amcl_seeded &&
      !amcl_robot_moving &&
      amcl_scan_ready;
    const bool amcl_runtime_ready =
      !amcl_runtime_status_authoritative ||
      amcl_runtime_status.ready ||
      amcl_runtime_waiting_seed_resolved;
    const bool amcl_runtime_not_ready =
      amcl_input_enabled_ &&
      amcl_runtime_status_authoritative &&
      !amcl_runtime_status.ready &&
      !amcl_runtime_waiting_seed_resolved;
    const bool amcl_upstream_ready =
      !amcl_input_enabled_ ||
      (amcl_process_alive &&
      amcl_lifecycle_active &&
      amcl_pose_publisher_available &&
      amcl_scan_admission_alive &&
      amcl_scan_status_publisher_available &&
      amcl_runtime_ready);
    const bool amcl_upstream_missing = amcl_input_enabled_ && !amcl_upstream_ready;
    const bool amcl_static_standby =
      amcl_process_ready &&
      amcl_seeded &&
      !amcl_robot_moving &&
      amcl_scan_ready &&
      ((amcl_pose_seen && !amcl_pose_fresh) ||
      amcl_runtime_static_standby ||
      amcl_runtime_waiting_seed_resolved);
    const bool amcl_tracking_ready =
      (amcl_process_ready &&
      amcl_seeded &&
      amcl_scan_ready &&
      (amcl_pose_fresh || amcl_static_standby)) ||
      (amcl_runtime_status_authoritative &&
      amcl_runtime_status.tracking_ready &&
      amcl_process_ready &&
      amcl_seeded &&
      amcl_scan_ready &&
      !amcl_robot_moving);
    const bool amcl_correction_ready =
      amcl_process_ready &&
      amcl_seeded &&
      amcl_scan_ready &&
      amcl_pose_fresh;
    const bool amcl_shadow_ready =
      amcl_input_enabled_ &&
      amcl_upstream_ready &&
      amcl_tracking_ready;
    const bool amcl_gated_ready =
      amcl_gate_mode_ == "gated" &&
      amcl_upstream_ready &&
      amcl_tracking_ready;
    const bool amcl_ready =
      amcl_gate_mode_ == "gated" ? amcl_gated_ready : amcl_shadow_ready;
    const bool amcl_correction_pending =
      amcl_gate_mode_ == "gated" &&
      amcl_ready &&
      !amcl_correction_ready;
    const bool amcl_correction_suppressed_after_seed =
      last_isaac_triggered_accept_sec_ > 0.0 &&
      now_sec - last_isaac_triggered_accept_sec_ < amcl_accept_after_isaac_delay_sec_;
    const double amcl_post_isaac_refine_reference_time_sec =
      amcl_post_isaac_refine_reference_sec();
    const double amcl_post_isaac_refine_age_sec =
      amcl_post_isaac_refine_reference_time_sec > 0.0 ?
      now_sec - amcl_post_isaac_refine_reference_time_sec : -1.0;
    const bool amcl_post_isaac_refine_active =
      amcl_post_isaac_refine_enabled_ &&
      amcl_post_isaac_refine_age_sec >= 0.0 &&
      amcl_post_isaac_refine_age_sec <= amcl_post_isaac_refine_window_sec_ &&
      amcl_gate_mode_ == "gated" &&
      (!amcl_post_isaac_refine_require_stationary_ || !amcl_robot_moving);
    const bool localization_degraded =
      amcl_input_enabled_ &&
      ((amcl_runtime_status_authoritative && amcl_runtime_status.degraded) ||
      amcl_runtime_not_ready ||
      amcl_upstream_missing ||
      (amcl_robot_moving && !amcl_pose_fresh) ||
      !amcl_tracking_ready);
    std::string amcl_degraded_reason =
      amcl_runtime_status_authoritative ? amcl_runtime_status.degraded_reason : std::string();
    if (amcl_degraded_reason.empty() && amcl_upstream_missing) {
      amcl_degraded_reason = "AMCL_UPSTREAM_MISSING";
    } else if (amcl_degraded_reason.empty() && amcl_runtime_not_ready) {
      amcl_degraded_reason = "AMCL_NOT_READY";
    } else if (amcl_degraded_reason.empty() && amcl_robot_moving && !amcl_pose_fresh) {
      amcl_degraded_reason = "AMCL_NOT_TRACKING";
    } else if (amcl_degraded_reason.empty() && !amcl_tracking_ready) {
      amcl_degraded_reason = "AMCL_TRACKING_NOT_READY";
    }
    if (!localization_degraded) {
      amcl_degraded_reason.clear();
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"localization_mode\":\"" << continuous_localization_mode_
        << "\",\"gate_mode\":\"" << last_gate_mode_
        << "\",\"active_correction_source\":\"" << json_escape(active_correction_source_)
        << "\",\"last_candidate_source\":\"" << json_escape(last_candidate_source_)
        << "\",\"last_accepted_source\":\"" << json_escape(last_accepted_source_)
        << "\",\"last_rejected_source\":\"" << json_escape(last_rejected_source_)
        << "\",\"last_explicit_relocalization_accept_time\":"
        << last_explicit_relocalization_accept_sec_
        << ",\"last_explicit_relocalization_source\":\""
        << json_escape(last_explicit_relocalization_source_)
        << "\",\"last_explicit_relocalization_sequence\":"
        << last_explicit_relocalization_sequence_
        << ",\"isaac_background_correction_removed\":true"
        << ",\"triggered_max_result_age_ms\":" << triggered_max_result_age_ms_
        << ",\"force_accept_armed_time\":" << force_accept_armed_sec_
        << ",\"force_accept_min_pose_stamp_slack_sec\":"
        << force_accept_min_pose_stamp_slack_sec_
        << ",\"force_accept_ignored_pretrigger_result_count\":"
        << force_accept_ignored_pretrigger_result_count_
        << ",\"last_force_accept_ignored_reason\":\""
        << json_escape(last_force_accept_ignored_reason_) << "\""
        << ",\"last_result_header_stamp\":" << last_result_header_stamp_sec_
        << ",\"last_result_receive_time\":" << last_result_receive_time_sec_
        << ",\"last_result_age_ms\":" << last_result_age_ms_
        << ",\"gate_result_age_limit_ms\":" << last_gate_result_age_limit_ms_
        << ",\"last_result_used_original_stamp\":true"
        << ",\"has_odom\":" << (has_odom_ ? "true" : "false")
        << ",\"last_tf_lookup_stamp\":" << last_tf_lookup_stamp_sec_
        << ",\"last_odom_tf_history_lookup_ok\":" << (last_odom_tf_history_lookup_ok_ ? "true" : "false")
        << ",\"latest_odom_tf_fresh\":" << (latest_odom_tf_fresh_ ? "true" : "false")
        << ",\"latest_odom_tf_age_ms\":" << latest_odom_tf_age_ms_
        << ",\"localization_result_hz\":" << result_hz
        << ",\"accepted_result_hz\":" << accepted_hz
        << ",\"accepted_result_count\":" << accepted_result_count_
        << ",\"rejected_result_count\":" << rejected_result_count_
        << ",\"triggered_result_count\":" << triggered_result_count_
        << ",\"shadow_candidate_count\":" << shadow_candidate_count_
        << ",\"amcl_input_enabled\":" << (amcl_input_enabled_ ? "true" : "false")
        << ",\"amcl_runtime_status_file\":\"" << json_escape(amcl_runtime_status_file_)
        << "\",\"amcl_runtime_status_available\":" << (amcl_runtime_status.available ? "true" : "false")
        << ",\"amcl_runtime_status_authoritative\":"
        << (amcl_runtime_status_authoritative ? "true" : "false")
        << ",\"amcl_status_file_stale\":" << (amcl_runtime_status.stale ? "true" : "false")
        << ",\"amcl_status_age_ms\":" << amcl_runtime_status.age_ms
        << ",\"amcl_status_source\":\"" << json_escape(amcl_status_source) << "\""
        << ",\"amcl_mode\":\"" << json_escape(amcl_runtime_status.mode)
        << "\",\"amcl_state\":\"" << json_escape(amcl_runtime_status.state)
        << "\",\"amcl_start_result\":\"" << json_escape(amcl_runtime_status.start_result)
        << "\",\"amcl_process_alive\":" << (amcl_process_alive ? "true" : "false")
        << ",\"amcl_node_exists\":" << (amcl_node_exists ? "true" : "false")
        << ",\"amcl_lifecycle_active\":" << (amcl_lifecycle_active ? "true" : "false")
        << ",\"amcl_process_ready\":" << (amcl_process_ready ? "true" : "false")
        << ",\"amcl_scan_admission_alive\":" << (amcl_scan_admission_alive ? "true" : "false")
        << ",\"amcl_pose_publisher_count\":" << graph_amcl_pose_publisher_count
        << ",\"amcl_scan_admission_status_publisher_count\":"
        << graph_scan_admission_status_publisher_count
        << ",\"amcl_degraded\":" << (localization_degraded ? "true" : "false")
        << ",\"amcl_degraded_reason\":\"" << json_escape(amcl_degraded_reason)
        << "\",\"amcl_runtime_status_stamp\":\"" << json_escape(amcl_runtime_status.stamp)
        << "\",\"amcl_upstream_missing\":" << (amcl_upstream_missing ? "true" : "false")
        << ",\"amcl_gate_mode\":\"" << json_escape(amcl_gate_mode_)
        << "\",\"amcl_pose_topic\":\"" << json_escape(amcl_pose_topic_)
        << "\",\"amcl_max_result_age_ms\":" << amcl_max_result_age_ms_
        << ",\"amcl_pose_max_age_ms\":" << amcl_pose_max_age_ms_
        << ",\"amcl_small_correction_translation_m\":" << amcl_small_correction_translation_m_
        << ",\"amcl_small_correction_yaw_rad\":" << amcl_small_correction_yaw_rad_
        << ",\"amcl_medium_correction_translation_m\":" << amcl_medium_correction_translation_m_
        << ",\"amcl_medium_correction_yaw_rad\":" << amcl_medium_correction_yaw_rad_
        << ",\"amcl_medium_correction_consistency_count\":"
        << amcl_medium_correction_consistency_count_
        << ",\"amcl_accept_corrections_while_moving\":"
        << (amcl_accept_corrections_while_moving_ ? "true" : "false")
        << ",\"amcl_moving_linear_speed_mps\":" << amcl_moving_linear_speed_mps_
        << ",\"amcl_moving_angular_speed_radps\":" << amcl_moving_angular_speed_radps_
        << ",\"amcl_robot_moving\":" << (amcl_robot_moving ? "true" : "false")
        << ",\"amcl_linear_speed_mps\":" << amcl_linear_speed_mps
        << ",\"amcl_angular_speed_radps\":" << amcl_angular_speed_rps
        << ",\"amcl_medium_candidate_agreement_count\":"
        << amcl_medium_candidate_agreement_count_
        << ",\"amcl_hard_reject_translation_m\":" << amcl_hard_reject_translation_m_
        << ",\"amcl_hard_reject_yaw_rad\":" << amcl_hard_reject_yaw_rad_
        << ",\"amcl_max_xy_covariance\":" << amcl_max_xy_covariance_
        << ",\"amcl_max_yaw_covariance\":" << amcl_max_yaw_covariance_
        << ",\"amcl_last_state\":\"" << json_escape(last_amcl_state_)
        << "\",\"amcl_pose_count\":" << amcl_pose_count_
        << ",\"amcl_candidate_count\":" << amcl_candidate_count_
        << ",\"amcl_accepted_count\":" << amcl_accepted_count_
        << ",\"amcl_rejected_count\":" << amcl_rejected_count_
        << ",\"amcl_shadow_candidate_count\":" << amcl_shadow_candidate_count_
        << ",\"amcl_suppressed_after_isaac_count\":" << amcl_suppressed_after_isaac_count_
        << ",\"amcl_post_isaac_refine_enabled\":"
        << (amcl_post_isaac_refine_enabled_ ? "true" : "false")
        << ",\"amcl_post_isaac_refine_active\":"
        << (amcl_post_isaac_refine_active ? "true" : "false")
        << ",\"amcl_post_isaac_refine_reference_time\":"
        << amcl_post_isaac_refine_reference_time_sec
        << ",\"amcl_post_isaac_refine_age_sec\":" << amcl_post_isaac_refine_age_sec
        << ",\"amcl_post_isaac_refine_window_sec\":" << amcl_post_isaac_refine_window_sec_
        << ",\"amcl_post_isaac_refine_max_translation_m\":"
        << amcl_post_isaac_refine_max_translation_m_
        << ",\"amcl_post_isaac_refine_max_yaw_rad\":"
        << amcl_post_isaac_refine_max_yaw_rad_
        << ",\"amcl_post_isaac_refine_consistency_count\":"
        << amcl_post_isaac_refine_consistency_count_
        << ",\"amcl_post_isaac_refine_agreement_count\":"
        << amcl_post_isaac_refine_agreement_count_
        << ",\"amcl_post_isaac_refine_candidate_count\":"
        << amcl_post_isaac_refine_candidate_count_
        << ",\"amcl_post_isaac_refine_accepted_count\":"
        << amcl_post_isaac_refine_accepted_count_
        << ",\"amcl_post_isaac_refine_rejected_count\":"
        << amcl_post_isaac_refine_rejected_count_
        << ",\"amcl_post_isaac_refine_waiting_count\":"
        << amcl_post_isaac_refine_waiting_count_
        << ",\"amcl_post_isaac_refined_sequence\":"
        << amcl_post_isaac_refined_sequence_
        << ",\"global_correction_paused\":" << (correction_paused_ ? "true" : "false")
        << ",\"correction_paused\":" << (correction_paused_ ? "true" : "false")
        << ",\"correction_pause_reason\":\"" << json_escape(correction_pause_reason_) << "\""
        << ",\"correction_pause_service\":\"" << json_escape(correction_pause_service_) << "\""
        << ",\"last_amcl_pose_age_ms\":" << last_amcl_pose_age_ms
        << ",\"amcl_last_pose_age_ms\":" << last_amcl_pose_age_ms
        << ",\"amcl_pose_age_ms\":" << last_amcl_pose_age_ms
        << ",\"amcl_pose_hz\":" << amcl_pose_hz
        << ",\"amcl_pose_fresh\":" << (amcl_pose_fresh ? "true" : "false")
        << ",\"amcl_robot_moving\":" << (amcl_robot_moving ? "true" : "false")
        << ",\"amcl_not_moving_no_update_ok\":" << (amcl_not_moving_no_update_ok ? "true" : "false")
        << ",\"amcl_seeded\":" << (amcl_seeded ? "true" : "false")
        << ",\"amcl_static_standby\":" << (amcl_static_standby ? "true" : "false")
        << ",\"amcl_tracking_ready\":" << (amcl_tracking_ready ? "true" : "false")
        << ",\"amcl_correction_ready\":" << (amcl_correction_ready ? "true" : "false")
        << ",\"amcl_correction_pending\":"
        << (amcl_correction_pending ? "true" : "false")
        << ",\"amcl_ready\":" << (amcl_ready ? "true" : "false")
        << ",\"amcl_shadow_ready\":" << (amcl_shadow_ready ? "true" : "false")
        << ",\"amcl_gated_ready\":" << (amcl_gated_ready ? "true" : "false")
        << ",\"last_amcl_xy_covariance\":" << last_amcl_xy_covariance_
        << ",\"last_amcl_yaw_covariance\":" << last_amcl_yaw_covariance_
        << ",\"amcl_seed_requested\":" << (amcl_seed_requested_ ? "true" : "false")
        << ",\"amcl_seed_succeeded\":" << (amcl_seed_succeeded_ ? "true" : "false")
        << ",\"amcl_seed_response_ok\":"
        << ((amcl_runtime_status.seed_response_ok || amcl_seed_succeeded_) ? "true" : "false")
        << ",\"amcl_nomotion_probe_used\":"
        << (amcl_runtime_status.nomotion_probe_used ? "true" : "false")
        << ",\"amcl_nomotion_pose_received\":"
        << (amcl_runtime_status.nomotion_pose_received ? "true" : "false")
        << ",\"amcl_nomotion_pose_count\":" << amcl_runtime_status.nomotion_pose_count
        << ",\"amcl_nomotion_pose_header_age_ms\":"
        << amcl_runtime_status.nomotion_pose_header_age_ms
        << ",\"amcl_seed_attempt_count\":" << amcl_seed_attempt_count_
        << ",\"amcl_seed_source\":\"" << json_escape(amcl_seed_source_)
        << "\",\"amcl_seed_last_error\":\"" << json_escape(amcl_seed_last_error_)
        << "\",\"amcl_initial_pose_seed_count\":" << amcl_initial_pose_seed_count_
        << ",\"amcl_initial_pose_published_count\":" << amcl_initial_pose_published_count_
        << ",\"amcl_initial_pose_age_ms\":" << amcl_initial_pose_age_ms
        << ",\"amcl_initial_pose_reason\":\"" << json_escape(last_amcl_initial_pose_reason_)
        << "\",\"amcl_initial_pose_subscribers\":" << last_amcl_initial_pose_subscribers_
        << ",\"amcl_scan_admission_enabled\":" << (amcl_scan_admission_enabled_ ? "true" : "false")
        << ",\"amcl_scan_admission_hz\":" << amcl_scan_admission_hz_
        << ",\"amcl_scan_admission_dropped_age_count\":" << amcl_scan_admission_dropped_age_count_
        << ",\"amcl_scan_admission_dropped_tf_count\":" << amcl_scan_admission_dropped_tf_count_
        << ",\"amcl_scan_frame_id\":\"" << json_escape(amcl_scan_frame_id_)
        << "\",\"amcl_scan_last_age_ms\":" << amcl_scan_last_age_ms_
        << ",\"amcl_message_filter_drop_detected\":" << (amcl_message_filter_drop_detected_ ? "true" : "false")
        << ",\"amcl_scan_admission_last_error\":\"" << json_escape(amcl_scan_admission_last_error_)
        << "\",\"last_accept_reason\":\"" << json_escape(last_accept_reason_)
        << "\",\"last_reject_reason\":\"" << json_escape(last_reject_reason_)
        << "\",\"amcl_last_reject_reason\":\"" << json_escape(last_reject_reason_)
        << "\",\"amcl_correction_suppressed_after_seed\":"
        << (amcl_correction_suppressed_after_seed ? "true" : "false")
        << ",\"localization_degraded\":" << (localization_degraded ? "true" : "false")
        << ",\"last_candidate_correction_translation_m\":" << last_candidate_correction_translation_m_
        << ",\"last_candidate_correction_yaw_rad\":" << last_candidate_correction_yaw_rad_
        << ",\"last_accepted_correction_translation_m\":" << last_accepted_correction_translation_m_
        << ",\"last_accepted_correction_yaw_rad\":" << last_accepted_correction_yaw_rad_
        << ",\"map_to_odom_age_ms\":" << map_to_odom_age_ms
        << ",\"map_odom_publish_loop_hz\":" << map_odom_publish_loop_hz
        << ",\"map_odom_publish_gap_ms\":" << map_odom_publish_gap_ms
        << ",\"map_odom_publish_gap_max_ms\":" << map_odom_publish_gap_max_ms
        << ",\"map_odom_publish_callback_duration_us\":"
        << map_odom_publish_callback_duration_us
        << ",\"map_odom_latest_accepted_sequence\":"
        << map_odom_latest_accepted_sequence
        << ",\"map_odom_last_published_sequence\":"
        << map_odom_last_published_sequence
        << ",\"map_odom_latest_source\":\"" << json_escape(map_odom_latest_source)
        << "\",\"map_odom_state_valid\":" << (map_odom_state_valid ? "true" : "false")
        << ",\"map_odom_correction_paused\":"
        << (map_odom_correction_paused ? "true" : "false")
        << ",\"map_odom_frozen_due_to_pause\":"
        << (map_odom_frozen_due_to_pause ? "true" : "false")
        << ",\"smoothing_enabled\":" << (map_odom_smoothing_enabled ? "true" : "false")
        << ",\"correction_active\":" << (map_odom_correction_active ? "true" : "false")
        << ",\"safe_for_goal_start\":" << (map_odom_safe_for_goal_start ? "true" : "false")
        << ",\"current_sequence\":" << map_odom_current_sequence
        << ",\"target_sequence\":" << map_odom_target_sequence
        << ",\"last_accepted_sequence\":" << map_odom_last_accepted_sequence
        << ",\"last_published_sequence\":" << map_odom_last_published_sequence
        << ",\"current_source\":\"" << json_escape(map_odom_current_source)
        << "\",\"target_source\":\"" << json_escape(map_odom_target_source)
        << "\",\"remaining_translation_error_m\":" << map_odom_remaining_translation_error_m
        << ",\"remaining_yaw_error_rad\":" << map_odom_remaining_yaw_error_rad
        << ",\"last_step_translation_m\":" << map_odom_last_step_translation_m
        << ",\"last_step_yaw_rad\":" << map_odom_last_step_yaw_rad
        << ",\"smoothing_policy\":\"" << json_escape(map_odom_smoothing_policy)
        << "\",\"smoothing_translation_rate_mps\":"
        << map_odom_active_smoothing_translation_rate_mps
        << ",\"smoothing_yaw_rate_radps\":" << map_odom_active_smoothing_yaw_rate_radps
        << ",\"configured_smoothing_translation_rate_mps\":"
        << map_odom_smoothing_translation_rate_mps_
        << ",\"configured_smoothing_yaw_rate_radps\":" << map_odom_smoothing_yaw_rate_radps_
        << ",\"explicit_relocalization_fast_smoothing_enabled\":"
        << (explicit_relocalization_fast_smoothing_enabled_ ? "true" : "false")
        << ",\"explicit_relocalization_fast_correction_translation_m\":"
        << explicit_relocalization_fast_correction_translation_m_
        << ",\"explicit_relocalization_fast_correction_yaw_rad\":"
        << explicit_relocalization_fast_correction_yaw_rad_
        << ",\"explicit_relocalization_fast_max_duration_sec\":"
        << explicit_relocalization_fast_max_duration_sec_
        << ",\"last_correction_delta_translation_m\":"
        << last_accepted_correction_translation_m_
        << ",\"last_correction_delta_yaw_rad\":" << last_accepted_correction_yaw_rad_
        << ",\"last_correction_source\":\"" << json_escape(map_odom_last_correction_source)
        << "\",\"last_correction_accept_time\":" << map_odom_last_correction_accept_time
        << ",\"last_correction_apply_time\":" << map_odom_last_correction_apply_time
        << ",\"large_correction_requires_recovery\":"
        << (map_odom_large_correction_requires_recovery_ ? "true" : "false")
        << ",\"large_correction_rejected_count\":" << large_correction_rejected_count_
        << ",\"online_correction_smoothed_count\":" << online_correction_smoothed_count_
        << ",\"online_correction_snap_count\":" << online_correction_snap_count_
        << ",\"map_odom_publish_missed_count\":" << map_odom_publish_missed_count
        << ",\"publisher_decoupled_from_correction\":true"
        << ",\"map_odom_publish_gap_warn_ms\":" << map_odom_publish_gap_warn_ms_
        << ",\"map_odom_publish_gap_fail_ms\":" << map_odom_publish_gap_fail_ms_
        << ",\"map_to_odom_publisher_owner\":\"robot_localization_bridge\""
        << ",\"expected_map_to_odom_owner\":\"robot_localization_bridge\""
        << ",\"has_map_to_odom\":" << (has_map_to_odom_ ? "true" : "false")
        << ",\"localization_settle_required\":false"
        << ",\"localization_settle_in_progress\":false"
        << ",\"localization_settle_start_time\":0.000"
        << ",\"localization_settle_reason\":\"none\""
        << ",\"localization_settle_min_ms\":0"
        << ",\"localization_settle_complete\":true"
        << ",\"localization_settle_failure_reason\":\"none\""
        << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  bool publish_tf_{true};
  bool two_d_mode_{true};
  bool require_result_frame_match_{true};
  bool amcl_input_enabled_{false};
  bool amcl_covariance_gate_enabled_{true};
  bool amcl_initial_pose_seed_enabled_{true};
  bool amcl_scan_admission_enabled_{false};
  bool amcl_accept_corrections_while_moving_{true};
  bool amcl_post_isaac_refine_enabled_{true};
  bool amcl_post_isaac_refine_require_stationary_{true};
  bool amcl_seed_requested_{false};
  bool amcl_seed_succeeded_{false};
  bool amcl_message_filter_drop_detected_{false};
  int amcl_medium_correction_consistency_count_{3};
  int amcl_medium_candidate_agreement_count_{0};
  int amcl_post_isaac_refine_consistency_count_{2};
  int amcl_post_isaac_refine_agreement_count_{0};
  int amcl_initial_pose_publish_repetitions_{3};
  int amcl_initial_pose_repeat_period_ms_{100};
  double jump_threshold_m_{1.0};
  double forced_jump_threshold_m_{20.0};
  double timeout_sec_{1.0};
  double publish_rate_hz_{10.0};
  double tf_future_stamp_offset_sec_{0.0};
  double triggered_max_result_age_ms_{5000.0};
  double force_accept_min_pose_stamp_slack_sec_{1.0};
  double max_odom_tf_age_ms_{100.0};
  double odom_tf_lookup_timeout_ms_{20.0};
  double triggered_hard_reject_translation_m_{20.0};
  double amcl_max_result_age_ms_{1000.0};
  double amcl_small_correction_translation_m_{0.07};
  double amcl_small_correction_yaw_rad_{0.20};
  double amcl_medium_correction_translation_m_{0.15};
  double amcl_medium_correction_yaw_rad_{0.20};
  double amcl_moving_linear_speed_mps_{0.02};
  double amcl_moving_angular_speed_radps_{0.02};
  double amcl_hard_reject_translation_m_{0.30};
  double amcl_hard_reject_yaw_rad_{0.8};
  double amcl_max_xy_covariance_{1.0};
  double amcl_max_yaw_covariance_{0.5};
  double amcl_accept_after_isaac_delay_sec_{2.0};
  double amcl_post_isaac_refine_window_sec_{10.0};
  double amcl_post_isaac_refine_max_translation_m_{0.12};
  double amcl_post_isaac_refine_max_yaw_rad_{0.10};
  double amcl_post_isaac_refine_agreement_translation_m_{0.08};
  double amcl_post_isaac_refine_agreement_yaw_rad_{0.08};
  double amcl_initial_pose_xy_covariance_{0.01};
  double amcl_initial_pose_yaw_covariance_{0.0076};
  double amcl_pose_max_age_ms_{1000.0};
  double amcl_scan_admission_hz_{0.0};
  double amcl_scan_last_age_ms_{-1.0};
  double last_amcl_scan_admission_status_received_sec_{0.0};
  double status_publish_period_sec_{1.0};
  double map_odom_publish_gap_warn_ms_{100.0};
  double map_odom_publish_gap_fail_ms_{250.0};
  double map_odom_smoothing_publish_rate_hz_{50.0};
  double map_odom_smoothing_translation_rate_mps_{0.20};
  double map_odom_smoothing_yaw_rate_radps_{0.25};
  double map_odom_smoothing_snap_translation_epsilon_m_{0.005};
  double map_odom_smoothing_snap_yaw_epsilon_rad_{0.005};
  double explicit_relocalization_fast_correction_translation_m_{1.0};
  double explicit_relocalization_fast_correction_yaw_rad_{0.35};
  double explicit_relocalization_fast_max_duration_sec_{3.0};
  double map_odom_large_correction_translation_m_{0.50};
  double map_odom_large_correction_yaw_rad_{0.35};
  double map_odom_online_hard_reject_translation_m_{0.80};
  double map_odom_online_hard_reject_yaw_rad_{0.80};
  double amcl_runtime_status_ttl_sec_{5.0};
  double latest_pose_received_sec_{0.0};
  double last_amcl_pose_received_sec_{0.0};
  double force_accept_armed_sec_{0.0};
  double last_result_header_stamp_sec_{-1.0};
  double last_result_receive_time_sec_{-1.0};
  double last_result_age_ms_{-1.0};
  double last_gate_result_age_limit_ms_{5000.0};
  double last_tf_lookup_stamp_sec_{-1.0};
  double latest_odom_tf_age_ms_{-1.0};
  double last_candidate_correction_translation_m_{0.0};
  double last_candidate_correction_yaw_rad_{0.0};
  double last_accepted_correction_translation_m_{0.0};
  double last_accepted_correction_yaw_rad_{0.0};
  double last_accepted_sec_{0.0};
  double last_explicit_relocalization_accept_sec_{0.0};
  double last_isaac_triggered_accept_sec_{0.0};
  double last_candidate_sec_{0.0};
  double last_status_publish_sec_{0.0};
  double last_amcl_xy_covariance_{-1.0};
  double last_amcl_yaw_covariance_{-1.0};
  double last_amcl_initial_pose_seed_sec_{0.0};
  std::uint64_t localization_result_count_{0U};
  std::uint64_t accepted_result_count_{0U};
  std::uint64_t rejected_result_count_{0U};
  std::uint64_t force_accept_ignored_pretrigger_result_count_{0U};
  std::uint64_t triggered_result_count_{0U};
  std::uint64_t last_explicit_relocalization_sequence_{0U};
  std::uint64_t shadow_candidate_count_{0U};
  std::uint64_t amcl_pose_count_{0U};
  std::uint64_t amcl_candidate_count_{0U};
  std::uint64_t amcl_accepted_count_{0U};
  std::uint64_t amcl_rejected_count_{0U};
  std::uint64_t amcl_shadow_candidate_count_{0U};
  std::uint64_t amcl_suppressed_after_isaac_count_{0U};
  std::uint64_t amcl_post_isaac_refine_candidate_count_{0U};
  std::uint64_t amcl_post_isaac_refine_accepted_count_{0U};
  std::uint64_t amcl_post_isaac_refine_rejected_count_{0U};
  std::uint64_t amcl_post_isaac_refine_waiting_count_{0U};
  std::uint64_t amcl_post_isaac_refined_sequence_{0U};
  std::uint64_t amcl_initial_pose_seed_count_{0U};
  std::uint64_t amcl_initial_pose_published_count_{0U};
  std::uint64_t amcl_seed_attempt_count_{0U};
  std::uint64_t amcl_scan_admission_dropped_age_count_{0U};
  std::uint64_t amcl_scan_admission_dropped_tf_count_{0U};
  std::size_t last_amcl_initial_pose_subscribers_{0U};
  std::uint64_t previous_localization_result_count_{0U};
  std::uint64_t previous_accepted_result_count_{0U};
  std::uint64_t previous_amcl_pose_count_{0U};
  std::uint64_t map_odom_publish_count_{0U};
  std::uint64_t previous_map_odom_publish_count_{0U};
  std::uint64_t map_odom_publish_missed_count_{0U};
  std::uint64_t map_odom_latest_accepted_sequence_{0U};
  std::uint64_t map_odom_last_published_sequence_{0U};
  std::uint64_t map_odom_current_sequence_snapshot_{0U};
  std::uint64_t map_odom_target_sequence_snapshot_{0U};
  std::uint64_t map_odom_last_accepted_sequence_snapshot_{0U};
  std::uint64_t map_odom_last_published_sequence_snapshot_{0U};
  std::uint64_t large_correction_rejected_count_{0U};
  std::uint64_t online_correction_smoothed_count_{0U};
  std::uint64_t online_correction_snap_count_{0U};
  double map_odom_last_publish_wall_sec_{0.0};
  double map_odom_last_publish_gap_ms_{-1.0};
  double map_odom_publish_gap_max_ms_{0.0};
  double map_odom_publish_callback_duration_us_{0.0};
  double map_odom_remaining_translation_error_m_snapshot_{0.0};
  double map_odom_remaining_yaw_error_rad_snapshot_{0.0};
  double map_odom_last_step_translation_m_snapshot_{0.0};
  double map_odom_last_step_yaw_rad_snapshot_{0.0};
  double map_odom_active_smoothing_translation_rate_mps_snapshot_{0.20};
  double map_odom_active_smoothing_yaw_rate_radps_snapshot_{0.25};
  double map_odom_last_correction_accept_time_snapshot_{0.0};
  double map_odom_last_correction_apply_time_snapshot_{0.0};
  bool map_odom_state_valid_snapshot_{false};
  bool map_odom_correction_paused_snapshot_{false};
  bool map_odom_frozen_due_to_pause_snapshot_{false};
  bool map_odom_smoothing_enabled_snapshot_{false};
  bool map_odom_correction_active_snapshot_{false};
  bool map_odom_safe_for_goal_start_snapshot_{true};
  bool explicit_relocalization_fast_smoothing_enabled_{true};
  std::string map_odom_latest_source_{"none"};
  std::string map_odom_current_source_snapshot_{"none"};
  std::string map_odom_target_source_snapshot_{"none"};
  std::string map_odom_smoothing_policy_snapshot_{"default"};
  std::string map_odom_last_correction_source_snapshot_{"none"};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string localization_topic_;
  std::string local_odom_topic_;
  std::string health_topic_;
  std::string status_topic_;
  std::string force_accept_service_;
  std::string correction_pause_service_;
  std::string amcl_pose_topic_;
  std::string amcl_runtime_status_file_;
  std::string amcl_gate_mode_{"shadow"};
  std::string amcl_initial_pose_topic_;
  std::string amcl_seed_service_;
  std::string amcl_scan_admission_status_topic_;
  std::string continuous_localization_mode_{"triggered"};
  std::string last_gate_mode_{"triggered"};
  std::string active_correction_source_{"none"};
  std::string last_candidate_source_{"none"};
  std::string last_accepted_source_{"none"};
  std::string last_explicit_relocalization_source_{"none"};
  std::string last_rejected_source_{"none"};
  std::string last_amcl_state_{"disabled"};
  std::string last_amcl_initial_pose_reason_{"none"};
  std::string amcl_seed_source_{"none"};
  std::string amcl_seed_last_error_{"none"};
  std::string amcl_scan_frame_id_;
  std::string amcl_scan_admission_last_error_{"none"};
  std::string last_amcl_scan_admission_status_;
  std::string correction_pause_reason_{"none"};
  std::string last_force_accept_ignored_reason_{"none"};

  bool has_pose_{false};
  bool has_odom_{false};
  bool has_map_to_odom_{false};
  bool has_last_pose_stamp_used_{false};
  bool has_last_health_{false};
  bool has_last_amcl_medium_candidate_{false};
  bool has_last_post_isaac_refine_candidate_{false};
  bool last_health_state_{false};
  bool force_accept_next_pose_{false};
  bool force_accept_next_pose_explicit_trigger_{false};
  bool correction_paused_{false};
  bool triggered_allow_large_correction_{true};
  bool map_odom_smoothing_enabled_{true};
  bool map_odom_large_correction_requires_recovery_{true};
  bool last_odom_tf_history_lookup_ok_{false};
  bool latest_odom_tf_fresh_{false};
  std::string last_health_reason_;
  std::string last_accept_reason_{"none"};
  std::string last_reject_reason_{"none"};
  builtin_interfaces::msg::Time last_pose_stamp_used_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  MapToOdom map_to_odom_;
  MapToOdom last_amcl_medium_candidate_;
  MapToOdom last_post_isaac_refine_candidate_;
  MapOdomState map_odom_state_;
  mutable std::mutex map_odom_state_mutex_;
  mutable std::mutex map_odom_publish_stats_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr amcl_scan_admission_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_initial_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_accept_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr correction_pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr amcl_seed_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr map_odom_publisher_callback_group_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
