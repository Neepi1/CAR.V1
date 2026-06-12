#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
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
    two_d_mode_ = declare_parameter<bool>("two_d_mode", true);
    const bool deprecated_continuous_localization_enabled = declare_parameter<bool>(
      "continuous_localization_enabled", false);
    continuous_localization_mode_ = declare_parameter<std::string>(
      "continuous_localization_mode", "triggered");
    const double legacy_max_result_age_ms =
      declare_parameter<double>("max_result_age_ms", 5000.0);
    triggered_max_result_age_ms_ = declare_parameter<double>(
      "triggered_max_result_age_ms", legacy_max_result_age_ms);
    max_odom_tf_age_ms_ = declare_parameter<double>("max_odom_tf_age_ms", 100.0);
    odom_tf_lookup_timeout_ms_ = declare_parameter<double>("odom_tf_lookup_timeout_ms", 20.0);
    triggered_allow_large_correction_ = declare_parameter<bool>(
      "triggered_allow_large_correction", true);
    triggered_hard_reject_translation_m_ = declare_parameter<double>(
      "triggered_hard_reject_translation_m", forced_jump_threshold_m_);
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    amcl_input_enabled_ = declare_parameter<bool>("amcl_input_enabled", false);
    amcl_gate_mode_ = declare_parameter<std::string>("amcl_gate_mode", "shadow");
    amcl_max_result_age_ms_ = declare_parameter<double>("amcl_max_result_age_ms", 500.0);
    amcl_small_correction_translation_m_ = declare_parameter<double>(
      "amcl_small_correction_translation_m", 0.20);
    amcl_small_correction_yaw_rad_ = declare_parameter<double>(
      "amcl_small_correction_yaw_rad", 0.20);
    amcl_large_correction_consistency_count_ = declare_parameter<int>(
      "amcl_large_correction_consistency_count", 3);
    amcl_hard_reject_translation_m_ = declare_parameter<double>(
      "amcl_hard_reject_translation_m", 1.0);
    amcl_hard_reject_yaw_rad_ = declare_parameter<double>(
      "amcl_hard_reject_yaw_rad", 0.8);
    amcl_covariance_gate_enabled_ = declare_parameter<bool>(
      "amcl_min_pose_covariance_ok", true);
    amcl_max_xy_covariance_ = declare_parameter<double>("amcl_max_xy_covariance", 1.0);
    amcl_max_yaw_covariance_ = declare_parameter<double>("amcl_max_yaw_covariance", 0.5);
    amcl_accept_after_isaac_delay_sec_ = declare_parameter<double>(
      "amcl_accept_when_isaac_recently_triggered_delay_sec", 2.0);
    amcl_initial_pose_topic_ = declare_parameter<std::string>(
      "amcl_initial_pose_topic", "/initialpose");
    amcl_initial_pose_seed_enabled_ = declare_parameter<bool>(
      "amcl_initial_pose_seed_enabled", true);
    amcl_initial_pose_xy_covariance_ = declare_parameter<double>(
      "amcl_initial_pose_xy_covariance", 0.25);
    amcl_initial_pose_yaw_covariance_ = declare_parameter<double>(
      "amcl_initial_pose_yaw_covariance", 0.25);
    amcl_seed_service_ = declare_parameter<std::string>(
      "amcl_seed_service", "/robot_localization_bridge/seed_amcl_initial_pose");
    status_publish_period_sec_ = declare_parameter<double>("status_publish_period_sec", 1.0);
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
    max_odom_tf_age_ms_ = std::max(1.0, max_odom_tf_age_ms_);
    odom_tf_lookup_timeout_ms_ = std::max(0.0, odom_tf_lookup_timeout_ms_);
    triggered_hard_reject_translation_m_ =
      std::max(jump_threshold_m_, triggered_hard_reject_translation_m_);
    amcl_max_result_age_ms_ = std::max(1.0, amcl_max_result_age_ms_);
    amcl_small_correction_translation_m_ =
      std::max(0.0, amcl_small_correction_translation_m_);
    amcl_small_correction_yaw_rad_ = std::max(0.0, amcl_small_correction_yaw_rad_);
    amcl_large_correction_consistency_count_ =
      std::max(1, amcl_large_correction_consistency_count_);
    amcl_hard_reject_translation_m_ =
      std::max(amcl_small_correction_translation_m_, amcl_hard_reject_translation_m_);
    amcl_hard_reject_yaw_rad_ =
      std::max(amcl_small_correction_yaw_rad_, amcl_hard_reject_yaw_rad_);
    amcl_max_xy_covariance_ = std::max(0.0, amcl_max_xy_covariance_);
    amcl_max_yaw_covariance_ = std::max(0.0, amcl_max_yaw_covariance_);
    amcl_accept_after_isaac_delay_sec_ =
      std::max(0.0, amcl_accept_after_isaac_delay_sec_);
    amcl_initial_pose_xy_covariance_ = std::max(0.0, amcl_initial_pose_xy_covariance_);
    amcl_initial_pose_yaw_covariance_ =
      std::max(0.0, amcl_initial_pose_yaw_covariance_);
    status_publish_period_sec_ = std::max(0.2, status_publish_period_sec_);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationBridgeNode::on_pose, this, std::placeholders::_1));
    if (amcl_input_enabled_) {
      amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        amcl_pose_topic_,
        rclcpp::QoS(20),
        std::bind(&LocalizationBridgeNode::on_amcl_pose, this, std::placeholders::_1));
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
    amcl_seed_srv_ = create_service<std_srvs::srv::Trigger>(
      amcl_seed_service_,
      std::bind(
        &LocalizationBridgeNode::on_amcl_seed_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&LocalizationBridgeNode::on_timer, this));
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    latest_pose_ = *msg;
    has_pose_ = true;
    latest_pose_received_sec_ = now().seconds();
    last_result_header_stamp_sec_ = stamp_to_sec(msg->header.stamp);
    last_result_receive_time_sec_ = latest_pose_received_sec_;
    ++localization_result_count_;
    refresh_state("pose");
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
    if (
      last_isaac_triggered_accept_sec_ > 0.0 &&
      now_sec - last_isaac_triggered_accept_sec_ < amcl_accept_after_isaac_delay_sec_)
    {
      ++amcl_suppressed_after_isaac_count_;
      last_amcl_state_ = "suppressed_after_isaac_triggered";
      last_reject_reason_ = "amcl_suppressed_after_isaac_triggered";
      last_rejected_source_ = amcl_source_name();
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
    refresh_state("timer");
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
    force_accept_next_pose_ = true;
    force_accept_next_pose_explicit_trigger_ = true;
    response->success = true;
    response->message = "next localization_result may update map->odom across normal jump threshold";
    RCLCPP_WARN(
      get_logger(),
      "force accepting next localization_result up to %.3f m map->odom jump",
      forced_jump_threshold_m_);
  }

  void on_amcl_seed_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped seed_pose;
    if (!current_map_base_pose(seed_pose)) {
      response->success = false;
      response->message = "cannot seed AMCL: map->odom or local odom is not available";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }
    publish_amcl_initial_pose(seed_pose, "current_map_base");
    response->success = true;
    response->message = "published AMCL /initialpose from current map->base_link";
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
      candidate.correction_translation_m = std::hypot(
        map_to_odom_x - map_to_odom_.x,
        map_to_odom_y - map_to_odom_.y);
      candidate.correction_yaw_rad = std::abs(normalize_yaw(map_to_odom_yaw - map_to_odom_.yaw));
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

  bool amcl_candidate_agrees_with_previous_large(const CandidateCorrection & candidate)
  {
    if (!has_last_amcl_large_candidate_) {
      last_amcl_large_candidate_ = candidate.transform;
      has_last_amcl_large_candidate_ = true;
      amcl_large_candidate_agreement_count_ = 1;
      return false;
    }
    const double dx = candidate.transform.x - last_amcl_large_candidate_.x;
    const double dy = candidate.transform.y - last_amcl_large_candidate_.y;
    const double dyaw =
      std::abs(normalize_yaw(candidate.transform.yaw - last_amcl_large_candidate_.yaw));
    if (
      std::hypot(dx, dy) <= amcl_small_correction_translation_m_ &&
      dyaw <= amcl_small_correction_yaw_rad_)
    {
      ++amcl_large_candidate_agreement_count_;
    } else {
      last_amcl_large_candidate_ = candidate.transform;
      amcl_large_candidate_agreement_count_ = 1;
    }
    return amcl_large_candidate_agreement_count_ >= amcl_large_correction_consistency_count_;
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

    if (candidate.correction_translation_m > jump_threshold_m_) {
      reject_candidate("triggered_jump_over_threshold", source);
      return false;
    }
    apply_candidate(candidate, source, "triggered_correction");
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
      reject_candidate(candidate.reject_reason, source, false);
      return false;
    }

    if (amcl_gate_mode_ == "shadow") {
      ++amcl_shadow_candidate_count_;
      ++shadow_candidate_count_;
      last_amcl_state_ = "shadow_candidate";
      last_reject_reason_ = "amcl_shadow_only";
      last_rejected_source_ = candidate.source;
      return false;
    }

    if (candidate_is_small(
        candidate,
        amcl_small_correction_translation_m_,
        amcl_small_correction_yaw_rad_))
    {
      has_last_amcl_large_candidate_ = false;
      amcl_large_candidate_agreement_count_ = 0;
      last_amcl_state_ = "accepted_small_correction";
      apply_candidate(candidate, source, "AMCL_SMALL_CORRECTION");
      return true;
    }

    if (candidate_is_hard_reject(
        candidate,
        amcl_hard_reject_translation_m_,
        amcl_hard_reject_yaw_rad_))
    {
      last_amcl_state_ = "hard_reject";
      reject_candidate("amcl_hard_reject_threshold", source, false);
      return false;
    }

    if (amcl_candidate_agrees_with_previous_large(candidate)) {
      last_amcl_state_ = "large_consistent_requires_isaac_recovery";
      reject_candidate("amcl_large_consistent_requires_isaac_recovery", source, false);
      return false;
    }

    last_amcl_state_ = "large_correction_waiting_for_consistency";
    reject_candidate("amcl_large_correction_waiting_for_consistency", source, false);
    return false;
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

  void apply_candidate(
    const CandidateCorrection & candidate,
    const char * source,
    const std::string & accept_reason)
  {
    map_to_odom_ = candidate.transform;
    has_map_to_odom_ = true;
    mark_latest_pose_stamp_used();
    last_accepted_correction_translation_m_ = candidate.correction_translation_m;
    last_accepted_correction_yaw_rad_ = candidate.correction_yaw_rad;
    last_accepted_sec_ = now().seconds();
    last_accept_reason_ = accept_reason;
    last_reject_reason_.clear();
    last_accepted_source_ = candidate.source;
    active_correction_source_ = candidate.source;
    ++accepted_result_count_;
    if (candidate.source == "amcl_gated") {
      ++amcl_accepted_count_;
    }
    if (candidate.source == "isaac_triggered") {
      last_isaac_triggered_accept_sec_ = last_accepted_sec_;
      if (amcl_initial_pose_seed_enabled_) {
        publish_amcl_initial_pose(candidate.map_base_pose, "isaac_triggered_accept");
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

  void publish_amcl_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & seed_pose,
    const std::string & reason)
  {
    if (!amcl_initial_pose_seed_enabled_ || !amcl_initial_pose_pub_) {
      return;
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
    }
    amcl_initial_pose_pub_->publish(msg);
    last_amcl_initial_pose_seed_sec_ = now().seconds();
    last_amcl_initial_pose_reason_ = reason;
    ++amcl_initial_pose_seed_count_;
  }

  bool current_map_base_pose(geometry_msgs::msg::PoseWithCovarianceStamped & pose) const
  {
    if (!has_map_to_odom_ || !has_odom_) {
      return false;
    }
    const double odom_x = latest_odom_.pose.pose.position.x;
    const double odom_y = latest_odom_.pose.pose.position.y;
    const double odom_yaw = yaw_from_quaternion(latest_odom_.pose.pose.orientation);
    const double cos_delta = std::cos(map_to_odom_.yaw);
    const double sin_delta = std::sin(map_to_odom_.yaw);
    const double map_x = map_to_odom_.x + (cos_delta * odom_x - sin_delta * odom_y);
    const double map_y = map_to_odom_.y + (sin_delta * odom_x + cos_delta * odom_y);
    const double map_yaw = normalize_yaw(map_to_odom_.yaw + odom_yaw);

    pose.header = latest_odom_.header;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = map_x;
    pose.pose.pose.position.y = map_y;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation = quaternion_from_yaw(map_yaw);
    fill_amcl_initial_pose_covariance(pose);
    return true;
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
    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    auto tf_stamp = now();
    if (tf_future_stamp_offset_sec_ > 0.0) {
      tf_stamp = tf_stamp + rclcpp::Duration::from_seconds(tf_future_stamp_offset_sec_);
    }
    tf.header.stamp = tf_stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_to_odom_.x;
    tf.transform.translation.y = map_to_odom_.y;
    if (!two_d_mode_ && has_pose_) {
      tf.transform.translation.z = latest_pose_.pose.pose.position.z - latest_odom_.pose.pose.position.z;
    }
    tf.transform.rotation = quaternion_from_yaw(map_to_odom_.yaw);
    tf_broadcaster_->sendTransform(tf);
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

    const double map_to_odom_age_ms = last_accepted_sec_ > 0.0 ?
      (now_sec - last_accepted_sec_) * 1000.0 : -1.0;
    const double last_amcl_pose_age_ms = last_amcl_pose_received_sec_ > 0.0 ?
      (now_sec - last_amcl_pose_received_sec_) * 1000.0 : -1.0;
    const double amcl_initial_pose_age_ms = last_amcl_initial_pose_seed_sec_ > 0.0 ?
      (now_sec - last_amcl_initial_pose_seed_sec_) * 1000.0 : -1.0;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"localization_mode\":\"" << continuous_localization_mode_
        << "\",\"gate_mode\":\"" << last_gate_mode_
        << "\",\"active_correction_source\":\"" << json_escape(active_correction_source_)
        << "\",\"last_candidate_source\":\"" << json_escape(last_candidate_source_)
        << "\",\"last_accepted_source\":\"" << json_escape(last_accepted_source_)
        << "\",\"last_rejected_source\":\"" << json_escape(last_rejected_source_)
        << "\",\"isaac_background_correction_removed\":true"
        << ",\"triggered_max_result_age_ms\":" << triggered_max_result_age_ms_
        << ",\"last_result_header_stamp\":" << last_result_header_stamp_sec_
        << ",\"last_result_receive_time\":" << last_result_receive_time_sec_
        << ",\"last_result_age_ms\":" << last_result_age_ms_
        << ",\"gate_result_age_limit_ms\":" << last_gate_result_age_limit_ms_
        << ",\"last_result_used_original_stamp\":true"
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
        << ",\"amcl_gate_mode\":\"" << json_escape(amcl_gate_mode_)
        << "\",\"amcl_pose_topic\":\"" << json_escape(amcl_pose_topic_)
        << "\",\"amcl_max_result_age_ms\":" << amcl_max_result_age_ms_
        << ",\"amcl_small_correction_translation_m\":" << amcl_small_correction_translation_m_
        << ",\"amcl_small_correction_yaw_rad\":" << amcl_small_correction_yaw_rad_
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
        << ",\"last_amcl_pose_age_ms\":" << last_amcl_pose_age_ms
        << ",\"last_amcl_xy_covariance\":" << last_amcl_xy_covariance_
        << ",\"last_amcl_yaw_covariance\":" << last_amcl_yaw_covariance_
        << ",\"amcl_initial_pose_seed_count\":" << amcl_initial_pose_seed_count_
        << ",\"amcl_initial_pose_age_ms\":" << amcl_initial_pose_age_ms
        << ",\"amcl_initial_pose_reason\":\"" << json_escape(last_amcl_initial_pose_reason_)
        << "\",\"amcl_initial_pose_subscribers\":" << last_amcl_initial_pose_subscribers_
        << ",\"last_accept_reason\":\"" << json_escape(last_accept_reason_)
        << "\",\"last_reject_reason\":\"" << json_escape(last_reject_reason_)
        << "\",\"last_candidate_correction_translation_m\":" << last_candidate_correction_translation_m_
        << ",\"last_candidate_correction_yaw_rad\":" << last_candidate_correction_yaw_rad_
        << ",\"last_accepted_correction_translation_m\":" << last_accepted_correction_translation_m_
        << ",\"last_accepted_correction_yaw_rad\":" << last_accepted_correction_yaw_rad_
        << ",\"map_to_odom_age_ms\":" << map_to_odom_age_ms
        << ",\"map_to_odom_publisher_owner\":\"robot_localization_bridge\""
        << ",\"has_map_to_odom\":" << (has_map_to_odom_ ? "true" : "false")
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
  int amcl_large_correction_consistency_count_{3};
  int amcl_large_candidate_agreement_count_{0};
  double jump_threshold_m_{1.0};
  double forced_jump_threshold_m_{20.0};
  double timeout_sec_{1.0};
  double publish_rate_hz_{10.0};
  double tf_future_stamp_offset_sec_{0.0};
  double triggered_max_result_age_ms_{5000.0};
  double max_odom_tf_age_ms_{100.0};
  double odom_tf_lookup_timeout_ms_{20.0};
  double triggered_hard_reject_translation_m_{20.0};
  double amcl_max_result_age_ms_{500.0};
  double amcl_small_correction_translation_m_{0.20};
  double amcl_small_correction_yaw_rad_{0.20};
  double amcl_hard_reject_translation_m_{1.0};
  double amcl_hard_reject_yaw_rad_{0.8};
  double amcl_max_xy_covariance_{1.0};
  double amcl_max_yaw_covariance_{0.5};
  double amcl_accept_after_isaac_delay_sec_{2.0};
  double amcl_initial_pose_xy_covariance_{0.25};
  double amcl_initial_pose_yaw_covariance_{0.25};
  double status_publish_period_sec_{1.0};
  double latest_pose_received_sec_{0.0};
  double last_amcl_pose_received_sec_{0.0};
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
  double last_isaac_triggered_accept_sec_{0.0};
  double last_candidate_sec_{0.0};
  double last_status_publish_sec_{0.0};
  double last_amcl_xy_covariance_{-1.0};
  double last_amcl_yaw_covariance_{-1.0};
  double last_amcl_initial_pose_seed_sec_{0.0};
  std::uint64_t localization_result_count_{0U};
  std::uint64_t accepted_result_count_{0U};
  std::uint64_t rejected_result_count_{0U};
  std::uint64_t triggered_result_count_{0U};
  std::uint64_t shadow_candidate_count_{0U};
  std::uint64_t amcl_pose_count_{0U};
  std::uint64_t amcl_candidate_count_{0U};
  std::uint64_t amcl_accepted_count_{0U};
  std::uint64_t amcl_rejected_count_{0U};
  std::uint64_t amcl_shadow_candidate_count_{0U};
  std::uint64_t amcl_suppressed_after_isaac_count_{0U};
  std::uint64_t amcl_initial_pose_seed_count_{0U};
  std::size_t last_amcl_initial_pose_subscribers_{0U};
  std::uint64_t previous_localization_result_count_{0U};
  std::uint64_t previous_accepted_result_count_{0U};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string localization_topic_;
  std::string local_odom_topic_;
  std::string health_topic_;
  std::string status_topic_;
  std::string force_accept_service_;
  std::string amcl_pose_topic_;
  std::string amcl_gate_mode_{"shadow"};
  std::string amcl_initial_pose_topic_;
  std::string amcl_seed_service_;
  std::string continuous_localization_mode_{"triggered"};
  std::string last_gate_mode_{"triggered"};
  std::string active_correction_source_{"none"};
  std::string last_candidate_source_{"none"};
  std::string last_accepted_source_{"none"};
  std::string last_rejected_source_{"none"};
  std::string last_amcl_state_{"disabled"};
  std::string last_amcl_initial_pose_reason_{"none"};

  bool has_pose_{false};
  bool has_odom_{false};
  bool has_map_to_odom_{false};
  bool has_last_pose_stamp_used_{false};
  bool has_last_health_{false};
  bool has_last_amcl_large_candidate_{false};
  bool last_health_state_{false};
  bool force_accept_next_pose_{false};
  bool force_accept_next_pose_explicit_trigger_{false};
  bool triggered_allow_large_correction_{true};
  bool last_odom_tf_history_lookup_ok_{false};
  bool latest_odom_tf_fresh_{false};
  std::string last_health_reason_;
  std::string last_accept_reason_{"none"};
  std::string last_reject_reason_{"none"};
  builtin_interfaces::msg::Time last_pose_stamp_used_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  MapToOdom map_to_odom_;
  MapToOdom last_amcl_large_candidate_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_initial_pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_accept_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr amcl_seed_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
