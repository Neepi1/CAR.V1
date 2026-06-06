#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

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

struct MapToOdom
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

}  // namespace

class LocalizationBridgeNode : public rclcpp::Node
{
public:
  LocalizationBridgeNode()
  : Node("robot_localization_bridge"),
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
    force_accept_service_ = declare_parameter<std::string>(
      "force_accept_service", "/robot_localization_bridge/force_accept_next_localization");
    two_d_mode_ = declare_parameter<bool>("two_d_mode", true);
    if (forced_jump_threshold_m_ < jump_threshold_m_) {
      RCLCPP_WARN(
        get_logger(),
        "forced_jump_threshold_m %.3f is below jump_threshold_m %.3f; clamping to normal threshold",
        forced_jump_threshold_m_,
        jump_threshold_m_);
      forced_jump_threshold_m_ = jump_threshold_m_;
    }
    if (publish_rate_hz_ < 1.0) {
      RCLCPP_WARN(
        get_logger(), "publish_rate_hz %.3f is too low; clamping to 1.0 Hz", publish_rate_hz_);
      publish_rate_hz_ = 1.0;
    }
    if (tf_future_stamp_offset_sec_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "tf_future_stamp_offset_sec %.3f is negative; clamping to 0.0",
        tf_future_stamp_offset_sec_);
      tf_future_stamp_offset_sec_ = 0.0;
    } else if (tf_future_stamp_offset_sec_ > 0.20) {
      RCLCPP_WARN(
        get_logger(),
        "tf_future_stamp_offset_sec %.3f is too large for Nav2; clamping to 0.20",
        tf_future_stamp_offset_sec_);
      tf_future_stamp_offset_sec_ = 0.20;
    }

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationBridgeNode::on_pose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_,
      rclcpp::QoS(20),
      std::bind(&LocalizationBridgeNode::on_odom, this, std::placeholders::_1));
    health_pub_ = create_publisher<std_msgs::msg::Bool>(health_topic_, rclcpp::QoS(10));
    force_accept_srv_ = create_service<std_srvs::srv::Trigger>(
      force_accept_service_,
      std::bind(
        &LocalizationBridgeNode::on_force_accept_request,
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
    refresh_state("pose");
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
    response->success = true;
    response->message = "next localization_result may update map->odom across normal jump threshold";
    RCLCPP_WARN(
      get_logger(),
      "force accepting next localization_result up to %.3f m map->odom jump",
      forced_jump_threshold_m_);
  }

  bool is_new_pose_stamp() const
  {
    if (!has_last_pose_stamp_used_) {
      return true;
    }
    return latest_pose_.header.stamp.sec != last_pose_stamp_used_.sec ||
           latest_pose_.header.stamp.nanosec != last_pose_stamp_used_.nanosec;
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
      const double map_x = latest_pose_.pose.pose.position.x;
      const double map_y = latest_pose_.pose.pose.position.y;
      const double map_yaw = yaw_from_quaternion(latest_pose_.pose.pose.orientation);
      const double odom_x = latest_odom_.pose.pose.position.x;
      const double odom_y = latest_odom_.pose.pose.position.y;
      const double odom_yaw = yaw_from_quaternion(latest_odom_.pose.pose.orientation);

      const double map_to_odom_yaw = std::atan2(std::sin(map_yaw - odom_yaw), std::cos(map_yaw - odom_yaw));
      const double cos_delta = std::cos(map_to_odom_yaw);
      const double sin_delta = std::sin(map_to_odom_yaw);
      const double map_to_odom_x = map_x - (cos_delta * odom_x - sin_delta * odom_y);
      const double map_to_odom_y = map_y - (sin_delta * odom_x + cos_delta * odom_y);

      if (has_map_to_odom_) {
        const double dx = map_to_odom_x - map_to_odom_.x;
        const double dy = map_to_odom_y - map_to_odom_.y;
        const double jump = std::hypot(dx, dy);
        if (jump > jump_threshold_m_) {
          if (!force_accept_next_pose_) {
            publish_health(false, "bridge map->odom jump rejected: " + std::to_string(jump) + " m (" + source + ")");
            return;
          }
          force_accept_next_pose_ = false;
          if (jump > forced_jump_threshold_m_) {
            publish_health(
              false,
              "bridge forced map->odom jump rejected: " + std::to_string(jump) + " m (" + source + ")");
            return;
          }
          RCLCPP_WARN(
            get_logger(),
            "forced map->odom jump accepted: %.3f m from %s",
            jump,
            source);
        }
      }
      force_accept_next_pose_ = false;

      map_to_odom_.x = map_to_odom_x;
      map_to_odom_.y = map_to_odom_y;
      map_to_odom_.yaw = map_to_odom_yaw;
      has_map_to_odom_ = true;
      last_pose_stamp_used_ = latest_pose_.header.stamp;
      has_last_pose_stamp_used_ = true;
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

  bool publish_tf_{true};
  bool two_d_mode_{true};
  double jump_threshold_m_{1.0};
  double forced_jump_threshold_m_{20.0};
  double timeout_sec_{1.0};
  double publish_rate_hz_{10.0};
  double tf_future_stamp_offset_sec_{0.0};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string localization_topic_;
  std::string local_odom_topic_;
  std::string health_topic_;
  std::string force_accept_service_;

  bool has_pose_{false};
  bool has_odom_{false};
  bool has_map_to_odom_{false};
  bool has_last_pose_stamp_used_{false};
  bool has_last_health_{false};
  bool last_health_state_{false};
  bool force_accept_next_pose_{false};
  double latest_pose_received_sec_{0.0};
  std::string last_health_reason_;
  builtin_interfaces::msg::Time last_pose_stamp_used_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_pose_;
  nav_msgs::msg::Odometry latest_odom_;
  MapToOdom map_to_odom_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_accept_srv_;
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
