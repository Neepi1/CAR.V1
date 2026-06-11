#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{
double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void rotate_xy(double & x, double & y, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double rx = c * x - s * y;
  const double ry = s * x + c * y;
  x = rx;
  y = ry;
}
}  // namespace

class LocalStateNode : public rclcpp::Node
{
public:
  LocalStateNode()
  : Node("robot_local_state")
  {
    mock_mode_ = declare_parameter<bool>("mock_mode", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    output_topic_ = declare_parameter<std::string>("output_topic", "/local_state/odometry");
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/wheel/odom");
    input_base_frame_ = declare_parameter<std::string>("input_base_frame", "base_link");
    declare_parameter<std::string>("input_imu_topic", "/lidar_imu");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_yaw_offset_rad_ = declare_parameter<double>("odom_yaw_offset_rad", 0.0);
    rotate_odom_position_with_yaw_offset_ =
      declare_parameter<bool>("rotate_odom_position_with_yaw_offset", true);
    anchor_pose_to_first_sample_ = declare_parameter<bool>("anchor_pose_to_first_sample", false);
    apply_twist_covariance_floor_ = declare_parameter<bool>("apply_twist_covariance_floor", false);
    apply_pose_covariance_floor_ = declare_parameter<bool>("apply_pose_covariance_floor", false);
    pose_covariance_floor_x_ = declare_parameter<double>("pose_covariance_floor_x", 0.0);
    pose_covariance_floor_y_ = declare_parameter<double>("pose_covariance_floor_y", 0.0);
    pose_covariance_floor_yaw_ = declare_parameter<double>("pose_covariance_floor_yaw", 0.0);
    twist_covariance_floor_vx_ = declare_parameter<double>("twist_covariance_floor_vx", 0.0);
    twist_covariance_floor_vy_ = declare_parameter<double>("twist_covariance_floor_vy", 0.0);
    twist_covariance_floor_vyaw_ = declare_parameter<double>("twist_covariance_floor_vyaw", 0.0);
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    publish_on_callback_ = declare_parameter<bool>("publish_on_callback", false);
    republish_latest_ = declare_parameter<bool>("republish_latest", true);
    republish_latest_max_age_sec_ =
      std::max(0.0, declare_parameter<double>("republish_latest_max_age_sec", 0.5));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(20));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    if (mock_mode_) {
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&LocalStateNode::on_mock_timer, this));
    } else {
      if (!publish_on_callback_ && !republish_latest_) {
        RCLCPP_WARN(
          get_logger(),
          "publish_on_callback=false and republish_latest=false would suppress odom output; "
          "enabling callback publication for compatibility");
        publish_on_callback_ = true;
      }
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        rclcpp::QoS(20),
        std::bind(&LocalStateNode::on_wheel_odom, this, std::placeholders::_1));
      if (republish_latest_) {
        timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
          std::bind(&LocalStateNode::on_republish_timer, this));
      }
    }
  }

private:
  void apply_canonical_odom_transform(nav_msgs::msg::Odometry & odom) const
  {
    if (std::abs(odom_yaw_offset_rad_) < 1e-12) {
      return;
    }

    if (rotate_odom_position_with_yaw_offset_) {
      rotate_xy(odom.pose.pose.position.x, odom.pose.pose.position.y, odom_yaw_offset_rad_);
    }

    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw + odom_yaw_offset_rad_));
  }

  static void apply_covariance_floor(double & value, const double floor)
  {
    if (floor <= 0.0) {
      return;
    }
    if (!std::isfinite(value) || value < floor) {
      value = floor;
    }
  }

  void apply_pose_anchor(nav_msgs::msg::Odometry & odom)
  {
    if (!anchor_pose_to_first_sample_) {
      return;
    }

    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    if (!has_pose_anchor_) {
      anchor_x_ = odom.pose.pose.position.x;
      anchor_y_ = odom.pose.pose.position.y;
      anchor_yaw_ = yaw;
      has_pose_anchor_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Anchored wheel odom pose at x=%.3f y=%.3f yaw=%.3f rad",
        anchor_x_,
        anchor_y_,
        anchor_yaw_);
    }

    double x = odom.pose.pose.position.x - anchor_x_;
    double y = odom.pose.pose.position.y - anchor_y_;
    rotate_xy(x, y, -anchor_yaw_);
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw - anchor_yaw_));
  }

  void apply_twist_covariance_floor(nav_msgs::msg::Odometry & odom) const
  {
    if (!apply_twist_covariance_floor_) {
      return;
    }
    apply_covariance_floor(odom.twist.covariance[0], twist_covariance_floor_vx_);
    apply_covariance_floor(odom.twist.covariance[7], twist_covariance_floor_vy_);
    apply_covariance_floor(odom.twist.covariance[35], twist_covariance_floor_vyaw_);
  }

  void apply_pose_covariance_floor(nav_msgs::msg::Odometry & odom) const
  {
    if (!apply_pose_covariance_floor_) {
      return;
    }
    apply_covariance_floor(odom.pose.covariance[0], pose_covariance_floor_x_);
    apply_covariance_floor(odom.pose.covariance[7], pose_covariance_floor_y_);
    apply_covariance_floor(odom.pose.covariance[35], pose_covariance_floor_yaw_);
  }

  void publish_local_state(const nav_msgs::msg::Odometry & odom)
  {
    odom_pub_->publish(odom);
    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg->child_frame_id.empty() && msg->child_frame_id != input_base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Input odom child_frame_id is '%s', expected native chassis frame '%s'. "
        "Publishing canonical child frame '%s'.",
        msg->child_frame_id.c_str(),
        input_base_frame_.c_str(),
        base_frame_.c_str());
    }
    nav_msgs::msg::Odometry local_odom = *msg;
    apply_pose_anchor(local_odom);
    apply_pose_covariance_floor(local_odom);
    apply_twist_covariance_floor(local_odom);
    apply_canonical_odom_transform(local_odom);
    local_odom.header.frame_id = odom_frame_;
    local_odom.child_frame_id = base_frame_;
    latest_local_odom_ = local_odom;
    latest_local_odom_received_sec_ = now().seconds();
    if (publish_on_callback_) {
      publish_local_state(local_odom);
    }
  }

  void on_republish_timer()
  {
    if (!latest_local_odom_) {
      return;
    }

    const auto stamp = now();
    const double age_sec = stamp.seconds() - latest_local_odom_received_sec_;
    if (age_sec > republish_latest_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping local odom republish because latest input odom is stale: %.3fs > %.3fs",
        age_sec, republish_latest_max_age_sec_);
      return;
    }

    auto odom = *latest_local_odom_;
    odom.header.stamp = stamp;
    publish_local_state(odom);
  }

  void on_mock_timer()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.orientation.w = 1.0;
    publish_local_state(odom);
  }

  bool mock_mode_{false};
  bool publish_tf_{true};
  std::string output_topic_;
  std::string input_odom_topic_;
  std::string input_base_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double odom_yaw_offset_rad_{0.0};
  bool rotate_odom_position_with_yaw_offset_{true};
  bool anchor_pose_to_first_sample_{false};
  bool has_pose_anchor_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};
  bool apply_twist_covariance_floor_{false};
  bool apply_pose_covariance_floor_{false};
  double pose_covariance_floor_x_{0.0};
  double pose_covariance_floor_y_{0.0};
  double pose_covariance_floor_yaw_{0.0};
  double twist_covariance_floor_vx_{0.0};
  double twist_covariance_floor_vy_{0.0};
  double twist_covariance_floor_vyaw_{0.0};
  double publish_rate_hz_{20.0};
  bool publish_on_callback_{false};
  bool republish_latest_{true};
  double republish_latest_max_age_sec_{0.5};
  std::optional<nav_msgs::msg::Odometry> latest_local_odom_;
  double latest_local_odom_received_sec_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalStateNode>());
  rclcpp::shutdown();
  return 0;
}
