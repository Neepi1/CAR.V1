#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
rclcpp::QoS make_qos(const std::size_t depth, const bool reliable)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.durability_volatile();
  if (reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}

double normalize_angle(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

tf2::Quaternion quaternion_from_msg(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  if (q.length2() <= 1.0e-24) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  return q;
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

tf2::Transform transform_from_pose(
  const geometry_msgs::msg::Point & position,
  const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::Transform(
    quaternion_from_msg(orientation),
    tf2::Vector3(position.x, position.y, position.z));
}

tf2::Transform transform_from_msg(const geometry_msgs::msg::TransformStamped & msg)
{
  const auto & t = msg.transform.translation;
  return tf2::Transform(
    quaternion_from_msg(msg.transform.rotation),
    tf2::Vector3(t.x, t.y, t.z));
}

double yaw_from_transform(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  transform.getBasis().getRPY(roll, pitch, yaw);
  return yaw;
}

builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time & stamp)
{
  std::int64_t total_ns = stamp.nanoseconds();
  if (total_ns < 0) {
    total_ns = 0;
  }
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(total_ns / 1000000000LL);
  msg.nanosec = static_cast<std::uint32_t>(total_ns % 1000000000LL);
  return msg;
}
}  // namespace

class FastlioOdomBridge : public rclcpp::Node
{
public:
  FastlioOdomBridge()
  : Node("fastlio_odom_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/fastlio/base_odometry");
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf_slam2d");
    output_odom_frame_ = declare_parameter<std::string>("output_odom_frame", "odom");
    output_base_frame_ = declare_parameter<std::string>("output_base_frame", "base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "lidar_link");
    anchor_on_first_sample_ = declare_parameter<bool>("anchor_on_first_sample", true);
    flatten_to_2d_ = declare_parameter<bool>("flatten_to_2d", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    restamp_output_to_now_ = declare_parameter<bool>("restamp_output_to_now", false);
    output_stamp_offset_sec_ = declare_parameter<double>("output_stamp_offset_sec", 0.0);
    input_reliable_ = declare_parameter<bool>("input_reliable", false);
    output_reliable_ = declare_parameter<bool>("output_reliable", true);
    input_qos_depth_ =
      static_cast<int>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("input_qos_depth", 1)));
    output_qos_depth_ =
      static_cast<int>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("output_qos_depth", 20)));
    tf_qos_depth_ =
      static_cast<int>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("tf_qos_depth", 100)));
    static_lookup_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("static_lookup_timeout_sec", 0.05));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, make_qos(static_cast<std::size_t>(output_qos_depth_), output_reliable_));
    if (publish_tf_) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
        tf_topic_, make_qos(static_cast<std::size_t>(tf_qos_depth_), true));
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      make_qos(static_cast<std::size_t>(input_qos_depth_), input_reliable_),
      std::bind(&FastlioOdomBridge::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "FAST-LIO C++ odom bridge: %s -> %s, frame=%s->%s, input_reliable=%s, "
      "output_reliable=%s, publish_tf=%s, restamp_output_to_now=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      output_odom_frame_.c_str(),
      output_base_frame_.c_str(),
      input_reliable_ ? "true" : "false",
      output_reliable_ ? "true" : "false",
      publish_tf_ ? "true" : "false",
      restamp_output_to_now_ ? "true" : "false");
  }

private:
  std::optional<tf2::Transform> lookup_base_from_child(const std::string & child_frame)
  {
    if (child_frame == output_base_frame_) {
      return tf2::Transform(
        tf2::Quaternion(0.0, 0.0, 0.0, 1.0),
        tf2::Vector3(0.0, 0.0, 0.0));
    }
    if (base_from_child_) {
      return base_from_child_;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        output_base_frame_,
        child_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(static_lookup_timeout_sec_));
      base_from_child_ = transform_from_msg(tf);
      RCLCPP_INFO(
        get_logger(),
        "using static %s->%s to convert FAST-LIO odom",
        output_base_frame_.c_str(),
        child_frame.c_str());
      return base_from_child_;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "waiting for static %s->%s: %s",
        output_base_frame_.c_str(),
        child_frame.c_str(),
        exc.what());
      return std::nullopt;
    }
  }

  builtin_interfaces::msg::Time output_stamp(const nav_msgs::msg::Odometry & msg)
  {
    if (!restamp_output_to_now_) {
      return msg.header.stamp;
    }
    auto stamp = get_clock()->now();
    if (std::abs(output_stamp_offset_sec_) > 1.0e-9) {
      stamp = stamp + rclcpp::Duration::from_seconds(output_stamp_offset_sec_);
    }
    return time_to_msg(stamp);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::string child_frame = msg->child_frame_id.empty() ? sensor_frame_ : msg->child_frame_id;
    if (!child_frame.empty() && child_frame.front() == '/') {
      child_frame.erase(0, 1);
    }

    const auto base_from_child = lookup_base_from_child(child_frame);
    if (!base_from_child) {
      return;
    }

    const auto odom_from_child =
      transform_from_pose(msg->pose.pose.position, msg->pose.pose.orientation);
    const auto odom_from_base = odom_from_child * base_from_child->inverse();
    auto translation = odom_from_base.getOrigin();
    double x = translation.x();
    double y = translation.y();
    double z = translation.z();
    double yaw = yaw_from_transform(odom_from_base);

    if (anchor_on_first_sample_) {
      if (!anchor_) {
        anchor_ = std::make_tuple(x, y, yaw);
      }
      const auto [anchor_x, anchor_y, anchor_yaw] = *anchor_;
      const double dx = x - anchor_x;
      const double dy = y - anchor_y;
      const double c = std::cos(anchor_yaw);
      const double s = std::sin(anchor_yaw);
      x = c * dx + s * dy;
      y = -s * dx + c * dy;
      yaw = normalize_angle(yaw - anchor_yaw);
    }

    if (flatten_to_2d_) {
      z = 0.0;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = output_stamp(*msg);
    odom.header.frame_id = output_odom_frame_;
    odom.child_frame_id = output_base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = quaternion_from_yaw(yaw);
    odom.pose.covariance = msg->pose.covariance;

    const auto base_rotation_from_child = base_from_child->getBasis();
    const auto base_translation_from_child = base_from_child->getOrigin();
    const tf2::Vector3 linear_child(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    const tf2::Vector3 angular_child(
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);
    const auto linear_base = base_rotation_from_child * linear_child;
    const auto angular_base = base_rotation_from_child * angular_child;
    const auto offset_velocity = angular_base.cross(base_translation_from_child);

    odom.twist.twist.linear.x = linear_base.x() - offset_velocity.x();
    odom.twist.twist.linear.y = linear_base.y() - offset_velocity.y();
    odom.twist.twist.linear.z =
      flatten_to_2d_ ? 0.0 : linear_base.z() - offset_velocity.z();
    odom.twist.twist.angular.x = flatten_to_2d_ ? 0.0 : angular_base.x();
    odom.twist.twist.angular.y = flatten_to_2d_ ? 0.0 : angular_base.y();
    odom.twist.twist.angular.z = angular_base.z();
    odom.twist.covariance = msg->twist.covariance;

    odom_pub_->publish(odom);

    if (!tf_pub_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = output_base_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tf);
    tf_pub_->publish(tf_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string tf_topic_;
  std::string output_odom_frame_;
  std::string output_base_frame_;
  std::string sensor_frame_;
  bool anchor_on_first_sample_{true};
  bool flatten_to_2d_{true};
  bool publish_tf_{false};
  bool restamp_output_to_now_{false};
  double output_stamp_offset_sec_{0.0};
  bool input_reliable_{false};
  bool output_reliable_{true};
  int input_qos_depth_{1};
  int output_qos_depth_{20};
  int tf_qos_depth_{100};
  double static_lookup_timeout_sec_{0.05};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::optional<tf2::Transform> base_from_child_;
  std::optional<std::tuple<double, double, double>> anchor_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
