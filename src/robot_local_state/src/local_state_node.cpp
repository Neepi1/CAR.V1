#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class LocalStateNode : public rclcpp::Node
{
public:
  LocalStateNode()
  : Node("robot_local_state"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    mock_mode_ = declare_parameter<bool>("mock_mode", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    output_topic_ = declare_parameter<std::string>("output_topic", "/local_state/odometry");
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/wheel/odom");
    declare_parameter<std::string>("input_imu_topic", "/lidar_imu");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(20));

    if (mock_mode_) {
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&LocalStateNode::on_mock_timer, this));
    } else {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        rclcpp::QoS(20),
        std::bind(&LocalStateNode::on_wheel_odom, this, std::placeholders::_1));
    }
  }

private:
  void publish_local_state(const nav_msgs::msg::Odometry & odom)
  {
    odom_pub_->publish(odom);
    if (!publish_tf_) {
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
    nav_msgs::msg::Odometry local_odom = *msg;
    local_odom.header.frame_id = odom_frame_;
    local_odom.child_frame_id = base_frame_;
    publish_local_state(local_odom);
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
  std::string odom_frame_;
  std::string base_frame_;
  double publish_rate_hz_{20.0};

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
