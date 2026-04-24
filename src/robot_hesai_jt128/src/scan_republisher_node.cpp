#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

bool env_bool(const char * name, const bool default_value)
{
  const char * raw = std::getenv(name);
  if (raw == nullptr) {
    return default_value;
  }
  std::string value{raw};
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

}  // namespace

class ScanRepublisherNode : public rclcpp::Node
{
public:
  ScanRepublisherNode()
  : Node("scan_republisher"),
    flip_scan_(env_bool("NJRH_SLAM2D_FLIP_SCAN", false))
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    restamp_to_now_ = declare_parameter<bool>("restamp_to_now", true);

    rclcpp::QoS sensor_qos{rclcpp::SensorDataQoS()};
    rclcpp::QoS pub_qos{rclcpp::KeepLast(10)};
    pub_qos.reliable();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, pub_qos);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_,
      sensor_qos,
      std::bind(&ScanRepublisherNode::on_scan, this, std::placeholders::_1));
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan outgoing = *msg;
    if (restamp_to_now_) {
      outgoing.header.stamp = now();
    }
    if (flip_scan_) {
      std::reverse(outgoing.ranges.begin(), outgoing.ranges.end());
      std::reverse(outgoing.intensities.begin(), outgoing.intensities.end());
      outgoing.angle_min = -msg->angle_max;
      outgoing.angle_max = -msg->angle_min;
      outgoing.angle_increment = msg->angle_increment;
    }
    pub_->publish(outgoing);
  }

  bool flip_scan_{false};
  bool restamp_to_now_{true};
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}
