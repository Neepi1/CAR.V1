#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

rclcpp::QoS make_qos(const std::size_t depth, const rmw_qos_reliability_policy_t reliability)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.durability_volatile();
  if (reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & msg)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : msg.fields) {
    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }
  }
  return has_x && has_y && has_z;
}

}  // namespace

class PointCloudAxisRemapNode : public rclcpp::Node
{
public:
  PointCloudAxisRemapNode()
  : Node("pointcloud_axis_remap"), logged_ready_(false), warned_missing_xyz_(false)
  {
    declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw");
    declare_parameter<std::string>("output_topic", "/lidar_points");
    declare_parameter<std::string>("output_frame_id", "lidar_link");
    declare_parameter<std::vector<double>>(
      "rotation_matrix",
      std::vector<double>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
      });

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    rotation_ = load_rotation_matrix();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, make_qos(10, RMW_QOS_POLICY_RELIABILITY_RELIABLE));
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      make_qos(10, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&PointCloudAxisRemapNode::on_cloud, this, std::placeholders::_1));
  }

private:
  std::array<float, 9> load_rotation_matrix() const
  {
    const auto raw = get_parameter("rotation_matrix").as_double_array();
    if (raw.size() != 9) {
      throw std::runtime_error("rotation_matrix must contain 9 values");
    }

    std::array<float, 9> rotation{};
    for (std::size_t i = 0; i < rotation.size(); ++i) {
      rotation[i] = static_cast<float>(raw[i]);
    }
    return rotation;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto output = *msg;
    output.header.frame_id = output_frame_id_;

    if (output.data.empty() || output.width == 0U || output.height == 0U) {
      publisher_->publish(output);
      return;
    }

    if (!has_xyz_fields(output)) {
      if (!warned_missing_xyz_) {
        RCLCPP_ERROR(
          get_logger(), "cloud on %s does not expose x/y/z fields", input_topic_.c_str());
        warned_missing_xyz_ = true;
      }
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

    const std::size_t point_count = static_cast<std::size_t>(output.width) * output.height;
    for (std::size_t index = 0; index < point_count; ++index, ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      *iter_x = rotation_[0] * x + rotation_[1] * y + rotation_[2] * z;
      *iter_y = rotation_[3] * x + rotation_[4] * y + rotation_[5] * z;
      *iter_z = rotation_[6] * x + rotation_[7] * y + rotation_[8] * z;
    }

    publisher_->publish(output);

    if (!logged_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "compiled canonical pointcloud remap ready: %s -> %s frame=%s",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_frame_id_.c_str());
      logged_ready_ = true;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::array<float, 9> rotation_{};
  bool logged_ready_;
  bool warned_missing_xyz_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudAxisRemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
