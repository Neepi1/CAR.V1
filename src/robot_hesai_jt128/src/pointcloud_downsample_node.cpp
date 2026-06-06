#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

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

bool find_float32_field_offset(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name,
  std::size_t & offset)
{
  for (const auto & field : msg.fields) {
    if (
      field.name == name &&
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count >= 1)
    {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

}  // namespace

class PointCloudDownsampleNode : public rclcpp::Node
{
public:
  PointCloudDownsampleNode()
  : Node("pointcloud_downsample"), logged_ready_(false), warned_missing_xyz_(false)
  {
    declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw");
    declare_parameter<std::string>("output_frame_id", "lidar_link");
    declare_parameter<std::string>("nav_output_topic", "/lidar_points_nav");
    declare_parameter<int>("nav_output_stride", 1);
    declare_parameter<int>("nav_output_qos_depth", 1);
    declare_parameter<std::string>("local_output_topic", "");
    declare_parameter<int>("local_output_stride", 1);
    declare_parameter<int>("local_output_qos_depth", 1);
    declare_parameter<int>("input_qos_depth", 1);
    declare_parameter<bool>("input_reliable", false);
    declare_parameter<std::vector<double>>(
      "rotation_matrix",
      std::vector<double>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
      });

    input_topic_ = get_parameter("input_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    nav_output_topic_ = get_parameter("nav_output_topic").as_string();
    nav_output_stride_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("nav_output_stride").as_int(), 1));
    local_output_topic_ = get_parameter("local_output_topic").as_string();
    local_output_stride_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("local_output_stride").as_int(), 1));
    const auto nav_output_qos_depth = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("nav_output_qos_depth").as_int(), 1));
    const auto local_output_qos_depth = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("local_output_qos_depth").as_int(), 1));
    const auto input_qos_depth = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("input_qos_depth").as_int(), 1));
    const auto input_reliability = get_parameter("input_reliable").as_bool() ?
      RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rotation_ = load_rotation_matrix();

    if (!nav_output_topic_.empty()) {
      nav_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        nav_output_topic_, make_qos(nav_output_qos_depth, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (!local_output_topic_.empty()) {
      local_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        local_output_topic_,
        make_qos(local_output_qos_depth, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, make_qos(input_qos_depth, input_reliability),
      std::bind(&PointCloudDownsampleNode::on_cloud, this, std::placeholders::_1));
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

  void rotate_point(
    std::uint8_t * point,
    const std::size_t x_offset,
    const std::size_t y_offset,
    const std::size_t z_offset) const
  {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::memcpy(&x, point + x_offset, sizeof(float));
    std::memcpy(&y, point + y_offset, sizeof(float));
    std::memcpy(&z, point + z_offset, sizeof(float));

    const float rotated_x = rotation_[0] * x + rotation_[1] * y + rotation_[2] * z;
    const float rotated_y = rotation_[3] * x + rotation_[4] * y + rotation_[5] * z;
    const float rotated_z = rotation_[6] * x + rotation_[7] * y + rotation_[8] * z;

    std::memcpy(point + x_offset, &rotated_x, sizeof(float));
    std::memcpy(point + y_offset, &rotated_y, sizeof(float));
    std::memcpy(point + z_offset, &rotated_z, sizeof(float));
  }

  void rotate_point_fast_xy(
    std::uint8_t * point,
    const std::size_t x_offset,
    const std::size_t y_offset,
    const bool negate_raw_y) const
  {
    float raw_x = 0.0F;
    float raw_y = 0.0F;
    std::memcpy(&raw_x, point + x_offset, sizeof(float));
    std::memcpy(&raw_y, point + y_offset, sizeof(float));

    const float rotated_x = negate_raw_y ? -raw_y : raw_y;
    const float rotated_y = -raw_x;
    std::memcpy(point + x_offset, &rotated_x, sizeof(float));
    std::memcpy(point + y_offset, &rotated_y, sizeof(float));
  }

  void publish_downsample(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::size_t stride,
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!publisher) {
      return;
    }

    const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    output->header = cloud.header;
    output->header.frame_id = output_frame_id_;
    output->fields = cloud.fields;
    output->is_bigendian = cloud.is_bigendian;
    output->point_step = cloud.point_step;
    output->height = 1;
    output->is_dense = cloud.is_dense;

    if (point_count == 0U || cloud.point_step == 0U || cloud.data.empty()) {
      output->width = 0;
      output->row_step = 0;
      publisher->publish(std::move(output));
      return;
    }

    const std::size_t sampled_count = (point_count + stride - 1U) / stride;
    output->width = static_cast<std::uint32_t>(sampled_count);
    output->row_step = static_cast<std::uint32_t>(sampled_count * cloud.point_step);
    output->data.resize(static_cast<std::size_t>(output->row_step));

    std::size_t x_offset = 0U;
    std::size_t y_offset = 0U;
    std::size_t z_offset = 0U;
    const bool can_rotate =
      !cloud.is_bigendian &&
      find_float32_field_offset(cloud, "x", x_offset) &&
      find_float32_field_offset(cloud, "y", y_offset) &&
      find_float32_field_offset(cloud, "z", z_offset) &&
      x_offset + sizeof(float) <= cloud.point_step &&
      y_offset + sizeof(float) <= cloud.point_step &&
      z_offset + sizeof(float) <= cloud.point_step;
    const bool fast_path_identity =
      rotation_[0] == 1.0F && rotation_[1] == 0.0F && rotation_[2] == 0.0F &&
      rotation_[3] == 0.0F && rotation_[4] == 1.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;
    const bool fast_path_raw_y_neg_raw_x =
      rotation_[0] == 0.0F && rotation_[1] == 1.0F && rotation_[2] == 0.0F &&
      rotation_[3] == -1.0F && rotation_[4] == 0.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;
    const bool fast_path_neg_raw_y_neg_raw_x =
      rotation_[0] == 0.0F && rotation_[1] == -1.0F && rotation_[2] == 0.0F &&
      rotation_[3] == -1.0F && rotation_[4] == 0.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;
    const bool can_fast_xy =
      can_rotate &&
      (fast_path_raw_y_neg_raw_x || fast_path_neg_raw_y_neg_raw_x);
    if (!can_rotate && !warned_missing_xyz_) {
      RCLCPP_ERROR(
        get_logger(),
        "cloud on %s does not expose little-endian FLOAT32 x/y/z fields; downsample will publish unrotated points",
        input_topic_.c_str());
      warned_missing_xyz_ = true;
    }

    std::size_t output_offset = 0U;
    if (cloud.height == 1U && cloud.row_step == cloud.width * cloud.point_step) {
      for (std::size_t point_index = 0U; point_index < point_count; point_index += stride) {
        const std::size_t input_offset = point_index * cloud.point_step;
        if (input_offset + cloud.point_step > cloud.data.size()) {
          break;
        }
        std::memcpy(output->data.data() + output_offset, cloud.data.data() + input_offset, cloud.point_step);
        if (can_fast_xy) {
          rotate_point_fast_xy(
            output->data.data() + output_offset, x_offset, y_offset, fast_path_neg_raw_y_neg_raw_x);
        } else if (can_rotate && !fast_path_identity) {
          rotate_point(output->data.data() + output_offset, x_offset, y_offset, z_offset);
        }
        output_offset += cloud.point_step;
      }
    } else {
      for (std::size_t flat_index = 0U; flat_index < point_count; flat_index += stride) {
        const std::size_t row = flat_index / cloud.width;
        const std::size_t column = flat_index % cloud.width;
        const std::size_t input_offset = row * cloud.row_step + column * cloud.point_step;
        if (input_offset + cloud.point_step > cloud.data.size()) {
          break;
        }
        std::memcpy(output->data.data() + output_offset, cloud.data.data() + input_offset, cloud.point_step);
        if (can_fast_xy) {
          rotate_point_fast_xy(
            output->data.data() + output_offset, x_offset, y_offset, fast_path_neg_raw_y_neg_raw_x);
        } else if (can_rotate && !fast_path_identity) {
          rotate_point(output->data.data() + output_offset, x_offset, y_offset, z_offset);
        }
        output_offset += cloud.point_step;
      }
    }

    if (output_offset != output->data.size()) {
      const std::size_t actual_count = output_offset / cloud.point_step;
      output->width = static_cast<std::uint32_t>(actual_count);
      output->row_step = static_cast<std::uint32_t>(output_offset);
      output->data.resize(output_offset);
    }
    publisher->publish(std::move(output));
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    publish_downsample(nav_publisher_, nav_output_stride_, *msg);
    publish_downsample(local_publisher_, local_output_stride_, *msg);

    if (!logged_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "compiled pointcloud downsample ready: input=%s frame=%s nav_output=%s stride=%zu local_output=%s stride=%zu",
        input_topic_.c_str(),
        output_frame_id_.c_str(),
        nav_output_topic_.empty() ? "(disabled)" : nav_output_topic_.c_str(),
        nav_output_stride_,
        local_output_topic_.empty() ? "(disabled)" : local_output_topic_.c_str(),
        local_output_stride_);
      logged_ready_ = true;
    }
  }

  std::string input_topic_;
  std::string output_frame_id_;
  std::string nav_output_topic_;
  std::size_t nav_output_stride_;
  std::string local_output_topic_;
  std::size_t local_output_stride_;
  std::array<float, 9> rotation_{};
  bool logged_ready_;
  bool warned_missing_xyz_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nav_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudDownsampleNode>());
  rclcpp::shutdown();
  return 0;
}
