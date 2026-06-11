#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "robot_hesai_jt128/pointcloud_accel_core.hpp"

namespace
{

std::string env_or_default(const char * name, const std::string & fallback)
{
  const char * value = std::getenv(name);
  return value && *value ? std::string(value) : fallback;
}

}  // namespace

class PointCloudAccelAxisNode : public rclcpp::Node
{
public:
  explicit PointCloudAccelAxisNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pointcloud_accel_axis_node", options)
  {
    robot_hesai_jt128::PointCloudAccelCoreOptions core_options;
    core_options.accel_ingress_profile = env_or_default(
      "NJRH_POINTCLOUD_INGRESS_PROFILE", "separate_process");
    core_options.input_path = "pointcloud2_topic";
    core_options.vendor_raw_ros_hop_required = true;
    core_options.vendor_raw_debug_publish_enabled = false;
    core_options.driver_integrated_process = false;
    core_options.driver_integrated_unavailable_reason = "not_selected";

    core_ = std::make_unique<robot_hesai_jt128::PointCloudAccelCore>(*this, core_options);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      core_->input_topic(),
      core_->input_qos(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        core_->process_pointcloud2(std::move(msg));
      });
  }

private:
  std::unique_ptr<robot_hesai_jt128::PointCloudAccelCore> core_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudAccelAxisNode>());
  rclcpp::shutdown();
  return 0;
}
