#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_hesai_jt128/pointcloud_accel_core.hpp"

namespace
{

constexpr char kUnavailableReason[] =
  "repo_owned_hesai_driver_overlay_not_available";

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hesai_accel_driver_node");

  node->declare_parameter<std::string>(
    "driver_integrated_unavailable_reason", kUnavailableReason);
  node->declare_parameter<bool>("driver_integrated_available", false);
  node->declare_parameter<bool>("publish_vendor_raw_debug", false);
  node->declare_parameter<bool>("publish_vendor_imu_raw_debug", false);
  node->declare_parameter<std::string>("accel_config_path", "");
  node->declare_parameter<std::string>("hesai_driver_config_path", "");

  robot_hesai_jt128::PointCloudAccelCoreOptions core_options;
  core_options.accel_ingress_profile = "driver_integrated";
  core_options.input_path = "driver_callback_unavailable";
  core_options.vendor_raw_ros_hop_required = false;
  core_options.vendor_raw_debug_publish_enabled =
    node->get_parameter("publish_vendor_raw_debug").as_bool();
  core_options.driver_integrated_process = true;
  core_options.driver_integrated_unavailable_reason =
    node->get_parameter("driver_integrated_unavailable_reason").as_string();

  (void)core_options;

  RCLCPP_ERROR(
    node->get_logger(),
    "DRIVER_INTEGRATED_UNAVAILABLE_REASON=%s; use "
    "NJRH_POINTCLOUD_INGRESS_PROFILE=separate_process until the Hesai driver "
    "source is available as a repo-owned overlay",
    node->get_parameter("driver_integrated_unavailable_reason").as_string().c_str());

  rclcpp::shutdown();
  return 3;
}
