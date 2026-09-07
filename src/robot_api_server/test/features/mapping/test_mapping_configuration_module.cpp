#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/mapping/mapping_configuration_module.hpp"

namespace robot_api_server::features::mapping
{
namespace
{

class MappingConfigurationModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

MappingConfigurationInputs test_inputs()
{
  MappingConfigurationInputs inputs;
  inputs.maps_root = "/test/maps";
  inputs.runtime_maps_dir = "/test/runtime_maps";
  return inputs;
}

TEST_F(MappingConfigurationModuleTest, PreservesProductionDefaultsAndPathProjections)
{
  auto node = std::make_shared<rclcpp::Node>("mapping_configuration_defaults_test");
  const auto config = MappingConfigurationModule::declare_parameters(
    *node, test_inputs());

  EXPECT_EQ(config.maps_root, "/test/maps");
  EXPECT_EQ(config.runtime_maps_dir, "/test/runtime_maps");
  EXPECT_EQ(
    config.process.start_command,
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/"
    "run_projected_map.sh");
  EXPECT_EQ(config.process.log_file, "/tmp/njrh_mapping2d_slam_toolbox.log");
  EXPECT_EQ(config.process.lidar_rps_xps_state_dir, "/tmp/njrh_slam2d_lidar_rps_xps");
  EXPECT_DOUBLE_EQ(config.process.graceful_stop_timeout_sec, 30.0);
  EXPECT_DOUBLE_EQ(config.scan_owner_restore_timeout_sec, 30.0);
  EXPECT_EQ(config.scan_owner_topic, "/scan");
  EXPECT_EQ(config.navigation_scan_owner_node, "pointcloud_accel_axis_node");
  EXPECT_EQ(
    config.resident_scan_control_service,
    "/pointcloud_accel_axis_node/set_scan_output_enabled");
  EXPECT_EQ(config.live_map_topic, "/map");
  EXPECT_DOUBLE_EQ(config.live_map_max_age_sec, 3.0);
}

TEST_F(MappingConfigurationModuleTest, AppliesEstablishedTimeoutAndFreshnessBounds)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("mapping_graceful_stop_timeout_sec", 1.0);
  options.append_parameter_override("mapping_scan_owner_restore_timeout_sec", 100.0);
  options.append_parameter_override("mapping_2d_live_map_max_age_sec", 0.01);
  auto node = std::make_shared<rclcpp::Node>(
    "mapping_configuration_bounds_test", options);

  const auto config = MappingConfigurationModule::declare_parameters(
    *node, test_inputs());

  EXPECT_DOUBLE_EQ(config.process.graceful_stop_timeout_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.scan_owner_restore_timeout_sec, 60.0);
  EXPECT_DOUBLE_EQ(config.live_map_max_age_sec, 0.1);
}

}  // namespace
}  // namespace robot_api_server::features::mapping
