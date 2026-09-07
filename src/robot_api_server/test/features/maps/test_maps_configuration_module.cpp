#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/maps/maps_configuration_module.hpp"

namespace robot_api_server::features::maps
{
namespace
{

class MapsConfigurationModuleTest : public ::testing::Test
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

TEST_F(MapsConfigurationModuleTest, PreservesProductionDefaultsAndSharedPaths)
{
  auto node = std::make_shared<rclcpp::Node>("maps_configuration_defaults_test");
  const auto config = MapsConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.runtime_paths.maps_root, "/workspaces/njrh-v3/workspace1/maps_release");
  EXPECT_EQ(
    config.runtime_paths.runtime_maps_dir,
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps");
  EXPECT_EQ(
    config.runtime_paths.runtime_map_context_file,
    "/tmp/njrh_runtime_map_context.json");
  EXPECT_EQ(
    config.runtime_paths.last_navigation_map_file,
    "/workspaces/njrh-v3/workspace1/maps_release/last_navigation_map.json");
  EXPECT_EQ(config.module.maps_root, config.runtime_paths.maps_root);
  EXPECT_EQ(config.module.runtime_maps_dir, config.runtime_paths.runtime_maps_dir);
  EXPECT_EQ(
    config.module.runtime_map_context_file,
    config.runtime_paths.runtime_map_context_file);
  EXPECT_EQ(config.module.map_frame, "map");
  EXPECT_EQ(config.module.base_frame, "base_link");
  EXPECT_EQ(config.module.keepout_mask_load_service, "/keepout_filter_mask_server/load_map");
  EXPECT_EQ(config.module.keepout_mask_state_service, "/keepout_filter_mask_server/get_state");
  EXPECT_EQ(
    config.module.keepout_filter_info_state_service,
    "/keepout_costmap_filter_info_server/get_state");
  EXPECT_EQ(
    config.module.keepout_mask_get_parameters_service,
    "/keepout_filter_mask_server/get_parameters");
  EXPECT_EQ(
    config.module.keepout_runtime_stage_root,
    "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/filters/runtime_nav2");
  EXPECT_EQ(config.module.keepout_mask_topic, "/keepout_filter_mask");
  EXPECT_EQ(config.module.keepout_filter_info_topic, "/costmap_filter_info/keepout");
  EXPECT_EQ(
    config.module.global_costmap_get_parameters_service,
    "/global_costmap/global_costmap/get_parameters");
  EXPECT_EQ(
    config.module.global_costmap_state_service,
    "/global_costmap/global_costmap/get_state");
  EXPECT_EQ(
    config.module.global_costmap_clear_service,
    "/global_costmap/clear_entirely_global_costmap");
  EXPECT_EQ(config.module.global_costmap_topic, "/global_costmap/costmap");
  EXPECT_DOUBLE_EQ(config.module.keepout_runtime_apply_timeout_sec, 5.0);
}

TEST_F(MapsConfigurationModuleTest, AppliesEstablishedKeepoutTimeoutClamp)
{
  rclcpp::NodeOptions low_options;
  low_options.append_parameter_override("keepout_runtime_apply_timeout_sec", 0.01);
  auto low_node = std::make_shared<rclcpp::Node>("maps_configuration_low_clamp_test", low_options);
  const auto low = MapsConfigurationModule::declare_parameters(*low_node);
  EXPECT_DOUBLE_EQ(low.module.keepout_runtime_apply_timeout_sec, 0.5);

  rclcpp::NodeOptions high_options;
  high_options.append_parameter_override("keepout_runtime_apply_timeout_sec", 99.0);
  auto high_node = std::make_shared<rclcpp::Node>(
    "maps_configuration_high_clamp_test", high_options);
  const auto high = MapsConfigurationModule::declare_parameters(*high_node);
  EXPECT_DOUBLE_EQ(high.module.keepout_runtime_apply_timeout_sec, 15.0);
}

}  // namespace
}  // namespace robot_api_server::features::maps
