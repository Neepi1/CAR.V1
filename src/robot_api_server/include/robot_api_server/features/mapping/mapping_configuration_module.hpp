#pragma once

#include <filesystem>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/mapping/mapping_module.hpp"

namespace robot_api_server::features::mapping
{

// Paths owned by maps and projected into mapping without a second ROS
// parameter declaration.
struct MappingConfigurationInputs
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_maps_dir;
};

// Declares every mapping-owned API parameter exactly once, applies the
// established bounds, and emits the complete MappingModuleConfig. It does not
// launch/stop mapping, alter scan ownership, save a map, or call ROS services.
class MappingConfigurationModule
{
public:
  static MappingModuleConfig declare_parameters(
    rclcpp::Node & node,
    const MappingConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::mapping
