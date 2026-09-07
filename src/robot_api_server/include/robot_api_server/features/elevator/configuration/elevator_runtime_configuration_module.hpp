#pragma once

#include <filesystem>
#include <string>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/elevator/elevator_module.hpp"

namespace robot_api_server::features::elevator::configuration
{

// Values owned by neighboring feature modules or shared composition.
struct ElevatorRuntimeConfigurationInputs
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string floor_switch_action{"/floor_manager/floor_switch"};
  double floor_switch_timeout_sec{120.0};
  std::string motion_allowed_topic{"/safety/motion_allowed"};
  std::string safety_status_topic{"/safety/status"};
  std::string navigation_status_topic{"/navigate_to_pose/_action/status"};
};

// Declares all elevator-runtime-owned API parameters and returns the complete
// validated ElevatorModule construction config. It does not call the arm
// black box, start an elevator transaction, publish motion, or add a gate.
class ElevatorRuntimeConfigurationModule
{
public:
  static ElevatorModuleConfig declare_parameters(
    rclcpp::Node & node,
    const ElevatorRuntimeConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::elevator::configuration
