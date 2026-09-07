#pragma once

#include <filesystem>
#include <string>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/navigation/mission/navigation_goal_execution_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_executor.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"

namespace robot_api_server::features::navigation::configuration
{

// Values owned by neighboring domains or shared runtime composition. This
// module consumes them without redeclaring their ROS parameters.
struct NavigationConfigurationInputs
{
  double service_timeout_sec{8.0};
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string navigate_to_pose_status_topic{"/navigate_to_pose/_action/status"};
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  double robot_pose_freshness_sec{0.5};

  // Ordinary terminal lateral correction deliberately shares Ranger mode
  // transition behavior with predock alignment.
  double lateral_divergence_epsilon_m{0.015};
  int lateral_divergence_count{2};
  std::string lateral_forced_mode{"side_slip"};
  std::string lateral_release_mode{"auto"};
};

// Complete validated configuration graph for ordinary navigation. Callers
// receive the existing submodule configs rather than copying scalar values.
struct NavigationConfiguration
{
  NavigationModuleConfig module;
  NavigationTerminalRuntimeConfig terminal_runtime;
  NavigationGoalExecutionConfig goal_execution;
  NavigationGoalExecutorConfig goal_executor;

  // Explicit projection into LocalizationModule for post-Nav2 verification.
  std::string amcl_nomotion_update_service{"/request_nomotion_update"};
};

// Declares every ordinary-navigation-owned ROS parameter exactly once,
// applies the legacy normalization/clamp graph, and emits ready-to-consume
// configs for the navigation module family.
class NavigationConfigurationModule
{
public:
  static NavigationConfiguration declare_parameters(
    rclcpp::Node & node,
    const NavigationConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::navigation::configuration
