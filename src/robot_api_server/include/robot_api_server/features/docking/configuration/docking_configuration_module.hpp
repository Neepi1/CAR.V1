#pragma once

#include <string>

#include "rclcpp/node.hpp"

#include "robot_api_server/features/docking/configuration/docking_predock_pose_resolver.hpp"
#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_correction_pause_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_http_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_execution_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_executor.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_status_module.hpp"
#include "robot_api_server/features/docking/lifecycle/pre_navigation_undock_module.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_alignment_policy.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_control_module.hpp"

namespace robot_api_server::features::docking::configuration
{

// Values owned by neighboring domains that constrain docking configuration.
// Docking never redeclares these ROS parameters.
struct DockingConfigurationInputs
{
  double service_timeout_sec{8.0};
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string map_frame{"map"};
  double robot_pose_freshness_sec{0.5};
  double charging_current_min_a{0.10};
  double full_soc_threshold_pct{99.0};
};

// The complete validated configuration graph consumed by the docking feature.
// Callers receive existing submodule configs, not a second set of scalar knobs.
struct DockingConfiguration
{
  DockingRuntimeConfig runtime;
  DockContactInterlockConfig contact_interlock;
  DockingCorrectionPauseConfig correction_pause;
  DockingPredockPoseResolverConfig predock_pose_resolver;
  predock_alignment::PredockAlignmentConfig alignment_policy;
  predock_alignment::PredockControlConfig predock_control;
  PreNavigationUndockConfig pre_navigation_undock;
  DockingJobExecutorConfig job_executor;
  DockingJobExecutionModuleConfig job_execution;
  DockingHttpConfig http;
  DockingStatusConfig status;

  // Explicit neighboring-domain projections retained by their current owners.
  double localization_default_relocalization_wait_sec{8.0};
  double localization_recent_result_max_age_sec{5.0};
  bool framework_state_machine_enabled{true};
};

// Declares every docking-owned ROS parameter once, applies the legacy clamps
// and dependencies, and emits ready-to-consume submodule configurations.
class DockingConfigurationModule
{
public:
  static DockingConfiguration declare_parameters(
    rclcpp::Node & node,
    const DockingConfigurationInputs & inputs);
};

}  // namespace robot_api_server::features::docking::configuration
