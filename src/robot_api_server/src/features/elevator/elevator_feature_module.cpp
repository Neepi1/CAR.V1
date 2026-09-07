#include "robot_api_server/features/elevator/elevator_feature_module.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/docking_feature_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_feature_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/maps_feature_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::elevator {

namespace {

void validate_dependencies(
    const ElevatorFeatureModuleDependencies &dependencies) {
  if (dependencies.maps == nullptr || dependencies.mapping == nullptr ||
      dependencies.floor_switch == nullptr ||
      dependencies.runtime_mode == nullptr ||
      dependencies.delayed_side_effect_unknown_count == nullptr ||
      !dependencies.localization || !dependencies.navigation ||
      !dependencies.docking || !dependencies.teleop ||
      dependencies.map_frame.empty() ||
      dependencies.robot_pose_freshness_sec <= 0.0) {
    throw std::invalid_argument(
        "ElevatorFeatureModule requires every runtime dependency");
  }
}

} // namespace

class ElevatorFeatureModule::Impl {
public:
  Impl(rclcpp::Node &node, ElevatorModuleConfig configuration,
       ElevatorFeatureModuleDependencies dependencies)
      : dependencies_(std::move(dependencies)) {
    validate_dependencies(dependencies_);

    ElevatorModulePorts ports;
    ports.floor_runtime_interlock_response =
        [this](const std::string &operation) {
          return dependencies_.floor_switch->interlock_response(operation);
        };
    ports.delayed_side_effect_unknown_probe = [this]() {
      return dependencies_.delayed_side_effect_unknown_count->load(
          std::memory_order_acquire);
    };
    ports.current_map_pose_probe = [this]() -> std::optional<ElevatorMapPose> {
      auto *localization = dependencies_.localization();
      if (localization == nullptr) {
        return std::nullopt;
      }
      const auto pose = localization->module().current_robot_pose_snapshot();
      if (!pose.available || pose.frame_id != dependencies_.map_frame ||
          !std::isfinite(pose.age_sec) || pose.age_sec < 0.0 ||
          pose.age_sec > dependencies_.robot_pose_freshness_sec ||
          !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
          !std::isfinite(pose.yaw)) {
        return std::nullopt;
      }
      return ElevatorMapPose{pose.x, pose.y, pose.yaw, pose.stamp_sec,
                             pose.age_sec};
    };
    ports.runtime_idle_probe = [this]() {
      auto *navigation = dependencies_.navigation();
      auto *teleop = dependencies_.teleop();
      auto *docking = dependencies_.docking();
      bool api_goal_idle =
          navigation != nullptr && !navigation->mission_runtime().running();
      api_goal_idle = api_goal_idle &&
                      dependencies_.delayed_side_effect_unknown_count->load(
                          std::memory_order_acquire) == 0U;
      api_goal_idle =
          api_goal_idle && navigation != nullptr &&
          !navigation->action_runtime().tracked_goal_snapshot().available;
      api_goal_idle =
          api_goal_idle && navigation != nullptr &&
          !navigation->action_runtime().status_snapshot().goal_active;
      api_goal_idle = api_goal_idle && teleop != nullptr && teleop->idle();
      const auto runtime = dependencies_.runtime_mode->snapshot();
      const bool docking_job_idle =
          docking == nullptr || !docking->job_running();
      return std::array<bool, 3>{
          api_goal_idle,
          !runtime.mapping_active &&
              !dependencies_.mapping->snapshot(false).start_job_running &&
              dependencies_.runtime_mode->transition_owner() != "mapping_start",
          !runtime.docking_active && docking_job_idle,
      };
    };
    module_ = std::make_unique<ElevatorModule>(
        node, dependencies_.maps->module().catalog(),
        dependencies_.maps->module().mutation_mutex(),
        dependencies_.maps->cross_asset_commit_mutex(),
        std::move(configuration), std::move(ports));
  }

  ElevatorFeatureModuleDependencies dependencies_;
  std::unique_ptr<ElevatorModule> module_;
};

ElevatorFeatureModule::ElevatorFeatureModule(
    rclcpp::Node &node, ElevatorModuleConfig configuration,
    ElevatorFeatureModuleDependencies dependencies)
    : impl_(std::make_unique<Impl>(node, std::move(configuration),
                                   std::move(dependencies))) {}

ElevatorFeatureModule::~ElevatorFeatureModule() = default;

ElevatorModule &ElevatorFeatureModule::module() { return *impl_->module_; }

} // namespace robot_api_server::features::elevator
