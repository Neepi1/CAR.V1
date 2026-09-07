#include "robot_api_server/features/floor_switch/floor_switch_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/docking/docking_feature_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_configuration_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"

namespace robot_api_server::features::floor_switch
{

namespace
{

void validate_dependencies(
  const FloorSwitchFeatureModuleDependencies & dependencies)
{
  if (dependencies.maps == nullptr || dependencies.runtime_mode == nullptr ||
    dependencies.delayed_side_effect_unknown_count == nullptr ||
    !dependencies.mapping || !dependencies.navigation ||
    !dependencies.docking || !dependencies.elevator)
  {
    throw std::invalid_argument(
            "FloorSwitchFeatureModule requires every runtime dependency provider");
  }
}

}  // namespace

class FloorSwitchFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    FloorSwitchConfiguration configuration,
    std::mutex & cross_asset_commit_mutex,
    FloorSwitchFeatureModuleDependencies dependencies)
  : floor_status_topic_(std::move(configuration.floor_status_topic)),
    live_action_(configuration.module.live_action),
    live_timeout_sec_(configuration.module.live_timeout_sec),
    dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    FloorSwitchModulePorts ports;
    ports.map_asset_integrity_degraded = [this]() {
        return dependencies_.maps->integrity_degraded();
      };
    ports.runtime_snapshot = [this]() {
        auto * mapping = dependencies_.mapping();
        auto * navigation = dependencies_.navigation();
        auto * docking = dependencies_.docking();
        if (mapping != nullptr) {
          (void)mapping->snapshot(true);
        }
        if (dependencies_.runtime_mode->transition_owner() != "mapping_start" &&
          navigation != nullptr)
        {
          navigation->module().refresh_runtime_state(false);
        }
        const auto runtime = dependencies_.runtime_mode->snapshot();
        FloorSwitchRuntimeSnapshot snapshot;
        snapshot.navigation_active = runtime.navigation_active;
        snapshot.mapping_active = runtime.mapping_active;
        snapshot.docking_active = runtime.docking_active;
        snapshot.mode_transition_owner = dependencies_.runtime_mode->transition_owner();
        snapshot.navigation_goal_running =
          navigation != nullptr && navigation->mission_runtime().running();
        snapshot.mapping_start_job_running =
          mapping != nullptr && mapping->snapshot(false).start_job_running;
        snapshot.docking_job_running =
          docking != nullptr && docking->job_running();
        snapshot.nav2_goal_active =
          navigation != nullptr &&
          navigation->action_runtime().status_snapshot().goal_active;
        snapshot.navigation_process_running =
          navigation != nullptr && navigation->module().process_running();
        return snapshot;
      };
    ports.validate_map_manifest_assets = [this](
      const MapManifest & manifest, std::string & error) {
        return dependencies_.maps->validate_manifest_assets(manifest, error);
      };
    ports.activate_map_manifest = [this](
      MapManifest manifest, MapAssetCommitTransaction & transaction) {
        dependencies_.maps->activate_manifest(std::move(manifest), transaction);
      };
    ports.acquire_motion_admission = [this](
      const ElevatorMotionAdmissionFence::Epoch expected_epoch) {
        auto * elevator = dependencies_.elevator();
        if (elevator == nullptr) {
          throw std::logic_error("elevator module is not initialized");
        }
        return elevator->acquire_motion_admission(expected_epoch);
      };
    ports.mark_delayed_side_effect_unknown = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_add(
          1U, std::memory_order_acq_rel);
      };
    ports.resolve_delayed_side_effect_unknown = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_sub(
          1U, std::memory_order_acq_rel);
      };
    module_ = std::make_unique<FloorSwitchModule>(
      node,
      std::move(callback_group),
      dependencies_.maps->catalog(),
      dependencies_.maps->mutation_mutex(),
      cross_asset_commit_mutex,
      std::move(configuration.module),
      std::move(ports));
  }

  std::string floor_status_topic_;
  std::string live_action_;
  double live_timeout_sec_{0.0};
  FloorSwitchFeatureModuleDependencies dependencies_;
  std::unique_ptr<FloorSwitchModule> module_;
};

FloorSwitchFeatureModule::FloorSwitchFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  FloorSwitchConfiguration configuration,
  std::mutex & cross_asset_commit_mutex,
  FloorSwitchFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      cross_asset_commit_mutex,
      std::move(dependencies)))
{
}

FloorSwitchFeatureModule::~FloorSwitchFeatureModule() = default;

FloorSwitchModule & FloorSwitchFeatureModule::module()
{
  return *impl_->module_;
}

const std::string & FloorSwitchFeatureModule::floor_status_topic() const
{
  return impl_->floor_status_topic_;
}

const std::string & FloorSwitchFeatureModule::live_action() const
{
  return impl_->live_action_;
}

double FloorSwitchFeatureModule::live_timeout_sec() const
{
  return impl_->live_timeout_sec_;
}

void FloorSwitchFeatureModule::shutdown()
{
  impl_->module_->shutdown();
}

}  // namespace robot_api_server::features::floor_switch
