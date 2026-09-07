#include "robot_api_server/features/maps/maps_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_feature_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/maps_configuration_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::maps
{

namespace
{

void validate_dependencies(const MapsFeatureModuleDependencies & dependencies)
{
  if (dependencies.runtime_mode == nullptr ||
    dependencies.delayed_side_effect_unknown_count == nullptr ||
    !dependencies.mapping || !dependencies.floor_switch ||
    !dependencies.navigation || !dependencies.elevator ||
    !dependencies.localization)
  {
    throw std::invalid_argument(
            "MapsFeatureModule requires every runtime dependency provider");
  }
}

}  // namespace

class MapsFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapsConfiguration configuration,
    MapsFeatureModuleDependencies dependencies)
  : runtime_paths_(std::move(configuration.runtime_paths)),
    dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    runtime_state_store_ = std::make_unique<MapRuntimeStateStore>(
      runtime_paths_.runtime_map_context_file,
      runtime_paths_.last_navigation_map_file);

    MapsModulePorts ports;
    ports.runtime_snapshot = [this](const bool refresh_runtime) {
        auto * mapping = dependencies_.mapping();
        auto * navigation = dependencies_.navigation();
        if (refresh_runtime && mapping != nullptr) {
          (void)mapping->snapshot(true);
        }
        if (refresh_runtime &&
          dependencies_.runtime_mode->transition_owner() != "mapping_start" &&
          navigation != nullptr)
        {
          navigation->module().refresh_runtime_state(false);
        }
        const auto runtime = dependencies_.runtime_mode->snapshot();
        MapsRuntimeSnapshot snapshot;
        snapshot.navigation_active = runtime.navigation_active;
        snapshot.mapping_active = runtime.mapping_active;
        snapshot.docking_active = runtime.docking_active;
        snapshot.mapping_start_job_running =
          mapping != nullptr && mapping->snapshot(false).start_job_running;
        snapshot.navigation_goal_running =
          navigation != nullptr && navigation->mission_runtime().running();
        snapshot.nav2_goal_active =
          navigation != nullptr &&
          navigation->action_runtime().status_snapshot().goal_active;
        return snapshot;
      };
    ports.floor_runtime_interlock_response = [this](const std::string & operation) {
        auto * floor_switch = dependencies_.floor_switch();
        return floor_switch != nullptr ?
               floor_switch->interlock_response(operation) : std::nullopt;
      };
    ports.wait_for_current_robot_pose = [this]() {
        auto * localization = dependencies_.localization();
        if (localization == nullptr) {
          return RobotPoseSnapshot{};
        }
        std::string error;
        return localization->module().wait_for_current_robot_pose(true, error);
      };
    ports.wait_for_terminal_actual_stop = [this](
      const std::string & context, std::string & detail) {
        auto * navigation = dependencies_.navigation();
        if (navigation == nullptr) {
          detail = "navigation feature module is not initialized";
          return false;
        }
        return navigation->terminal_runtime().wait_for_actual_stop(context, detail);
      };
    ports.query_elevator_reference = [this](const MapManifest & manifest) {
        auto * elevator = dependencies_.elevator();
        if (elevator == nullptr) {
          return HttpResponse{
          503,
          "application/json",
          error_json("elevator module is not initialized")};
        }
        return elevator->query_map_reference(manifest);
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
    module_ = std::make_unique<MapsModule>(
      node,
      std::move(callback_group),
      cross_asset_commit_mutex_,
      std::move(configuration.module),
      std::move(ports));
  }

  MapsRuntimePaths runtime_paths_;
  MapsFeatureModuleDependencies dependencies_;
  std::mutex cross_asset_commit_mutex_;
  std::unique_ptr<MapRuntimeStateStore> runtime_state_store_;
  std::unique_ptr<MapsModule> module_;
};

MapsFeatureModule::MapsFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  MapsConfiguration configuration,
  MapsFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      std::move(dependencies)))
{
}

MapsFeatureModule::~MapsFeatureModule() = default;

MapsModule & MapsFeatureModule::module() {return *impl_->module_;}

MapRuntimeStateStore & MapsFeatureModule::runtime_state_store()
{
  return *impl_->runtime_state_store_;
}

std::mutex & MapsFeatureModule::cross_asset_commit_mutex()
{
  return impl_->cross_asset_commit_mutex_;
}

const MapsRuntimePaths & MapsFeatureModule::runtime_paths() const
{
  return impl_->runtime_paths_;
}

}  // namespace robot_api_server::features::maps
