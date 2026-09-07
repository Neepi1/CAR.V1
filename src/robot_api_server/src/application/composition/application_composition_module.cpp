#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/application/composition/application_composition_module.hpp"
#include "robot_api_server/application/routing/application_router_module.hpp"
#include "robot_api_server/application/runtime_configuration/runtime_configuration_module.hpp"
#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/application/subscriptions/subscription_configuration_module.hpp"
#include "robot_api_server/application/subscriptions/subscription_feature_module.hpp"
#include "robot_api_server/features/docking/configuration/docking_configuration_module.hpp"
#include "robot_api_server/features/docking/docking_feature_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_runtime_module.hpp"
#include "robot_api_server/features/elevator/configuration/elevator_runtime_configuration_module.hpp"
#include "robot_api_server/features/elevator/elevator_feature_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_configuration_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_feature_module.hpp"
#include "robot_api_server/features/localization/localization_configuration_module.hpp"
#include "robot_api_server/features/localization/localization_feature_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_configuration_module.hpp"
#include "robot_api_server/features/mapping/mapping_feature_module.hpp"
#include "robot_api_server/features/maps/maps_configuration_module.hpp"
#include "robot_api_server/features/maps/maps_feature_module.hpp"
#include "robot_api_server/features/navigation/configuration/navigation_configuration_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/power/power_configuration_module.hpp"
#include "robot_api_server/features/power/power_feature_module.hpp"
#include "robot_api_server/features/safety/safety_configuration_module.hpp"
#include "robot_api_server/features/safety/safety_feature_module.hpp"
#include "robot_api_server/features/system_status/system_status_module.hpp"
#include "robot_api_server/features/system_status/system_status_wiring.hpp"
#include "robot_api_server/features/teleop/teleop_configuration_module.hpp"
#include "robot_api_server/features/teleop/teleop_feature_module.hpp"
#include "robot_api_server/infrastructure/http/api_gateway_configuration_module.hpp"
#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace {

using robot_api_server::HttpRequest;
using robot_api_server::application::routing::ApplicationRouterDependencies;
using robot_api_server::application::routing::ApplicationRouterModule;
using robot_api_server::application::routing::make_application_router_ports;
using robot_api_server::application::runtime_configuration::
    RuntimeConfigurationModule;
using robot_api_server::application::runtime_mode::RuntimeModeCoordinator;
using robot_api_server::application::subscriptions::
    SubscriptionConfigurationModule;
using robot_api_server::application::subscriptions::SubscriptionFeatureModule;
using robot_api_server::application::subscriptions::
    SubscriptionFeatureModuleDependencies;
using robot_api_server::features::docking::DockingFeatureLateDependencies;
using robot_api_server::features::docking::DockingFeatureModule;
using robot_api_server::features::docking::DockingFeatureModuleDependencies;
using robot_api_server::features::docking::configuration::
    DockingConfigurationInputs;
using robot_api_server::features::docking::configuration::
    DockingConfigurationModule;
using robot_api_server::features::elevator::ElevatorFeatureModule;
using robot_api_server::features::elevator::ElevatorFeatureModuleDependencies;
using robot_api_server::features::elevator::configuration::
    ElevatorRuntimeConfigurationInputs;
using robot_api_server::features::elevator::configuration::
    ElevatorRuntimeConfigurationModule;
using robot_api_server::features::floor_switch::FloorSwitchConfigurationInputs;
using robot_api_server::features::floor_switch::FloorSwitchConfigurationModule;
using robot_api_server::features::floor_switch::FloorSwitchFeatureModule;
using robot_api_server::features::floor_switch::
    FloorSwitchFeatureModuleDependencies;
using robot_api_server::features::localization::BridgeStatusSnapshot;
using robot_api_server::features::localization::LocalizationConfigurationInputs;
using robot_api_server::features::localization::LocalizationConfigurationModule;
using robot_api_server::features::localization::
    LocalizationFeatureLateDependencies;
using robot_api_server::features::localization::LocalizationFeatureModule;
using robot_api_server::features::localization::
    LocalizationFeatureModuleDependencies;
using robot_api_server::features::mapping::MappingConfigurationInputs;
using robot_api_server::features::mapping::MappingConfigurationModule;
using robot_api_server::features::mapping::MappingFeatureModule;
using robot_api_server::features::mapping::MappingFeatureModuleDependencies;
using robot_api_server::features::maps::MapsConfigurationModule;
using robot_api_server::features::maps::MapsFeatureModule;
using robot_api_server::features::maps::MapsFeatureModuleDependencies;
using robot_api_server::features::navigation::NavigationFeatureModule;
using robot_api_server::features::navigation::
    NavigationFeatureModuleDependencies;
using robot_api_server::features::navigation::configuration::
    NavigationConfigurationInputs;
using robot_api_server::features::navigation::configuration::
    NavigationConfigurationModule;
using robot_api_server::features::power::PowerConfigurationModule;
using robot_api_server::features::power::PowerFeatureModule;
using robot_api_server::features::power::PowerFeatureModuleDependencies;
using robot_api_server::features::safety::SafetyConfigurationModule;
using robot_api_server::features::safety::SafetyFeatureModule;
using robot_api_server::features::safety::SafetyFeatureModuleDependencies;
using robot_api_server::features::system_status::
    make_system_status_module_ports;
using robot_api_server::features::system_status::SystemStatusModule;
using robot_api_server::features::system_status::SystemStatusModuleConfig;
using robot_api_server::features::system_status::SystemStatusWiringDependencies;
using robot_api_server::features::teleop::TeleopConfigurationInputs;
using robot_api_server::features::teleop::TeleopConfigurationModule;
using robot_api_server::features::teleop::TeleopFeatureModule;
using robot_api_server::features::teleop::TeleopFeatureModuleDependencies;
using robot_api_server::infrastructure::http::ApiGatewayConfigurationModule;
using robot_api_server::infrastructure::http::ApiGatewayModule;
using robot_api_server::infrastructure::http::ApiGatewayModulePorts;

} // namespace

namespace robot_api_server::application::composition {

class ApplicationCompositionModule::Impl {

public:
  explicit Impl(rclcpp::Node &node) : node_(node) {
    configure_runtime_permissions();
    auto api_gateway_config =
        ApiGatewayConfigurationModule::declare_parameters(node_);
    auto maps_configuration =
        MapsConfigurationModule::declare_parameters(node_);
    const auto maps_runtime_paths = maps_configuration.runtime_paths;
    auto safety_configuration =
        SafetyConfigurationModule::declare_parameters(node_);
    auto power_configuration =
        PowerConfigurationModule::declare_parameters(node_);
    auto &power_module_config = power_configuration.module;
    const auto runtime_configuration =
        RuntimeConfigurationModule::declare_parameters(node_);
    MappingConfigurationInputs mapping_configuration_inputs;
    mapping_configuration_inputs.maps_root = maps_runtime_paths.maps_root;
    mapping_configuration_inputs.runtime_maps_dir =
        maps_runtime_paths.runtime_maps_dir;
    auto mapping_module_config = MappingConfigurationModule::declare_parameters(
        node_, mapping_configuration_inputs);
    auto subscription_module_config =
        SubscriptionConfigurationModule::declare_parameters(node_);
    LocalizationConfigurationInputs localization_configuration_inputs;
    localization_configuration_inputs.service_timeout_sec =
        runtime_configuration.service_timeout_sec;
    auto localization_configuration =
        LocalizationConfigurationModule::declare_parameters(
            node_, localization_configuration_inputs);
    FloorSwitchConfigurationInputs floor_switch_configuration_inputs;
    floor_switch_configuration_inputs.maps_root = maps_runtime_paths.maps_root;
    floor_switch_configuration_inputs.runtime_map_context_file =
        maps_runtime_paths.runtime_map_context_file;
    floor_switch_configuration_inputs.localization_health_topic =
        localization_configuration.floor_health_topic;
    floor_switch_configuration_inputs.service_timeout =
        runtime_configuration.service_timeout();
    auto floor_switch_configuration =
        FloorSwitchConfigurationModule::declare_parameters(
            node_, floor_switch_configuration_inputs);
    maps_configuration.module.map_frame =
        localization_configuration.module.map_frame;
    maps_configuration.module.base_frame =
        localization_configuration.module.base_frame;
    const std::string tf_map_frame =
        localization_configuration.module.map_frame;
    const std::string tf_base_frame =
        localization_configuration.module.base_frame;
    const double tf_pose_max_age_sec =
        localization_configuration.module.tf_pose_max_age_sec;
    const double robot_pose_freshness_sec =
        localization_configuration.module.robot_pose_freshness_sec;
    DockingConfigurationInputs docking_configuration_inputs;
    docking_configuration_inputs.service_timeout_sec =
        runtime_configuration.service_timeout_sec;
    docking_configuration_inputs.navigate_to_pose_action =
        runtime_configuration.navigate_to_pose_action;
    docking_configuration_inputs.map_frame = tf_map_frame;
    docking_configuration_inputs.robot_pose_freshness_sec =
        robot_pose_freshness_sec;
    docking_configuration_inputs.charging_current_min_a =
        power_module_config.charging_current_min_a;
    docking_configuration_inputs.full_soc_threshold_pct =
        power_module_config.full_soc_threshold_pct;
    auto docking_configuration = DockingConfigurationModule::declare_parameters(
        node_, docking_configuration_inputs);
    localization_configuration.module.default_relocalization_wait_sec =
        docking_configuration.localization_default_relocalization_wait_sec;
    localization_configuration.module.recent_result_max_age_sec =
        docking_configuration.localization_recent_result_max_age_sec;
    NavigationConfigurationInputs navigation_configuration_inputs;
    navigation_configuration_inputs.service_timeout_sec =
        runtime_configuration.service_timeout_sec;
    navigation_configuration_inputs.navigate_to_pose_action =
        runtime_configuration.navigate_to_pose_action;
    navigation_configuration_inputs.navigate_to_pose_status_topic =
        runtime_configuration.navigate_to_pose_status_topic;
    navigation_configuration_inputs.maps_root = maps_runtime_paths.maps_root;
    navigation_configuration_inputs.runtime_map_context_file =
        maps_runtime_paths.runtime_map_context_file;
    navigation_configuration_inputs.map_frame = tf_map_frame;
    navigation_configuration_inputs.base_frame = tf_base_frame;
    navigation_configuration_inputs.robot_pose_freshness_sec =
        robot_pose_freshness_sec;
    navigation_configuration_inputs.lateral_divergence_epsilon_m =
        docking_configuration.predock_control
            .lateral_align_divergence_epsilon_m;
    navigation_configuration_inputs.lateral_divergence_count =
        docking_configuration.predock_control.lateral_align_divergence_count;
    navigation_configuration_inputs.lateral_forced_mode =
        docking_configuration.predock_control.lateral_align_forced_mode;
    navigation_configuration_inputs.lateral_release_mode =
        docking_configuration.predock_control.lateral_align_release_mode;
    auto navigation_configuration =
        NavigationConfigurationModule::declare_parameters(
            node_, navigation_configuration_inputs);
    localization_configuration.module.amcl_nomotion_update_service =
        navigation_configuration.amcl_nomotion_update_service;

    SafetyFeatureModuleDependencies safety_dependencies;
    safety_dependencies.floor_switch = [this]() {
      return floor_switch_feature_module_
                 ? &floor_switch_feature_module_->module()
                 : nullptr;
    };
    safety_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    safety_dependencies.teleop = [this]() {
      return teleop_feature_module_ ? &teleop_feature_module_->module()
                                    : nullptr;
    };
    safety_dependencies.navigation = [this]() {
      return navigation_feature_module_.get();
    };
    safety_dependencies.docking = [this]() {
      return docking_feature_module_.get();
    };
    safety_feature_module_ = std::make_unique<SafetyFeatureModule>(
        node_, std::move(safety_configuration.module),
        std::move(safety_dependencies));

    PowerFeatureModuleDependencies power_dependencies;
    power_dependencies.docking = [this]() {
      return docking_feature_module_.get();
    };
    power_dependencies.teleop = [this]() {
      return teleop_feature_module_ ? &teleop_feature_module_->module()
                                    : nullptr;
    };
    power_feature_module_ = std::make_unique<PowerFeatureModule>(
        node_, std::move(power_module_config), std::move(power_dependencies));

    SubscriptionFeatureModuleDependencies subscription_dependencies;
    subscription_dependencies.system_status = [this]() {
      return system_status_module_.get();
    };
    subscription_dependencies.mapping = [this]() {
      return mapping_feature_module_ ? &mapping_feature_module_->module()
                                     : nullptr;
    };
    subscription_dependencies.localization = [this]() {
      return localization_feature_module_.get();
    };
    subscription_dependencies.teleop = [this]() {
      return teleop_feature_module_ ? &teleop_feature_module_->module()
                                    : nullptr;
    };
    subscription_feature_module_ = std::make_unique<SubscriptionFeatureModule>(
        node_, std::move(subscription_module_config),
        std::move(subscription_dependencies));
    TeleopConfigurationInputs teleop_configuration_inputs;
    teleop_configuration_inputs.subscription_max_ttl_ms =
        subscription_feature_module_->max_ttl_ms();
    auto teleop_configuration = TeleopConfigurationModule::declare_parameters(
        node_, teleop_configuration_inputs);
    auto &teleop_module_config = teleop_configuration.module;

    callback_group_ =
        node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    MapsFeatureModuleDependencies maps_dependencies;
    maps_dependencies.runtime_mode = &runtime_mode_;
    maps_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    maps_dependencies.mapping = [this]() {
      return mapping_feature_module_ ? &mapping_feature_module_->module()
                                     : nullptr;
    };
    maps_dependencies.floor_switch = [this]() {
      return floor_switch_feature_module_
                 ? &floor_switch_feature_module_->module()
                 : nullptr;
    };
    maps_dependencies.navigation = [this]() {
      return navigation_feature_module_.get();
    };
    maps_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    maps_dependencies.localization = [this]() {
      return localization_feature_module_.get();
    };
    maps_feature_module_ = std::make_unique<MapsFeatureModule>(
        node_, callback_group_, std::move(maps_configuration),
        std::move(maps_dependencies));

    FloorSwitchFeatureModuleDependencies floor_switch_dependencies;
    floor_switch_dependencies.maps = &maps_feature_module_->module();
    floor_switch_dependencies.runtime_mode = &runtime_mode_;
    floor_switch_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    floor_switch_dependencies.mapping = [this]() {
      return mapping_feature_module_ ? &mapping_feature_module_->module()
                                     : nullptr;
    };
    floor_switch_dependencies.navigation = [this]() {
      return navigation_feature_module_.get();
    };
    floor_switch_dependencies.docking = [this]() {
      return docking_feature_module_.get();
    };
    floor_switch_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    floor_switch_feature_module_ = std::make_unique<FloorSwitchFeatureModule>(
        node_, callback_group_, std::move(floor_switch_configuration),
        maps_feature_module_->cross_asset_commit_mutex(),
        std::move(floor_switch_dependencies));

    MappingFeatureModuleDependencies mapping_dependencies;
    mapping_dependencies.maps = &maps_feature_module_->module();
    mapping_dependencies.map_runtime_state_store =
        &maps_feature_module_->runtime_state_store();
    mapping_dependencies.floor_switch = &floor_switch_feature_module_->module();
    mapping_dependencies.runtime_mode = &runtime_mode_;
    mapping_dependencies.navigation = [this]() {
      return navigation_feature_module_.get();
    };
    mapping_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    mapping_dependencies.teleop = [this]() {
      return teleop_feature_module_ ? &teleop_feature_module_->module()
                                    : nullptr;
    };
    mapping_feature_module_ = std::make_unique<MappingFeatureModule>(
        node_, callback_group_, std::move(mapping_module_config),
        std::move(mapping_dependencies));

    TeleopFeatureModuleDependencies teleop_dependencies;
    teleop_dependencies.api_gateway = [this]() {
      return api_gateway_module_.get();
    };
    teleop_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    teleop_dependencies.power = &power_feature_module_->module();
    teleop_dependencies.mapping = &mapping_feature_module_->module();
    teleop_dependencies.localization = [this]() {
      return localization_feature_module_.get();
    };
    teleop_dependencies.subscriptions = &subscription_feature_module_->module();
    teleop_dependencies.tf_pose_max_age_sec = tf_pose_max_age_sec;
    teleop_feature_module_ = std::make_unique<TeleopFeatureModule>(
        node_, std::move(teleop_module_config), std::move(teleop_dependencies));

    LocalizationFeatureModuleDependencies localization_dependencies;
    localization_dependencies.floor_switch =
        &floor_switch_feature_module_->module();
    localization_dependencies.teleop = &teleop_feature_module_->module();
    localization_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    localization_dependencies.elevator = [this]() {
      return elevator_feature_module_ ? &elevator_feature_module_->module()
                                      : nullptr;
    };
    localization_feature_module_ = std::make_unique<LocalizationFeatureModule>(
        node_, callback_group_, std::move(localization_configuration),
        std::move(localization_dependencies));
    DockingFeatureModuleDependencies docking_dependencies;
    docking_dependencies.maps = &maps_feature_module_->module();
    docking_dependencies.map_runtime_state_store =
        &maps_feature_module_->runtime_state_store();
    docking_dependencies.mapping = &mapping_feature_module_->module();
    docking_dependencies.localization = &localization_feature_module_->module();
    docking_dependencies.floor_switch = &floor_switch_feature_module_->module();
    docking_dependencies.safety = &safety_feature_module_->module();
    docking_dependencies.teleop = &teleop_feature_module_->module();
    docking_dependencies.power = &power_feature_module_->module();
    docking_dependencies.runtime_mode = &runtime_mode_;
    docking_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    docking_dependencies.navigation_goal_running = [this]() {
      return navigation_feature_module_ &&
             navigation_feature_module_->mission_runtime().running();
    };
    docking_feature_module_ = std::make_unique<DockingFeatureModule>(
        node_, callback_group_, std::move(docking_configuration),
        std::move(docking_dependencies));

    ElevatorRuntimeConfigurationInputs elevator_configuration_inputs;
    elevator_configuration_inputs.maps_root = maps_runtime_paths.maps_root;
    elevator_configuration_inputs.runtime_map_context_file =
        maps_runtime_paths.runtime_map_context_file;
    elevator_configuration_inputs.navigate_to_pose_action =
        runtime_configuration.navigate_to_pose_action;
    elevator_configuration_inputs.floor_switch_action =
        floor_switch_feature_module_->live_action();
    elevator_configuration_inputs.floor_switch_timeout_sec =
        floor_switch_feature_module_->live_timeout_sec();
    elevator_configuration_inputs.motion_allowed_topic =
        safety_feature_module_->motion_allowed_topic();
    elevator_configuration_inputs.safety_status_topic =
        safety_feature_module_->status_topic();
    elevator_configuration_inputs.navigation_status_topic =
        runtime_configuration.navigate_to_pose_status_topic;
    auto elevator_module_config =
        ElevatorRuntimeConfigurationModule::declare_parameters(
            node_, elevator_configuration_inputs);

    ElevatorFeatureModuleDependencies elevator_dependencies;
    elevator_dependencies.maps = maps_feature_module_.get();
    elevator_dependencies.mapping = &mapping_feature_module_->module();
    elevator_dependencies.floor_switch =
        &floor_switch_feature_module_->module();
    elevator_dependencies.runtime_mode = &runtime_mode_;
    elevator_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    elevator_dependencies.localization = [this]() {
      return localization_feature_module_.get();
    };
    elevator_dependencies.navigation = [this]() {
      return navigation_feature_module_.get();
    };
    elevator_dependencies.docking = [this]() {
      return docking_feature_module_.get();
    };
    elevator_dependencies.teleop = [this]() {
      return teleop_feature_module_ ? &teleop_feature_module_->module()
                                    : nullptr;
    };
    elevator_dependencies.map_frame = tf_map_frame;
    elevator_dependencies.robot_pose_freshness_sec = robot_pose_freshness_sec;
    elevator_feature_module_ = std::make_unique<ElevatorFeatureModule>(
        node_, std::move(elevator_module_config),
        std::move(elevator_dependencies));

    NavigationFeatureModuleDependencies navigation_dependencies;
    navigation_dependencies.maps = &maps_feature_module_->module();
    navigation_dependencies.map_runtime_state_store =
        &maps_feature_module_->runtime_state_store();
    navigation_dependencies.mapping = &mapping_feature_module_->module();
    navigation_dependencies.localization =
        &localization_feature_module_->module();
    navigation_dependencies.floor_switch =
        &floor_switch_feature_module_->module();
    navigation_dependencies.elevator = &elevator_feature_module_->module();
    navigation_dependencies.safety = &safety_feature_module_->module();
    navigation_dependencies.teleop = &teleop_feature_module_->module();
    navigation_dependencies.dock_contact_interlock =
        &docking_feature_module_->contact_interlock();
    navigation_dependencies.docking_job_store =
        &docking_feature_module_->job_store();
    navigation_dependencies.docking_runtime =
        &docking_feature_module_->runtime();
    navigation_dependencies.pre_navigation_undock =
        &docking_feature_module_->pre_navigation_undock();
    navigation_dependencies.runtime_mode = &runtime_mode_;
    navigation_dependencies.delayed_side_effect_unknown_count =
        &delayed_side_effect_unknown_count_;
    navigation_dependencies.api_runtime_running = [this]() {
      return api_gateway_module_ && api_gateway_module_->running();
    };
    navigation_dependencies.post_relocalization_settle_json = [this]() {
      return localization_feature_module_->settle_state_json();
    };
    navigation_feature_module_ = std::make_unique<NavigationFeatureModule>(
        node_, callback_group_, std::move(navigation_configuration),
        std::move(navigation_dependencies));

    localization_feature_module_->complete(
        LocalizationFeatureLateDependencies{navigation_feature_module_.get()});

    docking_feature_module_->complete(DockingFeatureLateDependencies{
        &elevator_feature_module_->module(), navigation_feature_module_.get(),
        &localization_feature_module_->settle()});

    ApiGatewayModulePorts api_gateway_ports;
    api_gateway_ports.authenticated_route =
        [this](const HttpRequest &request,
               const bool maintenance_peer_is_loopback) {
          return application_router_module_->route(
              request, maintenance_peer_is_loopback);
        };
    api_gateway_ports.socket_route = [this](const int client_fd,
                                            const HttpRequest &request) {
      return teleop_feature_module_ &&
             teleop_feature_module_->handle_socket(client_fd, request);
    };
    api_gateway_ports.shutdown_socket_sessions = [this]() {
      if (teleop_feature_module_) {
        teleop_feature_module_->shutdown();
      }
    };
    api_gateway_module_ = std::make_unique<ApiGatewayModule>(
        node_.get_logger(), std::move(api_gateway_config),
        std::move(api_gateway_ports));

    SystemStatusModuleConfig system_status_config;
    system_status_config.floor_status_topic =
        floor_switch_feature_module_->floor_status_topic();
    system_status_config.map_frame = tf_map_frame;
    system_status_config.base_frame = tf_base_frame;
    system_status_config.navigate_to_pose_action =
        runtime_configuration.navigate_to_pose_action;
    system_status_config.docking_status_topic =
        docking_feature_module_->runtime().status_topic();
    system_status_config.localization_trigger_service =
        localization_feature_module_->module().trigger_service_name();
    system_status_config.localization_result_topic =
        localization_feature_module_->module().result_topic_name();
    system_status_config.bms_state_topic = power_feature_module_->state_topic();
    system_status_config.maps_root = maps_runtime_paths.maps_root.string();
    system_status_config.runtime_maps_dir =
        maps_runtime_paths.runtime_maps_dir.string();
    system_status_config.max_http_connections =
        api_gateway_module_->max_connections();
    auto system_status_ports =
        make_system_status_module_ports(SystemStatusWiringDependencies{
            &mapping_feature_module_->module(),
            &navigation_feature_module_->module(),
            &safety_feature_module_->module(), &power_feature_module_->module(),
            &runtime_mode_, &docking_feature_module_->contact_interlock(),
            &localization_feature_module_->module(),
            &navigation_feature_module_->mission_runtime(),
            &docking_feature_module_->job_store(),
            &localization_feature_module_->settle(),
            &floor_switch_feature_module_->module(),
            &maps_feature_module_->module(),
            &subscription_feature_module_->module(), api_gateway_module_.get(),
            &maps_feature_module_->runtime_state_store(),
            &delayed_side_effect_unknown_count_,
            [this](const BridgeStatusSnapshot &bridge,
                   const std::string &context, std::string &detail) {
              return navigation_feature_module_
                  ->bridge_status_safe_for_goal_start(bridge, context, detail);
            }});
    system_status_module_ = std::make_unique<SystemStatusModule>(
        node_, std::move(system_status_config), std::move(system_status_ports));

    application_router_module_ = std::make_unique<ApplicationRouterModule>(
        make_application_router_ports(ApplicationRouterDependencies{
            &elevator_feature_module_->module(), system_status_module_.get(),
            &maps_feature_module_->module(), &mapping_feature_module_->module(),
            &subscription_feature_module_->module(),
            &safety_feature_module_->module(),
            &floor_switch_feature_module_->module(),
            &localization_feature_module_->module(),
            &navigation_feature_module_->module(),
            &docking_feature_module_->http(), api_gateway_module_.get()}));

    api_gateway_module_->start();
  }

  ~Impl() {
    if (api_gateway_module_) {
      api_gateway_module_->stop();
    }
    if (docking_feature_module_) {
      docking_feature_module_->prepare_shutdown();
    }
    if (floor_switch_feature_module_) {
      floor_switch_feature_module_->shutdown();
    }
    if (mapping_feature_module_) {
      mapping_feature_module_->shutdown();
    }
    // ElevatorModule owns workers that call the injected runtime probes. Join
    // them while every mutex/state captured through `this` still exists.
    elevator_feature_module_.reset();
    if (navigation_feature_module_) {
      navigation_feature_module_->shutdown();
    }
    if (docking_feature_module_) {
      docking_feature_module_->shutdown();
    }
    teleop_feature_module_.reset();
  }

private:
  void configure_runtime_permissions() const { ::umask(0002); }

  rclcpp::Node &node_;
  std::atomic<std::uint64_t> delayed_side_effect_unknown_count_{0U};
  std::unique_ptr<ApiGatewayModule> api_gateway_module_;
  std::unique_ptr<ApplicationRouterModule> application_router_module_;
  std::unique_ptr<SystemStatusModule> system_status_module_;
  std::unique_ptr<MapsFeatureModule> maps_feature_module_;
  std::unique_ptr<FloorSwitchFeatureModule> floor_switch_feature_module_;
  std::unique_ptr<MappingFeatureModule> mapping_feature_module_;
  std::unique_ptr<LocalizationFeatureModule> localization_feature_module_;
  std::unique_ptr<NavigationFeatureModule> navigation_feature_module_;
  std::unique_ptr<DockingFeatureModule> docking_feature_module_;
  std::unique_ptr<ElevatorFeatureModule> elevator_feature_module_;
  std::unique_ptr<SafetyFeatureModule> safety_feature_module_;
  std::unique_ptr<SubscriptionFeatureModule> subscription_feature_module_;
  RuntimeModeCoordinator runtime_mode_;

  std::unique_ptr<TeleopFeatureModule> teleop_feature_module_;
  std::unique_ptr<PowerFeatureModule> power_feature_module_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

ApplicationCompositionModule::ApplicationCompositionModule(rclcpp::Node &node)
    : impl_(std::make_unique<Impl>(node)) {}

ApplicationCompositionModule::~ApplicationCompositionModule() = default;

} // namespace robot_api_server::application::composition
