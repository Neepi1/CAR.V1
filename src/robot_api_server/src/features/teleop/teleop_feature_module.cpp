#include "robot_api_server/features/teleop/teleop_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/application/subscriptions/subscription_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/localization/localization_feature_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/power/power_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"
#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

namespace robot_api_server::features::teleop {

namespace {

void validate_dependencies(
    const TeleopFeatureModuleDependencies &dependencies) {
  if (!dependencies.api_gateway || !dependencies.elevator ||
      dependencies.power == nullptr || dependencies.mapping == nullptr ||
      !dependencies.localization || dependencies.subscriptions == nullptr ||
      dependencies.tf_pose_max_age_sec <= 0.0) {
    throw std::invalid_argument(
        "TeleopFeatureModule requires every runtime dependency");
  }
}

} // namespace

class TeleopFeatureModule::Impl {
public:
  Impl(rclcpp::Node &node, TeleopModuleConfig configuration,
       TeleopFeatureModuleDependencies dependencies)
      : dependencies_(std::move(dependencies)) {
    validate_dependencies(dependencies_);

    TeleopModulePorts ports;
    ports.token_allowed = [this](const HttpRequest &request) {
      auto *gateway = dependencies_.api_gateway();
      return gateway != nullptr && gateway->token_allowed(request);
    };
    ports.elevator_interlock = [this]() {
      auto *elevator = dependencies_.elevator();
      if (elevator == nullptr) {
        throw std::logic_error("elevator module is not initialized");
      }
      return elevator->execution_interlock();
    };
    ports.capture_motion_admission_epoch = [this]() {
      auto *elevator = dependencies_.elevator();
      if (elevator == nullptr) {
        throw std::logic_error("elevator module is not initialized");
      }
      return elevator->capture_motion_admission_epoch();
    };
    ports.acquire_motion_admission =
        [this](const ElevatorMotionAdmissionFence::Epoch expected_epoch) {
          auto *elevator = dependencies_.elevator();
          if (elevator == nullptr) {
            throw std::logic_error("elevator module is not initialized");
          }
          return elevator->acquire_motion_admission(expected_epoch);
        };
    ports.charging_contact_active = [this]() {
      return dependencies_.power->charging_contact_active();
    };
    ports.mapping_snapshot = [this](const bool refresh_runtime) {
      const auto mapping = dependencies_.mapping->snapshot(refresh_runtime);
      return TeleopMappingSnapshot{
          mapping.process_running || mapping.process_active,
          mapping.live_map_available,
          mapping.live_map_age_sec,
          mapping.live_map_width,
          mapping.live_map_height,
          mapping.live_map_resolution,
          mapping.known_area_m2,
      };
    };
    ports.pose_snapshot = [this]() {
      TeleopPoseSnapshot pose;
      auto *localization = dependencies_.localization();
      if (localization == nullptr) {
        return pose;
      }
      const auto snapshot = localization->module().latest_pose_snapshot();
      pose.frame_id = snapshot.frame_id;
      pose.x = snapshot.x;
      pose.y = snapshot.y;
      pose.yaw = snapshot.yaw;
      pose.age_sec = snapshot.age_sec;
      pose.available = pose.age_sec <= dependencies_.tf_pose_max_age_sec;
      return pose;
    };
    ports.acquire_subscriptions =
        [this](const std::string &client_id,
               const std::vector<std::string> &resources,
               const std::chrono::milliseconds ttl) {
          dependencies_.subscriptions->acquire(client_id, resources, ttl);
        };
    ports.release_subscriptions =
        [this](const std::string &client_id,
               const std::vector<std::string> &resources) {
          dependencies_.subscriptions->release(client_id, resources);
        };
    ports.runtime_running = [this]() {
      auto *gateway = dependencies_.api_gateway();
      return gateway != nullptr && gateway->running();
    };
    module_ = std::make_unique<TeleopModule>(node, std::move(configuration),
                                             std::move(ports));
  }

  TeleopFeatureModuleDependencies dependencies_;
  std::unique_ptr<TeleopModule> module_;
};

TeleopFeatureModule::TeleopFeatureModule(
    rclcpp::Node &node, TeleopModuleConfig configuration,
    TeleopFeatureModuleDependencies dependencies)
    : impl_(std::make_unique<Impl>(node, std::move(configuration),
                                   std::move(dependencies))) {}

TeleopFeatureModule::~TeleopFeatureModule() = default;

TeleopModule &TeleopFeatureModule::module() { return *impl_->module_; }

bool TeleopFeatureModule::handle_socket(const int client_fd,
                                        const HttpRequest &request) {
  return impl_->module_->handle_socket(client_fd, request);
}

void TeleopFeatureModule::shutdown() { impl_->module_->shutdown(); }

} // namespace robot_api_server::features::teleop
