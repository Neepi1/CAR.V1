#include "robot_api_server/features/localization/localization_feature_module.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_configuration_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::localization
{

namespace
{

void validate_dependencies(
  const LocalizationFeatureModuleDependencies & dependencies)
{
  if (dependencies.floor_switch == nullptr ||
    dependencies.teleop == nullptr ||
    dependencies.delayed_side_effect_unknown_count == nullptr ||
    !dependencies.elevator)
  {
    throw std::invalid_argument(
            "LocalizationFeatureModule requires every core dependency");
  }
}

void validate_late_dependencies(
  const LocalizationFeatureLateDependencies & dependencies)
{
  if (dependencies.navigation == nullptr) {
    throw std::invalid_argument(
            "LocalizationFeatureModule requires the navigation dependency");
  }
}

}  // namespace

class LocalizationFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    LocalizationConfiguration configuration,
    LocalizationFeatureModuleDependencies dependencies)
  : configuration_(std::move(configuration)),
    dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    LocalizationModulePorts ports;
    ports.operation_blocked = [this](
      const std::string & operation,
      std::string & detail)
      {
        return dependencies_.floor_switch->operation_blocked(operation, detail);
      };
    ports.side_effect_started = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_add(
          1U, std::memory_order_acq_rel);
      };
    ports.side_effect_resolved = [this]() {
        dependencies_.delayed_side_effect_unknown_count->fetch_sub(
          1U, std::memory_order_acq_rel);
      };
    ports.run_admitted_operation = [this](
      const std::uint64_t motion_admission_epoch,
      const std::string & operation,
      const AdmittedLocalizationOperation & work)
      {
        if (const auto blocked =
          dependencies_.floor_switch->interlock_response(operation))
        {
          return *blocked;
        }
        return work();
      };
    ports.wait_for_settle = [this](
      const std::uint64_t sequence,
      const std::string & reason,
      const std::string & target_stage)
      {
        const auto result = wait_for_settle(sequence, reason, target_stage, {});
        return RelocalizationSettleResult{
        result.ok, result.failure_code, result.detail};
      };
    ports.settle_state_json = [this]() {
        return settle_state_json();
      };
    module_ = std::make_unique<LocalizationModule>(
      node,
      std::move(callback_group),
      std::move(configuration_.module),
      std::move(ports));

    // TF is a process-level localization input. Keeping it resident avoids
    // reliable /tf DDS endpoint churn from high-rate pose polling.
    module_->ensure_tf_subscription_active();
  }

  void complete(LocalizationFeatureLateDependencies dependencies)
  {
    validate_late_dependencies(dependencies);
    if (complete_) {
      throw std::logic_error("LocalizationFeatureModule is already complete");
    }
    late_dependencies_ = dependencies;

    PostRelocalizationSettlePorts ports;
    ports.bridge_status_snapshot = [this]() {
        return module_->bridge_status_snapshot();
      };
    ports.tf_chain_freshness_snapshot = [this]() {
        return module_->tf_chain_freshness_snapshot();
      };
    ports.tf_chain_freshness_detail = [this](
      const TfChainFreshnessSnapshot & snapshot) {
        return module_->tf_chain_freshness_detail(snapshot);
      };
    ports.base_to_lidar_static_tf_ready = [this]() {
        return module_->base_to_lidar_static_tf_ready();
      };
    ports.local_costmap_update_count = [this]() {
        return late_dependencies_.navigation->terminal_runtime().
               local_costmap_update_count();
      };
    ports.local_costmap_message_filter_drop_count = [this]() {
        return late_dependencies_.navigation->terminal_runtime().
               local_costmap_message_filter_drop_count();
      };
    ports.last_local_costmap_message_filter_drop_text = [this]() {
        return late_dependencies_.navigation->terminal_runtime().
               last_local_costmap_message_filter_drop_text();
      };
    ports.clear_teleop_command = [this]() {
        dependencies_.teleop->clear_command();
      };
    ports.publish_teleop_zero_burst = [this]() {
        dependencies_.teleop->publish_zero_burst();
      };
    ports.publish_navigation_zero_burst = [this]() {
        late_dependencies_.navigation->terminal_runtime().publish_zero_burst();
      };
    ports.steady_now = []() {
        return std::chrono::steady_clock::now();
      };
    ports.wall_time_seconds = []() {
        return robot_api_server::wall_time_seconds();
      };
    ports.sleep_for = [](const std::chrono::milliseconds duration) {
        std::this_thread::sleep_for(duration);
      };
    settle_ = std::make_unique<PostRelocalizationSettleModule>(
      std::move(configuration_.settle), std::move(ports));
    complete_ = true;
  }

  std::string settle_state_json() const
  {
    if (!settle_) {
      return "{\"required\":false,\"in_progress\":false,\"complete\":true,"
             "\"reason\":\"none\",\"target_stage\":\"none\","
             "\"expected_sequence\":0,\"start_time\":0.000,\"min_ms\":0,"
             "\"failure_reason\":\"none\",\"detail\":"
             "\"post relocalization settle module unavailable\"}";
    }
    return settle_->state_json();
  }

  PostRelocalizationSettleResult wait_for_settle(
    const std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested)
  {
    if (!settle_) {
      PostRelocalizationSettleResult result;
      result.expected_sequence = expected_sequence;
      result.detail = "post relocalization settle module unavailable";
      return result;
    }
    return settle_->wait_for_settle(
      expected_sequence, reason, target_next_stage, cancel_requested);
  }

  LocalizationConfiguration configuration_;
  LocalizationFeatureModuleDependencies dependencies_;
  LocalizationFeatureLateDependencies late_dependencies_;
  std::unique_ptr<LocalizationModule> module_;
  std::unique_ptr<PostRelocalizationSettleModule> settle_;
  bool complete_{false};
};

LocalizationFeatureModule::LocalizationFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  LocalizationConfiguration configuration,
  LocalizationFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      std::move(dependencies)))
{
}

LocalizationFeatureModule::~LocalizationFeatureModule() = default;

void LocalizationFeatureModule::complete(
  LocalizationFeatureLateDependencies dependencies)
{
  impl_->complete(std::move(dependencies));
}

bool LocalizationFeatureModule::complete() const {return impl_->complete_;}

LocalizationModule & LocalizationFeatureModule::module()
{
  return *impl_->module_;
}

PostRelocalizationSettleModule & LocalizationFeatureModule::settle()
{
  if (!impl_->settle_) {
    throw std::logic_error("LocalizationFeatureModule is not complete");
  }
  return *impl_->settle_;
}

std::string LocalizationFeatureModule::settle_state_json() const
{
  return impl_->settle_state_json();
}

PostRelocalizationSettleResult LocalizationFeatureModule::wait_for_settle(
  const std::uint64_t expected_sequence,
  const std::string & reason,
  const std::string & target_next_stage,
  const std::function<bool(std::string &)> & cancel_requested)
{
  return impl_->wait_for_settle(
    expected_sequence, reason, target_next_stage, cancel_requested);
}

}  // namespace robot_api_server::features::localization
