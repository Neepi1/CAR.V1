#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robot_api_server::features::elevator
{
class ElevatorModule;
}

namespace robot_api_server::features::floor_switch
{
class FloorSwitchModule;
}

namespace robot_api_server::features::navigation
{
class NavigationFeatureModule;
}

namespace robot_api_server::features::teleop
{
class TeleopModule;
}

namespace robot_api_server::features::localization
{

struct LocalizationConfiguration;
class LocalizationModule;
class PostRelocalizationSettleModule;
struct PostRelocalizationSettleResult;

// Stable services used by localization. Elevator is late-created in the
// application composition graph, so it is intentionally resolved only when
// an admitted API operation actually executes.
struct LocalizationFeatureModuleDependencies
{
  features::floor_switch::FloorSwitchModule * floor_switch{nullptr};
  features::teleop::TeleopModule * teleop{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};
  std::function<features::elevator::ElevatorModule * ()> elevator;
};

// The stability barrier consumes navigation costmap evidence, while
// navigation itself consumes localization. Attach this dependency once, after
// navigation construction and before the API gateway starts.
struct LocalizationFeatureLateDependencies
{
  features::navigation::NavigationFeatureModule * navigation{nullptr};
};

// Owns the complete API-side localization aggregate: canonical TF
// observation, Isaac/AMCL triggering and result evidence, bridge correction
// control, and the post-relocalization stability transaction.
class LocalizationFeatureModule
{
public:
  LocalizationFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    LocalizationConfiguration configuration,
    LocalizationFeatureModuleDependencies dependencies);
  ~LocalizationFeatureModule();

  LocalizationFeatureModule(const LocalizationFeatureModule &) = delete;
  LocalizationFeatureModule & operator=(const LocalizationFeatureModule &) = delete;

  void complete(LocalizationFeatureLateDependencies dependencies);
  bool complete() const;

  LocalizationModule & module();
  PostRelocalizationSettleModule & settle();

  std::string settle_state_json() const;
  PostRelocalizationSettleResult wait_for_settle(
    std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested = {});

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::localization
