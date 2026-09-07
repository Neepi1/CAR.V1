#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_global_localization/isaac_asset_reloader.hpp"

namespace robot_global_localization
{

struct RosComponentManagerOptions
{
  std::string container_name{"/occupancy_grid_localizer_container"};
  std::string expected_full_node_name{"/occupancy_grid_localizer"};
  std::chrono::nanoseconds operation_timeout{std::chrono::seconds(5)};
};

class RosComponentManagerPort final : public ComponentManagerPort
{
public:
  RosComponentManagerPort(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    RosComponentManagerOptions options);
  ~RosComponentManagerPort() override;

  RosComponentManagerPort(const RosComponentManagerPort &) = delete;
  RosComponentManagerPort & operator=(const RosComponentManagerPort &) = delete;

  ComponentOperationResult preflight() override;
  ComponentCaptureResult capture() override;
  ComponentOperationResult unload(std::uint64_t unique_id) override;
  ComponentOperationResult load(const ComponentLoadRequest & request) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_global_localization
