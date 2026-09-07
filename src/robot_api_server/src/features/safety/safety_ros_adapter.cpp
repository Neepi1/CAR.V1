#include "robot_api_server/features/safety/safety_ros_adapter.hpp"

#include <mutex>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_api_server::features::safety
{
namespace
{

struct SharedSafetyState
{
  mutable std::mutex mutex;
  SafetyStateSnapshot snapshot;
};

}  // namespace

struct SafetyRosAdapter::Impl
{
  Impl(rclcpp::Node & node, SafetyRosAdapterOptions options)
  : state(std::make_shared<SharedSafetyState>())
  {
    estop_publisher = node.create_publisher<std_msgs::msg::Bool>(
      options.estop_topic, rclcpp::QoS(10).transient_local());

    const auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto shared_state = state;
    status_subscription = node.create_subscription<std_msgs::msg::String>(
      options.status_topic, state_qos,
      [shared_state](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(shared_state->mutex);
        shared_state->snapshot.status = message->data;
      });
    motion_allowed_subscription = node.create_subscription<std_msgs::msg::Bool>(
      options.motion_allowed_topic, state_qos,
      [shared_state](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(shared_state->mutex);
        shared_state->snapshot.motion_allowed = message->data;
        shared_state->snapshot.motion_allowed_valid = true;
      });
  }

  SafetyStateSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    return state->snapshot;
  }

  std::shared_ptr<SharedSafetyState> state;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_allowed_subscription;
};

SafetyRosAdapter::SafetyRosAdapter(
  rclcpp::Node & node,
  SafetyRosAdapterOptions options)
: impl_(std::make_unique<Impl>(node, std::move(options)))
{
}

SafetyRosAdapter::~SafetyRosAdapter() = default;

SafetyStateSnapshot SafetyRosAdapter::snapshot() const
{
  return impl_->snapshot();
}

SafetyMotionDecision SafetyRosAdapter::motion_allowed_decision() const
{
  return evaluate_motion_allowed(snapshot());
}

SafetyHardBlockDecision SafetyRosAdapter::hard_block_decision() const
{
  return evaluate_hard_block(snapshot());
}

void SafetyRosAdapter::publish_estop(const bool active) const
{
  std_msgs::msg::Bool message;
  message.data = active;
  impl_->estop_publisher->publish(message);
}

}  // namespace robot_api_server::features::safety
