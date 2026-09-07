#pragma once

#include <memory>
#include <string>

#include "robot_api_server/features/safety/safety_state.hpp"

namespace rclcpp
{
class Node;
}

namespace robot_api_server::features::safety
{

struct SafetyRosAdapterOptions
{
  std::string estop_topic{"/safety/estop"};
  std::string status_topic{"/safety/status"};
  std::string motion_allowed_topic{"/safety/motion_allowed"};
};

// Owns the process-resident ROS edge between the App gateway and robot_safety.
// Final command arbitration, watchdogs, and interlocks remain in robot_safety.
class SafetyRosAdapter
{
public:
  SafetyRosAdapter(rclcpp::Node & node, SafetyRosAdapterOptions options);
  ~SafetyRosAdapter();

  SafetyRosAdapter(const SafetyRosAdapter &) = delete;
  SafetyRosAdapter & operator=(const SafetyRosAdapter &) = delete;

  SafetyStateSnapshot snapshot() const;
  SafetyMotionDecision motion_allowed_decision() const;
  SafetyHardBlockDecision hard_block_decision() const;

  void publish_estop(bool active) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::safety
