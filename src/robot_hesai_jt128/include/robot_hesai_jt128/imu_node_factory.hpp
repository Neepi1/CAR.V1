#ifndef ROBOT_HESAI_JT128__IMU_NODE_FACTORY_HPP_
#define ROBOT_HESAI_JT128__IMU_NODE_FACTORY_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace robot_hesai_jt128
{

// Only the canonical IMU output opts into intra-process communication. Vendor
// input and every other endpoint retain DDS even if options enables node-wide IPC.
std::shared_ptr<rclcpp::Node> make_imu_axis_remap_node(
  const rclcpp::NodeOptions & options,
  bool intra_process_output = false);

}  // namespace robot_hesai_jt128

#endif  // ROBOT_HESAI_JT128__IMU_NODE_FACTORY_HPP_
