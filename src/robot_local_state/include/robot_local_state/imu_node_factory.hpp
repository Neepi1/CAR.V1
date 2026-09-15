#ifndef ROBOT_LOCAL_STATE__IMU_NODE_FACTORY_HPP_
#define ROBOT_LOCAL_STATE__IMU_NODE_FACTORY_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace robot_local_state
{

// Only the raw/canonical IMU subscription opts into intra-process communication.
// Corrected output, bias, odometry, command and TF endpoints retain DDS.
std::shared_ptr<rclcpp::Node> make_imu_gyro_bias_filter_node(
  const rclcpp::NodeOptions & options,
  bool intra_process_input = false);

}  // namespace robot_local_state

#endif  // ROBOT_LOCAL_STATE__IMU_NODE_FACTORY_HPP_
