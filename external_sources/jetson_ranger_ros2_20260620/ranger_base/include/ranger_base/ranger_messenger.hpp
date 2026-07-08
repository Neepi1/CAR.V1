/**
* @file ranger_messenger.hpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#ifndef RANGER_MESSENGER_HPP
#define RANGER_MESSENGER_HPP

//std and c++ inlclude
#include <string>
#include <memory>
#include <cmath>

//ros include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

//third libaray inclue
#include "ugv_sdk/details/robot_base/ranger_base.hpp"
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include <eigen3/Eigen/Core>

//user msg include
#include <ranger_msgs/msg/system_state.hpp>
#include <ranger_msgs/msg/motion_state.hpp>
#include <ranger_msgs/msg/actuator_state_array.hpp>

#include "ranger_msgs/msg/actuator_state.hpp"
#include "ranger_msgs/msg/driver_state.hpp"
#include "ranger_msgs/msg/motor_state.hpp"

#include "ranger_base/ranger_params.hpp"

namespace westonrobot {
class RangerROSMessenger : public std::enable_shared_from_this<RangerROSMessenger>
{
  struct RobotParams {
    double track;
    double wheelbase;
    double max_linear_speed;
    double max_angular_speed;
    double max_speed_cmd;
    double max_steer_angle_central;
    double max_steer_angle_parallel;
    double max_steer_angle_ackermann;
    double max_round_angle;
    double min_turn_radius;
  };

  enum class RangerSubType { kRanger = 0, kRangerMiniV1, kRangerMiniV2 ,kRangerMiniV3};

 public:
  RangerROSMessenger(rclcpp::Node::SharedPtr& node);

  void Run();

 private:
  void LoadParameters();
  void SetupSubscription();
  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);
  void TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  double CalculateSteeringAngle(geometry_msgs::msg::Twist msg, double& radius);
  void UpdateOdometry(double linear, double angular, double angle, double dt);
  double CalculateDualAckermannModelYawRate(double linear, double angle);
  double SelectDualAckermannYawRate(double linear, double angular, double angle);
  double ScaleDualAckermannYawRate(double yaw_rate, double linear) const;
  double ScaleSpinningYawRate(double angular) const;
  bool ShouldHoldZeroCommandInSpinningMode() const;
  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RangerRobot> robot_;
  RangerSubType robot_type_;
  RobotParams robot_params_;

  // constants
  const double steer_angle_tolerance_ = 0.005;  // ~+-0.287 degrees

  // parameters
  std::string robot_model_;
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  int update_rate_;
  bool publish_odom_tf_;
  bool dual_ackermann_odom_use_feedback_twist_;
  double dual_ackermann_linear_odom_scale_;
  double dual_ackermann_linear_odom_scale_max_abs_yaw_rate_;
  double dual_ackermann_yaw_scale_max_abs_yaw_rate_;
  double dual_ackermann_near_straight_yaw_scale_positive_;
  double dual_ackermann_near_straight_yaw_scale_negative_;
  double dual_ackermann_yaw_bias_max_abs_yaw_rate_;
  double dual_ackermann_near_straight_yaw_bias_per_meter_;
  double spinning_base_to_center_x_;
  double spinning_base_to_center_y_;
  double spinning_yaw_scale_positive_;
  double spinning_yaw_scale_negative_;
  bool spinning_zero_cmd_hold_enabled_;
  double spinning_zero_cmd_hold_wz_threshold_radps_;

  uint8_t motion_mode_ = 0;
  bool latest_feedback_valid_ = false;
  uint8_t latest_feedback_motion_mode_ = 0;
  bool latest_feedback_mode_changing_ = false;
  double latest_feedback_angular_velocity_ = 0.0;
  bool latest_odom_twist_valid_ = false;
  double latest_odom_angular_velocity_ = 0.0;
  bool last_commanded_motion_mode_valid_ = false;
  uint8_t last_commanded_motion_mode_ = 0;
  bool parking_mode_;

  rclcpp::Publisher<ranger_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::MotionState>::SharedPtr motion_state_pub_;
  rclcpp::Publisher<ranger_msgs::msg::ActuatorStateArray>::SharedPtr actuator_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // odom variables
  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;
};
}  // namespace westonrobot

#endif  // RANGER_MESSENGER_HPP
