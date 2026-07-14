/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include "ranger_base/kinematics_model.hpp"

#include <algorithm>
#include <sstream>

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
// namespace {
// double DegreeToRadian(double x) { return x * M_PI / 180.0; }
// }  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node){

  node_ = node;
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(),"Failed to connect to the CAN port");
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(),"Invalid port name: %s", port_name_.c_str());
    return;
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  dual_ackermann_odom_use_feedback_twist_ =
      node_->declare_parameter<bool>("dual_ackermann_odom_use_feedback_twist", true);
  dual_ackermann_linear_odom_scale_ =
      node_->declare_parameter<double>("dual_ackermann_linear_odom_scale", 1.0);
  dual_ackermann_linear_odom_scale_max_abs_yaw_rate_ =
      node_->declare_parameter<double>(
          "dual_ackermann_linear_odom_scale_max_abs_yaw_rate", 0.06);
  dual_ackermann_yaw_scale_max_abs_yaw_rate_ =
      node_->declare_parameter<double>(
          "dual_ackermann_yaw_scale_max_abs_yaw_rate", 0.06);
  dual_ackermann_near_straight_yaw_scale_positive_ =
      node_->declare_parameter<double>(
          "dual_ackermann_near_straight_yaw_scale_positive", 1.0);
  dual_ackermann_near_straight_yaw_scale_negative_ =
      node_->declare_parameter<double>(
          "dual_ackermann_near_straight_yaw_scale_negative", 1.0);
  dual_ackermann_yaw_bias_max_abs_yaw_rate_ =
      node_->declare_parameter<double>(
          "dual_ackermann_yaw_bias_max_abs_yaw_rate", 0.03);
  dual_ackermann_near_straight_yaw_bias_per_meter_ =
      node_->declare_parameter<double>(
          "dual_ackermann_near_straight_yaw_bias_per_meter", 0.0);
  spinning_base_to_center_x_ =
      node_->declare_parameter<double>("spinning_base_to_center_x", 0.0);
  spinning_base_to_center_y_ =
      node_->declare_parameter<double>("spinning_base_to_center_y", 0.0);
  spinning_yaw_scale_positive_ =
      node_->declare_parameter<double>("spinning_yaw_scale_positive", 1.0);
  spinning_yaw_scale_negative_ =
      node_->declare_parameter<double>("spinning_yaw_scale_negative", 1.0);
  spinning_zero_cmd_hold_enabled_ =
      node_->declare_parameter<bool>("spinning_zero_cmd_hold_enabled", true);
  spinning_zero_cmd_hold_wz_threshold_radps_ =
      node_->declare_parameter<double>("spinning_zero_cmd_hold_wz_threshold_radps", 0.03);
  mode_switch_handshake_enabled_ =
      node_->declare_parameter<bool>("mode_switch_handshake_enabled", true);
  mode_switch_retry_period_sec_ =
      node_->declare_parameter<double>("mode_switch_retry_period_sec", 0.10);
  mode_switch_timeout_sec_ =
      node_->declare_parameter<double>("mode_switch_timeout_sec", 2.0);
  mode_switch_stable_duration_sec_ =
      node_->declare_parameter<double>("mode_switch_stable_duration_sec", 0.15);
  mode_switch_stop_linear_threshold_mps_ =
      node_->declare_parameter<double>("mode_switch_stop_linear_threshold_mps", 0.02);
  mode_switch_stop_angular_threshold_radps_ =
      node_->declare_parameter<double>("mode_switch_stop_angular_threshold_radps", 0.03);
  mode_status_topic_ =
      node_->declare_parameter<std::string>("mode_status_topic", "/ranger_base/status");
  legacy_mode_status_topic_ = node_->declare_parameter<std::string>(
      "legacy_mode_status_topic", "/ranger_mini3_mode_controller/status");

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n dual_ackermann_odom_use_feedback_twist: %d\n "
      "dual_ackermann_linear_odom_scale: %.6f\n "
      "dual_ackermann_linear_odom_scale_max_abs_yaw_rate: %.6f\n "
      "dual_ackermann_yaw_scale_max_abs_yaw_rate: %.6f\n "
      "dual_ackermann_near_straight_yaw_scale_positive: %.6f\n "
      "dual_ackermann_near_straight_yaw_scale_negative: %.6f\n "
      "dual_ackermann_yaw_bias_max_abs_yaw_rate: %.6f\n "
      "dual_ackermann_near_straight_yaw_bias_per_meter: %.6f\n "
      "spinning_base_to_center_x: %.3f\n "
      "spinning_base_to_center_y: %.3f\n "
      "spinning_yaw_scale_positive: %.6f\n "
      "spinning_yaw_scale_negative: %.6f\n "
      "spinning_zero_cmd_hold_enabled: %d\n "
      "spinning_zero_cmd_hold_wz_threshold_radps: %.6f\n "
      "mode_switch_handshake_enabled: %d\n "
      "mode_switch_retry_period_sec: %.3f\n "
      "mode_switch_timeout_sec: %.3f\n "
      "mode_switch_stable_duration_sec: %.3f\n "
      "mode_switch_stop_linear_threshold_mps: %.3f\n "
      "mode_switch_stop_angular_threshold_radps: %.3f\n "
      "mode_status_topic: %s\n "
      "legacy_mode_status_topic: %s\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_, dual_ackermann_odom_use_feedback_twist_,
      dual_ackermann_linear_odom_scale_,
      dual_ackermann_linear_odom_scale_max_abs_yaw_rate_,
      dual_ackermann_yaw_scale_max_abs_yaw_rate_,
      dual_ackermann_near_straight_yaw_scale_positive_,
      dual_ackermann_near_straight_yaw_scale_negative_,
      dual_ackermann_yaw_bias_max_abs_yaw_rate_,
      dual_ackermann_near_straight_yaw_bias_per_meter_,
      spinning_base_to_center_x_,
      spinning_base_to_center_y_,
      spinning_yaw_scale_positive_,
      spinning_yaw_scale_negative_,
      spinning_zero_cmd_hold_enabled_,
      spinning_zero_cmd_hold_wz_threshold_radps_,
      mode_switch_handshake_enabled_,
      mode_switch_retry_period_sec_,
      mode_switch_timeout_sec_,
      mode_switch_stable_duration_sec_,
      mode_switch_stop_linear_threshold_mps_,
      mode_switch_stop_angular_threshold_radps_,
      mode_status_topic_.c_str(),
      legacy_mode_status_topic_.c_str());

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV1Params::max_steer_angle_ackermann;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV2Params::max_steer_angle_ackermann;
    }
    if (robot_model_ == "ranger_mini_v3") {
      robot_type_ = RangerSubType::kRangerMiniV3;

      robot_params_.track = RangerMiniV3Params::track;
      robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV3Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV3Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV3Params::max_steer_angle_ackermann;
    }
     else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerParams::max_steer_angle_ackermann;
    }
  }
    parking_mode_ = false;

}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>("/system_state", 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  mode_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      mode_status_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  if (!legacy_mode_status_topic_.empty() && legacy_mode_status_topic_ != mode_status_topic_) {
    legacy_mode_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        legacy_mode_status_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  // subscriber
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1)
      );
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();
  const auto feedback_motion_mode = state.motion_mode_state.motion_mode;
  latest_feedback_valid_ = true;
  latest_feedback_motion_mode_ = feedback_motion_mode;
  latest_feedback_mode_changing_ = state.motion_mode_state.mode_changing != 0;
  latest_feedback_linear_velocity_ = state.motion_state.linear_velocity;
  latest_feedback_angular_velocity_ = state.motion_state.angular_velocity;
  motion_mode_ = feedback_motion_mode;

  // update odometry
  {
    double dt = (current_time_ - last_time_).seconds();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.angular_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = feedback_motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // publish motion mode
  {
    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = feedback_motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // publish actuator state
  {
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
    //                 actuator_state.motor_angles.angle_5,
    //                 actuator_state.motor_angles.angle_6,
    //                 actuator_state.motor_angles.angle_7,
    //                 actuator_state.motor_angles.angle_8);
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
    //                 actuator_state.motor_speeds.speed_1,
    //                 actuator_state.motor_speeds.speed_2,
    //                 actuator_state.motor_speeds.speed_3,
    //                 actuator_state.motor_speeds.speed_4);

    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;
    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state->driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state->driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state->motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state->driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state->current;
      motor_state_msg.pulse_count = actuator_state.actuator_hs_state->pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state->rpm;
      motor_state_msg.motor_angles = actuator_state.motor_angles.angle_5;
      motor_state_msg.motor_speeds = actuator_state.motor_speeds.speed_1;
      
      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_->publish(batt_msg);
  }

  PublishModeStatus();
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double angle, double dt) {
  // update odometry calculations
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    const double raw_body_vx = std::isfinite(linear) ? linear : 0.0;
    const double yaw_rate = SelectDualAckermannYawRate(raw_body_vx, angular, angle);
    const double yaw_rate_scale_limit =
        std::isfinite(dual_ackermann_linear_odom_scale_max_abs_yaw_rate_) &&
        dual_ackermann_linear_odom_scale_max_abs_yaw_rate_ >= 0.0 ?
        dual_ackermann_linear_odom_scale_max_abs_yaw_rate_ : 0.0;
    const bool near_straight = std::abs(yaw_rate) <= yaw_rate_scale_limit;
    const double linear_scale =
        near_straight &&
        std::isfinite(dual_ackermann_linear_odom_scale_) &&
        dual_ackermann_linear_odom_scale_ > 0.0 ?
        dual_ackermann_linear_odom_scale_ : 1.0;
    const double body_vx = raw_body_vx * linear_scale;

    if (std::abs(yaw_rate) < 1e-9) {
      position_x_ += body_vx * std::cos(theta_) * dt;
      position_y_ += body_vx * std::sin(theta_) * dt;
    } else {
      const double theta0 = theta_;
      const double theta1 = theta_ + yaw_rate * dt;
      position_x_ += (body_vx / yaw_rate) * (std::sin(theta1) - std::sin(theta0));
      position_y_ += -(body_vx / yaw_rate) * (std::cos(theta1) - std::cos(theta0));
      theta_ = theta1;
    }
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      u.phi = M_PI / 2.0;
    } else {
      u.phi = angle;
    }
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    const double spinning_yaw_rate = ScaleSpinningYawRate(angular);
    if (std::abs(spinning_base_to_center_x_) > 1e-6 ||
        std::abs(spinning_base_to_center_y_) > 1e-6) {
      const double theta0 = theta_;
      const double theta1 = theta_ + spinning_yaw_rate * dt;
      const double c0 = std::cos(theta0);
      const double s0 = std::sin(theta0);
      const double c1 = std::cos(theta1);
      const double s1 = std::sin(theta1);
      const double old_center_x =
          c0 * spinning_base_to_center_x_ - s0 * spinning_base_to_center_y_;
      const double old_center_y =
          s0 * spinning_base_to_center_x_ + c0 * spinning_base_to_center_y_;
      const double new_center_x =
          c1 * spinning_base_to_center_x_ - s1 * spinning_base_to_center_y_;
      const double new_center_y =
          s1 * spinning_base_to_center_x_ + c1 * spinning_base_to_center_y_;

      position_x_ += old_center_x - new_center_x;
      position_y_ += old_center_y - new_center_y;
      theta_ = theta1;
    } else {
      SpinningModel::state_type x = {position_x_, position_y_, theta_};
      SpinningModel::control_type u;
      u.w = spinning_yaw_rate;

      boost::numeric::odeint::integrate_const(
          boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
          SpinningModel(u), x, 0.0, dt, (dt / 10.0));

      position_x_ = x[0];
      position_y_ = x[1];
      theta_ = x[2];
    }
  }

  // update odometry topics
  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    const double raw_body_vx = std::isfinite(linear) ? linear : 0.0;
    const double yaw_rate = SelectDualAckermannYawRate(raw_body_vx, angular, angle);
    const double yaw_rate_scale_limit =
        std::isfinite(dual_ackermann_linear_odom_scale_max_abs_yaw_rate_) &&
        dual_ackermann_linear_odom_scale_max_abs_yaw_rate_ >= 0.0 ?
        dual_ackermann_linear_odom_scale_max_abs_yaw_rate_ : 0.0;
    const bool near_straight = std::abs(yaw_rate) <= yaw_rate_scale_limit;
    const double linear_scale =
        near_straight &&
        std::isfinite(dual_ackermann_linear_odom_scale_) &&
        dual_ackermann_linear_odom_scale_ > 0.0 ?
        dual_ackermann_linear_odom_scale_ : 1.0;
    const double body_vx = raw_body_vx * linear_scale;
    odom_msg.twist.twist.linear.x = body_vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = yaw_rate;
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    double phi = angle;

    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      phi = M_PI / 2.0;
    }
    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);

    odom_msg.twist.twist.angular.z = 0;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    const double spinning_yaw_rate = ScaleSpinningYawRate(angular);
    odom_msg.twist.twist.linear.x = spinning_yaw_rate * spinning_base_to_center_y_;
    odom_msg.twist.twist.linear.y = -spinning_yaw_rate * spinning_base_to_center_x_;
    odom_msg.twist.twist.angular.z = spinning_yaw_rate;
  }

  latest_odom_twist_valid_ = true;
  latest_odom_angular_velocity_ = odom_msg.twist.twist.angular.z;
  odom_pub_->publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  double steer_cmd = 0.0;
  double radius = std::numeric_limits<double>::infinity();
  constexpr double kCmdVelLateralDeadband = 1.0e-4;
  if (std::abs(msg->linear.y) <= kCmdVelLateralDeadband) {
    msg->linear.y = 0.0;
  }

  // analyze Twist msg and switch motion_mode
  // check for parking mode, only applicable to RangerMiniV2
  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  }

  const bool zero_cmd =
      std::abs(msg->linear.x) <= kCmdVelLateralDeadband &&
      std::abs(msg->linear.y) <= kCmdVelLateralDeadband &&
      std::abs(msg->angular.z) <= kCmdVelLateralDeadband;
  uint8_t desired_mode = MotionState::MOTION_MODE_DUAL_ACKERMAN;
  if (zero_cmd && ShouldHoldZeroCommandInSpinningMode()) {
    desired_mode = MotionState::MOTION_MODE_SPINNING;
  } else if (std::abs(msg->linear.y) > kCmdVelLateralDeadband) {
    if (msg->linear.x == 0.0 && robot_type_ == RangerSubType::kRangerMiniV1) {
      desired_mode = MotionState::MOTION_MODE_SIDE_SLIP;
    } else {
      desired_mode = MotionState::MOTION_MODE_PARALLEL;
    }
  } else {
    steer_cmd = CalculateSteeringAngle(*msg, radius);
    // Use minimum turn radius to switch between dual ackerman and spinning mode
    if (radius < robot_params_.min_turn_radius) {
      desired_mode = MotionState::MOTION_MODE_SPINNING;
    } else {
      desired_mode = MotionState::MOTION_MODE_DUAL_ACKERMAN;
    }
  }

  desired_motion_mode_valid_ = true;
  desired_motion_mode_ = desired_mode;
  if (!EnsureMotionModeReady(desired_mode)) {
    return;
  }
  motion_mode_ = desired_mode;

  // send motion command to robot
  switch (desired_mode) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      robot_->SetMotionCommand(msg->linear.x, steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_PARALLEL: {
      steer_cmd = atan(msg->linear.y / msg->linear.x);

      static double last_nonzero_x = 1.0; 
      
      if (msg->linear.x != 0.0) {
          last_nonzero_x = msg->linear.x; 
      }

      if (std::signbit(msg->linear.x))
      {
        steer_cmd = -steer_cmd;
      }
      
      if (steer_cmd > robot_params_.max_steer_angle_parallel) {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel) {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      double vel = 1.0;
      
      if (msg->linear.x == 0.0 && msg->linear.y != 0.0) {
          // std::cout << "MOTION_MODE_SIDE_SLIP" << std::endl;
          
          if (std::signbit(last_nonzero_x)) {
              steer_cmd = -std::abs(steer_cmd); 
          } else {
              steer_cmd = std::abs(steer_cmd);
          }
          vel = msg->linear.y >= 0 ? 1.0 : -1.0;
      } else {
          vel = msg->linear.x >= 0 ? 1.0 : -1.0;
      }
      robot_->SetMotionCommand(vel * sqrt(msg->linear.x * msg->linear.x +
                                          msg->linear.y * msg->linear.y),
                               steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_SPINNING: {
      double a_v = msg->angular.z;
      if (a_v > robot_params_.max_angular_speed) {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed) {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      break;
    }
    case MotionState::MOTION_MODE_SIDE_SLIP: {
      double l_v = msg->linear.y;
      if (l_v > robot_params_.max_linear_speed) {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed) {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      break;
    }
  }
}


geometry_msgs::msg::Quaternion RangerROSMessenger::createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  if (angular < 1e-6) {
    radius = std::numeric_limits<double>::infinity(); 
    return 0.0; 
  }
  if (linear < 1e-6) {
    radius = 0.0;
    return 0.0;
  }
  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1 : -1;

  double l, phi_i;
  l = robot_params_.wheelbase;

  // Keep /cmd_vel angular.z semantics as the requested body yaw rate.
  // The legacy formula atan((L / 2) / R) under-commands Ranger Mini 3
  // Ackermann steering because odometry later treats steering feedback as an
  // inner steering angle and converts it to the central angle again.
  const double central_arg = std::min((angular * l) / (2.0 * linear), 1.0);
  const double central_phi = std::asin(central_arg);
  phi_i = ConvertCentralAngleToInner(central_phi);

  const double max_phi_rad = 40.0 * M_PI / 180.0;
  phi_i = std::min(phi_i, max_phi_rad);

  return k * phi_i;
}

double RangerROSMessenger::CalculateDualAckermannModelYawRate(double linear,
                                                              double angle) {
  if (!std::isfinite(linear) || !std::isfinite(angle) ||
      std::abs(robot_params_.wheelbase) < 1e-9) {
    return 0.0;
  }

  return 2.0 * linear * std::sin(ConvertInnerAngleToCentral(angle)) /
         robot_params_.wheelbase;
}

double RangerROSMessenger::SelectDualAckermannYawRate(double linear,
                                                      double angular,
                                                      double angle) {
  const double model_yaw_rate = CalculateDualAckermannModelYawRate(linear, angle);
  double yaw_rate = model_yaw_rate;

  if (!dual_ackermann_odom_use_feedback_twist_ || !std::isfinite(angular)) {
    return ScaleDualAckermannYawRate(yaw_rate, linear);
  }

  // Some firmware builds may leave feedback angular velocity at zero in
  // Ackermann mode. In that case keep the previous steering-derived odometry.
  if (std::abs(angular) < 1e-6 && std::abs(model_yaw_rate) > 1e-3) {
    return ScaleDualAckermannYawRate(yaw_rate, linear);
  }

  yaw_rate = angular;
  return ScaleDualAckermannYawRate(yaw_rate, linear);
}

double RangerROSMessenger::ScaleDualAckermannYawRate(double yaw_rate,
                                                     double linear) const {
  if (!std::isfinite(yaw_rate)) {
    return yaw_rate;
  }

  double calibrated_yaw_rate = yaw_rate;
  const double max_abs_yaw_rate =
      std::isfinite(dual_ackermann_yaw_scale_max_abs_yaw_rate_) &&
      dual_ackermann_yaw_scale_max_abs_yaw_rate_ >= 0.0 ?
      dual_ackermann_yaw_scale_max_abs_yaw_rate_ : 0.0;
  if (std::abs(yaw_rate) <= max_abs_yaw_rate) {
    const double scale = yaw_rate >= 0.0 ?
        dual_ackermann_near_straight_yaw_scale_positive_ :
        dual_ackermann_near_straight_yaw_scale_negative_;
    if (std::isfinite(scale) && scale > 0.0) {
      calibrated_yaw_rate *= scale;
    }
  }

  const double bias_max_abs_yaw_rate =
      std::isfinite(dual_ackermann_yaw_bias_max_abs_yaw_rate_) &&
      dual_ackermann_yaw_bias_max_abs_yaw_rate_ >= 0.0 ?
      dual_ackermann_yaw_bias_max_abs_yaw_rate_ : 0.0;
  if (std::abs(yaw_rate) <= bias_max_abs_yaw_rate &&
      std::isfinite(linear) &&
      std::isfinite(dual_ackermann_near_straight_yaw_bias_per_meter_)) {
    calibrated_yaw_rate +=
        linear * dual_ackermann_near_straight_yaw_bias_per_meter_;
  }

  return calibrated_yaw_rate;
}

double RangerROSMessenger::ScaleSpinningYawRate(double angular) const {
  const double scale =
      angular >= 0.0 ? spinning_yaw_scale_positive_ : spinning_yaw_scale_negative_;
  if (!std::isfinite(scale) || scale <= 0.0) {
    return angular;
  }
  return angular * scale;
}

void RangerROSMessenger::ResetModeSwitchState(const uint8_t desired_mode) {
  const auto now = std::chrono::steady_clock::now();
  mode_switch_active_ = true;
  mode_switch_stop_stable_ = false;
  mode_switch_request_sent_ = false;
  mode_switch_target_ = desired_mode;
  mode_switch_state_ = ModeSwitchState::kStopping;
  mode_switch_started_at_ = now;
  mode_switch_stop_stable_since_ = now;
  mode_switch_last_request_at_ = now;
}

bool RangerROSMessenger::EnsureMotionModeReady(const uint8_t desired_mode) {
  if (!mode_switch_handshake_enabled_) {
    last_commanded_motion_mode_valid_ = true;
    last_commanded_motion_mode_ = desired_mode;
    robot_->SetMotionMode(desired_mode);
    mode_switch_active_ = false;
    mode_switch_state_ = ModeSwitchState::kStable;
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool feedback_ready =
      latest_feedback_valid_ &&
      latest_feedback_motion_mode_ == desired_mode &&
      !latest_feedback_mode_changing_;
  if (feedback_ready) {
    if (mode_switch_active_) {
      const double elapsed =
          std::chrono::duration<double>(now - mode_switch_started_at_).count();
      RCLCPP_INFO(
          node_->get_logger(),
          "Ranger mode switch confirmed: target=%s(%u) elapsed=%.3fs",
          MotionModeName(desired_mode), desired_mode, elapsed);
    }
    mode_switch_active_ = false;
    mode_switch_stop_stable_ = false;
    mode_switch_request_sent_ = false;
    mode_switch_state_ = ModeSwitchState::kStable;
    return true;
  }

  if (!mode_switch_active_ || mode_switch_target_ != desired_mode) {
    ResetModeSwitchState(desired_mode);
  }

  // The Ranger firmware ignores velocity commands while changing mode. Keep
  // the chassis at zero and do not release the requested command until the
  // mode feedback explicitly confirms the transition.
  robot_->SetMotionCommand(0.0, 0.0, 0.0);

  const double linear_threshold =
      std::max(0.0, mode_switch_stop_linear_threshold_mps_);
  const double angular_threshold =
      std::max(0.0, mode_switch_stop_angular_threshold_radps_);
  const bool stopped =
      latest_feedback_valid_ && latest_odom_twist_valid_ &&
      std::isfinite(latest_feedback_linear_velocity_) &&
      std::isfinite(latest_odom_angular_velocity_) &&
      std::abs(latest_feedback_linear_velocity_) <= linear_threshold &&
      std::abs(latest_odom_angular_velocity_) <= angular_threshold;

  if (!stopped) {
    mode_switch_stop_stable_ = false;
    mode_switch_state_ = ModeSwitchState::kStopping;
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Ranger mode switch waiting for stop: target=%s(%u) linear=%.4f angular=%.4f",
        MotionModeName(desired_mode), desired_mode,
        latest_feedback_linear_velocity_, latest_odom_angular_velocity_);
    return false;
  }

  if (!mode_switch_stop_stable_) {
    mode_switch_stop_stable_ = true;
    mode_switch_stop_stable_since_ = now;
  }
  const double stable_duration =
      std::chrono::duration<double>(now - mode_switch_stop_stable_since_).count();
  if (stable_duration < std::max(0.0, mode_switch_stable_duration_sec_)) {
    mode_switch_state_ = ModeSwitchState::kStopping;
    return false;
  }

  const double elapsed =
      std::chrono::duration<double>(now - mode_switch_started_at_).count();
  if (mode_switch_timeout_sec_ > 0.0 && elapsed >= mode_switch_timeout_sec_) {
    mode_switch_state_ = ModeSwitchState::kTimedOut;
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Ranger mode switch timeout: target=%s(%u) actual=%s(%u) changing=%s elapsed=%.3fs; holding zero",
        MotionModeName(desired_mode), desired_mode,
        MotionModeName(latest_feedback_motion_mode_), latest_feedback_motion_mode_,
        latest_feedback_mode_changing_ ? "true" : "false", elapsed);
  } else {
    mode_switch_state_ = ModeSwitchState::kWaitingAck;
  }

  const double since_last_request = mode_switch_request_sent_
      ? std::chrono::duration<double>(now - mode_switch_last_request_at_).count()
      : std::numeric_limits<double>::infinity();
  if (!mode_switch_request_sent_ ||
      since_last_request >= std::max(0.02, mode_switch_retry_period_sec_)) {
    robot_->SetMotionMode(desired_mode);
    last_commanded_motion_mode_valid_ = true;
    last_commanded_motion_mode_ = desired_mode;
    mode_switch_request_sent_ = true;
    mode_switch_last_request_at_ = now;
  }

  return false;
}

const char* RangerROSMessenger::ModeSwitchStateName() const {
  switch (mode_switch_state_) {
    case ModeSwitchState::kStable:
      return "stable";
    case ModeSwitchState::kStopping:
      return "stopping";
    case ModeSwitchState::kWaitingAck:
      return "waiting_ack";
    case ModeSwitchState::kTimedOut:
      return "mode_switch_timeout";
  }
  return "unknown";
}

const char* RangerROSMessenger::MotionModeName(const uint8_t mode) {
  switch (mode) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN:
      return "MOTION_MODE_DUAL_ACKERMAN";
    case MotionState::MOTION_MODE_PARALLEL:
      return "MOTION_MODE_PARALLEL";
    case MotionState::MOTION_MODE_SPINNING:
      return "MOTION_MODE_SPINNING";
    case MotionState::MOTION_MODE_SIDE_SLIP:
      return "MOTION_MODE_SIDE_SLIP";
    default:
      return "MOTION_MODE_UNKNOWN";
  }
}

const char* RangerROSMessenger::MotionModeShortName(const uint8_t mode) {
  switch (mode) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN:
      return "dual_ackerman";
    case MotionState::MOTION_MODE_PARALLEL:
      return "parallel";
    case MotionState::MOTION_MODE_SPINNING:
      return "spinning";
    case MotionState::MOTION_MODE_SIDE_SLIP:
      return "side_slip";
    default:
      return "unknown";
  }
}

void RangerROSMessenger::PublishModeStatus() {
  if (!mode_status_pub_) return;

  const uint8_t desired_mode = desired_motion_mode_valid_
      ? desired_motion_mode_
      : latest_feedback_motion_mode_;
  const bool aligned =
      desired_motion_mode_valid_ && latest_feedback_valid_ &&
      desired_mode == latest_feedback_motion_mode_ &&
      !latest_feedback_mode_changing_;
  double elapsed = 0.0;
  if (mode_switch_active_) {
    elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - mode_switch_started_at_).count();
  }

  std::ostringstream json;
  json << "{\"state\":\"" << ModeSwitchStateName() << "\""
       << ",\"valid\":true"
       << ",\"owner\":\"ranger_base\""
       << ",\"mode_switch_handshake_enabled\":"
       << (mode_switch_handshake_enabled_ ? "true" : "false")
       << ",\"desired_motion_mode\":{\"code\":" << static_cast<int>(desired_mode)
       << ",\"name\":\"" << MotionModeName(desired_mode) << "\""
       << ",\"short\":\"" << MotionModeShortName(desired_mode) << "\""
       << ",\"source\":\"cmd_vel\"}"
       << ",\"actual_motion_mode\":{\"available\":"
       << (latest_feedback_valid_ ? "true" : "false")
       << ",\"fresh\":" << (latest_feedback_valid_ ? "true" : "false")
       << ",\"code\":" << static_cast<int>(latest_feedback_motion_mode_)
       << ",\"name\":\"" << MotionModeName(latest_feedback_motion_mode_) << "\""
       << ",\"short\":\"" << MotionModeShortName(latest_feedback_motion_mode_) << "\""
       << ",\"mode_changing\":"
       << (latest_feedback_mode_changing_ ? "true" : "false") << "}"
       << ",\"mode_aligned\":" << (aligned ? "true" : "false")
       << ",\"motion_mode_matched\":" << (aligned ? "true" : "false")
       << ",\"mode_switch_elapsed_sec\":" << elapsed
       << "}";

  std_msgs::msg::String msg;
  msg.data = json.str();
  mode_status_pub_->publish(msg);
  if (legacy_mode_status_pub_) {
    legacy_mode_status_pub_->publish(msg);
  }
}

bool RangerROSMessenger::ShouldHoldZeroCommandInSpinningMode() const {
  if (!spinning_zero_cmd_hold_enabled_) {
    return false;
  }
  const bool feedback_spinning =
      latest_feedback_valid_ &&
      latest_feedback_motion_mode_ == MotionState::MOTION_MODE_SPINNING;
  const bool last_command_spinning =
      last_commanded_motion_mode_valid_ &&
      last_commanded_motion_mode_ == MotionState::MOTION_MODE_SPINNING;
  if (!feedback_spinning && !last_command_spinning) {
    return false;
  }
  if (!latest_feedback_valid_) {
    return true;
  }

  const double threshold =
      std::isfinite(spinning_zero_cmd_hold_wz_threshold_radps_) &&
      spinning_zero_cmd_hold_wz_threshold_radps_ >= 0.0 ?
      spinning_zero_cmd_hold_wz_threshold_radps_ : 0.03;
  const double stop_angular_velocity =
      latest_odom_twist_valid_ ? latest_odom_angular_velocity_ :
      latest_feedback_angular_velocity_;
  return latest_feedback_mode_changing_ ||
         !std::isfinite(stop_angular_velocity) ||
         std::abs(stop_angular_velocity) > threshold;
}

double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = std::abs(angle);

  phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                  (robot_params_.wheelbase * std::cos(phi_i) +
                   robot_params_.track * std::sin(phi_i)));

  phi *= angle >= 0 ? 1.0 : -1.0;
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = std::abs(angle);
  double phi_i = 0;

  phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                    (robot_params_.wheelbase * std::cos(phi) -
                     robot_params_.track * std::sin(phi)));
  phi_i *= angle >= 0 ? 1.0 : -1.0;
  return phi_i;
}
}  // namespace westonrobot
