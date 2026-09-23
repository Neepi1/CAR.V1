// Test-only link replacement: no SDK library, CAN socket or physical robot.
// GCC -fno-access-control exposes private callbacks only in this test target.
#include "ranger_base/ranger_messenger.hpp"

#include <array>
#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

namespace {
westonrobot::RangerCoreState fake_state{};
westonrobot::RangerActuatorState fake_actuator{};
std::vector<std::array<double, 3>> motion_calls;
std::vector<uint8_t> mode_calls;
}

namespace westonrobot {
RangerRobot::RangerRobot(Variant) : robot_(nullptr) {}
RangerRobot::~RangerRobot() = default;
bool RangerRobot::Connect(std::string) { return true; }
void RangerRobot::EnableCommandedMode() {}
std::string RangerRobot::RequestVersion(int) { return "FAKE_NO_CAN"; }
void RangerRobot::ResetRobotState() {}
ProtocolVersion RangerRobot::GetParserProtocolVersion() {
  return static_cast<ProtocolVersion>(0);
}
void RangerRobot::SetMotionMode(uint8_t mode) { mode_calls.push_back(mode); }
void RangerRobot::SetMotionCommand(double x, double steer, double angular) {
  motion_calls.push_back({{x, steer, angular}});
}
void RangerRobot::SetLightCommand(AgxLightMode, uint8_t, AgxLightMode, uint8_t) {}
RangerCoreState RangerRobot::GetRobotState() { return fake_state; }
RangerActuatorState RangerRobot::GetActuatorState() { return fake_actuator; }
RangerCommonSensorState RangerRobot::GetCommonSensorState() { return {}; }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("robot_model", "ranger_mini_v3"),
    rclcpp::Parameter("mode_switch_stable_duration_sec", 0.0)});
  auto node = std::make_shared<rclcpp::Node>("navlite_fake_chassis", options);
  westonrobot::RangerROSMessenger messenger(node);
  using Motion = ranger_msgs::msg::MotionState;
  fake_state.time_stamp = westonrobot::SdkClock::now();
  fake_actuator.time_stamp = fake_state.time_stamp;
  fake_state.motion_mode_state.motion_mode = Motion::MOTION_MODE_DUAL_ACKERMAN;
  messenger.PublishStateToROS();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  messenger.PublishStateToROS();
  auto send = [&](double vx, double vy, double wz) {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = vx; msg->linear.y = vy; msg->angular.z = wz;
    messenger.TwistCmdCallback(msg);
  };
  send(.3, 0, 0);
  assert(motion_calls.size() == 1 && motion_calls.back()[0] == .3);
  send(0, 0, 0);
  assert(motion_calls.size() == 2 && motion_calls.back()[0] == 0);
  send(0, .1, 0);
  assert(motion_calls.size() == 3 && motion_calls.back()[0] == 0);
  assert(mode_calls.back() == Motion::MOTION_MODE_PARALLEL);
  assert(messenger.mode_switch_active_);
  fake_state.motion_mode_state.motion_mode = Motion::MOTION_MODE_PARALLEL;
  fake_state.motion_mode_state.mode_changing = 1;
  messenger.PublishStateToROS();
  send(0, .1, 0);
  assert(motion_calls.back()[0] == 0);
  fake_state.motion_mode_state.mode_changing = 0;
  messenger.PublishStateToROS();
  send(0, .1, 0);
  assert(motion_calls.back()[0] == .1);
  assert(!messenger.mode_switch_active_);
  fake_state.motion_state.linear_velocity = .1;
  fake_state.motion_state.steering_angle = M_PI / 2.0;
  fake_actuator.motor_speeds.speed_1 = .4;
  messenger.PublishStateToROS();
  assert(std::abs(messenger.navlite_trace_.odom_[1] - .1) < 1e-6);
  assert(std::abs(messenger.navlite_trace_.wheel_speed_[0] - .4) < 1e-6);
  // No callback means no extra SDK command: the diagnostic observes a gap,
  // but never adds a driver watchdog or synthesizes zero.
  const auto submissions = motion_calls.size();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  messenger.PublishStateToROS();
  assert(motion_calls.size() == submissions);
  // Cached group stamp is deliberately unchanged, exposing its age.
  std::cout << "fake SDK messenger: zero/nonzero/mode hold/release/gap passed; no CAN\n";
  node.reset();
  rclcpp::shutdown();
}
