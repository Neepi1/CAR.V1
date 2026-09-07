#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationActionRuntimeConfig
{
  std::string action_name{"/navigate_to_pose"};
  std::string status_topic{"/navigate_to_pose/_action/status"};
  std::chrono::nanoseconds operation_timeout{std::chrono::seconds(8)};
};

struct NavigationActionRuntimePorts
{
  std::function<void()> delayed_side_effect_started;
  std::function<void()> delayed_side_effect_resolved;
  std::function<void()> latch_safety_stop;
};

struct NavigationActionStatusSnapshot
{
  bool observed{false};
  bool goal_active{false};
  std::chrono::steady_clock::time_point received_at{};
};

struct NavigationTrackedGoalSnapshot
{
  bool available{false};
  bool terminal_unknown{false};
  std::string pose_id;
  std::string building_id;
  std::string floor_id;
};

// Owns the shared NavigateToPose client, action-status evidence and exact-goal
// terminal proof. Ordinary navigation and docking may both use the client, but
// terminal-unknown accounting remains centralized and single-owner.
class NavigationActionRuntime
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Client = rclcpp_action::Client<NavigateToPose>;

  NavigationActionRuntime(
    rclcpp::Node & node,
    NavigationActionRuntimeConfig config,
    NavigationActionRuntimePorts ports);
  ~NavigationActionRuntime();

  NavigationActionRuntime(const NavigationActionRuntime &) = delete;
  NavigationActionRuntime & operator=(const NavigationActionRuntime &) = delete;

  Client::SharedPtr client() const;
  std::mutex & mutex();
  bool action_server_ready() const;
  bool wait_for_action_server(std::chrono::nanoseconds timeout) const;

  void track_goal(
    GoalHandle::SharedPtr goal_handle,
    std::string pose_id,
    std::string building_id,
    std::string floor_id);
  GoalHandle::SharedPtr active_goal() const;
  NavigationTrackedGoalSnapshot tracked_goal_snapshot() const;
  NavigationActionStatusSnapshot status_snapshot() const;
  void clear_tracked_goal_if_terminal_known();

  void mark_terminal_unknown(const GoalHandle::SharedPtr & goal_handle);
  void mark_terminal_proven(
    const GoalHandle::SharedPtr & goal_handle,
    bool clear_handle);

  bool cancel_goal_and_prove_terminal(
    const GoalHandle::SharedPtr & goal_handle,
    const std::string & operation,
    std::string & detail);
  bool cancel_active_goal(const std::string & operation, std::string & detail);
  bool cancel_all_goals_with_evidence(
    const std::string & operation,
    std::string & detail);

  static bool goal_status_terminal(std::int8_t status);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
