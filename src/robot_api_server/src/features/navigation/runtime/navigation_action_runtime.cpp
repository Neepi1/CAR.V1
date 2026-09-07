#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"

#include <algorithm>
#include <future>
#include <stdexcept>
#include <thread>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

#include "robot_api_server/features/navigation/runtime/navigation_cancel_policy.hpp"

namespace robot_api_server::features::navigation
{

namespace
{

using namespace std::chrono_literals;

void require_ports(const NavigationActionRuntimePorts & ports)
{
  if (!ports.delayed_side_effect_started ||
    !ports.delayed_side_effect_resolved ||
    !ports.latch_safety_stop)
  {
    throw std::invalid_argument(
            "navigation action runtime requires every evidence port");
  }
}

class PendingSideEffectEvidence
{
public:
  explicit PendingSideEffectEvidence(const NavigationActionRuntimePorts & ports)
  : ports_(ports)
  {
    ports_.delayed_side_effect_started();
  }

  PendingSideEffectEvidence(const PendingSideEffectEvidence &) = delete;
  PendingSideEffectEvidence & operator=(const PendingSideEffectEvidence &) = delete;

  void resolve()
  {
    if (!resolved_) {
      ports_.delayed_side_effect_resolved();
      resolved_ = true;
    }
  }

private:
  const NavigationActionRuntimePorts & ports_;
  bool resolved_{false};
};

}  // namespace

class NavigationActionRuntime::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    NavigationActionRuntimeConfig config,
    NavigationActionRuntimePorts ports)
  : node_(node), config_(std::move(config)), ports_(std::move(ports))
  {
    require_ports(ports_);
    client_ = rclcpp_action::create_client<NavigateToPose>(
      &node_, config_.action_name);
    status_sub_ = node_.create_subscription<action_msgs::msg::GoalStatusArray>(
      config_.status_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        handle_status(msg);
      });
  }

  Client::SharedPtr client() const
  {
    return client_;
  }

  std::mutex & mutex()
  {
    return action_mutex_;
  }

  bool action_server_ready() const
  {
    return client_ && client_->action_server_is_ready();
  }

  bool wait_for_action_server(const std::chrono::nanoseconds timeout) const
  {
    return client_ && client_->wait_for_action_server(timeout);
  }

  void track_goal(
    GoalHandle::SharedPtr goal_handle,
    std::string pose_id,
    std::string building_id,
    std::string floor_id)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_ = std::move(goal_handle);
    pose_id_ = std::move(pose_id);
    building_id_ = std::move(building_id);
    floor_id_ = std::move(floor_id);
  }

  GoalHandle::SharedPtr active_goal() const
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return active_goal_handle_;
  }

  NavigationTrackedGoalSnapshot tracked_goal_snapshot() const
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return NavigationTrackedGoalSnapshot{
      static_cast<bool>(active_goal_handle_),
      terminal_unknown_,
      pose_id_,
      building_id_,
      floor_id_};
  }

  NavigationActionStatusSnapshot status_snapshot() const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return NavigationActionStatusSnapshot{
      have_status_, action_goal_active_, status_received_at_};
  }

  void clear_tracked_goal_if_terminal_known()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!terminal_unknown_) {
      clear_goal_locked();
    }
  }

  void mark_terminal_unknown(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      return;
    }
    bool newly_unknown = false;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_handle_ == goal_handle && !terminal_unknown_) {
        terminal_unknown_ = true;
        ports_.delayed_side_effect_started();
        newly_unknown = true;
      }
    }
    if (newly_unknown) {
      ports_.latch_safety_stop();
    }
  }

  void mark_terminal_proven(
    const GoalHandle::SharedPtr & goal_handle,
    const bool clear_handle)
  {
    if (!goal_handle) {
      return;
    }
    bool resolve_unknown = false;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (active_goal_handle_ != goal_handle) {
        return;
      }
      if (terminal_unknown_) {
        terminal_unknown_ = false;
        resolve_unknown = true;
      }
      if (clear_handle) {
        clear_goal_locked();
      }
    }
    if (resolve_unknown) {
      ports_.delayed_side_effect_resolved();
    }
  }

  bool cancel_goal_and_prove_terminal(
    const GoalHandle::SharedPtr & goal_handle,
    const std::string & operation,
    std::string & detail)
  {
    if (!goal_handle) {
      detail = "no goal handle available for " + operation;
      return false;
    }
    if (NavigationActionRuntime::goal_status_terminal(goal_handle->get_status())) {
      mark_terminal_proven(goal_handle, true);
      detail = operation + " found the exact goal already terminal";
      return true;
    }

    try {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      auto cancel_future = client_->async_cancel_goal(goal_handle);
      if (cancel_future.wait_for(config_.operation_timeout) !=
        std::future_status::ready)
      {
        mark_terminal_unknown(goal_handle);
        detail = "timed out requesting " + operation +
          "; exact goal terminal state is unknown";
        return false;
      }
      const auto cancel_response = cancel_future.get();
      const auto & expected_goal_id = goal_handle->get_goal_id();
      const bool exact_goal_canceling = std::any_of(
        cancel_response->goals_canceling.cbegin(),
        cancel_response->goals_canceling.cend(),
        [&expected_goal_id](const auto & goal_info) {
          return goal_info.goal_id.uuid == expected_goal_id;
        });
      if (cancel_response->return_code !=
        action_msgs::srv::CancelGoal::Response::ERROR_NONE ||
        !exact_goal_canceling)
      {
        if (NavigationActionRuntime::goal_status_terminal(goal_handle->get_status())) {
          mark_terminal_proven(goal_handle, true);
          detail = operation +
            " observed the exact goal terminal after the cancel response";
          return true;
        }
        mark_terminal_unknown(goal_handle);
        detail = operation +
          " did not acknowledge the exact goal; return_code=" +
          std::to_string(cancel_response->return_code);
        return false;
      }

      auto result_future = client_->async_get_result(goal_handle);
      if (result_future.wait_for(config_.operation_timeout) !=
        std::future_status::ready)
      {
        if (NavigationActionRuntime::goal_status_terminal(goal_handle->get_status())) {
          mark_terminal_proven(goal_handle, true);
          detail = operation +
            " acknowledged the exact goal and status is terminal";
          return true;
        }
        mark_terminal_unknown(goal_handle);
        detail = operation +
          " acknowledged the exact goal but its terminal result is unknown";
        return false;
      }
      const auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::UNKNOWN) {
        mark_terminal_unknown(goal_handle);
        detail = operation + " returned an unknown result for the exact goal";
        return false;
      }
      mark_terminal_proven(goal_handle, true);
      detail = operation + " proved the exact goal terminal";
      return true;
    } catch (const std::exception & exc) {
      mark_terminal_unknown(goal_handle);
      detail = "exception during " + operation + ": " + exc.what();
      return false;
    } catch (...) {
      mark_terminal_unknown(goal_handle);
      detail = "unknown exception during " + operation;
      return false;
    }
  }

  bool cancel_active_goal(const std::string & operation, std::string & detail)
  {
    const auto goal = active_goal();
    if (!goal) {
      detail = "no active API goal handle cached";
      return false;
    }
    return cancel_goal_and_prove_terminal(goal, operation, detail);
  }

  bool cancel_all_goals_with_evidence(
    const std::string & operation,
    std::string & detail)
  {
    PendingSideEffectEvidence pending_side_effect(ports_);
    try {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      auto future = client_->async_cancel_all_goals();
      if (future.wait_for(config_.operation_timeout) !=
        std::future_status::ready)
      {
        ports_.latch_safety_stop();
        detail = operation + " timed out; the late cancel-all outcome is unknown";
        return false;
      }
      const auto response = future.get();
      const auto response_observed_at = std::chrono::steady_clock::now();
      if (response->return_code !=
        action_msgs::srv::CancelGoal::Response::ERROR_NONE)
      {
        pending_side_effect.resolve();
        ports_.latch_safety_stop();
        detail = operation + " rejected with return_code=" +
          std::to_string(response->return_code);
        return false;
      }
      if (robot_api_server::cancel_all_response_proves_idle(
          true, response->goals_canceling.size()))
      {
        pending_side_effect.resolve();
        detail = operation +
          " accepted with zero goals_canceling; the action server proves no "
          "cancelable navigation goal remained";
        return true;
      }

      const auto deadline =
        std::chrono::steady_clock::now() + config_.operation_timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        const auto status = status_snapshot();
        if (status.observed && status.received_at > response_observed_at &&
          !status.goal_active)
        {
          pending_side_effect.resolve();
          detail = operation +
            " accepted and a fresh action-status sample proves idle";
          return true;
        }
        std::this_thread::sleep_for(50ms);
      }
      ports_.latch_safety_stop();
      detail = operation +
        " was accepted but no fresh all-goals-terminal status was observed";
      return false;
    } catch (const std::exception & exc) {
      ports_.latch_safety_stop();
      detail = "exception during " + operation + ": " + exc.what();
      return false;
    } catch (...) {
      ports_.latch_safety_stop();
      detail = "unknown exception during " + operation;
      return false;
    }
  }

private:
  void handle_status(const action_msgs::msg::GoalStatusArray::SharedPtr & msg)
  {
    bool active = false;
    GoalHandle::SharedPtr unknown_goal;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (terminal_unknown_ && active_goal_handle_) {
        unknown_goal = active_goal_handle_;
      }
    }
    bool exact_unknown_goal_terminal = false;
    for (const auto & status : msg->status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
      {
        active = true;
      }
      if (unknown_goal &&
        status.goal_info.goal_id.uuid == unknown_goal->get_goal_id() &&
        NavigationActionRuntime::goal_status_terminal(status.status))
      {
        exact_unknown_goal_terminal = true;
      }
    }
    if (exact_unknown_goal_terminal) {
      mark_terminal_proven(unknown_goal, true);
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    have_status_ = true;
    action_goal_active_ = active;
    status_received_at_ = std::chrono::steady_clock::now();
  }

  void clear_goal_locked()
  {
    active_goal_handle_.reset();
    pose_id_.clear();
    building_id_.clear();
    floor_id_.clear();
  }

  rclcpp::Node & node_;
  NavigationActionRuntimeConfig config_;
  NavigationActionRuntimePorts ports_;
  Client::SharedPtr client_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  mutable std::mutex action_mutex_;
  mutable std::mutex goal_mutex_;
  GoalHandle::SharedPtr active_goal_handle_;
  bool terminal_unknown_{false};
  std::string pose_id_;
  std::string building_id_;
  std::string floor_id_;
  mutable std::mutex status_mutex_;
  bool have_status_{false};
  bool action_goal_active_{false};
  std::chrono::steady_clock::time_point status_received_at_{};
};

NavigationActionRuntime::NavigationActionRuntime(
  rclcpp::Node & node,
  NavigationActionRuntimeConfig config,
  NavigationActionRuntimePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

NavigationActionRuntime::~NavigationActionRuntime() = default;

NavigationActionRuntime::Client::SharedPtr NavigationActionRuntime::client() const
{
  return impl_->client();
}

std::mutex & NavigationActionRuntime::mutex()
{
  return impl_->mutex();
}

bool NavigationActionRuntime::action_server_ready() const
{
  return impl_->action_server_ready();
}

bool NavigationActionRuntime::wait_for_action_server(
  const std::chrono::nanoseconds timeout) const
{
  return impl_->wait_for_action_server(timeout);
}

void NavigationActionRuntime::track_goal(
  GoalHandle::SharedPtr goal_handle,
  std::string pose_id,
  std::string building_id,
  std::string floor_id)
{
  impl_->track_goal(
    std::move(goal_handle), std::move(pose_id), std::move(building_id),
    std::move(floor_id));
}

NavigationActionRuntime::GoalHandle::SharedPtr
NavigationActionRuntime::active_goal() const
{
  return impl_->active_goal();
}

NavigationTrackedGoalSnapshot NavigationActionRuntime::tracked_goal_snapshot() const
{
  return impl_->tracked_goal_snapshot();
}

NavigationActionStatusSnapshot NavigationActionRuntime::status_snapshot() const
{
  return impl_->status_snapshot();
}

void NavigationActionRuntime::clear_tracked_goal_if_terminal_known()
{
  impl_->clear_tracked_goal_if_terminal_known();
}

void NavigationActionRuntime::mark_terminal_unknown(
  const GoalHandle::SharedPtr & goal_handle)
{
  impl_->mark_terminal_unknown(goal_handle);
}

void NavigationActionRuntime::mark_terminal_proven(
  const GoalHandle::SharedPtr & goal_handle,
  const bool clear_handle)
{
  impl_->mark_terminal_proven(goal_handle, clear_handle);
}

bool NavigationActionRuntime::cancel_goal_and_prove_terminal(
  const GoalHandle::SharedPtr & goal_handle,
  const std::string & operation,
  std::string & detail)
{
  return impl_->cancel_goal_and_prove_terminal(goal_handle, operation, detail);
}

bool NavigationActionRuntime::cancel_active_goal(
  const std::string & operation,
  std::string & detail)
{
  return impl_->cancel_active_goal(operation, detail);
}

bool NavigationActionRuntime::cancel_all_goals_with_evidence(
  const std::string & operation,
  std::string & detail)
{
  return impl_->cancel_all_goals_with_evidence(operation, detail);
}

bool NavigationActionRuntime::goal_status_terminal(const std::int8_t status)
{
  return status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
         status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
         status == action_msgs::msg::GoalStatus::STATUS_ABORTED;
}

}  // namespace robot_api_server::features::navigation
