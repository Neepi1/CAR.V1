#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "robot_nav_config/navigation_recovery/recovery_state.hpp"

namespace robot_nav_config::navigation_recovery
{
// Non-moving boundary: asynchronous controller evidence, clock and status only.
class RecoveryMonitor
{
public:
  virtual ~RecoveryMonitor() = default;
  virtual double now() const = 0;
  virtual void invalidate() = 0;
  virtual std::optional<RecoveryState::Observation> sample(
    const nav_msgs::msg::Path &, const builtin_interfaces::msg::Time &) = 0;
  virtual void report(const geometry_msgs::msg::PoseStamped &, const std::string &,
    unsigned) = 0;
};

class OrdinaryRecoveryLoop : public BT::ControlNode
{
public:
  OrdinaryRecoveryLoop(const std::string & name, const BT::NodeConfiguration & config,
    std::unique_ptr<RecoveryMonitor> monitor)
  : ControlNode(name, config), monitor_(std::move(monitor)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {throw BT::RuntimeError("OrdinaryRecoveryLoop needs two children");}
    const auto goal = config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal");
    if (status() == BT::NodeStatus::IDLE || !goal_ || *goal_ != goal) {
      haltChildren();
      reset();
      goal_ = goal;
      clear_abort();
      const auto * existing_path = config().blackboard->getAny("path");
      if (!existing_path || existing_path->empty()) {
        config().blackboard->set("path", nav_msgs::msg::Path{});
      }
      config().blackboard->set("ordinary_attempt_started", builtin_interfaces::msg::Time{});
    }
    setStatus(BT::NodeStatus::RUNNING);
    nav_msgs::msg::Path path;
    builtin_interfaces::msg::Time started;
    config().blackboard->get("path", path);
    config().blackboard->get("ordinary_attempt_started", started);
    const auto observed = monitor_->sample(path, started);
    if (observed && observed->matched) {
      if (epoch_ && *epoch_ != observed->progress_epoch) {
        retries_ = 0;
        recovering_ = false;
      }
      epoch_ = observed->progress_epoch;
    }
    if (stage_ == Stage::Inspect) {
      if (!observed) {
        monitor_->report(goal, "checking_failure", retries_);
        return monitor_->now() - stage_at_ < 2.0 ? BT::NodeStatus::RUNNING : finish(false);
      }
      if (!observed->recoverable) {return finish(false);}
      // First retry retains the existing immediate recovery; repeated failures
      // wait 5 s, then 10 s. This is scheduling, not an obstacle-clear assertion.
      wait_until_ = monitor_->now() + std::min(10.0, 5.0 * retries_);
      recovering_ = true;
      stage_ = Stage::Wait;
    }
    if (stage_ == Stage::Wait) {
      monitor_->report(goal, "waiting", retries_);
      if (monitor_->now() < wait_until_) {return BT::NodeStatus::RUNNING;}
      ++retries_;
      clear_abort();
      stage_ = Stage::Retry;
    }
    const unsigned child = stage_ == Stage::Tracking ? 0 : 1;
    const auto result = children_nodes_[child]->executeTick();
    if (result == BT::NodeStatus::SUCCESS) {return finish(true);}
    if (result == BT::NodeStatus::FAILURE) {
      bool aborted = false;
      config().blackboard->get("ordinary_follow_aborted", aborted);
      haltChild(child);
      if (!aborted) {return finish(false);}
      stage_ = Stage::Inspect;
      stage_at_ = monitor_->now();
      monitor_->invalidate(); // never classify an abort using its pre-abort reply
      monitor_->report(goal, "checking_failure", retries_);
      return BT::NodeStatus::RUNNING;
    }
    monitor_->report(goal, observed && observed->matched && observed->waiting ? "waiting" :
      (recovering_ && observed && observed->matched ? "recovering" : "tracking"), retries_);
    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    if (goal_) {monitor_->report(*goal_, "idle", retries_);}
    ControlNode::halt();
    reset();
  }

private:
  enum class Stage {Tracking, Inspect, Wait, Retry};
  void clear_abort() {config().blackboard->set("ordinary_follow_aborted", false);}
  void reset()
  {
    stage_ = Stage::Tracking;
    retries_ = 0;
    recovering_ = false;
    epoch_.reset();
    goal_.reset();
    monitor_->invalidate();
  }
  BT::NodeStatus finish(bool success)
  {
    monitor_->report(*goal_, success ? "succeeded" : "failed", retries_);
    haltChildren();
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  std::unique_ptr<RecoveryMonitor> monitor_;
  Stage stage_{Stage::Tracking};
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::optional<std::uint64_t> epoch_;
  unsigned retries_{0};
  bool recovering_{false};
  double stage_at_{0.0}, wait_until_{0.0};
};
}  // namespace robot_nav_config::navigation_recovery
