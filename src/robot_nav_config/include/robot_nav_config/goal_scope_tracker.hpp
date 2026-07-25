#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace robot_nav_config
{

struct GoalSignature
{
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class GoalScopeTracker
{
public:
  void set_thresholds(const double xy_threshold, const double yaw_threshold)
  {
    xy_threshold_ = std::max(0.0, xy_threshold);
    yaw_threshold_ = std::max(0.0, yaw_threshold);
  }

  void reset()
  {
    have_goal_ = false;
    startup_alignment_consumed_ = false;
    goal_ = GoalSignature{};
  }

  bool observe_goal(const GoalSignature & goal)
  {
    const bool changed = !have_goal_ || goal.frame_id != goal_.frame_id ||
      std::hypot(goal.x - goal_.x, goal.y - goal_.y) > xy_threshold_ ||
      std::abs(shortest_angular_distance(goal_.yaw, goal.yaw)) > yaw_threshold_;
    if (changed) {
      goal_ = goal;
      have_goal_ = true;
      startup_alignment_consumed_ = false;
    }
    return changed;
  }

  void mark_startup_alignment_consumed()
  {
    if (have_goal_) {
      startup_alignment_consumed_ = true;
    }
  }

  bool have_goal() const
  {
    return have_goal_;
  }

  bool startup_alignment_consumed() const
  {
    return startup_alignment_consumed_;
  }

private:
  static double shortest_angular_distance(const double from, const double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  double xy_threshold_{0.01};
  double yaw_threshold_{0.01};
  bool have_goal_{false};
  bool startup_alignment_consumed_{false};
  GoalSignature goal_;
};

}  // namespace robot_nav_config
