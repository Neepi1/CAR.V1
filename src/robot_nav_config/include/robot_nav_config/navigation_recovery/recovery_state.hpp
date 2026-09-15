#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav_msgs/msg/path.hpp"

namespace robot_nav_config::navigation_recovery
{

// Shared only by plugins of one controller-server node, never by ROS topics.
// Evidence is written by the real progress checker; preparation is consumed
// by setPlan only for the exact new path. No timer or command permission here.
class RecoveryState
{
public:
  struct Observation {
    bool matched{false};
    bool recoverable{false};
    bool waiting{false};
    std::uint64_t progress_epoch{0};
  };
  void set_active(bool active);
  void reset_progress();
  void observe_control();
  void observe_progress(bool progressing, std::int64_t stamp_ns);
  void observe_pose(double x, double y, bool waiting, std::int64_t stamp_ns,
    const std::string & frame = "");
  Observation inspect(const nav_msgs::msg::Path & path, std::int64_t attempt_started_ns,
    std::int64_t now_ns);
  bool observe_plan(const nav_msgs::msg::Path & path, bool * preserve_startup = nullptr);
  bool prepare(const nav_msgs::msg::Path & path, std::int64_t attempt_started_ns,
    std::string & reason, std::int64_t task_started_ns = 0);

private:
  std::mutex mutex_;
  bool active_{false};
  bool selected_since_reset_{false};
  nav_msgs::msg::Path current_path_;
  std::optional<std::int64_t> failed_at_;
  std::optional<nav_msgs::msg::Path> prepared_path_;
  std::int64_t observed_at_{0}, anchor_at_{0}, advancing_at_{0};
  double anchor_x_{0.0}, anchor_y_{0.0}, last_x_{0.0}, last_y_{0.0};
  std::string observed_frame_;
  unsigned advancing_samples_{0};
  bool waiting_{false};
  std::uint64_t progress_epoch_{0};
  std::optional<std::uint64_t> rearmed_epoch_;
  std::int64_t prepared_task_{0}, rearmed_task_{0};
};

std::shared_ptr<RecoveryState> for_node(const void * node);

}  // namespace robot_nav_config::navigation_recovery
