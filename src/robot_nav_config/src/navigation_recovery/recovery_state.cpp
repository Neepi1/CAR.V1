#include "robot_nav_config/navigation_recovery/recovery_state.hpp"

#include <cmath>
#include <iterator>
#include <map>

namespace robot_nav_config::navigation_recovery
{
namespace
{
std::int64_t nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool same_goal(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
{
  if (a.poses.empty() || b.poses.empty() || a.header.frame_id.empty() ||
    a.header.frame_id != b.header.frame_id)
  {
    return false;
  }
  const auto & x = a.poses.back().pose;
  const auto & y = b.poses.back().pose;
  // The planner preserves the commissioned endpoint. Compare quaternion
  // orientation modulo sign, without widening the actual goal checker.
  const auto & p = x.orientation;
  const auto & q = y.orientation;
  const double dot = p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w;
  const double pn = p.x * p.x + p.y * p.y + p.z * p.z + p.w * p.w;
  const double qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::hypot(x.position.x - y.position.x, x.position.y - y.position.y) < 1e-6 &&
         std::isfinite(dot) && std::abs(pn - 1.0) < 1e-5 &&
         std::abs(qn - 1.0) < 1e-5 && std::abs(std::abs(dot) - 1.0) < 1e-8;
}
}  // namespace

std::shared_ptr<RecoveryState> for_node(const void * node)
{
  static std::mutex registry_mutex;
  static std::map<const void *, std::weak_ptr<RecoveryState>> registry;
  std::lock_guard<std::mutex> lock(registry_mutex);
  for (auto it = registry.begin(); it != registry.end();) {
    it = it->second.expired() ? registry.erase(it) : std::next(it);
  }
  auto state = registry[node].lock();
  if (!state) {
    state = std::make_shared<RecoveryState>();
    registry[node] = state;
  }
  return state;
}

void RecoveryState::set_active(const bool active)
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = active;
  selected_since_reset_ = false;
  failed_at_.reset();
  prepared_path_.reset();
  current_path_ = nav_msgs::msg::Path{};
  observed_at_ = anchor_at_ = advancing_at_ = 0;
  rearmed_epoch_.reset();
  waiting_ = false;
}

void RecoveryState::reset_progress()
{
  std::lock_guard<std::mutex> lock(mutex_);
  failed_at_.reset();
  selected_since_reset_ = false;
  observed_at_ = anchor_at_ = advancing_at_ = 0;
  waiting_ = false;
  // Humble may reset progress before setPlan. Preserve preparation until the
  // next exact-path comparison; a different path always discards it.
}

void RecoveryState::observe_control()
{
  std::lock_guard<std::mutex> lock(mutex_);
  selected_since_reset_ = active_ && !current_path_.poses.empty();
}

void RecoveryState::observe_progress(const bool progressing, const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (progressing) {
    failed_at_.reset();
  } else if (active_ && selected_since_reset_ && !failed_at_) {
    failed_at_ = stamp_ns;
  }
}

void RecoveryState::observe_pose(
  const double x, const double y, const bool waiting, const std::int64_t stamp_ns,
  const std::string & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !selected_since_reset_ || current_path_.poses.empty() ||
    !std::isfinite(x) || !std::isfinite(y)) {return;}
  const bool restart = waiting || observed_at_ <= 0 || stamp_ns <= observed_at_ ||
    stamp_ns - observed_at_ > 500000000LL || stamp_ns - advancing_at_ > 1000000000LL ||
    frame != observed_frame_;
  observed_frame_ = frame;
  observed_at_ = stamp_ns;
  waiting_ = waiting;
  if (restart) {
    anchor_at_ = advancing_at_ = stamp_ns;
    anchor_x_ = last_x_ = x;
    anchor_y_ = last_y_ = y;
    advancing_samples_ = 0;
    return;
  }
  if (std::hypot(x - last_x_, y - last_y_) >= 0.001) {
    ++advancing_samples_;
    advancing_at_ = stamp_ns;
    last_x_ = x;
    last_y_ = y;
  }
  // A checker returning true, yaw-only motion or small oscillations are not
  // evidence of leaving the blockage. Require sustained net translation in
  // the checker's frame. Its odom pose must never be subtracted from a map goal.
  if (stamp_ns - anchor_at_ >= 2000000000LL &&
    std::hypot(x - anchor_x_, y - anchor_y_) >= 0.20 && advancing_samples_ >= 3)
  {
    ++progress_epoch_;
    anchor_at_ = advancing_at_ = stamp_ns;
    anchor_x_ = last_x_ = x;
    anchor_y_ = last_y_ = y;
    advancing_samples_ = 0;
  }
}

RecoveryState::Observation RecoveryState::inspect(
  const nav_msgs::msg::Path & path, const std::int64_t attempt_started_ns,
  const std::int64_t now_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Observation out;
  out.progress_epoch = progress_epoch_;
  out.matched = active_ && selected_since_reset_ && attempt_started_ns > 0 &&
    same_goal(current_path_, path);
  out.recoverable = out.matched && failed_at_ && *failed_at_ >= attempt_started_ns;
  out.waiting = out.matched && waiting_ && observed_at_ >= attempt_started_ns &&
    now_ns >= observed_at_ && now_ns - observed_at_ <= 1000000000LL;
  return out;
}

bool RecoveryState::observe_plan(const nav_msgs::msg::Path & path, bool * preserve_startup)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!same_goal(current_path_, path)) {
    rearmed_epoch_.reset();
    observed_at_ = anchor_at_ = advancing_at_ = 0;
  }
  const bool prepared = active_ && prepared_path_ && *prepared_path_ == path;
  const bool rearm = prepared && (prepared_task_ != rearmed_task_ ||
    !rearmed_epoch_ || *rearmed_epoch_ != progress_epoch_);
  if (preserve_startup) {*preserve_startup = prepared && !rearm;}
  if (rearm) {rearmed_epoch_ = progress_epoch_; rearmed_task_ = prepared_task_;}
  prepared_path_.reset();
  failed_at_.reset();
  current_path_ = path;
  return rearm;
}

bool RecoveryState::prepare(
  const nav_msgs::msg::Path & path, const std::int64_t attempt_started_ns,
  std::string & reason, const std::int64_t task_started_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !selected_since_reset_ || !failed_at_ ||
    attempt_started_ns <= 0 || *failed_at_ < attempt_started_ns)
  {
    reason = "no_matching_progress_failure";
    return false;
  }
  if (!same_goal(current_path_, path)) {
    reason = "recovery_goal_mismatch";
    return false;
  }
  if (path.poses.size() < 2 || nanoseconds(path.header.stamp) <= *failed_at_) {
    reason = "recovery_requires_post_failure_plan";
    return false;
  }
  prepared_path_ = path;
  prepared_task_ = task_started_ns;
  reason = "progress_failure_replanned_recovery_prepared";
  return true;
}
}  // namespace robot_nav_config::navigation_recovery
