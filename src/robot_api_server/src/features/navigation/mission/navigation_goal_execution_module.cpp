#include "robot_api_server/features/navigation/mission/navigation_goal_execution_module.hpp"

#include <cmath>
#include <future>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_api_server/features/navigation/mission/navigation_goal_job.hpp"

using namespace std::chrono_literals;

namespace robot_api_server::features::navigation
{
namespace
{

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> callback)
  : callback_(std::move(callback))
  {
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

  ~ScopeExit()
  {
    if (callback_) {
      callback_();
    }
  }

private:
  std::function<void()> callback_;
};

// A timed-out action submission may still reach Nav2. Destruction without
// resolve() deliberately leaves the shared unknown-side-effect count set.
class PendingSideEffectEvidence
{
public:
  explicit PendingSideEffectEvidence(NavigationGoalExecutionPorts & ports)
  : ports_(ports)
  {
    ports_.delayed_side_effect_started();
  }

  PendingSideEffectEvidence(const PendingSideEffectEvidence &) = delete;
  PendingSideEffectEvidence & operator=(const PendingSideEffectEvidence &) = delete;

  void resolve() noexcept
  {
    if (!resolved_) {
      ports_.delayed_side_effect_resolved();
      resolved_ = true;
    }
  }

private:
  NavigationGoalExecutionPorts & ports_;
  bool resolved_{false};
};

}  // namespace

NavigationGoalExecutionModule::NavigationGoalExecutionModule(
  NavigationGoalExecutionConfig config,
  NavigationActionRuntime & action_runtime,
  NavigationMissionRuntime & mission_runtime,
  NavigationCompletionPolicy & completion_policy,
  NavigationBridgeWait & bridge_wait,
  NavigationTerminalControl & terminal_control,
  NavigationTerminalRuntimeModule & terminal_runtime,
  NavigationGoalExecutionPorts ports)
: config_(std::move(config)),
  action_runtime_(action_runtime),
  mission_runtime_(mission_runtime),
  completion_policy_(completion_policy),
  bridge_wait_(bridge_wait),
  terminal_control_(terminal_control),
  terminal_runtime_(terminal_runtime),
  ports_(std::move(ports))
{
  if (!ports_.floor_runtime_operation_blocked || !ports_.undock_before_navigation ||
    !ports_.bridge_safe_for_goal_start || !ports_.bridge_readiness_snapshot ||
    !ports_.request_amcl_nomotion_update || !ports_.request_bridge_correction_pause ||
    !ports_.robot_pose_snapshot || !ports_.safety_hard_blocked ||
    !ports_.docking_job_running || !ports_.dock_contact_blocked ||
    !ports_.cancel_active_goal || !ports_.cancel_goal_for_handoff ||
    !ports_.request_goal_cancel || !ports_.navigation_runtime_active ||
    !ports_.set_navigation_runtime_state || !ports_.timestamp_now ||
    !ports_.goal_stamp || !ports_.delayed_side_effect_started ||
    !ports_.delayed_side_effect_resolved || !ports_.warn)
  {
    throw std::invalid_argument("NavigationGoalExecutionModule requires all ports");
  }
}

bool NavigationGoalExecutionModule::try_acquire_predock_motion_owner(const bool lateral)
{
  std::lock_guard<std::mutex> lock(motion_owner_mutex_);
  if (ordinary_final_yaw_align_active_) {
    cmd_owner_conflict_detected_ = true;
    return false;
  }
  predock_yaw_align_active_ = true;
  predock_lateral_align_active_ = lateral;
  return true;
}

void NavigationGoalExecutionModule::release_predock_motion_owner()
{
  std::lock_guard<std::mutex> lock(motion_owner_mutex_);
  predock_yaw_align_active_ = false;
  predock_lateral_align_active_ = false;
}

bool NavigationGoalExecutionModule::ordinary_final_yaw_align_active() const
{
  std::lock_guard<std::mutex> lock(motion_owner_mutex_);
  return ordinary_final_yaw_align_active_;
}

void NavigationGoalExecutionModule::record_docking_cmd_owner_conflict()
{
  std::lock_guard<std::mutex> lock(motion_owner_mutex_);
  cmd_owner_conflict_detected_ = true;
}

bool NavigationGoalExecutionModule::request_goal_cancel(const std::string & reason)
{
  return ports_.request_goal_cancel(reason);
}

void NavigationGoalExecutionModule::clear_navigation_terminal_speed_limit()
{
  terminal_runtime_.clear_speed_limit();
}

void NavigationGoalExecutionModule::publish_navigation_terminal_speed_limit_for_goal(
  const StoredPose & target)
{
  terminal_runtime_.publish_speed_limit_for_goal(target);
}

void NavigationGoalExecutionModule::clear_navigation_terminal_reverse_permit(
  bool & permit_active,
  const std::string & context)
{
  terminal_runtime_.clear_reverse_permit(permit_active, context);
}

void NavigationGoalExecutionModule::update_navigation_terminal_reverse_permit_for_goal(
  const StoredPose & target,
  bool & permit_active,
  std::chrono::steady_clock::time_point & next_refresh_at,
  const std::string & context)
{
  terminal_runtime_.update_reverse_permit_for_goal(
    target, permit_active, next_refresh_at, context);
}

bool NavigationGoalExecutionModule::navigation_goal_cancel_requested(
  const std::uint64_t job_id,
  std::string & detail)
{
  return mission_runtime_.cancel_requested(job_id, detail);
}

bool NavigationGoalExecutionModule::floor_runtime_operation_blocked(
  const std::string & operation,
  std::string & detail,
  std::string * reason_code) const
{
  return ports_.floor_runtime_operation_blocked(operation, detail, reason_code);
}

bool NavigationGoalExecutionModule::undock_before_navigation_if_needed(
  const NavigationPreGoalDockSnapshot & dock_check,
  std::string & detail,
  bool & undock_performed)
{
  return ports_.undock_before_navigation(dock_check, detail, undock_performed);
}

bool NavigationGoalExecutionModule::bridge_safe_for_goal_start(
  const std::string & context,
  std::string & detail) const
{
  return ports_.bridge_safe_for_goal_start(context, detail);
}

bool NavigationGoalExecutionModule::cancel_active_navigation_goal(std::string & detail)
{
  return ports_.cancel_active_goal(detail);
}

void NavigationGoalExecutionModule::publish_final_yaw_align_zero_burst()
{
  terminal_runtime_.publish_zero_burst();
}

void NavigationGoalExecutionModule::finish_navigation_goal_job(
  const std::uint64_t job_id,
  const bool succeeded,
  const std::string & phase,
  const std::string & detail,
  const double final_distance,
  const double final_yaw_error,
  const int nav2_result_code,
  const bool nav2_succeeded,
  const bool position_reached,
  const bool final_yaw_align_requested,
  const bool final_yaw_align_succeeded,
  const bool final_yaw_align_blocked)
{
  NavigationGoalJobFinishSpec finish;
  finish.succeeded = succeeded;
  finish.phase = phase;
  finish.detail = detail;
  finish.completed_at = ports_.timestamp_now();
  finish.final_distance_m = final_distance;
  finish.final_yaw_error_rad = final_yaw_error;
  finish.nav2_result_code = nav2_result_code;
  finish.nav2_succeeded = nav2_succeeded;
  finish.position_reached = position_reached;
  finish.final_yaw_align_requested = final_yaw_align_requested;
  finish.final_yaw_align_succeeded = final_yaw_align_succeeded;
  finish.final_yaw_align_blocked = final_yaw_align_blocked;
  if (!mission_runtime_.finish(job_id, finish)) {
    return;
  }
  action_runtime_.clear_tracked_goal_if_terminal_known();
  if (!ports_.navigation_runtime_active()) {
    return;
  }
  if (succeeded || phase == "canceled") {
    ports_.set_navigation_runtime_state(true, "ready", detail, true);
  } else if (phase.rfind("degraded", 0) == 0) {
    ports_.set_navigation_runtime_state(true, "degraded", detail, true);
  } else {
    ports_.set_navigation_runtime_state(true, "failed", detail, false);
  }
}

FinalPoseCheck NavigationGoalExecutionModule::verify_navigation_final_pose(
  const StoredPose & target,
  const bool require_fresh_pose)
{
  std::string pose_error;
  const auto pose = ports_.robot_pose_snapshot(require_fresh_pose, pose_error);
  return completion_policy_.evaluate_final_pose(
    pose, target, require_fresh_pose, pose_error);
}

void NavigationGoalExecutionModule::update_navigation_goal_final_pose_fields(
  const std::uint64_t job_id,
  const FinalPoseCheck & check,
  const std::string & reason)
{
  (void)mission_runtime_.update(
    job_id,
    [&check, &reason](NavigationGoalJob & job) {
      apply_navigation_goal_final_pose(
        job,
        check.distance_m,
        check.yaw_error_rad,
        check.position_reached,
        reason.empty() ? check.reason : reason);
    });
}

bool NavigationGoalExecutionModule::post_nav2_terminal_lateral_correction_allowed(
  const StoredPose & target,
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy,
  double & forward_m,
  double & lateral_m,
  std::string & reason) const
{
  const auto gate = terminal_control_.lateral_correction_gate(
    target, check, goal_completion_policy);
  forward_m = gate.forward_m;
  lateral_m = gate.lateral_m;
  reason = gate.reason;
  return gate.allowed;
}

TerminalLateralCorrectionResult
NavigationGoalExecutionModule::run_post_nav2_terminal_lateral_correction(
  const std::uint64_t job_id,
  const StoredPose & target,
  const FinalPoseCheck & initial_check,
  const std::string & goal_completion_policy)
{
  return terminal_control_.run_lateral_correction(
    job_id, target, initial_check, goal_completion_policy, terminal_runtime_);
}

bool NavigationGoalExecutionModule::maybe_navigation_near_goal_stalled_handoff(
  const std::uint64_t job_id,
  const StoredPose & target,
  const GoalHandle::SharedPtr & goal_handle,
  const std::chrono::steady_clock::time_point wait_started_at,
  std::chrono::steady_clock::time_point & next_check_at,
  std::optional<double> & best_distance_m,
  std::chrono::steady_clock::time_point & last_progress_at,
  const std::string & observing_phase,
  std::string & detail)
{
  detail.clear();
  if (!config_.near_goal_stalled_handoff_enabled || !config_.final_verify_enabled) {
    return false;
  }

  const auto now_steady = std::chrono::steady_clock::now();
  if (now_steady < next_check_at) {
    return false;
  }
  next_check_at = now_steady + 500ms;

  auto check = verify_navigation_final_pose(target, true);
  update_navigation_goal_final_pose_fields(job_id, check, check.reason);
  if (!check.pose_available) {
    best_distance_m.reset();
    last_progress_at = now_steady;
    return false;
  }

  const auto mission = mission_runtime_.snapshot(job_id);
  if (!mission) {
    return false;
  }
  const auto & goal_completion_policy = mission->goal_completion_policy;

  double forward_m = 0.0;
  double lateral_m = 0.0;
  std::string recovery_gate_reason;
  if (!post_nav2_terminal_lateral_correction_allowed(
      target,
      check,
      goal_completion_policy,
      forward_m,
      lateral_m,
      recovery_gate_reason))
  {
    best_distance_m.reset();
    last_progress_at = now_steady;
    return false;
  }

  if (!best_distance_m ||
    check.distance_m < (*best_distance_m - config_.near_goal_stalled_handoff_improvement_epsilon_m))
  {
    best_distance_m = check.distance_m;
    last_progress_at = now_steady;
  }

  const double wait_sec = std::chrono::duration<double>(now_steady - wait_started_at).count();
  const double stalled_sec = std::chrono::duration<double>(now_steady - last_progress_at).count();

  std::ostringstream observe_detail;
  observe_detail << std::fixed << std::setprecision(3)
                 << "near-goal Nav2 handoff watch"
                 << " distance=" << check.distance_m
                 << " best_distance=" << (best_distance_m ? *best_distance_m : -1.0)
                 << " yaw_error=" << check.yaw_error_rad
                 << " forward_error=" << forward_m
                 << " lateral_error=" << lateral_m
                 << " recovery_distance=" << config_.terminal_recovery_max_distance_m
                 << " recovery_max_forward=" << config_.terminal_lateral_max_forward_m
                 << " wait_sec=" << wait_sec
                 << " min_wait_sec=" << config_.near_goal_stalled_handoff_min_wait_sec
                 << " stalled_sec=" << stalled_sec
                 << " stall_sec=" << config_.near_goal_stalled_handoff_stall_sec
                 << " improvement_epsilon="
                 << config_.near_goal_stalled_handoff_improvement_epsilon_m
                 << "; handoff requires executable terminal recovery";

  if (wait_sec < config_.near_goal_stalled_handoff_min_wait_sec ||
    stalled_sec < config_.near_goal_stalled_handoff_stall_sec)
  {
    const auto observation = observe_detail.str();
    (void)mission_runtime_.update(
      job_id,
      [&observing_phase, &observation](NavigationGoalJob & job) {
        job.phase = observing_phase;
        job.detail = observation;
      });
    return false;
  }

  std::string cancel_detail;
  const bool cancel_requested = ports_.cancel_goal_for_handoff(goal_handle, cancel_detail);
  if (!cancel_requested) {
    const auto failure_detail = observe_detail.str() + "; handoff cancel failed: " + cancel_detail;
    (void)mission_runtime_.update(
      job_id,
      [&observing_phase, &failure_detail](NavigationGoalJob & job) {
        job.phase = observing_phase;
        job.detail = failure_detail;
      });
    return false;
  }

  terminal_runtime_.publish_zero_burst();
  detail = observe_detail.str() + "; " + cancel_detail;
  (void)mission_runtime_.update(
    job_id,
    [&detail](NavigationGoalJob & job) {
      job.phase = "near_goal_nav2_stalled_handoff";
      job.detail =
      "Nav2 still executing near target; canceled Nav2 for API final verification: " + detail;
    });
  return true;
}

void NavigationGoalExecutionModule::update_post_nav2_bridge_wait_fields(
  const std::uint64_t job_id,
  const NavigationBridgeWaitResult & result)
{
  (void)mission_runtime_.update(
    job_id,
    [&result](NavigationGoalJob & job) {
      apply_navigation_goal_bridge_wait(job, result.elapsed_ms, result.timeout, result.detail);
    });
}

NavigationBridgeWaitRuntimePort::TimePoint NavigationGoalExecutionModule::bridge_wait_now() const
{
  return std::chrono::steady_clock::now();
}

void NavigationGoalExecutionModule::bridge_wait_sleep_for(
  const std::chrono::milliseconds duration)
{
  std::this_thread::sleep_for(duration);
}

bool NavigationGoalExecutionModule::bridge_wait_cancel_requested(
  const std::uint64_t job_id,
  std::string & detail)
{
  return navigation_goal_cancel_requested(job_id, detail);
}

void NavigationGoalExecutionModule::publish_bridge_wait_zero()
{
  geometry_msgs::msg::Twist zero;
  terminal_runtime_.publish_command(zero);
}

BridgeReadinessSnapshot NavigationGoalExecutionModule::bridge_wait_snapshot()
{
  return ports_.bridge_readiness_snapshot();
}

bool NavigationGoalExecutionModule::request_amcl_nomotion_update_for_final_verify(
  const BridgeReadinessSnapshot & bridge,
  std::string & detail)
{
  if (!config_.final_verify_request_amcl_nomotion_update) {
    detail = "AMCL no-motion update before final verify disabled by config";
    return true;
  }
  if (!bridge.available || !bridge.amcl_input_enabled) {
    detail =
      "AMCL no-motion update before final verify skipped: AMCL input disabled or bridge unavailable";
    return true;
  }
  if (!bridge.amcl_process_ready || !bridge.amcl_seeded || !bridge.amcl_tracking_ready) {
    std::ostringstream out;
    out << "AMCL no-motion update before final verify skipped: process_ready="
        << (bridge.amcl_process_ready ? "true" : "false")
        << " seeded=" << (bridge.amcl_seeded ? "true" : "false")
        << " tracking_ready=" << (bridge.amcl_tracking_ready ? "true" : "false");
    detail = out.str();
    return true;
  }
  if (bridge.amcl_correction_ready && !bridge.amcl_correction_pending) {
    detail = "AMCL no-motion update before final verify skipped: correction already ready";
    return true;
  }
  return ports_.request_amcl_nomotion_update("final_verify", detail, config_.service_timeout);
}

void NavigationGoalExecutionModule::request_bridge_wait_nomotion_update(
  const BridgeReadinessSnapshot & bridge,
  std::string & detail)
{
  if (!request_amcl_nomotion_update_for_final_verify(bridge, detail)) {
    ports_.warn("AMCL no-motion update before final pose verify did not complete: " + detail);
  }
}

NavigationBridgeWaitResult
NavigationGoalExecutionModule::wait_for_bridge_smoothing_before_final_verify(
  const std::uint64_t job_id)
{
  if (config_.final_verify_enabled && config_.final_verify_wait_bridge_smoothing) {
    (void)mission_runtime_.update(
      job_id,
      [](NavigationGoalJob & job) {
        if (job.state == "running") {
          job.phase = "post_nav2_final_verify_wait_bridge_smoothing";
          job.detail = "waiting for bridge map->odom smoothing before final pose verify";
        }
      });
  }
  const auto result = bridge_wait_.wait_before_final_verify(job_id, *this);
  update_post_nav2_bridge_wait_fields(job_id, result);
  return result;
}

NavigationBridgeWaitResult
NavigationGoalExecutionModule::wait_for_bridge_smoothing_before_final_yaw_align(
  const std::uint64_t job_id)
{
  if (config_.final_yaw_wait_bridge_smoothing) {
    (void)mission_runtime_.update(
      job_id,
      [](NavigationGoalJob & job) {
        if (job.state == "running") {
          job.phase = "final_yaw_align_wait_bridge_smoothing";
          job.detail = "waiting for bridge map->odom smoothing before final yaw alignment";
        }
      });
  }
  return bridge_wait_.wait_before_final_yaw_align(job_id, *this);
}

bool NavigationGoalExecutionModule::post_nav2_final_verify_retry_allowed(
  const std::uint64_t job_id,
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy,
  std::string & retry_reason,
  std::string & retry_phase)
{
  retry_reason.clear();
  retry_phase.clear();
  const auto mission = mission_runtime_.snapshot(job_id);
  if (!mission) {
    return false;
  }
  const auto decision = completion_policy_.final_verify_retry(
    check, goal_completion_policy, mission->final_verify_retry_count);
  retry_reason = decision.reason;
  retry_phase = decision.phase;
  return decision.allowed;
}

bool NavigationGoalExecutionModule::post_nav2_final_verify_acceptance_slack_allowed(
  const std::uint64_t job_id,
  const FinalPoseCheck & check,
  std::string & detail)
{
  detail.clear();
  const auto mission = mission_runtime_.snapshot(job_id);
  if (!mission) {
    return false;
  }
  const auto decision = completion_policy_.final_verify_acceptance_slack(
    check, mission->final_verify_retry_count);
  detail = decision.detail;
  return decision.allowed;
}

bool NavigationGoalExecutionModule::nav2_failed_near_goal_retry_allowed(
  const std::uint64_t job_id,
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy,
  const double yaw_align_xy_gate,
  std::string & retry_reason,
  std::string & retry_phase)
{
  retry_reason.clear();
  retry_phase.clear();
  const auto mission = mission_runtime_.snapshot(job_id);
  if (!mission) {
    return false;
  }
  const auto decision = completion_policy_.nav2_failed_near_goal_retry(
    check, goal_completion_policy, yaw_align_xy_gate, mission->final_verify_retry_count);
  retry_reason = decision.reason;
  retry_phase = decision.phase;
  return decision.allowed;
}

NavigationRepositionResult NavigationGoalExecutionModule::run_post_nav2_final_verify_retry(
  const std::uint64_t job_id,
  const StoredPose & target,
  const std::string & retry_reason,
  const std::string & retry_phase)
{
  NavigationRepositionResult result;
  result.attempted = true;
  terminal_runtime_.clear_speed_limit();
  (void)mission_runtime_.update(
    job_id,
    [&retry_phase, &retry_reason](NavigationGoalJob & job) {
      job.phase = retry_phase;
      job.detail = "retrying same Nav2 goal after final pose verify " + retry_reason;
      job.final_verify_retry_reason = retry_reason;
      job.final_verify_retry_goal_sent = false;
      ++job.final_verify_retry_count;
    });

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = ports_.goal_stamp();
  goal.pose.pose.position.x = target.x;
  goal.pose.pose.position.y = target.y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

  GoalHandle::SharedPtr goal_handle;
  try {
    std::lock_guard<std::mutex> action_lock(action_runtime_.mutex());
    if (!action_runtime_.wait_for_action_server(config_.service_timeout)) {
      result.detail =
        "action unavailable during post-Nav2 final verify retry: " + config_.action_name;
      return result;
    }
    PendingSideEffectEvidence pending_side_effect(ports_);
    auto future = action_runtime_.client()->async_send_goal(goal);
    if (future.wait_for(config_.service_timeout) != std::future_status::ready) {
      result.detail = "timed out sending post-Nav2 final verify retry goal";
      return result;
    }
    goal_handle = future.get();
    pending_side_effect.resolve();
  } catch (const std::exception & exc) {
    result.detail = std::string("exception sending post-Nav2 final verify retry goal: ") +
      exc.what();
    return result;
  } catch (...) {
    result.detail = "unknown exception sending post-Nav2 final verify retry goal";
    return result;
  }

  if (!goal_handle) {
    result.detail = "post-Nav2 final verify retry goal was rejected by Nav2";
    return result;
  }

  bool terminal_reverse_permit_active = false;
  auto next_terminal_reverse_permit_refresh = std::chrono::steady_clock::now();
  ScopeExit terminal_reverse_permit_cleanup([this, &terminal_reverse_permit_active]() {
      terminal_runtime_.clear_reverse_permit(
        terminal_reverse_permit_active,
        "post_nav2_final_verify_retry_scope_exit");
    });

  (void)mission_runtime_.update(
    job_id,
    [&retry_reason](NavigationGoalJob & job) {
      job.final_verify_retry_goal_sent = true;
      job.detail = "same Nav2 goal resent after final pose verify " + retry_reason;
    });
  action_runtime_.track_goal(goal_handle, target.id, "", "");

  terminal_runtime_.update_reverse_permit_for_goal(
    target,
    terminal_reverse_permit_active,
    next_terminal_reverse_permit_refresh,
    "post_nav2_final_verify_retry");
  auto result_future = action_runtime_.client()->async_get_result(goal_handle);
  const auto result_wait_started = std::chrono::steady_clock::now();
  auto next_handoff_check = result_wait_started;
  auto last_near_goal_progress = result_wait_started;
  std::optional<double> best_near_goal_distance;
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.goal_result_timeout_sec));
  terminal_runtime_.publish_speed_limit_for_goal(target);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    terminal_runtime_.publish_speed_limit_for_goal(target);
    terminal_runtime_.update_reverse_permit_for_goal(
      target,
      terminal_reverse_permit_active,
      next_terminal_reverse_permit_refresh,
      "post_nav2_final_verify_retry");
    std::string cancel_detail;
    if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
      std::string action_cancel_detail;
      cancel_active_navigation_goal(action_cancel_detail);
      terminal_runtime_.clear_reverse_permit(
        terminal_reverse_permit_active,
        "post_nav2_final_verify_retry_canceled");
      terminal_runtime_.clear_speed_limit();
      result.canceled = true;
      result.detail = "post-Nav2 final verify retry canceled: " +
        cancel_detail + "; " + action_cancel_detail;
      return result;
    }
    std::string handoff_detail;
    if (maybe_navigation_near_goal_stalled_handoff(
        job_id,
        target,
        goal_handle,
        result_wait_started,
        next_handoff_check,
        best_near_goal_distance,
        last_near_goal_progress,
        retry_phase + "_near_goal_watch",
        handoff_detail))
    {
      terminal_runtime_.clear_reverse_permit(
        terminal_reverse_permit_active,
        "post_nav2_final_verify_retry_handoff");
      terminal_runtime_.clear_speed_limit();
      result.near_goal_stalled_handoff = true;
      result.nav2_result_code = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
      result.detail =
        "post-Nav2 final verify retry handed off near goal before Nav2 result; " +
        handoff_detail;
      return result;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      std::string action_cancel_detail;
      cancel_active_navigation_goal(action_cancel_detail);
      terminal_runtime_.clear_reverse_permit(
        terminal_reverse_permit_active,
        "post_nav2_final_verify_retry_timeout");
      terminal_runtime_.clear_speed_limit();
      result.detail =
        "timed out waiting for post-Nav2 final verify retry result; " + action_cancel_detail;
      return result;
    }
  }
  terminal_runtime_.clear_reverse_permit(
    terminal_reverse_permit_active,
    "post_nav2_final_verify_retry_result");
  terminal_runtime_.clear_speed_limit();

  const auto action_result = result_future.get();
  action_runtime_.mark_terminal_proven(goal_handle, true);
  result.nav2_result_code = static_cast<int>(action_result.code);
  result.nav2_succeeded = action_result.code == rclcpp_action::ResultCode::SUCCEEDED;
  result.succeeded = result.nav2_succeeded;
  std::ostringstream detail;
  detail << "post-Nav2 final verify retry result code " << result.nav2_result_code
         << " reason=" << retry_reason;
  result.detail = detail.str();
  return result;
}

void NavigationGoalExecutionModule::update_navigation_goal_final_yaw_fields(
  const std::uint64_t job_id,
  const FinalYawAlignResult & result)
{
  NavigationGoalFinalYawUpdate update;
  update.attempted = result.attempted;
  update.succeeded = result.succeeded;
  update.blocked = result.blocked;
  update.blocked_reason = result.blocked_reason;
  update.duration_sec = result.duration_sec;
  update.initial_yaw_error_rad = result.initial_yaw_error_rad;
  update.final_yaw_error_rad = result.final_yaw_error_rad;
  update.observed_xy_drift_m = result.observed_xy_drift_m;
  (void)mission_runtime_.update(
    job_id,
    [&update](NavigationGoalJob & job) {
      apply_navigation_goal_final_yaw(job, update);
    });
}

FinalYawAlignResult NavigationGoalExecutionModule::run_final_yaw_align(
  const std::uint64_t job_id,
  const StoredPose & target,
  const FinalPoseCheck & initial_check)
{
  FinalYawAlignResult result;
  result.initial_yaw_error_rad = initial_check.yaw_error_rad;
  result.final_yaw_error_rad = initial_check.yaw_error_rad;
  result.attempted = true;

  if (ports_.docking_job_running()) {
    result.blocked = true;
    result.phase = "blocked_by_docking";
    result.blocked_reason = "docking_phase_active";
    result.detail = "ordinary final yaw alignment blocked because docking phase is active";
    (void)mission_runtime_.update(
      job_id,
      [&result](NavigationGoalJob & job) {
        job.cmd_owner_conflict_detected = true;
        job.final_yaw_align_blocked_by_docking = true;
        job.final_yaw_align_blocked_reason = result.blocked_reason;
        job.yaw_align_active = false;
        job.ordinary_final_yaw_align_active = false;
      });
    terminal_runtime_.publish_zero_burst();
    return result;
  }

  std::string dock_contact_detail;
  if (ports_.dock_contact_blocked(dock_contact_detail)) {
    result.blocked = true;
    result.phase = "blocked_by_docked_contact";
    result.blocked_reason = "DOCKED_OR_CHARGING_CONTACT";
    result.detail = "final yaw alignment blocked by dock/contact gate: " + dock_contact_detail;
    (void)mission_runtime_.update(
      job_id,
      [&result](NavigationGoalJob & job) {
        job.final_yaw_align_blocked_by_docking = true;
        job.final_yaw_align_blocked_reason = result.blocked_reason;
        job.yaw_align_active = false;
        job.ordinary_final_yaw_align_active = false;
      });
    terminal_runtime_.publish_zero_burst();
    return result;
  }

  {
    std::lock_guard<std::mutex> owner_lock(motion_owner_mutex_);
    if (predock_yaw_align_active_) {
      cmd_owner_conflict_detected_ = true;
      result.blocked = true;
      result.phase = "blocked_by_docking";
      result.blocked_reason = "predock_yaw_align_active";
      result.detail =
        "ordinary final yaw alignment blocked because PREDOCK_YAW_ALIGN owns /cmd_vel_docking";
      (void)mission_runtime_.update(
        job_id,
        [&result](NavigationGoalJob & job) {
          job.cmd_owner_conflict_detected = true;
          job.final_yaw_align_blocked_by_docking = true;
          job.final_yaw_align_blocked_reason = result.blocked_reason;
          job.yaw_align_active = false;
          job.ordinary_final_yaw_align_active = false;
          job.predock_yaw_align_active = true;
        });
      terminal_runtime_.publish_zero_burst();
      return result;
    }
    ordinary_final_yaw_align_active_ = true;
  }
  (void)mission_runtime_.update(
    job_id,
    [](NavigationGoalJob & job) {
      job.yaw_align_active = true;
      job.ordinary_final_yaw_align_active = true;
    });
  auto release_final_yaw_owner = [&]() {
      {
        std::lock_guard<std::mutex> owner_lock(motion_owner_mutex_);
        ordinary_final_yaw_align_active_ = false;
      }
      (void)mission_runtime_.update(
        job_id,
        [](NavigationGoalJob & job) {
          job.yaw_align_active = false;
          job.ordinary_final_yaw_align_active = false;
        });
    };

  const auto bridge_wait = wait_for_bridge_smoothing_before_final_yaw_align(job_id);
  if (bridge_wait.canceled || bridge_wait.timeout) {
    result.blocked = true;
    result.canceled = bridge_wait.canceled;
    result.phase = bridge_wait.canceled ? "canceled" : "failed_final_yaw_align";
    result.blocked_reason = bridge_wait.canceled ?
      "canceled" : "bridge_smoothing_active_before_final_yaw";
    result.detail = bridge_wait.detail;
    terminal_runtime_.publish_zero_burst();
    release_final_yaw_owner();
    return result;
  }

  bool correction_pause_active = false;
  if (config_.pause_global_correction_during_final_yaw) {
    std::string pause_detail;
    if (!ports_.request_bridge_correction_pause(
        true,
        pause_detail,
        config_.service_timeout,
        config_.pause_global_correction_during_final_yaw))
    {
      result.blocked = true;
      result.phase = "failed_final_yaw_align";
      result.blocked_reason = "global_correction_pause_failed";
      result.detail =
        "final yaw alignment refused because global correction pause failed: " + pause_detail;
      terminal_runtime_.publish_zero_burst();
      release_final_yaw_owner();
      (void)mission_runtime_.update(
        job_id,
        [&result](NavigationGoalJob & job) {
          job.final_yaw_align_blocked_reason = result.blocked_reason;
        });
      return result;
    }
    correction_pause_active = true;
    result.detail = "global correction paused for final_yaw_align: " + pause_detail;
  }

  result = terminal_control_.run_final_yaw_motion(
    job_id, target, initial_check, terminal_runtime_);
  if (correction_pause_active) {
    std::string release_detail;
    const bool release_ok = ports_.request_bridge_correction_pause(
      false,
      release_detail,
      config_.service_timeout,
      config_.pause_global_correction_during_final_yaw);
    if (!release_ok) {
      if (!result.detail.empty()) {
        result.detail += "; ";
      }
      result.detail += "global correction pause release failed: " + release_detail;
    } else if (!release_detail.empty()) {
      if (!result.detail.empty()) {
        result.detail += "; ";
      }
      result.detail += "global correction resumed after final_yaw_align: " + release_detail;
    }
  }
  release_final_yaw_owner();
  return result;
}

NavigationRepositionResult NavigationGoalExecutionModule::run_reposition_after_yaw_drift(
  const std::uint64_t job_id,
  const StoredPose & target)
{
  NavigationRepositionResult result;
  result.attempted = true;
  terminal_runtime_.clear_speed_limit();
  (void)mission_runtime_.update(
    job_id,
    [](NavigationGoalJob & job) {
      job.phase = "REPOSITION_AFTER_YAW_DRIFT";
      job.detail = "repositioning to original XY after final yaw drift exceeded guard";
      ++job.reposition_after_yaw_drift_retry_count;
    });

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = ports_.goal_stamp();
  goal.pose.pose.position.x = target.x;
  goal.pose.pose.position.y = target.y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

  GoalHandle::SharedPtr goal_handle;
  try {
    std::lock_guard<std::mutex> action_lock(action_runtime_.mutex());
    if (!action_runtime_.wait_for_action_server(config_.service_timeout)) {
      result.detail = "action unavailable during reposition: " + config_.action_name;
      return result;
    }
    PendingSideEffectEvidence pending_side_effect(ports_);
    auto future = action_runtime_.client()->async_send_goal(goal);
    if (future.wait_for(config_.service_timeout) != std::future_status::ready) {
      result.detail = "timed out sending reposition goal";
      return result;
    }
    goal_handle = future.get();
    pending_side_effect.resolve();
  } catch (const std::exception & exc) {
    result.detail = std::string("exception sending reposition goal: ") + exc.what();
    return result;
  } catch (...) {
    result.detail = "unknown exception sending reposition goal";
    return result;
  }

  if (!goal_handle) {
    result.detail = "reposition goal was rejected by Nav2";
    return result;
  }

  action_runtime_.track_goal(goal_handle, target.id, "", "");

  bool terminal_reverse_permit_active = false;
  auto next_terminal_reverse_permit_refresh = std::chrono::steady_clock::now();
  ScopeExit terminal_reverse_permit_cleanup([this, &terminal_reverse_permit_active]() {
      terminal_runtime_.clear_reverse_permit(
        terminal_reverse_permit_active,
        "reposition_after_yaw_drift_scope_exit");
    });
  auto result_future = action_runtime_.client()->async_get_result(goal_handle);
  terminal_runtime_.publish_speed_limit_for_goal(target);
  terminal_runtime_.update_reverse_permit_for_goal(
    target,
    terminal_reverse_permit_active,
    next_terminal_reverse_permit_refresh,
    "reposition_after_yaw_drift");
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.reposition_after_yaw_drift_timeout_sec));
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    terminal_runtime_.publish_speed_limit_for_goal(target);
    terminal_runtime_.update_reverse_permit_for_goal(
      target,
      terminal_reverse_permit_active,
      next_terminal_reverse_permit_refresh,
      "reposition_after_yaw_drift");
    std::string cancel_detail;
    if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
      std::string action_cancel_detail;
      cancel_active_navigation_goal(action_cancel_detail);
      terminal_runtime_.clear_speed_limit();
      result.canceled = true;
      result.detail = "reposition canceled: " + cancel_detail + "; " + action_cancel_detail;
      return result;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      std::string action_cancel_detail;
      cancel_active_navigation_goal(action_cancel_detail);
      terminal_runtime_.clear_speed_limit();
      result.detail = "timed out waiting for reposition result; " + action_cancel_detail;
      return result;
    }
  }
  terminal_runtime_.clear_reverse_permit(
    terminal_reverse_permit_active,
    "reposition_after_yaw_drift_result");
  terminal_runtime_.clear_speed_limit();

  const auto action_result = result_future.get();
  action_runtime_.mark_terminal_proven(goal_handle, true);
  result.nav2_result_code = static_cast<int>(action_result.code);
  result.nav2_succeeded = action_result.code == rclcpp_action::ResultCode::SUCCEEDED;
  result.succeeded = result.nav2_succeeded;
  std::ostringstream detail;
  detail << "reposition after yaw drift result code " << result.nav2_result_code;
  result.detail = detail.str();
  return result;
}

void NavigationGoalExecutionModule::finish_navigation_goal_send_failure(
  const std::uint64_t job_id,
  const std::string & phase,
  const std::string & detail)
{
  finish_navigation_goal_job(
    job_id,
    false,
    phase,
    detail,
    -1.0,
    -1.0,
    0,
    false,
    false,
    false,
    false,
    false);
}

bool NavigationGoalExecutionModule::send_initial_navigation_goal_to_nav2(
  const std::uint64_t job_id,
  const nav2_msgs::action::NavigateToPose::Goal & goal,
  const std::string & pose_id,
  const std::string & building_id,
  const std::string & floor_id,
  GoalHandle::SharedPtr & goal_handle)
{
  std::string floor_interlock_detail;
  if (floor_runtime_operation_blocked("navigation_goal_worker", floor_interlock_detail)) {
    finish_navigation_goal_send_failure(
      job_id, "blocked_floor_transition", floor_interlock_detail);
    return false;
  }
  if (!mission_runtime_.update_running(
      job_id,
      [](NavigationGoalJob & job) {
        job.phase = "sending_nav2_goal";
        job.detail = "waiting for Nav2 action server";
      }))
  {
    return false;
  }

  std::string cancel_detail;
  if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
    finish_navigation_goal_send_failure(
      job_id,
      "canceled",
      "navigation goal canceled before Nav2 action send: " + cancel_detail);
    return false;
  }

  try {
    std::lock_guard<std::mutex> action_lock(action_runtime_.mutex());
    if (!action_runtime_.wait_for_action_server(config_.service_timeout)) {
      finish_navigation_goal_send_failure(
        job_id,
        "failed_send_nav2_goal",
        "action unavailable: " + config_.action_name);
      return false;
    }

    if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
      finish_navigation_goal_send_failure(
        job_id,
        "canceled",
        "navigation goal canceled before Nav2 accepted it: " + cancel_detail);
      return false;
    }
    if (floor_runtime_operation_blocked("navigation_goal_submit", floor_interlock_detail)) {
      finish_navigation_goal_send_failure(
        job_id, "blocked_floor_transition", floor_interlock_detail);
      return false;
    }

    (void)mission_runtime_.update_running(
      job_id,
      [](NavigationGoalJob & job) {
        job.detail = "sending NavigateToPose goal to Nav2";
      });

    PendingSideEffectEvidence pending_side_effect(ports_);
    auto future = action_runtime_.client()->async_send_goal(goal);
    const auto deadline = std::chrono::steady_clock::now() + config_.service_timeout;
    while (future.wait_for(100ms) != std::future_status::ready) {
      if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
        finish_navigation_goal_send_failure(
          job_id,
          "canceled",
          "navigation goal canceled before Nav2 accepted it: " + cancel_detail);
        return false;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        finish_navigation_goal_send_failure(
          job_id,
          "failed_send_nav2_goal",
          "timed out sending navigation goal");
        return false;
      }
    }
    goal_handle = future.get();
    pending_side_effect.resolve();
  } catch (const std::exception & exc) {
    finish_navigation_goal_send_failure(
      job_id,
      "failed_send_nav2_goal",
      std::string("exception sending navigation goal: ") + exc.what());
    return false;
  } catch (...) {
    finish_navigation_goal_send_failure(
      job_id,
      "failed_send_nav2_goal",
      "unknown exception sending navigation goal");
    return false;
  }

  if (!goal_handle) {
    finish_navigation_goal_send_failure(
      job_id,
      "failed_send_nav2_goal",
      "navigation goal was rejected by Nav2");
    return false;
  }

  action_runtime_.track_goal(goal_handle, pose_id, building_id, floor_id);
  return true;
}

}  // namespace robot_api_server::features::navigation
