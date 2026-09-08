#include "robot_api_server/features/navigation/mission/navigation_goal_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_api_server/features/navigation/navigation_module.hpp"

namespace robot_api_server::features::navigation
{

using namespace std::chrono_literals;

namespace
{

class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> cleanup)
  : cleanup_(std::move(cleanup))
  {
  }

  ~ScopeExit()
  {
    if (cleanup_) {
      cleanup_();
    }
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;

private:
  std::function<void()> cleanup_;
};

}  // namespace

NavigationGoalExecutor::NavigationGoalExecutor(
  NavigationGoalExecutorConfig config,
  NavigationActionRuntime & action_runtime,
  NavigationMissionRuntime & mission_runtime,
  NavigationGoalExecutionPort & port)
: config_(std::move(config)),
  action_runtime_(action_runtime),
  mission_runtime_(mission_runtime),
  port_(port)
{
}

void NavigationGoalExecutor::finish_send_failure(
  const std::uint64_t job_id,
  const std::string & phase,
  const std::string & detail)
{
  port_.finish_navigation_goal_job(
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

bool NavigationGoalExecutor::wait_for_goal_start_readiness(
  const std::uint64_t job_id, std::string & detail)
{
  const auto deadline = std::chrono::steady_clock::now() + config_.service_timeout;
  std::string last_detail;
  while (std::chrono::steady_clock::now() < deadline) {
    std::string cancel_detail;
    if (port_.navigation_goal_cancel_requested(job_id, cancel_detail)) {
      detail = "navigation goal canceled before goal-start readiness: " + cancel_detail;
      finish_send_failure(job_id, "canceled", detail);
      return false;
    }

    if (port_.bridge_safe_for_goal_start("navigation goal", last_detail)) {
      detail = "normal path relocalization disabled; " + last_detail;
      return true;
    }

    (void)mission_runtime_.update_running(
      job_id,
      [&last_detail](NavigationGoalJob & job) {
        job.phase = "waiting_for_goal_start_readiness";
        job.detail = last_detail;
        job.pre_navigation_relocalization_detail = last_detail;
      });
    std::this_thread::sleep_for(100ms);
  }

  detail = last_detail.empty() ?
    "timed out waiting for goal-start localization readiness" :
    last_detail + "; timed out waiting for goal-start localization readiness";
  finish_send_failure(job_id, "failed_goal_start_readiness", detail);
  return false;
}

bool NavigationGoalExecutor::run_pre_send_sequence(
  const std::uint64_t job_id,
  const NavigationPreGoalDockSnapshot & dock_snapshot)
{
  std::string floor_interlock_detail;
  if (port_.floor_runtime_operation_blocked(
      "navigation_pre_send", floor_interlock_detail))
  {
    finish_send_failure(job_id, "blocked_floor_transition", floor_interlock_detail);
    return false;
  }
  (void)mission_runtime_.update_running(
    job_id,
    [&dock_snapshot](NavigationGoalJob & job) {
      job.phase = dock_snapshot.pre_navigation_recovery_required ?
        "pre_navigation_dock_recovery" : "waiting_for_goal_start_readiness";
      job.detail = dock_snapshot.pre_navigation_recovery_required ?
        "resolving dock safety state before Nav2 goal send" :
        "checking localization readiness before Nav2 goal send";
    });

  bool pre_navigation_undock = false;
  std::string pre_navigation_undock_detail;
  if (!port_.undock_before_navigation_if_needed(
      dock_snapshot,
      pre_navigation_undock_detail,
      pre_navigation_undock))
  {
    const bool undocked_but_not_ready =
      pre_navigation_undock_detail.find("post-undock navigation readiness failed") !=
      std::string::npos;
    const std::string failure_detail =
      std::string(
      undocked_but_not_ready ?
      "navigation requires post-undock localization readiness before goal start: " :
      "navigation requires successful undock first: ") + pre_navigation_undock_detail;
    (void)mission_runtime_.update(
      job_id,
      [pre_navigation_undock, &pre_navigation_undock_detail](NavigationGoalJob & job) {
        job.pre_navigation_undock = pre_navigation_undock;
        job.pre_navigation_undock_detail = pre_navigation_undock_detail;
      });
    finish_send_failure(job_id, "failed_pre_navigation_undock", failure_detail);
    return false;
  }

  (void)mission_runtime_.update_running(
    job_id,
    [pre_navigation_undock, &pre_navigation_undock_detail](NavigationGoalJob & job) {
      job.pre_navigation_undock = pre_navigation_undock;
      job.pre_navigation_undock_detail = pre_navigation_undock_detail;
      job.phase = "waiting_for_goal_start_readiness";
      job.detail = "checking localization readiness before Nav2 goal send";
    });

  std::string readiness_detail;
  if (!wait_for_goal_start_readiness(job_id, readiness_detail)) {
    return false;
  }

  (void)mission_runtime_.update_running(
    job_id,
    [&readiness_detail](NavigationGoalJob & job) {
      job.pre_navigation_relocalization_requested = false;
      job.pre_navigation_relocalization_succeeded = false;
      job.pre_navigation_relocalization_detail = readiness_detail;
      job.phase = "ready_to_send_nav2_goal";
      job.detail = readiness_detail;
    });
  return true;
}

void NavigationGoalExecutor::run_guarded(
  const std::uint64_t job_id,
  const nav2_msgs::action::NavigateToPose::Goal & goal,
  const StoredPose & target,
  const std::string & pose_id,
  const std::string & building_id,
  const std::string & floor_id,
  const NavigationPreGoalDockSnapshot & dock_snapshot)
{
  try {
    port_.clear_navigation_terminal_speed_limit();
    if (!run_pre_send_sequence(job_id, dock_snapshot)) {
      return;
    }
    NavigationActionRuntime::GoalHandle::SharedPtr goal_handle;
    if (!port_.send_initial_navigation_goal_to_nav2(
        job_id,
        goal,
        pose_id,
        building_id,
        floor_id,
        goal_handle))
    {
      return;
    }
    run(job_id, goal_handle, target);
  } catch (const std::exception & exception) {
    port_.clear_navigation_terminal_speed_limit();
    finish_send_failure(
      job_id,
      "exception",
      std::string("navigation goal worker exception: ") + exception.what());
  } catch (...) {
    port_.clear_navigation_terminal_speed_limit();
    finish_send_failure(job_id, "exception", "navigation goal worker unknown exception");
  }
}

void NavigationGoalExecutor::run(
  const std::uint64_t job_id,
  const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
  const StoredPose & target)
{
    (void)mission_runtime_.update(
      job_id,
      [](NavigationGoalJob & job) {
        job.phase = "waiting_for_nav2_result";
        job.detail = "waiting for Nav2 result";
      });

    bool terminal_reverse_permit_active = false;
    auto next_terminal_reverse_permit_refresh = std::chrono::steady_clock::now();
    ScopeExit terminal_reverse_permit_cleanup([this, &terminal_reverse_permit_active]() {
      port_.clear_navigation_terminal_reverse_permit(
        terminal_reverse_permit_active,
        "ordinary_navigation_scope_exit");
    });
    auto result_future = action_runtime_.client()->async_get_result(goal_handle);
    bool near_goal_stalled_handoff = false;
    std::string near_goal_stalled_handoff_detail;
    const auto result_wait_started = std::chrono::steady_clock::now();
    auto next_handoff_check = result_wait_started;
    auto last_near_goal_progress = result_wait_started;
    std::optional<double> best_near_goal_distance;
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.navigation_goal_result_timeout_sec));
    port_.publish_navigation_terminal_speed_limit_for_goal(target);
    port_.update_navigation_terminal_reverse_permit_for_goal(
      target,
      terminal_reverse_permit_active,
      next_terminal_reverse_permit_refresh,
      "ordinary_navigation");
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      port_.publish_navigation_terminal_speed_limit_for_goal(target);
      port_.update_navigation_terminal_reverse_permit_for_goal(
        target,
        terminal_reverse_permit_active,
        next_terminal_reverse_permit_refresh,
        "ordinary_navigation");
      std::string cancel_detail;
      if (port_.navigation_goal_cancel_requested(job_id, cancel_detail)) {
        std::string action_cancel_detail;
        port_.cancel_active_navigation_goal(action_cancel_detail);
        port_.publish_final_yaw_align_zero_burst();
        port_.clear_navigation_terminal_speed_limit();
        port_.finish_navigation_goal_job(
          job_id,
          false,
          "canceled",
          "navigation goal canceled while waiting for Nav2 result: " + cancel_detail + "; " + action_cancel_detail,
          -1.0,
          -1.0,
          0,
          false,
          false,
          false,
          false,
          false);
        return;
      }
      if (port_.maybe_navigation_near_goal_stalled_handoff(
          job_id,
          target,
          goal_handle,
          result_wait_started,
          next_handoff_check,
          best_near_goal_distance,
          last_near_goal_progress,
          "waiting_for_nav2_result_near_goal_watch",
          near_goal_stalled_handoff_detail))
      {
        near_goal_stalled_handoff = true;
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        std::string cancel_detail;
        port_.cancel_active_navigation_goal(cancel_detail);
        port_.clear_navigation_terminal_speed_limit();
        port_.finish_navigation_goal_job(
          job_id,
          false,
          "timeout",
          "timed out waiting for navigation result; " + cancel_detail,
          -1.0,
          -1.0,
          0,
          false,
          false,
          false,
          false,
          false);
        return;
      }
    }
    port_.clear_navigation_terminal_reverse_permit(
      terminal_reverse_permit_active,
      "ordinary_navigation_result");
    port_.clear_navigation_terminal_speed_limit();

    bool nav2_succeeded = false;
    int result_code = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
    rclcpp_action::ResultCode nav2_action_code = rclcpp_action::ResultCode::ABORTED;
    if (!near_goal_stalled_handoff) {
      const auto result = result_future.get();
      action_runtime_.mark_terminal_proven(goal_handle, true);
      nav2_action_code = result.code;
      nav2_succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      result_code = static_cast<int>(result.code);
    }

    std::string cancel_detail;
    if (port_.navigation_goal_cancel_requested(job_id, cancel_detail) ||
      nav2_action_code == rclcpp_action::ResultCode::CANCELED)
    {
      port_.finish_navigation_goal_job(
        job_id,
        false,
        "canceled",
        cancel_detail.empty() ? "navigation goal canceled" : "navigation goal canceled: " + cancel_detail,
        -1.0,
        -1.0,
        result_code,
        nav2_succeeded,
        false,
        false,
        false,
        false);
      return;
    }

    std::string goal_completion_policy = "pose_required";
    if (const auto mission = mission_runtime_.snapshot(job_id)) {
      goal_completion_policy = mission->goal_completion_policy;
    }

    (void)mission_runtime_.update(
      job_id,
      [](NavigationGoalJob & job) {
        job.phase = "final_pose_auditing";
        job.detail = "auditing final map->base_link pose after Nav2 result";
      });

    auto bridge_wait = port_.wait_for_bridge_smoothing_before_final_verify(job_id);
    if (bridge_wait.canceled) {
      port_.finish_navigation_goal_job(
        job_id,
        false,
        "canceled",
        bridge_wait.detail.empty() ?
        "navigation goal canceled during post-Nav2 final pose verify bridge wait" :
        bridge_wait.detail,
        -1.0,
        -1.0,
        result_code,
        nav2_succeeded,
        false,
        false,
        false,
        false);
      return;
    }
    bool final_pose_bridge_ready = !bridge_wait.timeout;
    std::string final_pose_bridge_wait_detail = bridge_wait.detail;

    auto pose_check = port_.verify_navigation_final_pose(target, true);
    double distance = pose_check.distance_m;
    double yaw_error = pose_check.yaw_error_rad;
    bool position_reached = pose_check.position_reached;
    if (near_goal_stalled_handoff) {
      port_.update_navigation_goal_final_pose_fields(
        job_id,
        pose_check,
        pose_check.reason +
          "; near_goal_nav2_stalled_handoff=true " + near_goal_stalled_handoff_detail);
    } else {
      port_.update_navigation_goal_final_pose_fields(job_id, pose_check, pose_check.reason);
    }

    const bool pose_required_goal = goal_completion_policy == "pose_required";
    const bool final_yaw_align_allowed =
      pose_required_goal && config_.api_final_yaw_align_fallback_enabled && config_.navigation_final_yaw_align_enabled;
    const double final_yaw_align_xy_gate =
      std::max(config_.navigation_goal_position_success_tolerance_m, config_.navigation_final_yaw_align_max_xy_drift_m);
    auto yaw_align_candidate = [&]() {
      return final_pose_bridge_ready &&
        final_yaw_align_allowed &&
        pose_check.pose_available &&
        pose_check.distance_m <= final_yaw_align_xy_gate &&
        pose_check.yaw_error_rad > config_.navigation_final_yaw_align_trigger_rad;
    };
    auto refresh_final_pose_check = [&]() {
      pose_check = port_.verify_navigation_final_pose(target, true);
      distance = pose_check.distance_m;
      yaw_error = pose_check.yaw_error_rad;
      position_reached = pose_check.position_reached;
      port_.update_navigation_goal_final_pose_fields(job_id, pose_check, pose_check.reason);
    };
    bool final_yaw_align_requested = false;
    bool final_yaw_align_succeeded = false;
    bool final_yaw_align_blocked = false;
    FinalYawAlignResult final_yaw_result;
    auto nav2_failed_near_goal_yaw_first_candidate = [&]() {
      return config_.navigation_nav2_failed_near_goal_retry_enabled &&
        !nav2_succeeded &&
        final_pose_bridge_ready &&
        final_yaw_align_allowed &&
        pose_check.pose_available &&
        pose_check.distance_m > final_yaw_align_xy_gate &&
        pose_check.distance_m <= config_.navigation_terminal_recovery_max_distance_m &&
        pose_check.yaw_error_rad > config_.navigation_final_yaw_align_trigger_rad;
    };
    auto run_final_yaw_align_attempt = [&](
        const std::string & phase,
        const std::string & detail) {
      final_yaw_align_requested = true;
      (void)mission_runtime_.update(
        job_id,
        [&phase, &detail](NavigationGoalJob & job) {
          job.phase = phase;
          job.detail = detail;
          job.final_yaw_align_requested = true;
          ++job.final_yaw_align_retry_count;
        });
      final_yaw_result = port_.run_final_yaw_align(job_id, target, pose_check);
      port_.update_navigation_goal_final_yaw_fields(job_id, final_yaw_result);
      final_yaw_align_succeeded = final_yaw_result.succeeded;
      final_yaw_align_blocked = final_yaw_result.blocked;
      if (final_yaw_align_succeeded) {
        auto post_yaw_bridge_wait = port_.wait_for_bridge_smoothing_before_final_verify(job_id);
        if (post_yaw_bridge_wait.canceled) {
          port_.finish_navigation_goal_job(
            job_id,
            false,
            "canceled",
            post_yaw_bridge_wait.detail.empty() ?
            "navigation goal canceled during post-final-yaw bridge smoothing wait" :
            post_yaw_bridge_wait.detail,
            distance,
            yaw_error,
            result_code,
            nav2_succeeded,
            position_reached,
            final_yaw_align_requested,
            final_yaw_align_succeeded,
            final_yaw_align_blocked);
          return true;
        }
        final_pose_bridge_ready = !post_yaw_bridge_wait.timeout;
        final_pose_bridge_wait_detail = post_yaw_bridge_wait.detail;
      }
      refresh_final_pose_check();
      std::ostringstream yaw_reason;
      yaw_reason << pose_check.reason
                 << "; final_yaw_align_attempted=true"
                 << " final_yaw_align_succeeded="
                 << (final_yaw_align_succeeded ? "true" : "false")
                 << " final_yaw_align_detail=" << final_yaw_result.detail
                 << " final_pose_bridge_ready="
                 << (final_pose_bridge_ready ? "true" : "false")
                 << " final_pose_bridge_wait_detail=" << final_pose_bridge_wait_detail;
      port_.update_navigation_goal_final_pose_fields(job_id, pose_check, yaw_reason.str());
      if (final_yaw_result.canceled) {
        port_.finish_navigation_goal_job(
          job_id,
          false,
          "canceled",
          final_yaw_result.detail.empty() ?
          "navigation goal canceled during final yaw alignment" :
          final_yaw_result.detail,
          distance,
          yaw_error,
          result_code,
          nav2_succeeded,
          position_reached,
          final_yaw_align_requested,
          final_yaw_align_succeeded,
          final_yaw_align_blocked);
        return true;
      }
      return false;
    };

    double salvage_terminal_forward_m = 0.0;
    double salvage_terminal_lateral_m = 0.0;
    std::string salvage_terminal_lateral_reason;
    const bool terminal_lateral_candidate_before_salvage =
      port_.post_nav2_terminal_lateral_correction_allowed(
      target,
      pose_check,
      goal_completion_policy,
      salvage_terminal_forward_m,
      salvage_terminal_lateral_m,
      salvage_terminal_lateral_reason);

    if (!nav2_succeeded && final_yaw_align_allowed && !yaw_align_candidate() &&
      !terminal_lateral_candidate_before_salvage &&
      !nav2_failed_near_goal_yaw_first_candidate())
    {
      const auto salvage_deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(std::min(8.0, config_.navigation_final_yaw_align_timeout_sec)));
      while (std::chrono::steady_clock::now() < salvage_deadline) {
        std::string yaw_cancel_detail;
        if (port_.navigation_goal_cancel_requested(job_id, yaw_cancel_detail)) {
          break;
        }
        if (yaw_align_candidate()) {
          break;
        }
        (void)mission_runtime_.update(
          job_id,
          [](NavigationGoalJob & job) {
            job.phase = "final_pose_salvage_waiting";
            job.detail =
              "waiting briefly for final pose to enter yaw-only alignment gate";
          });
        std::this_thread::sleep_for(200ms);
        refresh_final_pose_check();
      }
    }

    if (nav2_failed_near_goal_yaw_first_candidate()) {
      if (run_final_yaw_align_attempt(
          "nav2_failed_near_goal_yaw_aligning",
          "Nav2 failed near target but outside yaw-alignable XY; aligning yaw before same-goal retry"))
      {
        return;
      }
    }

    if (!nav2_succeeded && !near_goal_stalled_handoff && !yaw_align_candidate()) {
      std::string retry_reason;
      std::string retry_phase;
      if (port_.nav2_failed_near_goal_retry_allowed(
          job_id,
          pose_check,
          goal_completion_policy,
          final_yaw_align_xy_gate,
          retry_reason,
          retry_phase))
      {
        auto retry_result = port_.run_post_nav2_final_verify_retry(job_id, target, retry_reason, retry_phase);
        if (retry_result.canceled) {
          port_.finish_navigation_goal_job(
            job_id,
            false,
            "canceled",
            retry_result.detail.empty() ?
            "navigation goal canceled during near-goal Nav2 retry" :
            retry_result.detail,
            distance,
            yaw_error,
            retry_result.nav2_result_code,
            false,
            position_reached,
            false,
            false,
            false);
          return;
        }
        if (retry_result.nav2_result_code != 0) {
          result_code = retry_result.nav2_result_code;
        }
        nav2_succeeded = retry_result.nav2_succeeded;
        refresh_final_pose_check();
        std::ostringstream retry_detail;
        retry_detail << pose_check.reason
                     << "; near_goal_nav2_retry_attempted=true"
                     << " near_goal_nav2_retry_succeeded="
                     << (retry_result.succeeded ? "true" : "false")
                     << " near_goal_nav2_retry_detail=" << retry_result.detail;
        port_.update_navigation_goal_final_pose_fields(job_id, pose_check, retry_detail.str());
      }
    }

    if (yaw_align_candidate()) {
      if (run_final_yaw_align_attempt(
          nav2_succeeded ? "position_reached_yaw_aligning" : "nav2_failed_yaw_aligning",
          nav2_succeeded ?
          "Nav2 result requires final yaw alignment fallback" :
          "Nav2 failed after reaching yaw-alignable XY; attempting final yaw alignment fallback"))
      {
        return;
      }
    }

    if (final_yaw_align_requested && !final_yaw_align_succeeded) {
      std::ostringstream yaw_detail;
      yaw_detail << pose_check.reason
                 << "; final_yaw_align_attempted=true"
                 << " final_yaw_align_succeeded=false";
      if (!final_yaw_result.detail.empty()) {
        yaw_detail << " final_yaw_align_detail=" << final_yaw_result.detail;
      }
      port_.update_navigation_goal_final_pose_fields(job_id, pose_check, yaw_detail.str());
    }

    bool audit_position_within_tolerance = false;
    bool audit_yaw_within_tolerance = false;
    bool slack_position_within_tolerance = false;
    bool commercial_position_complete = false;
    bool commercial_yaw_complete = false;
    std::string slack_detail;
    bool terminal_lateral_correction_attempted = false;
    bool terminal_pose_correction_pending = false;
    double terminal_pose_forward_m = 0.0;
    double terminal_pose_lateral_m = 0.0;
    std::string terminal_pose_correction_reason;
    TerminalLateralCorrectionResult terminal_lateral_result;
    auto recompute_commercial_completion = [&]() {
      audit_position_within_tolerance = pose_check.pose_available && pose_check.position_reached;
      audit_yaw_within_tolerance =
        !pose_required_goal ||
        (pose_check.pose_available && yaw_error <= config_.navigation_final_yaw_tolerance_rad);
      slack_detail.clear();
      slack_position_within_tolerance =
        port_.post_nav2_final_verify_acceptance_slack_allowed(job_id, pose_check, slack_detail);
      terminal_pose_correction_pending = port_.post_nav2_terminal_lateral_correction_allowed(
        target,
        pose_check,
        goal_completion_policy,
        terminal_pose_forward_m,
        terminal_pose_lateral_m,
        terminal_pose_correction_reason);
      if (terminal_lateral_correction_attempted) {
        const bool strict_lateral_reached = pose_check.pose_available &&
          std::fabs(terminal_pose_lateral_m) <=
          config_.post_nav2_final_verify_terminal_lateral_target_m;
        commercial_position_complete = final_pose_bridge_ready &&
          terminal_lateral_result.succeeded &&
          audit_position_within_tolerance &&
          strict_lateral_reached;
        commercial_yaw_complete = final_pose_bridge_ready &&
          terminal_lateral_result.succeeded &&
          pose_check.pose_available &&
          yaw_error <= config_.navigation_final_yaw_align_success_tolerance_rad;
      } else {
        commercial_position_complete = final_pose_bridge_ready &&
          (audit_position_within_tolerance || slack_position_within_tolerance) &&
          !terminal_pose_correction_pending;
        commercial_yaw_complete = final_pose_bridge_ready && audit_yaw_within_tolerance &&
          !terminal_pose_correction_pending;
      }
    };
    recompute_commercial_completion();

    while (pose_check.pose_available &&
      (!commercial_position_complete || !commercial_yaw_complete))
    {
      if (!final_pose_bridge_ready) {
        auto retry_bridge_wait = port_.wait_for_bridge_smoothing_before_final_verify(job_id);
        if (retry_bridge_wait.canceled) {
          port_.finish_navigation_goal_job(
            job_id,
            false,
            "canceled",
            retry_bridge_wait.detail.empty() ?
            "navigation goal canceled during final bridge readiness retry" :
            retry_bridge_wait.detail,
            distance,
            yaw_error,
            result_code,
            nav2_succeeded,
            position_reached,
            final_yaw_align_requested,
            final_yaw_align_succeeded,
            final_yaw_align_blocked);
          return;
        }
        final_pose_bridge_ready = !retry_bridge_wait.timeout;
        final_pose_bridge_wait_detail = retry_bridge_wait.detail;
        refresh_final_pose_check();
        std::ostringstream bridge_detail;
        bridge_detail << pose_check.reason
                      << "; final_pose_bridge_ready="
                      << (final_pose_bridge_ready ? "true" : "false")
                      << " final_pose_bridge_wait_detail=" << final_pose_bridge_wait_detail;
        port_.update_navigation_goal_final_pose_fields(job_id, pose_check, bridge_detail.str());
        recompute_commercial_completion();
        if (!final_pose_bridge_ready) {
          break;
        }
        continue;
      }
      if (!terminal_lateral_correction_attempted) {
        terminal_lateral_result = port_.run_post_nav2_terminal_lateral_correction(
          job_id, target, pose_check, goal_completion_policy);
        if (terminal_lateral_result.attempted) {
          terminal_lateral_correction_attempted = true;
          if (terminal_lateral_result.canceled) {
            port_.finish_navigation_goal_job(
              job_id,
              false,
              "canceled",
              terminal_lateral_result.detail.empty() ?
              "navigation goal canceled during terminal lateral correction" :
              terminal_lateral_result.detail,
              distance,
              yaw_error,
              result_code,
              nav2_succeeded,
              position_reached,
              final_yaw_align_requested,
              final_yaw_align_succeeded,
              final_yaw_align_blocked);
            return;
          }

          refresh_final_pose_check();
          std::ostringstream lateral_detail;
          lateral_detail << pose_check.reason
                         << "; terminal_lateral_correction_attempted=true"
                         << " " << NavigationTerminalControl::lateral_correction_diagnostics(terminal_lateral_result);
          port_.update_navigation_goal_final_pose_fields(job_id, pose_check, lateral_detail.str());
          recompute_commercial_completion();
          if (commercial_position_complete && commercial_yaw_complete) {
            break;
          }
        }
      }

      std::string retry_reason;
      std::string retry_phase;
      if (!port_.post_nav2_final_verify_retry_allowed(
          job_id,
          pose_check,
          goal_completion_policy,
          retry_reason,
          retry_phase))
      {
        break;
      }

      auto retry_result = port_.run_post_nav2_final_verify_retry(job_id, target, retry_reason, retry_phase);
      if (retry_result.canceled) {
        port_.finish_navigation_goal_job(
          job_id,
          false,
          "canceled",
          retry_result.detail.empty() ?
          "navigation goal canceled during commercial final verify retry" :
          retry_result.detail,
          distance,
          yaw_error,
          retry_result.nav2_result_code,
          nav2_succeeded,
          position_reached,
          final_yaw_align_requested,
          final_yaw_align_succeeded,
          final_yaw_align_blocked);
        return;
      }
      if (retry_result.nav2_result_code != 0) {
        result_code = retry_result.nav2_result_code;
      }
      nav2_succeeded = nav2_succeeded || retry_result.nav2_succeeded;

      auto retry_bridge_wait = port_.wait_for_bridge_smoothing_before_final_verify(job_id);
      if (retry_bridge_wait.canceled) {
        port_.finish_navigation_goal_job(
          job_id,
          false,
          "canceled",
          retry_bridge_wait.detail.empty() ?
          "navigation goal canceled during post-retry final pose verify bridge wait" :
          retry_bridge_wait.detail,
          distance,
          yaw_error,
          result_code,
          nav2_succeeded,
          position_reached,
          final_yaw_align_requested,
          final_yaw_align_succeeded,
          final_yaw_align_blocked);
        return;
      }
      final_pose_bridge_ready = !retry_bridge_wait.timeout;
      final_pose_bridge_wait_detail = retry_bridge_wait.detail;

      refresh_final_pose_check();
      std::ostringstream retry_detail;
      retry_detail << pose_check.reason
                   << "; commercial_final_verify_retry_attempted=true"
                   << " retry_reason=" << retry_reason
                   << " retry_phase=" << retry_phase
                   << " retry_nav2_succeeded="
                   << (retry_result.nav2_succeeded ? "true" : "false")
                   << " retry_detail=" << retry_result.detail
                   << " final_pose_bridge_ready="
                   << (final_pose_bridge_ready ? "true" : "false")
                   << " final_pose_bridge_wait_detail=" << final_pose_bridge_wait_detail;
      port_.update_navigation_goal_final_pose_fields(job_id, pose_check, retry_detail.str());
      recompute_commercial_completion();
    }

    if (!pose_check.pose_available) {
      std::ostringstream detail;
      detail << "final pose audit unavailable after Nav2 result; " << pose_check.reason
             << "; localization unavailable for commercial completion";
      port_.finish_navigation_goal_job(
        job_id,
        false,
        "failed_final_pose_verify",
        detail.str(),
        distance,
        yaw_error,
        result_code,
        nav2_succeeded,
        false,
        final_yaw_align_requested,
        final_yaw_align_succeeded,
        final_yaw_align_blocked);
      return;
    }

    if (!commercial_position_complete || !commercial_yaw_complete) {
      int retry_count = 0;
      (void)mission_runtime_.update(
        job_id,
        [&retry_count](NavigationGoalJob & job) {
          retry_count = job.final_verify_retry_count;
          job.final_pose_verified = false;
          job.task_complete = false;
          job.final_verify_failure_is_terminal = false;
        });
      std::ostringstream detail;
      detail << "navigation goal degraded after commercial final verification"
             << "; " << pose_check.reason
             << "; final_pose_bridge_ready="
             << (final_pose_bridge_ready ? "true" : "false")
             << "; final_pose_bridge_wait_detail=" << final_pose_bridge_wait_detail
             << "; commercial_position_complete="
             << (commercial_position_complete ? "true" : "false")
             << "; commercial_yaw_complete="
             << (commercial_yaw_complete ? "true" : "false")
             << "; retry_count=" << retry_count
             << "/" << config_.post_nav2_final_verify_max_retry_count;
      if (slack_position_within_tolerance) {
        detail << "; " << slack_detail;
      }
      if (final_yaw_align_requested) {
        detail << "; final_yaw_align_attempted=true"
               << " final_yaw_align_succeeded="
               << (final_yaw_align_succeeded ? "true" : "false")
               << " final_yaw_align_detail=" << final_yaw_result.detail;
      }
      if (terminal_lateral_correction_attempted) {
        detail << "; terminal_lateral_correction_attempted=true"
               << " " << NavigationTerminalControl::lateral_correction_diagnostics(terminal_lateral_result);
      }
      port_.finish_navigation_goal_job(
        job_id,
        false,
        "degraded_final_pose_verify",
        detail.str(),
        distance,
        yaw_error,
        result_code,
        nav2_succeeded,
        commercial_position_complete,
        final_yaw_align_requested,
        final_yaw_align_succeeded,
        final_yaw_align_blocked);
      return;
    }

    std::ostringstream audit_reason;
    audit_reason << (slack_position_within_tolerance ? slack_detail : pose_check.reason)
                 << "; commercial_final_verify=true"
                 << "; final_pose_bridge_ready="
                 << (final_pose_bridge_ready ? "true" : "false")
                 << "; final_pose_bridge_wait_detail=" << final_pose_bridge_wait_detail
                 << "; audit_position_within_tolerance="
                 << (audit_position_within_tolerance ? "true" : "false")
                 << "; slack_position_within_tolerance="
                 << (slack_position_within_tolerance ? "true" : "false")
                 << "; audit_yaw_within_tolerance="
                 << (audit_yaw_within_tolerance ? "true" : "false");
    if (terminal_lateral_correction_attempted) {
      audit_reason << "; terminal_lateral_correction_attempted=true"
                   << " " << NavigationTerminalControl::lateral_correction_diagnostics(terminal_lateral_result);
    }
    const auto final_audit_reason = audit_reason.str();
    (void)mission_runtime_.update(
      job_id,
      [&goal_completion_policy,
      final_yaw_align_requested,
      final_yaw_align_succeeded,
      final_yaw_align_blocked,
      &final_audit_reason](NavigationGoalJob & job) {
        job.position_reached = true;
        job.yaw_align_required = goal_completion_policy == "pose_required";
        job.yaw_align_active = false;
        job.yaw_align_failed = false;
        job.final_yaw_align_requested = final_yaw_align_requested;
        job.final_yaw_align_attempted = final_yaw_align_requested;
        job.final_yaw_align_succeeded = final_yaw_align_succeeded;
        job.final_yaw_align_blocked = final_yaw_align_blocked;
        if (!final_yaw_align_blocked) {
          job.final_yaw_align_blocked_reason.clear();
        }
        job.final_pose_verified = true;
        job.final_pose_verify_reason = final_audit_reason;
        job.final_verify_failure_is_terminal = false;
      });

    std::ostringstream detail;
    detail << "navigation goal reached by commercial final verification";
    if (goal_completion_policy == "position_only") {
      detail << " under position_only policy";
    } else {
      detail << " under pose_required policy";
    }
    detail << "; final pose audit: " << audit_reason.str();
    if (distance >= 0.0) {
      detail << "; final distance=" << distance;
    }
    if (yaw_error >= 0.0) {
      detail << " yaw_error=" << yaw_error;
    }

    port_.finish_navigation_goal_job(
      job_id,
      true,
      "final_pose_verified",
      detail.str(),
      distance,
      yaw_error,
      result_code,
      nav2_succeeded,
      true,
      final_yaw_align_requested,
      final_yaw_align_succeeded,
      final_yaw_align_blocked);
}


}  // namespace robot_api_server::features::navigation
