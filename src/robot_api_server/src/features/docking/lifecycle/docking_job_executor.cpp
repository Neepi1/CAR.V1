#include "robot_api_server/features/docking/lifecycle/docking_job_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <thread>
#include <utility>

#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"

using namespace std::chrono_literals;

namespace robot_api_server::features::docking
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
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

}  // namespace

DockingJobExecutor::DockingJobExecutor(
  DockingJobExecutorConfig config,
  navigation::NavigationActionRuntime & navigation_action_runtime,
  predock_alignment::PredockControlModule & predock_control,
  DockingJobExecutionPort & port)
: config_(std::move(config)),
  navigation_action_runtime_(navigation_action_runtime),
  predock_control_(predock_control),
  port_(port)
{
}

void DockingJobExecutor::run(const std::uint64_t job_id)
  {
    std::string floor_interlock_detail;
    if (port_.floor_runtime_operation_blocked(
        "docking_worker", floor_interlock_detail))
    {
      port_.finish_docking_job(job_id, false, "failed", floor_interlock_detail);
      return;
    }
    DockingJob job;
    {
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id != job_id) {
        return;
      }
      job = port_.docking_job_unsafe();
    }

    if (job.resume_navigation) {
      bool action_available = false;
      {
        std::lock_guard<std::mutex> action_lock(navigation_action_runtime_.mutex());
        action_available = navigation_action_runtime_.wait_for_action_server(500ms);
      }
      if (!action_available) {
        port_.set_docking_job_phase(job_id, "RESOLVE_DOCK_PROFILE");
        std::string resume_detail;
        if (!port_.resume_navigation_runtime_for_docking(job, resume_detail)) {
          port_.finish_docking_job(
            job_id, false, "failed", "failed to start navigation runtime: " + resume_detail);
          return;
        }
      }
    }

    port_.set_docking_job_phase(job_id, "DOCK_REQUESTED");
    const auto action_deadline = std::chrono::steady_clock::now() + port_.docking_navigation_start_timeout();
    bool action_available = false;
    while (std::chrono::steady_clock::now() < action_deadline) {
      if (port_.docking_cancel_requested(job_id)) {
        port_.finish_docking_job(job_id, true, "canceled", "docking canceled before approach navigation");
        return;
      }
      std::lock_guard<std::mutex> action_lock(navigation_action_runtime_.mutex());
      if (navigation_action_runtime_.wait_for_action_server(500ms)) {
        action_available = true;
        break;
      }
    }
    if (!action_available) {
      port_.finish_docking_job(job_id, false, "failed", "action unavailable: " + config_.navigate_to_pose_action);
      return;
    }

    if (config_.cancel_active_goal_before_predock) {
      port_.set_docking_job_phase(job_id, "DOCK_REQUESTED");
      std::string cancel_detail;
      const bool cancel_requested = port_.cancel_active_navigation_goal(cancel_detail);
      bool final_yaw_owner_active = false;
      final_yaw_owner_active = port_.ordinary_final_yaw_align_active();
      if (final_yaw_owner_active) {
        port_.record_docking_cmd_owner_conflict();
      }
      if (final_yaw_owner_active) {
        port_.request_navigation_goal_cancel("docking predock request preempts ordinary final_yaw_align");
        port_.publish_final_yaw_align_zero_burst();
        const auto owner_deadline = std::chrono::steady_clock::now() + 1500ms;
        while (std::chrono::steady_clock::now() < owner_deadline) {
          final_yaw_owner_active = port_.ordinary_final_yaw_align_active();
          if (!final_yaw_owner_active) {
            break;
          }
          std::this_thread::sleep_for(50ms);
        }
      }
      port_.clear_teleop_command();
      port_.publish_teleop_zero_burst();
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().active_navigation_cancel_requested = cancel_requested;
          port_.docking_job_unsafe().active_navigation_cancel_detail = cancel_detail;
          port_.docking_job_unsafe().ordinary_final_yaw_align_active = final_yaw_owner_active;
          port_.docking_job_unsafe().cmd_owner_conflict_detected =
            port_.docking_job_unsafe().cmd_owner_conflict_detected || final_yaw_owner_active;
          port_.docking_job_unsafe().docking_blocked_by_final_yaw_align = final_yaw_owner_active;
          port_.docking_job_unsafe().detail = cancel_detail;
        }
      }
      if (final_yaw_owner_active) {
        port_.finish_docking_job_with_code(
          job_id,
          "DOCK_FAILED_CMD_OWNER_CONFLICT",
          "ordinary final_yaw_align did not release cmd owner before predock navigation");
        return;
      }
    }

    if (config_.relocalize_before_predock) {
      port_.set_docking_job_phase(job_id, "BEFORE_PREDOCK_RELOCALIZE");
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().relocalization_requested = true;
          port_.docking_job_unsafe().relocalization_detail = "waiting for fresh localization_result";
        }
      }
      port_.set_docking_runtime_state(true, "relocalizing", "triggering localization before predock navigation");
      std::string localization_detail;
      std::uint64_t relocalization_sequence = 0U;
      const bool relocalized = port_.trigger_localization_and_wait_for_result(
        "docking_start_before_predock:" + job.dock_id,
        localization_detail,
        -1.0,
        &relocalization_sequence);
      bool settle_ok = true;
      if (relocalized) {
        port_.set_docking_job_phase(job_id, "BEFORE_PREDOCK_SETTLE");
        port_.set_docking_runtime_state(
          true,
          "post_relocalization_settle",
          "settling after relocalization before predock Nav2 goal");
        const auto settle = port_.wait_for_docking_relocalization_settle_barrier(
          relocalization_sequence,
          "before_predock",
          "nav2_goal",
          [this, job_id](std::string & cancel_detail) {
            if (!port_.docking_cancel_requested(job_id)) {
              return false;
            }
            cancel_detail = "CANCELLED_BY_APP: docking canceled during before-predock settle barrier";
            return true;
          });
        settle_ok = settle.ok;
        localization_detail += "; " +
          (settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail);
      }
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().relocalization_succeeded = relocalized && settle_ok;
          port_.docking_job_unsafe().relocalization_detail = localization_detail;
          port_.docking_job_unsafe().detail = localization_detail;
        }
      }
      if (!relocalized || !settle_ok) {
        port_.finish_docking_job_with_code(
          job_id,
          "DOCK_FAILED_PREDOCK_RELOCALIZATION",
          "relocalize/settle before predock failed: " + localization_detail);
        return;
      }
    }

    {
      std::string bridge_goal_start_detail;
      if (!port_.bridge_safe_for_goal_start("docking predock navigation", bridge_goal_start_detail)) {
        port_.finish_docking_job_with_code(
          job_id,
          "DOCK_FAILED_LOCALIZATION_NOT_READY",
          bridge_goal_start_detail);
        return;
      }
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
        if (!port_.docking_job_unsafe().detail.empty()) {
          port_.docking_job_unsafe().detail += "; ";
        }
        port_.docking_job_unsafe().detail += bridge_goal_start_detail;
      }
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = port_.docking_goal_stamp();
    goal.pose.pose.position.x = job.approach_x;
    goal.pose.pose.position.y = job.approach_y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(job.approach_yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(job.approach_yaw * 0.5);
    goal.behavior_tree = config_.predock_behavior_tree;

    StoredPose predock_speed_limit_target;
    predock_speed_limit_target.id =
      job.predock_pose_id.empty() ? job.dock_id + "_predock" : job.predock_pose_id;
    predock_speed_limit_target.name = "docking_predock";
    predock_speed_limit_target.type = "dock_predock";
    predock_speed_limit_target.x = job.approach_x;
    predock_speed_limit_target.y = job.approach_y;
    predock_speed_limit_target.yaw = job.approach_yaw;

    bool nav2_predock_succeeded = false;
    bool nav2_predock_aborted = false;
    bool nav2_predock_early_handoff = false;
    std::string nav2_predock_result_detail;
    std::string nav2_predock_handoff_detail;
    const int predock_nav_max_attempts = std::max(1, job.max_retries + 1);
    for (int predock_nav_attempt = 1; predock_nav_attempt <= predock_nav_max_attempts;
      ++predock_nav_attempt)
    {
      DockingJobExecutionPort::GoalHandle::SharedPtr goal_handle;
      port_.set_docking_job_phase(job_id, "NAV_TO_STAGING_NATIVE_NAV2");
      std::string send_detail;
      if (!port_.send_predock_navigation_goal(goal, goal_handle, send_detail)) {
        port_.finish_docking_job(job_id, false, "failed", send_detail);
        return;
      }
      if (!goal_handle) {
        port_.finish_docking_job(job_id, false, "failed", "predock navigation goal rejected by Nav2");
        return;
      }
      port_.mark_docking_nav_goal_sent(job_id, goal_handle, job.building_id, job.floor_id);
      port_.set_navigation_runtime_state(true, "navigating", "docking predock navigation accepted");
      port_.set_docking_runtime_state(
        true,
        "NAV_TO_STAGING_NATIVE_NAV2",
        "navigating to docking approach pose with native Nav2 coarse approach; "
        "API will own final yaw/lateral handoff; attempt " +
          std::to_string(predock_nav_attempt) + "/" + std::to_string(predock_nav_max_attempts));

      bool terminal_reverse_permit_active = false;
      auto next_terminal_reverse_permit_refresh = std::chrono::steady_clock::now();
      ScopeExit terminal_reverse_permit_cleanup([this, &terminal_reverse_permit_active]() {
        port_.clear_navigation_terminal_reverse_permit(
          terminal_reverse_permit_active,
          "predock_navigation_scope_exit");
      });
      auto result_future = navigation_action_runtime_.client()->async_get_result(goal_handle);
      port_.set_docking_job_phase(job_id, "NAV_TO_STAGING_NATIVE_NAV2");
      const auto predock_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_.predock_nav_timeout_sec));
      auto next_handoff_check = std::chrono::steady_clock::now() + 500ms;
      port_.publish_navigation_terminal_speed_limit_for_goal(predock_speed_limit_target);
      port_.update_navigation_terminal_reverse_permit_for_goal(
        predock_speed_limit_target,
        terminal_reverse_permit_active,
        next_terminal_reverse_permit_refresh,
        "predock_navigation");
      while (result_future.wait_for(200ms) != std::future_status::ready) {
        port_.publish_navigation_terminal_speed_limit_for_goal(predock_speed_limit_target);
        port_.update_navigation_terminal_reverse_permit_for_goal(
          predock_speed_limit_target,
          terminal_reverse_permit_active,
          next_terminal_reverse_permit_refresh,
          "predock_navigation");
        const auto predock_contact = port_.bms_charging_contact_snapshot();
        if (predock_contact.have_state && predock_contact.fresh && predock_contact.contact_stable) {
          port_.clear_navigation_terminal_reverse_permit(
            terminal_reverse_permit_active,
            "predock_navigation_bms_contact");
          const auto contact_stop_started = std::chrono::steady_clock::now();
          const auto contact_detected_at = utc_timestamp_iso8601();
          port_.set_docking_job_phase(job_id, "PREDOCK_CONTACT_STOP");
          port_.set_docking_runtime_state(
            true,
            "PREDOCK_CONTACT_STOP",
            "stable BMS charging contact detected during predock Nav2; canceling navigation and holding zero");

          port_.reset_terminal_actual_stop_stability();
          std::string cancel_detail;
          const bool cancel_requested = port_.cancel_active_navigation_goal(cancel_detail);
          port_.clear_navigation_terminal_speed_limit();
          port_.clear_teleop_command();
          port_.publish_teleop_zero_burst();

          std::string stop_detail;
          const bool actual_stop_confirmed = port_.wait_for_terminal_actual_stop(
            "predock BMS contact stop",
            stop_detail,
            false);
          port_.publish_teleop_zero_burst();
          const auto contact_after_stop = port_.bms_charging_contact_snapshot();
          const bool charging_confirmed =
            contact_after_stop.have_state && contact_after_stop.fresh &&
            contact_after_stop.contact_stable;
          const double stop_duration_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - contact_stop_started).count();

          std::ostringstream contact_detail;
          contact_detail << "stable BMS charging contact during predock Nav2"
                         << " reason=" << predock_contact.reason
                         << " contact_stable_duration_sec="
                         << predock_contact.contact_stable_duration_sec
                         << " cancel_requested=" << (cancel_requested ? "true" : "false")
                         << " cancel_detail=" << cancel_detail
                         << " actual_stop_confirmed="
                         << (actual_stop_confirmed ? "true" : "false")
                         << " stop_duration_sec=" << stop_duration_sec
                         << " stop_detail=" << stop_detail
                         << " charging_confirmed_after_stop="
                         << (charging_confirmed ? "true" : "false");
          {
            std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
            if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
              port_.docking_job_unsafe().predock_nav_contact_detected = true;
              port_.docking_job_unsafe().predock_nav_canceled_for_contact = cancel_requested;
              port_.docking_job_unsafe().predock_contact_stop_confirmed = actual_stop_confirmed;
              port_.docking_job_unsafe().predock_contact_stop_duration_sec = stop_duration_sec;
              port_.docking_job_unsafe().predock_contact_reason = predock_contact.reason;
              port_.docking_job_unsafe().predock_contact_detected_at = contact_detected_at;
              port_.docking_job_unsafe().predock_contact_stop_detail = contact_detail.str();
              port_.docking_job_unsafe().detail = contact_detail.str();
            }
          }
          if (charging_confirmed) {
            port_.finish_docking_job(job_id, true, "charging", contact_detail.str());
          } else {
            port_.finish_docking_job_with_code(
              job_id,
              "DOCK_FAILED_PREDOCK_CONTACT_DROPPED",
              contact_detail.str() +
                "; charging contact dropped after stop; explicit undock/retry is required");
          }
          return;
        }
        if (port_.docking_cancel_requested(job_id)) {
          port_.clear_navigation_terminal_reverse_permit(
            terminal_reverse_permit_active,
            "predock_navigation_canceled");
          std::string cancel_detail;
          port_.cancel_active_navigation_goal(cancel_detail);
          port_.clear_navigation_terminal_speed_limit();
          port_.finish_docking_job(job_id, true, "canceled", cancel_detail);
          return;
        }
        const auto now_steady = std::chrono::steady_clock::now();
        if (now_steady >= next_handoff_check) {
          next_handoff_check = now_steady + 500ms;
          auto handoff_assessment = predock_control_.probe_handoff(job_id, job);
          const auto & handoff_check = handoff_assessment.check;
          if (config_.predock_early_handoff_enabled &&
            handoff_assessment.recovery_allowed)
          {
            std::string cancel_detail;
            const bool exact_goal_terminal =
              port_.cancel_active_navigation_goal(cancel_detail);
            if (!exact_goal_terminal) {
              port_.clear_navigation_terminal_reverse_permit(
                terminal_reverse_permit_active,
                "predock_navigation_handoff_terminal_unproven");
              port_.clear_navigation_terminal_speed_limit();
              port_.clear_teleop_command();
              port_.publish_teleop_zero_burst();
              port_.finish_docking_job_with_code(
                job_id,
                "PREDOCK_NAV_TERMINAL_UNPROVEN",
                "predock early handoff refused because the exact Nav2 goal "
                "has no proven terminal result: " + cancel_detail);
              return;
            }
            port_.clear_teleop_command();
            port_.publish_teleop_zero_burst();
            nav2_predock_early_handoff = true;
            nav2_predock_result_detail =
              "predock native Nav2 early handoff to docking-owned yaw/lateral capture";
            std::ostringstream handoff_detail;
            handoff_detail << nav2_predock_result_detail
                           << " attempt " << predock_nav_attempt << "/"
                           << predock_nav_max_attempts
                           << " exact_goal_terminal=true"
                           << " cancel_detail=" << cancel_detail
                           << "; " << handoff_check.detail;
            nav2_predock_handoff_detail = handoff_detail.str();
            port_.set_docking_job_phase(job_id, "STAGING_NAV2_EARLY_HANDOFF");
            port_.set_navigation_runtime_state(
              true,
              "ready",
              "predock Nav2 goal canceled for docking-owned final capture");
            port_.set_docking_runtime_state(
              true,
              "STAGING_NAV2_EARLY_HANDOFF",
              nav2_predock_handoff_detail);
            {
              std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
              if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
                port_.docking_job_unsafe().predock_nav_early_handoff = true;
                port_.docking_job_unsafe().predock_nav_handoff_detail = nav2_predock_handoff_detail;
                port_.docking_job_unsafe().detail = nav2_predock_handoff_detail;
              }
            }
            break;
          }
        }
        if (now_steady > predock_deadline) {
          port_.clear_navigation_terminal_reverse_permit(
            terminal_reverse_permit_active,
            "predock_navigation_timeout");
          std::string cancel_detail;
          port_.cancel_active_navigation_goal(cancel_detail);
          port_.clear_navigation_terminal_speed_limit();
          port_.finish_docking_job(job_id, false, "failed", "timed out navigating to predock pose; " + cancel_detail);
          return;
        }
      }
      port_.clear_navigation_terminal_reverse_permit(
        terminal_reverse_permit_active,
        "predock_navigation_result");
      port_.clear_navigation_terminal_speed_limit();
      if (nav2_predock_early_handoff) {
        break;
      }
      const auto result = result_future.get();
      navigation_action_runtime_.mark_terminal_proven(goal_handle, true);
      nav2_predock_succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      nav2_predock_aborted = result.code == rclcpp_action::ResultCode::ABORTED;
      std::ostringstream nav2_predock_result;
      nav2_predock_result << "predock navigation result code " << static_cast<int>(result.code)
                          << " attempt " << predock_nav_attempt << "/" << predock_nav_max_attempts;
      nav2_predock_result_detail = nav2_predock_result.str();
      if (!nav2_predock_succeeded && !nav2_predock_aborted) {
        std::ostringstream detail;
        detail << "predock navigation failed with result code " << static_cast<int>(result.code);
        port_.clear_navigation_terminal_speed_limit();
        port_.finish_docking_job_with_code(
          job_id, "DOCK_FAILED_PREDOCK_NAV", detail.str());
        return;
      }
      if (nav2_predock_succeeded) {
        break;
      }

      port_.set_docking_job_phase(job_id, "STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK");
      port_.set_navigation_runtime_state(
        true,
        "ready",
        "predock native Nav2 aborted; checking docking handoff tolerance");
      port_.set_docking_runtime_state(
        true,
        "STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK",
        nav2_predock_result_detail +
          "; checking current pose before retry or docking-owned recovery");
      auto retry_assessment = predock_control_.verify_staging_pose(job_id, job);
      const auto & retry_check = retry_assessment.check;
      if (retry_assessment.recovery_allowed) {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id &&
          port_.docking_job_unsafe().state == "running")
        {
          port_.docking_job_unsafe().detail =
            nav2_predock_result_detail +
            "; Nav2 predock action aborted but current XY is inside docking handoff; " +
            "continuing with docking-owned yaw/lateral recovery; " + retry_check.detail;
        }
        break;
      }
      if (predock_nav_attempt >= predock_nav_max_attempts) {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id &&
          port_.docking_job_unsafe().state == "running")
        {
          port_.docking_job_unsafe().detail =
            nav2_predock_result_detail +
            "; predock native Nav2 exhausted retry budget outside docking recovery window; " +
            retry_check.detail;
        }
        break;
      }
      port_.set_docking_job_phase(job_id, "RESTAGE_RETRY");
      port_.set_docking_runtime_state(
        true,
        "RESTAGE_RETRY",
        nav2_predock_result_detail +
          "; retrying predock native Nav2 because current pose is outside docking recovery window; " +
          retry_check.detail);
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          ++port_.docking_job_unsafe().retry_count;
          port_.docking_job_unsafe().detail =
            nav2_predock_result_detail +
            "; retrying predock native Nav2 because current pose is outside docking recovery window; " +
            retry_check.detail;
        }
      }
      std::this_thread::sleep_for(300ms);
    }
    if (nav2_predock_succeeded) {
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().nav_goal_succeeded = true;
          port_.docking_job_unsafe().predock_nav_handoff_detail = nav2_predock_result_detail;
        }
      }
      port_.set_docking_job_phase(job_id, "STAGING_NAV2_GOAL_SUCCEEDED");
      port_.set_navigation_runtime_state(true, "ready", "predock native Nav2 goal reached");
    } else if (nav2_predock_early_handoff) {
      port_.set_docking_job_phase(job_id, "STAGING_NAV2_EARLY_HANDOFF");
      port_.set_navigation_runtime_state(
        true,
        "ready",
        "predock native Nav2 canceled after entering docking-owned handoff window");
      port_.set_docking_runtime_state(
        true,
        "STAGING_NAV2_EARLY_HANDOFF",
        nav2_predock_handoff_detail);
    } else {
      port_.set_docking_job_phase(job_id, "STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK");
      port_.set_navigation_runtime_state(
        true,
        "ready",
        "predock native Nav2 aborted; checking docking handoff tolerance");
      port_.set_docking_runtime_state(
        true,
        "STAGING_NAV2_GOAL_ABORTED_HANDOFF_CHECK",
        nav2_predock_result_detail + "; checking current pose before docking-owned recovery or terminal failure");
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
        port_.docking_job_unsafe().detail =
          nav2_predock_result_detail + "; checking current pose before docking-owned recovery or terminal failure";
      }
    }

    // The FollowPath action is now terminal, but that alone is not physical
    // proof that the chassis has stopped. Establish a one-way ownership
    // boundary before robot_docking_manager is allowed to publish near-field
    // commands. This uses local/wheel feedback and deliberately does not
    // require a particular Ranger steering mode.
    port_.set_docking_job_phase(job_id, "PREDOCK_NAV2_STOP_VERIFY");
    port_.set_navigation_runtime_state(
      true,
      "ready",
      "predock Nav2 action is terminal; proving actual chassis stop before docking handoff");
    port_.set_docking_runtime_state(
      true,
      "PREDOCK_NAV2_STOP_VERIFY",
      "holding zero and proving actual chassis stop before robot_docking_manager takes ownership");
    port_.reset_terminal_actual_stop_stability();
    port_.clear_teleop_command();
    port_.publish_teleop_zero_burst();
    std::string predock_nav_stop_detail;
    if (!port_.wait_for_terminal_actual_stop(
        "predock Nav2 to docking-manager ownership handoff",
        predock_nav_stop_detail,
        false))
    {
      port_.publish_teleop_zero_burst();
      const std::string detail =
        "predock Nav2 action reached a terminal result, but actual chassis stop "
        "could not be proven; docking manager was not started: " +
        predock_nav_stop_detail;
      port_.set_docking_runtime_state(
        true, "DOCK_FAILED_PREDOCK_NAV_STOP_UNPROVEN", detail);
      port_.finish_docking_job_with_code(
        job_id, "DOCK_FAILED_PREDOCK_NAV_STOP_UNPROVEN", detail);
      return;
    }
    port_.publish_teleop_zero_burst();
    {
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id == job_id &&
        port_.docking_job_unsafe().state == "running")
      {
        auto & handoff_detail = port_.docking_job_unsafe().predock_nav_handoff_detail;
        if (!handoff_detail.empty()) {
          handoff_detail += "; ";
        }
        handoff_detail += "actual chassis stop proven before ownership handoff: " +
          predock_nav_stop_detail;
        port_.docking_job_unsafe().detail = handoff_detail;
      }
    }

    port_.set_docking_job_phase(job_id, "PREDOCK_POSE_VERIFY");
    port_.set_docking_runtime_state(true, "PREDOCK_POSE_VERIFY", "verifying staging pose before GS2 handoff");
    auto predock_assessment = predock_control_.verify_staging_pose(job_id, job);
    auto predock_check = predock_assessment.check;

    bool predock_yaw_aligned = false;
    const bool predock_recovery_allowed = predock_assessment.recovery_allowed;
    if (!predock_recovery_allowed) {
      const std::string failure_code = nav2_predock_succeeded ?
        "PREDOCK_NATIVE_GOAL_VERIFY_FAILED" :
        "DOCK_FAILED_PREDOCK_NAV_OUTSIDE_HANDOFF_WINDOW";
      const std::string detail =
        (nav2_predock_succeeded ?
        "Nav2 predock action succeeded outside the docking recovery window; " :
        nav2_predock_result_detail +
        "; Nav2 predock action ended outside the docking recovery window; ") +
        predock_check.detail;
      port_.set_docking_job_phase(job_id, failure_code);
      port_.set_docking_runtime_state(true, failure_code, detail);
      port_.finish_docking_job_with_code(job_id, failure_code, detail);
      return;
    }

    port_.set_docking_job_phase(job_id, "PREDOCK_ALIGNMENT_DEFERRED_FOR_BRIDGE_SETTLE");
    port_.set_docking_runtime_state(
      true,
      "PREDOCK_ALIGNMENT_DEFERRED_FOR_BRIDGE_SETTLE",
      "predock is inside the docking recovery window; deferring physical yaw/lateral alignment "
      "until map->odom smoothing has settled and global corrections are frozen");
    {
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
        port_.docking_job_unsafe().detail =
          "predock pose accepted for deferred staging alignment; " + predock_check.detail;
      }
    }

    if (config_.relocalize_after_predock) {
      port_.set_docking_job_phase(job_id, "AFTER_PREDOCK_RELOCALIZE");
      port_.clear_teleop_command();
      port_.publish_teleop_zero_burst();
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().post_predock_relocalization_requested = true;
          port_.docking_job_unsafe().post_predock_relocalization_required = config_.relocalize_after_predock_required;
          port_.docking_job_unsafe().post_predock_relocalization_detail =
            "waiting for fresh localization_result after predock navigation";
          port_.docking_job_unsafe().detail = port_.docking_job_unsafe().post_predock_relocalization_detail;
        }
      }
      port_.set_docking_runtime_state(
        true,
        "AFTER_PREDOCK_RELOCALIZE",
        "predock reached; triggering localization before fine docking");
      std::string localization_detail;
      std::uint64_t relocalization_sequence = 0U;
      const bool relocalized = port_.trigger_localization_and_wait_for_result(
        "docking_after_predock:" + job.dock_id,
        localization_detail,
        -1.0,
        &relocalization_sequence);
      bool settle_ok = true;
      {
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().post_predock_relocalization_succeeded = relocalized;
          port_.docking_job_unsafe().post_predock_relocalization_detail = localization_detail;
          port_.docking_job_unsafe().detail = localization_detail;
        }
      }
      if (!relocalized && config_.relocalize_after_predock_required) {
        port_.finish_docking_job_with_code(
          job_id,
          "DOCK_FAILED_PREDOCK_RELOCALIZATION",
            "relocalize after predock failed before fine docking: " + localization_detail);
        return;
      }
      if (relocalized) {
        port_.set_docking_job_phase(job_id, "AFTER_PREDOCK_SETTLE");
        port_.set_docking_runtime_state(
          true,
          "AFTER_PREDOCK_SETTLE",
          "settling after predock relocalization before fine docking");
        const auto settle = port_.wait_for_docking_relocalization_settle_barrier(
          relocalization_sequence,
          "after_predock",
          "fine_docking",
          [this, job_id](std::string & cancel_detail) {
            if (!port_.docking_cancel_requested(job_id)) {
              return false;
            }
            cancel_detail = "CANCELLED_BY_APP: docking canceled during after-predock settle barrier";
            return true;
          });
        settle_ok = settle.ok;
        localization_detail += "; " +
          (settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail);
        {
          std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
          if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
            port_.docking_job_unsafe().post_predock_relocalization_succeeded = settle_ok;
            port_.docking_job_unsafe().post_predock_relocalization_detail = localization_detail;
            port_.docking_job_unsafe().detail = localization_detail;
          }
        }
        if (!settle_ok) {
          port_.finish_docking_job_with_code(
            job_id,
            "DOCK_FAILED_PREDOCK_SETTLE",
            "post-predock relocalization settle failed before fine docking: " + localization_detail);
          return;
        }
      }
      if (relocalized && config_.validate_predock_pose_after_relocalization) {
        std::string pose_check_detail;
        if (!predock_control_.validate_current_pose_near_approach(job, pose_check_detail)) {
          port_.finish_docking_job_with_code(
            job_id,
            "FINE_DOCKING_ENTRY_CONDITION_FAILED",
            "post-predock localization pose is outside approach tolerance; refusing fine docking: " +
              pose_check_detail);
          return;
        }
        std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
        if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
          port_.docking_job_unsafe().detail += "; " + pose_check_detail;
          port_.docking_job_unsafe().post_predock_relocalization_detail += "; " + pose_check_detail;
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(port_.docking_job_mutex());
      if (port_.docking_job_unsafe().id == job_id && port_.docking_job_unsafe().state == "running") {
        port_.docking_job_unsafe().post_predock_settle_complete = true;
        port_.docking_job_unsafe().dock_staging_handoff_ready = predock_yaw_aligned;
      }
    }

    if (!predock_control_.start_fine_docking_handoff(job_id, job)) {
      return;
    }
  }

void DockingJobExecutor::run_guarded(const std::uint64_t job_id)
  {
    try {
      run(job_id);
    } catch (const std::exception & exc) {
      port_.finish_docking_job(
        job_id,
        false,
        "failed",
        std::string("docking worker exception: ") + exc.what());
    } catch (...) {
      port_.finish_docking_job(job_id, false, "failed", "docking worker unknown exception");
    }
  }


}  // namespace robot_api_server::features::docking
