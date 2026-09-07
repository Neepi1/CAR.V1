#include "robot_api_server/features/docking/predock_alignment/predock_control_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

#include "rclcpp/logging.hpp"

using namespace std::chrono_literals;

namespace robot_api_server::features::docking::predock_alignment
{
namespace
{

bool transition_active(const std::string & detail)
{
  return detail.rfind("LOCALIZATION_TRANSITION_ACTIVE:", 0U) == 0U;
}

}  // namespace

PredockControlModule::PredockControlModule(
  rclcpp::Logger logger,
  PredockControlConfig config,
  PredockAlignmentPolicy & policy,
  DockingJobStore & job_store,
  PredockControlPorts ports)
: config_(std::move(config)),
  logger_(std::move(logger)),
  policy_(policy),
  job_store_(job_store),
  ports_(std::move(ports))
{
}

PredockPoseVerification PredockControlModule::evaluate_pose(const DockingJob & job)
{
  std::string pose_error;
  const auto pose = ports_.wait_for_current_pose(true, pose_error);
  return policy_.evaluate_pose(job, pose, pose_error);
}

void PredockControlModule::record_pose(
  const std::uint64_t job_id,
  const PredockPoseVerification & check)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id == job_id && current.state == "running") {
    policy_.apply_pose_verification(current, check);
  }
}

void PredockControlModule::record_yaw(
  const std::uint64_t job_id,
  const PredockYawAlignResult & result)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id == job_id && current.state == "running") {
    policy_.apply_yaw_align_result(current, result);
  }
}

void PredockControlModule::record_lateral(
  const std::uint64_t job_id,
  const PredockLateralAlignResult & result)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id == job_id && current.state == "running") {
    policy_.apply_lateral_align_result(current, result);
  }
}

void PredockControlModule::record_fine_entry(
  const std::uint64_t job_id,
  const PredockPoseVerification & check,
  const bool ok,
  const std::string & failure_code,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id == job_id && current.state == "running") {
    policy_.apply_fine_entry_check(current, check, ok, failure_code, detail);
  }
}

PredockAssessment PredockControlModule::probe_handoff(
  const std::uint64_t job_id,
  const DockingJob & job)
{
  PredockAssessment result;
  result.check = evaluate_pose(job);
  result.recovery_allowed = policy_.pose_allows_staging_recovery(result.check);
  result.yaw_aligned = policy_.yaw_angles_met(result.check);
  if (result.recovery_allowed) {
    record_pose(job_id, result.check);
  }
  return result;
}

PredockAssessment PredockControlModule::verify_staging_pose(
  const std::uint64_t job_id,
  const DockingJob & job)
{
  PredockAssessment result;
  result.check = evaluate_pose(job);
  result.recovery_allowed = policy_.pose_allows_staging_recovery(result.check);
  result.yaw_aligned = policy_.yaw_angles_met(result.check);
  record_pose(job_id, result.check);
  return result;
}

bool PredockControlModule::validate_current_pose_near_approach(
  const DockingJob & job,
  std::string & detail)
{
  if (!config_.validate_pose_after_relocalization) {
    detail = "post-predock pose check disabled";
    return true;
  }
  const auto check = evaluate_pose(job);
  detail = check.detail;
  return policy_.pose_inside_handoff_window(check);
}

bool PredockControlModule::yaw_fallback_allowed() const
{
  return config_.yaw_align_enabled && config_.yaw_align_fallback_enabled;
}

void PredockControlModule::publish_yaw_zero_burst()
{
  const geometry_msgs::msg::Twist zero;
  for (int i = 0; i < config_.yaw_align_zero_cmd_count; ++i) {
    ports_.publish_command(zero);
    std::this_thread::sleep_for(40ms);
  }
}

void PredockControlModule::publish_lateral_zero_burst()
{
  const geometry_msgs::msg::Twist zero;
  for (int i = 0; i < config_.lateral_align_zero_cmd_count; ++i) {
    ports_.publish_command(zero);
    std::this_thread::sleep_for(40ms);
  }
}

void PredockControlModule::stop_and_release_motion()
{
  publish_yaw_zero_burst();
  ports_.publish_forced_mode(config_.lateral_align_release_mode);
}

bool PredockControlModule::acquire_motion_owner(
  const std::uint64_t job_id,
  const bool lateral,
  std::string & failure_detail)
{
  if (ports_.try_acquire_motion_owner(lateral)) {
    return true;
  }
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    auto & current = job_store_.job_unsafe();
    if (current.id == job_id && current.state == "running") {
      current.cmd_owner_conflict_detected = true;
      current.ordinary_final_yaw_align_active = true;
      current.docking_blocked_by_final_yaw_align = true;
      current.detail = lateral ?
        "PREDOCK_LATERAL_ALIGN waiting for ordinary final_yaw_align owner to stop" :
        "PREDOCK_YAW_ALIGN waiting for ordinary final_yaw_align owner to stop";
    }
  }
  ports_.request_navigation_goal_cancel(
    "docking request preempts ordinary final_yaw_align");
  ports_.publish_final_yaw_zero_burst();
  const auto deadline = std::chrono::steady_clock::now() + 1500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    if (ports_.try_acquire_motion_owner(lateral)) {
      std::lock_guard<std::mutex> lock(job_store_.mutex());
      auto & current = job_store_.job_unsafe();
      if (current.id == job_id && current.state == "running") {
        current.docking_blocked_by_final_yaw_align = false;
        current.ordinary_final_yaw_align_active = false;
      }
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  failure_detail = lateral ?
    "ordinary final_yaw_align did not release cmd owner before predock lateral alignment" :
    "ordinary final_yaw_align did not release cmd owner before predock yaw alignment";
  return false;
}

void PredockControlModule::release_motion_owner(const std::uint64_t job_id)
{
  ports_.release_motion_owner();
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id == job_id) {
    current.predock_yaw_align_active = false;
    current.predock_lateral_align_active = false;
  }
}

bool PredockControlModule::bridge_safe_for_fine_entry(
  const localization::BridgeStatusSnapshot & bridge,
  std::string & detail) const
{
  std::string base_detail;
  if (ports_.bridge_safe_for_goal_start(bridge, "docking fine docking", base_detail)) {
    detail = base_detail;
    return true;
  }

  const bool amcl_not_tracking =
    base_detail.find("AMCL_NOT_TRACKING") != std::string::npos ||
    bridge.amcl_degraded_reason == "AMCL_NOT_TRACKING";
  const bool canonical_bridge_ready =
    bridge.available && bridge.has_map_to_odom &&
    bridge.map_to_odom_publisher_owner == "robot_localization_bridge";
  const bool no_bridge_transition =
    bridge.safe_for_goal_start && !bridge.correction_active &&
    !bridge.amcl_correction_pending &&
    std::fabs(bridge.remaining_translation_error_m) <= 0.02 &&
    std::fabs(bridge.remaining_yaw_error_rad) <= 0.05;
  if (amcl_not_tracking && canonical_bridge_ready && no_bridge_transition) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << base_detail
        << "; AMCL_NOT_TRACKING tolerated_for_fine_docking_after_predock_verified"
        << " map_to_odom_owner=" << bridge.map_to_odom_publisher_owner
        << " safe_for_goal_start=" << (bridge.safe_for_goal_start ? "true" : "false")
        << " correction_active=" << (bridge.correction_active ? "true" : "false")
        << " amcl_correction_pending=" <<
      (bridge.amcl_correction_pending ? "true" : "false")
        << " remaining_translation=" << bridge.remaining_translation_error_m
        << " remaining_yaw=" << bridge.remaining_yaw_error_rad;
    detail = out.str();
    return true;
  }
  detail = base_detail;
  return false;
}

void PredockControlModule::update_bridge_settle_state(
  const std::uint64_t job_id,
  const bool started,
  const bool complete,
  const std::string & failure_code,
  const std::string & detail,
  const double duration_sec,
  const localization::BridgeStatusSnapshot & bridge)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & current = job_store_.job_unsafe();
  if (current.id != job_id || current.state != "running") {
    return;
  }
  current.fine_bridge_settle_started = started;
  current.fine_bridge_settle_complete = complete;
  current.fine_bridge_settle_failure_code = failure_code;
  current.fine_bridge_settle_detail = detail;
  current.fine_bridge_settle_duration_sec = duration_sec;
  current.fine_bridge_settle_remaining_translation_m =
    bridge.available ? bridge.remaining_translation_error_m : -1.0;
  current.fine_bridge_settle_remaining_yaw_rad =
    bridge.available ? bridge.remaining_yaw_error_rad : -1.0;
  current.detail = detail;
}

bool PredockControlModule::wait_for_bridge_smoothing(
  const std::uint64_t job_id,
  std::string & failure_code,
  std::string & detail)
{
  if (!config_.fine_wait_for_bridge_smoothing_enabled) {
    const bool ok = bridge_safe_for_fine_entry(ports_.bridge_status_snapshot(), detail);
    failure_code = ok ? "NONE" : "DOCK_FAILED_LOCALIZATION_NOT_READY";
    return ok;
  }

  job_store_.set_phase(job_id, "FINE_DOCKING_BRIDGE_SETTLE");
  ports_.set_docking_runtime_state(
    true, "FINE_DOCKING_BRIDGE_SETTLE",
    "waiting for bridge map->odom smoothing before fine docking", true);
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started +
    std::chrono::milliseconds(config_.fine_bridge_smoothing_wait_timeout_ms);
  const auto sample_period =
    std::chrono::milliseconds(config_.fine_bridge_smoothing_sample_period_ms);
  int stable_samples = 0;
  localization::BridgeStatusSnapshot last_bridge;
  std::string last_detail;

  while (std::chrono::steady_clock::now() <= deadline) {
    if (job_store_.cancel_requested(job_id)) {
      failure_code = "CANCELLED_BY_APP";
      detail = "CANCELLED_BY_APP: docking canceled during fine bridge smoothing wait";
      update_bridge_settle_state(
        job_id, true, false, failure_code, detail,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
        last_bridge);
      return false;
    }
    if (config_.fine_bridge_smoothing_zero_cmd_during_wait) {
      ports_.clear_teleop_command();
      ports_.publish_teleop_zero();
      ports_.publish_command(geometry_msgs::msg::Twist{});
    }

    last_bridge = ports_.bridge_status_snapshot();
    std::string sample_detail;
    const bool safe = bridge_safe_for_fine_entry(last_bridge, sample_detail);
    last_detail = sample_detail;
    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    if (safe) {
      ++stable_samples;
      std::ostringstream progress;
      progress << "bridge smoothing settled before docking fine docking"
               << " stable_samples=" << stable_samples << "/"
               << config_.fine_bridge_smoothing_stable_samples
               << " elapsed_sec=" << std::fixed << std::setprecision(3) << elapsed_sec
               << "; " << sample_detail;
      update_bridge_settle_state(
        job_id, true,
        stable_samples >= config_.fine_bridge_smoothing_stable_samples,
        "", progress.str(), elapsed_sec, last_bridge);
      if (stable_samples >= config_.fine_bridge_smoothing_stable_samples) {
        failure_code = "NONE";
        detail = progress.str();
        return true;
      }
    } else {
      stable_samples = 0;
      update_bridge_settle_state(
        job_id, true, false, "", sample_detail, elapsed_sec, last_bridge);
      if (!transition_active(sample_detail)) {
        failure_code = "DOCK_FAILED_LOCALIZATION_NOT_READY";
        detail = sample_detail;
        return false;
      }
    }
    std::this_thread::sleep_for(sample_period);
  }

  failure_code = "DOCK_FAILED_FINE_LOCALIZATION_TRANSITION_TIMEOUT";
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "bridge smoothing did not settle before docking fine docking"
      << " timeout_ms=" << config_.fine_bridge_smoothing_wait_timeout_ms
      << " remaining_translation=" << last_bridge.remaining_translation_error_m
      << " remaining_yaw=" << last_bridge.remaining_yaw_error_rad
      << " current_sequence=" << last_bridge.current_sequence
      << " target_sequence=" << last_bridge.target_sequence
      << " last_detail=" << last_detail;
  detail = out.str();
  update_bridge_settle_state(
    job_id, true, false, failure_code, detail,
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
    last_bridge);
  return false;
}

PredockYawAlignResult PredockControlModule::run_yaw_align(
  const std::uint64_t job_id,
  const double expected_yaw,
  const PredockPoseVerification & initial_check,
  const double success_tolerance_rad)
{
  const double target_tolerance_rad = success_tolerance_rad > 0.0 ?
    success_tolerance_rad : config_.yaw_align_tolerance_rad;
  PredockYawAlignResult result;
  result.attempted = true;
  result.actual_spin_required = config_.yaw_align_require_actual_spin;
  result.initial_error_rad = initial_check.base_yaw_error_rad;
  result.final_error_rad = initial_check.base_yaw_error_rad;
  if (!yaw_fallback_allowed()) {
    result.succeeded = result.final_error_rad <= target_tolerance_rad;
    result.failure_code = result.succeeded ? "NONE" : "PREDOCK_YAW_NOT_ALIGNED_AFTER_NAV2";
    result.detail = result.succeeded ?
      "predock yaw align fallback disabled; yaw already within trigger" :
      "predock yaw align fallback disabled; Nav2 native predock goal did not finish within yaw tolerance";
    return result;
  }
  if (!initial_check.pose_available) {
    result.blocked = true;
    result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
    result.detail = initial_check.detail;
    return result;
  }
  if (initial_check.base_yaw_error_rad <= target_tolerance_rad) {
    result.succeeded = true;
    result.failure_code = "NONE";
    result.detail = "predock yaw already aligned";
    return result;
  }
  if (initial_check.base_yaw_error_rad > config_.yaw_align_hard_fail_rad) {
    result.blocked = true;
    result.failure_code = "PREDOCK_YAW_HARD_FAIL";
    result.detail = "predock yaw error exceeds hard fail limit";
    return result;
  }

  std::string owner_failure;
  if (!acquire_motion_owner(job_id, false, owner_failure)) {
    result.blocked = true;
    result.failure_code = "PREDOCK_YAW_ALIGN_OWNER_CONFLICT";
    result.detail = owner_failure;
    publish_yaw_zero_burst();
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    auto & current = job_store_.job_unsafe();
    if (current.id == job_id && current.state == "running") {
      current.predock_yaw_align_active = true;
    }
  }

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.yaw_align_timeout_sec));
  const auto tick = std::chrono::milliseconds(config_.yaw_align_period_ms);
  auto last_yaw_motion_time = started;
  const double first_yaw = initial_check.pose.yaw;
  double last_yaw = initial_check.pose.yaw;
  int success_hold = 0;
  bool nonzero_command_published = false;
  bool actual_spin_confirmed = !config_.yaw_align_require_actual_spin;
  std::chrono::steady_clock::time_point first_nonzero_command_time;

  while (std::chrono::steady_clock::now() < deadline) {
    if (job_store_.cancel_requested(job_id)) {
      result.canceled = true;
      result.blocked = true;
      result.failure_code = "CANCELLED_BY_APP";
      result.detail = "predock yaw alignment canceled by App";
      break;
    }
    std::string safety_detail;
    if (ports_.safety_motion_hard_blocked(safety_detail)) {
      result.blocked = true;
      result.failure_code = "DOCK_FAILED_SAFETY_BLOCKED";
      result.detail = "predock yaw alignment blocked by safety: " + safety_detail;
      break;
    }

    auto pose = ports_.current_pose();
    if (!pose.available || pose.frame_id != config_.map_frame) {
      result.blocked = true;
      result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
      result.detail = "predock yaw alignment has no map-frame pose";
      break;
    }
    if (pose.age_sec > config_.robot_pose_freshness_sec) {
      result.blocked = true;
      result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
      result.detail = "predock yaw alignment pose is stale";
      break;
    }

    const double signed_error = policy_.predock_yaw_error(pose.yaw, expected_yaw);
    result.final_error_rad = std::fabs(signed_error);
    const double command_speed = std::clamp(
      std::fabs(config_.yaw_align_kp * signed_error),
      config_.yaw_align_min_speed_radps,
      config_.yaw_align_max_speed_radps);
    const double stop_threshold =
      ports_.yaw_stop_threshold_rad(command_speed, target_tolerance_rad);
    result.observed_yaw_motion_rad = std::max(
      result.observed_yaw_motion_rad,
      std::fabs(policy_.normalize_yaw_error(pose.yaw - first_yaw)));
    if (std::fabs(policy_.normalize_yaw_error(pose.yaw - last_yaw)) >=
      config_.yaw_align_motion_epsilon_rad)
    {
      last_yaw_motion_time = std::chrono::steady_clock::now();
      last_yaw = pose.yaw;
    }

    if (result.final_error_rad <= stop_threshold) {
      if (nonzero_command_published && config_.yaw_align_require_actual_spin &&
        !actual_spin_confirmed)
      {
        result.blocked = true;
        result.failure_code = "PREDOCK_YAW_ALIGN_NO_CONFIRMED_PHYSICAL_SPIN";
        result.detail =
          "predock map yaw entered tolerance before Ranger feedback confirmed SPINNING=2";
        break;
      }
      ++success_hold;
      ports_.publish_command(geometry_msgs::msg::Twist{});
      if (success_hold >= config_.yaw_align_success_hold_count) {
        std::string stop_detail;
        const bool actual_stopped = ports_.wait_for_actual_stop("predock_yaw_align", stop_detail);
        auto settled_pose = ports_.current_pose();
        if (!settled_pose.available || settled_pose.frame_id != config_.map_frame) {
          result.blocked = true;
          result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
          result.detail =
            "predock yaw alignment could not verify pose after actual stop wait: " + stop_detail;
          break;
        }
        if (settled_pose.age_sec > config_.robot_pose_freshness_sec) {
          result.blocked = true;
          result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
          result.detail =
            "predock yaw alignment pose is stale after actual stop wait: " + stop_detail;
          break;
        }
        result.final_error_rad = std::fabs(
          policy_.predock_yaw_error(settled_pose.yaw, expected_yaw));
        if (result.final_error_rad <= target_tolerance_rad) {
          result.succeeded = true;
          result.failure_code = "NONE";
          result.detail = actual_stopped ?
            "predock yaw aligned after actual angular velocity settled: " + stop_detail :
            "predock yaw aligned after stop-wait timeout and pose recheck: " + stop_detail;
          break;
        }
        success_hold = 0;
        last_yaw = settled_pose.yaw;
        last_yaw_motion_time = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(tick);
      continue;
    }
    success_hold = 0;

    geometry_msgs::msg::Twist twist;
    twist.angular.z = std::copysign(command_speed, signed_error);
    ports_.reset_actual_stop_stability();
    const auto command_time = std::chrono::steady_clock::now();
    if (!nonzero_command_published) {
      first_nonzero_command_time = command_time;
    }
    ports_.publish_command(twist);
    nonzero_command_published = true;

    if (config_.yaw_align_require_actual_spin && !actual_spin_confirmed) {
      const auto mode_status = ports_.motion_mode_snapshot();
      const bool actual_spinning = mode_status.available &&
        mode_status.actual_available && mode_status.actual_fresh &&
        mode_status.actual_motion_mode_code == 2;
      if (actual_spinning) {
        actual_spin_confirmed = true;
      } else {
        const double mode_wait_sec = std::chrono::duration<double>(
          command_time - first_nonzero_command_time).count();
        if (mode_wait_sec >= config_.yaw_align_mode_switch_timeout_sec) {
          result.blocked = true;
          result.failure_code = "PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT";
          std::ostringstream detail;
          detail << "Ranger feedback did not confirm SPINNING=2 before predock yaw command timeout"
                 << " wait_sec=" << mode_wait_sec
                 << " status_available=" << (mode_status.available ? "true" : "false")
                 << " actual_available=" <<
            (mode_status.actual_available ? "true" : "false")
                 << " actual_fresh=" << (mode_status.actual_fresh ? "true" : "false")
                 << " actual_code=" << mode_status.actual_motion_mode_code
                 << " status_age_sec=" << mode_status.age_sec;
          result.detail = detail.str();
          break;
        }
      }
    }
    const double no_motion_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_yaw_motion_time).count();
    if (no_motion_sec >= config_.yaw_align_no_yaw_motion_timeout_sec) {
      result.blocked = true;
      result.failure_code = "PREDOCK_YAW_ALIGN_NO_YAW_MOTION";
      result.detail = "predock yaw command published but map-frame yaw did not change";
      break;
    }
    std::this_thread::sleep_for(tick);
  }

  result.duration_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  publish_yaw_zero_burst();
  release_motion_owner(job_id);
  if (!result.succeeded && !result.blocked) {
    result.blocked = true;
    result.failure_code = "PREDOCK_YAW_ALIGN_TIMEOUT";
    result.detail = "predock yaw alignment timed out";
  }
  if (result.detail.empty()) {
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(3)
           << "predock yaw align final_error=" << result.final_error_rad
           << " initial_error=" << result.initial_error_rad
           << " observed_yaw_motion=" << result.observed_yaw_motion_rad
           << " duration_sec=" << result.duration_sec;
    result.detail = detail.str();
  }
  return result;
}

PredockLateralAlignResult PredockControlModule::run_lateral_align(
  const std::uint64_t job_id,
  const DockingJob & job,
  const PredockPoseVerification & initial_check)
{
  PredockLateralAlignResult result;
  result.initial_error_m = initial_check.lateral_m;
  result.final_error_m = initial_check.lateral_m;
  if (!config_.lateral_align_enabled) {
    result.succeeded = initial_check.pose_available &&
      initial_check.lateral_abs_m <= config_.lateral_align_target_m;
    result.failure_code = result.succeeded ? "NONE" : "PREDOCK_LATERAL_NOT_ALIGNED";
    result.detail = result.succeeded ?
      "predock lateral alignment disabled; lateral already within target" :
      "predock lateral alignment disabled and lateral error is outside target; " +
      initial_check.detail;
    return result;
  }
  if (!initial_check.pose_available) {
    result.blocked = true;
    result.failure_code = "PREDOCK_LATERAL_NOT_ALIGNED";
    result.detail = initial_check.detail;
    return result;
  }
  if (!initial_check.xy_ok && !policy_.lateral_capture_allowed(initial_check)) {
    result.blocked = true;
    result.failure_code = "PREDOCK_LATERAL_NOT_ALIGNED";
    result.detail =
      "predock lateral alignment requires forward error inside handoff and lateral error "
      "inside correction window; " + initial_check.detail;
    return result;
  }
  if (!initial_check.base_yaw_ok || !initial_check.contact_yaw_ok) {
    result.blocked = true;
    result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
    result.detail = "predock lateral alignment requires yaw to be aligned first; " +
      initial_check.detail;
    return result;
  }
  if (initial_check.lateral_abs_m <= config_.lateral_align_target_m) {
    result.succeeded = true;
    result.failure_code = "NONE";
    result.detail = "predock lateral already aligned";
    return result;
  }
  if (initial_check.lateral_abs_m > config_.lateral_align_max_correction_m) {
    result.blocked = true;
    result.failure_code = "PREDOCK_LATERAL_HARD_FAIL";
    result.detail = "predock lateral error exceeds maximum correction window; " +
      initial_check.detail;
    return result;
  }

  result.attempted = true;
  std::string owner_failure;
  if (!acquire_motion_owner(job_id, true, owner_failure)) {
    result.blocked = true;
    result.failure_code = "PREDOCK_LATERAL_ALIGN_OWNER_CONFLICT";
    result.detail = owner_failure;
    publish_lateral_zero_burst();
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    auto & current = job_store_.job_unsafe();
    if (current.id == job_id && current.state == "running") {
      current.predock_lateral_align_active = true;
      current.predock_yaw_align_active = true;
    }
  }

  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.lateral_align_timeout_sec));
  const auto tick = std::chrono::milliseconds(config_.lateral_align_period_ms);
  auto last_lateral_motion_time = started;
  const double first_lateral = initial_check.lateral_m;
  double last_lateral = initial_check.lateral_m;
  double best_lateral_abs = initial_check.lateral_abs_m;
  double direction_multiplier = config_.lateral_align_command_sign;
  int success_hold = 0;
  int divergence_count = 0;
  bool nonzero_command_published = false;
  bool direction_reversed = false;

  ports_.publish_forced_mode(config_.lateral_align_forced_mode);
  while (std::chrono::steady_clock::now() < deadline) {
    if (job_store_.cancel_requested(job_id)) {
      result.canceled = true;
      result.blocked = true;
      result.failure_code = "CANCELLED_BY_APP";
      result.detail = "predock lateral alignment canceled by App";
      break;
    }
    std::string safety_detail;
    if (ports_.safety_motion_hard_blocked(safety_detail)) {
      result.blocked = true;
      result.failure_code = "DOCK_FAILED_SAFETY_BLOCKED";
      result.detail = "predock lateral alignment blocked by safety: " + safety_detail;
      break;
    }

    auto check = evaluate_pose(job);
    record_pose(job_id, check);
    if (!check.pose_available) {
      result.blocked = true;
      result.failure_code = "PREDOCK_LATERAL_NOT_ALIGNED";
      result.detail = "predock lateral alignment has no map-frame pose; " + check.detail;
      break;
    }
    if (!policy_.lateral_align_yaw_gate_met(check)) {
      std::ostringstream detail;
      detail << std::fixed << std::setprecision(3)
             << "predock yaw exceeded lateral alignment gate"
             << " base_yaw_error=" << check.base_yaw_error_rad
             << " contact_yaw_error=" << check.contact_yaw_error_rad
             << " lateral_yaw_gate=" << policy_.lateral_align_yaw_gate_rad()
             << "; " << check.detail;
      result.blocked = true;
      result.failure_code = "PREDOCK_YAW_NOT_ALIGNED";
      result.detail = detail.str();
      break;
    }
    if (!policy_.lateral_capture_allowed(check)) {
      result.blocked = true;
      result.failure_code = "PREDOCK_LATERAL_HARD_FAIL";
      result.detail =
        "predock lateral capture moved outside forward/lateral correction window; " +
        check.detail;
      break;
    }

    result.final_error_m = check.lateral_m;
    result.observed_lateral_motion_m = std::max(
      result.observed_lateral_motion_m,
      std::fabs(check.lateral_m - first_lateral));
    if (check.lateral_abs_m < best_lateral_abs) {
      best_lateral_abs = check.lateral_abs_m;
      divergence_count = 0;
    } else if (nonzero_command_published &&
      check.lateral_abs_m > best_lateral_abs + config_.lateral_align_divergence_epsilon_m)
    {
      ++divergence_count;
    } else {
      divergence_count = 0;
    }

    if (nonzero_command_published &&
      divergence_count >= config_.lateral_align_divergence_count)
    {
      const double previous_best_lateral_abs = best_lateral_abs;
      publish_lateral_zero_burst();
      if (config_.lateral_align_auto_reverse_on_divergence && !direction_reversed) {
        direction_multiplier *= -1.0;
        direction_reversed = true;
        divergence_count = 0;
        best_lateral_abs = check.lateral_abs_m;
        last_lateral = check.lateral_m;
        last_lateral_motion_time = std::chrono::steady_clock::now();
        nonzero_command_published = false;
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
               << "predock lateral error diverged; reversing side-slip command direction once"
               << " current_lateral=" << check.lateral_m
               << " current_abs=" << check.lateral_abs_m
               << " best_abs=" << previous_best_lateral_abs
               << " divergence_epsilon=" << config_.lateral_align_divergence_epsilon_m
               << " direction_multiplier=" << direction_multiplier;
        const auto detail_text = detail.str();
        RCLCPP_WARN(logger_, "%s", detail_text.c_str());
        {
          std::lock_guard<std::mutex> lock(job_store_.mutex());
          auto & current = job_store_.job_unsafe();
          if (current.id == job_id && current.state == "running") {
            current.detail = detail_text;
          }
        }
        std::this_thread::sleep_for(tick);
        continue;
      }
      result.blocked = true;
      result.failure_code = "PREDOCK_LATERAL_ALIGN_DIVERGING";
      std::ostringstream detail;
      detail << std::fixed << std::setprecision(3)
             << "predock lateral error diverged after side-slip command"
             << " current_lateral=" << check.lateral_m
             << " current_abs=" << check.lateral_abs_m
             << " best_abs=" << previous_best_lateral_abs
             << " divergence_epsilon=" << config_.lateral_align_divergence_epsilon_m
             << " direction_reversed=" << (direction_reversed ? "true" : "false")
             << "; " << check.detail;
      result.detail = detail.str();
      break;
    }

    if (std::fabs(check.lateral_m - last_lateral) >=
      config_.lateral_align_motion_epsilon_m)
    {
      last_lateral_motion_time = std::chrono::steady_clock::now();
      last_lateral = check.lateral_m;
    }
    if (check.lateral_abs_m <= config_.lateral_align_target_m) {
      ++success_hold;
      ports_.publish_command(geometry_msgs::msg::Twist{});
      if (success_hold >= config_.yaw_align_success_hold_count) {
        result.succeeded = true;
        result.failure_code = "NONE";
        result.detail = direction_reversed ?
          "predock lateral aligned after reversing divergent side-slip direction" :
          "predock lateral aligned";
        break;
      }
      std::this_thread::sleep_for(tick);
      continue;
    }
    success_hold = 0;
    if (check.lateral_abs_m > config_.lateral_align_max_correction_m) {
      result.blocked = true;
      result.failure_code = "PREDOCK_LATERAL_HARD_FAIL";
      result.detail = "predock lateral drifted outside maximum correction window; " +
        check.detail;
      break;
    }

    const double min_speed = std::min(0.010, config_.lateral_align_speed_mps);
    const double command_speed = std::clamp(
      std::fabs(config_.lateral_align_kp * check.lateral_m),
      min_speed,
      config_.lateral_align_speed_mps);
    geometry_msgs::msg::Twist twist;
    twist.linear.y = direction_multiplier * std::copysign(command_speed, check.lateral_m);
    ports_.publish_forced_mode(config_.lateral_align_forced_mode);
    ports_.publish_command(twist);
    nonzero_command_published = true;
    const double no_motion_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_lateral_motion_time).count();
    if (no_motion_sec >= config_.lateral_align_no_motion_timeout_sec) {
      result.blocked = true;
      result.failure_code = "PREDOCK_LATERAL_ALIGN_NO_LATERAL_MOTION";
      result.detail =
        "predock lateral command published but map-frame lateral error did not change";
      break;
    }
    std::this_thread::sleep_for(tick);
  }

  result.duration_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  publish_lateral_zero_burst();
  ports_.publish_forced_mode(config_.lateral_align_release_mode);
  release_motion_owner(job_id);
  if (!result.succeeded && !result.blocked) {
    result.blocked = true;
    result.failure_code = "PREDOCK_LATERAL_ALIGN_TIMEOUT";
    result.detail = "predock lateral alignment timed out";
  }
  if (result.detail.empty()) {
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(3)
           << "predock lateral align final_error=" << result.final_error_m
           << " initial_error=" << result.initial_error_m
           << " observed_lateral_motion=" << result.observed_lateral_motion_m
           << " duration_sec=" << result.duration_sec;
    result.detail = detail.str();
  }
  return result;
}

bool PredockControlModule::ensure_lateral_alignment(
  const std::uint64_t job_id,
  const DockingJob & job,
  PredockPoseVerification & check,
  bool & yaw_aligned,
  const std::string & align_phase,
  const std::string & verify_phase,
  const std::string & align_detail)
{
  const int max_cycles = std::max(1, config_.staging_capture_max_cycles);
  std::string last_detail = check.detail;
  for (int cycle = 1; cycle <= max_cycles; ++cycle) {
    job_store_.set_phase(job_id, verify_phase);
    std::ostringstream verify_detail;
    verify_detail << "closed-loop predock staging capture verify cycle " << cycle
                  << "/" << max_cycles;
    ports_.set_docking_runtime_state(true, verify_phase, verify_detail.str(), true);

    check = evaluate_pose(job);
    record_pose(job_id, check);
    last_detail = check.detail;
    yaw_aligned = policy_.yaw_angles_met(check);
    if (policy_.staging_target_met(check)) {
      PredockLateralAlignResult already_aligned;
      already_aligned.attempted = false;
      already_aligned.succeeded = true;
      already_aligned.failure_code = "NONE";
      already_aligned.initial_error_m = check.lateral_m;
      already_aligned.final_error_m = check.lateral_m;
      already_aligned.detail =
        "predock staging capture already satisfies strict XY/yaw/lateral target";
      record_lateral(job_id, already_aligned);
      std::lock_guard<std::mutex> lock(job_store_.mutex());
      auto & current = job_store_.job_unsafe();
      if (current.id == job_id && current.state == "running") {
        current.predock_lateral_aligned = true;
        current.dock_staging_handoff_ready = true;
      }
      return true;
    }
    if (!check.pose_available) {
      job_store_.finish_with_code(
        job_id, "PREDOCK_LATERAL_NOT_ALIGNED",
        "predock staging capture has no fresh map-frame pose; " + check.detail);
      return false;
    }
    if (!policy_.yaw_angles_met(check)) {
      if (check.base_yaw_error_rad > config_.yaw_align_hard_fail_rad ||
        check.contact_yaw_error_rad > config_.yaw_align_hard_fail_rad)
      {
        job_store_.finish_with_code(
          job_id, "PREDOCK_YAW_HARD_FAIL",
          "predock staging capture yaw exceeds hard fail limit; " + check.detail);
        return false;
      }
      if (!yaw_fallback_allowed()) {
        job_store_.finish_with_code(
          job_id, "PREDOCK_YAW_NOT_ALIGNED",
          "predock staging capture requires yaw correction but yaw fallback is disabled; " +
          check.detail);
        return false;
      }

      const std::string yaw_phase = align_phase + "_YAW_RECAPTURE";
      job_store_.set_phase(job_id, yaw_phase);
      std::ostringstream detail;
      detail << "recapturing predock yaw in closed-loop staging cycle " << cycle
             << "/" << max_cycles;
      ports_.set_docking_runtime_state(true, yaw_phase, detail.str(), true);
      auto yaw_result = run_yaw_align(job_id, check.expected_base_yaw, check);
      record_yaw(job_id, yaw_result);
      if (!yaw_result.succeeded) {
        job_store_.finish_with_code(job_id, yaw_result.failure_code, yaw_result.detail);
        return false;
      }
      std::this_thread::sleep_for(300ms);
      continue;
    }

    if (!policy_.lateral_error_target_met(check) ||
      !policy_.forward_capture_window_met(check))
    {
      if (!policy_.lateral_capture_allowed(check)) {
        job_store_.finish_with_code(
          job_id, "PREDOCK_LATERAL_HARD_FAIL",
          "predock staging capture cannot fix current centerline/forward window by side-slip; " +
          check.detail);
        return false;
      }
      job_store_.set_phase(job_id, align_phase);
      std::ostringstream detail;
      detail << align_detail << "; closed-loop lateral capture cycle " << cycle
             << "/" << max_cycles;
      ports_.set_docking_runtime_state(true, align_phase, detail.str(), true);
      auto lateral_result = run_lateral_align(job_id, job, check);
      record_lateral(job_id, lateral_result);
      if (!lateral_result.succeeded) {
        if (lateral_result.failure_code == "PREDOCK_YAW_NOT_ALIGNED" &&
          cycle < max_cycles)
        {
          std::this_thread::sleep_for(300ms);
          continue;
        }
        job_store_.finish_with_code(
          job_id, lateral_result.failure_code, lateral_result.detail);
        return false;
      }
      std::this_thread::sleep_for(300ms);
      continue;
    }
    std::this_thread::sleep_for(300ms);
  }

  job_store_.finish_with_code(
    job_id, "PREDOCK_LATERAL_NOT_ALIGNED",
    "predock staging capture exhausted closed-loop cycles; " + last_detail);
  return false;
}

bool PredockControlModule::evaluate_fine_entry(
  const DockingJob & job,
  const bool yaw_aligned,
  PredockPoseVerification & check,
  std::string & failure_code,
  std::string & detail)
{
  check = evaluate_pose(job);
  const auto observation = ports_.observation_snapshot();
  const double lateral_m = check.pose_available ? check.lateral_abs_m : -1.0;
  std::string current_policy = job.goal_completion_policy;
  bool handoff_ready = job.dock_staging_handoff_ready;
  bool predock_pose_verified = job.predock_pose_verified;
  bool post_predock_settle_complete = job.post_predock_settle_complete;
  bool correction_pause_applied = job.global_correction_paused;
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    const auto & current = job_store_.job_unsafe();
    if (current.id == job.id) {
      current_policy = current.goal_completion_policy;
      handoff_ready = current.dock_staging_handoff_ready;
      predock_pose_verified = current.predock_pose_verified;
      post_predock_settle_complete = current.post_predock_settle_complete;
      correction_pause_applied = current.global_correction_paused;
    }
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "fine docking entry check"
      << " goal_completion_policy=" << current_policy
      << " dock_staging_handoff_ready=" << (handoff_ready ? "true" : "false")
      << " predock_pose_verified=" << (predock_pose_verified ? "true" : "false")
      << " post_predock_settle_complete=" <<
    (post_predock_settle_complete ? "true" : "false")
      << " global_correction_pause_applied=" <<
    (correction_pause_applied ? "true" : "false")
      << " pose_available=" << (check.pose_available ? "true" : "false")
      << " distance=" << check.distance_m << "/" << config_.fine_entry_max_distance_m
      << " forward=" << check.forward_m
      << " forward_capture_min=" << config_.forward_capture_min_m
      << " forward_capture_max=" << config_.forward_capture_max_m
      << " lateral=" << lateral_m << "/" << config_.fine_entry_max_lateral_m
      << " base_yaw_error=" << check.base_yaw_error_rad << "/" <<
    config_.fine_entry_max_yaw_rad
      << " contact_yaw_error=" << check.contact_yaw_error_rad << "/" <<
    config_.fine_entry_max_yaw_rad
      << " observation_backend=" << config_.observation_backend
      << " observation_age_sec=" << observation.age_sec
      << " observation_usable=" << (observation.usable ? "true" : "false")
      << " observation_detail=" << observation.detail
      << " require_observation_fresh=" <<
    (config_.fine_entry_require_observation_fresh ? "true" : "false")
      << " predock_yaw_aligned=" << (yaw_aligned ? "true" : "false");
  detail = out.str();

  if (current_policy != "dock_staging" || !handoff_ready ||
    !post_predock_settle_complete)
  {
    failure_code = "FINE_DOCKING_ENTRY_CONDITION_FAILED";
    return false;
  }
  if (config_.pause_global_correction_during_fine && !correction_pause_applied) {
    failure_code = "FINE_DOCKING_ENTRY_CONDITION_FAILED";
    return false;
  }
  if (!check.pose_available) {
    failure_code = "FINE_DOCKING_ENTRY_CONDITION_FAILED";
    return false;
  }
  if (config_.fine_entry_require_observation_fresh &&
    (observation.age_sec < 0.0 || observation.age_sec > 0.5 || !observation.usable))
  {
    failure_code = config_.observation_backend == "target_observation" ?
      "DOCK_TARGET_OBSERVATION_TIMEOUT" : "GS2_DOCK_DETECT_TIMEOUT";
    return false;
  }
  if (!config_.delegate_staging_motion_to_manager &&
    config_.fine_entry_require_predock_yaw_aligned && !yaw_aligned)
  {
    failure_code = "PREDOCK_YAW_NOT_ALIGNED";
    return false;
  }
  if (config_.delegate_staging_motion_to_manager) {
    if (!policy_.pose_allows_staging_recovery(check)) {
      failure_code = "FINE_DOCKING_ENTRY_CONDITION_FAILED";
      return false;
    }
    failure_code = "NONE";
    detail += "; staging_motion_owner=robot_docking_manager"
      " residual_yaw_lateral_delegated=true";
    return true;
  }
  if (!policy_.forward_capture_window_met(check)) {
    failure_code = "FINE_DOCKING_ENTRY_CONDITION_FAILED";
    return false;
  }
  if (lateral_m > config_.fine_entry_max_lateral_m) {
    failure_code = "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE";
    return false;
  }
  if (check.base_yaw_error_rad > config_.fine_entry_max_yaw_rad ||
    check.contact_yaw_error_rad > config_.fine_entry_max_yaw_rad)
  {
    failure_code = "FINE_DOCKING_REJECTED_YAW_TOO_LARGE";
    return false;
  }
  failure_code = "NONE";
  return true;
}

bool PredockControlModule::start_fine_docking_handoff(
  const std::uint64_t job_id,
  const DockingJob & job)
{
  job_store_.set_phase(job_id, "GS2_DOCK_DETECT");
  if (job_store_.cancel_requested(job_id)) {
    job_store_.finish(
      job_id, true, "canceled", "docking canceled before GS2 fine docking");
    return false;
  }
  std::string ensure_detail;
  if (!ports_.ensure_docking_manager_running(ensure_detail)) {
    job_store_.finish(job_id, false, "failed", ensure_detail);
    return false;
  }

  bool fine_pause_applied = false;
  auto resume_pause = [&](const std::string & reason) {
      if (!fine_pause_applied) {
        return;
      }
      std::string resume_detail;
      (void)ports_.set_global_correction_paused(
        job_id, false, reason, resume_detail);
      fine_pause_applied = false;
    };

  std::string bridge_detail;
  std::string bridge_failure_code;
  if (!wait_for_bridge_smoothing(job_id, bridge_failure_code, bridge_detail)) {
    if (bridge_failure_code == "CANCELLED_BY_APP") {
      job_store_.finish(job_id, true, "canceled", bridge_detail);
    } else {
      job_store_.finish_with_code(job_id, bridge_failure_code, bridge_detail);
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    auto & current = job_store_.job_unsafe();
    if (current.id == job_id && current.state == "running") {
      if (!current.detail.empty()) {
        current.detail += "; ";
      }
      current.detail += bridge_detail;
    }
  }

  std::string pause_detail;
  if (!ports_.set_global_correction_paused(
      job_id, true, "docking_staging_alignment", pause_detail))
  {
    job_store_.finish_with_code(job_id, "DOCK_FAILED_PREDOCK_SETTLE", pause_detail);
    return false;
  }
  fine_pause_applied = true;

  job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE");
  ports_.set_docking_runtime_state(
    true, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE",
    "rechecking staging pose after bridge smoothing before GS2 handoff", true);
  auto assessment = verify_staging_pose(job_id, job);
  auto check = assessment.check;
  bool yaw_aligned = assessment.yaw_aligned;
  if (config_.delegate_staging_motion_to_manager) {
    if (!assessment.recovery_allowed) {
      const std::string detail =
        "post-bridge staging pose is outside the docking-manager capture window; " +
        check.detail;
      job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE_FAILED");
      ports_.set_docking_runtime_state(
        true, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE_FAILED", detail, true);
      resume_pause("fine_docking_entry_outside_manager_capture");
      job_store_.finish_with_code(
        job_id, "PREDOCK_POSE_DRIFTED_AFTER_BRIDGE_SETTLE", detail);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(job_store_.mutex());
      auto & current = job_store_.job_unsafe();
      if (current.id == job_id && current.state == "running") {
        current.post_predock_settle_complete = true;
        current.dock_staging_handoff_ready = true;
        current.predock_yaw_aligned = yaw_aligned;
        current.predock_lateral_aligned = policy_.lateral_target_met(check);
        current.detail =
          "staging residual delegated to robot_docking_manager; API issued no physical "
          "yaw/lateral command; " + check.detail;
      }
    }
    ports_.set_docking_runtime_state(
      true, "PREDOCK_RESIDUAL_DELEGATED",
      "staging pose is inside the near-field capture window; robot_docking_manager is the "
      "single physical yaw/lateral/forward owner", true);
  } else if (yaw_aligned) {
    PredockYawAlignResult already_aligned;
    already_aligned.attempted = false;
    already_aligned.succeeded = true;
    already_aligned.actual_spin_required = false;
    already_aligned.failure_code = "NONE";
    already_aligned.initial_error_rad = check.base_yaw_error_rad;
    already_aligned.final_error_rad = check.base_yaw_error_rad;
    already_aligned.detail = "predock yaw verified after bridge smoothing";
    record_yaw(job_id, already_aligned);
  } else {
    if (!assessment.recovery_allowed || !yaw_fallback_allowed()) {
      PredockYawAlignResult failed;
      failed.attempted = false;
      failed.succeeded = false;
      failed.actual_spin_required = false;
      failed.initial_error_rad = check.base_yaw_error_rad;
      failed.final_error_rad = check.base_yaw_error_rad;
      failed.failure_code = !assessment.recovery_allowed ?
        "PREDOCK_POSE_DRIFTED_AFTER_BRIDGE_SETTLE" :
        "PREDOCK_YAW_NOT_ALIGNED_AFTER_BRIDGE_SETTLE";
      failed.detail =
        "post-bridge smoothing PREDOCK_POSE_VERIFY failed before fine docking; " +
        check.detail;
      record_yaw(job_id, failed);
      job_store_.set_phase(job_id, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE_FAILED");
      ports_.set_docking_runtime_state(
        true, "PREDOCK_POSE_VERIFY_AFTER_BRIDGE_SETTLE_FAILED", failed.detail, true);
      job_store_.finish_with_code(job_id, failed.failure_code, failed.detail);
      return false;
    }

    job_store_.set_phase(job_id, "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE");
    ports_.set_docking_runtime_state(
      true, "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE",
      "aligning staging yaw after bridge smoothing before GS2 handoff", true);
    auto yaw_result = run_yaw_align(
      job_id, check.expected_base_yaw, check,
      std::min(config_.yaw_align_tolerance_rad, config_.fine_entry_max_yaw_rad));
    record_yaw(job_id, yaw_result);
    if (!yaw_result.succeeded) {
      job_store_.finish_with_code(job_id, yaw_result.failure_code, yaw_result.detail);
      return false;
    }

    job_store_.set_phase(job_id, "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY");
    ports_.set_docking_runtime_state(
      true, "PREDOCK_YAW_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY",
      "verifying staging pose after post-bridge yaw alignment", true);
    std::this_thread::sleep_for(300ms);
    assessment = verify_staging_pose(job_id, job);
    check = assessment.check;
    yaw_aligned = assessment.yaw_aligned;
    if (!yaw_aligned) {
      job_store_.finish_with_code(
        job_id, "PREDOCK_YAW_NOT_ALIGNED_AFTER_BRIDGE_SETTLE",
        "post-bridge yaw alignment did not satisfy final staging pose check; " +
        check.detail);
      return false;
    }
  }

  if (!config_.delegate_staging_motion_to_manager) {
    if (!ensure_lateral_alignment(
        job_id, job, check, yaw_aligned,
        "PREDOCK_LATERAL_ALIGN_AFTER_BRIDGE_SETTLE",
        "PREDOCK_LATERAL_ALIGN_AFTER_BRIDGE_SETTLE_VERIFY",
        "aligning staging lateral after bridge settle with map->odom frozen"))
    {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(job_store_.mutex());
      auto & current = job_store_.job_unsafe();
      if (current.id == job_id && current.state == "running") {
        current.post_predock_settle_complete = true;
        current.dock_staging_handoff_ready = yaw_aligned;
      }
    }
  }

  job_store_.set_phase(job_id, "FINE_DOCKING_ENTRY_CHECK");
  ports_.set_docking_runtime_state(
    true, "FINE_DOCKING_ENTRY_CHECK",
    "checking active fine docking observation conditions", true);
  PredockPoseVerification fine_entry_check;
  std::string fine_entry_failure_code;
  std::string fine_entry_detail;
  bool fine_entry_ok = evaluate_fine_entry(
    job, yaw_aligned, fine_entry_check,
    fine_entry_failure_code, fine_entry_detail);
  bool retry_allowed = false;
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    const auto & current = job_store_.job_unsafe();
    retry_allowed = current.id == job_id && current.retry_count < current.max_retries;
  }
  if (!fine_entry_ok && config_.fine_retry_on_yaw_reject &&
    yaw_fallback_allowed() &&
    fine_entry_failure_code == "FINE_DOCKING_REJECTED_YAW_TOO_LARGE" &&
    retry_allowed)
  {
    job_store_.set_phase(job_id, "RESTAGE_RETRY");
    {
      std::lock_guard<std::mutex> lock(job_store_.mutex());
      auto & current = job_store_.job_unsafe();
      if (current.id == job_id && current.state == "running") {
        ++current.retry_count;
      }
    }
    job_store_.set_phase(job_id, "PREDOCK_YAW_ALIGN_RECOVERY");
    auto retry_yaw = run_yaw_align(
      job_id, fine_entry_check.expected_base_yaw, fine_entry_check,
      config_.fine_entry_max_yaw_rad);
    record_yaw(job_id, retry_yaw);
    yaw_aligned = retry_yaw.succeeded;
    if (!retry_yaw.succeeded) {
      fine_entry_ok = false;
      fine_entry_failure_code = retry_yaw.failure_code;
      fine_entry_detail = retry_yaw.detail;
    } else {
      fine_entry_ok = evaluate_fine_entry(
        job, yaw_aligned, fine_entry_check,
        fine_entry_failure_code, fine_entry_detail);
    }
  }
  record_fine_entry(
    job_id, fine_entry_check, fine_entry_ok,
    fine_entry_failure_code, fine_entry_detail);
  if (!fine_entry_ok) {
    resume_pause("fine_docking_entry_failed");
    job_store_.finish_with_code(
      job_id, fine_entry_failure_code, fine_entry_detail);
    return false;
  }
  if (job_store_.cancel_requested(job_id)) {
    resume_pause("fine_docking_canceled_before_start");
    job_store_.finish(
      job_id, true, "canceled", "docking canceled before GS2 fine docking start");
    return false;
  }

  std::string floor_detail;
  if (ports_.floor_runtime_operation_blocked("docking_fine_start", floor_detail)) {
    resume_pause("floor_transition_blocked");
    job_store_.finish(job_id, false, "failed", floor_detail);
    return false;
  }
  std::string service_detail;
  if (!ports_.start_fine_docking(service_detail)) {
    resume_pause("fine_docking_start_failed");
    job_store_.finish(job_id, false, "failed", service_detail);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    auto & current = job_store_.job_unsafe();
    if (current.id == job_id && current.state == "running") {
      current.docking_service_called = true;
      current.phase = "FINE_ALIGN";
      current.detail = service_detail;
    }
  }
  ports_.set_docking_runtime_state(true, "FINE_ALIGN", service_detail, true);
  return true;
}

}  // namespace robot_api_server::features::docking::predock_alignment
