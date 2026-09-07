#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace robot_api_server::features::navigation
{
namespace
{

double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp =
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp =
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double value)
{
  constexpr double kPi = 3.14159265358979323846;
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

}  // namespace

NavigationTerminalControl::NavigationTerminalControl(TerminalControlConfig config)
: config_(std::move(config))
{
}

const TerminalControlConfig & NavigationTerminalControl::config() const noexcept
{
  return config_;
}

double NavigationTerminalControl::speed_limit_for_distance(const double distance_m) const
{
  if (distance_m <= config_.speed_limit_crawl_distance_m) {
    return config_.speed_limit_final_mps;
  }
  if (distance_m <= config_.speed_limit_near_distance_m) {
    return config_.speed_limit_crawl_mps;
  }
  if (distance_m <= config_.speed_limit_mid_distance_m) {
    return config_.speed_limit_near_mps;
  }
  if (distance_m <= config_.speed_limit_far_distance_m) {
    return config_.speed_limit_mid_mps;
  }
  return config_.speed_limit_far_mps;
}

double NavigationTerminalControl::final_yaw_command_speed(
  const double signed_yaw_error_rad) const
{
  const double yaw_abs = std::fabs(signed_yaw_error_rad);
  const double max_speed =
    yaw_abs <= config_.final_yaw_slowdown_start_rad ?
    config_.final_yaw_slow_max_speed_radps :
    config_.final_yaw_max_speed_radps;
  return std::clamp(
    std::fabs(config_.final_yaw_kp * signed_yaw_error_rad),
    config_.final_yaw_min_speed_radps,
    max_speed);
}

double NavigationTerminalControl::yaw_stop_threshold(
  const double command_speed_radps,
  const double success_tolerance_rad) const
{
  if (!config_.yaw_stop_lead_enabled) {
    return success_tolerance_rad;
  }
  const double lead = std::clamp(
    std::fabs(command_speed_radps) * config_.yaw_stop_lead_time_sec,
    config_.yaw_stop_lead_min_rad,
    config_.yaw_stop_lead_max_rad);
  return std::max(success_tolerance_rad, lead);
}

void NavigationTerminalControl::goal_error_in_base_frame(
  const StoredPose & target,
  const RobotPoseSnapshot & pose,
  double & forward_m,
  double & lateral_m)
{
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  forward_m = c * dx + s * dy;
  lateral_m = -s * dx + c * dy;
}

LateralCorrectionGate NavigationTerminalControl::lateral_correction_gate(
  const StoredPose & target,
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy) const
{
  LateralCorrectionGate result;
  if (!config_.final_verify_enabled ||
    !config_.api_velocity_correction_enabled ||
    !config_.lateral_correction_enabled ||
    goal_completion_policy != "pose_required")
  {
    result.reason = "terminal lateral correction disabled by policy";
    return result;
  }
  if (!check.pose_available) {
    result.reason = "terminal lateral correction requires fresh final pose";
    return result;
  }
  if (check.distance_m > config_.terminal_recovery_max_distance_m) {
    result.reason = "terminal lateral correction gate not met: " + check.reason;
    return result;
  }

  goal_error_in_base_frame(target, check.pose, result.forward_m, result.lateral_m);
  if (std::fabs(result.forward_m) > config_.lateral_max_forward_m) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "terminal lateral correction skipped: forward_error=" << result.forward_m
        << " max_forward=" << config_.lateral_max_forward_m;
    result.reason = out.str();
    return result;
  }

  const double forward_target_m = config_.lateral_target_m * 0.5;
  const bool lateral_hysteresis_entry =
    std::fabs(result.lateral_m) > config_.lateral_trigger_m;
  const bool needs_terminal_xy_correction =
    lateral_hysteresis_entry ||
    (check.distance_m > config_.goal_position_success_tolerance_m &&
    (std::fabs(result.forward_m) > forward_target_m ||
    std::fabs(result.lateral_m) > config_.lateral_target_m));
  const bool needs_terminal_yaw_correction =
    check.yaw_error_rad > config_.final_yaw_success_tolerance_rad;
  if (!needs_terminal_xy_correction && !needs_terminal_yaw_correction) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "terminal pose correction skipped: distance=" << check.distance_m
        << " forward_error=" << result.forward_m
        << " lateral_error=" << result.lateral_m
        << " forward_target=" << forward_target_m
        << " lateral_target=" << config_.lateral_target_m
        << " lateral_trigger=" << config_.lateral_trigger_m
        << " yaw_error=" << check.yaw_error_rad
        << " yaw_target=" << config_.final_yaw_success_tolerance_rad;
    result.reason = out.str();
    return result;
  }

  result.allowed = true;
  return result;
}

TerminalCorrectionStep NavigationTerminalControl::correction_step(
  const double forward_error_m,
  const double lateral_error_m,
  const double signed_yaw_error_rad,
  const double lateral_direction_multiplier) const
{
  TerminalCorrectionStep result;
  const double yaw_abs = std::fabs(signed_yaw_error_rad);
  const double lateral_abs = std::fabs(lateral_error_m);
  const double forward_target_m = config_.lateral_target_m * 0.5;
  const bool correcting_yaw = yaw_abs > config_.final_yaw_success_tolerance_rad;
  const bool correcting_lateral = !correcting_yaw && lateral_abs > config_.lateral_target_m;
  const bool correcting_forward = !correcting_yaw && !correcting_lateral &&
    std::fabs(forward_error_m) > forward_target_m;
  const double min_speed = std::min(0.030, config_.lateral_speed_mps);

  if (correcting_yaw) {
    result.axis = TerminalCorrectionAxis::kYaw;
    result.command.angular.z = std::copysign(
      final_yaw_command_speed(signed_yaw_error_rad), signed_yaw_error_rad);
  } else if (correcting_lateral) {
    result.axis = TerminalCorrectionAxis::kLateral;
    const double speed = std::clamp(
      std::fabs(config_.lateral_kp * lateral_error_m),
      min_speed,
      config_.lateral_speed_mps);
    result.command.linear.y = lateral_direction_multiplier *
      std::copysign(speed, lateral_error_m);
  } else if (correcting_forward) {
    result.axis = TerminalCorrectionAxis::kForward;
    const double speed = std::clamp(
      std::fabs(config_.lateral_kp * forward_error_m),
      min_speed,
      config_.lateral_speed_mps);
    result.command.linear.x = std::copysign(speed, forward_error_m);
  }
  return result;
}

TerminalCostmapCheck NavigationTerminalControl::costmap_path_clear(
  const TerminalCostmapContext & context,
  const double forward_probe_m,
  const double lateral_probe_m,
  const std::chrono::steady_clock::time_point now) const
{
  TerminalCostmapCheck result;
  if (!config_.costmap_guard_enabled) {
    result.clear = true;
    result.detail = "terminal recovery local costmap guard disabled";
    return result;
  }
  if (!context.grid ||
    context.grid_received_at == std::chrono::steady_clock::time_point{})
  {
    result.detail = "terminal recovery blocked: local costmap unavailable";
    return result;
  }

  const double grid_age_sec =
    std::chrono::duration<double>(now - context.grid_received_at).count();
  if (grid_age_sec > config_.costmap_max_age_sec) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "terminal recovery blocked: local costmap stale"
        << " age=" << grid_age_sec
        << " max_age=" << config_.costmap_max_age_sec;
    result.detail = out.str();
    return result;
  }
  if (!context.robot_pose_available ||
    context.robot_pose_received_at == std::chrono::steady_clock::time_point{})
  {
    result.detail = "terminal recovery blocked: no robot pose in local costmap frame " +
      context.grid_frame;
    return result;
  }

  const double pose_age_sec =
    std::chrono::duration<double>(now - context.robot_pose_received_at).count();
  if (pose_age_sec > config_.costmap_max_age_sec) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "terminal recovery blocked: costmap-frame robot pose stale"
        << " frame=" << context.grid_frame
        << " age=" << pose_age_sec
        << " max_age=" << config_.costmap_max_age_sec;
    result.detail = out.str();
    return result;
  }

  const auto & info = context.grid->info;
  const std::size_t expected_cells =
    static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
  if (info.resolution <= 0.0 || info.width == 0U || info.height == 0U ||
    context.grid->data.size() < expected_cells)
  {
    result.detail = "terminal recovery blocked: malformed local costmap";
    return result;
  }

  const double c = std::cos(context.robot_yaw);
  const double s = std::sin(context.robot_yaw);
  const double end_x = context.robot_x + c * forward_probe_m - s * lateral_probe_m;
  const double end_y = context.robot_y + s * forward_probe_m + c * lateral_probe_m;
  const double origin_yaw = quaternion_yaw(info.origin.orientation);
  const double origin_c = std::cos(origin_yaw);
  const double origin_s = std::sin(origin_yaw);
  const double probe_distance_m = std::hypot(forward_probe_m, lateral_probe_m);
  const double sample_spacing_m = std::max(0.01, static_cast<double>(info.resolution) * 0.5);
  const int sample_count = std::max(
    1, static_cast<int>(std::ceil(probe_distance_m / sample_spacing_m)));
  int max_cost = 0;

  for (int sample = 0; sample <= sample_count; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(sample_count);
    const double world_x = context.robot_x + ratio * (end_x - context.robot_x);
    const double world_y = context.robot_y + ratio * (end_y - context.robot_y);
    const double dx = world_x - info.origin.position.x;
    const double dy = world_y - info.origin.position.y;
    const double local_x = origin_c * dx + origin_s * dy;
    const double local_y = -origin_s * dx + origin_c * dy;
    const int cell_x = static_cast<int>(std::floor(local_x / info.resolution));
    const int cell_y = static_cast<int>(std::floor(local_y / info.resolution));
    if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(info.width) || cell_y >= static_cast<int>(info.height))
    {
      result.detail = "terminal recovery blocked: probe leaves local costmap bounds";
      return result;
    }
    const std::size_t index =
      static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(info.width) +
      static_cast<std::size_t>(cell_x);
    const int cost = static_cast<int>(context.grid->data[index]);
    if (cost < 0 || cost >= config_.costmap_occupied_threshold) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "terminal recovery blocked by local costmap"
          << " frame=" << context.grid_frame
          << " sample=" << sample << "/" << sample_count
          << " cost=" << cost
          << " occupied_threshold=" << config_.costmap_occupied_threshold
          << " forward_probe=" << forward_probe_m
          << " lateral_probe=" << lateral_probe_m;
      result.detail = out.str();
      return result;
    }
    max_cost = std::max(max_cost, cost);
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "terminal recovery local costmap path clear"
      << " frame=" << context.grid_frame
      << " age=" << grid_age_sec
      << " pose_age=" << pose_age_sec
      << " samples=" << (sample_count + 1)
      << " max_cost=" << max_cost;
  result.clear = true;
  result.detail = out.str();
  return result;
}

void NavigationTerminalControl::publish_zero_burst(
  NavigationTerminalRuntimePort & runtime) const
{
  const geometry_msgs::msg::Twist zero;
  for (int index = 0; index < config_.zero_command_count; ++index) {
    runtime.terminal_publish_command(zero);
    runtime.terminal_sleep_for(std::chrono::milliseconds(40));
  }
}

TerminalLateralCorrectionResult NavigationTerminalControl::run_lateral_correction(
  const std::uint64_t job_id,
  const StoredPose & target,
  const FinalPoseCheck & initial_check,
  const std::string & goal_completion_policy,
  NavigationTerminalRuntimePort & runtime) const
{
  TerminalLateralCorrectionResult result;
  const auto gate = lateral_correction_gate(target, initial_check, goal_completion_policy);
  double forward_m = gate.forward_m;
  double lateral_m = gate.lateral_m;
  if (!gate.allowed) {
    result.detail = gate.reason;
    return result;
  }

  result.attempted = true;
  result.initial_forward_m = forward_m;
  result.initial_lateral_m = lateral_m;
  result.final_forward_m = forward_m;
  result.final_lateral_m = lateral_m;
  result.final_distance_m = initial_check.distance_m;
  runtime.terminal_set_job_phase(
    job_id,
    "terminal_pose_correcting",
    "running deterministic terminal pose correction after Nav2 final verify");

  const auto started = runtime.terminal_now();
  const auto deadline = started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.lateral_timeout_sec));
  const auto tick = std::chrono::milliseconds(67);
  int success_hold = 0;
  double direction_multiplier = config_.lateral_command_sign;
  const double initial_lateral_abs = std::fabs(lateral_m);
  double correction_phase_initial_lateral_abs = initial_lateral_abs;
  const double reversal_min_progress_m =
    std::max(0.020, config_.lateral_divergence_epsilon_m);
  double best_lateral_abs = initial_lateral_abs;
  int divergence_count = 0;
  bool direction_reversed = false;
  bool nonzero_command_published = false;
  const bool reverse_permit_requested =
    config_.reverse_permit_enabled && runtime.terminal_reverse_permit_available();
  const auto refresh_reverse_permit = [&]() {
      if (reverse_permit_requested) {
        runtime.terminal_publish_reverse_permit(true);
      }
    };
  const auto clear_reverse_permit = [&]() {
      if (reverse_permit_requested) {
        runtime.terminal_publish_reverse_permit(false);
      }
    };
  const auto strict_terminal_pose_reached = [&](const FinalPoseCheck & check,
      const double lateral_error) {
      return check.pose_available &&
        check.distance_m <= config_.goal_position_success_tolerance_m &&
        std::fabs(lateral_error) <= config_.lateral_target_m &&
        check.yaw_error_rad <= config_.final_yaw_success_tolerance_rad;
    };

  refresh_reverse_permit();
  while (runtime.terminal_now() < deadline) {
    refresh_reverse_permit();
    std::string cancel_detail;
    if (runtime.terminal_cancel_requested(job_id, cancel_detail)) {
      result.canceled = true;
      result.blocked = true;
      result.detail = "terminal lateral correction canceled: " + cancel_detail;
      break;
    }

    std::string safety_detail;
    if (runtime.terminal_safety_hard_blocked(safety_detail)) {
      result.blocked = true;
      result.detail = "terminal lateral correction blocked by safety: " + safety_detail;
      break;
    }

    auto check = runtime.terminal_verify_final_pose(target, true);
    runtime.terminal_update_final_pose(job_id, check, check.reason);
    if (!check.pose_available) {
      result.blocked = true;
      result.detail = "terminal lateral correction has no fresh final pose: " + check.reason;
      break;
    }
    goal_error_in_base_frame(target, check.pose, forward_m, lateral_m);
    result.final_forward_m = forward_m;
    result.final_lateral_m = lateral_m;
    result.final_distance_m = check.distance_m;
    const double lateral_abs = std::fabs(lateral_m);
    const double signed_yaw_error = normalize_angle(target.yaw - check.pose.yaw);
    const double yaw_abs = std::fabs(signed_yaw_error);

    if (strict_terminal_pose_reached(check, lateral_m)) {
      ++success_hold;
      runtime.terminal_publish_command(geometry_msgs::msg::Twist{});
      if (success_hold >= 2) {
        runtime.terminal_set_job_phase(
          job_id,
          "terminal_pose_settling",
          "terminal pose is inside the strict gate; waiting for wheel stop and DUAL_ACKERMAN mode exit");
        runtime.terminal_reset_actual_stop_stability();
        publish_zero_burst(runtime);
        runtime.terminal_publish_motion_mode(config_.lateral_release_mode);
        clear_reverse_permit();
        const auto settle_started = runtime.terminal_now();
        std::string settle_detail;
        const bool settled = runtime.terminal_wait_actual_stop(
          "terminal pose settle timeout", settle_detail);
        const double settle_elapsed =
          std::chrono::duration<double>(runtime.terminal_now() - settle_started).count();
        if (result.settle_duration_sec < 0.0) {
          result.settle_duration_sec = 0.0;
        }
        result.settle_duration_sec += settle_elapsed;
        result.settle_confirmed = settled;
        result.settle_detail = settle_detail;

        auto settled_check = runtime.terminal_verify_final_pose(target, true);
        runtime.terminal_update_final_pose(job_id, settled_check, settled_check.reason);
        if (!settled_check.pose_available) {
          result.blocked = true;
          result.detail = "terminal pose settle completed without a fresh final pose: " +
            settled_check.reason + "; " + settle_detail;
          break;
        }
        goal_error_in_base_frame(target, settled_check.pose, forward_m, lateral_m);
        result.final_forward_m = forward_m;
        result.final_lateral_m = lateral_m;
        result.final_distance_m = settled_check.distance_m;
        result.settled_forward_m = forward_m;
        result.settled_lateral_m = lateral_m;
        result.settled_distance_m = settled_check.distance_m;

        if (!settled) {
          result.blocked = true;
          result.detail = "terminal pose did not reach physical settle gate: " + settle_detail;
          break;
        }
        if (strict_terminal_pose_reached(settled_check, lateral_m)) {
          result.succeeded = true;
          result.detail = direction_reversed ?
            "terminal lateral correction reached strict settled pose gate after reversing divergent side-slip direction" :
            "terminal lateral correction reached strict settled pose gate";
          break;
        }

        const bool settled_pose_inside_correction_gate =
          settled_check.distance_m <= config_.terminal_recovery_max_distance_m &&
          std::fabs(forward_m) <= config_.lateral_max_forward_m;
        if (settled_pose_inside_correction_gate &&
          result.settle_recheck_count < config_.terminal_settle_max_recheck_count)
        {
          ++result.settle_recheck_count;
          success_hold = 0;
          divergence_count = 0;
          nonzero_command_published = false;
          correction_phase_initial_lateral_abs = std::fabs(lateral_m);
          best_lateral_abs = correction_phase_initial_lateral_abs;
          refresh_reverse_permit();
          continue;
        }

        std::ostringstream out;
        out << std::fixed << std::setprecision(3)
            << "terminal pose moved outside strict gate after physical settle"
            << " distance=" << settled_check.distance_m
            << " forward_error=" << forward_m
            << " lateral_error=" << lateral_m
            << " yaw_error=" << settled_check.yaw_error_rad
            << " rechecks=" << result.settle_recheck_count
            << "/" << config_.terminal_settle_max_recheck_count
            << " settle=" << settle_detail;
        result.blocked = true;
        result.detail = out.str();
        break;
      }
      runtime.terminal_sleep_for(tick);
      continue;
    }
    success_hold = 0;

    if (check.distance_m > config_.terminal_recovery_max_distance_m ||
      std::fabs(forward_m) > config_.lateral_max_forward_m)
    {
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "terminal pose correction exited gate"
          << " distance=" << check.distance_m
          << " forward_error=" << forward_m
          << " lateral_error=" << lateral_m
          << " yaw_error=" << yaw_abs
          << " max_forward=" << config_.lateral_max_forward_m;
      result.blocked = true;
      result.detail = out.str();
      break;
    }

    const double forward_target_m = config_.lateral_target_m * 0.5;
    const auto step = correction_step(
      forward_m, lateral_m, signed_yaw_error, direction_multiplier);
    const bool correcting_lateral = step.axis == TerminalCorrectionAxis::kLateral;
    if (correcting_lateral && lateral_abs + 1e-6 < best_lateral_abs) {
      best_lateral_abs = lateral_abs;
      divergence_count = 0;
    } else if (correcting_lateral && nonzero_command_published &&
      lateral_abs > best_lateral_abs + config_.lateral_divergence_epsilon_m)
    {
      ++divergence_count;
    } else {
      divergence_count = 0;
    }

    if (nonzero_command_published &&
      divergence_count >= config_.lateral_divergence_count)
    {
      publish_zero_burst(runtime);
      const bool no_meaningful_progress =
        best_lateral_abs > correction_phase_initial_lateral_abs - reversal_min_progress_m;
      if (!direction_reversed && no_meaningful_progress) {
        direction_multiplier *= -1.0;
        direction_reversed = true;
        divergence_count = 0;
        best_lateral_abs = lateral_abs;
        continue;
      }
      if (!direction_reversed && !no_meaningful_progress) {
        divergence_count = 0;
        std::ostringstream warning;
        warning << std::fixed << std::setprecision(3)
                << "terminal lateral correction kept original side-slip direction after meaningful progress"
                << " initial_lateral=" << correction_phase_initial_lateral_abs
                << " best_lateral=" << best_lateral_abs
                << " current_lateral=" << lateral_abs
                << " min_progress=" << reversal_min_progress_m;
        runtime.terminal_warn(warning.str());
        continue;
      }
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "terminal lateral correction diverged after side-slip command"
          << " best_lateral=" << best_lateral_abs
          << " current_lateral=" << lateral_abs
          << " divergence_epsilon=" << config_.lateral_divergence_epsilon_m
          << " direction_reversed=true";
      result.blocked = true;
      result.detail = out.str();
      break;
    }

    std::string phase;
    std::ostringstream phase_detail;
    phase_detail << std::fixed << std::setprecision(3);
    if (step.axis == TerminalCorrectionAxis::kYaw) {
      runtime.terminal_publish_motion_mode(config_.lateral_release_mode);
      phase = "terminal_pose_yaw_aligning";
      phase_detail << "terminal pose correction: yaw first"
                   << " yaw_error=" << yaw_abs
                   << " forward_error=" << forward_m
                   << " lateral_error=" << lateral_m
                   << " cmd_wz=" << step.command.angular.z;
    } else if (step.axis == TerminalCorrectionAxis::kLateral) {
      runtime.terminal_publish_motion_mode(config_.lateral_forced_mode);
      phase = "terminal_pose_lateral_correcting";
      phase_detail << "terminal pose correction: lateral second"
                   << " yaw_error=" << yaw_abs
                   << " forward_error=" << forward_m
                   << " lateral_error=" << lateral_m
                   << " cmd_y=" << step.command.linear.y;
    } else if (step.axis == TerminalCorrectionAxis::kForward) {
      runtime.terminal_publish_motion_mode(config_.lateral_release_mode);
      phase = "terminal_pose_forward_correcting";
      phase_detail << "terminal pose correction: forward/reverse third"
                   << " yaw_error=" << yaw_abs
                   << " forward_error=" << forward_m
                   << " lateral_error=" << lateral_m
                   << " cmd_x=" << step.command.linear.x;
    }
    if (!phase.empty()) {
      runtime.terminal_set_job_phase(job_id, phase, phase_detail.str());
    }

    if (std::fabs(step.command.linear.x) > 1e-6 ||
      std::fabs(step.command.linear.y) > 1e-6)
    {
      double forward_probe_m = 0.0;
      double lateral_probe_m = 0.0;
      if (std::fabs(step.command.linear.y) > 1e-6) {
        const double remaining_m = std::max(0.0, lateral_abs - config_.lateral_target_m);
        lateral_probe_m = std::copysign(
          std::min(config_.costmap_lookahead_m, remaining_m), step.command.linear.y);
      } else {
        const double remaining_m =
          std::max(0.0, std::fabs(forward_m) - forward_target_m);
        forward_probe_m = std::copysign(
          std::min(config_.costmap_lookahead_m, remaining_m), step.command.linear.x);
      }
      const auto costmap_check = costmap_path_clear(
        runtime.terminal_costmap_context(runtime.terminal_now()),
        forward_probe_m,
        lateral_probe_m,
        runtime.terminal_now());
      if (!costmap_check.clear) {
        result.blocked = true;
        result.detail = costmap_check.detail;
        break;
      }
    }
    runtime.terminal_publish_command(step.command);
    nonzero_command_published =
      std::fabs(step.command.linear.x) > 1e-6 ||
      std::fabs(step.command.linear.y) > 1e-6;
    runtime.terminal_sleep_for(tick);
  }

  publish_zero_burst(runtime);
  runtime.terminal_publish_motion_mode(config_.lateral_release_mode);
  clear_reverse_permit();
  result.direction_reversed = direction_reversed;
  result.duration_sec =
    std::chrono::duration<double>(runtime.terminal_now() - started).count();
  if (result.detail.empty()) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "terminal lateral correction timed out"
        << " final_distance=" << result.final_distance_m
        << " final_forward=" << result.final_forward_m
        << " final_lateral=" << result.final_lateral_m
        << " direction_reversed=" << (direction_reversed ? "true" : "false");
    result.detail = out.str();
  }
  return result;
}

FinalYawAlignResult NavigationTerminalControl::run_final_yaw_motion(
  const std::uint64_t job_id,
  const StoredPose & target,
  const FinalPoseCheck & initial_check,
  NavigationTerminalRuntimePort & runtime) const
{
  FinalYawAlignResult result;
  result.attempted = true;
  result.initial_yaw_error_rad = initial_check.yaw_error_rad;
  result.final_yaw_error_rad = initial_check.yaw_error_rad;

  const auto started = runtime.terminal_now();
  const auto deadline = started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.final_yaw_timeout_sec));
  const auto tick = std::chrono::milliseconds(67);

  while (runtime.terminal_now() < deadline) {
    std::string cancel_detail;
    if (runtime.terminal_cancel_requested(job_id, cancel_detail)) {
      result.canceled = true;
      result.blocked = true;
      result.phase = "canceled";
      result.blocked_reason = "canceled";
      result.detail = "final yaw alignment canceled: " + cancel_detail;
      break;
    }

    std::string safety_detail;
    if (runtime.terminal_safety_hard_blocked(safety_detail)) {
      result.blocked = true;
      result.phase = "blocked_by_safety";
      result.blocked_reason = safety_detail;
      result.detail = "final yaw alignment blocked by safety: " + safety_detail;
      break;
    }

    std::string dock_detail;
    if (runtime.terminal_dock_contact_blocked(dock_detail)) {
      result.blocked = true;
      result.phase = "blocked_by_docked_contact";
      result.blocked_reason = "DOCKED_OR_CHARGING_CONTACT";
      result.detail = "final yaw alignment stopped by dock/contact gate: " + dock_detail;
      break;
    }

    const auto pose = runtime.terminal_current_robot_pose();
    if (!pose.available || pose.frame_id != config_.map_frame) {
      result.blocked = true;
      result.phase = "failed_final_yaw_align";
      result.blocked_reason = "no map-frame robot pose";
      result.detail = "final yaw alignment has no fresh pose: " + result.blocked_reason;
      break;
    }
    if (config_.final_yaw_require_fresh_pose &&
      pose.age_sec > config_.robot_pose_freshness_sec)
    {
      result.blocked = true;
      result.phase = "failed_final_yaw_align";
      result.blocked_reason = "stale map-frame robot pose";
      result.detail = "final yaw alignment has no fresh pose: " + result.blocked_reason;
      break;
    }

    const double xy_drift =
      std::hypot(pose.x - initial_check.pose.x, pose.y - initial_check.pose.y);
    result.observed_xy_drift_m = std::max(result.observed_xy_drift_m, xy_drift);
    if (xy_drift > config_.final_yaw_max_xy_drift_m) {
      result.blocked = true;
      result.phase = "failed_final_pose_verify";
      result.blocked_reason = "max_xy_drift_exceeded";
      std::ostringstream detail;
      detail << std::fixed << std::setprecision(3)
             << "final yaw alignment stopped because xy drift=" << xy_drift
             << " max=" << config_.final_yaw_max_xy_drift_m;
      result.detail = detail.str();
      break;
    }

    const double signed_error = normalize_angle(target.yaw - pose.yaw);
    result.final_yaw_error_rad = std::fabs(signed_error);
    const double command_speed = final_yaw_command_speed(signed_error);
    const double stop_threshold = yaw_stop_threshold(
      command_speed, config_.final_yaw_success_tolerance_rad);
    if (result.final_yaw_error_rad <= stop_threshold) {
      publish_zero_burst(runtime);
      std::string stop_detail;
      const bool actual_stopped = runtime.terminal_wait_yaw_actual_stop(
        "final_yaw_align", stop_detail);
      const auto settled_pose = runtime.terminal_current_robot_pose();
      if (!settled_pose.available || settled_pose.frame_id != config_.map_frame) {
        result.blocked = true;
        result.phase = "failed_final_yaw_align";
        result.blocked_reason = "no map-frame robot pose after actual stop wait";
        result.detail = "final yaw alignment could not verify pose after actual stop wait: " +
          stop_detail;
        break;
      }
      if (config_.final_yaw_require_fresh_pose &&
        settled_pose.age_sec > config_.robot_pose_freshness_sec)
      {
        result.blocked = true;
        result.phase = "failed_final_yaw_align";
        result.blocked_reason = "stale map-frame robot pose after actual stop wait";
        result.detail = "final yaw alignment pose is stale after actual stop wait: " + stop_detail;
        break;
      }
      const double settled_xy_drift = std::hypot(
        settled_pose.x - initial_check.pose.x,
        settled_pose.y - initial_check.pose.y);
      result.observed_xy_drift_m =
        std::max(result.observed_xy_drift_m, settled_xy_drift);
      if (settled_xy_drift > config_.final_yaw_max_xy_drift_m) {
        result.blocked = true;
        result.phase = "failed_final_pose_verify";
        result.blocked_reason = "max_xy_drift_exceeded_after_actual_stop_wait";
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
               << "final yaw alignment stopped after actual stop wait because xy drift="
               << settled_xy_drift << " max=" << config_.final_yaw_max_xy_drift_m
               << "; " << stop_detail;
        result.detail = detail.str();
        break;
      }
      result.final_yaw_error_rad =
        std::fabs(normalize_angle(target.yaw - settled_pose.yaw));
      if (result.final_yaw_error_rad <= config_.final_yaw_success_tolerance_rad) {
        result.succeeded = true;
        result.phase = "final_pose_verifying";
        result.detail = actual_stopped ?
          "final yaw aligned after actual angular velocity settled: " + stop_detail :
          "final yaw aligned after stop-wait timeout and pose recheck: " + stop_detail;
        break;
      }
      runtime.terminal_sleep_for(tick);
      continue;
    }

    geometry_msgs::msg::Twist command;
    command.angular.z = std::copysign(command_speed, signed_error);
    runtime.terminal_reset_yaw_actual_stop_stability();
    runtime.terminal_publish_command(command);
    runtime.terminal_sleep_for(tick);
  }

  result.duration_sec =
    std::chrono::duration<double>(runtime.terminal_now() - started).count();
  publish_zero_burst(runtime);
  if (!result.succeeded && !result.blocked) {
    result.blocked = true;
    result.phase = "failed_final_yaw_align";
    result.blocked_reason = "timeout";
    result.detail = "final yaw alignment timed out";
  }
  return result;
}

std::string NavigationTerminalControl::lateral_correction_diagnostics(
  const TerminalLateralCorrectionResult & result)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(4)
      << "attempted=" << (result.attempted ? "true" : "false")
      << " succeeded=" << (result.succeeded ? "true" : "false")
      << " blocked=" << (result.blocked ? "true" : "false")
      << " initial_forward=" << result.initial_forward_m
      << " initial_lateral=" << result.initial_lateral_m
      << " final_forward=" << result.final_forward_m
      << " final_lateral=" << result.final_lateral_m
      << " final_distance=" << result.final_distance_m
      << " settled_forward=" << result.settled_forward_m
      << " settled_lateral=" << result.settled_lateral_m
      << " settled_distance=" << result.settled_distance_m
      << " settle_confirmed=" << (result.settle_confirmed ? "true" : "false")
      << " settle_duration_sec=" << result.settle_duration_sec
      << " settle_recheck_count=" << result.settle_recheck_count
      << " direction_reversed=" << (result.direction_reversed ? "true" : "false")
      << " duration_sec=" << result.duration_sec
      << " settle_detail=" << result.settle_detail
      << " detail=" << result.detail;
  return out.str();
}

}  // namespace robot_api_server::features::navigation
