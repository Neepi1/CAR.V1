#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::localization
{

class PostRelocalizationSettleModule::Impl
{
public:
  Impl(
    PostRelocalizationSettleConfig config,
    PostRelocalizationSettlePorts ports)
  : config_(std::move(config)), ports_(std::move(ports))
  {
    if (!ports_.bridge_status_snapshot || !ports_.tf_chain_freshness_snapshot ||
      !ports_.tf_chain_freshness_detail || !ports_.base_to_lidar_static_tf_ready ||
      !ports_.local_costmap_update_count ||
      !ports_.local_costmap_message_filter_drop_count ||
      !ports_.last_local_costmap_message_filter_drop_text ||
      !ports_.clear_teleop_command || !ports_.publish_teleop_zero_burst ||
      !ports_.publish_navigation_zero_burst || !ports_.steady_now ||
      !ports_.wall_time_seconds || !ports_.sleep_for)
    {
      throw std::invalid_argument(
              "post-relocalization settle module requires every integration port");
    }
  }

  PostRelocalizationSettleResult wait_for_settle(
    const std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested)
  {
    PostRelocalizationSettleResult result;
    result.expected_sequence = expected_sequence;
    const bool is_post_undock = reason == "post_undock";
    const bool settle_enabled = is_post_undock ? config_.post_undock_enabled : config_.enabled;
    const int configured_min_ms = is_post_undock ? config_.post_undock_min_ms : config_.min_ms;
    const int configured_max_ms = is_post_undock ? config_.post_undock_max_ms : config_.max_ms;
    const int configured_stable_tf_samples = is_post_undock ?
      config_.post_undock_stable_tf_samples : config_.stable_tf_samples;
    const int configured_tf_sample_period_ms = is_post_undock ?
      config_.post_undock_tf_sample_period_ms : config_.tf_sample_period_ms;
    const int configured_required_local_costmap_updates = is_post_undock ?
      config_.post_undock_required_local_costmap_updates :
      config_.required_local_costmap_updates;
    const bool configured_reject_if_new_message_filter_drop = is_post_undock ?
      config_.post_undock_reject_if_new_message_filter_drop :
      config_.reject_if_new_message_filter_drop;
    const bool configured_zero_cmd = is_post_undock ?
      config_.post_undock_zero_cmd_during_settle : config_.zero_cmd;

    if (!settle_enabled) {
      result.ok = true;
      result.failure_code = "NONE";
      result.detail = "post relocalization settle disabled";
      return result;
    }

    if (expected_sequence == 0U) {
      result.failure_code = "POST_RELOCALIZATION_SEQUENCE_MISMATCH";
      result.detail =
        "post relocalization settle missing expected explicit relocalization sequence";
      return result;
    }

    const auto initial_bridge = ports_.bridge_status_snapshot();
    int min_ms = configured_min_ms;
    if (std::fabs(initial_bridge.last_accepted_correction_translation_m) >=
      config_.large_correction_translation_m ||
      std::fabs(initial_bridge.last_accepted_correction_yaw_rad) >=
      config_.large_correction_yaw_rad)
    {
      min_ms = std::max(min_ms, config_.large_correction_min_ms);
    }

    const auto start = ports_.steady_now();
    auto settle_start = start;
    auto deadline = start + std::chrono::milliseconds(configured_max_ms);
    auto min_deadline = start + std::chrono::milliseconds(min_ms);
    const auto sample_period = std::chrono::milliseconds(configured_tf_sample_period_ms);
    const std::uint64_t baseline_costmap_updates = ports_.local_costmap_update_count();
    const std::uint64_t baseline_local_costmap_drops =
      ports_.local_costmap_message_filter_drop_count();
    int stable_samples = 0;
    bool sequence_observed = false;
    auto mark_sequence_observed = [&]() {
        if (sequence_observed) {
          return;
        }
        sequence_observed = true;
        settle_start = ports_.steady_now();
        deadline = settle_start + std::chrono::milliseconds(configured_max_ms);
        min_deadline = settle_start + std::chrono::milliseconds(min_ms);
        stable_samples = 0;
      };
    std::string last_failure_code = "POST_RELOCALIZATION_SETTLE_TIMEOUT";
    std::string last_detail = "waiting for first settle sample";
    bool last_sample_ok = false;

    update_state(
      [&](PostRelocalizationSettleState & state) {
        state.required = true;
        state.in_progress = true;
        state.complete = false;
        state.reason = reason;
        state.target_stage = target_next_stage;
        state.failure_reason = "none";
        state.detail = "waiting for post relocalization settle";
        state.expected_sequence = expected_sequence;
        state.min_ms = min_ms;
        state.start_wall_time = ports_.wall_time_seconds();
      });

    if (configured_zero_cmd) {
      publish_zero_commands();
    }

    while (ports_.steady_now() <= deadline) {
      if (configured_zero_cmd) {
        publish_zero_commands();
      }

      std::string cancel_detail;
      if (cancel_requested && cancel_requested(cancel_detail)) {
        result.failure_code = "CANCELLED_BY_APP";
        result.detail = cancel_detail.empty() ?
          "post relocalization settle canceled by app" : cancel_detail;
        update_failure_state(result);
        return result;
      }

      const auto bridge = ports_.bridge_status_snapshot();
      const auto tf = ports_.tf_chain_freshness_snapshot();
      const std::uint64_t costmap_updates =
        ports_.local_costmap_update_count() - baseline_costmap_updates;
      result.observed_sequence = bridge.last_explicit_relocalization_sequence;
      result.local_costmap_updates = costmap_updates;

      bool sample_ok = true;
      std::ostringstream sample_detail;
      sample_detail << std::fixed << std::setprecision(3)
                    << "reason=" << reason
                    << " target=" << target_next_stage
                    << " expected_seq=" << expected_sequence
                    << " observed_seq=" << bridge.last_explicit_relocalization_sequence
                    << " bridge_age_sec=" << bridge.age_sec
                    << " map_to_odom_age_ms=" << bridge.map_to_odom_age_ms
                    << " map_odom_pub_hz=" << bridge.map_odom_publish_loop_hz
                    << " map_odom_pub_gap_ms=" << bridge.map_odom_publish_gap_ms
                    << " map_odom_pub_seq=" << bridge.map_odom_last_published_sequence
                    << "/" << bridge.map_odom_latest_accepted_sequence
                    << " costmap_updates=" << costmap_updates
                    << " stable_samples=" << stable_samples;

      if (!bridge.available) {
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; bridge_status unavailable";
      } else if (bridge.map_to_odom_publisher_owner != "robot_localization_bridge") {
        result.failure_code = "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER";
        result.detail = "post relocalization settle rejected wrong map->odom owner: " +
          bridge.map_to_odom_publisher_owner;
        update_failure_state(result);
        return result;
      } else if (bridge.last_explicit_relocalization_sequence != expected_sequence) {
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_SEQUENCE_MISMATCH";
        sample_detail << "; sequence mismatch";
        if (bridge.last_explicit_relocalization_sequence > expected_sequence) {
          result.failure_code = last_failure_code;
          result.detail = sample_detail.str();
          update_failure_state(result);
          return result;
        }
      } else if (!bridge.publisher_decoupled_from_correction) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_PUBLISHER_NOT_DECOUPLED";
        sample_detail << "; bridge publisher is not decoupled from correction callbacks";
      } else if (!bridge.has_map_to_odom) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; bridge has no map->odom";
      } else if (!bridge.map_odom_state_valid) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; bridge map_odom_state is not valid";
      } else if (bridge.map_odom_publish_gap_ms < 0.0 ||
        bridge.map_odom_publish_gap_ms > config_.map_odom_publish_gap_fail_ms)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_GAP";
        sample_detail << "; bridge map->odom publish gap "
                      << bridge.map_odom_publish_gap_ms
                      << "ms exceeds fail threshold "
                      << config_.map_odom_publish_gap_fail_ms << "ms";
      } else if (bridge.map_odom_latest_accepted_sequence > 0U &&
        bridge.map_odom_last_published_sequence < bridge.map_odom_latest_accepted_sequence)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_SEQUENCE_LAG";
        sample_detail << "; bridge map->odom publish sequence lag: published="
                      << bridge.map_odom_last_published_sequence
                      << " accepted=" << bridge.map_odom_latest_accepted_sequence;
      } else if (bridge.correction_active || !bridge.safe_for_goal_start) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_CORRECTION_ACTIVE";
        sample_detail << "; bridge correction still active or unsafe for goal start"
                      << " correction_active=" << (bridge.correction_active ? "true" : "false")
                      << " safe_for_goal_start=" <<
          (bridge.safe_for_goal_start ? "true" : "false")
                      << " remaining_translation_error_m=" <<
          bridge.remaining_translation_error_m
                      << " remaining_yaw_error_rad=" << bridge.remaining_yaw_error_rad;
      } else if (!tf.have_map_to_odom || tf.map_to_odom_age_sec < 0.0 ||
        tf.map_to_odom_age_sec > config_.tf_chain_freshness_sec)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; api tf map->odom not fresh: " <<
          ports_.tf_chain_freshness_detail(tf);
      } else if (!tf.have_odom_to_base || tf.odom_to_base_age_sec < 0.0 ||
        tf.odom_to_base_age_sec > config_.tf_chain_freshness_sec)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH";
        sample_detail << "; api tf odom->base_link not fresh: " <<
          ports_.tf_chain_freshness_detail(tf);
      } else if (!tf.have_map_pose || tf.map_pose_age_sec < 0.0 ||
        tf.map_pose_age_sec > config_.robot_pose_freshness_sec)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE";
        sample_detail << "; api map pose not fresh: " << ports_.tf_chain_freshness_detail(tf);
      } else if (!ports_.base_to_lidar_static_tf_ready()) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE";
        sample_detail << "; static " << config_.base_frame << "->"
                      << config_.static_lidar_frame << " not observed";
      } else if (config_.require_local_costmap_update &&
        costmap_updates < static_cast<std::uint64_t>(
          configured_required_local_costmap_updates))
      {
        mark_sequence_observed();
        if (is_post_undock) {
          sample_detail << "; post-undock warning only: local_costmap updates below required="
                        << configured_required_local_costmap_updates;
        } else {
          sample_ok = false;
          last_failure_code = "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED";
          sample_detail << "; local_costmap updates below required="
                        << configured_required_local_costmap_updates;
        }
      } else if (configured_reject_if_new_message_filter_drop &&
        ports_.local_costmap_message_filter_drop_count() > baseline_local_costmap_drops)
      {
        mark_sequence_observed();
        if (is_post_undock) {
          sample_detail << "; post-undock warning only: local_costmap MessageFilter drop: "
                        << ports_.last_local_costmap_message_filter_drop_text();
        } else {
          sample_ok = false;
          last_failure_code = "POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS";
          sample_detail << "; local_costmap MessageFilter drop: "
                        << ports_.last_local_costmap_message_filter_drop_text();
        }
      } else if (bridge.amcl_scan_admission_enabled &&
        (!bridge.amcl_scan_admission_alive || bridge.amcl_message_filter_drop_detected ||
        lower_copy(bridge.amcl_scan_admission_last_error).find("tf") != std::string::npos ||
        lower_copy(bridge.amcl_scan_admission_last_error).find("transform") !=
        std::string::npos))
      {
        mark_sequence_observed();
        if (is_post_undock) {
          sample_detail << "; post-undock warning only: amcl scan admission not clean: alive="
                        << (bridge.amcl_scan_admission_alive ? "true" : "false")
                        << " error=" << bridge.amcl_scan_admission_last_error;
        } else {
          sample_ok = false;
          last_failure_code = "POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR";
          sample_detail << "; amcl scan admission not clean: alive="
                        << (bridge.amcl_scan_admission_alive ? "true" : "false")
                        << " error=" << bridge.amcl_scan_admission_last_error;
        }
      } else if (!sequence_observed) {
        mark_sequence_observed();
      }

      if (sample_ok) {
        ++stable_samples;
        last_sample_ok = true;
        last_failure_code = "POST_RELOCALIZATION_STABLE_SAMPLE_TIMEOUT";
      } else {
        stable_samples = 0;
        last_sample_ok = false;
      }
      last_detail = sample_detail.str();
      result.stable_samples = stable_samples;
      update_state(
        [&](PostRelocalizationSettleState & state) {
          state.detail = last_detail;
        });

      if (stable_samples >= configured_stable_tf_samples && ports_.steady_now() >= min_deadline) {
        result.ok = true;
        result.failure_code = "NONE";
        result.detail = last_detail + "; post relocalization settle passed";
        result.elapsed_ms =
          std::chrono::duration<double, std::milli>(ports_.steady_now() - start).count();
        update_success_state(result);
        return result;
      }

      ports_.sleep_for(sample_period);
    }

    if (is_post_undock && last_sample_ok) {
      result.ok = true;
      result.failure_code = "NONE";
      result.detail = last_detail + "; post-undock settle warning: stable_samples=" +
        std::to_string(stable_samples) + "/" +
        std::to_string(configured_stable_tf_samples) +
        " before timeout, but hard readiness conditions passed; releasing pending Nav2 goal";
      result.elapsed_ms =
        std::chrono::duration<double, std::milli>(ports_.steady_now() - start).count();
      update_success_state(result);
      return result;
    }

    result.failure_code = last_failure_code;
    result.detail = last_detail + "; timed out waiting for post relocalization settle";
    result.elapsed_ms =
      std::chrono::duration<double, std::milli>(ports_.steady_now() - start).count();
    update_failure_state(result);
    return result;
  }

  PostRelocalizationSettleState state_snapshot() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
  }

  std::string state_json() const
  {
    const auto state = state_snapshot();
    std::ostringstream out;
    out << "{\"required\":" << (state.required ? "true" : "false")
        << ",\"in_progress\":" << (state.in_progress ? "true" : "false")
        << ",\"complete\":" << (state.complete ? "true" : "false")
        << ",\"reason\":" << json_string(state.reason)
        << ",\"target_stage\":" << json_string(state.target_stage)
        << ",\"expected_sequence\":" << state.expected_sequence
        << ",\"start_time\":" << std::fixed << std::setprecision(3)
        << state.start_wall_time
        << ",\"min_ms\":" << state.min_ms
        << ",\"failure_reason\":" << json_string(state.failure_reason)
        << ",\"detail\":" << json_string(state.detail) << "}";
    return out.str();
  }

private:
  void publish_zero_commands()
  {
    ports_.clear_teleop_command();
    ports_.publish_teleop_zero_burst();
    ports_.publish_navigation_zero_burst();
  }

  void update_state(
    const std::function<void(PostRelocalizationSettleState &)> & update)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    update(state_);
  }

  void update_success_state(const PostRelocalizationSettleResult & result)
  {
    update_state(
      [&](PostRelocalizationSettleState & state) {
        state.in_progress = false;
        state.complete = true;
        state.failure_reason = "none";
        state.detail = result.detail;
      });
  }

  void update_failure_state(const PostRelocalizationSettleResult & result)
  {
    update_state(
      [&](PostRelocalizationSettleState & state) {
        state.in_progress = false;
        state.complete = false;
        state.failure_reason = result.failure_code;
        state.detail = result.detail;
      });
  }

  PostRelocalizationSettleConfig config_;
  PostRelocalizationSettlePorts ports_;
  mutable std::mutex state_mutex_;
  PostRelocalizationSettleState state_;
};

PostRelocalizationSettleModule::PostRelocalizationSettleModule(
  PostRelocalizationSettleConfig config,
  PostRelocalizationSettlePorts ports)
: impl_(std::make_unique<Impl>(std::move(config), std::move(ports)))
{
}

PostRelocalizationSettleModule::~PostRelocalizationSettleModule() = default;

PostRelocalizationSettleResult PostRelocalizationSettleModule::wait_for_settle(
  const std::uint64_t expected_sequence,
  const std::string & reason,
  const std::string & target_next_stage,
  const std::function<bool(std::string &)> & cancel_requested)
{
  return impl_->wait_for_settle(
    expected_sequence, reason, target_next_stage, cancel_requested);
}

PostRelocalizationSettleState PostRelocalizationSettleModule::state_snapshot() const
{
  return impl_->state_snapshot();
}

std::string PostRelocalizationSettleModule::state_json() const
{
  return impl_->state_json();
}

}  // namespace robot_api_server::features::localization
