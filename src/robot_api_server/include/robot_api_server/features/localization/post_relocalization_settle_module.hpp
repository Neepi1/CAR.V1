#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "robot_api_server/features/localization/localization_module.hpp"

namespace robot_api_server::features::localization
{

struct PostRelocalizationSettleResult
{
  bool ok{false};
  std::string failure_code{"POST_RELOCALIZATION_SETTLE_TIMEOUT"};
  std::string detail;
  std::uint64_t expected_sequence{0U};
  std::uint64_t observed_sequence{0U};
  int stable_samples{0};
  std::uint64_t local_costmap_updates{0U};
  double elapsed_ms{0.0};
};

struct PostRelocalizationSettleState
{
  bool required{false};
  bool in_progress{false};
  bool complete{true};
  std::string reason{"none"};
  std::string target_stage{"none"};
  std::string failure_reason{"none"};
  std::string detail;
  std::uint64_t expected_sequence{0U};
  int min_ms{0};
  double start_wall_time{0.0};
};

struct PostRelocalizationSettleConfig
{
  bool enabled{true};
  int min_ms{800};
  int max_ms{3000};
  int stable_tf_samples{5};
  int tf_sample_period_ms{100};
  bool zero_cmd{true};
  bool require_local_costmap_update{true};
  int required_local_costmap_updates{2};
  bool reject_if_new_message_filter_drop{true};
  double map_odom_publish_gap_warn_ms{100.0};
  double map_odom_publish_gap_fail_ms{250.0};

  bool post_undock_enabled{true};
  int post_undock_min_ms{800};
  int post_undock_max_ms{3000};
  int post_undock_stable_tf_samples{5};
  int post_undock_tf_sample_period_ms{100};
  int post_undock_required_local_costmap_updates{2};
  bool post_undock_reject_if_new_message_filter_drop{true};
  bool post_undock_zero_cmd_during_settle{true};

  double large_correction_translation_m{0.5};
  double large_correction_yaw_rad{0.3};
  int large_correction_min_ms{1500};
  double tf_chain_freshness_sec{0.30};
  double robot_pose_freshness_sec{0.5};
  std::string base_frame{"base_link"};
  std::string static_lidar_frame{"lidar_level_link"};
};

struct PostRelocalizationSettlePorts
{
  using TimePoint = std::chrono::steady_clock::time_point;

  std::function<BridgeStatusSnapshot()> bridge_status_snapshot;
  std::function<TfChainFreshnessSnapshot()> tf_chain_freshness_snapshot;
  std::function<std::string(const TfChainFreshnessSnapshot &)> tf_chain_freshness_detail;
  std::function<bool()> base_to_lidar_static_tf_ready;
  std::function<std::uint64_t()> local_costmap_update_count;
  std::function<std::uint64_t()> local_costmap_message_filter_drop_count;
  std::function<std::string()> last_local_costmap_message_filter_drop_text;
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_teleop_zero_burst;
  std::function<void()> publish_navigation_zero_burst;
  std::function<TimePoint()> steady_now;
  std::function<double()> wall_time_seconds;
  std::function<void(std::chrono::milliseconds)> sleep_for;
};

// Owns the complete stability barrier between an accepted explicit
// relocalization and the next motion stage. It observes the canonical TF tree
// and local-costmap evidence but never publishes TF or bypasses robot_safety.
class PostRelocalizationSettleModule
{
public:
  PostRelocalizationSettleModule(
    PostRelocalizationSettleConfig config,
    PostRelocalizationSettlePorts ports);
  ~PostRelocalizationSettleModule();

  PostRelocalizationSettleModule(const PostRelocalizationSettleModule &) = delete;
  PostRelocalizationSettleModule & operator=(
    const PostRelocalizationSettleModule &) = delete;

  PostRelocalizationSettleResult wait_for_settle(
    std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested = {});

  PostRelocalizationSettleState state_snapshot() const;
  std::string state_json() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::localization
