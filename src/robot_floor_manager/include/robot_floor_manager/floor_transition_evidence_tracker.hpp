#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "robot_floor_manager/floor_transition_core.hpp"

namespace robot_floor_manager
{

struct FloorTransitionTarget
{
  std::string transaction_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct LocalizationHealthEvidence
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::uint64_t localizer_generation{0U};
  std::uint64_t explicit_relocalization_sequence{0U};
  bool localizer_ready{false};
  bool bridge_ready{false};
  bool tf_unique{false};
  bool transition_active{false};
  bool runtime_context_valid{false};
  bool amcl_ready{false};
};

struct LocalizerAssetEvidence
{
  std::string transaction_id;
  bool applying{false};
  bool success{false};
  bool reloaded{false};
  bool active_identity_valid{false};
  std::string active_building_id;
  std::string active_floor_id;
  std::string active_map_id;
  std::uint64_t active_asset_epoch{0U};
  std::string active_asset_digest;
  std::uint64_t localizer_generation{0U};
  bool localizer_ready{false};
  std::string requested_building_id;
  std::string requested_floor_id;
  std::string requested_map_id;
  std::uint64_t requested_asset_epoch{0U};
  std::string requested_asset_digest;
  std::string code;
  std::string detail;
};

struct LocalizerApplyOutcome
{
  bool exact_request{false};
  bool terminal{false};
  bool success{false};
  std::uint64_t localizer_generation{0U};
  std::string code;
  std::string detail;
};

// Thread-compatible value tracker once externally serialized. It converts
// typed ROS observations into exact, fresh transaction evidence and never
// performs ROS calls or filesystem writes.
class FloorTransitionEvidenceTracker
{
public:
  void begin(const FloorTransitionTarget & target, double steady_now_sec);

  void observe_motion_interlock(
    bool hold_active,
    const std::vector<std::string> & hold_keys,
    double received_steady_sec);
  void observe_nav_graph_ready(bool ready, double received_steady_sec);
  void observe_nav_activity(bool active, double received_steady_sec);
  void observe_wheel_odom(
    double linear_x,
    double linear_y,
    double angular_z,
    double received_steady_sec);
  void observe_local_odom(
    double linear_x,
    double linear_y,
    double angular_z,
    double received_steady_sec);
  void observe_correction_pause(
    bool paused,
    const std::vector<std::string> & lease_keys,
    double received_steady_sec);
  void observe_localization_health(
    const LocalizationHealthEvidence & health,
    double received_steady_sec);
  void observe_localizer_asset(
    const LocalizerAssetEvidence & state,
    double received_steady_sec);
  void observe_global_costmap(
    std::uint64_t receive_sequence,
    double received_steady_sec);
  void observe_local_costmap(
    std::uint64_t receive_sequence,
    double received_steady_sec);

  void mark_costmaps_cleared(double steady_now_sec);

  FloorTransitionEvidence preconditions(
    double steady_now_sec,
    double max_age_sec,
    double stable_duration_sec,
    double stopped_linear_threshold_mps,
    double stopped_angular_threshold_radps,
    double nav_idle_bootstrap_grace_sec = 2.0) const;
  FloorTransitionEvidence floor_pause(
    double steady_now_sec,
    double max_age_sec) const;
  FloorTransitionEvidence pause_handoff(
    double steady_now_sec,
    double max_age_sec) const;
  bool corrections_released(
    double steady_now_sec,
    double max_age_sec) const;
  FloorTransitionEvidence target_localizer(
    std::uint64_t expected_generation,
    double steady_now_sec,
    double max_age_sec) const;
  LocalizerApplyOutcome localizer_apply_outcome(
    std::uint64_t baseline_generation,
    double steady_now_sec,
    double max_age_sec) const;
  FloorTransitionEvidence target_localization(
    double steady_now_sec,
    double max_age_sec,
    std::uint64_t expected_localizer_generation = 0U) const;
  FloorTransitionEvidence fresh_costmaps(
    double steady_now_sec,
    double max_age_sec,
    std::uint64_t expected_localizer_generation = 0U) const;

  std::uint64_t begin_localizer_generation() const noexcept;
  std::uint64_t begin_explicit_relocalization_sequence() const noexcept;

private:
  struct BinaryObservation
  {
    bool value{false};
    double received_steady_sec{-1.0};
  };

  struct KeyObservation
  {
    bool active{false};
    std::vector<std::string> keys;
    double received_steady_sec{-1.0};
  };

  struct VelocityObservation
  {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
    double received_steady_sec{-1.0};
  };

  struct CostmapObservation
  {
    std::uint64_t receive_sequence{0U};
    double received_steady_sec{-1.0};
  };

  bool fresh(double received_steady_sec, double now_sec, double max_age_sec) const;
  bool odom_stopped(
    const std::deque<VelocityObservation> & observations,
    double now_sec,
    double max_age_sec,
    double stable_duration_sec,
    double linear_threshold,
    double angular_threshold) const;
  bool exact_target(const LocalizationHealthEvidence & health) const;
  FloorTransitionEvidence target_identity_evidence() const;

  FloorTransitionTarget target_;
  double begin_steady_sec_{-1.0};
  KeyObservation motion_interlock_;
  bool nav_graph_ready_{false};
  double nav_graph_ready_since_steady_sec_{-1.0};
  BinaryObservation nav_active_;
  std::deque<VelocityObservation> wheel_odom_;
  std::deque<VelocityObservation> local_odom_;
  KeyObservation correction_pause_;
  LocalizationHealthEvidence localization_health_;
  double localization_health_received_steady_sec_{-1.0};
  LocalizerAssetEvidence localizer_asset_;
  double localizer_asset_received_steady_sec_{-1.0};
  std::uint64_t begin_localizer_generation_{0U};
  std::uint64_t begin_explicit_relocalization_sequence_{0U};
  CostmapObservation global_costmap_;
  CostmapObservation local_costmap_;
  std::uint64_t global_costmap_clear_baseline_{0U};
  std::uint64_t local_costmap_clear_baseline_{0U};
  double costmaps_cleared_steady_sec_{-1.0};
};

}  // namespace robot_floor_manager
