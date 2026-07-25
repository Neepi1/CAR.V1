#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_docking_perception
{

struct PlanarPoint
{
  double x{0.0};
  double y{0.0};
};

struct DockGeometryConfig
{
  double charge_contact_x_m{0.398};
  double charge_contact_y_m{0.0};
  double min_forward_gap_m{0.02};
  double max_forward_gap_m{1.50};
  double lateral_gate_m{0.45};
  double front_cluster_window_m{0.04};
  double line_inlier_threshold_m{0.015};
  double min_lateral_span_m{0.12};
  double max_lateral_span_m{0.36};
  double expected_lateral_span_m{0.235};
  double expected_lateral_span_tolerance_m{0.06};
  double near_feature_expected_lateral_span_m{0.113};
  double near_feature_lateral_span_tolerance_m{0.025};
  double near_feature_max_forward_gap_m{0.30};
  double near_feature_center_y_offset_m{-0.0418};
  double max_rms_error_m{0.015};
  double min_confidence{0.25};
  std::size_t min_points{30U};
  std::size_t fit_iterations{3U};
};

struct DockTargetEstimate
{
  bool valid{false};
  double forward_gap_m{0.0};
  double lateral_error_m{0.0};
  double yaw_error_rad{0.0};
  double lateral_span_m{0.0};
  double confidence{0.0};
  std::size_t inlier_count{0U};
  double rms_error_m{0.0};
  std::string reason{"not_evaluated"};
};

DockTargetEstimate estimate_dock_target(
  const std::vector<PlanarPoint> & points,
  const DockGeometryConfig & config);

}  // namespace robot_docking_perception
