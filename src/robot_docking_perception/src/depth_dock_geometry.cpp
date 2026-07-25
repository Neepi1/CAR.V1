#include "robot_docking_perception/depth_dock_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace robot_docking_perception
{

namespace
{

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double percentile_from_sorted(const std::vector<double> & values, const double percentile)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double index = percentile * static_cast<double>(values.size() - 1U);
  const auto lower_index = static_cast<std::size_t>(std::floor(index));
  const auto upper_index = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower_index);
  return values[lower_index] * (1.0 - fraction) + values[upper_index] * fraction;
}

struct LateralLimits
{
  double lower{0.0};
  double upper{0.0};
};

LateralLimits robust_lateral_limits(const std::vector<PlanarPoint> & points)
{
  std::vector<double> ys;
  ys.reserve(points.size());
  for (const auto & point : points) {
    ys.push_back(point.y);
  }
  std::sort(ys.begin(), ys.end());
  return {percentile_from_sorted(ys, 0.05), percentile_from_sorted(ys, 0.95)};
}

struct LineFit
{
  bool valid{false};
  double slope{0.0};
  double intercept{0.0};
};

enum class FeatureKind
{
  None,
  FixedFace,
  NearProtrusion
};

struct FeatureMatch
{
  bool valid{false};
  double span_score{0.0};
  FeatureKind kind{FeatureKind::None};
};

FeatureMatch match_calibrated_feature(
  const double lateral_span_m,
  const double forward_gap_m,
  const DockGeometryConfig & config)
{
  FeatureMatch match;

  if (config.expected_lateral_span_m > 0.0 &&
    config.expected_lateral_span_tolerance_m > 0.0)
  {
    const double error = std::abs(lateral_span_m - config.expected_lateral_span_m);
    if (error <= config.expected_lateral_span_tolerance_m) {
      const double score = 1.0 - error / config.expected_lateral_span_tolerance_m;
      match.valid = true;
      if (score > match.span_score || match.kind == FeatureKind::None) {
        match.span_score = score;
        match.kind = FeatureKind::FixedFace;
      }
    }
  }

  if (config.near_feature_expected_lateral_span_m > 0.0 &&
    config.near_feature_lateral_span_tolerance_m > 0.0 &&
    forward_gap_m <= config.near_feature_max_forward_gap_m)
  {
    const double error = std::abs(
      lateral_span_m - config.near_feature_expected_lateral_span_m);
    if (error <= config.near_feature_lateral_span_tolerance_m) {
      const double score = 1.0 - error / config.near_feature_lateral_span_tolerance_m;
      match.valid = true;
      if (score > match.span_score || match.kind == FeatureKind::None) {
        match.span_score = score;
        match.kind = FeatureKind::NearProtrusion;
      }
    }
  }

  return match;
}

LineFit fit_x_from_y(const std::vector<PlanarPoint> & points)
{
  LineFit fit;
  if (points.size() < 2U) {
    return fit;
  }

  const double count = static_cast<double>(points.size());
  const double mean_x = std::accumulate(
    points.begin(), points.end(), 0.0,
    [](const double sum, const PlanarPoint & point) {return sum + point.x;}) / count;
  const double mean_y = std::accumulate(
    points.begin(), points.end(), 0.0,
    [](const double sum, const PlanarPoint & point) {return sum + point.y;}) / count;

  double covariance = 0.0;
  double variance_y = 0.0;
  for (const auto & point : points) {
    const double delta_x = point.x - mean_x;
    const double delta_y = point.y - mean_y;
    covariance += delta_x * delta_y;
    variance_y += delta_y * delta_y;
  }
  if (variance_y <= 1e-12) {
    return fit;
  }

  fit.valid = true;
  fit.slope = covariance / variance_y;
  fit.intercept = mean_x - fit.slope * mean_y;
  return fit;
}

double orthogonal_residual(const PlanarPoint & point, const LineFit & fit)
{
  return (point.x - (fit.slope * point.y + fit.intercept)) /
         std::sqrt(1.0 + fit.slope * fit.slope);
}

}  // namespace

DockTargetEstimate estimate_dock_target(
  const std::vector<PlanarPoint> & points,
  const DockGeometryConfig & config)
{
  DockTargetEstimate estimate;

  if (config.min_points < 2U || config.front_cluster_window_m <= 0.0 ||
    config.line_inlier_threshold_m <= 0.0 || config.max_rms_error_m <= 0.0 ||
    config.min_forward_gap_m >= config.max_forward_gap_m ||
    config.expected_lateral_span_tolerance_m <= 0.0 ||
    config.near_feature_lateral_span_tolerance_m <= 0.0 ||
    config.near_feature_max_forward_gap_m <= config.min_forward_gap_m)
  {
    estimate.reason = "invalid_config";
    return estimate;
  }

  std::vector<PlanarPoint> filtered;
  filtered.reserve(points.size());
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const double forward_gap = point.x - config.charge_contact_x_m;
    if (forward_gap < config.min_forward_gap_m || forward_gap > config.max_forward_gap_m) {
      continue;
    }
    if (std::abs(point.y - config.charge_contact_y_m) > config.lateral_gate_m) {
      continue;
    }
    filtered.push_back(point);
  }

  if (filtered.size() < config.min_points) {
    estimate.reason = "insufficient_points";
    return estimate;
  }

  std::sort(
    filtered.begin(), filtered.end(),
    [](const PlanarPoint & left, const PlanarPoint & right) {return left.x < right.x;});

  std::vector<PlanarPoint> candidate;
  double candidate_median_x = std::numeric_limits<double>::infinity();
  bool saw_dense_span_candidate = false;
  for (std::size_t start_index = 0U; start_index < filtered.size();) {
    std::size_t end_index = start_index;
    while (end_index < filtered.size() &&
      filtered[end_index].x - filtered[start_index].x <= config.front_cluster_window_m)
    {
      ++end_index;
    }

    const std::size_t window_size = end_index - start_index;
    if (window_size < config.min_points) {
      start_index = end_index;
      continue;
    }

    std::vector<PlanarPoint> window(
      filtered.begin() + static_cast<std::ptrdiff_t>(start_index),
      filtered.begin() + static_cast<std::ptrdiff_t>(end_index));
    const auto [lower_y, upper_y] = robust_lateral_limits(window);
    const double lateral_span = upper_y - lower_y;
    if (lateral_span < config.min_lateral_span_m ||
      (config.max_lateral_span_m > 0.0 && lateral_span > config.max_lateral_span_m))
    {
      start_index = end_index;
      continue;
    }

    saw_dense_span_candidate = true;

    const double median_x = window[window.size() / 2U].x;
    const double median_forward_gap = median_x - config.charge_contact_x_m;
    if (!match_calibrated_feature(
        lateral_span, median_forward_gap, config).valid)
    {
      start_index = end_index;
      continue;
    }
    if (median_x < candidate_median_x) {
      candidate = std::move(window);
      candidate_median_x = median_x;
    }
    start_index = end_index;
  }

  if (candidate.size() < config.min_points) {
    estimate.reason = saw_dense_span_candidate ?
      "no_known_dock_feature_cluster" : "no_dense_wide_front_cluster";
    return estimate;
  }

  std::vector<PlanarPoint> inliers = candidate;
  LineFit fit;
  for (std::size_t iteration = 0U; iteration < config.fit_iterations; ++iteration) {
    fit = fit_x_from_y(inliers);
    if (!fit.valid) {
      estimate.reason = "degenerate_lateral_geometry";
      return estimate;
    }

    std::vector<PlanarPoint> next_inliers;
    next_inliers.reserve(inliers.size());
    for (const auto & point : candidate) {
      if (std::abs(orthogonal_residual(point, fit)) <= config.line_inlier_threshold_m) {
        next_inliers.push_back(point);
      }
    }
    if (next_inliers.size() < config.min_points) {
      estimate.reason = "insufficient_line_inliers";
      return estimate;
    }
    if (next_inliers.size() == inliers.size()) {
      inliers = std::move(next_inliers);
      break;
    }
    inliers = std::move(next_inliers);
  }

  fit = fit_x_from_y(inliers);
  if (!fit.valid) {
    estimate.reason = "degenerate_lateral_geometry";
    return estimate;
  }

  double squared_residual_sum = 0.0;
  for (const auto & point : inliers) {
    const double residual = orthogonal_residual(point, fit);
    squared_residual_sum += residual * residual;
  }
  estimate.rms_error_m = std::sqrt(
    squared_residual_sum / static_cast<double>(inliers.size()));

  const auto [lower_y, upper_y] = robust_lateral_limits(inliers);
  estimate.lateral_span_m = upper_y - lower_y;
  const double dock_center_y = 0.5 * (lower_y + upper_y);
  estimate.forward_gap_m =
    fit.slope * config.charge_contact_y_m + fit.intercept - config.charge_contact_x_m;

  // A positive fitted slope means the vehicle is counter-clockwise from the dock normal,
  // so the required vehicle correction is clockwise.
  estimate.yaw_error_rad = -std::atan(fit.slope);
  estimate.inlier_count = inliers.size();

  const FeatureMatch feature_match = match_calibrated_feature(
    estimate.lateral_span_m, estimate.forward_gap_m, config);
  if (!feature_match.valid) {
    estimate.reason = "fitted_span_not_calibrated_feature";
    return estimate;
  }
  const double calibrated_center_y = config.charge_contact_y_m +
    (feature_match.kind == FeatureKind::NearProtrusion ?
    config.near_feature_center_y_offset_m : 0.0);
  estimate.lateral_error_m = dock_center_y - calibrated_center_y;

  const double count_score = clamp01(
    static_cast<double>(inliers.size()) / (1.5 * static_cast<double>(config.min_points)));
  const double inlier_score = clamp01(
    static_cast<double>(inliers.size()) / static_cast<double>(candidate.size()));
  const double rms_score = clamp01(1.0 - estimate.rms_error_m / config.max_rms_error_m);
  estimate.confidence =
    0.20 * count_score + 0.30 * feature_match.span_score +
    0.25 * inlier_score + 0.25 * rms_score;

  if (estimate.forward_gap_m < config.min_forward_gap_m ||
    estimate.forward_gap_m > config.max_forward_gap_m)
  {
    estimate.reason = "fitted_gap_out_of_range";
    return estimate;
  }
  if (estimate.lateral_span_m < config.min_lateral_span_m) {
    estimate.reason = "lateral_span_too_small";
    return estimate;
  }
  if (config.max_lateral_span_m > 0.0 &&
    estimate.lateral_span_m > config.max_lateral_span_m)
  {
    estimate.reason = "lateral_span_too_large";
    return estimate;
  }
  if (estimate.rms_error_m > config.max_rms_error_m) {
    estimate.reason = "line_fit_rms_too_large";
    return estimate;
  }
  if (estimate.confidence < config.min_confidence) {
    estimate.reason = "confidence_too_low";
    return estimate;
  }

  estimate.valid = true;
  estimate.reason = "ok";
  return estimate;
}

}  // namespace robot_docking_perception
