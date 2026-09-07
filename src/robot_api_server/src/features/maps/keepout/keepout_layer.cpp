#include "robot_api_server/features/maps/keepout/keepout_layer.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <system_error>
#include <utility>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

constexpr const char * kRendererVersion = "njrh.keepout.raster.v1";
constexpr std::size_t kMaxFeatures = 128U;
constexpr std::size_t kMaxPointsPerFeature = 1024U;
constexpr std::size_t kMaxTotalPoints = 4096U;
constexpr std::size_t kMaxCostmapVerificationSamples = kMaxFeatures;
constexpr std::uint64_t kMaxRasterWorkUnits = 10000000U;

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Line
{
  std::string id;
  std::string name;
  double width_m{0.0};
  std::vector<Point> points;
};

struct Polygon
{
  std::string id;
  std::string name;
  std::vector<Point> points;
};

struct Document
{
  std::vector<Line> lines;
  std::vector<Polygon> polygons;
};

struct PgmImage
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::vector<std::uint8_t> pixels;
};

struct FileSnapshot
{
  fs::path path;
  bool existed{false};
  std::string content;
};

double finite_number(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsScalar()) {
    throw KeepoutLayerError("INVALID_GEOMETRY", 422, field + " must be a finite number");
  }
  try {
    const double value = node.as<double>();
    if (!std::isfinite(value)) {
      throw KeepoutLayerError("INVALID_GEOMETRY", 422, field + " must be finite");
    }
    return value;
  } catch (const KeepoutLayerError &) {
    throw;
  } catch (const YAML::Exception &) {
    throw KeepoutLayerError("INVALID_GEOMETRY", 422, field + " must be a finite number");
  }
}

std::string required_id(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsScalar()) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, field + " is required");
  }
  const auto value = node.as<std::string>();
  if (!safe_asset_id(value)) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, field + " is invalid");
  }
  return value;
}

std::string optional_name(const YAML::Node & node, const std::string & fallback)
{
  if (!node) {
    return fallback;
  }
  if (!node.IsScalar()) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, "keepout name must be a string");
  }
  const auto value = node.as<std::string>();
  return value.empty() ? fallback : value;
}

Point parse_point(const YAML::Node & node, const std::string & field)
{
  if (!node || !node.IsMap()) {
    throw KeepoutLayerError("INVALID_GEOMETRY", 422, field + " must be an object");
  }
  return {finite_number(node["x"], field + ".x"), finite_number(node["y"], field + ".y")};
}

bool points_equal(const Point & lhs, const Point & rhs)
{
  constexpr double epsilon = 1e-9;
  return std::fabs(lhs.x - rhs.x) <= epsilon && std::fabs(lhs.y - rhs.y) <= epsilon;
}

double cross_product(const Point & a, const Point & b, const Point & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool point_on_segment(const Point & point, const Point & start, const Point & end)
{
  constexpr double epsilon = 1e-9;
  return std::fabs(cross_product(start, end, point)) <= epsilon &&
         point.x >= std::min(start.x, end.x) - epsilon &&
         point.x <= std::max(start.x, end.x) + epsilon &&
         point.y >= std::min(start.y, end.y) - epsilon &&
         point.y <= std::max(start.y, end.y) + epsilon;
}

bool segments_intersect(
  const Point & a,
  const Point & b,
  const Point & c,
  const Point & d)
{
  constexpr double epsilon = 1e-9;
  const double ab_c = cross_product(a, b, c);
  const double ab_d = cross_product(a, b, d);
  const double cd_a = cross_product(c, d, a);
  const double cd_b = cross_product(c, d, b);
  if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
    ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon)))
  {
    return true;
  }
  return (std::fabs(ab_c) <= epsilon && point_on_segment(c, a, b)) ||
         (std::fabs(ab_d) <= epsilon && point_on_segment(d, a, b)) ||
         (std::fabs(cd_a) <= epsilon && point_on_segment(a, c, d)) ||
         (std::fabs(cd_b) <= epsilon && point_on_segment(b, c, d));
}

void validate_polygon(std::vector<Point> & points, const std::string & id)
{
  if (points.size() >= 2U && points_equal(points.front(), points.back())) {
    points.pop_back();
  }
  if (points.size() < 3U) {
    throw KeepoutLayerError(
            "INVALID_GEOMETRY", 422, "keepout polygon must contain three distinct points: " + id);
  }
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (points_equal(points[index], points[(index + 1U) % points.size()])) {
      throw KeepoutLayerError(
              "INVALID_GEOMETRY", 422, "keepout polygon has a zero-length edge: " + id);
    }
  }

  double twice_area = 0.0;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const auto & current = points[index];
    const auto & next = points[(index + 1U) % points.size()];
    twice_area += current.x * next.y - next.x * current.y;
  }
  if (std::fabs(twice_area) <= 1e-9) {
    throw KeepoutLayerError("INVALID_GEOMETRY", 422, "keepout polygon has zero area: " + id);
  }

  for (std::size_t first = 0U; first < points.size(); ++first) {
    const auto first_next = (first + 1U) % points.size();
    for (std::size_t second = first + 1U; second < points.size(); ++second) {
      const auto second_next = (second + 1U) % points.size();
      if (first == second || first_next == second || second_next == first) {
        continue;
      }
      if (segments_intersect(
          points[first], points[first_next], points[second], points[second_next]))
      {
        throw KeepoutLayerError(
                "INVALID_GEOMETRY", 422, "keepout polygon self-intersects: " + id);
      }
    }
  }
}

void assert_optional_selector(
  const YAML::Node & root,
  const char * field,
  const std::string & expected)
{
  const auto value = root[field];
  if (!value) {
    return;
  }
  if (!value.IsScalar() || value.as<std::string>() != expected) {
    throw KeepoutLayerError(
            "MAP_SELECTOR_MISMATCH",
            409,
            std::string(field) + " does not match the selected map");
  }
}

Document parse_document(const std::string & json, const MapManifest & map)
{
  YAML::Node root;
  try {
    root = YAML::Load(json);
  } catch (const YAML::Exception & error) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, "invalid JSON document: " + std::string(error.what()));
  }
  if (!root || !root.IsMap()) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, "JSON object is required");
  }

  assert_optional_selector(root, "building_id", map.building_id);
  assert_optional_selector(root, "floor_id", map.floor_id);
  assert_optional_selector(root, "map_id", map.map_id);

  const auto line_nodes = root["keepout_lines"];
  const auto polygon_nodes = root["keepout_polygons"];
  if (!line_nodes || !line_nodes.IsSequence()) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, "keepout_lines array is required");
  }
  if (!polygon_nodes || !polygon_nodes.IsSequence()) {
    throw KeepoutLayerError("INVALID_DOCUMENT", 400, "keepout_polygons array is required");
  }
  if (line_nodes.size() + polygon_nodes.size() > kMaxFeatures) {
    throw KeepoutLayerError(
            "KEEP_OUT_COMPLEXITY_LIMIT",
            422,
            "keepout layer exceeds the maximum of " + std::to_string(kMaxFeatures) + " features");
  }
  Document document;
  std::set<std::string> ids;
  std::size_t total_points = 0U;
  for (std::size_t index = 0U; index < line_nodes.size(); ++index) {
    const auto line_node = line_nodes[index];
    if (!line_node.IsMap()) {
      throw KeepoutLayerError("INVALID_DOCUMENT", 400, "keepout_lines entries must be objects");
    }
    Line line;
    line.id = required_id(line_node["id"], "keepout_lines.id");
    if (!ids.insert(line.id).second) {
      throw KeepoutLayerError("INVALID_DOCUMENT", 400, "duplicate keepout id: " + line.id);
    }
    line.name = optional_name(line_node["name"], line.id);
    line.width_m = finite_number(line_node["width_m"], "keepout_lines.width_m");
    if (line.width_m <= 0.0) {
      throw KeepoutLayerError("INVALID_GEOMETRY", 422, "keepout line width_m must be positive");
    }
    const auto points = line_node["points"];
    if (!points || !points.IsSequence() || points.size() < 2U) {
      throw KeepoutLayerError(
              "INVALID_GEOMETRY", 422, "keepout line must contain at least two points");
    }
    if (points.size() > kMaxPointsPerFeature ||
      total_points + points.size() > kMaxTotalPoints)
    {
      throw KeepoutLayerError(
              "KEEP_OUT_COMPLEXITY_LIMIT", 422, "keepout line point limit exceeded");
    }
    for (std::size_t point_index = 0U; point_index < points.size(); ++point_index) {
      line.points.push_back(
        parse_point(points[point_index], "keepout_lines.points[" + std::to_string(point_index) + "]"));
      if (point_index > 0U && points_equal(
          line.points[point_index - 1U], line.points[point_index]))
      {
        throw KeepoutLayerError(
                "INVALID_GEOMETRY", 422, "keepout line contains a zero-length segment: " + line.id);
      }
    }
    total_points += points.size();
    document.lines.push_back(std::move(line));
  }

  for (std::size_t index = 0U; index < polygon_nodes.size(); ++index) {
    const auto polygon_node = polygon_nodes[index];
    if (!polygon_node.IsMap()) {
      throw KeepoutLayerError("INVALID_DOCUMENT", 400, "keepout_polygons entries must be objects");
    }
    Polygon polygon;
    polygon.id = required_id(polygon_node["id"], "keepout_polygons.id");
    if (!ids.insert(polygon.id).second) {
      throw KeepoutLayerError("INVALID_DOCUMENT", 400, "duplicate keepout id: " + polygon.id);
    }
    polygon.name = optional_name(polygon_node["name"], polygon.id);
    const auto points = polygon_node["polygon"];
    if (!points || !points.IsSequence() || points.size() < 3U) {
      throw KeepoutLayerError(
              "INVALID_GEOMETRY", 422, "keepout polygon must contain at least three points");
    }
    if (points.size() > kMaxPointsPerFeature ||
      total_points + points.size() > kMaxTotalPoints)
    {
      throw KeepoutLayerError(
              "KEEP_OUT_COMPLEXITY_LIMIT", 422, "keepout polygon point limit exceeded");
    }
    for (std::size_t point_index = 0U; point_index < points.size(); ++point_index) {
      polygon.points.push_back(
        parse_point(
          points[point_index],
          "keepout_polygons.polygon[" + std::to_string(point_index) + "]"));
    }
    validate_polygon(polygon.points, polygon.id);
    total_points += points.size();
    document.polygons.push_back(std::move(polygon));
  }
  return document;
}

std::string canonical_document_json(const Document & document, const MapManifest & map)
{
  std::ostringstream json;
  json << std::setprecision(17)
       << "{\"schema\":\"njrh.keepout.semantic.v1\","
       << "\"building_id\":" << json_string(map.building_id) << ","
       << "\"floor_id\":" << json_string(map.floor_id) << ","
       << "\"map_id\":" << json_string(map.map_id) << ","
       << "\"keepout_lines\":[";
  for (std::size_t line_index = 0U; line_index < document.lines.size(); ++line_index) {
    if (line_index > 0U) {
      json << ",";
    }
    const auto & line = document.lines[line_index];
    json << "{\"id\":" << json_string(line.id)
         << ",\"name\":" << json_string(line.name)
         << ",\"width_m\":" << line.width_m
         << ",\"points\":[";
    for (std::size_t point_index = 0U; point_index < line.points.size(); ++point_index) {
      if (point_index > 0U) {
        json << ",";
      }
      json << "{\"x\":" << line.points[point_index].x
           << ",\"y\":" << line.points[point_index].y << "}";
    }
    json << "]}";
  }
  json << "],\"keepout_polygons\":[";
  for (std::size_t polygon_index = 0U;
    polygon_index < document.polygons.size(); ++polygon_index)
  {
    if (polygon_index > 0U) {
      json << ",";
    }
    const auto & polygon = document.polygons[polygon_index];
    json << "{\"id\":" << json_string(polygon.id)
         << ",\"name\":" << json_string(polygon.name)
         << ",\"polygon\":[";
    for (std::size_t point_index = 0U; point_index < polygon.points.size(); ++point_index) {
      if (point_index > 0U) {
        json << ",";
      }
      json << "{\"x\":" << polygon.points[point_index].x
           << ",\"y\":" << polygon.points[point_index].y << "}";
    }
    json << "]}";
  }
  json << "]}\n";
  return json.str();
}

Point world_to_local(const Point & point, const MapYamlInfo & map)
{
  const double dx = point.x - map.origin[0];
  const double dy = point.y - map.origin[1];
  const double cosine = std::cos(map.origin[2]);
  const double sine = std::sin(map.origin[2]);
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

double point_to_segment_distance(const Point & point, const Point & start, const Point & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }
  const double projection = std::clamp(
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared,
    0.0,
    1.0);
  return std::hypot(
    point.x - (start.x + projection * dx),
    point.y - (start.y + projection * dy));
}

bool point_in_polygon(const Point & point, const std::vector<Point> & polygon)
{
  bool inside = false;
  for (std::size_t current = 0U, previous = polygon.size() - 1U;
    current < polygon.size(); previous = current++)
  {
    const auto & a = polygon[current];
    const auto & b = polygon[previous];
    const bool crosses =
      ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

struct CellBounds
{
  std::uint32_t min_column{0U};
  std::uint32_t max_column{0U};
  std::uint32_t min_row{0U};
  std::uint32_t max_row{0U};
  bool intersects{false};
};

CellBounds cell_bounds_for_points(
  const std::vector<Point> & points,
  const double padding,
  const MapYamlInfo & map)
{
  const auto x_limits = std::minmax_element(
    points.begin(), points.end(),
    [](const Point & lhs, const Point & rhs) {return lhs.x < rhs.x;});
  const auto y_limits = std::minmax_element(
    points.begin(), points.end(),
    [](const Point & lhs, const Point & rhs) {return lhs.y < rhs.y;});
  const auto clamp_index = [](const double value, const std::uint32_t size) {
      return static_cast<std::uint32_t>(
        std::clamp(value, 0.0, static_cast<double>(size) - 1.0));
    };
  const double raw_min_column =
    std::floor((x_limits.first->x - padding) / map.resolution) - 1.0;
  const double raw_max_column =
    std::ceil((x_limits.second->x + padding) / map.resolution) + 1.0;
  const double raw_min_row =
    std::floor((y_limits.first->y - padding) / map.resolution) - 1.0;
  const double raw_max_row =
    std::ceil((y_limits.second->y + padding) / map.resolution) + 1.0;
  CellBounds bounds;
  bounds.intersects =
    raw_max_column >= 0.0 && raw_max_row >= 0.0 &&
    raw_min_column < static_cast<double>(map.width) &&
    raw_min_row < static_cast<double>(map.height);
  if (!bounds.intersects) {
    return bounds;
  }
  bounds.min_column = clamp_index(raw_min_column, map.width);
  bounds.max_column = clamp_index(raw_max_column, map.width);
  bounds.min_row = clamp_index(raw_min_row, map.height);
  bounds.max_row = clamp_index(raw_max_row, map.height);
  return bounds;
}

std::uint64_t cell_count(const CellBounds & bounds)
{
  if (!bounds.intersects) {
    return 0U;
  }
  const auto columns =
    static_cast<std::uint64_t>(bounds.max_column) - bounds.min_column + 1U;
  const auto rows =
    static_cast<std::uint64_t>(bounds.max_row) - bounds.min_row + 1U;
  return columns * rows;
}

class RasterWorkBudget
{
public:
  void consume(
    const std::uint64_t cells,
    const std::uint64_t work_per_cell,
    const std::string & feature_id)
  {
    if (cells == 0U || work_per_cell == 0U) {
      return;
    }
    const auto remaining = kMaxRasterWorkUnits - used_;
    if (work_per_cell > remaining || cells > remaining / work_per_cell) {
      throw KeepoutLayerError(
              "KEEP_OUT_COMPLEXITY_LIMIT",
              422,
              "keepout raster work limit exceeded while processing feature: " + feature_id);
    }
    used_ += cells * work_per_cell;
  }

private:
  std::uint64_t used_{0U};
};

PgmImage read_pgm(const fs::path & path);

std::vector<std::uint8_t> render_document(
  const Document & document,
  const MapYamlInfo & map,
  const PgmImage & nav_map,
  std::size_t & active_cells,
  std::vector<std::size_t> & feature_free_samples)
{
  const auto pixel_count = static_cast<std::size_t>(map.width) * map.height;
  if (nav_map.width != map.width || nav_map.height != map.height ||
    nav_map.pixels.size() != pixel_count)
  {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH",
            422,
            "nav map PGM dimensions do not match its YAML geometry");
  }
  std::vector<std::uint8_t> pixels(
    pixel_count,
    254U);
  const double cell_cover_margin = map.resolution * std::sqrt(0.5);
  RasterWorkBudget work_budget;
  feature_free_samples.clear();
  feature_free_samples.reserve(document.lines.size() + document.polygons.size());

  for (const auto & line : document.lines) {
    std::vector<Point> local_points;
    local_points.reserve(line.points.size());
    for (const auto & point : line.points) {
      local_points.push_back(world_to_local(point, map));
    }
    const double radius = line.width_m * 0.5 + cell_cover_margin;
    std::vector<CellBounds> segment_bounds;
    segment_bounds.reserve(local_points.size() - 1U);
    for (std::size_t segment = 1U; segment < local_points.size(); ++segment) {
      const std::vector<Point> endpoints{
        local_points[segment - 1U], local_points[segment]};
      const auto bounds = cell_bounds_for_points(endpoints, radius, map);
      work_budget.consume(cell_count(bounds), 1U, line.id);
      segment_bounds.push_back(bounds);
    }
    bool feature_hit = false;
    bool feature_free_hit = false;
    std::size_t feature_free_sample = 0U;
    for (std::size_t segment = 1U; segment < local_points.size(); ++segment) {
      const auto & bounds = segment_bounds[segment - 1U];
      if (!bounds.intersects) {
        continue;
      }
      for (std::uint64_t row_value = bounds.min_row;
        row_value <= bounds.max_row; ++row_value)
      {
        const auto row_from_bottom = static_cast<std::uint32_t>(row_value);
        for (std::uint64_t column_value = bounds.min_column;
          column_value <= bounds.max_column; ++column_value)
        {
          const auto column = static_cast<std::uint32_t>(column_value);
          const Point cell{
            (static_cast<double>(column) + 0.5) * map.resolution,
            (static_cast<double>(row_from_bottom) + 0.5) * map.resolution};
          if (point_to_segment_distance(
              cell, local_points[segment - 1U], local_points[segment]) > radius)
          {
            continue;
          }
          feature_hit = true;
          const std::uint32_t pgm_row = map.height - 1U - row_from_bottom;
          const auto pixel_index =
            static_cast<std::size_t>(pgm_row) * map.width + column;
          pixels[pixel_index] = 0U;
          if (!feature_free_hit && nav_map.pixels[pixel_index] >= 250U) {
            feature_free_hit = true;
            feature_free_sample = pixel_index;
          }
        }
      }
    }
    if (!feature_hit) {
      throw KeepoutLayerError(
              "INVALID_GEOMETRY",
              422,
              "keepout feature does not intersect the selected map: " + line.id);
    }
    if (!feature_free_hit) {
      throw KeepoutLayerError(
              "KEEP_OUT_NO_TRAVERSABLE_INTERSECTION",
              422,
              "keepout feature does not cover a free nav-map cell: " + line.id);
    }
    feature_free_samples.push_back(feature_free_sample);
  }

  for (const auto & polygon : document.polygons) {
    std::vector<Point> local_points;
    local_points.reserve(polygon.points.size());
    for (const auto & point : polygon.points) {
      local_points.push_back(world_to_local(point, map));
    }
    const auto bounds = cell_bounds_for_points(local_points, cell_cover_margin, map);
    work_budget.consume(
      cell_count(bounds),
      static_cast<std::uint64_t>(local_points.size()) * 2U,
      polygon.id);
    bool feature_hit = false;
    bool feature_free_hit = false;
    std::size_t feature_free_sample = 0U;
    if (bounds.intersects) {
      for (std::uint64_t row_value = bounds.min_row;
        row_value <= bounds.max_row; ++row_value)
      {
        const auto row_from_bottom = static_cast<std::uint32_t>(row_value);
        for (std::uint64_t column_value = bounds.min_column;
          column_value <= bounds.max_column; ++column_value)
        {
          const auto column = static_cast<std::uint32_t>(column_value);
          const Point cell{
            (static_cast<double>(column) + 0.5) * map.resolution,
            (static_cast<double>(row_from_bottom) + 0.5) * map.resolution};
          bool blocked = point_in_polygon(cell, local_points);
          if (!blocked) {
            for (std::size_t edge = 0U; edge < local_points.size(); ++edge) {
              const auto next = (edge + 1U) % local_points.size();
              if (point_to_segment_distance(
                  cell, local_points[edge], local_points[next]) <= cell_cover_margin)
              {
                blocked = true;
                break;
              }
            }
          }
          if (blocked) {
            feature_hit = true;
            const std::uint32_t pgm_row = map.height - 1U - row_from_bottom;
            const auto pixel_index =
              static_cast<std::size_t>(pgm_row) * map.width + column;
            pixels[pixel_index] = 0U;
            if (!feature_free_hit && nav_map.pixels[pixel_index] >= 250U) {
              feature_free_hit = true;
              feature_free_sample = pixel_index;
            }
          }
        }
      }
    }
    if (!feature_hit) {
      throw KeepoutLayerError(
              "INVALID_GEOMETRY",
              422,
              "keepout feature does not intersect the selected map: " + polygon.id);
    }
    if (!feature_free_hit) {
      throw KeepoutLayerError(
              "KEEP_OUT_NO_TRAVERSABLE_INTERSECTION",
              422,
              "keepout feature does not cover a free nav-map cell: " + polygon.id);
    }
    feature_free_samples.push_back(feature_free_sample);
  }

  active_cells = static_cast<std::size_t>(
    std::count(pixels.begin(), pixels.end(), static_cast<std::uint8_t>(0U)));
  if (active_cells == pixels.size() && active_cells > 0U) {
    throw KeepoutLayerError(
            "KEEP_OUT_MASK_ALL_BLOCKED",
            422,
            "keepout geometry blocks every cell in the selected map");
  }
  return pixels;
}

std::string pgm_payload(
  const std::uint32_t width,
  const std::uint32_t height,
  const std::vector<std::uint8_t> & pixels)
{
  std::string payload = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  payload.append(reinterpret_cast<const char *>(pixels.data()), pixels.size());
  return payload;
}

std::string next_pgm_token(std::istream & input)
{
  std::string token;
  char character = '\0';
  while (input.get(character)) {
    if (std::isspace(static_cast<unsigned char>(character)) != 0) {
      continue;
    }
    if (character == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    token.push_back(character);
    break;
  }
  while (input.get(character)) {
    if (std::isspace(static_cast<unsigned char>(character)) != 0) {
      break;
    }
    if (character == '#') {
      std::string ignored;
      std::getline(input, ignored);
      break;
    }
    token.push_back(character);
  }
  return token;
}

PgmImage read_pgm(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH", 422, "failed to read keepout mask: " + path.string());
  }
  const auto magic = next_pgm_token(input);
  const auto width_token = next_pgm_token(input);
  const auto height_token = next_pgm_token(input);
  const auto max_token = next_pgm_token(input);
  if ((magic != "P5" && magic != "P2") ||
    width_token.empty() || height_token.empty() || max_token.empty())
  {
    throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "invalid keepout PGM header");
  }

  PgmImage image;
  unsigned long max_value = 0UL;
  try {
    image.width = static_cast<std::uint32_t>(std::stoul(width_token));
    image.height = static_cast<std::uint32_t>(std::stoul(height_token));
    max_value = std::stoul(max_token);
  } catch (const std::exception &) {
    throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "invalid keepout PGM dimensions");
  }
  if (image.width == 0U || image.height == 0U || max_value == 0UL || max_value > 255UL) {
    throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "unsupported keepout PGM geometry");
  }
  const auto count = static_cast<std::size_t>(image.width) * image.height;
  image.pixels.resize(count);
  if (magic == "P5") {
    input.read(
      reinterpret_cast<char *>(image.pixels.data()),
      static_cast<std::streamsize>(image.pixels.size()));
    if (input.gcount() != static_cast<std::streamsize>(image.pixels.size())) {
      throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "truncated keepout PGM payload");
    }
  } else {
    for (std::size_t index = 0U; index < count; ++index) {
      const auto token = next_pgm_token(input);
      if (token.empty()) {
        throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "truncated keepout PGM payload");
      }
      try {
        const auto value = std::stoul(token);
        if (value > max_value) {
          throw std::out_of_range("PGM sample exceeds max value");
        }
        image.pixels[index] = static_cast<std::uint8_t>(
          std::lround(static_cast<double>(value) * 255.0 / static_cast<double>(max_value)));
      } catch (const std::exception &) {
        throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "invalid keepout PGM sample");
      }
    }
  }
  return image;
}

void reject_unmanaged_non_neutral_mask(
  const ReplaceKeepoutCommand & command,
  const MapYamlInfo & map_info)
{
  if (!fs::exists(command.map.keepout_mask_pgm)) {
    return;
  }
  const auto existing = read_pgm(command.map.keepout_mask_pgm);
  if (existing.width != map_info.width || existing.height != map_info.height) {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH", 422, "existing keepout mask dimensions do not match nav map");
  }
  const bool non_neutral = std::any_of(
    existing.pixels.begin(), existing.pixels.end(),
    [](const std::uint8_t value) {return value < 250U;});
  const auto semantic_path =
    command.map.root / "filters" / "keepout_semantic_layer.json";
  if (!fs::exists(semantic_path)) {
    if (!non_neutral || command.allow_unmanaged_non_neutral_mask) {
      return;
    }
    throw KeepoutLayerError(
            "UNMANAGED_NON_NEUTRAL_MASK",
            409,
            "existing keepout mask has active cells but no editable semantic layer");
  }

  try {
    const auto document = parse_document(read_binary_file(semantic_path), command.map);
    const auto nav_map = read_pgm(command.map.nav_map_pgm);
    std::size_t semantic_active_cells = 0U;
    std::vector<std::size_t> semantic_samples;
    const auto semantic_pixels = render_document(
      document, map_info, nav_map, semantic_active_cells, semantic_samples);
    const bool same_occupancy = std::equal(
      existing.pixels.begin(),
      existing.pixels.end(),
      semantic_pixels.begin(),
      [](const std::uint8_t existing_value, const std::uint8_t semantic_value) {
        return (existing_value < 250U) == (semantic_value < 250U);
      });
    if (!same_occupancy) {
      throw std::runtime_error(
              "semantic geometry does not reproduce the existing keepout occupancy");
    }
  } catch (const std::exception & error) {
    throw KeepoutLayerError(
            "KEEP_OUT_INTEGRITY_MISMATCH",
            409,
            "existing keepout semantic layer and mask disagree: " +
            std::string(error.what()));
  }
}

std::string mask_yaml_text(const KeepoutAssetPaths & paths, const MapYamlInfo & map)
{
  std::ostringstream yaml;
  yaml << std::fixed << std::setprecision(12)
       << "image: " << paths.mask_pgm.filename().string() << "\n"
       << "resolution: " << map.resolution << "\n"
       << "origin: [" << map.origin[0] << ", " << map.origin[1] << ", " << map.origin[2] << "]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.196\n"
       << "mode: trinary\n";
  return yaml.str();
}

std::string revision_for(
  const std::string & semantic_json,
  const MapManifest & map,
  const MapYamlInfo & info,
  const std::vector<std::uint8_t> & pixels)
{
  std::ostringstream source;
  source << kRendererVersion << '\n' << map.map_id << '\n'
         << info.width << 'x' << info.height << '@' << std::setprecision(17) << info.resolution << '\n'
         << info.origin[0] << ',' << info.origin[1] << ',' << info.origin[2] << '\n'
         << trim(semantic_json) << '\n';
  source.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  return "keepout-v1-fnv64-" + fixed_hex(fnv1a64(source.str()), 16);
}

std::string occupancy_digest_for_pgm(
  const MapYamlInfo & map,
  const std::vector<std::uint8_t> & pixels)
{
  std::string blocked;
  blocked.resize(static_cast<std::size_t>(map.width) * map.height);
  for (std::uint32_t row_from_bottom = 0U; row_from_bottom < map.height; ++row_from_bottom) {
    const auto pgm_row = map.height - 1U - row_from_bottom;
    for (std::uint32_t column = 0U; column < map.width; ++column) {
      blocked[static_cast<std::size_t>(row_from_bottom) * map.width + column] =
        pixels[static_cast<std::size_t>(pgm_row) * map.width + column] < 250U ? '\1' : '\0';
    }
  }
  return "occupancy-fnv64-" + fixed_hex(fnv1a64(blocked), 16);
}

std::vector<KeepoutCostmapSample> blocked_costmap_samples(
  const MapYamlInfo & map,
  const std::vector<std::uint8_t> & mask_pixels,
  const PgmImage & nav_map,
  const std::vector<std::size_t> & preferred_indices)
{
  if (nav_map.width != map.width || nav_map.height != map.height ||
    nav_map.pixels.size() != mask_pixels.size())
  {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH",
            422,
            "nav map PGM dimensions do not match its YAML geometry");
  }
  std::vector<std::size_t> candidates;
  for (std::size_t index = 0U; index < mask_pixels.size(); ++index) {
    if (mask_pixels[index] == 0U && nav_map.pixels[index] >= 250U) {
      candidates.push_back(index);
    }
  }
  if (candidates.empty()) {
    return {};
  }

  const auto make_sample = [&](const std::size_t pixel_index) {
      const std::uint32_t pgm_row =
        static_cast<std::uint32_t>(pixel_index / map.width);
      const std::uint32_t column =
        static_cast<std::uint32_t>(pixel_index % map.width);
      const std::uint32_t row_from_bottom = map.height - 1U - pgm_row;
      const double local_x = (static_cast<double>(column) + 0.5) * map.resolution;
      const double local_y =
        (static_cast<double>(row_from_bottom) + 0.5) * map.resolution;
      const double cosine = std::cos(map.origin[2]);
      const double sine = std::sin(map.origin[2]);
      return KeepoutCostmapSample{
        map.origin[0] + cosine * local_x - sine * local_y,
        map.origin[1] + sine * local_x + cosine * local_y};
    };

  std::vector<KeepoutCostmapSample> samples;
  samples.reserve(
    std::min(
      kMaxCostmapVerificationSamples,
      preferred_indices.size() + candidates.size()));
  std::set<std::size_t> selected_indices;
  for (const auto pixel_index : preferred_indices) {
    if (samples.size() >= kMaxCostmapVerificationSamples) {
      break;
    }
    if (pixel_index >= mask_pixels.size() ||
      mask_pixels[pixel_index] != 0U ||
      nav_map.pixels[pixel_index] < 250U)
    {
      throw KeepoutLayerError(
              "MAP_GEOMETRY_MISMATCH",
              422,
              "invalid per-feature costmap verification sample");
    }
    // Preserve one entry per feature even when multiple overlapping features
    // intentionally select the same map cell.
    samples.push_back(make_sample(pixel_index));
    selected_indices.insert(pixel_index);
  }

  const std::size_t remaining_capacity =
    kMaxCostmapVerificationSamples - samples.size();
  const std::size_t extra_count = std::min(remaining_capacity, candidates.size());
  for (std::size_t sample = 0U; sample < extra_count; ++sample) {
    const std::size_t candidate_offset = std::min(
      candidates.size() - 1U,
      (sample * candidates.size() + candidates.size() / 2U) / extra_count);
    const std::size_t pixel_index = candidates[candidate_offset];
    if (selected_indices.insert(pixel_index).second) {
      samples.push_back(make_sample(pixel_index));
    }
  }
  return samples;
}

std::vector<KeepoutCostmapSample> cleared_costmap_samples(
  const MapYamlInfo & map,
  const fs::path & previous_mask_path,
  const std::vector<std::uint8_t> & new_mask_pixels,
  const PgmImage & nav_map)
{
  if (!fs::exists(previous_mask_path)) {
    return {};
  }
  const auto previous = read_pgm(previous_mask_path);
  if (previous.width != map.width || previous.height != map.height ||
    previous.pixels.size() != new_mask_pixels.size() ||
    nav_map.pixels.size() != new_mask_pixels.size())
  {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH",
            422,
            "previous keepout mask dimensions do not match the selected nav map");
  }
  std::vector<std::size_t> candidates;
  for (std::size_t index = 0U; index < new_mask_pixels.size(); ++index) {
    if (previous.pixels[index] < 250U &&
      new_mask_pixels[index] >= 250U &&
      nav_map.pixels[index] >= 250U)
    {
      candidates.push_back(index);
    }
  }
  if (candidates.empty()) {
    return {};
  }

  const auto make_sample = [&](const std::size_t pixel_index) {
      const std::uint32_t pgm_row =
        static_cast<std::uint32_t>(pixel_index / map.width);
      const std::uint32_t column =
        static_cast<std::uint32_t>(pixel_index % map.width);
      const std::uint32_t row_from_bottom = map.height - 1U - pgm_row;
      const double local_x = (static_cast<double>(column) + 0.5) * map.resolution;
      const double local_y =
        (static_cast<double>(row_from_bottom) + 0.5) * map.resolution;
      const double cosine = std::cos(map.origin[2]);
      const double sine = std::sin(map.origin[2]);
      return KeepoutCostmapSample{
        map.origin[0] + cosine * local_x - sine * local_y,
        map.origin[1] + sine * local_x + cosine * local_y};
    };

  const auto sample_count = std::min(kMaxCostmapVerificationSamples, candidates.size());
  std::vector<KeepoutCostmapSample> samples;
  samples.reserve(sample_count);
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const std::size_t candidate_offset = std::min(
      candidates.size() - 1U,
      (sample * candidates.size() + candidates.size() / 2U) / sample_count);
    samples.push_back(make_sample(candidates[candidate_offset]));
  }
  return samples;
}

std::string temp_suffix()
{
  static std::atomic<std::uint64_t> sequence{0U};
  return ".tmp." + std::to_string(static_cast<long long>(::getpid())) + "." +
         std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed));
}

std::runtime_error posix_file_error(
  const std::string & action,
  const fs::path & path,
  const int error_number)
{
  return std::runtime_error(
    action + " " + path.string() + ": " + std::strerror(error_number));
}

void fsync_descriptor(const int descriptor, const fs::path & path)
{
  while (::fsync(descriptor) != 0) {
    if (errno == EINTR) {
      continue;
    }
    throw posix_file_error("failed to fsync", path, errno);
  }
}

void close_descriptor(int & descriptor, const fs::path & path)
{
  if (descriptor < 0) {
    return;
  }
  const int result = ::close(descriptor);
  const int close_error = errno;
  descriptor = -1;
  if (result != 0) {
    // Retrying close() after EINTR can close a newly-reused descriptor on Linux.
    throw posix_file_error("failed to close", path, close_error);
  }
}

fs::path parent_directory_for(const fs::path & path)
{
  return path.parent_path().empty() ? fs::path(".") : path.parent_path();
}

void fsync_parent_directory(const fs::path & path)
{
  const auto parent = parent_directory_for(path);
  int descriptor = ::open(
    parent.c_str(),
    O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (descriptor < 0) {
    throw posix_file_error("failed to open parent directory for", path, errno);
  }
  try {
    fsync_descriptor(descriptor, parent);
    close_descriptor(descriptor, parent);
  } catch (...) {
    if (descriptor >= 0) {
      (void)::close(descriptor);
    }
    throw;
  }
}

void atomic_write(const fs::path & path, const std::string & content)
{
  const auto parent = parent_directory_for(path);
  std::error_code directory_error;
  fs::create_directories(parent, directory_error);
  if (directory_error) {
    throw std::runtime_error(
            "failed to create keepout asset directory " + parent.string() +
            ": " + directory_error.message());
  }

  fs::path temporary;
  int descriptor = -1;
  for (std::size_t attempt = 0U; attempt < 16U; ++attempt) {
    temporary = fs::path(path.string() + temp_suffix());
    descriptor = ::open(
      temporary.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (descriptor >= 0) {
      break;
    }
    if (errno != EEXIST) {
      throw posix_file_error("failed to create temporary file for", path, errno);
    }
  }
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to create a unique temporary file for " + path.string());
  }

  bool renamed = false;
  try {
    std::size_t offset = 0U;
    while (offset < content.size()) {
      const auto remaining = content.size() - offset;
      const auto chunk = std::min(
        remaining,
        static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
      const auto written = ::write(descriptor, content.data() + offset, chunk);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw posix_file_error("failed to write temporary file for", path, errno);
      }
      if (written == 0) {
        throw std::runtime_error(
                "zero-byte write while updating keepout asset " + path.string());
      }
      offset += static_cast<std::size_t>(written);
    }
    fsync_descriptor(descriptor, temporary);
    close_descriptor(descriptor, temporary);
    if (::rename(temporary.c_str(), path.c_str()) != 0) {
      throw posix_file_error("failed to atomically replace", path, errno);
    }
    renamed = true;
    fsync_parent_directory(path);
  } catch (...) {
    if (descriptor >= 0) {
      (void)::close(descriptor);
    }
    if (!renamed) {
      (void)::unlink(temporary.c_str());
    }
    throw;
  }
}

void durable_remove(const fs::path & path)
{
  if (::unlink(path.c_str()) != 0) {
    if (errno == ENOENT) {
      return;
    }
    throw posix_file_error("failed to remove newly-created keepout asset", path, errno);
  }
  fsync_parent_directory(path);
}

fs::path commit_marker_path(const KeepoutAssetPaths & projection);

std::vector<FileSnapshot> capture_asset_snapshots(
  const std::vector<KeepoutAssetPaths> & projections)
{
  std::vector<FileSnapshot> snapshots;
  std::set<std::string> seen;
  const auto capture = [&](const fs::path & path, std::vector<FileSnapshot> & output) {
      const auto normalized = path.lexically_normal().string();
      if (!seen.insert(normalized).second) {
        return;
      }
      FileSnapshot snapshot;
      snapshot.path = path;
      snapshot.existed = fs::exists(path);
      if (snapshot.existed) {
        if (!fs::is_regular_file(path)) {
          throw KeepoutLayerError(
                  "ARTIFACT_COMMIT_FAILED", 500, "keepout asset is not a regular file: " + path.string());
        }
        snapshot.content = read_binary_file(path);
      }
      output.push_back(std::move(snapshot));
    };
  for (const auto & projection : projections) {
    capture(commit_marker_path(projection), snapshots);
    capture(projection.semantic_json, snapshots);
    capture(projection.mask_pgm, snapshots);
    capture(projection.mask_yaml, snapshots);
  }
  return snapshots;
}

bool file_matches(const fs::path & path, const std::string & expected)
{
  return fs::exists(path) && fs::is_regular_file(path) && read_binary_file(path) == expected;
}

fs::path commit_marker_path(const KeepoutAssetPaths & projection)
{
  return projection.mask_yaml.parent_path() / "keepout_commit.json";
}

std::string commit_marker_text(
  const KeepoutAssetPaths & projection,
  const std::string & revision,
  const std::string & semantic,
  const std::string & pgm,
  const std::string & yaml)
{
  std::ostringstream marker;
  marker << "{\"schema\":\"njrh.keepout.commit.v1\","
         << "\"revision\":" << json_string(revision) << ","
         << "\"semantic_file\":" << json_string(projection.semantic_json.filename().string()) << ","
         << "\"mask_yaml_file\":" << json_string(projection.mask_yaml.filename().string()) << ","
         << "\"mask_pgm_file\":" << json_string(projection.mask_pgm.filename().string()) << ","
         << "\"semantic_fnv64\":" << json_string(fixed_hex(fnv1a64(semantic), 16)) << ","
         << "\"mask_yaml_fnv64\":" << json_string(fixed_hex(fnv1a64(yaml), 16)) << ","
         << "\"mask_pgm_fnv64\":" << json_string(fixed_hex(fnv1a64(pgm), 16))
         << "}\n";
  return marker.str();
}

bool projections_match(
  const std::vector<KeepoutAssetPaths> & projections,
  const std::string & semantic,
  const std::string & pgm,
  const MapYamlInfo & map_info,
  const std::string & revision)
{
  return std::all_of(
    projections.begin(), projections.end(),
    [&](const KeepoutAssetPaths & projection) {
      const auto yaml = mask_yaml_text(projection, map_info);
      return file_matches(projection.semantic_json, semantic) &&
             file_matches(projection.mask_pgm, pgm) &&
             file_matches(projection.mask_yaml, yaml) &&
             file_matches(
        commit_marker_path(projection),
        commit_marker_text(projection, revision, semantic, pgm, yaml));
    });
}

void restore_asset_snapshots(const std::vector<FileSnapshot> & snapshots)
{
  for (auto iterator = snapshots.rbegin(); iterator != snapshots.rend(); ++iterator) {
    if (iterator->existed) {
      atomic_write(iterator->path, iterator->content);
    } else {
      durable_remove(iterator->path);
    }
  }
}

KeepoutMaskSummary previous_mask_summary(
  const ReplaceKeepoutCommand & command,
  const MapYamlInfo & map_info)
{
  KeepoutMaskSummary summary;
  summary.width = map_info.width;
  summary.height = map_info.height;
  summary.resolution = map_info.resolution;
  summary.origin_x = map_info.origin[0];
  summary.origin_y = map_info.origin[1];
  summary.origin_yaw = map_info.origin[2];
  if (!fs::exists(command.map.keepout_mask_pgm)) {
    summary.revision = "keepout-v1-absent";
    return summary;
  }
  const auto image = read_pgm(command.map.keepout_mask_pgm);
  if (image.width != map_info.width || image.height != map_info.height) {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH", 422, "existing keepout mask dimensions do not match nav map");
  }
  summary.active_cells = static_cast<std::size_t>(
    std::count_if(
      image.pixels.begin(), image.pixels.end(),
      [](const std::uint8_t value) {return value < 250U;}));
  summary.occupancy_digest = occupancy_digest_for_pgm(map_info, image.pixels);
  summary.blocked_costmap_samples =
    blocked_costmap_samples(
      map_info, image.pixels, read_pgm(command.map.nav_map_pgm), {});
  const auto semantic = read_optional_text_file(
    command.map.root / "filters" / "keepout_semantic_layer.json");
  summary.revision = revision_for(semantic, command.map, map_info, image.pixels);
  return summary;
}

bool runtime_proof_complete(const KeepoutRuntimeProof & proof)
{
  return proof.mask_server_active &&
         proof.filter_plugin_enabled &&
         proof.load_map_succeeded &&
         proof.mask_topic_matches &&
         proof.global_costmap_cleared &&
         proof.global_costmap_updated &&
         proof.global_costmap_content_matches;
}

bool runtime_may_have_changed(
  const KeepoutRuntimeProof & proof,
  const bool adapter_reported_success)
{
  return adapter_reported_success ||
         proof.mutation_state == KeepoutRuntimeMutationState::MAY_HAVE_CHANGED ||
         proof.load_map_succeeded ||
         proof.mask_topic_matches ||
         proof.global_costmap_cleared ||
         proof.global_costmap_updated ||
         proof.global_costmap_content_matches;
}

}  // namespace

KeepoutLayerError::KeepoutLayerError(
  std::string code,
  const int http_status,
  const std::string & message)
: std::runtime_error(message), code_(std::move(code)), http_status_(http_status)
{
}

const std::string & KeepoutLayerError::code() const noexcept
{
  return code_;
}

int KeepoutLayerError::http_status() const noexcept
{
  return http_status_;
}

KeepoutMaskSummary KeepoutLayerModule::inspect(const MapManifest & map) const
{
  if (map.map_id.empty() || map.building_id.empty() || map.floor_id.empty()) {
    throw KeepoutLayerError("MAP_NOT_FOUND", 404, "selected map manifest is incomplete");
  }
  const auto map_info = read_nav_map_info(map.nav_map_yaml);
  if (!map_info) {
    throw KeepoutLayerError(
            "MAP_GEOMETRY_MISMATCH", 422, "failed to read selected nav map geometry");
  }
  ReplaceKeepoutCommand command;
  command.map = map;
  reject_unmanaged_non_neutral_mask(command, *map_info);
  return previous_mask_summary(command, *map_info);
}

ReplaceKeepoutResult KeepoutLayerModule::replace(
  const ReplaceKeepoutCommand & command,
  KeepoutRuntimePort * runtime) const
{
  if (command.map.map_id.empty() || command.map.building_id.empty() || command.map.floor_id.empty()) {
    throw KeepoutLayerError("MAP_NOT_FOUND", 404, "selected map manifest is incomplete");
  }
  const auto map_info = read_nav_map_info(command.map.nav_map_yaml);
  if (!map_info) {
    throw KeepoutLayerError("MAP_GEOMETRY_MISMATCH", 422, "failed to read selected nav map geometry");
  }
  reject_unmanaged_non_neutral_mask(command, *map_info);
  const auto previous_mask = previous_mask_summary(command, *map_info);
  if (command.expected_revision &&
    *command.expected_revision != previous_mask.revision)
  {
    throw KeepoutLayerError(
            "REVISION_CONFLICT",
            412,
            "keepout layer changed since it was read; expected " +
            *command.expected_revision + " but current revision is " +
            previous_mask.revision);
  }
  const auto document = parse_document(command.request_json, command.map);
  const auto semantic = canonical_document_json(document, command.map);
  const auto nav_map = read_pgm(command.map.nav_map_pgm);
  std::size_t active_cells = 0U;
  std::vector<std::size_t> feature_free_samples;
  const auto pixels = render_document(
    document, *map_info, nav_map, active_cells, feature_free_samples);
  const auto verification_samples =
    blocked_costmap_samples(*map_info, pixels, nav_map, feature_free_samples);
  const auto cleared_samples =
    cleared_costmap_samples(*map_info, command.map.keepout_mask_pgm, pixels, nav_map);
  if (active_cells > 0U && verification_samples.empty()) {
    throw KeepoutLayerError(
            "KEEP_OUT_NO_TRAVERSABLE_INTERSECTION",
            422,
            "keepout geometry does not cover any free cell in the selected nav map");
  }
  const auto pgm = pgm_payload(map_info->width, map_info->height, pixels);
  const auto revision = revision_for(semantic, command.map, *map_info, pixels);

  if (command.projections.empty()) {
    throw KeepoutLayerError("ARTIFACT_COMMIT_FAILED", 500, "no keepout asset projection was provided");
  }
  const bool changed =
    previous_mask.revision != revision ||
    !projections_match(command.projections, semantic, pgm, *map_info, revision);
  const auto snapshots = changed ?
    capture_asset_snapshots(command.projections) : std::vector<FileSnapshot>{};
  if (changed) {
    try {
      for (const auto & projection : command.projections) {
        atomic_write(projection.semantic_json, semantic);
        atomic_write(projection.mask_pgm, pgm);
        const auto yaml = mask_yaml_text(projection, *map_info);
        atomic_write(projection.mask_yaml, yaml);
      }
      for (const auto & projection : command.projections) {
        const auto yaml = mask_yaml_text(projection, *map_info);
        atomic_write(
          commit_marker_path(projection),
          commit_marker_text(projection, revision, semantic, pgm, yaml));
      }
      if (!projections_match(command.projections, semantic, pgm, *map_info, revision)) {
        throw std::runtime_error(
                "keepout artifact readback did not match the committed transaction");
      }
    } catch (const KeepoutLayerError & error) {
      try {
        restore_asset_snapshots(snapshots);
      } catch (const std::exception & rollback_error) {
        throw KeepoutLayerError(
                "ARTIFACT_ROLLBACK_FAILED",
                500,
                std::string(error.what()) + "; rollback failed: " + rollback_error.what());
      }
      throw;
    } catch (const std::exception & error) {
      try {
        restore_asset_snapshots(snapshots);
      } catch (const std::exception & rollback_error) {
        throw KeepoutLayerError(
                "ARTIFACT_ROLLBACK_FAILED",
                500,
                std::string(error.what()) + "; rollback failed: " + rollback_error.what());
      }
      throw KeepoutLayerError("ARTIFACT_COMMIT_FAILED", 500, error.what());
    }
  }

  ReplaceKeepoutResult result;
  result.persisted = true;
  result.changed = changed;
  result.runtime_selected = command.runtime_selected;
  result.effective_on_next_activation = !command.runtime_selected;
  result.outcome = !changed && !command.runtime_selected ?
    "NO_CHANGE" :
    (command.runtime_selected ? "SAVED_NOT_APPLIED" : "SAVED_DEFERRED_INACTIVE");
  result.revision = revision;
  result.previous_revision = previous_mask.revision;
  result.mask.width = map_info->width;
  result.mask.height = map_info->height;
  result.mask.resolution = map_info->resolution;
  result.mask.origin_x = map_info->origin[0];
  result.mask.origin_y = map_info->origin[1];
  result.mask.origin_yaw = map_info->origin[2];
  result.mask.active_cells = active_cells;
  result.mask.occupancy_digest = occupancy_digest_for_pgm(*map_info, pixels);
  result.mask.revision = revision;
  result.mask.blocked_costmap_samples = verification_samples;
  result.mask.cleared_costmap_samples = cleared_samples;

  if (command.runtime_selected && command.runtime_apply_requested) {
    if (runtime == nullptr) {
      if (changed) {
        try {
          restore_asset_snapshots(snapshots);
        } catch (const std::exception & rollback_error) {
          throw KeepoutLayerError(
                  "ARTIFACT_ROLLBACK_FAILED",
                  500,
                  "runtime keepout adapter is unavailable; asset rollback failed: " +
                  std::string(rollback_error.what()));
        }
      }
      throw KeepoutLayerError("FILTER_RUNTIME_UNAVAILABLE", 503, "runtime keepout adapter is unavailable");
    }
    std::string error;
    bool applied = false;
    std::string failure_code = "RUNTIME_LOAD_FAILED";
    try {
      applied = runtime->apply(
        command.projections.front().mask_yaml, result.mask, result.runtime_proof, error);
    } catch (const std::exception & runtime_error) {
      error = runtime_error.what();
    }
    const bool runtime_changed =
      runtime_may_have_changed(result.runtime_proof, applied);
    if (applied && !runtime_proof_complete(result.runtime_proof)) {
      applied = false;
      failure_code = "RUNTIME_PROOF_FAILED";
      error =
        "runtime adapter did not prove mask server active, keepout plugin enabled, "
        "mask reload, mask topic match, global costmap clear, a post-clear costmap update, "
        "and matching global costmap content";
    }
    if (!applied) {
      if (error.empty()) {
        error = "runtime keepout adapter rejected the update";
      }
      std::string asset_rollback_error;
      if (changed) {
        try {
          restore_asset_snapshots(snapshots);
        } catch (const std::exception & rollback_error) {
          asset_rollback_error = rollback_error.what();
        }
      }

      std::string runtime_rollback_error;
      if (runtime_changed) {
        KeepoutRuntimeProof rollback_proof;
        bool restored = false;
        try {
          restored = runtime->restore(
            command.projections.front().mask_yaml,
            previous_mask,
            rollback_proof,
            runtime_rollback_error);
        } catch (const std::exception & restore_error) {
          runtime_rollback_error = restore_error.what();
        }
        if (restored && !runtime_proof_complete(rollback_proof)) {
          restored = false;
          runtime_rollback_error =
            "runtime adapter returned incomplete proof while restoring previous mask";
        }
        if (!restored && runtime_rollback_error.empty()) {
          runtime_rollback_error =
            "runtime adapter rejected the previous keepout mask";
        }
      }

      if (!runtime_rollback_error.empty() || !asset_rollback_error.empty()) {
        std::string rollback_detail = error;
        if (!asset_rollback_error.empty()) {
          rollback_detail += "; asset rollback failed: " + asset_rollback_error;
        }
        if (!runtime_rollback_error.empty()) {
          rollback_detail +=
            "; previous keepout mask runtime restore failed: " +
            runtime_rollback_error;
        }
        throw KeepoutLayerError(
                runtime_rollback_error.empty() ?
                "ARTIFACT_ROLLBACK_FAILED" : "RUNTIME_ROLLBACK_FAILED",
                500,
                rollback_detail);
      }
      throw KeepoutLayerError(failure_code, 503, error);
    }
    result.runtime_effective = true;
    result.effective_on_next_activation = false;
    result.outcome = "APPLIED";
  }
  return result;
}

}  // namespace robot_api_server
