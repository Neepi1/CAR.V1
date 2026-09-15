// Standalone algorithm microbenchmark, not a ROS node or whole-process CPU claim.
// Build this file with the package's ROS include paths and -std=c++17 -O2.
// Usage: benchmark_fused_normalization [paired_samples=400]
// Input construction/reset, output verification and checksums are outside timing.
// The retained two-pass reference mirrors the old canonical-A loop followed by
// build_latest_normalized_buffer/read_point, for this valid contiguous fixture.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_hesai_jt128/detail/fused_normalization.hpp"

#if defined(_MSC_VER)
#define NORMALIZATION_NOINLINE __declspec(noinline)
#else
#define NORMALIZATION_NOINLINE __attribute__((noinline))
#endif

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using WallClock = std::chrono::steady_clock;

struct Point
{
  float x;
  float y;
  float z;
  float intensity;
};
static_assert(sizeof(float) == 4U && sizeof(Point) == 16U, "Expected FLOAT32 XYZI");

constexpr std::size_t kPointCount = 40000U;
constexpr std::size_t kPointStep = 32U;
constexpr std::array<float, 9> kRotation{0, 1, 0, -1, 0, 0, 0, 0, 1};
volatile std::uint64_t checksum_sink = 0U;

float read_float32(const std::uint8_t * data)
{
  float value;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

template<class Value>
void put(Cloud & cloud, const std::size_t offset, const Value value)
{
  std::memcpy(cloud.data.data() + offset, &value, sizeof(value));
}

bool find_float32_field_offset(
  const Cloud & cloud, const std::string & name, std::size_t & offset)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name &&
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count >= 1U)
    {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

struct Offsets
{
  bool valid{false};
  bool has_intensity{false};
  std::size_t x{0U}, y{0U}, z{0U}, intensity{0U};
};

Offsets field_offsets(const Cloud & cloud)
{
  Offsets offsets;
  offsets.valid = !cloud.is_bigendian &&
    find_float32_field_offset(cloud, "x", offsets.x) &&
    find_float32_field_offset(cloud, "y", offsets.y) &&
    find_float32_field_offset(cloud, "z", offsets.z);
  offsets.has_intensity = offsets.valid &&
    find_float32_field_offset(cloud, "intensity", offsets.intensity) &&
    offsets.intensity + sizeof(float) <= cloud.point_step;
  return offsets;
}

std::optional<Point> read_point(
  const Cloud & cloud, const Offsets & offsets, const std::size_t flat_index)
{
  if (!offsets.valid || cloud.width == 0U || cloud.point_step == 0U) {
    return std::nullopt;
  }
  const std::size_t row = flat_index / cloud.width;
  const std::size_t column = flat_index % cloud.width;
  const std::size_t input_offset = row * cloud.row_step + column * cloud.point_step;
  if (input_offset + cloud.point_step > cloud.data.size()) {
    return std::nullopt;
  }
  const auto * data = cloud.data.data() + input_offset;
  return Point{
    read_float32(data + offsets.x), read_float32(data + offsets.y),
    read_float32(data + offsets.z),
    offsets.has_intensity ? read_float32(data + offsets.intensity) : 0.0F};
}

NORMALIZATION_NOINLINE std::optional<bool> old_two_pass(
  Cloud & cloud, std::vector<Point> & points)
{
  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
  for (std::size_t index = 0U; index < point_count; ++index) {
    auto * data = cloud.data.data() + index * cloud.point_step;
    // Same float exchange/negation as the old loop. memcpy avoids inheriting
    // its reinterpret_cast alignment/aliasing assumption in the benchmark.
    const float raw_x = read_float32(data);
    const float raw_y = read_float32(data + sizeof(float));
    const float canonical_y = -raw_x;
    std::memcpy(data, &raw_y, sizeof(float));
    std::memcpy(data + sizeof(float), &canonical_y, sizeof(float));
  }

  points.clear();
  const auto offsets = field_offsets(cloud);
  if (!offsets.valid || cloud.point_step == 0U || cloud.width == 0U || cloud.height == 0U) {
    return false;
  }
  if (points.capacity() < point_count) {
    points.reserve(point_count);
  }
  for (std::size_t index = 0U; index < point_count; ++index) {
    const auto point = read_point(cloud, offsets, index);
    if (!point) {
      break;
    }
    points.push_back(*point);
  }
  return offsets.has_intensity;
}

NORMALIZATION_NOINLINE std::optional<bool> fused(
  Cloud & cloud, std::vector<Point> & points)
{
  // This is the production helper, not a benchmark implementation of it.
  return robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, kRotation, points);
}

Cloud fixture()
{
  Cloud cloud;
  cloud.header.frame_id = "lidar_link";
  cloud.header.stamp.sec = 12345;
  cloud.header.stamp.nanosec = 678901234U;
  cloud.width = static_cast<std::uint32_t>(kPointCount);
  cloud.height = 1U;
  cloud.point_step = static_cast<std::uint32_t>(kPointStep);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  const auto add_field = [&cloud](
    const char * name, const std::uint32_t offset, const std::uint8_t datatype) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = 1U;
      cloud.fields.push_back(field);
    };
  add_field("x", 0U, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4U, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8U, sensor_msgs::msg::PointField::FLOAT32);
  add_field("intensity", 16U, sensor_msgs::msg::PointField::FLOAT32);
  add_field("ring", 20U, sensor_msgs::msg::PointField::UINT16);
  add_field("timestamp", 24U, sensor_msgs::msg::PointField::FLOAT64);
  cloud.data.resize(kPointCount * kPointStep);
  for (std::size_t i = 0U; i < cloud.data.size(); ++i) {
    cloud.data[i] = static_cast<std::uint8_t>((i * 37U + 19U) & 0xffU);
  }
  for (std::size_t i = 0U; i < kPointCount; ++i) {
    const auto offset = i * kPointStep;
    put(cloud, offset, (static_cast<float>(i % 401U) - 200.0F) * 0.125F);
    put(cloud, offset + 4U, (static_cast<float>(i % 397U) - 198.0F) * 0.0625F);
    put(cloud, offset + 8U, (static_cast<float>(i % 53U) - 26.0F) * 0.03125F);
    put(cloud, offset + 16U, static_cast<float>(i % 256U));
    put(cloud, offset + 20U, static_cast<std::uint16_t>(i % 128U));
    put(cloud, offset + 24U, 12345.0 + static_cast<double>(i) * 0.000002);
  }
  return cloud;
}

double cpu_now_us()
{
#if defined(__linux__)
  timespec stamp{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &stamp) != 0) {
    throw std::runtime_error("CLOCK_THREAD_CPUTIME_ID unavailable");
  }
  return static_cast<double>(stamp.tv_sec) * 1.0e6 + stamp.tv_nsec * 1.0e-3;
#else
  const auto stamp = std::clock();
  if (stamp == static_cast<std::clock_t>(-1)) {
    throw std::runtime_error("process CPU clock unavailable");
  }
  return static_cast<double>(stamp) * 1.0e6 / CLOCKS_PER_SEC;
#endif
}

struct Sample
{
  double wall_us;
  double cpu_us;
};
using Algorithm = std::optional<bool> (*)(Cloud &, std::vector<Point> &);

Sample run_one(
  const Cloud & source, Cloud & output, std::vector<Point> & points, Algorithm algorithm)
{
  output = source;  // Deliberately excluded; both paths receive identical warm input.
  const auto cpu_start = cpu_now_us();
  const auto wall_start = WallClock::now();
  std::atomic_signal_fence(std::memory_order_seq_cst);
  const auto result = algorithm(output, points);
  std::atomic_signal_fence(std::memory_order_seq_cst);
  const auto wall_end = WallClock::now();
  const auto cpu_end = cpu_now_us();
  if (!result.has_value() || !*result || points.size() != kPointCount) {
    throw std::runtime_error("algorithm did not accept the complete FLOAT32-intensity fixture");
  }
  return Sample{
    std::chrono::duration<double, std::micro>(wall_end - wall_start).count(),
    cpu_end - cpu_start};
}

std::uint64_t checksum(const Cloud & cloud, const std::vector<Point> & points)
{
  std::uint64_t hash = 14695981039346656037ULL;
  const auto consume = [&hash](const std::uint8_t * data, const std::size_t size) {
      for (std::size_t i = 0U; i < size; ++i) {
        hash = (hash ^ data[i]) * 1099511628211ULL;
      }
    };
  consume(cloud.data.data(), cloud.data.size());
  consume(reinterpret_cast<const std::uint8_t *>(points.data()), points.size() * sizeof(Point));
  return hash;
}

void verify(
  const Cloud & old_cloud, const std::vector<Point> & old_points,
  const Cloud & fused_cloud, const std::vector<Point> & fused_points)
{
  // Generated message equality includes metadata, all fields, padding and data.
  if (!(old_cloud == fused_cloud) || old_points.size() != fused_points.size()) {
    throw std::runtime_error("full PointCloud2 or normalized point count differs");
  }
  for (std::size_t i = 0U; i < old_points.size(); ++i) {
    if (std::memcmp(&old_points[i], &fused_points[i], sizeof(Point)) != 0) {
      throw std::runtime_error("normalized point bytes differ at index " + std::to_string(i));
    }
  }
  checksum_sink = (checksum_sink * 1099511628211ULL) ^ checksum(old_cloud, old_points);
  checksum_sink = (checksum_sink * 1099511628211ULL) ^ checksum(fused_cloud, fused_points);
}

double percentile(std::vector<double> values, const double fraction)
{
  std::sort(values.begin(), values.end());
  if (fraction == 0.5 && values.size() % 2U == 0U) {
    return (values[values.size() / 2U - 1U] + values[values.size() / 2U]) * 0.5;
  }
  const auto index = static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1U;
  return values.at(index);
}

void report(const char * label, const std::vector<Sample> & samples)
{
  std::vector<double> wall, cpu;
  for (const auto & sample : samples) {
    wall.push_back(sample.wall_us);
    cpu.push_back(sample.cpu_us);
  }
  std::cout << label << " wall_median_us=" << percentile(wall, 0.5)
            << " wall_p95_us=" << percentile(wall, 0.95)
            << " cpu_median_us=" << percentile(cpu, 0.5)
            << " cpu_p95_us=" << percentile(cpu, 0.95) << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc > 2) {
      throw std::runtime_error("usage: benchmark_fused_normalization [paired_samples=400]");
    }
    std::size_t paired_samples = 400U;
    if (argc == 2) {
      std::size_t consumed = 0U;
      const std::string argument = argv[1];
      const auto parsed = std::stoull(argument, &consumed);
      if (consumed != argument.size() || argument.empty() || argument.front() == '-' ||
        parsed < 4U || parsed > 1000000U)
      {
        throw std::runtime_error("paired_samples must be an integer between 4 and 1000000");
      }
      paired_samples = static_cast<std::size_t>(parsed);
    }

    const auto source = fixture();
    auto old_cloud = source;
    auto fused_cloud = source;
    std::vector<Point> old_points, fused_points;
    old_points.reserve(kPointCount);
    fused_points.reserve(kPointCount);
    std::vector<Sample> old_samples, fused_samples;
    old_samples.reserve(paired_samples);
    fused_samples.reserve(paired_samples);
    constexpr std::size_t warmup_pairs = 20U;
    for (std::size_t pair = 0U; pair < warmup_pairs + paired_samples; ++pair) {
      Sample old_sample{}, fused_sample{};
      if (pair % 2U == 0U) {
        old_sample = run_one(source, old_cloud, old_points, old_two_pass);
        fused_sample = run_one(source, fused_cloud, fused_points, fused);
      } else {
        fused_sample = run_one(source, fused_cloud, fused_points, fused);
        old_sample = run_one(source, old_cloud, old_points, old_two_pass);
      }
      verify(old_cloud, old_points, fused_cloud, fused_points);
      if (pair >= warmup_pairs) {
        old_samples.push_back(old_sample);
        fused_samples.push_back(fused_sample);
      }
    }

    std::cout << std::fixed << std::setprecision(3)
              << "points=" << kPointCount << " point_step=" << kPointStep
              << " intensity_offset=16 paired_samples=" << paired_samples
              << " warmup_pairs=" << warmup_pairs << " order=alternating_AB_BA\n";
#if defined(__linux__)
    std::cout << "cpu_clock=thread_cpu_time\n";
#else
    std::cout << "cpu_clock=process_cpu_time (single-threaded benchmark)\n";
#endif
    report("old_two_pass", old_samples);
    report("production_fused_helper", fused_samples);
    std::cout << "full_cloud_and_normalized_byte_equivalence=PASS checksum="
              << checksum_sink << '\n'
              << "Scope: warm-input, preallocated CPU algorithm microbenchmark only. "
              << "Excludes input copy, ROS publish, DDS, scan worker and process overhead; "
              << "not whole-process CPU savings or hardware acceptance.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "benchmark failed: " << error.what() << '\n';
    return 1;
  }
}
