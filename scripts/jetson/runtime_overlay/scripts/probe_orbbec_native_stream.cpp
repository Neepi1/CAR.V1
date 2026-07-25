#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;

double elapsedSeconds(const Clock::time_point &start, const Clock::time_point &end) {
  return std::chrono::duration<double>(end - start).count();
}

double parsePositiveDouble(const char *value, const char *name) {
  char *end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a positive number");
  }
  return parsed;
}

std::uint32_t parsePositiveUint(const char *value, const char *name) {
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value || *end != '\0' || parsed == 0 || parsed > UINT32_MAX) {
    throw std::invalid_argument(std::string(name) + " must be a positive integer");
  }
  return static_cast<std::uint32_t>(parsed);
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--list-profiles") {
      ob::Pipeline pipeline;
      const auto profiles = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
      std::cout << "[orbbec-native] depth_profile_count="
                << profiles->getCount() << std::endl;
      for (std::uint32_t index = 0; index < profiles->getCount(); ++index) {
        const auto profile = profiles->getProfile(index);
        if (!profile->is<ob::VideoStreamProfile>()) {
          continue;
        }
        const auto video = profile->as<ob::VideoStreamProfile>();
        std::cout << "[orbbec-native] profile_index=" << index
                  << " width=" << video->getWidth()
                  << " height=" << video->getHeight()
                  << " fps=" << video->getFps()
                  << " format="
                  << ob::TypeHelper::convertOBFormatTypeToString(video->getFormat())
                  << std::endl;
      }
      return 0;
    }

    const double duration_sec =
        argc > 1 ? parsePositiveDouble(argv[1], "duration_sec") : 180.0;
    const double max_gap_sec =
        argc > 2 ? parsePositiveDouble(argv[2], "max_gap_sec") : 1.0;
    const std::uint32_t width =
        argc > 3 ? parsePositiveUint(argv[3], "width") : 424;
    const std::uint32_t height =
        argc > 4 ? parsePositiveUint(argv[4], "height") : 266;
    const std::uint32_t fps =
        argc > 5 ? parsePositiveUint(argv[5], "fps") : 30;

    std::cout << std::fixed << std::setprecision(3)
              << "[orbbec-native] profile=" << width << "x" << height
              << "@" << fps << " format=Y16"
              << " duration_sec=" << duration_sec
              << " max_gap_sec=" << max_gap_sec << std::endl;

    ob::Pipeline pipeline;
    auto config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH, width, height, fps, OB_FORMAT_Y16);
    pipeline.start(config);

    const auto started_at = Clock::now();
    auto last_frame_at = started_at;
    auto last_report_at = started_at;
    std::uint64_t frame_count = 0;
    std::uint64_t timeout_count = 0;
    double max_observed_gap_sec = 0.0;
    bool failed = false;
    std::string failure_reason;

    while (elapsedSeconds(started_at, Clock::now()) < duration_sec) {
      auto frameset = pipeline.waitForFrames(1100);
      const auto now = Clock::now();
      const double gap_sec = elapsedSeconds(last_frame_at, now);

      if (!frameset || !frameset->getDepthFrame()) {
        ++timeout_count;
        max_observed_gap_sec = std::max(max_observed_gap_sec, gap_sec);
        if (gap_sec >= max_gap_sec) {
          failed = true;
          failure_reason = "depth_frame_timeout";
          std::cerr << "[orbbec-native] stream_failure_sec="
                    << elapsedSeconds(started_at, now)
                    << " gap_sec=" << gap_sec
                    << " timeouts=" << timeout_count << std::endl;
          break;
        }
        continue;
      }

      if (frame_count > 0) {
        max_observed_gap_sec = std::max(max_observed_gap_sec, gap_sec);
      }
      last_frame_at = now;
      ++frame_count;

      if (elapsedSeconds(last_report_at, now) >= 10.0) {
        const double run_sec = elapsedSeconds(started_at, now);
        std::cout << "[orbbec-native] progress_sec=" << run_sec
                  << " frames=" << frame_count
                  << " rate_hz=" << (frame_count / run_sec)
                  << " max_gap_sec=" << max_observed_gap_sec
                  << " timeouts=" << timeout_count << std::endl;
        last_report_at = now;
      }
    }

    try {
      pipeline.stop();
    } catch (const ob::Error &error) {
      std::cerr << "[orbbec-native] stop_warning"
                << " function=" << error.getFunction()
                << " args=" << error.getArgs()
                << " message=" << error.what() << std::endl;
      if (!failed) {
        failed = true;
        failure_reason = "sdk_stop_exception";
      }
    }

    const double run_sec = elapsedSeconds(started_at, Clock::now());
    const double rate_hz = run_sec > 0.0 ? frame_count / run_sec : 0.0;
    const double min_rate_hz = std::max(1.0, static_cast<double>(fps) * 0.8);
    if (!failed && rate_hz < min_rate_hz) {
      failed = true;
      failure_reason = "depth_rate_below_expected";
    }

    std::cout << "[orbbec-native] verdict=" << (failed ? "FAIL" : "PASS")
              << " reason=" << (failed ? failure_reason : "stable")
              << " run_sec=" << run_sec
              << " frames=" << frame_count
              << " rate_hz=" << rate_hz
              << " min_rate_hz=" << min_rate_hz
              << " max_gap_sec=" << max_observed_gap_sec
              << " timeouts=" << timeout_count << std::endl;
    return failed ? 2 : 0;
  } catch (const ob::Error &error) {
    std::cerr << "[orbbec-native] verdict=FAIL reason=sdk_exception"
              << " function=" << error.getFunction()
              << " args=" << error.getArgs()
              << " message=" << error.what() << std::endl;
    return 3;
  } catch (const std::exception &error) {
    std::cerr << "[orbbec-native] verdict=FAIL reason=exception"
              << " message=" << error.what() << std::endl;
    return 4;
  }
}
