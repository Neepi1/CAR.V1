#pragma once

// Diagnostic transport only. The producer never opens files, serializes JSON,
// waits for space, or changes control state. One producer / one writer.
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>
#include <unistd.h>

namespace robot_nav_config::navlite_snapshot
{
#ifndef NAVLITE_SNAPSHOT_REQUEST_PATH
#define NAVLITE_SNAPSHOT_REQUEST_PATH "/tmp/njrh_navlite_capture.request"
#endif
constexpr std::size_t kSlots = 32, kMapCells = 65536, kPathPoses = 2048;
constexpr std::size_t kSteps = 256, kHistory = 128, kFootprint = 32;
template<std::size_t N> inline bool text(char (&out)[N], const char * source) noexcept
{
  if (!source) {out[0] = 0; return false;}
  for (std::size_t n = 0; n + 1 < N; ++n) {
    out[n] = source[n];
    if (source[n] == 0) {return false;}
  }
  out[N - 1] = 0;
  return source[N - 1] != 0;
}

struct Metadata
{
  uint64_t generation{}, compute_seq{}, dropped_before{};
  int64_t start_monotonic_ns{}, end_monotonic_ns{}, start_wall_ns{}, pose_stamp_ns{}, path_stamp_ns{};
  int64_t map_copy_monotonic_ns{}, model_input_monotonic_ns{};
  char pose_frame[96]{}, path_frame[96]{}, map_frame[96]{}, base_frame[96]{};
  char stage[64]{}, exception_type[96]{}, exception[512]{};
  bool incomplete{}, command_returned{}, sequence_valid{}, model_enabled{}, smoother_valid{};
  int fail_flag{-1}; unsigned optimize_passes{}, passes_entered_failed{};
  uint32_t map_width{}, map_height{}, map_count{}, path_count{}, sequence_count{}, command_offset{};
  uint32_t history_count{}, footprint_count{}, path_original_count{}, sequence_original_count{};
  double resolution{}, origin_x{}, origin_y{}, model_dt{}, min_turning_radius{};
  double smoother_age_sec{-1}, issued_age_sec{-1}, smoother_linear{}, smoother_angular{};
  std::array<double, 7> pose{};
  // vx,vy,vz,wx,wy,wz. Input odom is the controller's Twist: no source stamp/covariance exists here.
  std::array<double, 6> odom{}, command{};
  // linear_delay,tau,accel,decel,steering_delay,tau,wheelbase,track
  std::array<double, 8> dynamics{};
  // linear_accel,linear_decel,angular_accel,angular_decel (positive magnitudes)
  std::array<double, 4> smoother_limits{};
  // vx_min,vx_max,vy,wz -- effective optimizer constraints at this calculation.
  std::array<double, 4> constraints{};
  std::array<std::array<double, 3>, kFootprint> footprint{};
  std::array<std::array<double, 3>, kHistory> issued{}; // relative time, vx, wz
};
struct Frame : Metadata
{
  std::array<unsigned char, kMapCells> map{};
  std::array<std::array<double, 7>, kPathPoses> path{};
  std::array<int64_t, kPathPoses> path_stamps{};
  std::array<std::array<char, 96>, kPathPoses> path_frames{};
  std::array<std::array<double, 3>, kSteps> sequence{}; // pre-shift final vx,vy,wz
};

class Recorder
{
public:
  explicit Recorder(std::string request = NAVLITE_SNAPSHOT_REQUEST_PATH,
    std::string root = "/tmp/njrh_reports")
  : request_(std::move(request)), root_(std::filesystem::weakly_canonical(root)),
    slots_(std::make_unique<Frame[]>(kSlots)), worker_([this] {run();}) {}
  ~Recorder() {stop();}
  Recorder(const Recorder &) = delete;
  Recorder & operator=(const Recorder &) = delete;
  static int64_t monotonic_ns() noexcept
  {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  static int64_t wall_ns() noexcept
  {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
  bool enabled() const noexcept {return active_.load(std::memory_order_acquire) != 0;}
  uint64_t dropped() const noexcept {return dropped_.load(std::memory_order_relaxed);}

  Frame * begin(uint64_t seq) noexcept
  {
    const auto generation = active_.load(std::memory_order_acquire);
    if (!generation) {return nullptr;}
    if (producer_busy_.test_and_set(std::memory_order_acquire)) {++dropped_; return nullptr;}
    in_flight_.store(true, std::memory_order_release);
    if (active_.load(std::memory_order_acquire) != generation) {
      in_flight_.store(false, std::memory_order_release);
      producer_busy_.clear(std::memory_order_release); return nullptr;
    }
    const auto head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= kSlots) {
      ++dropped_; in_flight_.store(false, std::memory_order_release);
      producer_busy_.clear(std::memory_order_release); return nullptr;
    }
    auto & frame = slots_[head % kSlots];
    static_cast<Metadata &>(frame) = Metadata{};
    frame.generation = generation; frame.compute_seq = seq; frame.dropped_before = dropped();
    frame.start_monotonic_ns = monotonic_ns(); frame.start_wall_ns = wall_ns();
    return &frame;
  }
  void finish(Frame * frame) noexcept
  {
    if (!frame) {return;}
    frame->end_monotonic_ns = monotonic_ns();
    head_.fetch_add(1, std::memory_order_release);
    in_flight_.store(false, std::memory_order_release);
    producer_busy_.clear(std::memory_order_release);
  }
  // Called only after the owner's compute callback has stopped. No business/costmap lock is held.
  void stop() noexcept
  {
    active_.store(0, std::memory_order_release);
    stop_.store(true, std::memory_order_release); wake_.notify_all();
    try {if (worker_.joinable()) {worker_.join();}} catch (...) {}
  }

private:
  using Json = nlohmann::json;
  using Clock = std::chrono::steady_clock;
  void close(const std::string & reason)
  {
    active_.store(0, std::memory_order_release);
    if (directory_.empty()) {return;}
    binary_.flush(); index_.flush();
    const bool io_ok = binary_.good() && index_.good();
    const auto pending = head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    const bool in_flight = in_flight_.load(std::memory_order_acquire);
    std::ofstream status(directory_ / "status.json");
    status << Json{{"schema", 1}, {"session", session_}, {"reason", reason},
      {"frames", written_}, {"payload_bytes", bytes_}, {"dropped", dropped() - dropped_base_},
      {"dropped_lifetime", dropped()}, {"dropped_baseline", dropped_base_},
      {"stale_generation_frames", stale_}, {"write_errors", errors_ + (io_ok ? 0 : 1)},
      {"pending_at_close", pending}, {"in_flight_at_close", in_flight},
      {"complete", (reason == "owner_stopped" || reason == "request_removed") &&
        io_ok && errors_ == 0 && dropped() == dropped_base_ && stale_ == 0 && !pending && !in_flight},
      {"ended_monotonic_ns", monotonic_ns()}}.dump(2);
    binary_.close(); index_.close(); directory_.clear();
  }
  bool contained(const std::filesystem::path & path) const
  {
    auto r = root_.begin(), p = path.begin();
    for (; r != root_.end(); ++r, ++p) {if (p == path.end() || *r != *p) {return false;}}
    return true;
  }
  void request()
  {
    if (!directory_.empty() && monotonic_ns() >= deadline_) {close("deadline");}
    if (!directory_.empty() && std::filesystem::space(directory_).available < minimum_free_) {
      close("disk_low");
    }
    if (!std::filesystem::is_regular_file(request_)) {
      if (!directory_.empty()) {close("request_removed");} return;
    }
    if (std::filesystem::file_size(request_) > 4096) {return;}
    std::ifstream input(request_); Json j; input >> j;
    if (j.at("schema").get<int>() != 1) {return;}
    const auto session = j.at("session").get<std::string>();
    if (session.empty() || session.size() > 64 || session.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos) {return;}
    if (session == seen_session_) {return;}
    const auto output = std::filesystem::canonical(j.at("output_dir").get<std::string>());
    const auto deadline = j.at("deadline_monotonic_ns").get<int64_t>();
    const auto maximum = j.at("max_bytes").get<uint64_t>();
    const auto minimum_free = j.value("min_free_bytes", uint64_t(64 * 1024 * 1024));
    if (!contained(output) || !std::filesystem::is_directory(output) || deadline <= monotonic_ns() ||
      deadline - monotonic_ns() > 3600000000000LL || maximum < 1048576 || maximum > 2147483648ULL ||
      minimum_free > 8589934592ULL) {return;}
    if (!directory_.empty()) {close("superseded");}
    seen_session_ = session;
    const auto target = output / ("mppi_" + std::to_string(getpid()) + "_" + std::to_string(monotonic_ns()));
    if (!std::filesystem::create_directory(target)) {return;}
    binary_.open(target / "payload.bin", std::ios::binary | std::ios::out);
    index_.open(target / "frames.jsonl", std::ios::out);
    if (!binary_ || !index_) {++errors_; binary_.close(); index_.close(); return;}
    session_ = session; directory_ = target; deadline_ = deadline; maximum_ = maximum;
    minimum_free_ = minimum_free; dropped_base_ = dropped();
    bytes_ = written_ = stale_ = errors_ = file_bytes_ = 0;
    std::ofstream manifest(target / "schema.json");
    const uint16_t endian = 1;
    const auto schema = Json{{"schema", 1}, {"kind", "mppi_internal_snapshot"},
      {"session", session}, {"pid", getpid()},
      {"byte_order", *reinterpret_cast<const uint8_t *>(&endian) ? "little" : "big"},
      {"float_format", "IEEE754 binary64"}, {"payload_blocks", "offset,length,dtype,shape"},
      {"sequence_columns", {"vx", "vy", "wz"}},
      {"path_columns", {"x", "y", "z", "qx", "qy", "qz", "qw"}},
      {"sequence_semantics", "post-filter,pre-shift; selected offset recorded; no forward rollout on control thread"},
      {"odom_source_stamp", nullptr}, {"odom_covariance", nullptr},
      {"limits", {{"slots", kSlots}, {"map_cells", kMapCells}, {"path_poses", kPathPoses},
        {"sequence_steps", kSteps}, {"history", kHistory}, {"footprint_points", kFootprint}}}}.dump(2);
    manifest << schema; file_bytes_ = schema.size();
    if (!manifest) {close("schema_write_error"); return;}
    active_.store(++generation_, std::memory_order_release);
  }
  Json block(const void * data, std::size_t bytes, const char * dtype, Json shape)
  {
    Json j{{"offset", bytes_}, {"length", bytes}, {"dtype", dtype}, {"shape", std::move(shape)}};
    (void)data;
    bytes_ += bytes; return j;
  }
  void write(const Frame & f)
  {
    if (directory_.empty() || f.generation != generation_) {++stale_; return;}
    const auto previous_bytes = bytes_;
    Json j{{"schema", 1}, {"compute_seq", f.compute_seq}, {"session", session_},
      {"start_monotonic_ns", f.start_monotonic_ns}, {"end_monotonic_ns", f.end_monotonic_ns},
      {"start_wall_ns", f.start_wall_ns}, {"dropped_before", f.dropped_before},
      {"session_dropped_before", f.dropped_before >= dropped_base_ ? f.dropped_before - dropped_base_ : 0},
      {"map_copy_monotonic_ns", f.map_copy_monotonic_ns},
      {"model_input_monotonic_ns", f.model_input_monotonic_ns},
      {"incomplete", f.incomplete}, {"pose", f.pose}, {"odom", f.odom},
      {"pose_frame", f.pose_frame}, {"pose_stamp_ns", f.pose_stamp_ns},
      {"odom_source_stamp", nullptr}, {"odom_covariance", nullptr},
      {"map_frame", f.map_frame}, {"base_frame", f.base_frame},
      {"resolution", f.resolution}, {"origin", {f.origin_x, f.origin_y}},
      {"map_dimensions", {f.map_width, f.map_height}},
      {"path_frame", f.path_frame}, {"path_stamp_ns", f.path_stamp_ns},
      {"path_original_count", f.path_original_count},
      {"sequence_original_count", f.sequence_original_count}, {"command_offset", f.command_offset},
      {"sequence_valid", f.sequence_valid}, {"command_returned", f.command_returned},
      {"command", f.command}, {"stage", f.stage}, {"exception_type", f.exception_type},
      {"exception", f.exception}, {"fail_flag", f.fail_flag}, {"optimize_passes", f.optimize_passes},
      {"passes_entered_failed", f.passes_entered_failed},
      {"model_enabled", f.model_enabled}, {"dynamics", f.dynamics},
      {"smoother_limits", f.smoother_limits}, {"model_dt", f.model_dt},
      {"constraints", f.constraints}, {"min_turning_radius", f.min_turning_radius},
      {"smoother_valid", f.smoother_valid}, {"smoother_age_sec", f.smoother_age_sec},
      {"smoother_initial", {f.smoother_linear, f.smoother_angular}}, {"issued_age_sec", f.issued_age_sec}};
    j["costmap"] = block(f.map.data(), f.map_count, "u1", {f.map_height, f.map_width});
    j["path"] = block(f.path.data(), f.path_count * 7 * sizeof(double), "f8", {f.path_count, 7});
    j["path_stamps"] = block(f.path_stamps.data(), f.path_count * sizeof(int64_t), "i8", {f.path_count});
    j["sequence"] = block(f.sequence.data(), f.sequence_count * 3 * sizeof(double), "f8", {f.sequence_count, 3});
    j["footprint"] = Json::array(); j["issued"] = Json::array();
    for (unsigned i = 0; i < f.footprint_count; ++i) {j["footprint"].push_back(f.footprint[i]);}
    for (unsigned i = 0; i < f.history_count; ++i) {j["issued"].push_back(f.issued[i]);}
    j["path_frames"] = Json::array();
    bool common = true;
    for (unsigned i = 0; i < f.path_count; ++i) {
      if (std::strcmp(f.path_frames[i].data(), f.path_frame) != 0) {common = false; break;}
    }
    j["path_frames_all_equal_header"] = common;
    if (!common) {for (unsigned i = 0; i < f.path_count; ++i) {j["path_frames"].push_back(f.path_frames[i].data());}}
    const auto line = j.dump();
    const auto frame_bytes = bytes_ - previous_bytes + line.size() + 1;
    if (file_bytes_ + frame_bytes + 8192 > maximum_) {
      bytes_ = previous_bytes; close("disk_budget"); return;
    }
    binary_.write(reinterpret_cast<const char *>(f.map.data()), f.map_count);
    binary_.write(reinterpret_cast<const char *>(f.path.data()), f.path_count * 7 * sizeof(double));
    binary_.write(reinterpret_cast<const char *>(f.path_stamps.data()), f.path_count * sizeof(int64_t));
    binary_.write(reinterpret_cast<const char *>(f.sequence.data()), f.sequence_count * 3 * sizeof(double));
    binary_.flush();  // Writer only; index follows successfully written payload.
    if (!binary_) {++errors_; close("payload_write_error"); return;}
    index_ << line << '\n'; index_.flush();
    if (!index_) {++errors_; close("index_write_error"); return;}
    ++written_; file_bytes_ += frame_bytes;
  }
  void drain()
  {
    auto tail = tail_.load(std::memory_order_relaxed);
    const auto head = head_.load(std::memory_order_acquire);
    while (tail != head) {
      write(slots_[tail % kSlots]);
      tail_.store(++tail, std::memory_order_release);
    }
  }
  void run() noexcept
  {
    auto next = Clock::now();
    try {
      while (!stop_.load(std::memory_order_acquire)) {
        drain();
        if (Clock::now() >= next) {
          try {request();} catch (...) {++errors_;}
          next = Clock::now() + std::chrono::milliseconds(500);
        }
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(enabled() ? 50 : 500),
          [this] {return stop_.load();});
      }
      drain(); close("owner_stopped");
    } catch (...) {
      ++errors_; active_.store(0, std::memory_order_release);
      try {close("writer_exception");} catch (...) {}
    }
  }
  std::string request_; std::filesystem::path root_, directory_;
  std::unique_ptr<Frame[]> slots_;
  std::atomic<uint64_t> active_{0}, head_{0}, tail_{0}, dropped_{0};
  std::atomic_flag producer_busy_ = ATOMIC_FLAG_INIT;
  std::atomic<bool> stop_{false};
  std::atomic<bool> in_flight_{false};
  std::mutex wake_mutex_; std::condition_variable wake_;
  std::string session_, seen_session_; std::ofstream binary_, index_;
  uint64_t generation_{0}, maximum_{0}, bytes_{0}, written_{0}, stale_{0}, errors_{0}, file_bytes_{0};
  uint64_t dropped_base_{0}, minimum_free_{64 * 1024 * 1024};
  int64_t deadline_{0};
  std::thread worker_;
};
}  // namespace robot_nav_config::navlite_snapshot
