#include "robot_api_server/features/mapping/runtime/mapping_process_runtime.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <sys/wait.h>
#include <unistd.h>

#include "robot_api_server/infrastructure/http/http_common.hpp"
#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"

namespace robot_api_server::features::mapping::runtime
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

bool is_mapping_2d_launcher_command(const std::string & cmdline)
{
  static constexpr std::array<const char *, 4> kPatterns{
    "run_projected_map.sh",
    "jt128_slam_toolbox_mapping.launch.py",
    "jt128_2d_mapping.launch.py",
    "run_jt128_2d_mapping.sh",
  };
  return std::any_of(kPatterns.begin(), kPatterns.end(), [&cmdline](const char * pattern) {
      return cmdline.find(pattern) != std::string::npos;
    });
}

bool is_private_slam2d_fastlio_process(
  const std::string & cmdline,
  const std::string & environ)
{
  static constexpr std::array<const char *, 3> kPatterns{
    "ros2 run fast_lio fastlio_mapping",
    "fast_lio/lib/fast_lio/fastlio_mapping",
    "fastlio_mapping --ros-args",
  };
  const bool is_fastlio = std::any_of(
    kPatterns.begin(), kPatterns.end(), [&cmdline](const char * pattern) {
      return cmdline.find(pattern) != std::string::npos;
    });
  return is_fastlio &&
         environ.find("NJRH_SLAM2D_PRIVATE_FASTLIO=1") != std::string::npos;
}

bool is_mapping_2d_residual_process_command(
  const std::string & cmdline,
  const std::string & environ)
{
  static constexpr std::array<const char *, 4> kPatterns{
    "slam_toolbox",
    "fastlio_mapping_odom_bridge.py",
    "/mapping/fastlio_odometry",
    "/tf_slam2d",
  };
  const bool is_fastlio_process =
    cmdline.find("ros2 run fast_lio fastlio_mapping") != std::string::npos ||
    cmdline.find("fast_lio/lib/fast_lio/fastlio_mapping") != std::string::npos ||
    cmdline.find("fastlio_mapping --ros-args") != std::string::npos;
  if (is_fastlio_process) {
    return is_private_slam2d_fastlio_process(cmdline, environ);
  }
  const bool marked_scan_pipeline =
    environ.find("NJRH_SLAM2D_MAPPING_PIPELINE=1") != std::string::npos &&
    (cmdline.find("nav_cloud_preprocessor") != std::string::npos ||
    cmdline.find("pointcloud_to_laserscan_node") != std::string::npos);
  return marked_scan_pipeline || std::any_of(
    kPatterns.begin(), kPatterns.end(), [&cmdline](const char * pattern) {
      return cmdline.find(pattern) != std::string::npos;
    });
}

bool is_safe_mapping_lidar_rps_xps_path(const std::string & path)
{
  const bool is_rps =
    path.size() >= 9U && path.compare(path.size() - 9U, 9U, "/rps_cpus") == 0;
  const bool is_xps =
    path.size() >= 9U && path.compare(path.size() - 9U, 9U, "/xps_cpus") == 0;
  return path.rfind("/sys/class/net/", 0U) == 0U &&
         path.find("/queues/") != std::string::npos &&
         path.find("..") == std::string::npos &&
         (is_rps || is_xps);
}

bool is_safe_mapping_lidar_rps_xps_value(const std::string & value)
{
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](const unsigned char ch) {
           return std::isxdigit(ch) != 0 || ch == ',';
         });
}

MappingProcessRuntime::MappingProcessRuntime(
  MappingProcessRuntimeConfig config,
  RuntimeStateCallback runtime_state_callback,
  LogCallback log_callback)
: config_(std::move(config)),
  runtime_state_callback_(std::move(runtime_state_callback)),
  log_callback_(std::move(log_callback))
{
}

bool MappingProcessRuntime::start_command_available() const
{
  return !config_.start_command.empty() && fs::exists(config_.start_command);
}

const std::string & MappingProcessRuntime::start_command() const
{
  return config_.start_command;
}

void MappingProcessRuntime::update_runtime_state(
  const bool active,
  const std::string & state,
  const std::string & detail,
  const bool healthy) const
{
  if (runtime_state_callback_) {
    runtime_state_callback_(active, state, detail, healthy);
  }
}

void MappingProcessRuntime::log(
  const MappingRuntimeLogLevel level,
  const std::string & message) const
{
  if (log_callback_) {
    log_callback_(level, message);
  }
}

bool MappingProcessRuntime::tracked_process_running_locked()
{
  if (pid_ <= 0) {
    return false;
  }
  int status = 0;
  const pid_t wait_result = ::waitpid(pid_, &status, WNOHANG);
  if (wait_result == pid_) {
    pid_ = -1;
    active_ = false;
    update_runtime_state(false, "stopped", "2D mapping process exited");
    return false;
  }
  if (::kill(pid_, 0) == 0) {
    return true;
  }
  if (errno == ESRCH) {
    pid_ = -1;
    active_ = false;
    update_runtime_state(false, "stopped", "2D mapping process is not alive");
    return false;
  }
  return true;
}

namespace
{

std::set<pid_t> discover_mapping_process_groups()
{
  std::set<pid_t> groups;
  const pid_t self_pid = ::getpid();
  for (const pid_t process_id : list_proc_pids()) {
    if (process_id <= 1 || process_id == self_pid) {
      continue;
    }
    if (!is_mapping_2d_launcher_command(read_proc_cmdline(process_id))) {
      continue;
    }
    const pid_t pgid = ::getpgid(process_id);
    groups.insert(pgid > 0 ? pgid : process_id);
  }
  return groups;
}

std::set<pid_t> discover_mapping_residual_processes()
{
  std::set<pid_t> processes;
  const pid_t self_pid = ::getpid();
  for (const pid_t process_id : list_proc_pids()) {
    if (process_id <= 1 || process_id == self_pid) {
      continue;
    }
    if (is_mapping_2d_residual_process_command(
        read_proc_cmdline(process_id), read_proc_environ(process_id)))
    {
      processes.insert(process_id);
    }
  }
  return processes;
}

std::string read_mapping_state_table(const fs::path & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open file");
  }
  const std::istreambuf_iterator<char> begin(file);
  const std::istreambuf_iterator<char> end;
  std::string contents(begin, end);
  if (file.bad()) {
    throw std::runtime_error("failed while reading file");
  }
  return contents;
}

}  // namespace

bool MappingProcessRuntime::recover_process_locked()
{
  if (tracked_process_running_locked()) {
    return true;
  }
  if (pid_ <= 0 && !discover_mapping_process_groups().empty()) {
    if (!active_ || started_at_ == std::chrono::steady_clock::time_point{}) {
      started_at_ = std::chrono::steady_clock::now();
    }
    active_ = true;
    return true;
  }
  return false;
}

MappingProcessSnapshot MappingProcessRuntime::snapshot_locked(const bool running) const
{
  return MappingProcessSnapshot{pid_, active_, running, started_at_};
}

MappingProcessSnapshot MappingProcessRuntime::tracked_snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_locked(tracked_process_running_locked());
}

MappingProcessSnapshot MappingProcessRuntime::recover_snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_locked(recover_process_locked());
}

MappingProcessStartResult MappingProcessRuntime::start(
  const std::function<void()> & before_launch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (tracked_process_running_locked()) {
    return {true, true, pid_, "2D mapping chain is already running"};
  }

  if (before_launch) {
    before_launch();
  }

  const pid_t child_pid = ::fork();
  if (child_pid < 0) {
    return {false, false, -1, "failed to fork 2D slam_toolbox mapping process"};
  }
  if (child_pid == 0) {
    prepare_child_process(config_.log_file);
    ::execl(
      "/bin/bash", "bash", config_.start_command.c_str(),
      static_cast<char *>(nullptr));
    ::_exit(127);
  }

  pid_ = child_pid;
  active_ = true;
  started_at_ = std::chrono::steady_clock::now();
  return {true, false, child_pid, "2D mapping process started"};
}

std::size_t MappingProcessRuntime::terminate_residual_processes() const
{
  std::set<pid_t> processes = discover_mapping_residual_processes();
  if (processes.empty()) {
    return 0U;
  }
  const std::size_t requested = processes.size();
  for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
    for (const auto process_id : processes) {
      ::kill(process_id, signal);
    }
    std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);
    for (auto it = processes.begin(); it != processes.end();) {
      if (!process_pid_is_live(*it)) {
        it = processes.erase(it);
      } else {
        ++it;
      }
    }
    if (processes.empty()) {
      break;
    }
  }
  return requested;
}

std::size_t MappingProcessRuntime::restore_lidar_rps_xps_state() const
{
  if (config_.lidar_rps_xps_state_dir.empty()) {
    return 0U;
  }
  const fs::path state_dir(config_.lidar_rps_xps_state_dir);
  const fs::path table_path = state_dir / "rps_xps.tsv";
  if (!fs::exists(table_path) || !fs::is_regular_file(table_path)) {
    return 0U;
  }

  std::size_t restored = 0U;
  std::size_t failures = 0U;
  std::string table_text;
  try {
    table_text = read_mapping_state_table(table_path);
  } catch (const std::exception & exception) {
    log(
      MappingRuntimeLogLevel::Warning,
      "failed to read mapping LiDAR RPS/XPS state table " + table_path.string() +
      ": " + exception.what());
    return 0U;
  }

  std::istringstream table(table_text);
  std::string line;
  while (std::getline(table, line)) {
    const auto tab = line.find('\t');
    if (tab == std::string::npos) {
      continue;
    }
    const std::string path = trim(line.substr(0, tab));
    const std::string value = trim(line.substr(tab + 1));
    if (!is_safe_mapping_lidar_rps_xps_path(path) ||
      !is_safe_mapping_lidar_rps_xps_value(value))
    {
      ++failures;
      log(MappingRuntimeLogLevel::Warning,
        "skipping unsafe mapping LiDAR RPS/XPS restore entry: " + path);
      continue;
    }

    std::ofstream file(path);
    if (!file) {
      ++failures;
      log(MappingRuntimeLogLevel::Warning,
        "failed to open mapping LiDAR RPS/XPS restore target: " + path);
      continue;
    }
    file << value << '\n';
    if (!file) {
      ++failures;
      log(MappingRuntimeLogLevel::Warning,
        "failed to write mapping LiDAR RPS/XPS restore target: " + path);
      continue;
    }
    ++restored;
  }

  if (failures == 0U) {
    std::error_code error;
    fs::remove_all(state_dir, error);
    if (error) {
      log(MappingRuntimeLogLevel::Warning,
        "failed to remove mapping LiDAR RPS/XPS state dir " + state_dir.string() +
        ": " + error.message());
    }
  }
  if (restored > 0U) {
    log(MappingRuntimeLogLevel::Info,
      "restored " + std::to_string(restored) + " mapping LiDAR RPS/XPS queue settings");
  }
  return restored;
}

std::size_t MappingProcessRuntime::terminate_process_groups_locked()
{
  std::set<pid_t> groups = discover_mapping_process_groups();
  if (pid_ > 0) {
    const pid_t pgid = ::getpgid(pid_);
    groups.insert(pgid > 0 ? pgid : pid_);
  }

  const std::size_t requested_groups = groups.size();
  if (!groups.empty()) {
    for (const auto pgid : groups) {
      signal_process_group(pgid, SIGINT);
    }

    const auto graceful_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(config_.graceful_stop_timeout_sec);
    while (!groups.empty() && std::chrono::steady_clock::now() < graceful_deadline) {
      for (auto it = groups.begin(); it != groups.end();) {
        if (!process_group_has_live_process(*it)) {
          it = groups.erase(it);
        } else {
          ++it;
        }
      }
      if (!groups.empty()) {
        std::this_thread::sleep_for(100ms);
      }
    }

    for (const int signal : {SIGTERM, SIGKILL}) {
      for (const auto pgid : groups) {
        signal_process_group(pgid, signal);
      }
      std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);
      for (auto it = groups.begin(); it != groups.end();) {
        if (!process_group_has_live_process(*it)) {
          it = groups.erase(it);
        } else {
          ++it;
        }
      }
      if (groups.empty()) {
        break;
      }
    }
  }

  if (pid_ > 0) {
    int status = 0;
    while (::waitpid(pid_, &status, WNOHANG) == pid_) {
    }
  }
  pid_ = -1;
  active_ = false;
  update_runtime_state(false, "stopped", "2D mapping runtime stopped");
  const std::size_t requested_residuals = terminate_residual_processes();
  restore_lidar_rps_xps_state();
  return requested_groups + requested_residuals;
}

std::size_t MappingProcessRuntime::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminate_process_groups_locked();
}

}  // namespace robot_api_server::features::mapping::runtime
