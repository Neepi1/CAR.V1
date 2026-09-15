#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>

#include <sys/types.h>

namespace robot_api_server::features::mapping::runtime
{

bool is_mapping_2d_launcher_command(const std::string & cmdline);
bool is_private_slam2d_fastlio_process(
  const std::string & cmdline,
  const std::string & environ);
bool is_mapping_2d_residual_process_command(
  const std::string & cmdline,
  const std::string & environ);
bool is_safe_mapping_lidar_rps_xps_path(const std::string & path);
bool is_safe_mapping_lidar_rps_xps_value(const std::string & value);

enum class MappingRuntimeLogLevel
{
  Info,
  Warning,
};

struct MappingProcessRuntimeConfig
{
  std::string start_command;
  std::string log_file;
  std::string lidar_rps_xps_state_dir;
  double graceful_stop_timeout_sec{30.0};
};

struct MappingProcessSnapshot
{
  pid_t pid{-1};
  bool active{false};
  bool running{false};
  std::chrono::steady_clock::time_point started_at{};
};

struct MappingProcessStartResult
{
  bool ok{false};
  bool already_running{false};
  pid_t pid{-1};
  std::string detail;
};

class MappingProcessRuntime
{
public:
  using RuntimeStateCallback = std::function<void(
      bool active,
      const std::string & state,
      const std::string & detail,
      bool healthy)>;
  using LogCallback = std::function<void(
      MappingRuntimeLogLevel level,
      const std::string & message)>;

  MappingProcessRuntime(
    MappingProcessRuntimeConfig config,
    RuntimeStateCallback runtime_state_callback = {},
    LogCallback log_callback = {});

  bool start_command_available() const;
  const std::string & start_command() const;
  MappingProcessSnapshot tracked_snapshot();
  MappingProcessSnapshot recover_snapshot();
  MappingProcessStartResult start(const std::function<void()> & before_launch = {});
  std::size_t stop();

private:
  bool tracked_process_running_locked();
  bool recover_process_locked();
  MappingProcessSnapshot snapshot_locked(bool running) const;
  std::size_t terminate_process_groups_locked(pid_t tracked_pid);
  std::size_t terminate_residual_processes() const;
  std::size_t restore_lidar_rps_xps_state() const;
  void update_runtime_state(
    bool active,
    const std::string & state,
    const std::string & detail,
    bool healthy = true) const;
  void log(MappingRuntimeLogLevel level, const std::string & message) const;

  MappingProcessRuntimeConfig config_;
  RuntimeStateCallback runtime_state_callback_;
  LogCallback log_callback_;
  // Serialize ownership changes, not status reads, across child-exit waits.
  std::mutex operation_mutex_;
  mutable std::mutex mutex_;
  bool stopping_{false};
  pid_t pid_{-1};
  bool active_{false};
  std::chrono::steady_clock::time_point started_at_{};
};

}  // namespace robot_api_server::features::mapping::runtime
