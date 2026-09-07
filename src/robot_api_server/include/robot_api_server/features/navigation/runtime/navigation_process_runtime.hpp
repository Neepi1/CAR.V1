#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <sys/types.h>

namespace robot_api_server::features::navigation
{

struct NavigationProcessRuntimeConfig
{
  std::filesystem::path resume_command;
  std::filesystem::path resume_log_file;
  std::filesystem::path stop_command;
  std::filesystem::path stop_log_file;
};

struct NavigationProcessLaunchSpec
{
  std::string building_id;
  std::string floor_id;
  std::vector<std::pair<std::string, std::string>> environment;
};

// Owns only the external navigation runtime process lifecycle. It deliberately
// does not stop a healthy resident runtime during API process destruction.
class NavigationProcessRuntime
{
public:
  explicit NavigationProcessRuntime(NavigationProcessRuntimeConfig config);
  ~NavigationProcessRuntime();

  NavigationProcessRuntime(const NavigationProcessRuntime &) = delete;
  NavigationProcessRuntime & operator=(const NavigationProcessRuntime &) = delete;

  bool resume_command_available() const;
  bool stop_command_available() const;
  std::string resume_command() const;
  std::string resume_log_file() const;
  std::string stop_command() const;
  std::string stop_log_file() const;

  bool running();
  pid_t pid() const;
  void terminate_managed_process();
  bool launch_replacing(
    const NavigationProcessLaunchSpec & spec,
    pid_t & launched_pid,
    std::string & detail);
  bool stop_stack(std::string & detail);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
