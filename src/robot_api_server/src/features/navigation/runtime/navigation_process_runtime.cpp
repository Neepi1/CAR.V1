#include "robot_api_server/features/navigation/runtime/navigation_process_runtime.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <sstream>
#include <thread>
#include <utility>

#include <sys/wait.h>
#include <unistd.h>

#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"

namespace robot_api_server::features::navigation
{

using namespace std::chrono_literals;

class NavigationProcessRuntime::Impl
{
public:
  explicit Impl(NavigationProcessRuntimeConfig config)
  : config_(std::move(config))
  {
  }

  bool resume_command_available() const
  {
    return !config_.resume_command.empty() &&
           std::filesystem::exists(config_.resume_command);
  }

  bool stop_command_available() const
  {
    return !config_.stop_command.empty() &&
           std::filesystem::exists(config_.stop_command);
  }

  std::string resume_command() const {return config_.resume_command.string();}
  std::string resume_log_file() const {return config_.resume_log_file.string();}
  std::string stop_command() const {return config_.stop_command.string();}
  std::string stop_log_file() const {return config_.stop_log_file.string();}

  bool running()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_locked();
  }

  pid_t pid() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pid_;
  }

  void terminate_managed_process()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    terminate_locked();
  }

  bool launch_replacing(
    const NavigationProcessLaunchSpec & spec,
    pid_t & launched_pid,
    std::string & detail)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!resume_command_available()) {
      detail = "navigation resume command is not available: " + resume_command();
      return false;
    }
    terminate_locked();

    const pid_t child = ::fork();
    if (child < 0) {
      detail = "failed to fork navigation resume process";
      return false;
    }
    if (child == 0) {
      prepare_child_process(config_.resume_log_file.string());
      for (const auto & entry : spec.environment) {
        ::setenv(entry.first.c_str(), entry.second.c_str(), 1);
      }
      ::execl(
        "/bin/bash",
        "bash",
        config_.resume_command.c_str(),
        spec.building_id.c_str(),
        spec.floor_id.c_str(),
        static_cast<char *>(nullptr));
      ::_exit(127);
    }

    pid_ = child;
    launched_pid = child;
    detail = "navigation runtime process launched";
    return true;
  }

  bool stop_stack(std::string & detail)
  {
    if (!stop_command_available()) {
      detail = "navigation stop command is not available: " + stop_command();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminate_locked();
    }

    const pid_t child = ::fork();
    if (child < 0) {
      detail = "failed to fork navigation stop process";
      return false;
    }
    if (child == 0) {
      prepare_child_process(config_.stop_log_file.string());
      ::execl(
        "/bin/bash", "bash", config_.stop_command.c_str(),
        static_cast<char *>(nullptr));
      ::_exit(127);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    bool exited = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const pid_t wait_result = ::waitpid(child, &status, WNOHANG);
      if (wait_result == child) {
        exited = true;
        break;
      }
      if (wait_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        detail = "failed waiting for navigation stop process";
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!exited) {
      ::kill(-child, SIGKILL);
      ::waitpid(child, &status, 0);
      detail = "timed out waiting for navigation stop command; log_file=" + stop_log_file();
      return false;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      std::ostringstream out;
      out << "navigation stop command failed";
      if (WIFEXITED(status)) {
        out << " with exit code " << WEXITSTATUS(status);
      }
      out << "; log_file=" << stop_log_file();
      detail = out.str();
      return false;
    }
    detail = "navigation runtime stack stopped; log_file=" + stop_log_file();
    return true;
  }

private:
  bool running_locked()
  {
    if (pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(pid_, &status, WNOHANG);
    if (wait_result == pid_) {
      pid_ = -1;
      return false;
    }
    if (::kill(pid_, 0) == 0) {
      return true;
    }
    if (errno == ESRCH) {
      pid_ = -1;
      return false;
    }
    return true;
  }

  void terminate_locked()
  {
    if (!running_locked()) {
      pid_ = -1;
      return;
    }
    const pid_t pgid = ::getpgid(pid_);
    if (pgid > 0) {
      signal_process_group(pgid, SIGINT);
      std::this_thread::sleep_for(800ms);
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGTERM);
        std::this_thread::sleep_for(800ms);
      }
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGKILL);
      }
    } else {
      ::kill(pid_, SIGINT);
    }
    int status = 0;
    while (::waitpid(pid_, &status, WNOHANG) == pid_) {
    }
    pid_ = -1;
  }

  NavigationProcessRuntimeConfig config_;
  mutable std::mutex mutex_;
  pid_t pid_{-1};
};

NavigationProcessRuntime::NavigationProcessRuntime(NavigationProcessRuntimeConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

NavigationProcessRuntime::~NavigationProcessRuntime() = default;

bool NavigationProcessRuntime::resume_command_available() const
{
  return impl_->resume_command_available();
}

bool NavigationProcessRuntime::stop_command_available() const
{
  return impl_->stop_command_available();
}

std::string NavigationProcessRuntime::resume_command() const {return impl_->resume_command();}
std::string NavigationProcessRuntime::resume_log_file() const {return impl_->resume_log_file();}
std::string NavigationProcessRuntime::stop_command() const {return impl_->stop_command();}
std::string NavigationProcessRuntime::stop_log_file() const {return impl_->stop_log_file();}
bool NavigationProcessRuntime::running() {return impl_->running();}
pid_t NavigationProcessRuntime::pid() const {return impl_->pid();}
void NavigationProcessRuntime::terminate_managed_process() {impl_->terminate_managed_process();}

bool NavigationProcessRuntime::launch_replacing(
  const NavigationProcessLaunchSpec & spec,
  pid_t & launched_pid,
  std::string & detail)
{
  return impl_->launch_replacing(spec, launched_pid, detail);
}

bool NavigationProcessRuntime::stop_stack(std::string & detail)
{
  return impl_->stop_stack(detail);
}

}  // namespace robot_api_server::features::navigation
