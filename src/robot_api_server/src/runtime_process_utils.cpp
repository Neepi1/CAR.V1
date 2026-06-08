#include "robot_api_server/runtime_process_utils.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <fstream>
#include <iterator>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

namespace fs = std::filesystem;

void set_close_on_exec(const int fd)
{
  if (fd < 0) {
    return;
  }
  const int flags = ::fcntl(fd, F_GETFD);
  if (flags < 0) {
    return;
  }
  ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

void close_inherited_fds()
{
  long max_fd = ::sysconf(_SC_OPEN_MAX);
  if (max_fd < 0) {
    max_fd = 4096;
  }
  max_fd = std::min<long>(max_fd, 65536);
  for (int fd = STDERR_FILENO + 1; fd < max_fd; ++fd) {
    ::close(fd);
  }
}

void prepare_child_process(const std::string & log_file)
{
  ::setsid();
  const int log_fd = ::open(log_file.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
  if (log_fd >= 0) {
    ::dup2(log_fd, STDOUT_FILENO);
    ::dup2(log_fd, STDERR_FILENO);
    ::close(log_fd);
  }
  close_inherited_fds();
}

bool is_pid_directory(const fs::path & path)
{
  const auto name = path.filename().string();
  return !name.empty() && std::all_of(name.begin(), name.end(), [](const unsigned char c) {
    return std::isdigit(c) != 0;
  });
}

std::string read_proc_cmdline(const pid_t pid)
{
  std::ifstream file(fs::path("/proc") / std::to_string(pid) / "cmdline", std::ios::binary);
  if (!file) {
    return {};
  }
  std::string cmdline((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
  return trim(cmdline);
}

std::string read_proc_environ(const pid_t pid)
{
  std::ifstream file(fs::path("/proc") / std::to_string(pid) / "environ", std::ios::binary);
  if (!file) {
    return {};
  }
  std::string environ((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::replace(environ.begin(), environ.end(), '\0', '\n');
  return environ;
}

bool process_group_has_live_process(const pid_t pgid)
{
  if (pgid <= 0 || !fs::exists("/proc")) {
    return false;
  }
  const pid_t self_pid = ::getpid();
  for (const auto & entry : fs::directory_iterator("/proc")) {
    if (!entry.is_directory() || !is_pid_directory(entry.path())) {
      continue;
    }
    const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
    if (pid <= 1 || pid == self_pid) {
      continue;
    }
    if (::getpgid(pid) == pgid) {
      return true;
    }
  }
  return false;
}

bool process_pid_is_live(const pid_t pid)
{
  if (pid <= 0) {
    return false;
  }
  if (::kill(pid, 0) == 0) {
    return true;
  }
  return errno != ESRCH;
}

bool signal_process_group(const pid_t pgid, const int signal)
{
  if (pgid <= 0) {
    return false;
  }
  if (::kill(-pgid, signal) == 0) {
    return true;
  }
  return errno == ESRCH;
}

}  // namespace robot_api_server
