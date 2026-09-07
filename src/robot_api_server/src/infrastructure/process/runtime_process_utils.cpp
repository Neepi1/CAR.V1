#include "robot_api_server/infrastructure/process/runtime_process_utils.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <system_error>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server
{

namespace fs = std::filesystem;

namespace
{

std::string read_proc_file(const pid_t pid, const char * filename)
{
  try {
    std::ifstream file(fs::path("/proc") / std::to_string(pid) / filename, std::ios::binary);
    if (!file) {
      return {};
    }
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  } catch (const std::ios_base::failure &) {
    // A process can disappear between opening and reading its procfs entry.
    return {};
  }
}

}  // namespace

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
  std::string cmdline = read_proc_file(pid, "cmdline");
  std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
  return trim(cmdline);
}

std::string read_proc_environ(const pid_t pid)
{
  std::string environ = read_proc_file(pid, "environ");
  std::replace(environ.begin(), environ.end(), '\0', '\n');
  return environ;
}

std::vector<pid_t> list_proc_pids()
{
  std::vector<pid_t> pids;
  std::error_code iterator_error;
  fs::directory_iterator iterator(
    fs::path("/proc"), fs::directory_options::skip_permission_denied, iterator_error);
  const fs::directory_iterator end;
  while (!iterator_error && iterator != end) {
    const fs::path path = iterator->path();
    std::error_code type_error;
    if (iterator->is_directory(type_error) && !type_error && is_pid_directory(path)) {
      const std::string name = path.filename().string();
      char * parse_end = nullptr;
      errno = 0;
      const long parsed = std::strtol(name.c_str(), &parse_end, 10);
      if (
        errno == 0 && parse_end != name.c_str() && *parse_end == '\0' && parsed > 0 &&
        parsed <= static_cast<long>(std::numeric_limits<pid_t>::max()))
      {
        pids.push_back(static_cast<pid_t>(parsed));
      }
    }
    iterator.increment(iterator_error);
  }
  return pids;
}

bool process_group_has_live_process(const pid_t pgid)
{
  if (pgid <= 0) {
    return false;
  }
  const pid_t self_pid = ::getpid();
  for (const pid_t pid : list_proc_pids()) {
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
