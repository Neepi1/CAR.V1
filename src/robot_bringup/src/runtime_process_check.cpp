#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

// stdout is a single, non-executable record: STATUS SUPERVISOR_COUNT NODE_COUNT.
// 0: unique, 50: observed missing/duplicate; 40-42: observer failure, never recovery.
struct ObservationError : std::runtime_error {
  int code;
  ObservationError(int value, const std::string &message)
  : std::runtime_error(message), code(value) {}
};

struct Fd {
  int value;
  explicit Fd(int fd) : value(fd) {}
  ~Fd() { if (value >= 0) ::close(value); }
  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;
};

[[noreturn]] void read_error(const std::string &operation, int error = errno, int code = 41) {
  throw ObservationError(code, operation + ": errno=" + std::to_string(error) +
    " (" + std::strerror(error) + ")");
}

bool vanished(int error) { return error == ENOENT || error == ESRCH; }

std::string read_file(int directory, const char *name, std::size_t limit) {
  const Fd file(::openat(directory, name, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
  if (file.value < 0) read_error(std::string("open ") + name);
  struct stat metadata {};
  if (::fstat(file.value, &metadata) < 0) read_error(std::string("stat ") + name);
  if (!S_ISREG(metadata.st_mode)) throw ObservationError(41, std::string(name) + " is not a regular proc file");
  std::string text;
  char buffer[4096];
  for (;;) {
    const ssize_t count = ::read(file.value, buffer, sizeof(buffer));
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) read_error(std::string("read ") + name);
    if (count == 0) return text;
    if (text.size() + static_cast<std::size_t>(count) > limit) {
      throw ObservationError(41, std::string(name) + " exceeds observer read bound");
    }
    text.append(buffer, static_cast<std::size_t>(count));
  }
}

bool decimal(const std::string &text) {
  return !text.empty() && text.find_first_not_of("0123456789") == std::string::npos;
}

std::uint64_t integer(const std::string &text) {
  if (!decimal(text)) throw ObservationError(41, "invalid proc integer");
  try { return std::stoull(text); }
  catch (const std::exception &) { throw ObservationError(41, "proc integer overflow"); }
}

struct Identity {
  std::string pid;
  std::uint64_t start_time;
  std::uint64_t flags;
  char state;
  bool dead() const { return state == 'Z' || state == 'X' || state == 'x'; }
  bool kernel_thread() const { return (flags & 0x00200000U) != 0; }
};

Identity identity(int directory, const std::string &pid) {
  const std::string stat = read_file(directory, "stat", 65536);
  // comm may contain spaces, newlines and ')'; fields resume after its last ')'.
  const auto begin = stat.find(" (");
  const auto end = stat.rfind(')');
  if (begin == std::string::npos || end == std::string::npos || end <= begin ||
    stat.substr(0, begin) != pid)
  {
    throw ObservationError(41, "invalid proc stat identity");
  }
  std::istringstream fields(stat.substr(end + 1));
  std::vector<std::string> values;
  std::string value;
  while (fields >> value && values.size() < 20) values.push_back(value);
  if (values.size() < 20 || values[0].size() != 1) {
    throw ObservationError(41, "truncated proc stat");
  }
  return {pid, integer(values[19]), integer(values[6]), values[0][0]};
}

bool same_process(const Identity &a, const Identity &b) {
  return a.pid == b.pid && a.start_time == b.start_time;
}

bool absent_from_root(int root, const std::string &pid) {
  struct stat metadata {};
  if (::fstatat(root, pid.c_str(), &metadata, AT_SYMLINK_NOFOLLOW) == 0) return false;
  const int error = errno;
  if (vanished(error)) return true;
  read_error("check process disappearance", error);
}

std::string executable(int directory, const std::string &pid) {
  char buffer[65536];
  const ssize_t count = ::readlinkat(directory, "exe", buffer, sizeof(buffer));
  if (count < 0) {
    const int error = errno;
    // exe may disappear before the PID directory/state reflects exit. This is
    // unavailable evidence, not proof that a live owner is absent.
    read_error("read process executable pid=" + pid, error, vanished(error) ? 42 : 41);
  }
  if (static_cast<std::size_t>(count) == sizeof(buffer)) throw ObservationError(41, "proc exe truncated");
  return std::string(buffer, static_cast<std::size_t>(count));
}

std::string canonical(const std::string &path) {
  char *resolved = ::realpath(path.c_str(), nullptr);
  if (!resolved) throw ObservationError(40, "expected executable cannot be resolved: " + path);
  std::string result(resolved);
  std::free(resolved);
  return result;
}

bool exact_argument(const std::string &cmdline, const std::string &expected) {
  if (!cmdline.empty() && cmdline.back() != '\0') throw ObservationError(41, "unterminated proc cmdline");
  std::size_t start = 0;
  while (start < cmdline.size()) {
    const auto end = cmdline.find('\0', start);
    if (cmdline.compare(start, end - start, expected) == 0) return true;
    start = end + 1;
  }
  return false;
}

struct Options {
  std::string proc_root = "/proc";
  std::string supervisor_exe;
  std::string supervisor_arg;
  std::string api_exe;
};

struct Candidate {
  Identity id;
  bool supervisor;
  bool node;
};

int check(const Options &options) {
  const auto supervisor_exe = canonical(options.supervisor_exe);
  const auto api_exe = canonical(options.api_exe);
  const Fd root(::open(options.proc_root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (root.value < 0) throw ObservationError(40, "cannot open proc root");
  const auto boot = read_file(root.value, "sys/kernel/random/boot_id", 4096);
  if (boot.empty()) throw ObservationError(41, "empty proc boot identity");
  const int scan_fd = ::dup(root.value);
  if (scan_fd < 0) read_error("duplicate proc descriptor");
  DIR *entries = ::fdopendir(scan_fd);
  if (!entries) {
    const int error = errno;
    ::close(scan_fd);
    read_error("enumerate proc", error);
  }
  struct CloseDirectory { DIR *value; ~CloseDirectory() { ::closedir(value); } } guard{entries};
  std::vector<Candidate> candidates;
  for (;;) {
    errno = 0;
    const auto *entry = ::readdir(entries);
    if (!entry) {
      if (errno != 0) read_error("enumerate proc");
      break;
    }
    const std::string pid(entry->d_name);
    if (!decimal(pid)) continue;
    // Pin the proc directory so PID reuse cannot mix stat/exe/cmdline reads.
    const Fd process(::openat(root.value, pid.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
    if (process.value < 0) {
      if (vanished(errno)) continue;
      read_error("open process directory");
    }
    bool potential_owner = false;
    try {
      const auto before = identity(process.value, pid);
      if (before.dead() || before.kernel_thread()) continue;
      const auto exe = executable(process.value, pid);
      potential_owner = exe == supervisor_exe || exe == api_exe;
      if (!potential_owner) continue;
      const bool supervisor = exe == supervisor_exe && exact_argument(
        read_file(process.value, "cmdline", 8 * 1024 * 1024), options.supervisor_arg);
      const bool node = exe == api_exe;
      const auto after = identity(process.value, pid);
      if (!same_process(before, after) || after.dead() || executable(process.value, pid) != exe) {
        throw ObservationError(42, "potential owner changed while sampling");
      }
      if (supervisor || node) candidates.push_back({before, supervisor, node});
    } catch (const ObservationError &error) {
      // Short-lived unrelated processes are normal. An ambiguous owner is not
      // evidence of zero owners and must not trigger complete-chain recovery.
      if (absent_from_root(root.value, pid)) {
        if (!potential_owner) continue;
        throw ObservationError(42, "potential owner exited while sampling pid=" + pid + ": " + error.what());
      }
      try {
        if (identity(process.value, pid).dead()) {
          if (!potential_owner) continue;
          throw ObservationError(42, "potential owner exited while sampling pid=" + pid + ": " + error.what());
        }
      } catch (const ObservationError &last) {
        if (last.code == 42) throw;
      }
      throw error;
    }
  }
  std::size_t supervisors = 0;
  std::size_t nodes = 0;
  // Revalidate only matches, not another full /proc traversal. Do not count a
  // process that exited or exec'ed between the first PID and the last one.
  for (const auto &candidate : candidates) {
    const Fd process(::openat(root.value, candidate.id.pid.c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
    if (process.value < 0) {
      if (vanished(errno)) throw ObservationError(42, "owner exited before result");
      read_error("reopen owner directory");
    }
    const auto now = identity(process.value, candidate.id.pid);
    const auto exe = executable(process.value, candidate.id.pid);
    if (!same_process(candidate.id, now) || now.dead() ||
      (candidate.node && exe != api_exe) ||
      (candidate.supervisor && (exe != supervisor_exe || !exact_argument(
      read_file(process.value, "cmdline", 8 * 1024 * 1024), options.supervisor_arg))))
    {
      throw ObservationError(42, "owner identity changed before result");
    }
    const auto after = identity(process.value, candidate.id.pid);
    if (!same_process(now, after) || after.dead()) {
      throw ObservationError(42, "owner exited or changed during result validation");
    }
    supervisors += candidate.supervisor;
    nodes += candidate.node;
  }
  if (read_file(root.value, "sys/kernel/random/boot_id", 4096) != boot) {
    throw ObservationError(42, "boot identity changed while sampling");
  }
  const bool unique = supervisors == 1 && nodes == 1;
  std::cout << (unique ? "unique" : "ownership_fault") << ' ' << supervisors << ' ' << nodes << '\n';
  return unique ? 0 : 50;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    Options options;
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc) throw ObservationError(40, "missing option value");
      const std::string name(argv[i]);
      if (name == "--proc-root") options.proc_root = argv[i + 1];
      else if (name == "--supervisor-exe") options.supervisor_exe = argv[i + 1];
      else if (name == "--supervisor-arg") options.supervisor_arg = argv[i + 1];
      else if (name == "--api-exe") options.api_exe = argv[i + 1];
      else throw ObservationError(40, "unknown option: " + name);
    }
    if (options.supervisor_exe.empty() || options.supervisor_arg.empty() || options.api_exe.empty()) {
      throw ObservationError(40, "required: --supervisor-exe EXE --supervisor-arg ARG --api-exe EXE [--proc-root /proc]");
    }
    return check(options);
  } catch (const ObservationError &error) {
    const char *status = error.code == 40 ? "observer_unavailable" :
      (error.code == 42 ? "observer_transient" : "observer_error");
    std::cout << status << " - -\n";
    std::cerr << "[runtime-process-check] " << error.what() << '\n';
    return error.code;
  } catch (const std::exception &error) {
    std::cout << "observer_error - -\n";
    std::cerr << "[runtime-process-check] " << error.what() << '\n';
    return 41;
  }
}
