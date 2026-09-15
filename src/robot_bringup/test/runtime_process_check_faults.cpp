// LD_PRELOAD fault injector used only by test_runtime_process_check.py on proc fixtures.
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <fstream>
#include <cstdarg>
#include <string>
#include <unistd.h>

extern "C" int open(const char *path, int flags, ...) {
  using Open = int (*)(const char *, int, ...);
  static auto actual = reinterpret_cast<Open>(dlsym(RTLD_NEXT, "open"));
  const char *target = std::getenv("NJRH_TEST_PROC_TARGET");
  const char *action = std::getenv("NJRH_TEST_PROC_FAULT");
  if (target && action && std::strcmp(action, "root_permission") == 0 &&
    std::strcmp(path, target) == 0)
  {
    errno = EACCES;
    return -1;
  }
  if ((flags & O_CREAT) || (flags & O_TMPFILE) == O_TMPFILE) {
    va_list args;
    va_start(args, flags);
    const mode_t mode = va_arg(args, mode_t);
    va_end(args);
    return actual(path, flags, mode);
  }
  return actual(path, flags);
}

extern "C" ssize_t readlinkat(int fd, const char *path, char *buffer, size_t size) noexcept {
  using Readlink = ssize_t (*)(int, const char *, char *, size_t);
  static auto actual = reinterpret_cast<Readlink>(dlsym(RTLD_NEXT, "readlinkat"));
  static bool fired = false;
  static int matching_reads = 0;
  const char *target = std::getenv("NJRH_TEST_PROC_TARGET");
  const char *action = std::getenv("NJRH_TEST_PROC_FAULT");
  char fd_path[128];
  char directory[65536];
  std::snprintf(fd_path, sizeof(fd_path), "/proc/self/fd/%d", fd);
  const ssize_t length = ::readlink(fd_path, directory, sizeof(directory) - 1);
  if (length < 0 || !target || !action || fired || std::strcmp(path, "exe") != 0) {
    return actual(fd, path, buffer, size);
  }
  directory[length] = '\0';
  if (std::strcmp(directory, target) != 0) return actual(fd, path, buffer, size);
  const char *on_read = std::getenv("NJRH_TEST_PROC_FAULT_ON_READ");
  if (++matching_reads < (on_read ? std::atoi(on_read) : 1)) return actual(fd, path, buffer, size);
  fired = true;
  if (std::strcmp(action, "exe_enoent") == 0 || std::strcmp(action, "exe_esrch") == 0) {
    errno = std::strcmp(action, "exe_enoent") == 0 ? ENOENT : ESRCH;
    return -1;
  }
  if (std::strcmp(action, "permission") == 0 || std::strcmp(action, "io") == 0) {
    errno = std::strcmp(action, "permission") == 0 ? EACCES : EIO;
    return -1;
  }
  const auto result = actual(fd, path, buffer, size);
  if (std::strcmp(action, "exit_after_exe") == 0 || std::strcmp(action, "exit_before_exe") == 0) {
    if (::rename(target, (std::string(target) + ".gone").c_str()) != 0) std::abort();
    if (std::strcmp(action, "exit_before_exe") == 0) { errno = ENOENT; return -1; }
  } else if (std::strcmp(action, "reuse") == 0) {
    const char *replacement = std::getenv("NJRH_TEST_REPLACEMENT_STAT");
    if (!replacement) std::abort();
    std::ofstream file(std::string(target) + "/stat");
    file << replacement;
    if (!file) std::abort();
  } else if (std::strcmp(action, "boot_change") == 0) {
    const std::string process(target);
    std::ofstream file(process.substr(0, process.rfind('/')) + "/sys/kernel/random/boot_id");
    file << "changed-boot\n";
    if (!file) std::abort();
  }
  return result;
}
