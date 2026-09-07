#include "robot_safety/persistent_sequence_allocator.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cerrno>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace robot_safety
{
namespace
{

std::uint64_t time_based_base(const std::uint64_t block_size)
{
  const auto milliseconds =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (milliseconds <= 0) {
    throw std::runtime_error("system clock cannot seed command sequence");
  }
  const auto now = static_cast<std::uint64_t>(milliseconds);
  if (now > std::numeric_limits<std::uint64_t>::max() / block_size) {
    throw std::runtime_error("system clock command sequence overflows");
  }
  return now * block_size;
}

std::uint64_t reserve_base(
  const std::filesystem::path & state_file,
  const std::uint64_t block_size)
{
  if (state_file.empty() || !state_file.is_absolute()) {
    throw std::invalid_argument(
            "persistent command sequence requires an absolute state file");
  }
  if (block_size < 2U) {
    throw std::invalid_argument("command sequence block is too small");
  }
  const auto time_base = time_based_base(block_size);
#ifdef _WIN32
  return time_base;
#else
  constexpr std::size_t kMaximumJournalBytes = 1U << 20U;
  const int descriptor = ::open(
    state_file.c_str(),
    O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW | O_APPEND,
    S_IRUSR | S_IWUSR);
  if (descriptor < 0) {
    throw std::runtime_error(
            "cannot open command sequence state: errno=" +
            std::to_string(errno));
  }
  const auto close_descriptor = [&descriptor]() {
      (void)::flock(descriptor, LOCK_UN);
      (void)::close(descriptor);
    };
  if (::flock(descriptor, LOCK_EX) != 0) {
    const auto code = errno;
    (void)::close(descriptor);
    throw std::runtime_error(
            "cannot lock command sequence state: errno=" +
            std::to_string(code));
  }
  try {
    struct stat metadata {};
    if (::fstat(descriptor, &metadata) != 0) {
      throw std::runtime_error(
              "cannot inspect command sequence state: errno=" +
              std::to_string(errno));
    }
    if (!S_ISREG(metadata.st_mode)) {
      throw std::runtime_error("command sequence state is not a regular file");
    }
    if (metadata.st_uid != ::geteuid()) {
      throw std::runtime_error(
              "command sequence state is not owned by the runtime user");
    }
    if (
      metadata.st_size < 0 ||
      static_cast<std::uint64_t>(metadata.st_size) >
      kMaximumJournalBytes)
    {
      throw std::runtime_error("command sequence state exceeds size limit");
    }

    std::string journal(static_cast<std::size_t>(metadata.st_size), '\0');
    std::size_t read_offset = 0U;
    while (read_offset < journal.size()) {
      const auto count = ::pread(
        descriptor,
        journal.data() + read_offset,
        journal.size() - read_offset,
        static_cast<off_t>(read_offset));
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        throw std::runtime_error(
                "cannot read command sequence state: errno=" +
                std::to_string(errno));
      }
      read_offset += static_cast<std::size_t>(count);
    }

    std::uint64_t previous_base = 0U;
    std::size_t line_start = 0U;
    while (line_start < journal.size()) {
      const auto newline = journal.find('\n', line_start);
      if (newline == std::string::npos) {
        // A process may have died during its append. Only newline-terminated
        // records are committed; the next writer marks this tail discarded.
        break;
      }
      auto line = journal.substr(line_start, newline - line_start);
      if (line.empty()) {
        throw std::runtime_error("command sequence state is malformed");
      }
      constexpr char kDiscardSuffix[] = "!discard";
      if (
        line.size() >= sizeof(kDiscardSuffix) - 1U &&
        line.compare(
          line.size() - (sizeof(kDiscardSuffix) - 1U),
          sizeof(kDiscardSuffix) - 1U,
          kDiscardSuffix) == 0)
      {
        line_start = newline + 1U;
        continue;
      }
      constexpr char kRecordPrefix[] = "R:";
      if (line.rfind(kRecordPrefix, 0U) == 0U) {
        line.erase(0U, sizeof(kRecordPrefix) - 1U);
      }
      std::uint64_t parsed = 0U;
      const auto conversion = std::from_chars(
        line.data(), line.data() + line.size(), parsed);
      if (
        conversion.ec != std::errc{} ||
        conversion.ptr != line.data() + line.size())
      {
        throw std::runtime_error("command sequence state is malformed");
      }
      previous_base = std::max(previous_base, parsed);
      line_start = newline + 1U;
    }

    if (
      previous_base >
      std::numeric_limits<std::uint64_t>::max() - block_size)
    {
      throw std::runtime_error("command sequence state is exhausted");
    }
    const auto reserved_base =
      std::max(time_base, previous_base + block_size);
    if (
      reserved_base >
      std::numeric_limits<std::uint64_t>::max() - (block_size - 1U))
    {
      throw std::runtime_error("command sequence reservation overflows");
    }

    std::string encoded;
    if (!journal.empty() && journal.back() != '\n') {
      encoded += "!discard\n";
    }
    encoded += "R:";
    encoded += std::to_string(reserved_base);
    encoded.push_back('\n');
    if (journal.size() + encoded.size() > kMaximumJournalBytes) {
      throw std::runtime_error("command sequence state is exhausted");
    }

    std::size_t written = 0U;
    while (written < encoded.size()) {
      const auto count = ::write(
        descriptor,
        encoded.data() + written,
        encoded.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        throw std::runtime_error(
                "cannot write command sequence state: errno=" +
                std::to_string(errno));
      }
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) {
      throw std::runtime_error(
              "cannot fsync command sequence state: errno=" +
              std::to_string(errno));
    }
    const auto parent = state_file.parent_path();
    const int parent_descriptor = ::open(
      parent.c_str(), O_RDONLY | O_CLOEXEC | O_DIRECTORY | O_NOFOLLOW);
    if (parent_descriptor < 0) {
      throw std::runtime_error(
              "cannot open command sequence parent: errno=" +
              std::to_string(errno));
    }
    const auto parent_sync = ::fsync(parent_descriptor);
    const auto parent_sync_error = errno;
    (void)::close(parent_descriptor);
    if (parent_sync != 0) {
      throw std::runtime_error(
              "cannot fsync command sequence parent: errno=" +
              std::to_string(parent_sync_error));
    }
    close_descriptor();
    return reserved_base;
  } catch (...) {
    close_descriptor();
    throw;
  }
#endif
}

}  // namespace

PersistentSequenceAllocator::PersistentSequenceAllocator(
  const std::filesystem::path & state_file,
  const std::uint64_t block_size)
{
  base_ = reserve_base(state_file, block_size);
  limit_ = base_ + block_size - 1U;
  current_.store(base_, std::memory_order_relaxed);
}

std::optional<std::uint64_t> PersistentSequenceAllocator::next() noexcept
{
  auto current = current_.load(std::memory_order_relaxed);
  while (current < limit_) {
    const auto next_value = current + 1U;
    if (
      current_.compare_exchange_weak(
        current, next_value,
        std::memory_order_relaxed, std::memory_order_relaxed))
    {
      return next_value;
    }
  }
  return std::nullopt;
}

void PersistentSequenceAllocator::synchronize(
  const std::uint64_t observed) noexcept
{
  auto current = current_.load(std::memory_order_relaxed);
  const auto bounded = std::min(observed, limit_);
  while (
    current < bounded &&
    !current_.compare_exchange_weak(
      current, bounded,
      std::memory_order_relaxed, std::memory_order_relaxed))
  {
  }
}

std::uint64_t PersistentSequenceAllocator::base() const noexcept
{
  return base_;
}

std::uint64_t PersistentSequenceAllocator::limit() const noexcept
{
  return limit_;
}

}  // namespace robot_safety
