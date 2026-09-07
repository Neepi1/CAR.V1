#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <optional>

namespace robot_safety
{

// Reserves a durable, non-overlapping sequence block per process generation.
// A replacement process therefore emits values greater than every value the
// old process could still have in transport.
class PersistentSequenceAllocator
{
public:
  explicit PersistentSequenceAllocator(
    const std::filesystem::path & state_file,
    std::uint64_t block_size = 1ULL << 20U);

  std::optional<std::uint64_t> next() noexcept;
  void synchronize(std::uint64_t observed) noexcept;

  std::uint64_t base() const noexcept;
  std::uint64_t limit() const noexcept;

private:
  std::uint64_t base_{0U};
  std::uint64_t limit_{0U};
  std::atomic<std::uint64_t> current_{0U};
};

}  // namespace robot_safety
