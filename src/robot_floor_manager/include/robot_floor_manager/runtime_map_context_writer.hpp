#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace robot_floor_manager
{

struct RuntimeMapContextRecord
{
  std::string state;
  bool confirmed{false};
  std::string message;
  std::string transaction_id;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::uint64_t localizer_generation{0U};
  std::uint64_t explicit_relocalization_sequence{0U};
  double updated_at_sec{0.0};
};

// Publishes the compatibility runtime-map context through a same-directory
// durable rename. A failed validation or pre-rename write leaves the previous
// context untouched.
class AtomicRuntimeMapContextWriter
{
public:
  bool write(
    const std::filesystem::path & path,
    const RuntimeMapContextRecord & record,
    std::string & error) const noexcept;
};

}  // namespace robot_floor_manager
