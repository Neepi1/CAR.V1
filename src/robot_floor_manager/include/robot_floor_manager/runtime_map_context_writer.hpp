#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
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
  // Only the cold-start ownership handoff file uses these fields. It is not
  // another motion lock and must never be published as a ready map context.
  bool startup_handoff{false};
  std::string request_nonce;
  std::uint64_t explicit_sequence_baseline{0U};
  bool speed_filter_enabled{false};
  std::string asset_root;
  std::string nav_map_yaml;
  std::string localizer_map_png;
  std::string localizer_params_yaml;
  std::string keepout_mask_yaml;
  std::string speed_mask_yaml;
};

struct FloorStartupHandoffAck
{
  std::string state;
  std::string failure;
  std::string detail;
  std::uint64_t explicit_relocalization_sequence{0U};
  std::uint64_t localizer_generation{0U};
  // Present only on the startup owner's exact post-cleanup terminal ACK.
  // Missing fields in legacy ACKs are unknown, never proof of quiescence.
  std::optional<bool> cleanup_completed;
  std::optional<bool> effects_settled;
  std::optional<bool> owner_available;
};

// A prior startup, another map, duplicate keys, or a reused transaction ID
// cannot acknowledge this request. Malformed/partial files simply return none.
std::optional<FloorStartupHandoffAck> read_floor_startup_handoff_ack(
  const std::filesystem::path & path, const RuntimeMapContextRecord & expected);

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
