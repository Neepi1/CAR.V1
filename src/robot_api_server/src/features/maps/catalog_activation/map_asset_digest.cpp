#include "robot_api_server/features/maps/catalog_activation/map_asset_digest.hpp"

#include <utility>

#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace robot_api_server
{

std::string sha256_hex(const std::string_view content)
{
  return robot_map_asset_identity::sha256_hex(content);
}

bool is_canonical_sha256_digest(const std::string_view digest) noexcept
{
  return robot_map_asset_identity::is_canonical_sha256_digest(digest);
}

std::string canonical_map_asset_digest(
  const std::vector<MapAssetDigestEntry> & entries)
{
  std::vector<robot_map_asset_identity::DigestEntry> shared_entries;
  shared_entries.reserve(entries.size());
  for (const auto & entry : entries) {
    shared_entries.push_back(
      robot_map_asset_identity::DigestEntry{
        entry.logical_name,
        entry.content,
      });
  }
  return robot_map_asset_identity::canonical_map_asset_digest(shared_entries);
}

}  // namespace robot_api_server
