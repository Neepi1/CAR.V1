#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace robot_api_server
{

struct MapAssetDigestEntry
{
  std::string logical_name;
  std::string content;
};

std::string sha256_hex(std::string_view content);
bool is_canonical_sha256_digest(std::string_view digest) noexcept;
std::string canonical_map_asset_digest(
  const std::vector<MapAssetDigestEntry> & entries);

}  // namespace robot_api_server
