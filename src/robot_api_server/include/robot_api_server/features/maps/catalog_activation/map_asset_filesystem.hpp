#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <sys/types.h>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server::features::maps
{

bool path_lexically_within(
  const std::filesystem::path & child,
  const std::filesystem::path & parent);

bool same_normalized_path(
  const std::filesystem::path & left,
  const std::filesystem::path & right);

bool same_exact_map_asset_source(
  const MapManifest & left,
  const MapManifest & right);

class ExactMapAssetSourceDrift : public std::runtime_error
{
public:
  explicit ExactMapAssetSourceDrift(const std::string & message);
};

bool safe_bundle_directory(
  const std::filesystem::path & directory,
  const std::filesystem::path & managed_root);

void durable_sync_directory(const std::filesystem::path & directory);

void durable_sync_regular_file(const std::filesystem::path & path);

std::string read_regular_text_file_checked(
  const std::filesystem::path & path,
  std::size_t maximum_size = 16U * 1024U * 1024U);

bool regular_file_contents_equal_checked(
  const std::filesystem::path & left_path,
  const std::filesystem::path & right_path,
  std::uintmax_t maximum_size,
  std::string & error);

void durable_write_text_file_atomic(
  const std::filesystem::path & path,
  const std::string & content,
  mode_t mode = 0600);

void durable_remove_file(const std::filesystem::path & path);

std::filesystem::path map_delete_tombstone_path(
  const std::filesystem::path & map_root);

bool safe_bundle_regular_file(
  const std::filesystem::path & path,
  const std::filesystem::path & bundle_root);

}  // namespace robot_api_server::features::maps
