#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server
{

struct ElevatorFloorAssetSnapshot
{
  MapManifest manifest;
  std::optional<MapYamlInfo> map_info;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  bool required_assets_complete{false};
};

using ElevatorFloorAssetResolver = std::function<std::optional<ElevatorFloorAssetSnapshot>(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id)>;

enum class ElevatorConfigurationCommandType
{
  kSaveDraft,
  kPublish,
  kRollback,
};

struct ElevatorConfigurationCommand
{
  ElevatorConfigurationCommandType type{
    ElevatorConfigurationCommandType::kSaveDraft};
  std::string building_id;
  std::string document;
  std::string expected_draft_revision;
  std::string expected_release_id;
  std::string release_id;
  std::string actor_id;
};

struct ElevatorConfigurationQuery
{
  std::string building_id;
  std::string release_id;
  std::string floor_id;
  std::string map_id;
};

struct ElevatorConfigurationReply
{
  int status{500};
  std::string code{"INTERNAL_ERROR"};
  std::string body{"{}"};

  bool ok() const noexcept
  {
    return status >= 200 && status < 300;
  }
};

class ElevatorConfigurationModule
{
public:
  ElevatorConfigurationModule(
    std::filesystem::path maps_root,
    ElevatorFloorAssetResolver floor_asset_resolver);
  ~ElevatorConfigurationModule();

  ElevatorConfigurationModule(const ElevatorConfigurationModule &) = delete;
  ElevatorConfigurationModule & operator=(const ElevatorConfigurationModule &) = delete;
  ElevatorConfigurationModule(ElevatorConfigurationModule &&) noexcept;
  ElevatorConfigurationModule & operator=(ElevatorConfigurationModule &&) noexcept;

  ElevatorConfigurationReply execute(const ElevatorConfigurationCommand & command);
  ElevatorConfigurationReply query(const ElevatorConfigurationQuery & query) const;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};

bool is_elevator_internal_pose_type(const std::string & type) noexcept;
bool is_reserved_elevator_pose_id(const std::string & pose_id) noexcept;

}  // namespace robot_api_server
