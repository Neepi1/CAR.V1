#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server
{

struct KeepoutAssetPaths
{
  std::filesystem::path semantic_json;
  std::filesystem::path mask_yaml;
  std::filesystem::path mask_pgm;
};

struct KeepoutCostmapSample
{
  double map_x{0.0};
  double map_y{0.0};
};

struct KeepoutMaskSummary
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::size_t active_cells{0U};
  std::string occupancy_digest;
  std::string revision;
  std::vector<KeepoutCostmapSample> blocked_costmap_samples;
  std::vector<KeepoutCostmapSample> cleared_costmap_samples;
};

enum class KeepoutRuntimeMutationState
{
  NOT_MUTATED,
  MAY_HAVE_CHANGED,
};

struct KeepoutRuntimeProof
{
  // Runtime adapters must set this before issuing the first state-changing request.
  // The transaction layer uses it to distinguish preflight rejection from a
  // partially-applied filter update that requires a runtime rollback.
  KeepoutRuntimeMutationState mutation_state{KeepoutRuntimeMutationState::NOT_MUTATED};
  bool mask_server_active{false};
  bool filter_plugin_enabled{false};
  bool load_map_succeeded{false};
  bool mask_topic_matches{false};
  bool global_costmap_cleared{false};
  bool global_costmap_updated{false};
  bool global_costmap_content_matches{false};
  std::size_t costmap_samples_checked{0U};
  std::size_t costmap_samples_blocked{0U};
  std::size_t costmap_samples_cleared{0U};
  std::string detail;
};

class KeepoutRuntimePort
{
public:
  virtual ~KeepoutRuntimePort() = default;

  virtual bool apply(
    const std::filesystem::path & mask_yaml,
    const KeepoutMaskSummary & expected,
    KeepoutRuntimeProof & proof,
    std::string & error) = 0;

  virtual bool restore(
    const std::filesystem::path & mask_yaml,
    const KeepoutMaskSummary & expected,
    KeepoutRuntimeProof & proof,
    std::string & error) = 0;
};

struct ReplaceKeepoutCommand
{
  std::string request_json;
  std::optional<std::string> expected_revision;
  MapManifest map;
  std::vector<KeepoutAssetPaths> projections;
  bool runtime_selected{false};
  bool runtime_apply_requested{false};
  bool allow_unmanaged_non_neutral_mask{false};
};

struct ReplaceKeepoutResult
{
  bool persisted{false};
  bool changed{false};
  bool runtime_selected{false};
  bool runtime_effective{false};
  bool effective_on_next_activation{false};
  std::string outcome;
  std::string revision;
  std::string previous_revision;
  KeepoutMaskSummary mask;
  KeepoutRuntimeProof runtime_proof;
};

class KeepoutLayerError : public std::runtime_error
{
public:
  KeepoutLayerError(std::string code, int http_status, const std::string & message);

  const std::string & code() const noexcept;
  int http_status() const noexcept;

private:
  std::string code_;
  int http_status_;
};

class KeepoutLayerModule
{
public:
  KeepoutMaskSummary inspect(const MapManifest & map) const;

  ReplaceKeepoutResult replace(
    const ReplaceKeepoutCommand & command,
    KeepoutRuntimePort * runtime) const;
};

}  // namespace robot_api_server
