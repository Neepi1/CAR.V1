#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace robot_global_localization
{

enum class LocalizerParameterType
{
  kNotSet,
  kBool,
  kInteger,
  kDouble,
  kString,
  kByteArray,
  kBoolArray,
  kIntegerArray,
  kDoubleArray,
  kStringArray,
};

struct LocalizerParameter
{
  std::string name;
  LocalizerParameterType type{LocalizerParameterType::kNotSet};
  bool bool_value{false};
  std::int64_t integer_value{0};
  double double_value{0.0};
  std::string string_value;
  std::vector<std::uint8_t> byte_array_value;
  std::vector<bool> bool_array_value;
  std::vector<std::int64_t> integer_array_value;
  std::vector<double> double_array_value;
  std::vector<std::string> string_array_value;
};

struct FloorAssetIdentity
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct FloorAssetRequest
{
  std::string transaction_id;
  FloorAssetIdentity identity;
  std::filesystem::path nav_map_yaml;
  std::filesystem::path localizer_map_png;
  std::filesystem::path localizer_params_yaml;
};

struct ComponentDescription
{
  std::string package_name;
  std::string plugin_name;
  std::string node_name;
  std::string node_namespace;
  std::uint8_t log_level{0U};
  std::vector<std::string> remap_rules;
};

struct ComponentSnapshot
{
  std::uint64_t unique_id{0U};
  std::string full_node_name;
  std::vector<LocalizerParameter> parameters;
};

struct ComponentLoadRequest
{
  ComponentDescription description;
  std::vector<LocalizerParameter> parameters;
};

enum class ComponentPresence
{
  kUnknown,
  kAbsent,
  kPresent,
  kAmbiguous,
};

struct ComponentOperationResult
{
  bool success{false};
  std::string failure_code;
  std::string message;
  std::uint64_t unique_id{0U};
  ComponentPresence resulting_presence{ComponentPresence::kUnknown};
};

struct ComponentCaptureResult
{
  bool success{false};
  std::string failure_code;
  std::string message;
  ComponentSnapshot snapshot;
};

class ComponentManagerPort
{
public:
  virtual ~ComponentManagerPort() = default;

  virtual ComponentOperationResult preflight() = 0;
  virtual ComponentCaptureResult capture() = 0;
  virtual ComponentOperationResult unload(std::uint64_t unique_id) = 0;
  virtual ComponentOperationResult load(const ComponentLoadRequest & request) = 0;
};

struct LocalizerAssetState
{
  std::string transaction_id;
  FloorAssetIdentity requested_identity;
  FloorAssetIdentity active_identity;
  std::filesystem::path active_nav_map_yaml;
  std::filesystem::path active_localizer_map_png;
  std::filesystem::path active_localizer_params_yaml;
  std::uint64_t localizer_generation{0U};
  bool active_identity_valid{false};
  bool applying{false};
  bool localizer_ready{false};
  bool idempotent{false};
  bool reloaded{false};
  bool rollback_attempted{false};
  bool rollback_succeeded{false};
  std::string failure_code;
  std::string detail;
};

struct ApplyFloorAssetResult
{
  bool success{false};
  bool idempotent{false};
  LocalizerAssetState state;
};

struct IsaacAssetReloaderOptions
{
  std::filesystem::path allowed_asset_root;
  std::string expected_full_node_name{"/occupancy_grid_localizer"};
  ComponentDescription component;
  std::uintmax_t max_yaml_bytes{2U * 1024U * 1024U};
  std::uintmax_t max_png_bytes{512U * 1024U * 1024U};
};

class IsaacAssetReloader
{
public:
  explicit IsaacAssetReloader(IsaacAssetReloaderOptions options);

  ApplyFloorAssetResult apply(
    const FloorAssetRequest & request,
    ComponentManagerPort & component_manager);

  ApplyFloorAssetResult bootstrap_from_runtime_context(
    const std::filesystem::path & runtime_context_path,
    ComponentManagerPort & component_manager);

  const LocalizerAssetState & state() const noexcept;

private:
  IsaacAssetReloaderOptions options_;
  LocalizerAssetState state_;
};

}  // namespace robot_global_localization
