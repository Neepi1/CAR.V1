#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "robot_global_localization/isaac_asset_reloader.hpp"

namespace fs = std::filesystem;

namespace
{

using robot_global_localization::ApplyFloorAssetResult;
using robot_global_localization::ComponentCaptureResult;
using robot_global_localization::ComponentLoadRequest;
using robot_global_localization::ComponentManagerPort;
using robot_global_localization::ComponentOperationResult;
using robot_global_localization::ComponentPresence;
using robot_global_localization::ComponentSnapshot;
using robot_global_localization::FloorAssetRequest;
using robot_global_localization::IsaacAssetReloader;
using robot_global_localization::IsaacAssetReloaderOptions;
using robot_global_localization::LocalizerParameter;
using robot_global_localization::LocalizerParameterType;

LocalizerParameter string_parameter(std::string name, std::string value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kString;
  parameter.string_value = std::move(value);
  return parameter;
}

LocalizerParameter double_parameter(std::string name, const double value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kDouble;
  parameter.double_value = value;
  return parameter;
}

LocalizerParameter integer_parameter(std::string name, const std::int64_t value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kInteger;
  parameter.integer_value = value;
  return parameter;
}

LocalizerParameter double_array_parameter(
  std::string name,
  std::vector<double> value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kDoubleArray;
  parameter.double_array_value = std::move(value);
  return parameter;
}

const LocalizerParameter * find_parameter(
  const std::vector<LocalizerParameter> & parameters,
  const std::string & name)
{
  for (const auto & parameter : parameters) {
    if (parameter.name == name) {
      return &parameter;
    }
  }
  return nullptr;
}

class FakeComponentManager final : public ComponentManagerPort
{
public:
  ComponentOperationResult preflight_result{true, "", "ready", 0U};
  ComponentCaptureResult capture_result{
    true,
    "",
    "captured",
    ComponentSnapshot{
      41U,
      "/occupancy_grid_localizer",
      {
        string_parameter("map_yaml_path", "/old/localizer_params.yaml"),
        string_parameter("image", "old.png"),
        double_parameter("resolution", 0.1),
        double_array_parameter("origin", {0.0, 0.0, 0.0}),
        double_parameter("occupied_thresh", 0.7),
        integer_parameter("batch_size", 1024),
      },
    },
  };
  ComponentOperationResult unload_result{true, "", "unloaded", 0U};
  std::vector<ComponentOperationResult> load_results{
    ComponentOperationResult{true, "", "loaded", 42U},
  };
  std::size_t preflight_calls{0U};
  std::size_t capture_calls{0U};
  std::vector<std::uint64_t> unloaded_ids;
  std::vector<ComponentLoadRequest> load_requests;

  ComponentOperationResult preflight() override
  {
    ++preflight_calls;
    return preflight_result;
  }

  ComponentCaptureResult capture() override
  {
    ++capture_calls;
    return capture_result;
  }

  ComponentOperationResult unload(const std::uint64_t unique_id) override
  {
    unloaded_ids.push_back(unique_id);
    return unload_result;
  }

  ComponentOperationResult load(const ComponentLoadRequest & request) override
  {
    load_requests.push_back(request);
    if (load_results.empty()) {
      return {false, "UNEXPECTED_LOAD", "no fake load result", 0U};
    }
    auto result = load_results.front();
    load_results.erase(load_results.begin());
    return result;
  }
};

class IsaacAssetReloaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto unique = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    root_ = fs::temp_directory_path() / ("njrh_localizer_reload_" + unique);
    asset_root_ = root_ / "maps_release";
    map_root_ = asset_root_ / "B11" / "F2" / "maps" / "map_f2";
    fs::create_directories(map_root_ / "nav");
    fs::create_directories(map_root_ / "localizer");

    nav_yaml_ = map_root_ / "nav" / "nav_map.yaml";
    localizer_png_ = map_root_ / "localizer" / "localizer_map.png";
    localizer_yaml_ = map_root_ / "localizer" / "localizer_params.yaml";

    write_text(nav_yaml_, "image: nav_map.pgm\nresolution: 0.05\n");
    write_png(localizer_png_);
    write_text(
      localizer_yaml_,
      "image: localizer_map.png\n"
      "mode: trinary\n"
      "resolution: 0.05\n"
      "origin: [1.0, -2.0, 0.3]\n"
      "negate: 0\n"
      "occupied_thresh: 0.65\n"
      "free_thresh: 0.196\n");
  }

  void TearDown() override
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  static void write_text(const fs::path & path, const std::string & text)
  {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open());
    stream << text;
    ASSERT_TRUE(stream.good());
  }

  static void write_png(const fs::path & path)
  {
    static constexpr unsigned char kPngHeader[] = {
      0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a,
    };
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open());
    stream.write(
      reinterpret_cast<const char *>(kPngHeader),
      static_cast<std::streamsize>(sizeof(kPngHeader)));
    ASSERT_TRUE(stream.good());
  }

  static void replace_text(
    const fs::path & path,
    const std::string & from,
    const std::string & to)
  {
    std::ifstream input(path, std::ios::binary);
    ASSERT_TRUE(input.is_open());
    std::ostringstream contents;
    contents << input.rdbuf();
    ASSERT_TRUE(input.good() || input.eof());
    std::string updated = contents.str();
    const std::size_t position = updated.find(from);
    ASSERT_NE(position, std::string::npos);
    updated.replace(position, from.size(), to);
    write_text(path, updated);
  }

  IsaacAssetReloaderOptions options() const
  {
    IsaacAssetReloaderOptions options;
    options.allowed_asset_root = asset_root_;
    options.component.package_name = "isaac_ros_occupancy_grid_localizer";
    options.component.plugin_name =
      "nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode";
    options.component.node_name = "occupancy_grid_localizer";
    options.component.remap_rules = {
      "flatscan:=/flatscan",
      "localization_result:=/localization_result",
    };
    return options;
  }

  FloorAssetRequest request() const
  {
    FloorAssetRequest request;
    request.transaction_id = "tx-17";
    request.identity.building_id = "B11";
    request.identity.floor_id = "F2";
    request.identity.map_id = "map_f2";
    request.identity.asset_epoch = 17U;
    request.identity.asset_digest = "sha256:" + std::string(64U, 'a');
    request.nav_map_yaml = nav_yaml_;
    request.localizer_map_png = localizer_png_;
    request.localizer_params_yaml = localizer_yaml_;
    return request;
  }

  fs::path write_bootstrap_runtime_context()
  {
    const fs::path current_root = asset_root_ / "B11" / "F2" / "current";
    fs::create_directories(current_root / "nav");
    fs::create_directories(current_root / "localizer");
    const fs::path current_nav = current_root / "nav" / "nav_map.yaml";
    const fs::path current_png =
      current_root / "localizer" / "localizer_map.png";
    const fs::path current_yaml =
      current_root / "localizer" / "localizer_params.yaml";
    write_text(current_nav, "image: nav_map.pgm\nresolution: 0.05\n");
    write_png(current_png);
    write_text(
      current_yaml,
      "image: localizer_map.png\n"
      "mode: trinary\n"
      "resolution: 0.05\n"
      "origin: [1.0, -2.0, 0.3]\n"
      "negate: 0\n"
      "occupied_thresh: 0.65\n"
      "free_thresh: 0.196\n");
    write_text(
      current_root / "manifest.json",
      "{\n"
      "  \"schema\": \"njrh.map_manifest.v2\",\n"
      "  \"asset_epoch\": 17,\n"
      "  \"asset_digest_algorithm\": \"sha256\",\n"
      "  \"asset_digest_contract\": \"njrh-map-asset-bundle-v1\",\n"
      "  \"asset_digest\": \"sha256:" + std::string(64U, 'a') + "\",\n"
      "  \"map_id\": \"map_f2\",\n"
      "  \"building_id\": \"B11\",\n"
      "  \"floor_id\": \"F2\",\n"
      "  \"active\": true\n"
      "}\n");
    const fs::path runtime_context = root_ / "runtime_map_context.json";
    write_text(
      runtime_context,
      "{\n"
      "  \"schema\": \"njrh.runtime_map_context.v1\",\n"
      "  \"state\": \"ready\",\n"
      "  \"confirmed\": true,\n"
      "  \"transaction_id\": \"floor-live-17\",\n"
      "  \"building_id\": \"B11\",\n"
      "  \"floor_id\": \"F2\",\n"
      "  \"map_id\": \"map_f2\",\n"
      "  \"asset_epoch\": 17,\n"
      "  \"asset_digest\": \"sha256:" + std::string(64U, 'a') + "\",\n"
      "  \"localizer_generation\": 8,\n"
      "  \"explicit_relocalization_sequence\": 12,\n"
      "  \"updated_at\": 1234.5\n"
      "}\n");
    return runtime_context;
  }

  fs::path root_;
  fs::path asset_root_;
  fs::path map_root_;
  fs::path nav_yaml_;
  fs::path localizer_png_;
  fs::path localizer_yaml_;
};

TEST_F(IsaacAssetReloaderTest, AppliesExactTargetAndPreservesCapturedTuning)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  ASSERT_TRUE(result.success) << result.state.failure_code << ": " << result.state.detail;
  EXPECT_FALSE(result.idempotent);
  EXPECT_EQ(result.state.localizer_generation, 1U);
  EXPECT_TRUE(result.state.localizer_ready);
  EXPECT_EQ(result.state.active_identity.building_id, "B11");
  EXPECT_EQ(result.state.active_identity.floor_id, "F2");
  EXPECT_EQ(result.state.active_identity.map_id, "map_f2");
  EXPECT_EQ(result.state.active_identity.asset_epoch, 17U);
  EXPECT_EQ(
    result.state.active_identity.asset_digest,
    "sha256:" + std::string(64U, 'a'));

  ASSERT_EQ(components.unloaded_ids, std::vector<std::uint64_t>({41U}));
  ASSERT_EQ(components.load_requests.size(), 1U);
  const auto & parameters = components.load_requests.front().parameters;

  const auto * map_yaml_path = find_parameter(parameters, "map_yaml_path");
  ASSERT_NE(map_yaml_path, nullptr);
  EXPECT_EQ(map_yaml_path->string_value, fs::canonical(localizer_yaml_).string());

  const auto * image = find_parameter(parameters, "image");
  ASSERT_NE(image, nullptr);
  EXPECT_EQ(image->string_value, "localizer_map.png");

  const auto * resolution = find_parameter(parameters, "resolution");
  ASSERT_NE(resolution, nullptr);
  EXPECT_DOUBLE_EQ(resolution->double_value, 0.05);

  const auto * origin = find_parameter(parameters, "origin");
  ASSERT_NE(origin, nullptr);
  EXPECT_EQ(origin->double_array_value, std::vector<double>({1.0, -2.0, 0.3}));

  const auto * occupied_thresh = find_parameter(parameters, "occupied_thresh");
  ASSERT_NE(occupied_thresh, nullptr);
  EXPECT_DOUBLE_EQ(occupied_thresh->double_value, 0.65);

  const auto * batch_size = find_parameter(parameters, "batch_size");
  ASSERT_NE(batch_size, nullptr);
  EXPECT_EQ(batch_size->integer_value, 1024);
}

TEST_F(IsaacAssetReloaderTest, RejectsAnAssetPathContainingAnIntermediateSymlink)
{
  const fs::path alias = asset_root_ / "linked_map";
  std::error_code error;
  fs::create_directory_symlink(map_root_, alias, error);
  if (error) {
    GTEST_SKIP() << "directory symlinks unavailable: " << error.message();
  }

  FloorAssetRequest unsafe_request = request();
  unsafe_request.nav_map_yaml = alias / "nav" / "nav_map.yaml";
  unsafe_request.localizer_map_png =
    alias / "localizer" / "localizer_map.png";
  unsafe_request.localizer_params_yaml =
    alias / "localizer" / "localizer_params.yaml";

  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  const ApplyFloorAssetResult result =
    reloader.apply(unsafe_request, components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "ASSET_PATH_SYMLINK");
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, ExactActiveIdentityIsIdempotentWithoutComponentCalls)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  ASSERT_TRUE(reloader.apply(request(), components).success);

  FloorAssetRequest repeated_request = request();
  repeated_request.transaction_id = "tx-18";
  const ApplyFloorAssetResult repeated =
    reloader.apply(repeated_request, components);

  ASSERT_TRUE(repeated.success);
  EXPECT_TRUE(repeated.idempotent);
  EXPECT_TRUE(repeated.state.idempotent);
  EXPECT_FALSE(repeated.state.reloaded);
  EXPECT_EQ(repeated.state.transaction_id, "tx-18");
  EXPECT_EQ(repeated.state.localizer_generation, 1U);
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.capture_calls, 1U);
  EXPECT_EQ(components.unloaded_ids.size(), 1U);
  EXPECT_EQ(components.load_requests.size(), 1U);
}

TEST_F(IsaacAssetReloaderTest, RejectsPathDriftForAnExactActiveIdentity)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  ASSERT_TRUE(reloader.apply(request(), components).success);

  const fs::path alternate_nav_yaml = map_root_ / "nav" / "alternate_nav_map.yaml";
  write_text(alternate_nav_yaml, "image: nav_map.pgm\nresolution: 0.05\n");
  FloorAssetRequest conflicting_request = request();
  conflicting_request.transaction_id = "tx-path-conflict";
  conflicting_request.nav_map_yaml = alternate_nav_yaml;

  const ApplyFloorAssetResult conflicting =
    reloader.apply(conflicting_request, components);

  EXPECT_FALSE(conflicting.success);
  EXPECT_EQ(conflicting.state.failure_code, "ASSET_IDENTITY_PATH_CONFLICT");
  EXPECT_EQ(conflicting.state.localizer_generation, 1U);
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.unloaded_ids.size(), 1U);
  EXPECT_EQ(components.load_requests.size(), 1U);
}

TEST_F(IsaacAssetReloaderTest, RejectsInvalidExactIdentityBeforeComponentPreflight)
{
  FloorAssetRequest invalid_request = request();
  invalid_request.identity.asset_digest = "sha256:ABC";
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;

  const ApplyFloorAssetResult result =
    reloader.apply(invalid_request, components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "ASSET_DIGEST_INVALID");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
}

TEST_F(IsaacAssetReloaderTest, RejectsLocalizerYamlImageMismatchBeforeComponentPreflight)
{
  const fs::path other_png = map_root_ / "localizer" / "other.png";
  write_png(other_png);
  FloorAssetRequest mismatched_request = request();
  mismatched_request.localizer_map_png = other_png;
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;

  const ApplyFloorAssetResult result =
    reloader.apply(mismatched_request, components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "LOCALIZER_IMAGE_MISMATCH");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
}

TEST_F(IsaacAssetReloaderTest, TargetLoadFailureRestoresCapturedParametersButReturnsFailure)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  components.load_results = {
    ComponentOperationResult{
      false, "LOCALIZER_LOAD_TIMEOUT", "target timed out", 0U,
      ComponentPresence::kAbsent},
    ComponentOperationResult{
      true, "", "old component restored", 43U,
      ComponentPresence::kPresent},
  };

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "LOCALIZER_LOAD_TIMEOUT");
  EXPECT_TRUE(result.state.rollback_attempted);
  EXPECT_TRUE(result.state.rollback_succeeded);
  EXPECT_TRUE(result.state.localizer_ready);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_EQ(result.state.localizer_generation, 1U);
  ASSERT_EQ(components.load_requests.size(), 2U);
  const auto * restored_map_yaml =
    find_parameter(components.load_requests.back().parameters, "map_yaml_path");
  ASSERT_NE(restored_map_yaml, nullptr);
  EXPECT_EQ(restored_map_yaml->string_value, "/old/localizer_params.yaml");
}

TEST_F(IsaacAssetReloaderTest, AmbiguousFailedTargetSkipsUnsafeRollback)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  components.load_results = {
    ComponentOperationResult{
      false, "LOCALIZER_LOAD_TIMEOUT", "target presence ambiguous", 0U,
      ComponentPresence::kAmbiguous},
  };

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.rollback_attempted);
  EXPECT_FALSE(result.state.rollback_succeeded);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.localizer_generation, 0U);
  EXPECT_EQ(components.load_requests.size(), 1U);
}

TEST_F(IsaacAssetReloaderTest, UnloadTimeoutWithConfirmedAbsenceRestoresOldComponent)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  components.unload_result = {
    false, "LOCALIZER_UNLOAD_TIMEOUT", "old component disappeared after timeout",
    0U, ComponentPresence::kAbsent};
  components.load_results = {
    ComponentOperationResult{
      true, "", "old component restored", 43U,
      ComponentPresence::kPresent},
  };

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "LOCALIZER_UNLOAD_TIMEOUT");
  EXPECT_TRUE(result.state.rollback_attempted);
  EXPECT_TRUE(result.state.rollback_succeeded);
  EXPECT_TRUE(result.state.localizer_ready);
  EXPECT_EQ(result.state.localizer_generation, 1U);
  ASSERT_EQ(components.load_requests.size(), 1U);
  const auto * restored_map_yaml =
    find_parameter(components.load_requests.front().parameters, "map_yaml_path");
  ASSERT_NE(restored_map_yaml, nullptr);
  EXPECT_EQ(restored_map_yaml->string_value, "/old/localizer_params.yaml");
}

TEST_F(IsaacAssetReloaderTest, MissingCompositionCapabilityFailsBeforeCaptureOrUnload)
{
  IsaacAssetReloader reloader(options());
  FakeComponentManager components;
  components.preflight_result = {
    false, "COMPONENT_MANAGER_CAPABILITY_MISSING",
    "load_node unavailable", 0U, ComponentPresence::kUnknown};

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
    result.state.failure_code, "COMPONENT_MANAGER_CAPABILITY_MISSING");
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, InvalidReplacementDescriptionFailsBeforeComponentPreflight)
{
  IsaacAssetReloaderOptions invalid_options = options();
  invalid_options.component.plugin_name.clear();
  IsaacAssetReloader reloader(invalid_options);
  FakeComponentManager components;

  const ApplyFloorAssetResult result = reloader.apply(request(), components);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.state.failure_code, "COMPONENT_CONFIG_INVALID");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, BootstrapsExactLiveIdentityFromDurableReadyContext)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  const fs::path current_root = asset_root_ / "B11" / "F2" / "current";
  FakeComponentManager components;
  components.capture_result.snapshot.parameters = {
    string_parameter(
      "map_yaml_path",
      fs::canonical(
        current_root / "localizer" / "localizer_params.yaml").string()),
    string_parameter("image", "localizer_map.png"),
    double_parameter("resolution", 0.05),
    double_array_parameter("origin", {1.0, -2.0, 0.3}),
    double_parameter("occupied_thresh", 0.65),
    integer_parameter("batch_size", 1024),
  };
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  ASSERT_TRUE(result.success) << result.state.failure_code << ": " << result.state.detail;
  EXPECT_FALSE(result.idempotent);
  EXPECT_TRUE(result.state.active_identity_valid);
  EXPECT_TRUE(result.state.localizer_ready);
  EXPECT_EQ(result.state.transaction_id, "floor-live-17");
  EXPECT_EQ(result.state.active_identity.building_id, "B11");
  EXPECT_EQ(result.state.active_identity.floor_id, "F2");
  EXPECT_EQ(result.state.active_identity.map_id, "map_f2");
  EXPECT_EQ(result.state.active_identity.asset_epoch, 17U);
  EXPECT_EQ(
    result.state.active_identity.asset_digest,
    "sha256:" + std::string(64U, 'a'));
  EXPECT_EQ(result.state.localizer_generation, 8U);
  EXPECT_EQ(
    result.state.active_nav_map_yaml,
    fs::canonical(current_root / "nav" / "nav_map.yaml"));
  EXPECT_EQ(
    result.state.active_localizer_map_png,
    fs::canonical(current_root / "localizer" / "localizer_map.png"));
  EXPECT_EQ(
    result.state.active_localizer_params_yaml,
    fs::canonical(current_root / "localizer" / "localizer_params.yaml"));
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.capture_calls, 1U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, MissingDurableGenerationFailsClosedBeforeComponentPreflight)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  replace_text(
    runtime_context,
    "  \"localizer_generation\": 8,\n",
    "");
  FakeComponentManager components;
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.localizer_generation, 0U);
  EXPECT_EQ(result.state.failure_code, "RUNTIME_CONTEXT_IDENTITY_INVALID");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, UnconfirmedDurableContextFailsClosedBeforeComponentPreflight)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  replace_text(runtime_context, "\"confirmed\": true", "\"confirmed\": false");
  FakeComponentManager components;
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.failure_code, "RUNTIME_CONTEXT_NOT_READY");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, DuplicateDurableContextFieldFailsClosed)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  replace_text(
    runtime_context,
    "  \"state\": \"ready\",\n",
    "  \"state\": \"ready\",\n"
    "  \"state\": \"ready\",\n");
  FakeComponentManager components;
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.failure_code, "RUNTIME_CONTEXT_INVALID");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
}

TEST_F(IsaacAssetReloaderTest, CurrentManifestIdentityDriftFailsClosedBeforeComponentPreflight)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  const fs::path manifest =
    asset_root_ / "B11" / "F2" / "current" / "manifest.json";
  replace_text(manifest, "\"map_id\": \"map_f2\"", "\"map_id\": \"map_other\"");
  FakeComponentManager components;
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.failure_code, "RUNTIME_ASSET_IDENTITY_MISMATCH");
  EXPECT_EQ(components.preflight_calls, 0U);
  EXPECT_EQ(components.capture_calls, 0U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, LiveParameterDriftFailsClosedWithoutReload)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  const fs::path current_root = asset_root_ / "B11" / "F2" / "current";
  FakeComponentManager components;
  components.capture_result.snapshot.parameters = {
    string_parameter(
      "map_yaml_path",
      fs::canonical(
        current_root / "localizer" / "localizer_params.yaml").string()),
    string_parameter("image", "localizer_map.png"),
    double_parameter("resolution", 0.10),
    double_array_parameter("origin", {1.0, -2.0, 0.3}),
    double_parameter("occupied_thresh", 0.65),
  };
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.localizer_generation, 0U);
  EXPECT_EQ(result.state.failure_code, "LIVE_LOCALIZER_PARAMETER_MISMATCH");
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.capture_calls, 1U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

TEST_F(IsaacAssetReloaderTest, MissingLiveComponentFailsClosedWithoutMutation)
{
  const fs::path runtime_context = write_bootstrap_runtime_context();
  FakeComponentManager components;
  components.capture_result = {
    false,
    "LOCALIZER_COMPONENT_NOT_FOUND",
    "configured Isaac component is absent",
    {},
  };
  IsaacAssetReloader reloader(options());

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, components);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.state.active_identity_valid);
  EXPECT_FALSE(result.state.localizer_ready);
  EXPECT_EQ(result.state.localizer_generation, 0U);
  EXPECT_EQ(result.state.failure_code, "LOCALIZER_COMPONENT_NOT_FOUND");
  EXPECT_EQ(components.preflight_calls, 1U);
  EXPECT_EQ(components.capture_calls, 1U);
  EXPECT_TRUE(components.unloaded_ids.empty());
  EXPECT_TRUE(components.load_requests.empty());
}

}  // namespace
