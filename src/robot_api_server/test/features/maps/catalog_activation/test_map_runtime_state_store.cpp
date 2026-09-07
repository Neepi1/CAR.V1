#include <chrono>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"

#include \
  "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"

namespace robot_api_server::features::maps
{
namespace
{

class ScopedTestDirectory
{
public:
  ScopedTestDirectory()
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("njrh_map_runtime_state_store_test_" + std::to_string(suffix));
    std::filesystem::create_directories(path_);
  }

  ~ScopedTestDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept {return path_;}

private:
  std::filesystem::path path_;
};

MapManifest test_manifest()
{
  MapManifest manifest;
  manifest.map_id = "map_001";
  manifest.display_name = "Commercial floor";
  manifest.building_id = "B15";
  manifest.floor_id = "F1";
  manifest.asset_epoch = 42U;
  manifest.asset_digest = "sha256:test";
  return manifest;
}

TEST(MapRuntimeStateStoreTest, PreservesRuntimeContextFormatAndClearSemantics)
{
  ScopedTestDirectory directory;
  const auto context_file = directory.path() / "runtime_context.json";
  const auto selection_file = directory.path() / "last_selection.json";
  MapRuntimeStateStore store(context_file, selection_file);
  const auto manifest = test_manifest();

  EXPECT_FALSE(store.read_runtime_map_context().has_value());
  store.write_runtime_map_context(
    manifest, "ready", true, "runtime confirmed", "NAV2_READY");

  const auto context = store.read_runtime_map_context();
  ASSERT_TRUE(context.has_value());
  EXPECT_TRUE(context->confirmed);
  EXPECT_EQ(context->state, "ready");
  EXPECT_EQ(context->startup_stage, "NAV2_READY");
  EXPECT_EQ(context->message, "runtime confirmed");
  EXPECT_EQ(context->map_id, manifest.map_id);
  EXPECT_EQ(context->building_id, manifest.building_id);
  EXPECT_EQ(context->floor_id, manifest.floor_id);
  EXPECT_EQ(context->asset_epoch, manifest.asset_epoch);
  EXPECT_EQ(context->asset_digest, manifest.asset_digest);
  EXPECT_GT(context->updated_at_sec, 0.0);

  store.clear_runtime_map_context();
  EXPECT_FALSE(std::filesystem::exists(context_file));
  EXPECT_FALSE(store.read_runtime_map_context().has_value());
}

TEST(MapRuntimeStateStoreTest, PreservesLastNavigationSelectionSchema)
{
  ScopedTestDirectory directory;
  const auto selection_file = directory.path() / "last_selection.json";
  MapRuntimeStateStore store(directory.path() / "runtime_context.json", selection_file);
  const auto manifest = test_manifest();

  store.write_last_navigation_map_selection(manifest, "navigation_start");
  const auto body = read_text_file(selection_file);
  EXPECT_NE(body.find("\"schema\": \"njrh.last_navigation_map.v1\""), std::string::npos);
  EXPECT_NE(body.find("\"reason\": \"navigation_start\""), std::string::npos);
  EXPECT_NE(body.find("\"map_id\": \"map_001\""), std::string::npos);
  EXPECT_NE(body.find("\"building_id\": \"B15\""), std::string::npos);
  EXPECT_NE(body.find("\"floor_id\": \"F1\""), std::string::npos);
}

TEST(MapRuntimeStateStoreTest, EmptyPathsRemainNoOp)
{
  MapRuntimeStateStore store({}, {});
  const auto manifest = test_manifest();
  EXPECT_NO_THROW(store.write_runtime_map_context(manifest, "ready", true, "ok"));
  EXPECT_FALSE(store.read_runtime_map_context().has_value());
  EXPECT_NO_THROW(store.clear_runtime_map_context());
  EXPECT_NO_THROW(store.write_last_navigation_map_selection(manifest, "test"));
}

}  // namespace
}  // namespace robot_api_server::features::maps
