#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/keepout_layer.hpp"
#include "robot_api_server/map_manifest_io.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_keepout_test_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & path() const
  {
    return root_;
  }

private:
  fs::path root_;
};

class FailingRuntimePort : public KeepoutRuntimePort
{
public:
  bool apply(
    const fs::path &,
    const KeepoutMaskSummary & expected,
    KeepoutRuntimeProof & proof,
    std::string & error) override
  {
    ++apply_calls;
    applied_active_cells = expected.active_cells;
    proof.mutation_state = KeepoutRuntimeMutationState::MAY_HAVE_CHANGED;
    error = "synthetic runtime load failure";
    return false;
  }

  bool restore(
    const fs::path &,
    const KeepoutMaskSummary & expected,
    KeepoutRuntimeProof & proof,
    std::string &) override
  {
    ++restore_calls;
    restored_active_cells = expected.active_cells;
    proof.mask_server_active = true;
    proof.filter_plugin_enabled = true;
    proof.load_map_succeeded = true;
    proof.mask_topic_matches = true;
    proof.global_costmap_cleared = true;
    proof.global_costmap_updated = true;
    proof.global_costmap_content_matches = true;
    return true;
  }

  int apply_calls{0};
  int restore_calls{0};
  std::size_t applied_active_cells{0U};
  std::size_t restored_active_cells{999U};
};

class IncompleteProofRuntimePort : public KeepoutRuntimePort
{
public:
  bool apply(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof & proof,
    std::string &) override
  {
    ++apply_calls;
    proof.load_map_succeeded = true;
    return true;
  }

  bool restore(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof & proof,
    std::string &) override
  {
    ++restore_calls;
    proof.mask_server_active = true;
    proof.filter_plugin_enabled = true;
    proof.load_map_succeeded = true;
    proof.mask_topic_matches = true;
    proof.global_costmap_cleared = true;
    proof.global_costmap_updated = true;
    proof.global_costmap_content_matches = true;
    return true;
  }

  int apply_calls{0};
  int restore_calls{0};
};

class SuccessfulRuntimePort : public KeepoutRuntimePort
{
public:
  bool apply(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof & proof,
    std::string &) override
  {
    ++apply_calls;
    proof.mutation_state = KeepoutRuntimeMutationState::MAY_HAVE_CHANGED;
    proof.mask_server_active = true;
    proof.filter_plugin_enabled = true;
    proof.load_map_succeeded = true;
    proof.mask_topic_matches = true;
    proof.global_costmap_cleared = true;
    proof.global_costmap_updated = true;
    proof.global_costmap_content_matches = true;
    proof.detail = "synthetic complete proof";
    return true;
  }

  bool restore(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof &,
    std::string &) override
  {
    ++restore_calls;
    return false;
  }

  int apply_calls{0};
  int restore_calls{0};
};

class PreflightFailingRuntimePort : public KeepoutRuntimePort
{
public:
  bool apply(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof &,
    std::string & error) override
  {
    ++apply_calls;
    error = "synthetic preflight rejection";
    return false;
  }

  bool restore(
    const fs::path &,
    const KeepoutMaskSummary &,
    KeepoutRuntimeProof &,
    std::string &) override
  {
    ++restore_calls;
    return false;
  }

  int apply_calls{0};
  int restore_calls{0};
};

std::vector<std::uint8_t> read_p5_pixels(
  const fs::path & path,
  std::uint32_t & width,
  std::uint32_t & height)
{
  std::ifstream input(path, std::ios::binary);
  std::string magic;
  std::uint32_t max_value = 0U;
  input >> magic >> width >> height >> max_value;
  input.get();
  EXPECT_EQ(magic, "P5");
  EXPECT_EQ(max_value, 255U);
  std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
  input.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  EXPECT_EQ(input.gcount(), static_cast<std::streamsize>(pixels.size()));
  return pixels;
}

MapManifest create_neutral_test_map(
  const fs::path & root,
  const std::string & map_id,
  const std::uint32_t width,
  const std::uint32_t height,
  const double resolution = 1.0)
{
  MapManifest map;
  map.map_id = map_id;
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = root / "maps" / map.map_id;
  fill_manifest_paths(map);
  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");

  std::ostringstream yaml;
  yaml << "image: nav_map.pgm\n"
       << "resolution: " << resolution << "\n"
       << "origin: [0.0, 0.0, 0.0]\n";
  write_text_file(map.nav_map_yaml, yaml.str());
  write_pgm_file(
    map.nav_map_pgm,
    width,
    height,
    std::vector<std::uint8_t>(static_cast<std::size_t>(width) * height, 254U));

  std::ostringstream mask_yaml;
  mask_yaml << "image: keepout_mask.pgm\n"
            << "resolution: " << resolution << "\n"
            << "origin: [0.0, 0.0, 0.0]\n";
  write_text_file(map.keepout_mask_yaml, mask_yaml.str());
  write_pgm_file(
    map.keepout_mask_pgm,
    width,
    height,
    std::vector<std::uint8_t>(static_cast<std::size_t>(width) * height, 254U));
  return map;
}

TEST(KeepoutLayerModule, RasterizesMapFrameLineWithRotatedOriginAndPgmYFlip)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_rotated";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\n"
    "resolution: 0.500000\n"
    "origin: [10.000000, 20.000000, 1.5707963267948966]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "mode: trinary\n");
  write_pgm_file(map.nav_map_pgm, 10U, 8U, std::vector<std::uint8_t>(80U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\n"
    "resolution: 0.500000\n"
    "origin: [10.000000, 20.000000, 1.5707963267948966]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "mode: trinary\n");
  write_pgm_file(map.keepout_mask_pgm, 10U, 8U, std::vector<std::uint8_t>(80U, 254U));

  // Local grid points (u, v)=(1,1)->(3,1), rotated by +90 degrees about (10,20).
  const std::string request =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_rotated\","
    "\"keepout_lines\":[{"
      "\"id\":\"line_1\","
      "\"name\":\"rotated line\","
      "\"width_m\":0.6,"
      "\"points\":[{\"x\":9.0,\"y\":21.0},{\"x\":9.0,\"y\":23.0}]"
    "}],"
    "\"keepout_polygons\":[],"
    "\"reload_filter\":false"
    "}";

  ReplaceKeepoutCommand command;
  command.request_json = request;
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  const auto result = module.replace(command, nullptr);

  EXPECT_TRUE(result.persisted);
  EXPECT_TRUE(result.changed);
  EXPECT_FALSE(result.runtime_effective);
  EXPECT_EQ(result.outcome, "SAVED_DEFERRED_INACTIVE");
  EXPECT_EQ(result.mask.width, 10U);
  EXPECT_EQ(result.mask.height, 8U);
  EXPECT_GT(result.mask.active_cells, 0U);

  std::uint32_t width = 0U;
  std::uint32_t height = 0U;
  const auto pixels = read_p5_pixels(map.keepout_mask_pgm, width, height);
  ASSERT_EQ(width, 10U);
  ASSERT_EQ(height, 8U);

  // Local midpoint (u,v)=(2,1) -> grid col=4,row-from-bottom=2 -> PGM row=5.
  EXPECT_EQ(pixels[5U * width + 4U], 0U);
  // A distant cell stays neutral.
  EXPECT_EQ(pixels[0U], 254U);
}

TEST(KeepoutLayerModule, RasterizesKeepoutPolygonInterior)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_polygon";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\n"
    "resolution: 1.000000\n"
    "origin: [0.000000, 0.000000, 0.000000]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "mode: trinary\n");
  write_pgm_file(map.nav_map_pgm, 6U, 6U, std::vector<std::uint8_t>(36U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\n"
    "resolution: 1.000000\n"
    "origin: [0.000000, 0.000000, 0.000000]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "mode: trinary\n");
  write_pgm_file(map.keepout_mask_pgm, 6U, 6U, std::vector<std::uint8_t>(36U, 254U));

  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_polygon\","
    "\"keepout_lines\":[],"
    "\"keepout_polygons\":[{"
      "\"id\":\"polygon_1\","
      "\"name\":\"room\","
      "\"polygon\":["
        "{\"x\":1.0,\"y\":1.0},"
        "{\"x\":4.0,\"y\":1.0},"
        "{\"x\":4.0,\"y\":4.0},"
        "{\"x\":1.0,\"y\":4.0}"
      "]"
    "}],"
    "\"reload_filter\":false"
    "}";
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  const auto result = module.replace(command, nullptr);

  EXPECT_TRUE(result.persisted);
  EXPECT_GT(result.mask.active_cells, 0U);
  std::uint32_t width = 0U;
  std::uint32_t height = 0U;
  const auto pixels = read_p5_pixels(map.keepout_mask_pgm, width, height);
  ASSERT_EQ(width, 6U);
  ASSERT_EQ(height, 6U);
  EXPECT_EQ(pixels[3U * width + 2U], 0U);
  EXPECT_EQ(pixels[0U], 254U);
}

TEST(KeepoutLayerModule, RejectsUnmanagedNonNeutralMaskWithoutChangingIt)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_unmanaged";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 3U, 3U, std::vector<std::uint8_t>(9U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n");
  auto unmanaged_pixels = std::vector<std::uint8_t>(9U, 254U);
  unmanaged_pixels[4U] = 0U;
  write_pgm_file(map.keepout_mask_pgm, 3U, 3U, unmanaged_pixels);
  const auto original = read_binary_file(map.keepout_mask_pgm);

  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_unmanaged\","
    "\"keepout_lines\":[],"
    "\"keepout_polygons\":[],"
    "\"reload_filter\":false"
    "}";
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected unmanaged keepout mask rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "UNMANAGED_NON_NEUTRAL_MASK");
    EXPECT_EQ(error.http_status(), 409);
  }

  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), original);
  EXPECT_FALSE(fs::exists(command.projections.front().semantic_json));
}

TEST(KeepoutLayerModule, RuntimeFailureRollsBackAllAssetsAndRestoresPreviousMask)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_runtime_rollback";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  const std::string map_yaml =
    "image: nav_map.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n";
  write_text_file(map.nav_map_yaml, map_yaml);
  write_pgm_file(map.nav_map_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));
  const std::string old_mask_yaml =
    "image: keepout_mask.pgm\n"
    "resolution: 1.0\n"
    "origin: [0.0, 0.0, 0.0]\n";
  write_text_file(map.keepout_mask_yaml, old_mask_yaml);
  write_pgm_file(map.keepout_mask_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));
  const auto semantic_path = map.root / "filters" / "keepout_semantic_layer.json";
  const std::string old_semantic =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_runtime_rollback\","
    "\"keepout_lines\":[],\"keepout_polygons\":[],\"reload_filter\":true}\n";
  write_text_file(semantic_path, old_semantic);

  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);
  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_runtime_rollback\","
    "\"keepout_lines\":[{"
      "\"id\":\"line_1\","
      "\"name\":\"line\","
      "\"width_m\":0.6,"
      "\"points\":[{\"x\":0.5,\"y\":1.5},{\"x\":3.5,\"y\":1.5}]"
    "}],"
    "\"keepout_polygons\":[],"
    "\"reload_filter\":true"
    "}";
  command.map = map;
  command.runtime_selected = true;
  command.runtime_apply_requested = true;
  command.projections.push_back({semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});

  FailingRuntimePort runtime;
  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, &runtime);
    FAIL() << "Expected runtime load failure";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "RUNTIME_LOAD_FAILED");
    EXPECT_EQ(error.http_status(), 503);
  }

  EXPECT_EQ(runtime.apply_calls, 1);
  EXPECT_EQ(runtime.restore_calls, 1);
  EXPECT_GT(runtime.applied_active_cells, 0U);
  EXPECT_EQ(runtime.restored_active_cells, 0U);
  EXPECT_EQ(read_text_file(semantic_path), old_semantic);
  EXPECT_EQ(read_text_file(map.keepout_mask_yaml), old_mask_yaml);
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
}

TEST(KeepoutLayerModule, RejectsSelfIntersectingPolygonBeforeWritingAssets)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_bow_tie";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 6U, 6U, std::vector<std::uint8_t>(36U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 6U, 6U, std::vector<std::uint8_t>(36U, 254U));
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);

  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_bow_tie\","
    "\"keepout_lines\":[],"
    "\"keepout_polygons\":[{"
      "\"id\":\"bad_polygon\","
      "\"name\":\"bow tie\","
      "\"polygon\":["
        "{\"x\":1.0,\"y\":1.0},"
        "{\"x\":4.0,\"y\":4.0},"
        "{\"x\":1.0,\"y\":4.0},"
        "{\"x\":4.0,\"y\":1.0}"
      "]"
    "}],"
    "\"reload_filter\":false"
    "}";
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected self-intersection rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "INVALID_GEOMETRY");
    EXPECT_EQ(error.http_status(), 422);
  }
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
  EXPECT_FALSE(fs::exists(command.projections.front().semantic_json));
}

TEST(KeepoutLayerModule, IncompleteRuntimeProofCannotBeReportedAsEffective)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_incomplete_proof";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));
  const auto semantic_path = map.root / "filters" / "keepout_semantic_layer.json";
  const std::string old_semantic =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_incomplete_proof\","
    "\"keepout_lines\":[],\"keepout_polygons\":[],\"reload_filter\":true}\n";
  write_text_file(semantic_path, old_semantic);
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);

  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_incomplete_proof\","
    "\"keepout_lines\":[{"
      "\"id\":\"line_1\","
      "\"name\":\"line\","
      "\"width_m\":0.6,"
      "\"points\":[{\"x\":0.5,\"y\":1.5},{\"x\":3.5,\"y\":1.5}]"
    "}],"
    "\"keepout_polygons\":[],"
    "\"reload_filter\":true"
    "}";
  command.map = map;
  command.runtime_selected = true;
  command.runtime_apply_requested = true;
  command.projections.push_back({semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});

  IncompleteProofRuntimePort runtime;
  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, &runtime);
    FAIL() << "Expected incomplete runtime proof rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "RUNTIME_PROOF_FAILED");
    EXPECT_EQ(error.http_status(), 503);
  }
  EXPECT_EQ(runtime.apply_calls, 1);
  EXPECT_EQ(runtime.restore_calls, 1);
  EXPECT_EQ(read_text_file(semantic_path), old_semantic);
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
}

TEST(KeepoutLayerModule, IdenticalInactiveReplaceReturnsNoChangeWithStableRevision)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_idempotent";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));

  ReplaceKeepoutCommand command;
  command.request_json =
    "{"
    "\"building_id\":\"B1\","
    "\"floor_id\":\"F1\","
    "\"map_id\":\"map_idempotent\","
    "\"keepout_lines\":[{"
      "\"id\":\"line_1\","
      "\"name\":\"line\","
      "\"width_m\":0.6,"
      "\"points\":[{\"x\":0.5,\"y\":1.5},{\"x\":3.5,\"y\":1.5}]"
    "}],"
    "\"keepout_polygons\":[],"
    "\"reload_filter\":true"
    "}";
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  const auto first = module.replace(command, nullptr);
  const auto semantic_after_first = read_text_file(command.projections.front().semantic_json);
  const auto pgm_after_first = read_binary_file(map.keepout_mask_pgm);
  const auto second = module.replace(command, nullptr);

  EXPECT_TRUE(first.changed);
  EXPECT_FALSE(second.changed);
  EXPECT_EQ(second.outcome, "NO_CHANGE");
  EXPECT_EQ(second.revision, first.revision);
  EXPECT_EQ(second.previous_revision, first.revision);
  EXPECT_TRUE(second.effective_on_next_activation);
  EXPECT_EQ(read_text_file(command.projections.front().semantic_json), semantic_after_first);
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), pgm_after_first);
}

TEST(KeepoutLayerModule, ClearingLastLineProducesACompletelyNeutralMask)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_clear";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));

  ReplaceKeepoutCommand command;
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_clear\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":0.6,"
    "\"points\":[{\"x\":0.5,\"y\":1.5},{\"x\":3.5,\"y\":1.5}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":true}";

  const KeepoutLayerModule module;
  const auto populated = module.replace(command, nullptr);
  ASSERT_GT(populated.mask.active_cells, 0U);

  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_clear\","
    "\"keepout_lines\":[],\"keepout_polygons\":[],\"reload_filter\":true}";
  const auto cleared = module.replace(command, nullptr);
  EXPECT_TRUE(cleared.changed);
  EXPECT_EQ(cleared.mask.active_cells, 0U);
  EXPECT_FALSE(cleared.mask.cleared_costmap_samples.empty());

  std::uint32_t width = 0U;
  std::uint32_t height = 0U;
  const auto pixels = read_p5_pixels(map.keepout_mask_pgm, width, height);
  EXPECT_EQ(
    std::count(pixels.begin(), pixels.end(), static_cast<std::uint8_t>(254U)),
    pixels.size());
}

TEST(KeepoutLayerModule, CompleteRuntimeProofReturnsApplied)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_applied";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 8U, 8U, std::vector<std::uint8_t>(64U, 254U));
  write_text_file(
    map.root / "filters" / "keepout_semantic_layer.json",
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_applied\","
    "\"keepout_lines\":[],\"keepout_polygons\":[],\"reload_filter\":true}\n");

  ReplaceKeepoutCommand command;
  command.map = map;
  command.runtime_selected = true;
  command.runtime_apply_requested = true;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_applied\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":0.6,"
    "\"points\":[{\"x\":0.5,\"y\":1.5},{\"x\":3.5,\"y\":1.5}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":true}";

  SuccessfulRuntimePort runtime;
  const KeepoutLayerModule module;
  const auto result = module.replace(command, &runtime);
  EXPECT_EQ(result.outcome, "APPLIED");
  EXPECT_TRUE(result.runtime_effective);
  EXPECT_FALSE(result.effective_on_next_activation);
  EXPECT_EQ(runtime.apply_calls, 1);
  EXPECT_EQ(runtime.restore_calls, 0);
}

TEST(KeepoutLayerModule, RejectsAFilterThatBlocksTheEntireMap)
{
  TemporaryDirectory temporary;
  MapManifest map;
  map.map_id = "map_all_blocked";
  map.safe_map_name = "nav_map";
  map.building_id = "B1";
  map.floor_id = "F1";
  map.root = temporary.path() / "maps" / map.map_id;
  fill_manifest_paths(map);

  fs::create_directories(map.root / "nav");
  fs::create_directories(map.root / "filters");
  write_text_file(
    map.nav_map_yaml,
    "image: nav_map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.nav_map_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));
  write_text_file(
    map.keepout_mask_yaml,
    "image: keepout_mask.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n");
  write_pgm_file(map.keepout_mask_pgm, 4U, 4U, std::vector<std::uint8_t>(16U, 254U));

  ReplaceKeepoutCommand command;
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\",\"map_id\":\"map_all_blocked\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":100.0,"
    "\"points\":[{\"x\":0.0,\"y\":0.0},{\"x\":3.0,\"y\":3.0}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":true}";

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected all-blocked map rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "KEEP_OUT_MASK_ALL_BLOCKED");
    EXPECT_EQ(error.http_status(), 422);
  }
  EXPECT_FALSE(fs::exists(command.projections.front().semantic_json));
}

TEST(KeepoutLayerModule, RejectsAnOutOfMapFeatureEvenWhenAnotherFeatureIsValid)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_per_feature_intersection", 10U, 10U);
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);

  ReplaceKeepoutCommand command;
  command.map = map;
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_per_feature_intersection\","
    "\"keepout_lines\":["
      "{\"id\":\"valid_line\",\"width_m\":0.2,"
      "\"points\":[{\"x\":1.0,\"y\":1.0},{\"x\":4.0,\"y\":1.0}]},"
      "{\"id\":\"off_map_line\",\"width_m\":0.2,"
      "\"points\":[{\"x\":100.0,\"y\":100.0},{\"x\":104.0,\"y\":100.0}]}"
    "],\"keepout_polygons\":[],\"reload_filter\":false}";

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected per-feature map-intersection rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "INVALID_GEOMETRY");
    EXPECT_EQ(error.http_status(), 422);
    EXPECT_NE(std::string(error.what()).find("off_map_line"), std::string::npos);
  }
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
  EXPECT_FALSE(fs::exists(command.projections.front().semantic_json));
}

TEST(KeepoutLayerModule, RejectsRasterWorkThatExceedsTheBoundedBudget)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_raster_budget", 400U, 400U);
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);

  std::ostringstream request;
  request << "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
          << "\"map_id\":\"map_raster_budget\","
          << "\"keepout_lines\":[{\"id\":\"expensive_line\","
          << "\"width_m\":0.1,\"points\":[";
  for (std::size_t index = 0U; index < 71U; ++index) {
    if (index > 0U) {
      request << ',';
    }
    if (index % 2U == 0U) {
      request << "{\"x\":1.0,\"y\":1.0}";
    } else {
      request << "{\"x\":398.0,\"y\":398.0}";
    }
  }
  request << "]}],\"keepout_polygons\":[],\"reload_filter\":false}";

  ReplaceKeepoutCommand command;
  command.map = map;
  command.request_json = request.str();
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected bounded-raster-work rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "KEEP_OUT_COMPLEXITY_LIMIT");
    EXPECT_EQ(error.http_status(), 422);
    EXPECT_NE(std::string(error.what()).find("expensive_line"), std::string::npos);
  }
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
  EXPECT_FALSE(fs::exists(command.projections.front().semantic_json));
}

TEST(KeepoutLayerModule, KeepsAtLeastOneCostmapSamplePerFeature)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_feature_samples", 12U, 12U);

  ReplaceKeepoutCommand command;
  command.map = map;
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_feature_samples\","
    "\"keepout_lines\":["
      "{\"id\":\"line_a\",\"width_m\":0.2,"
      "\"points\":[{\"x\":1.0,\"y\":1.0},{\"x\":4.0,\"y\":1.0}]},"
      "{\"id\":\"line_b\",\"width_m\":0.2,"
      "\"points\":[{\"x\":7.0,\"y\":8.0},{\"x\":10.0,\"y\":8.0}]}"
    "],\"keepout_polygons\":[],\"reload_filter\":false}";
  command.projections.push_back(
    {map.root / "filters" / "keepout_semantic_layer.json",
      map.keepout_mask_yaml,
      map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  const auto result = module.replace(command, nullptr);
  EXPECT_TRUE(result.persisted);
  EXPECT_GE(result.mask.blocked_costmap_samples.size(), 2U);
}

TEST(KeepoutLayerModule, RuntimePreflightFailureRollsBackAssetsWithoutRuntimeRestore)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_runtime_preflight", 8U, 8U, 0.5);
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);
  const auto old_yaml = read_text_file(map.keepout_mask_yaml);
  const auto semantic_path =
    map.root / "filters" / "keepout_semantic_layer.json";

  ReplaceKeepoutCommand command;
  command.map = map;
  command.runtime_selected = true;
  command.runtime_apply_requested = true;
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_runtime_preflight\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":0.2,"
    "\"points\":[{\"x\":0.5,\"y\":1.0},{\"x\":3.0,\"y\":1.0}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":true}";
  command.projections.push_back(
    {semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});

  PreflightFailingRuntimePort runtime;
  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, &runtime);
    FAIL() << "Expected runtime preflight rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "RUNTIME_LOAD_FAILED");
    EXPECT_EQ(error.http_status(), 503);
  }
  EXPECT_EQ(runtime.apply_calls, 1);
  EXPECT_EQ(runtime.restore_calls, 0);
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
  EXPECT_EQ(read_text_file(map.keepout_mask_yaml), old_yaml);
  EXPECT_FALSE(fs::exists(semantic_path));
}

TEST(KeepoutLayerModule, RejectsAStaleExpectedRevisionBeforeOverwritingAssets)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_revision_conflict", 10U, 10U);
  const auto semantic_path =
    map.root / "filters" / "keepout_semantic_layer.json";

  ReplaceKeepoutCommand command;
  command.map = map;
  command.projections.push_back(
    {semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_revision_conflict\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":0.2,"
    "\"points\":[{\"x\":1.0,\"y\":1.0},{\"x\":4.0,\"y\":1.0}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":false}";

  const KeepoutLayerModule module;
  const auto first = module.replace(command, nullptr);
  const auto semantic_after_first = read_text_file(semantic_path);
  const auto pgm_after_first = read_binary_file(map.keepout_mask_pgm);

  command.expected_revision = first.previous_revision;
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_revision_conflict\","
    "\"keepout_lines\":[{\"id\":\"line_2\",\"width_m\":0.2,"
    "\"points\":[{\"x\":1.0,\"y\":5.0},{\"x\":4.0,\"y\":5.0}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":false}";
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected stale revision rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "REVISION_CONFLICT");
    EXPECT_EQ(error.http_status(), 412);
  }
  EXPECT_EQ(read_text_file(semantic_path), semantic_after_first);
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), pgm_after_first);
}

TEST(KeepoutLayerModule, RejectsSemanticAndMaskOccupancyMismatch)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_integrity_mismatch", 10U, 10U);
  const auto semantic_path =
    map.root / "filters" / "keepout_semantic_layer.json";
  write_text_file(
    semantic_path,
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_integrity_mismatch\","
    "\"keepout_lines\":[{\"id\":\"line_1\",\"width_m\":0.2,"
    "\"points\":[{\"x\":1.0,\"y\":1.0},{\"x\":4.0,\"y\":1.0}]}],"
    "\"keepout_polygons\":[],\"reload_filter\":false}\n");
  const auto old_pgm = read_binary_file(map.keepout_mask_pgm);

  ReplaceKeepoutCommand command;
  command.map = map;
  command.request_json =
    "{\"building_id\":\"B1\",\"floor_id\":\"F1\","
    "\"map_id\":\"map_integrity_mismatch\","
    "\"keepout_lines\":[],\"keepout_polygons\":[],"
    "\"reload_filter\":false}";
  command.projections.push_back(
    {semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});

  const KeepoutLayerModule module;
  try {
    (void)module.replace(command, nullptr);
    FAIL() << "Expected keepout semantic/mask integrity rejection";
  } catch (const KeepoutLayerError & error) {
    EXPECT_EQ(error.code(), "KEEP_OUT_INTEGRITY_MISMATCH");
    EXPECT_EQ(error.http_status(), 409);
  }
  EXPECT_EQ(read_binary_file(map.keepout_mask_pgm), old_pgm);
}

TEST(KeepoutLayerModule, CanonicalRevisionIgnoresTransportFieldsAndJsonWhitespace)
{
  TemporaryDirectory temporary;
  const auto map = create_neutral_test_map(
    temporary.path(), "map_canonical_revision", 10U, 10U);
  const auto semantic_path =
    map.root / "filters" / "keepout_semantic_layer.json";

  ReplaceKeepoutCommand command;
  command.map = map;
  command.projections.push_back(
    {semantic_path, map.keepout_mask_yaml, map.keepout_mask_pgm});
  command.request_json =
    "{\n"
    "  \"building_id\": \"B1\",\n"
    "  \"floor_id\": \"F1\",\n"
    "  \"map_id\": \"map_canonical_revision\",\n"
    "  \"keepout_lines\": [{\"id\":\"line_1\",\"name\":\"line\","
    "\"width_m\":0.2,\"points\":[{\"x\":1.0,\"y\":1.0},"
    "{\"x\":4.0,\"y\":1.0}]}],\n"
    "  \"keepout_polygons\": [],\n"
    "  \"reload_filter\": false\n"
    "}";

  const KeepoutLayerModule module;
  const auto first = module.replace(command, nullptr);
  command.expected_revision = first.revision;
  command.request_json =
    "{\"reload_filter\":true,\"expected_revision\":\"transport-only\","
    "\"map_id\":\"map_canonical_revision\",\"floor_id\":\"F1\","
    "\"building_id\":\"B1\",\"keepout_polygons\":[],"
    "\"keepout_lines\":[{\"points\":[{\"y\":1,\"x\":1},{\"y\":1,\"x\":4}],"
    "\"width_m\":0.2,\"name\":\"line\",\"id\":\"line_1\"}]}";
  const auto second = module.replace(command, nullptr);
  const auto inspected = module.inspect(map);

  EXPECT_FALSE(second.changed);
  EXPECT_EQ(second.outcome, "NO_CHANGE");
  EXPECT_EQ(second.revision, first.revision);
  EXPECT_EQ(inspected.revision, first.revision);
  EXPECT_TRUE(fs::exists(map.root / "filters" / "keepout_commit.json"));
  const auto semantic = read_text_file(semantic_path);
  EXPECT_NE(semantic.find("\"schema\":\"njrh.keepout.semantic.v1\""), std::string::npos);
  EXPECT_EQ(semantic.find("reload_filter"), std::string::npos);
  EXPECT_EQ(semantic.find("expected_revision"), std::string::npos);
}

}  // namespace
}  // namespace robot_api_server
