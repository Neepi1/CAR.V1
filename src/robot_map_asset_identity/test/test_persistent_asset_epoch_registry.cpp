#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_map_asset_identity/map_asset_identity.hpp"

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace robot_map_asset_identity
{
namespace
{

constexpr char kDigestA[] =
  "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
  "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
constexpr char kDigestB[] =
  "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
  "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr std::size_t kMaximumRegistryFileSize = 32U * 1024U * 1024U;
constexpr std::size_t kMaximumBindingCount = 100000U;
constexpr char kBindingsHeader[] =
  "{\"schema\":\"njrh.map_asset_bindings.v1\",\"bindings\":[";
constexpr char kBindingsFooter[] = "]}";

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto seed = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("robot_map_asset_identity_test_" + std::to_string(seed));
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept {return path_;}

private:
  std::filesystem::path path_;
};

void write_binary(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output.write(content.data(), static_cast<std::streamsize>(content.size()));
  ASSERT_TRUE(output.good());
}

std::string read_binary(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to read test file: " + path.string());
  }
  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
}

std::string binding_record(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & map_id,
  const std::uint64_t epoch,
  const std::string & digest = kDigestA)
{
  return "{\"building_id\":\"" + building_id +
         "\",\"floor_id\":\"" + floor_id +
         "\",\"map_id\":\"" + map_id +
         "\",\"asset_epoch\":\"" + std::to_string(epoch) +
         "\",\"asset_digest\":\"" + digest + "\"}";
}

std::string fixed_width_identifier(
  const char prefix,
  const std::uint64_t index)
{
  auto value = std::string(1U, prefix) + std::to_string(index);
  value.append(128U - value.size(), 'x');
  return value;
}

template<typename RecordFactory>
void write_bindings_file(
  const std::filesystem::path & path,
  const std::size_t count,
  RecordFactory && record_factory)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output << kBindingsHeader << '\n';
  for (std::size_t index = 0U; index < count; ++index) {
    output << record_factory(index);
    if (index + 1U < count) {
      output << ',';
    }
    output << '\n';
  }
  output << kBindingsFooter << '\n';
  ASSERT_TRUE(output.good());
}

TEST(PersistentAssetEpochRegistry, BindsIdempotentlyAndIncrementsGlobally)
{
  TemporaryDirectory temporary;
  const AssetIdentityKey first{"building_a", "floor_1", "map_main"};
  const AssetIdentityKey second{"building_a", "floor_2", "map_main"};

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_EQ(registry.bind(first, kDigestA), (AssetIdentity{1U, kDigestA}));
  EXPECT_EQ(registry.bind(first, kDigestA), (AssetIdentity{1U, kDigestA}));
  EXPECT_EQ(registry.bind(first, kDigestB), (AssetIdentity{2U, kDigestB}));
  EXPECT_EQ(registry.bind(second, kDigestA), (AssetIdentity{3U, kDigestA}));
  EXPECT_EQ(registry.bind(first, kDigestA), (AssetIdentity{4U, kDigestA}));

  PersistentAssetEpochRegistry reopened(temporary.path());
  ASSERT_TRUE(reopened.lookup(first).has_value());
  EXPECT_EQ(*reopened.lookup(first), (AssetIdentity{4U, kDigestA}));
  EXPECT_EQ(*reopened.lookup(second), (AssetIdentity{3U, kDigestA}));
}

TEST(PersistentAssetEpochRegistry, RecoversHighWatermarkFromBindingsWhenStateIsMissing)
{
  TemporaryDirectory temporary;
  const AssetIdentityKey first{"building_a", "floor_1", "map_main"};
  const AssetIdentityKey second{"building_a", "floor_2", "map_main"};

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_EQ(registry.bind(first, kDigestA).asset_epoch, 1U);
  EXPECT_EQ(registry.bind(second, kDigestA).asset_epoch, 2U);

  std::filesystem::remove(
    temporary.path() / ".map_asset_registry" / "epoch_state.json");

  PersistentAssetEpochRegistry recovered(temporary.path());
  EXPECT_EQ(recovered.bind(second, kDigestA).asset_epoch, 2U);
  EXPECT_TRUE(std::filesystem::is_regular_file(
      temporary.path() / ".map_asset_registry" / "epoch_state.json"));
  EXPECT_EQ(
    recovered.bind(
      AssetIdentityKey{"building_b", "floor_1", "map_main"}, kDigestB).asset_epoch,
    3U);
}

TEST(PersistentAssetEpochRegistry, RejectsBadInputsAndDoesNotCreateEpochZero)
{
  TemporaryDirectory temporary;
  PersistentAssetEpochRegistry registry(temporary.path());

  EXPECT_THROW(
    registry.bind(AssetIdentityKey{"", "floor_1", "map_main"}, kDigestA),
    std::invalid_argument);
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "../floor_1", "map_main"}, kDigestA),
    std::invalid_argument);
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}, "sha256:bad"),
    std::invalid_argument);
  EXPECT_THROW(
    registry.lookup(AssetIdentityKey{"building_a", "floor/1", "map_main"}),
    std::invalid_argument);
}

TEST(PersistentAssetEpochRegistry, LookupIsStrictlyReadOnlyWhenRegistryIsMissing)
{
  TemporaryDirectory temporary;
  PersistentAssetEpochRegistry registry(temporary.path());

  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(
      temporary.path() / ".map_asset_registry"));

  const auto empty_registry = temporary.path() / ".map_asset_registry";
  std::filesystem::create_directory(empty_registry);
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);
  EXPECT_TRUE(std::filesystem::is_empty(empty_registry));
}

TEST(PersistentAssetEpochRegistry, FailsClosedOnCorruptOrRegressedState)
{
  TemporaryDirectory temporary;
  const auto registry_root = temporary.path() / ".map_asset_registry";
  PersistentAssetEpochRegistry registry(temporary.path());
  registry.bind(AssetIdentityKey{"building_a", "floor_1", "map_main"}, kDigestA);

  write_binary(registry_root / "epoch_state.json", "not-json\n");
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_2", "map_main"}, kDigestA),
    std::runtime_error);

  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"1\"}\n");
  write_binary(
    registry_root / "bindings.json",
    "{\"schema\":\"njrh.map_asset_bindings.v1\",\"bindings\":[\n"
    "{\"building_id\":\"building_a\",\"floor_id\":\"floor_1\","
    "\"map_id\":\"map_main\",\"asset_epoch\":\"2\","
    "\"asset_digest\":\"" + std::string(kDigestA) + "\"}\n]}\n");
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);

  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"0\"}\n");
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_2", "map_main"}, kDigestA),
    std::runtime_error);

  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"18446744073709551615\"}\n");
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_2", "map_main"}, kDigestA),
    std::overflow_error);
}

TEST(PersistentAssetEpochRegistry, RefusesEpochReuseAfterRegistryLoss)
{
  TemporaryDirectory temporary;
  const AssetIdentityKey first{"building_a", "floor_1", "map_main"};
  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_EQ(registry.bind(first, kDigestA).asset_epoch, 1U);

  const auto map_root =
    temporary.path() / "building_a" / "floor_1" / "maps" / "map_main";
  std::filesystem::create_directories(map_root);
  write_binary(
    map_root / "manifest.json",
    "{\"schema\":\"njrh.map_manifest.v2\",\"asset_epoch\":1}\n");
  std::filesystem::remove_all(
    temporary.path() / ".map_asset_registry");

  PersistentAssetEpochRegistry reopened(temporary.path());
  EXPECT_THROW(
    reopened.bind(
      AssetIdentityKey{"building_a", "floor_2", "map_main"}, kDigestB),
    std::runtime_error);
}

TEST(PersistentAssetEpochRegistry, RefusesNonEmptyRegistryRollbackBelowManifest)
{
  TemporaryDirectory temporary;
  const AssetIdentityKey first{"building_a", "floor_1", "map_main"};
  const AssetIdentityKey second{"building_a", "floor_2", "map_main"};
  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_EQ(registry.bind(first, kDigestA).asset_epoch, 1U);

  const auto registry_root = temporary.path() / ".map_asset_registry";
  const auto old_state = read_binary(registry_root / "epoch_state.json");
  const auto old_bindings = read_binary(registry_root / "bindings.json");
  EXPECT_EQ(registry.bind(second, kDigestB).asset_epoch, 2U);

  const auto first_map_root =
    temporary.path() / "building_a" / "floor_1" / "maps" / "map_main";
  std::filesystem::create_directories(first_map_root);
  write_binary(
    first_map_root / "manifest.json",
    "{\"schema\":\"njrh.map_manifest.v2\",\"asset_epoch\":1}\n");
  const auto second_map_root =
    temporary.path() / "building_a" / "floor_2" / "maps" / "map_main";
  std::filesystem::create_directories(second_map_root);
  write_binary(
    second_map_root / "manifest.json",
    "{\n"
    "  \"schema\": \"njrh.map_manifest.v2\",\n"
    "  \"asset_epoch\": 2,\n"
    "  \"display_name\": \"floor two\"\n"
    "}\n");

  write_binary(registry_root / "epoch_state.json", old_state);
  write_binary(registry_root / "bindings.json", old_bindings);

  EXPECT_THROW(registry.bind(first, kDigestA), std::runtime_error);
  EXPECT_THROW(registry.lookup(first), std::runtime_error);
  EXPECT_EQ(read_binary(registry_root / "epoch_state.json"), old_state);
  EXPECT_EQ(read_binary(registry_root / "bindings.json"), old_bindings);
}

TEST(PersistentAssetEpochRegistry, FailsClosedOnMalformedV2Manifest)
{
  TemporaryDirectory temporary;
  const auto map_root =
    temporary.path() / "building_a" / "floor_1" / "maps" / "map_main";
  std::filesystem::create_directories(map_root);
  write_binary(
    map_root / "manifest.json",
    "{\"schema\":\"njrh.map_manifest.v2\","
    "\"asset_epoch\":1,\"asset_\\u0065poch\":2}\n");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}, kDigestA),
    std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(
      temporary.path() / ".map_asset_registry" / "epoch_state.json"));
  EXPECT_FALSE(std::filesystem::exists(
      temporary.path() / ".map_asset_registry" / "bindings.json"));
}

TEST(PersistentAssetEpochRegistry, RefusesBindingCountBeyondReadableLimit)
{
  TemporaryDirectory temporary;
  const auto registry_root = temporary.path() / ".map_asset_registry";
  std::filesystem::create_directories(registry_root);
  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"100000\"}\n");
  write_bindings_file(
    registry_root / "bindings.json",
    kMaximumBindingCount,
    [](const std::size_t index) {
      return binding_record(
        "b", "f" + std::to_string(index), "m", index + 1U);
    });
  ASSERT_LE(
    std::filesystem::file_size(registry_root / "bindings.json"),
    kMaximumRegistryFileSize);
  const auto state_before =
    read_binary(registry_root / "epoch_state.json");
  const auto bindings_size_before =
    std::filesystem::file_size(registry_root / "bindings.json");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_new", "floor_new", "map_new"}, kDigestB),
    std::runtime_error);
  EXPECT_EQ(read_binary(registry_root / "epoch_state.json"), state_before);
  EXPECT_EQ(
    std::filesystem::file_size(registry_root / "bindings.json"),
    bindings_size_before);
}

TEST(PersistentAssetEpochRegistry, RefusesSerializedBindingsBeyondReadableLimit)
{
  TemporaryDirectory temporary;
  std::vector<std::string> records;
  std::size_t serialized_size =
    std::string(kBindingsHeader).size() + 1U +
    std::string(kBindingsFooter).size() + 1U;
  while (records.size() < kMaximumBindingCount) {
    const auto index = records.size();
    auto record = binding_record(
      fixed_width_identifier('b', 0U),
      fixed_width_identifier('f', index),
      fixed_width_identifier('m', 0U),
      index + 1U);
    const auto added_size = record.size() + 1U +
      (records.empty() ? 0U : 1U);
    if (added_size > kMaximumRegistryFileSize - serialized_size) {
      break;
    }
    serialized_size += added_size;
    records.emplace_back(std::move(record));
  }
  ASSERT_FALSE(records.empty());
  ASSERT_LT(records.size(), kMaximumBindingCount);

  const auto proposed_record = binding_record(
    std::string(128U, 'z'),
    std::string(128U, 'y'),
    std::string(128U, 'x'),
    records.size() + 1U,
    kDigestB);
  ASSERT_GT(
    serialized_size + proposed_record.size() + 2U,
    kMaximumRegistryFileSize);

  const auto registry_root = temporary.path() / ".map_asset_registry";
  std::filesystem::create_directories(registry_root);
  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"" + std::to_string(records.size()) + "\"}\n");
  write_bindings_file(
    registry_root / "bindings.json",
    records.size(),
    [&records](const std::size_t index) {
      return records[index];
    });
  ASSERT_EQ(
    std::filesystem::file_size(registry_root / "bindings.json"),
    serialized_size);
  const auto state_before =
    read_binary(registry_root / "epoch_state.json");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{
        std::string(128U, 'z'),
        std::string(128U, 'y'),
        std::string(128U, 'x')},
      kDigestB),
    std::runtime_error);
  EXPECT_EQ(read_binary(registry_root / "epoch_state.json"), state_before);
  EXPECT_EQ(
    std::filesystem::file_size(registry_root / "bindings.json"),
    serialized_size);
}

TEST(PersistentAssetEpochRegistry, RejectsDuplicateKeysAndEpochsInBindings)
{
  TemporaryDirectory temporary;
  const auto registry_root = temporary.path() / ".map_asset_registry";
  std::filesystem::create_directories(registry_root);
  write_binary(
    registry_root / "epoch_state.json",
    "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
    "\"high_watermark\":\"2\"}\n");
  const std::string record =
    "{\"building_id\":\"building_a\",\"floor_id\":\"floor_1\","
    "\"map_id\":\"map_main\",\"asset_epoch\":\"1\","
    "\"asset_digest\":\"" + std::string(kDigestA) + "\"}";
  write_binary(
    registry_root / "bindings.json",
    "{\"schema\":\"njrh.map_asset_bindings.v1\",\"bindings\":[\n" +
    record + ",\n" + record + "\n]}\n");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);

  write_binary(
    registry_root / "bindings.json",
    "{\"schema\":\"njrh.map_asset_bindings.v1\",\"bindings\":[\n" +
    record + ",\n"
    "{\"building_id\":\"building_a\",\"floor_id\":\"floor_2\","
    "\"map_id\":\"map_main\",\"asset_epoch\":\"1\","
    "\"asset_digest\":\"" + std::string(kDigestB) + "\"}\n]}\n");
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);
}

TEST(PersistentAssetEpochRegistry, AuditIgnoresUnrelatedManifestFiles)
{
  TemporaryDirectory temporary;
  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_EQ(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}, kDigestA)
    .asset_epoch,
    1U);

  const auto elevator_release =
    temporary.path() / "building_a" / ".elevator_config" /
    "releases" / "release_1";
  const auto current_projection =
    temporary.path() / "building_a" / "floor_1" / "current";
  std::filesystem::create_directories(elevator_release);
  std::filesystem::create_directories(current_projection);
  write_binary(elevator_release / "manifest.json", "{not-json");
  write_binary(current_projection / "manifest.json", "{also-not-json");

  EXPECT_EQ(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_2", "map_next"}, kDigestB)
    .asset_epoch,
    2U);
}

#ifndef _WIN32
TEST(PersistentAssetEpochRegistry, LookupUsesReadOnlySharedRegistryLock)
{
  TemporaryDirectory temporary;
  const AssetIdentityKey key{"building_a", "floor_1", "map_main"};
  PersistentAssetEpochRegistry registry(temporary.path());
  const auto expected = registry.bind(key, kDigestA);
  const auto registry_root = temporary.path() / ".map_asset_registry";
  const auto lock_path = registry_root / "registry.lock";
  const auto state_path = registry_root / "epoch_state.json";
  const auto bindings_path = registry_root / "bindings.json";

  ASSERT_EQ(::chmod(lock_path.c_str(), 0400), 0);
  ASSERT_EQ(::chmod(state_path.c_str(), 0400), 0);
  ASSERT_EQ(::chmod(bindings_path.c_str(), 0400), 0);
  ASSERT_EQ(::chmod(registry_root.c_str(), 0500), 0);
  try {
    EXPECT_EQ(registry.lookup(key), std::optional<AssetIdentity>{expected});
  } catch (...) {
    (void)::chmod(registry_root.c_str(), 0700);
    (void)::chmod(lock_path.c_str(), 0600);
    (void)::chmod(state_path.c_str(), 0600);
    (void)::chmod(bindings_path.c_str(), 0600);
    throw;
  }
  EXPECT_EQ(::chmod(registry_root.c_str(), 0700), 0);
  EXPECT_EQ(::chmod(lock_path.c_str(), 0600), 0);
  EXPECT_EQ(::chmod(state_path.c_str(), 0600), 0);
  EXPECT_EQ(::chmod(bindings_path.c_str(), 0600), 0);
}

TEST(PersistentAssetEpochRegistry, FailsClosedOnSymlinkManifestDuringAudit)
{
  TemporaryDirectory temporary;
  const auto map_root =
    temporary.path() / "building_a" / "floor_1" / "maps" / "map_main";
  std::filesystem::create_directories(map_root);
  const auto outside = temporary.path() / "outside_manifest.json";
  write_binary(
    outside,
    "{\"schema\":\"njrh.map_manifest.v2\",\"asset_epoch\":9}\n");
  std::filesystem::create_symlink(outside, map_root / "manifest.json");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.bind(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}, kDigestA),
    std::runtime_error);
}

TEST(PersistentAssetEpochRegistry, RejectsSymlinkAndNonRegularRegistryPaths)
{
  TemporaryDirectory temporary;
  const auto registry_root = temporary.path() / ".map_asset_registry";
  std::filesystem::create_directories(registry_root);
  const auto outside = temporary.path() / "outside";
  write_binary(outside, "outside");
  std::filesystem::create_symlink(outside, registry_root / "epoch_state.json");

  PersistentAssetEpochRegistry registry(temporary.path());
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);

  std::filesystem::remove(registry_root / "epoch_state.json");
  std::filesystem::create_directory(registry_root / "bindings.json");
  EXPECT_THROW(
    registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);

  TemporaryDirectory root_symlink_temporary;
  const auto outside_directory =
    root_symlink_temporary.path() / "outside_registry";
  std::filesystem::create_directory(outside_directory);
  std::filesystem::create_directory_symlink(
    outside_directory,
    root_symlink_temporary.path() / ".map_asset_registry");
  PersistentAssetEpochRegistry root_symlink_registry(
    root_symlink_temporary.path());
  EXPECT_THROW(
    root_symlink_registry.lookup(
      AssetIdentityKey{"building_a", "floor_1", "map_main"}),
    std::runtime_error);
}

TEST(PersistentAssetEpochRegistry, FlockSerializesConcurrentProcesses)
{
  TemporaryDirectory temporary;
  constexpr int kProcessCount = 6;
  int start_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(start_pipe), 0);

  std::vector<pid_t> children;
  children.reserve(kProcessCount);
  for (int index = 0; index < kProcessCount; ++index) {
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
      (void)::close(start_pipe[1]);
      char token = '\0';
      if (::read(start_pipe[0], &token, 1U) != 1) {
        ::_exit(10);
      }
      try {
        PersistentAssetEpochRegistry registry(temporary.path());
        const auto identity = registry.bind(
          AssetIdentityKey{
            "building_a",
            "floor_" + std::to_string(index),
            "map_main"},
          index % 2 == 0 ? kDigestA : kDigestB);
        ::_exit(identity.asset_epoch == 0U ? 11 : 0);
      } catch (...) {
        ::_exit(12);
      }
    }
    children.push_back(child);
  }

  ASSERT_EQ(::close(start_pipe[0]), 0);
  for (int index = 0; index < kProcessCount; ++index) {
    ASSERT_EQ(::write(start_pipe[1], "x", 1U), 1);
  }
  ASSERT_EQ(::close(start_pipe[1]), 0);

  for (const auto child : children) {
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
  }

  PersistentAssetEpochRegistry registry(temporary.path());
  std::set<std::uint64_t> epochs;
  for (int index = 0; index < kProcessCount; ++index) {
    const auto identity = registry.lookup(
      AssetIdentityKey{
        "building_a",
        "floor_" + std::to_string(index),
        "map_main"});
    ASSERT_TRUE(identity.has_value());
    epochs.insert(identity->asset_epoch);
  }
  EXPECT_EQ(epochs.size(), static_cast<std::size_t>(kProcessCount));
  EXPECT_EQ(*epochs.begin(), 1U);
  EXPECT_EQ(*epochs.rbegin(), static_cast<std::uint64_t>(kProcessCount));
}
#endif

}  // namespace
}  // namespace robot_map_asset_identity
