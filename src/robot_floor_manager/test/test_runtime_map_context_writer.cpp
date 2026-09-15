#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "robot_floor_manager/runtime_map_context_writer.hpp"

namespace robot_floor_manager
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path_ = fs::temp_directory_path() /
      ("robot_floor_context_test_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  const fs::path & path() const {return path_;}

private:
  fs::path path_;
};

std::string read_file(const fs::path & path)
{
  std::ifstream input(path);
  std::ostringstream content;
  content << input.rdbuf();
  return content.str();
}

RuntimeMapContextRecord record(const bool confirmed)
{
  RuntimeMapContextRecord value;
  value.state = confirmed ? "ready" : "floor_switch_pending";
  value.confirmed = confirmed;
  value.message = confirmed ? "target committed" : "source invalidated";
  value.transaction_id = "floor-live-1";
  value.building_id = "B11";
  value.floor_id = "F2";
  value.map_id = "map-f2";
  value.asset_epoch = 42U;
  value.asset_digest =
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  value.localizer_generation = 8U;
  value.explicit_relocalization_sequence = 12U;
  value.updated_at_sec = 1234.5;
  return value;
}

TEST(RuntimeMapContextWriter, AtomicallyPublishesExactCommittedIdentity)
{
  TemporaryDirectory temporary;
  const auto target = temporary.path() / "runtime.json";
  std::string error;

  ASSERT_TRUE(
    AtomicRuntimeMapContextWriter{}.write(target, record(false), error))
    << error;
  ASSERT_TRUE(
    AtomicRuntimeMapContextWriter{}.write(target, record(true), error))
    << error;

  const auto body = read_file(target);
  EXPECT_NE(body.find("\"schema\":\"njrh.runtime_map_context.v1\""), std::string::npos);
  EXPECT_NE(body.find("\"state\":\"ready\""), std::string::npos);
  EXPECT_NE(body.find("\"confirmed\":true"), std::string::npos);
  EXPECT_NE(body.find("\"transaction_id\":\"floor-live-1\""), std::string::npos);
  EXPECT_NE(body.find("\"building_id\":\"B11\""), std::string::npos);
  EXPECT_NE(body.find("\"floor_id\":\"F2\""), std::string::npos);
  EXPECT_NE(body.find("\"map_id\":\"map-f2\""), std::string::npos);
  EXPECT_NE(body.find("\"asset_epoch\":42"), std::string::npos);
  EXPECT_NE(body.find("\"localizer_generation\":8"), std::string::npos);
  EXPECT_NE(
    body.find("\"explicit_relocalization_sequence\":12"),
    std::string::npos);
  EXPECT_EQ(body.back(), '\n');

  for (const auto & entry : fs::directory_iterator(temporary.path())) {
    EXPECT_EQ(entry.path().filename(), "runtime.json");
  }
}

TEST(RuntimeMapContextWriter, RefusesInvalidIdentityWithoutReplacingPriorFile)
{
  TemporaryDirectory temporary;
  const auto target = temporary.path() / "runtime.json";
  std::string error;
  ASSERT_TRUE(
    AtomicRuntimeMapContextWriter{}.write(target, record(true), error))
    << error;
  const auto original = read_file(target);

  auto invalid = record(false);
  invalid.asset_epoch = 0U;
  EXPECT_FALSE(
    AtomicRuntimeMapContextWriter{}.write(target, invalid, error));
  EXPECT_EQ(read_file(target), original);
}

RuntimeMapContextRecord handoff_record(const fs::path & directory)
{
  auto value = record(false);
  value.startup_handoff = true;
  value.state = "requested";
  value.request_nonce = "floor_123";
  value.explicit_sequence_baseline = 11U;
  const auto root = directory / "B11/F2/maps/map-f2";
  value.asset_root = root.string();
  value.nav_map_yaml = (root / "nav/map.yaml").string();
  value.localizer_map_png = (root / "localizer/map.png").string();
  value.localizer_params_yaml = (root / "localizer/map.yaml").string();
  value.keepout_mask_yaml = (root / "filters/keepout_mask.yaml").string();
  value.speed_mask_yaml = (root / "filters/speed_mask.yaml").string();
  return value;
}

TEST(FloorStartupHandoff, SerializesExactRequestAndRejectsEscapingAsset)
{
  TemporaryDirectory temporary;
  const auto target = temporary.path() / "request.json";
  auto value = handoff_record(temporary.path());
  std::string error;
  ASSERT_TRUE(AtomicRuntimeMapContextWriter{}.write(target, value, error)) << error;
  const auto body = read_file(target);
  EXPECT_NE(body.find("\"schema\":\"njrh.floor_startup_handoff.v1\""), std::string::npos);
  EXPECT_NE(body.find("\"request_nonce\":\"floor_123\""), std::string::npos);
  EXPECT_NE(body.find("\"explicit_sequence_baseline\":11"), std::string::npos);
  EXPECT_NE(body.find("\"asset_digest\":\"sha256:"), std::string::npos);
  value.nav_map_yaml = (temporary.path() / "other.yaml").string();
  EXPECT_FALSE(AtomicRuntimeMapContextWriter{}.write(target, value, error));
  EXPECT_EQ(read_file(target), body);
}

std::string ack_body(const RuntimeMapContextRecord & expected)
{
  return "{\"schema\":\"njrh.floor_startup_handoff_ack.v1\",\"version\":1,"
    "\"transaction_id\":\"" + expected.transaction_id + "\","
    "\"request_nonce\":\"" + expected.request_nonce + "\","
    "\"building_id\":\"B11\",\"floor_id\":\"F2\",\"map_id\":\"map-f2\","
    "\"asset_epoch\":42,\"asset_digest\":\"" + expected.asset_digest + "\","
    "\"state\":\"runtime_ready\",\"failure\":\"\",\"detail\":\"ok\","
    "\"explicit_relocalization_sequence\":12,\"localizer_generation\":8}";
}

TEST(FloorStartupHandoff, AckRequiresFreshExactRequestAndSupportedSchema)
{
  TemporaryDirectory temporary;
  const auto path = temporary.path() / "ack.json";
  auto expected = handoff_record(temporary.path());
  const auto valid = ack_body(expected);
  auto write = [&](const std::string & text) {std::ofstream(path) << text;};
  write(valid);
  const auto accepted = read_floor_startup_handoff_ack(path, expected);
  ASSERT_TRUE(accepted.has_value());
  EXPECT_EQ(accepted->explicit_relocalization_sequence, 12U);
  EXPECT_EQ(accepted->localizer_generation, 8U);

  expected.request_nonce = "floor_other";
  EXPECT_FALSE(read_floor_startup_handoff_ack(path, expected));
  expected = handoff_record(temporary.path());
  expected.explicit_sequence_baseline = 12U;
  EXPECT_FALSE(read_floor_startup_handoff_ack(path, expected));
  expected = handoff_record(temporary.path());
  for (const auto & change : std::vector<std::pair<std::string, std::string>>{
      {"njrh.floor_startup_handoff_ack.v1", "other.schema"},
      {"\"version\":1", "\"version\":2"},
      {"\"localizer_generation\":8", "\"localizer_generation\":0"},
      {"\"asset_epoch\":42", "\"asset_epoch\":41"},
      {"\"map_id\":\"map-f2\"", "\"map_id\":\"other-map\""},
      {"\"version\":1", "\"version\":1,\"version\":1"}})
  {
    auto invalid = valid;
    invalid.replace(invalid.find(change.first), change.first.size(), change.second);
    write(invalid);
    EXPECT_FALSE(read_floor_startup_handoff_ack(path, expected)) << invalid;
  }
  write("{broken");
  EXPECT_FALSE(read_floor_startup_handoff_ack(path, expected));
}

}  // namespace
}  // namespace robot_floor_manager
