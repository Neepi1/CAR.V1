#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

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

}  // namespace
}  // namespace robot_floor_manager
