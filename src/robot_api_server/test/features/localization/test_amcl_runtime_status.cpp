#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "gtest/gtest.h"

#include "robot_api_server/features/localization/amcl_runtime_status.hpp"

namespace robot_api_server::features::localization
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
      ("njrh_amcl_runtime_status_test_" + std::to_string(
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

void write_text(const fs::path & path, const std::string & content)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output << content;
  ASSERT_TRUE(output.good());
}

TEST(AmclRuntimeStatus, ParsesFreshWriterSnapshot)
{
  TemporaryDirectory temporary;
  const auto status_file = temporary.path() / "amcl.env";
  write_text(
    status_file,
    "AMCL_STATUS_STAMP_SEC=\"100.0\"\n"
    "AMCL_MODE=\"gated\"\n"
    "AMCL_STATE=\"AMCL_READY\"\n"
    "AMCL_START_RESULT=\"ready\"\n"
    "AMCL_READY=\"true\"\n"
    "AMCL_DEGRADED=\"false\"\n"
    "AMCL_FAILURE_REASON=\"\"\n"
    "AMCL_PID_ALIVE=\"true\"\n"
    "SCAN_ADMISSION_ALIVE=\"true\"\n"
    "AMCL_POSE_PUBLISHER_COUNT=\"1\"\n"
    "SCAN_ADMISSION_STATUS_PUBLISHER_COUNT=\"1\"\n"
    "AMCL_SEED_SUCCEEDED=\"true\"\n"
    "AMCL_SEED_RESPONSE_OK=\"false\"\n"
    "AMCL_NOMOTION_PROBE_USED=\"true\"\n"
    "AMCL_NOMOTION_POSE_RECEIVED=\"true\"\n"
    "AMCL_NOMOTION_POSE_COUNT=\"2\"\n"
    "AMCL_NOMOTION_POSE_HEADER_AGE_MS=\"12.5\"\n"
    "AMCL_PROCESS_READY=\"true\"\n"
    "AMCL_STATIC_STANDBY=\"true\"\n"
    "AMCL_TRACKING_READY=\"true\"\n"
    "AMCL_CORRECTION_READY=\"false\"\n"
    "AMCL_NOT_MOVING_NO_UPDATE_OK=\"true\"\n"
    "TIMESTAMP=\"2026-08-31T00:00:00+00:00\"\n");

  const auto status = read_amcl_runtime_status(status_file, 102.0, 5.0);

  EXPECT_TRUE(status.available);
  EXPECT_EQ(status.mode, "gated");
  EXPECT_EQ(status.state, "AMCL_READY");
  EXPECT_EQ(status.start_result, "ready");
  EXPECT_TRUE(status.ready);
  EXPECT_FALSE(status.degraded);
  EXPECT_TRUE(status.process_alive);
  EXPECT_TRUE(status.scan_admission_alive);
  EXPECT_EQ(status.pose_publisher_count, 1);
  EXPECT_EQ(status.scan_admission_status_publisher_count, 1);
  EXPECT_TRUE(status.seed_succeeded);
  EXPECT_FALSE(status.seed_response_ok);
  EXPECT_TRUE(status.seeded);
  EXPECT_TRUE(status.nomotion_probe_used);
  EXPECT_TRUE(status.nomotion_pose_received);
  EXPECT_EQ(status.nomotion_pose_count, 2);
  EXPECT_DOUBLE_EQ(status.nomotion_pose_header_age_ms, 12.5);
  EXPECT_TRUE(status.process_ready);
  EXPECT_TRUE(status.static_standby);
  EXPECT_TRUE(status.tracking_ready);
  EXPECT_FALSE(status.correction_ready);
  EXPECT_TRUE(status.not_moving_no_update_ok);
  EXPECT_EQ(status.stamp, "2026-08-31T00:00:00+00:00");
  EXPECT_DOUBLE_EQ(status.stamp_sec, 100.0);
  EXPECT_DOUBLE_EQ(status.age_ms, 2000.0);
  EXPECT_FALSE(status.stale);
}

TEST(AmclRuntimeStatus, MissingAndReadableEmptyFilesKeepDistinctAvailability)
{
  TemporaryDirectory temporary;
  const auto missing_file = temporary.path() / "missing.env";
  const auto missing = read_amcl_runtime_status(missing_file, 100.0, 5.0);

  EXPECT_FALSE(missing.available);
  EXPECT_TRUE(missing.stale);
  EXPECT_DOUBLE_EQ(missing.age_ms, -1.0);

  const auto empty_file = temporary.path() / "empty.env";
  write_text(empty_file, "# heartbeat has not populated fields yet\n");
  const auto empty = read_amcl_runtime_status(empty_file, 100.0, 5.0);

  EXPECT_TRUE(empty.available);
  EXPECT_TRUE(empty.stale);
  EXPECT_DOUBLE_EQ(empty.age_ms, -1.0);
}

TEST(AmclRuntimeStatus, TtlBoundaryAndFutureStampMatchFreshnessContract)
{
  TemporaryDirectory temporary;
  const auto status_file = temporary.path() / "amcl.env";
  write_text(status_file, "AMCL_STATUS_STAMP_SEC=100.0\n");

  const auto boundary = read_amcl_runtime_status(status_file, 105.0, 5.0);
  EXPECT_DOUBLE_EQ(boundary.age_ms, 5000.0);
  EXPECT_FALSE(boundary.stale);

  const auto expired = read_amcl_runtime_status(status_file, 105.001, 5.0);
  EXPECT_NEAR(expired.age_ms, 5001.0, 1.0e-6);
  EXPECT_TRUE(expired.stale);

  const auto future = read_amcl_runtime_status(status_file, 99.0, 5.0);
  EXPECT_DOUBLE_EQ(future.age_ms, 0.0);
  EXPECT_FALSE(future.stale);
}

TEST(AmclRuntimeStatus, LastKeyEscapingAndInvalidFieldsMatchLegacyParser)
{
  TemporaryDirectory temporary;
  const auto status_file = temporary.path() / "amcl.env";
  write_text(
    status_file,
    "  # ignored comment\n"
    "AMCL_MODE=shadow\n"
    "AMCL_MODE=\"gated\"\n"
    "AMCL_READY=TRUE\n"
    "AMCL_DEGRADED=True\n"
    "AMCL_FAILURE_REASON=\"quote: \\\"x\\\" path: C:\\\\maps\"\n"
    "AMCL_POSE_PUBLISHER_COUNT=invalid\n"
    "AMCL_NOMOTION_POSE_COUNT=2samples\n"
    "AMCL_NOMOTION_POSE_HEADER_AGE_MS=invalid\n"
    "AMCL_SEEDED=false\n"
    "AMCL_SEED_RESPONSE_OK=1\n"
    "AMCL_STATUS_STAMP_SEC=invalid\n");

  const auto status = read_amcl_runtime_status(status_file, 100.0, 5.0);

  EXPECT_TRUE(status.available);
  EXPECT_EQ(status.mode, "gated");
  EXPECT_FALSE(status.ready);
  EXPECT_TRUE(status.degraded);
  EXPECT_EQ(status.degraded_reason, "quote: \"x\" path: C:\\maps");
  EXPECT_EQ(status.pose_publisher_count, 0);
  EXPECT_EQ(status.nomotion_pose_count, 2);
  EXPECT_DOUBLE_EQ(status.nomotion_pose_header_age_ms, -1.0);
  EXPECT_TRUE(status.seed_response_ok);
  EXPECT_TRUE(status.seeded);
  EXPECT_DOUBLE_EQ(status.stamp_sec, -1.0);
  EXPECT_DOUBLE_EQ(status.age_ms, -1.0);
  EXPECT_TRUE(status.stale);
}

}  // namespace
}  // namespace robot_api_server::features::localization
