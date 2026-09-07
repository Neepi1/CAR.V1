#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/mapping/runtime/mapping_process_runtime.hpp"
#include "robot_api_server/features/mapping/runtime/mapping_start_job.hpp"

namespace mapping_runtime = robot_api_server::features::mapping::runtime;

TEST(MappingProcessClassifier, RecognizesOnlyMappingOwnedLaunchersAndResiduals)
{
  EXPECT_TRUE(mapping_runtime::is_mapping_2d_launcher_command(
      "/bin/bash /workspace/scripts/run_projected_map.sh"));
  EXPECT_TRUE(mapping_runtime::is_mapping_2d_launcher_command(
      "ros2 launch robot_fastlio_mapping jt128_2d_mapping.launch.py"));
  EXPECT_FALSE(mapping_runtime::is_mapping_2d_launcher_command(
      "ros2 launch robot_bringup navigation.launch.py"));

  const std::string private_fastlio =
    "ros2 run fast_lio fastlio_mapping --ros-args -r /Odometry:=/mapping/fastlio_odometry";
  EXPECT_TRUE(mapping_runtime::is_mapping_2d_residual_process_command(
      private_fastlio, "NJRH_SLAM2D_PRIVATE_FASTLIO=1\n"));
  EXPECT_FALSE(mapping_runtime::is_mapping_2d_residual_process_command(
      private_fastlio, "RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"));

  EXPECT_FALSE(mapping_runtime::is_mapping_2d_residual_process_command(
      "ros2 run robot_local_perception nav_cloud_preprocessor", ""));
  EXPECT_FALSE(mapping_runtime::is_mapping_2d_residual_process_command(
      "ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node", ""));
  EXPECT_TRUE(mapping_runtime::is_mapping_2d_residual_process_command(
      "ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node",
      "NJRH_SLAM2D_MAPPING_PIPELINE=1\n"));
}

TEST(MappingProcessClassifier, RpsXpsRestoreInputsStayNarrow)
{
  EXPECT_TRUE(mapping_runtime::is_safe_mapping_lidar_rps_xps_path(
      "/sys/class/net/eth0/queues/rx-0/rps_cpus"));
  EXPECT_TRUE(mapping_runtime::is_safe_mapping_lidar_rps_xps_path(
      "/sys/class/net/eth0/queues/tx-0/xps_cpus"));
  EXPECT_FALSE(mapping_runtime::is_safe_mapping_lidar_rps_xps_path(
      "/tmp/rps_cpus"));
  EXPECT_FALSE(mapping_runtime::is_safe_mapping_lidar_rps_xps_path(
      "/sys/class/net/../etc/queues/rx-0/rps_cpus"));

  EXPECT_TRUE(mapping_runtime::is_safe_mapping_lidar_rps_xps_value("0f,00"));
  EXPECT_FALSE(mapping_runtime::is_safe_mapping_lidar_rps_xps_value("0f;reboot"));
}

TEST(MappingStartJobTracker, PreservesPublicJsonAndSerializesOneRunningJob)
{
  mapping_runtime::MappingStartJobTracker tracker;
  EXPECT_FALSE(tracker.running());
  EXPECT_NE(tracker.json().find("\"state\":\"idle\""), std::string::npos);

  const auto first = tracker.begin(true, "2026-09-01T00:00:00Z");
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(*first, 1U);
  EXPECT_TRUE(tracker.running());
  EXPECT_FALSE(tracker.begin(false, "2026-09-01T00:00:01Z").has_value());

  EXPECT_TRUE(tracker.set_phase(*first, "cancel_navigation", "cancel queued"));
  EXPECT_TRUE(tracker.set_navigation_result(*first, true, false));
  EXPECT_TRUE(tracker.request_cancel("stop requested during startup"));
  EXPECT_TRUE(tracker.cancel_requested(*first));

  const auto running_json = tracker.json();
  EXPECT_NE(running_json.find("\"navigation_was_active\":true"), std::string::npos);
  EXPECT_NE(running_json.find("\"navigation_cancel_ok\":true"), std::string::npos);
  EXPECT_NE(running_json.find("\"cancel_requested\":true"), std::string::npos);
  EXPECT_NE(running_json.find("stop requested during startup"), std::string::npos);

  EXPECT_TRUE(tracker.finish(
      *first, "canceled", "2D mapping start canceled", -1,
      "2026-09-01T00:00:02Z"));
  EXPECT_FALSE(tracker.running());
  EXPECT_FALSE(tracker.request_cancel("late cancel"));
  EXPECT_FALSE(tracker.set_phase(*first, "stale", "must not apply"));
  EXPECT_FALSE(tracker.finish(
      *first + 1U, "failed", "stale finish", -1,
      "2026-09-01T00:00:03Z"));

  const auto terminal = tracker.snapshot();
  EXPECT_EQ(terminal.state, "canceled");
  EXPECT_EQ(terminal.phase, "finished");
  EXPECT_EQ(terminal.detail, "2D mapping start canceled");
  EXPECT_EQ(terminal.finished_at, "2026-09-01T00:00:02Z");

  const auto second = tracker.begin(false, "2026-09-01T00:00:04Z");
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(*second, 2U);
  EXPECT_FALSE(tracker.snapshot().navigation_was_active);
}
