#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/runtime/navigation_process_runtime.hpp"

namespace navigation = robot_api_server::features::navigation;
using namespace std::chrono_literals;

TEST(NavigationProcessRuntime, RejectsUnavailableCommandsWithoutChangingOwnership)
{
  navigation::NavigationProcessRuntimeConfig config;
  config.resume_command = "/definitely/not/a/navigation/resume/script";
  config.stop_command = "/definitely/not/a/navigation/stop/script";
  navigation::NavigationProcessRuntime runtime(config);

  navigation::NavigationProcessLaunchSpec launch;
  launch.building_id = "B1";
  launch.floor_id = "F1";
  pid_t pid = -1;
  std::string detail;
  EXPECT_FALSE(runtime.launch_replacing(launch, pid, detail));
  EXPECT_EQ(pid, -1);
  EXPECT_FALSE(runtime.running());
  EXPECT_FALSE(runtime.stop_stack(detail));
}

TEST(NavigationProcessRuntime, ReapsACompletedManagedResumeProcess)
{
  navigation::NavigationProcessRuntimeConfig config;
  config.resume_command = "/bin/true";
  config.resume_log_file = "/tmp/navigation_process_runtime_test.log";
  config.stop_command = "/bin/true";
  config.stop_log_file = "/tmp/navigation_process_runtime_stop_test.log";
  navigation::NavigationProcessRuntime runtime(config);

  navigation::NavigationProcessLaunchSpec launch;
  launch.building_id = "B1";
  launch.floor_id = "F1";
  pid_t pid = -1;
  std::string detail;
  ASSERT_TRUE(runtime.launch_replacing(launch, pid, detail));
  EXPECT_GT(pid, 0);
  for (int attempt = 0; attempt < 100 && runtime.running(); ++attempt) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(runtime.running());
  EXPECT_EQ(runtime.pid(), -1);
}
