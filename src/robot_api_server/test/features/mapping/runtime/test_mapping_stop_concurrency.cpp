// Link with the real mapping runtime and these process-OS fakes, NOT with
// runtime_process_utils.cpp. Never signals or enumerates real robot processes.
#include <atomic>
#include <chrono>
#include <csignal>
#include <future>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "robot_api_server/features/mapping/runtime/mapping_process_runtime.hpp"

using namespace std::chrono_literals;
namespace {
std::atomic<bool> stopping{false};
std::atomic<bool> finish_stop{false};
constexpr pid_t fake_pid = 99999999;
}
namespace robot_api_server {
std::vector<pid_t> list_proc_pids() { return finish_stop ? std::vector<pid_t>{} : std::vector<pid_t>{fake_pid}; }
std::string read_proc_cmdline(pid_t) { return "/fixture/run_projected_map.sh"; }
std::string read_proc_environ(pid_t) { return {}; }
bool process_group_has_live_process(pid_t) { return !finish_stop; }
bool process_pid_is_live(pid_t) { return !finish_stop; }
bool signal_process_group(pid_t, int signal) { if (signal == SIGINT) { stopping = true; } return true; }
void prepare_child_process(const std::string &) { std::abort(); }
}

TEST(MappingStopConcurrency, StatusRemainsResponsiveDuringChildShutdown)
{
  namespace runtime = robot_api_server::features::mapping::runtime;
  stopping = false;
  finish_stop = false;
  runtime::MappingProcessRuntime process({"", "", "", 3.0});
  ASSERT_TRUE(process.recover_snapshot().running);
  auto stop = std::async(std::launch::async, [&] { return process.stop(); });
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!stopping && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  auto snapshot = std::async(std::launch::async, [&] { return process.recover_snapshot(); });
  const auto status = snapshot.wait_for(100ms);
  auto start = std::async(std::launch::async, [&] {
    // This callback terminates the fixture attempt before fork(), after it has
    // acquired the real start/stop ownership lock.
    return process.start([] { throw std::runtime_error("fixture start reached"); });
  });
  const auto start_status = start.wait_for(50ms);
  // Always release the fixture before assertions so failures cannot deadlock.
  finish_stop = true;
  EXPECT_EQ(status, std::future_status::ready);
  EXPECT_EQ(start_status, std::future_status::timeout);
  stop.get();
  snapshot.get();
  EXPECT_THROW(start.get(), std::runtime_error);
  EXPECT_FALSE(process.tracked_snapshot().active);
}
