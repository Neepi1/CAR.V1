// Real router + mapping module + save writer + process runtime. Only OS process
// discovery/signalling is replaced. Run in private PID/network/mount + /tmp.
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_api_server/application/routing/application_router_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;
namespace {
std::atomic<bool> stop_entered{false}, finish_stop{false};
constexpr pid_t fake_pid = 99999999;
}
namespace robot_api_server {
void set_close_on_exec(int) {}
void close_inherited_fds() {}
bool is_pid_directory(const fs::path &) {return false;}
std::vector<pid_t> list_proc_pids() {return finish_stop ? std::vector<pid_t>{} : std::vector<pid_t>{fake_pid};}
std::string read_proc_cmdline(pid_t) {return "/fixture/run_projected_map.sh";}
std::string read_proc_environ(pid_t) {return {};}
bool process_group_has_live_process(pid_t) {return !finish_stop;}
bool process_pid_is_live(pid_t) {return !finish_stop;}
bool signal_process_group(pid_t, int signal) {if (signal == SIGINT) {stop_entered = true;} return true;}
void prepare_child_process(const std::string &) {std::abort();}
}

TEST(MappingSaveRouting, SlowShutdownDoesNotLockStatusHeartbeatOrMapAssets)
{
  using namespace robot_api_server;
  using namespace robot_api_server::features::mapping;
  namespace routing = robot_api_server::application::routing;
  stop_entered = false;
  finish_stop = false;
  int argc = 0;
  rclcpp::init(argc, nullptr);
  char pattern[] = "/tmp/mapping_save_routing_XXXXXX";
  const auto created = mkdtemp(pattern);
  ASSERT_NE(created, nullptr);
  const fs::path root(created);
  {
    auto node = std::make_shared<rclcpp::Node>("mapping_save_routing_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    application::runtime_mode::RuntimeModeCoordinator mode;
    MapCatalog catalog(root / "maps");
    std::mutex assets;
    MappingModuleConfig config;
    config.maps_root = root / "maps";
    config.runtime_maps_dir = root / "runtime";
    config.process.graceful_stop_timeout_sec = 40.0;
    config.process.lidar_rps_xps_state_dir = (root / "rps").string();
    config.live_map_topic = "/fixture_mapping/grid";
    config.resident_scan_control_service = "/fixture_mapping/scan";
    MappingModulePorts ports;
    ports.floor_runtime_operation_blocked = [](const auto &, auto &) {return false;};
    ports.floor_runtime_interlock_response = [](const auto &) {return std::optional<HttpResponse>{};};
    ports.map_asset_integrity_degraded = [] {return false;};
    ports.acquire_motion_admission = [](auto) -> ElevatorMotionAdmissionFence::AdmissionGuard {
        throw std::runtime_error("mapping must not acquire elevator test admission");
      };
    ports.navigation_goal_running = [] {return false;};
    ports.cancel_navigation = [] {return MappingNavigationActionResult{true, ""};};
    ports.stop_navigation_runtime = [] {return MappingNavigationActionResult{true, ""};};
    ports.clear_runtime_map_context = [] {};
    MappingModule mapping(*node, node->create_callback_group(rclcpp::CallbackGroupType::Reentrant),
      mode, catalog, assets, config, std::move(ports));
    mapping.set_live_map_page_active(true);
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(config.live_map_topic, 1);
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 4;
    grid.info.height = 4;
    grid.info.resolution = 0.05;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(16, 0);
    grid.data[3] = 100;
    const auto map_deadline = std::chrono::steady_clock::now() + 3s;
    while (!mapping.snapshot(false).live_map_available && std::chrono::steady_clock::now() < map_deadline) {
      publisher->publish(grid);
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_TRUE(mapping.snapshot(false).live_map_available);

    routing::ApplicationRouterModulePorts router_ports;
    router_ports.capture_motion_admission_epoch = []() -> std::uint64_t {
        throw std::runtime_error("unrelated route captured elevator epoch");
      };
    router_ports.elevator_interlock = [](const auto &) -> std::optional<HttpResponse> {
        throw std::runtime_error("unrelated route queried elevator interlock");
      };
    router_ports.gateway_token_configured = [] {return false;};
    router_ports.system_status = [&](const auto & request) -> std::optional<HttpResponse> {
        if (request.path == "/api/v1/status") {return HttpResponse{200, "application/json", mapping.status_json()};}
        return std::nullopt;
      };
    auto plain = [](const auto &) {return std::optional<HttpResponse>{};};
    auto admitted = [](const auto &, auto) {return std::optional<HttpResponse>{};};
    router_ports.maps = admitted;
    router_ports.elevator = [](const auto &, auto, bool, bool) {return std::optional<HttpResponse>{};};
    router_ports.mapping = [&](const auto & request, auto epoch) {return mapping.handle_http(request, epoch);};
    router_ports.gateway_metadata = plain;
    router_ports.subscriptions = [](const auto & request) -> std::optional<HttpResponse> {
        if (request.path == "/api/v1/subscriptions/heartbeat") {return HttpResponse{};}
        return std::nullopt;
      };
    router_ports.safety = admitted;
    router_ports.floor_switch = admitted;
    router_ports.localization = admitted;
    router_ports.navigation = admitted;
    router_ports.docking = admitted;
    routing::ApplicationRouterModule router(std::move(router_ports));
    HttpRequest save;
    save.method = "POST";
    save.path = "/api/v1/mapping/2d/save";
    save.body = R"({"async":true,"request_id":"save_fixture","building_id":"B10","floor_id":"F10","map_name":"fixture"})";
    const auto begin = std::chrono::steady_clock::now();
    const auto accepted = router.route(save, false);
    EXPECT_EQ(accepted.status, 202) << accepted.body;
    EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s);
    mapping.set_live_map_page_active(false); // The worker must own its frozen grid.
    const auto stop_deadline = std::chrono::steady_clock::now() + 3s;
    while (!stop_entered && std::chrono::steady_clock::now() < stop_deadline) {std::this_thread::sleep_for(5ms);}
    EXPECT_TRUE(stop_entered);
    auto reads = std::async(std::launch::async, [&] {
        const auto until = std::chrono::steady_clock::now() + 31s;
        long long max_ms = 0;
        do {
          const auto start = std::chrono::steady_clock::now();
          HttpRequest status; status.method = "GET"; status.path = "/api/v1/status";
          EXPECT_EQ(router.route(status, false).status, 200);
          status.method = "POST"; status.path = "/api/v1/subscriptions/heartbeat";
          EXPECT_EQ(router.route(status, false).status, 200);
          max_ms = std::max(max_ms, static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count()));
          std::this_thread::sleep_for(100ms);
        } while (std::chrono::steady_clock::now() < until);
        return max_ms;
      });
    const auto asset_available = assets.try_lock();
    if (asset_available) {assets.unlock();}
    EXPECT_TRUE(asset_available);
    EXPECT_EQ(router.route(save, false).status, 200); // Existing task, no second asset.
    const auto read_state = reads.wait_for(33s);
    finish_stop = true; // Always release before joining or asserting terminal state.
    mapping.shutdown();
    EXPECT_EQ(read_state, std::future_status::ready);
    EXPECT_LT(reads.get(), 500);
    const auto done = router.route(save, false);
    EXPECT_TRUE(json_bool_value(done.body, "map_saved", false)) << done.body;
    EXPECT_TRUE(json_bool_value(done.body, "mapping_stopped", false)) << done.body;
    EXPECT_EQ(catalog.read_all_map_manifests().size(), 1U);
    features::maps::durable_write_text_file_atomic(config.runtime_maps_dir / "save_jobs/stale.json",
      R"({"request_id":"stale","request_key":"key","state":"running","map_saved":true,"mapping_stopped":false,"result":{"map_id":"missing_map","asset_digest":"wrong"}})");
    HttpRequest stale; stale.method = "GET"; stale.path = "/api/v1/mapping/2d/save/status";
    stale.query["request_id"] = "stale";
    const auto unproven = router.route(stale, false);
    EXPECT_FALSE(json_bool_value(unproven.body, "map_saved", true)) << unproven.body;
    EXPECT_NE(unproven.body.find("asset_verification_error"), std::string::npos);
    executor.remove_node(node);
  }
  rclcpp::shutdown();
  fs::remove_all(root);
}
