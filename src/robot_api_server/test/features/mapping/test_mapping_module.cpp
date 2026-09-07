#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/mapping/mapping_module.hpp"

namespace
{

namespace fs = std::filesystem;
using robot_api_server::ElevatorExecutionInterlock;
using robot_api_server::ElevatorMotionAdmissionFence;
using robot_api_server::HttpRequest;
using robot_api_server::MapCatalog;
using robot_api_server::application::runtime_mode::RuntimeModeCoordinator;
using robot_api_server::features::mapping::MappingModule;
using robot_api_server::features::mapping::MappingModuleConfig;
using robot_api_server::features::mapping::MappingModulePorts;
using robot_api_server::features::mapping::MappingNavigationActionResult;

class MappingModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("robot_api_mapping_module_test_" +
      std::to_string(reinterpret_cast<std::uintptr_t>(this)));
    fs::create_directories(root_ / "maps");
    fs::create_directories(root_ / "runtime_maps");

    node_ = std::make_shared<rclcpp::Node>("mapping_module_contract_test");
    catalog_ = std::make_unique<MapCatalog>(root_ / "maps");

    MappingModuleConfig config;
    config.process.start_command = (root_ / "missing_mapping_start.sh").string();
    config.process.log_file = (root_ / "mapping.log").string();
    config.process.lidar_rps_xps_state_dir = (root_ / "rps_xps").string();
    config.maps_root = root_ / "maps";
    config.runtime_maps_dir = root_ / "runtime_maps";
    config.live_map_topic = "/mapping_module_test/map";
    config.resident_scan_control_service = "/mapping_module_test/scan_control";

    MappingModulePorts ports;
    ports.floor_runtime_operation_blocked =
      [](const std::string &, std::string &) {return false;};
    ports.floor_runtime_interlock_response =
      [](const std::string &) {return std::nullopt;};
    ports.map_asset_integrity_degraded = []() {return false;};
    ports.acquire_motion_admission = [this](const auto expected_epoch) {
        return admission_fence_.acquire_for_submission(
          expected_epoch, []() {return ElevatorExecutionInterlock{};});
      };
    ports.navigation_goal_running = []() {return false;};
    ports.cancel_navigation = []() {
        return MappingNavigationActionResult{true, "navigation already idle"};
      };
    ports.stop_navigation_runtime = []() {
        return MappingNavigationActionResult{true, "navigation runtime stopped"};
      };
    ports.clear_runtime_map_context = []() {};

    module_ = std::make_unique<MappingModule>(
      *node_,
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant),
      runtime_mode_,
      *catalog_,
      asset_mutex_,
      std::move(config),
      std::move(ports));
  }

  void TearDown() override
  {
    if (module_) {
      module_->shutdown();
      module_.reset();
    }
    catalog_.reset();
    node_.reset();
    std::error_code error;
    fs::remove_all(root_, error);
  }

  std::optional<robot_api_server::HttpResponse> handle(
    const std::string & method,
    const std::string & path)
  {
    HttpRequest request;
    request.method = method;
    request.path = path;
    return module_->handle_http(request, admission_fence_.capture_epoch());
  }

  fs::path root_;
  std::shared_ptr<rclcpp::Node> node_;
  RuntimeModeCoordinator runtime_mode_;
  ElevatorMotionAdmissionFence admission_fence_;
  std::mutex asset_mutex_;
  std::unique_ptr<MapCatalog> catalog_;
  std::unique_ptr<MappingModule> module_;
};

TEST_F(MappingModuleTest, OwnsOnlyMappingRoutes)
{
  EXPECT_FALSE(handle("GET", "/api/v1/status").has_value());

  const auto response = handle("GET", "/api/v1/mapping/2d/map");
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->status, 409);
  EXPECT_NE(response->body.find("live_map resource is not acquired"), std::string::npos);
}

TEST_F(MappingModuleTest, RejectsStartWhenConfiguredLauncherIsMissing)
{
  const auto response = handle("POST", "/api/v1/mapping/2d/start");
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->status, 503);
  EXPECT_NE(response->body.find("start command is not available"), std::string::npos);

  const auto state = module_->snapshot(false);
  EXPECT_FALSE(state.process_active);
  EXPECT_FALSE(state.process_running);
  EXPECT_FALSE(state.start_job_running);
}

TEST_F(MappingModuleTest, FormatsIdleStatusThroughTheModuleBoundary)
{
  const auto status = module_->status_json();
  EXPECT_NE(status.find("\"active\":false"), std::string::npos);
  EXPECT_NE(status.find("\"state\":\"stopped\""), std::string::npos);
  EXPECT_NE(status.find("\"live_map_available\":false"), std::string::npos);
  EXPECT_NE(status.find("\"start_job\":"), std::string::npos);
}

}  // namespace
