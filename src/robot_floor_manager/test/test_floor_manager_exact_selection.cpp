#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_map_asset_identity/map_asset_identity.hpp"

#include "../src/floor_manager_node.cpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

class ScopedRclcppRuntime
{
public:
  ScopedRclcppRuntime()
  {
    rclcpp::init(0, nullptr);
  }

  ~ScopedRclcppRuntime()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  ScopedRclcppRuntime(const ScopedRclcppRuntime &) = delete;
  ScopedRclcppRuntime & operator=(const ScopedRclcppRuntime &) = delete;
};

TEST(FloorManagerBridgeContract, PreMutationAbortAcceptsRestoredSourceEpoch)
{
  using Request = robot_interfaces::srv::BeginFloorTransition::Request;
  EXPECT_TRUE(bridge_response_requires_target_epoch(Request::OP_BEGIN));
  EXPECT_TRUE(bridge_response_requires_target_epoch(Request::OP_COMMIT));
  EXPECT_FALSE(bridge_response_requires_target_epoch(Request::OP_ABORT));
  EXPECT_FALSE(
    bridge_response_requires_target_epoch(Request::OP_ABORT_PREMUTATION));
}

TEST(FloorManagerTimeoutContract, LocalizationTransactionHasIndependentOuterBudget)
{
  ScopedRclcppRuntime runtime;
  const auto sequence_file = fs::temp_directory_path() /
    ("floor_timeout_sequence_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("motion_hold_sequence_state_file", sequence_file.string()),
  });
  auto floor_manager = std::make_shared<FloorManagerNode>(options);

  EXPECT_DOUBLE_EQ(floor_manager->get_parameter("service_timeout_sec").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(
    floor_manager->get_parameter("localizer_apply_timeout_sec").as_double(), 30.0);
  EXPECT_DOUBLE_EQ(
    floor_manager->get_parameter("localization_trigger_timeout_sec").as_double(), 75.0);

  floor_manager.reset();
  std::error_code error;
  fs::remove(sequence_file, error);
}

class ExactMapFixture
{
public:
  ExactMapFixture()
  {
    root_ = fs::temp_directory_path() /
      ("robot_floor_manager_exact_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    maps_root_ = root_ / "maps_release";
    map_root_ = maps_root_ / building_id_ / floor_id_ / "maps" / map_id_;

    std::vector<robot_map_asset_identity::DigestEntry> digest_entries;
    const auto role_files = roles();
    for (std::size_t index = 0U; index < role_files.size(); ++index) {
      const auto content =
        role_files[index].first + "-content-" + std::to_string(index);
      write_file(role_files[index].second, content);
      digest_entries.push_back({role_files[index].first, content});
    }
    write_file(map_root_ / "poses.yaml", "poses: []\n");
    asset_digest_ =
      robot_map_asset_identity::canonical_map_asset_digest(digest_entries);
    robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps_root_);
    const auto identity = registry.bind(
      {building_id_, floor_id_, map_id_}, asset_digest_);
    asset_epoch_ = identity.asset_epoch;

    std::ostringstream manifest;
    manifest << "{\n"
             << "  \"schema\": \"njrh.map_manifest.v2\",\n"
             << "  \"asset_epoch\": " << asset_epoch_ << ",\n"
             << "  \"asset_digest_algorithm\": \"sha256\",\n"
             << "  \"asset_digest_contract\": \"njrh-map-asset-bundle-v1\",\n"
             << "  \"asset_digest\": \"" << asset_digest_ << "\",\n"
             << "  \"building_id\": \"" << building_id_ << "\",\n"
             << "  \"floor_id\": \"" << floor_id_ << "\",\n"
             << "  \"map_id\": \"" << map_id_ << "\",\n"
             << "  \"safe_map_name\": \"" << safe_map_name_ << "\"\n"
             << "}\n";
    write_file(map_root_ / "manifest.json", manifest.str());
  }

  ~ExactMapFixture()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & maps_root() const
  {
    return maps_root_;
  }

  const fs::path & map_root() const
  {
    return map_root_;
  }

  const std::string & building_id() const
  {
    return building_id_;
  }

  const std::string & floor_id() const
  {
    return floor_id_;
  }

  const std::string & map_id() const
  {
    return map_id_;
  }

  std::uint64_t asset_epoch() const
  {
    return asset_epoch_;
  }

  const std::string & asset_digest() const
  {
    return asset_digest_;
  }

private:
  void write_file(const fs::path & path, const std::string & content)
  {
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
  }

  std::vector<std::pair<std::string, fs::path>> roles() const
  {
    return {
      {"nav_map_yaml", map_root_ / "nav" / (safe_map_name_ + ".yaml")},
      {"nav_map_pgm", map_root_ / "nav" / (safe_map_name_ + ".pgm")},
      {"localizer_map_png", map_root_ / "localizer" / (safe_map_name_ + ".png")},
      {
        "localizer_params_yaml",
        map_root_ / "localizer" / (safe_map_name_ + ".yaml")
      },
      {"keepout_mask_yaml", map_root_ / "filters" / "keepout_mask.yaml"},
      {"keepout_mask_pgm", map_root_ / "filters" / "keepout_mask.pgm"},
      {"speed_mask_yaml", map_root_ / "filters" / "speed_mask.yaml"},
      {"speed_mask_pgm", map_root_ / "filters" / "speed_mask.pgm"},
      {"binary_mask_yaml", map_root_ / "filters" / "binary_mask.yaml"},
      {"binary_mask_pgm", map_root_ / "filters" / "binary_mask.pgm"},
      {
        "asset_report_json",
        map_root_ / "reports" / "asset_report.json"
      },
    };
  }

  fs::path root_;
  fs::path maps_root_;
  fs::path map_root_;
  std::string building_id_{"B10"};
  std::string floor_id_{"F2"};
  std::string map_id_{"map_exact_unselected"};
  std::string safe_map_name_{"delivery-map"};
  std::uint64_t asset_epoch_{0U};
  std::string asset_digest_;
};

TEST(FloorManagerExactSelection, SelectsInactiveSourceBundleWithoutCurrentProjection)
{
  ScopedRclcppRuntime runtime;
  ExactMapFixture fixture;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("maps_root", fixture.maps_root().string()),
    rclcpp::Parameter(
      "motion_hold_sequence_state_file",
      (fixture.maps_root().parent_path() / "hold_sequence.state").string()),
  });
  auto floor_manager = std::make_shared<FloorManagerNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("floor_manager_exact_test_client");
  auto client = client_node->create_client<robot_interfaces::srv::SwitchFloor>(
    "/floor_manager/switch_floor");
  std::mutex status_mutex;
  std::vector<std::string> status_history;
  auto status_subscription = client_node->create_subscription<std_msgs::msg::String>(
    "/floor_manager/status",
    rclcpp::QoS(10).reliable().transient_local(),
    [&](const std_msgs::msg::String::SharedPtr message) {
      std::lock_guard<std::mutex> lock(status_mutex);
      status_history.push_back(message->data);
    });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(floor_manager);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  std::shared_ptr<robot_interfaces::srv::SwitchFloor::Response> response;
  if (client->wait_for_service(2s)) {
    auto request = std::make_shared<robot_interfaces::srv::SwitchFloor::Request>();
    request->building_id = fixture.building_id();
    request->floor_id = fixture.floor_id();
    request->map_id = fixture.map_id();
    request->expected_asset_epoch = fixture.asset_epoch();
    request->expected_asset_digest = fixture.asset_digest();
    request->resume_navigation = false;
    auto future = client->async_send_request(request);
    if (future.wait_for(3s) == std::future_status::ready) {
      response = future.get();
    }
  }

  EXPECT_NE(response, nullptr);
  if (response) {
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_EQ(response->code, "OK");
    EXPECT_EQ(response->selected_building_id, fixture.building_id());
    EXPECT_EQ(response->selected_floor_id, fixture.floor_id());
    EXPECT_EQ(response->selected_map_id, fixture.map_id());
    EXPECT_EQ(response->asset_epoch, fixture.asset_epoch());
    EXPECT_EQ(response->asset_digest, fixture.asset_digest());
    EXPECT_EQ(
      response->nav_map_yaml,
      fs::absolute(fixture.map_root() / "nav" / "delivery-map.yaml").string());
  }
  EXPECT_FALSE(fs::exists(fixture.maps_root() / "B10" / "F2" / "current"));
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    EXPECT_FALSE(status_history.empty());
    if (!status_history.empty()) {
      EXPECT_EQ(status_history.back(), "idle");
      for (const auto & status : status_history) {
        EXPECT_NE(status.rfind("switching:", 0U), 0U)
          << "source preflight must not publish a switching runtime state";
        EXPECT_NE(status.rfind("active:", 0U), 0U)
          << "source preflight must not publish an active runtime state";
      }
    }
  }

  executor.cancel();
  spin_thread.join();
}

}  // namespace
