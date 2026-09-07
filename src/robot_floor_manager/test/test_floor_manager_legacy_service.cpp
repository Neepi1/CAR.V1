#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/floor_switch.hpp"
#include "robot_interfaces/msg/floor_switch_status.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"

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

class TemporaryFloorAssets
{
public:
  TemporaryFloorAssets()
  {
    root_ = fs::temp_directory_path() /
      ("robot_floor_manager_legacy_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto floor_root = root_ / "B10" / "F2";
    const std::vector<fs::path> files{
      floor_root / "nav" / "nav_map.yaml",
      floor_root / "nav" / "nav_map.pgm",
      floor_root / "localizer" / "localizer_map.png",
      floor_root / "localizer" / "localizer_params.yaml",
      floor_root / "filters" / "keepout_mask.yaml",
      floor_root / "filters" / "keepout_mask.pgm",
      floor_root / "filters" / "speed_mask.yaml",
      floor_root / "filters" / "speed_mask.pgm",
      floor_root / "filters" / "binary_mask.yaml",
      floor_root / "filters" / "binary_mask.pgm",
      floor_root / "reports" / "asset_report.json",
      floor_root / "poses.yaml",
    };
    for (const auto & file : files) {
      fs::create_directories(file.parent_path());
      std::ofstream stream(file);
      stream << "test\n";
    }
  }

  ~TemporaryFloorAssets()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & root() const
  {
    return root_;
  }

private:
  fs::path root_;
};

TEST(FloorManagerLegacyService, LiveSwitchNeverSucceedsWithoutFilterAndCostmapProof)
{
  ScopedRclcppRuntime runtime;
  TemporaryFloorAssets assets;
  std::atomic_bool keepout_loaded{false};
  std::atomic_bool global_cleared{false};
  std::atomic_bool local_cleared{false};

  auto fake = std::make_shared<rclcpp::Node>("floor_manager_legacy_fake_ports");
  auto map_load = fake->create_service<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map",
    [](const std::shared_ptr<nav2_msgs::srv::LoadMap::Request>,
      std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
    {
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    });
  auto keepout_load = fake->create_service<nav2_msgs::srv::LoadMap>(
    "/keepout_filter_mask_server/load_map",
    [&keepout_loaded](
      const std::shared_ptr<nav2_msgs::srv::LoadMap::Request>,
      std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
    {
      keepout_loaded = true;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    });
  auto localizer_apply = fake->create_service<robot_interfaces::srv::ApplyFloorAssets>(
    "/global_localization/apply_floor_assets",
    [](const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Request>,
      std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> response)
    {
      response->success = true;
    });
  auto localization_trigger = fake->create_service<robot_interfaces::srv::TriggerLocalization>(
    "/global_localization/trigger",
    [](const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request>,
      std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response)
    {
      response->accepted = true;
    });
  auto global_clear = fake->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap",
    [&global_cleared](
      const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
      std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
    {
      global_cleared = true;
    });
  auto local_clear = fake->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap",
    [&local_cleared](
      const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
      std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
    {
      local_cleared = true;
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("maps_root", assets.root().string()),
    rclcpp::Parameter("service_timeout_sec", 0.5),
    rclcpp::Parameter(
      "motion_hold_sequence_state_file",
      (assets.root() / "hold_sequence.state").string()),
  });
  auto floor_manager = std::make_shared<FloorManagerNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("floor_manager_legacy_test_client");
  auto client = client_node->create_client<robot_interfaces::srv::SwitchFloor>(
    "/floor_manager/switch_floor");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(fake);
  executor.add_node(floor_manager);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  std::shared_ptr<robot_interfaces::srv::SwitchFloor::Response> response;
  if (client->wait_for_service(2s)) {
    auto request = std::make_shared<robot_interfaces::srv::SwitchFloor::Request>();
    request->building_id = "B10";
    request->floor_id = "F2";
    request->resume_navigation = true;
    auto future = client->async_send_request(request);
    if (future.wait_for(3s) == std::future_status::ready) {
      response = future.get();
    }
  }

  EXPECT_NE(response, nullptr);
  if (response) {
    const bool complete_live_proof =
      keepout_loaded.load() && global_cleared.load() && local_cleared.load();
    EXPECT_FALSE(response->success && !complete_live_proof)
      << "legacy live switch returned success without filter/costmap proof";
  }

  executor.cancel();
  spin_thread.join();
}

TEST(FloorManagerFloorSwitchAction, ProductionDefaultAbortsWithoutMutationCapability)
{
  using FloorSwitch = robot_interfaces::action::FloorSwitch;

  ScopedRclcppRuntime runtime;
  std::atomic_int map_load_calls{0};
  std::atomic_int localizer_apply_calls{0};
  std::atomic_int localization_trigger_calls{0};
  std::atomic_int costmap_clear_calls{0};
  TemporaryFloorAssets isolated_state;

  auto fake = std::make_shared<rclcpp::Node>("floor_switch_action_fake_ports");
  auto map_load = fake->create_service<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map",
    [&map_load_calls](
      const std::shared_ptr<nav2_msgs::srv::LoadMap::Request>,
      std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
    {
      ++map_load_calls;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    });
  auto localizer_apply = fake->create_service<robot_interfaces::srv::ApplyFloorAssets>(
    "/global_localization/apply_floor_assets",
    [&localizer_apply_calls](
      const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Request>,
      std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> response)
    {
      ++localizer_apply_calls;
      response->success = true;
    });
  auto localization_trigger = fake->create_service<robot_interfaces::srv::TriggerLocalization>(
    "/global_localization/trigger",
    [&localization_trigger_calls](
      const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request>,
      std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response)
    {
      ++localization_trigger_calls;
      response->accepted = true;
    });
  auto global_clear = fake->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap",
    [&costmap_clear_calls](
      const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
      std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
    {
      ++costmap_clear_calls;
    });
  auto local_clear = fake->create_service<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap",
    [&costmap_clear_calls](
      const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
      std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>)
    {
      ++costmap_clear_calls;
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "motion_hold_sequence_state_file",
      (isolated_state.root() / "hold_sequence.state").string()),
  });
  auto floor_manager = std::make_shared<FloorManagerNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("floor_switch_action_test_client");
  auto action_client = rclcpp_action::create_client<FloorSwitch>(
    client_node, "/floor_manager/floor_switch");

  std::mutex status_mutex;
  std::shared_ptr<robot_interfaces::msg::FloorSwitchStatus> last_status;
  auto status_sub =
    client_node->create_subscription<robot_interfaces::msg::FloorSwitchStatus>(
    "/floor_manager/transition_status",
    rclcpp::QoS(1).reliable().transient_local(),
    [&status_mutex, &last_status](
      const robot_interfaces::msg::FloorSwitchStatus::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(status_mutex);
      last_status = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fake);
  executor.add_node(floor_manager);
  executor.add_node(client_node);

  std::shared_ptr<FloorSwitch::Result> result;
  rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::UNKNOWN;
  if (action_client->wait_for_action_server(2s)) {
    FloorSwitch::Goal goal;
    goal.transaction_id = "elevator-tx-17";
    goal.building_id = "B10";
    goal.floor_id = "F2";
    goal.map_id = "map_f2";
    goal.expected_asset_epoch = 17U;
    goal.expected_asset_digest =
      "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
    auto goal_future = action_client->async_send_goal(goal);
    if (executor.spin_until_future_complete(goal_future, 2s) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      const auto goal_handle = goal_future.get();
      if (goal_handle) {
        auto result_future = action_client->async_get_result(goal_handle);
        if (executor.spin_until_future_complete(result_future, 2s) ==
          rclcpp::FutureReturnCode::SUCCESS)
        {
          const auto wrapped = result_future.get();
          result_code = wrapped.code;
          result = wrapped.result;
        }
      }
    }
  }

  EXPECT_NE(result, nullptr);
  if (result) {
    EXPECT_EQ(result_code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(result->success);
    EXPECT_EQ(
      result->failure_code,
      static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kLiveSwitchDisabled));
    EXPECT_FALSE(result->recovery_required);
  }
  EXPECT_EQ(map_load_calls.load(), 0);
  EXPECT_EQ(localizer_apply_calls.load(), 0);
  EXPECT_EQ(localization_trigger_calls.load(), 0);
  EXPECT_EQ(costmap_clear_calls.load(), 0);
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    EXPECT_NE(last_status, nullptr);
    if (last_status) {
      EXPECT_EQ(last_status->state, "BLOCKED");
      EXPECT_EQ(last_status->transaction_id, "elevator-tx-17");
      EXPECT_EQ(last_status->requested_asset_epoch, 17U);
      EXPECT_EQ(
        last_status->requested_asset_digest,
        "sha256:0123456789abcdef0123456789abcdef"
        "0123456789abcdef0123456789abcdef");
      EXPECT_EQ(last_status->asset_epoch, 0U);
      EXPECT_TRUE(last_status->asset_digest.empty())
        << "a blocked request must not be published as active asset evidence";
    }
  }

  executor.cancel();
}

TEST(FloorManagerFloorSwitchAction, ForeignCancellationCannotCancelActiveTransaction)
{
  EXPECT_TRUE(cancel_matches_active_floor_switch(true, "tx-active", "tx-active"));
  EXPECT_FALSE(cancel_matches_active_floor_switch(true, "tx-active", "tx-other"));
  EXPECT_FALSE(cancel_matches_active_floor_switch(false, "tx-active", "tx-active"));
  EXPECT_FALSE(cancel_matches_active_floor_switch(true, "tx-active", ""));
}

TEST(FloorManagerBridgeContract, AbortMayEchoRestoredSourceEpoch)
{
  using Request = robot_interfaces::srv::BeginFloorTransition::Request;
  EXPECT_TRUE(bridge_response_requires_target_epoch(Request::OP_BEGIN));
  EXPECT_TRUE(bridge_response_requires_target_epoch(Request::OP_COMMIT));
  EXPECT_FALSE(bridge_response_requires_target_epoch(Request::OP_ABORT));
  EXPECT_FALSE(
    bridge_response_requires_target_epoch(Request::OP_ABORT_PREMUTATION));
}

TEST(FloorManagerBridgeContract, RejectedBeginWithInvalidContextRequiresRecovery)
{
  EXPECT_FALSE(bridge_begin_rejection_requires_recovery(false, true));
  EXPECT_TRUE(bridge_begin_rejection_requires_recovery(false, false));
  EXPECT_FALSE(bridge_begin_rejection_requires_recovery(true, false));
}

TEST(FloorManagerFilterPolicy, KeepoutIsAlwaysRuntimeRequired)
{
  const auto roles = runtime_filter_reload_roles(false);
  ASSERT_EQ(roles.size(), 1U);
  EXPECT_EQ(roles.front(), RuntimeFilterRole::kKeepout);
}

TEST(FloorManagerFilterPolicy, SpeedIsRuntimeRequiredOnlyWhenEnabled)
{
  const auto roles = runtime_filter_reload_roles(true);
  ASSERT_EQ(roles.size(), 2U);
  EXPECT_EQ(roles[0], RuntimeFilterRole::kKeepout);
  EXPECT_EQ(roles[1], RuntimeFilterRole::kSpeed);
}

}  // namespace
