#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
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

#include "composition_interfaces/srv/list_nodes.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_global_localization/isaac_asset_reloader.hpp"
#include "robot_global_localization/ros_component_manager_port.hpp"

namespace fs = std::filesystem;

namespace
{

using namespace std::chrono_literals;
using robot_global_localization::ApplyFloorAssetResult;
using robot_global_localization::ComponentLoadRequest;
using robot_global_localization::ComponentPresence;
using robot_global_localization::IsaacAssetReloader;
using robot_global_localization::IsaacAssetReloaderOptions;
using robot_global_localization::LocalizerParameter;
using robot_global_localization::LocalizerParameterType;
using robot_global_localization::RosComponentManagerOptions;
using robot_global_localization::RosComponentManagerPort;

class FakeIsaacCompositionRuntime final : public rclcpp::Node
{
public:
  explicit FakeIsaacCompositionRuntime(
    const std::chrono::milliseconds load_delay = 0ms,
    const std::chrono::milliseconds unload_response_delay = 0ms)
  : Node("fake_isaac_composition_runtime"),
    load_delay_(load_delay),
    unload_response_delay_(unload_response_delay)
  {
    list_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    load_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    list_nodes_service_ = create_service<composition_interfaces::srv::ListNodes>(
      "/occupancy_grid_localizer_container/_container/list_nodes",
      [this](
        const std::shared_ptr<composition_interfaces::srv::ListNodes::Request>,
        std::shared_ptr<composition_interfaces::srv::ListNodes::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!loaded_ && transient_invalid_list_responses_ > 0U) {
          --transient_invalid_list_responses_;
          response->full_node_names.push_back("/occupancy_grid_localizer");
          return;
        }
        if (loaded_) {
          response->full_node_names.push_back("/occupancy_grid_localizer");
          response->unique_ids.push_back(unique_id_);
        }
      },
      rmw_qos_profile_services_default,
      list_callback_group_);
    unload_service_ = create_service<composition_interfaces::srv::UnloadNode>(
      "/occupancy_grid_localizer_container/_container/unload_node",
      [this](
        const std::shared_ptr<composition_interfaces::srv::UnloadNode::Request> request,
        std::shared_ptr<composition_interfaces::srv::UnloadNode::Response> response)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++unload_calls_;
          response->success = loaded_ && request->unique_id == unique_id_;
          if (response->success) {
            loaded_ = false;
          } else {
            response->error_message = "unexpected component id";
          }
        }
        std::this_thread::sleep_for(unload_response_delay_);
      });
    load_service_ = create_service<composition_interfaces::srv::LoadNode>(
      "/occupancy_grid_localizer_container/_container/load_node",
      [this](
        const std::shared_ptr<composition_interfaces::srv::LoadNode::Request> request,
        std::shared_ptr<composition_interfaces::srv::LoadNode::Response> response)
      {
        std::this_thread::sleep_for(load_delay_);
        std::lock_guard<std::mutex> lock(mutex_);
        ++load_calls_;
        received_load_ = *request;
        if (loaded_) {
          response->success = false;
          response->error_message = "duplicate component";
          return;
        }
        loaded_ = true;
        unique_id_ = 42U;
        response->success = true;
        response->full_node_name = "/occupancy_grid_localizer";
        response->unique_id = unique_id_;
      },
      rmw_qos_profile_services_default,
      load_callback_group_);
    list_parameters_service_ = create_service<rcl_interfaces::srv::ListParameters>(
      "/occupancy_grid_localizer/list_parameters",
      [](
        const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request>,
        std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response)
      {
        response->result.names = {
          "map_yaml_path",
          "image",
          "resolution",
          "origin",
          "occupied_thresh",
          "batch_size",
        };
      });
    get_parameters_service_ = create_service<rcl_interfaces::srv::GetParameters>(
      "/occupancy_grid_localizer/get_parameters",
      [this](
        const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & name : request->names) {
          rcl_interfaces::msg::ParameterValue value;
          if (name == "map_yaml_path") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            value.string_value = map_yaml_path_;
          } else if (name == "image") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            value.string_value = image_;
          } else if (name == "resolution") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            value.double_value = resolution_;
          } else if (name == "origin") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            value.double_array_value = origin_;
          } else if (name == "occupied_thresh") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            value.double_value = occupied_thresh_;
          } else if (name == "batch_size") {
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            value.integer_value = 1024;
          }
          response->values.push_back(std::move(value));
        }
      });
  }

  void set_map_parameters(
    std::string map_yaml_path,
    std::string image,
    const double resolution,
    std::vector<double> origin,
    const double occupied_thresh)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_yaml_path_ = std::move(map_yaml_path);
    image_ = std::move(image);
    resolution_ = resolution;
    origin_ = std::move(origin);
    occupied_thresh_ = occupied_thresh;
  }

  composition_interfaces::srv::LoadNode::Request received_load() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_load_;
  }

  bool loaded() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return loaded_;
  }

  std::size_t unload_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return unload_calls_;
  }

  std::size_t load_calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_calls_;
  }

  void set_transient_invalid_list_responses_after_unload(const std::size_t count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    transient_invalid_list_responses_ = count;
  }

private:
  mutable std::mutex mutex_;
  std::chrono::milliseconds load_delay_{0};
  std::chrono::milliseconds unload_response_delay_{0};
  bool loaded_{true};
  std::uint64_t unique_id_{41U};
  std::string map_yaml_path_{"/old/localizer_params.yaml"};
  std::string image_{"old.png"};
  double resolution_{0.1};
  std::vector<double> origin_{0.0, 0.0, 0.0};
  double occupied_thresh_{0.7};
  std::size_t unload_calls_{0U};
  std::size_t load_calls_{0U};
  std::size_t transient_invalid_list_responses_{0U};
  composition_interfaces::srv::LoadNode::Request received_load_;
  rclcpp::Service<composition_interfaces::srv::ListNodes>::SharedPtr list_nodes_service_;
  rclcpp::Service<composition_interfaces::srv::UnloadNode>::SharedPtr unload_service_;
  rclcpp::Service<composition_interfaces::srv::LoadNode>::SharedPtr load_service_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::CallbackGroup::SharedPtr list_callback_group_;
  rclcpp::CallbackGroup::SharedPtr load_callback_group_;
};

class RosComponentManagerPortTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // These tests deliberately create composition-manager services with the
    // production names.  Keep them off the live robot ROS graph even when the
    // binary is launched directly instead of through CTest.
    ASSERT_EQ(::setenv("ROS_DOMAIN_ID", "198", 1), 0);
    ASSERT_EQ(::setenv("ROS_LOCALHOST_ONLY", "1", 1), 0);
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
    const auto unique = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    root_ = fs::temp_directory_path() / ("njrh_ros_bootstrap_" + unique);
  }

  void TearDown() override
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  static void write_text(const fs::path & path, const std::string & text)
  {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open());
    stream << text;
    ASSERT_TRUE(stream.good());
  }

  static void write_png(const fs::path & path)
  {
    static constexpr unsigned char kPngHeader[] = {
      0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a,
    };
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open());
    stream.write(
      reinterpret_cast<const char *>(kPngHeader),
      static_cast<std::streamsize>(sizeof(kPngHeader)));
    ASSERT_TRUE(stream.good());
  }

  fs::path create_ready_runtime_context(fs::path & asset_root)
  {
    asset_root = root_ / "maps_release";
    const fs::path current_root = asset_root / "B11" / "F2" / "current";
    fs::create_directories(current_root / "nav");
    fs::create_directories(current_root / "localizer");
    write_text(
      current_root / "nav" / "nav_map.yaml",
      "image: nav_map.pgm\nresolution: 0.05\n");
    write_png(current_root / "localizer" / "localizer_map.png");
    write_text(
      current_root / "localizer" / "localizer_params.yaml",
      "image: localizer_map.png\n"
      "resolution: 0.05\n"
      "origin: [1.0, -2.0, 0.3]\n"
      "occupied_thresh: 0.65\n"
      "free_thresh: 0.196\n");
    write_text(
      current_root / "manifest.json",
      "{\n"
      "  \"schema\": \"njrh.map_manifest.v2\",\n"
      "  \"asset_epoch\": 17,\n"
      "  \"asset_digest_algorithm\": \"sha256\",\n"
      "  \"asset_digest_contract\": \"njrh-map-asset-bundle-v1\",\n"
      "  \"asset_digest\": \"sha256:" + std::string(64U, 'a') + "\",\n"
      "  \"map_id\": \"map_f2\",\n"
      "  \"building_id\": \"B11\",\n"
      "  \"floor_id\": \"F2\",\n"
      "  \"active\": true\n"
      "}\n");
    const fs::path runtime_context = root_ / "runtime_map_context.json";
    write_text(
      runtime_context,
      "{\n"
      "  \"schema\": \"njrh.runtime_map_context.v1\",\n"
      "  \"state\": \"ready\",\n"
      "  \"confirmed\": true,\n"
      "  \"transaction_id\": \"floor-live-17\",\n"
      "  \"building_id\": \"B11\",\n"
      "  \"floor_id\": \"F2\",\n"
      "  \"map_id\": \"map_f2\",\n"
      "  \"asset_epoch\": 17,\n"
      "  \"asset_digest\": \"sha256:" + std::string(64U, 'a') + "\",\n"
      "  \"localizer_generation\": 8,\n"
      "  \"explicit_relocalization_sequence\": 12,\n"
      "  \"updated_at\": 1234.5\n"
      "}\n");
    return runtime_context;
  }

  fs::path root_;
};

class ExecutorSpinGuard
{
public:
  explicit ExecutorSpinGuard(rclcpp::Executor & executor)
  : executor_(executor),
    thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorSpinGuard()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

TEST_F(RosComponentManagerPortTest, CapturesThenReplacesTheExactIsaacComponent)
{
  auto client_node = std::make_shared<rclcpp::Node>("component_port_test_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>();
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions options;
  options.container_name = "/occupancy_grid_localizer_container";
  options.expected_full_node_name = "/occupancy_grid_localizer";
  options.operation_timeout = 3s;
  RosComponentManagerPort port(*client_node, callback_group, options);

  const auto preflight = port.preflight();
  ASSERT_TRUE(preflight.success) << preflight.failure_code << ": " << preflight.message;

  const auto capture = port.capture();
  ASSERT_TRUE(capture.success) << capture.failure_code << ": " << capture.message;
  EXPECT_EQ(capture.snapshot.unique_id, 41U);
  EXPECT_EQ(capture.snapshot.full_node_name, "/occupancy_grid_localizer");
  ASSERT_EQ(capture.snapshot.parameters.size(), 6U);
  EXPECT_EQ(capture.snapshot.parameters.back().name, "batch_size");
  EXPECT_EQ(capture.snapshot.parameters.back().type, LocalizerParameterType::kInteger);
  EXPECT_EQ(capture.snapshot.parameters.back().integer_value, 1024);

  const auto unload = port.unload(capture.snapshot.unique_id);
  ASSERT_TRUE(unload.success) << unload.failure_code << ": " << unload.message;
  EXPECT_EQ(unload.resulting_presence, ComponentPresence::kAbsent);

  ComponentLoadRequest load_request;
  load_request.description.package_name = "isaac_ros_occupancy_grid_localizer";
  load_request.description.plugin_name =
    "nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode";
  load_request.description.node_name = "occupancy_grid_localizer";
  load_request.description.remap_rules = {
    "flatscan:=/flatscan",
    "localization_result:=/localization_result",
  };
  load_request.parameters = capture.snapshot.parameters;

  const auto load = port.load(load_request);
  ASSERT_TRUE(load.success) << load.failure_code << ": " << load.message;
  EXPECT_EQ(load.unique_id, 42U);
  EXPECT_EQ(load.resulting_presence, ComponentPresence::kPresent);

  const auto received_load = fake_runtime->received_load();
  EXPECT_EQ(received_load.package_name, load_request.description.package_name);
  EXPECT_EQ(received_load.plugin_name, load_request.description.plugin_name);
  EXPECT_EQ(received_load.node_name, load_request.description.node_name);
  EXPECT_EQ(received_load.remap_rules, load_request.description.remap_rules);
  ASSERT_EQ(received_load.parameters.size(), load_request.parameters.size());
  EXPECT_EQ(received_load.parameters.back().name, "batch_size");
  EXPECT_EQ(
    received_load.parameters.back().value.type,
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(received_load.parameters.back().value.integer_value, 1024);

}

TEST_F(RosComponentManagerPortTest, LateLoadTimeoutIsNeverSuccessAndLeavesNoTarget)
{
  auto client_node = std::make_shared<rclcpp::Node>("component_port_timeout_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>(350ms);
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions options;
  options.container_name = "/occupancy_grid_localizer_container";
  options.expected_full_node_name = "/occupancy_grid_localizer";
  options.operation_timeout = 200ms;
  RosComponentManagerPort port(*client_node, callback_group, options);

  const auto preflight = port.preflight();
  EXPECT_TRUE(preflight.success) << preflight.failure_code << ": " << preflight.message;
  const auto capture = port.capture();
  EXPECT_TRUE(capture.success) << capture.failure_code << ": " << capture.message;
  const auto unload = port.unload(capture.snapshot.unique_id);
  EXPECT_TRUE(unload.success) << unload.failure_code << ": " << unload.message;

  ComponentLoadRequest load_request;
  load_request.description.package_name = "isaac_ros_occupancy_grid_localizer";
  load_request.description.plugin_name =
    "nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode";
  load_request.description.node_name = "occupancy_grid_localizer";
  load_request.parameters = capture.snapshot.parameters;

  const auto load = port.load(load_request);
  EXPECT_FALSE(load.success);
  EXPECT_EQ(load.failure_code, "LOCALIZER_LOAD_TIMEOUT");
  std::this_thread::sleep_for(250ms);
  EXPECT_FALSE(fake_runtime->loaded());

}

TEST_F(
  RosComponentManagerPortTest,
  UnloadWaitsForExplicitAbsenceAcrossTransientGraphAmbiguity)
{
  auto client_node = std::make_shared<rclcpp::Node>("component_port_unload_recovery_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>();
  fake_runtime->set_transient_invalid_list_responses_after_unload(3U);
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions options;
  options.container_name = "/occupancy_grid_localizer_container";
  options.expected_full_node_name = "/occupancy_grid_localizer";
  options.operation_timeout = 2s;
  RosComponentManagerPort port(*client_node, callback_group, options);

  const auto preflight = port.preflight();
  ASSERT_TRUE(preflight.success) << preflight.failure_code << ": " << preflight.message;
  const auto unload = port.unload(preflight.unique_id);

  EXPECT_TRUE(unload.success) << unload.failure_code << ": " << unload.message;
  EXPECT_EQ(unload.resulting_presence, ComponentPresence::kAbsent);
  EXPECT_FALSE(fake_runtime->loaded());
  EXPECT_EQ(fake_runtime->unload_calls(), 1U);
}

TEST_F(
  RosComponentManagerPortTest,
  UnloadFailsClosedWhenGraphNeverProvesComponentAbsence)
{
  auto client_node = std::make_shared<rclcpp::Node>("component_port_unload_unknown_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>();
  fake_runtime->set_transient_invalid_list_responses_after_unload(1000U);
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions options;
  options.container_name = "/occupancy_grid_localizer_container";
  options.expected_full_node_name = "/occupancy_grid_localizer";
  options.operation_timeout = 350ms;
  RosComponentManagerPort port(*client_node, callback_group, options);

  const auto preflight = port.preflight();
  ASSERT_TRUE(preflight.success) << preflight.failure_code << ": " << preflight.message;
  const auto unload = port.unload(preflight.unique_id);

  EXPECT_FALSE(unload.success);
  EXPECT_EQ(unload.failure_code, "LOCALIZER_UNLOAD_NOT_CONFIRMED");
  EXPECT_EQ(unload.resulting_presence, ComponentPresence::kUnknown);
  EXPECT_FALSE(fake_runtime->loaded());
  EXPECT_EQ(fake_runtime->unload_calls(), 1U);
}

TEST_F(
  RosComponentManagerPortTest,
  LostUnloadResponseWithConfirmedAbsenceRemainsFailure)
{
  auto client_node = std::make_shared<rclcpp::Node>("component_port_unload_timeout_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>(0ms, 800ms);
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions options;
  options.container_name = "/occupancy_grid_localizer_container";
  options.expected_full_node_name = "/occupancy_grid_localizer";
  options.operation_timeout = 500ms;
  RosComponentManagerPort port(*client_node, callback_group, options);

  const auto preflight = port.preflight();
  ASSERT_TRUE(preflight.success) << preflight.failure_code << ": " << preflight.message;
  const auto unload = port.unload(preflight.unique_id);

  EXPECT_FALSE(unload.success);
  EXPECT_EQ(unload.failure_code, "LOCALIZER_UNLOAD_TIMEOUT");
  EXPECT_EQ(unload.resulting_presence, ComponentPresence::kAbsent);
  EXPECT_FALSE(fake_runtime->loaded());
  EXPECT_EQ(fake_runtime->unload_calls(), 1U);
  std::this_thread::sleep_for(400ms);
}

TEST_F(
  RosComponentManagerPortTest,
  BootstrapProvesExactLiveIsaacIdentityThroughRosWithoutMutation)
{
  fs::path asset_root;
  const fs::path runtime_context = create_ready_runtime_context(asset_root);
  const fs::path localizer_yaml =
    asset_root / "B11" / "F2" / "current" /
    "localizer" / "localizer_params.yaml";

  auto client_node = std::make_shared<rclcpp::Node>("component_bootstrap_test_client");
  auto fake_runtime = std::make_shared<FakeIsaacCompositionRuntime>();
  fake_runtime->set_map_parameters(
    fs::canonical(localizer_yaml).string(),
    "localizer_map.png",
    0.05,
    {1.0, -2.0, 0.3},
    0.65);
  auto callback_group =
    client_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(client_node);
  executor.add_node(fake_runtime);
  ExecutorSpinGuard spin_guard(executor);
  std::this_thread::sleep_for(300ms);

  RosComponentManagerOptions component_options;
  component_options.container_name = "/occupancy_grid_localizer_container";
  component_options.expected_full_node_name = "/occupancy_grid_localizer";
  component_options.operation_timeout = 3s;
  RosComponentManagerPort port(*client_node, callback_group, component_options);

  IsaacAssetReloaderOptions reloader_options;
  reloader_options.allowed_asset_root = asset_root;
  reloader_options.component.package_name =
    "isaac_ros_occupancy_grid_localizer";
  reloader_options.component.plugin_name =
    "nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode";
  reloader_options.component.node_name = "occupancy_grid_localizer";
  reloader_options.component.remap_rules = {
    "flatscan:=/flatscan",
    "localization_result:=/localization_result",
  };
  IsaacAssetReloader reloader(reloader_options);

  const ApplyFloorAssetResult result =
    reloader.bootstrap_from_runtime_context(runtime_context, port);

  ASSERT_TRUE(result.success) << result.state.failure_code << ": " << result.state.detail;
  EXPECT_TRUE(result.state.active_identity_valid);
  EXPECT_TRUE(result.state.localizer_ready);
  EXPECT_EQ(result.state.active_identity.building_id, "B11");
  EXPECT_EQ(result.state.active_identity.floor_id, "F2");
  EXPECT_EQ(result.state.active_identity.map_id, "map_f2");
  EXPECT_EQ(result.state.active_identity.asset_epoch, 17U);
  EXPECT_EQ(result.state.localizer_generation, 8U);
  EXPECT_TRUE(fake_runtime->loaded());
  EXPECT_EQ(fake_runtime->unload_calls(), 0U);
  EXPECT_EQ(fake_runtime->load_calls(), 0U);
}

}  // namespace
