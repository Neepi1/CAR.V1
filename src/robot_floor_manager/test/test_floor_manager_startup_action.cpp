// Real FloorSwitch action integration smoke. Only external ROS ports and the
// startup-owner filesystem protocol are simulated; no private node API is used.
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_map_asset_identity/map_asset_identity.hpp"
#include "../src/floor_manager_node.cpp"

namespace
{
using namespace std::chrono_literals;
using FloorSwitch = robot_interfaces::action::FloorSwitch;
using Begin = robot_interfaces::srv::BeginFloorTransition;
using Hold = robot_interfaces::srv::SetMotionHold;
using Pause = robot_interfaces::srv::SetCorrectionPause;
using Nav = nav2_msgs::action::NavigateToPose;
namespace fs = std::filesystem;

// Same canonical role bundle + epoch registry contract as ExactMapFixture in
// test_floor_manager_exact_selection.cpp. These files are never installed.
class StartupMapFixture
{
public:
  StartupMapFixture()
  {
    root = fs::temp_directory_path() / ("floor_startup_action_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    maps = root / "maps_release";
    bundle = maps / "B10" / "F2" / "maps" / "smoke_map";
    const std::vector<std::pair<std::string, std::string>> roles{
      {"nav_map_yaml", "nav/delivery.yaml"}, {"nav_map_pgm", "nav/delivery.pgm"},
      {"localizer_map_png", "localizer/delivery.png"},
      {"localizer_params_yaml", "localizer/delivery.yaml"},
      {"keepout_mask_yaml", "filters/keepout_mask.yaml"},
      {"keepout_mask_pgm", "filters/keepout_mask.pgm"},
      {"speed_mask_yaml", "filters/speed_mask.yaml"},
      {"speed_mask_pgm", "filters/speed_mask.pgm"},
      {"binary_mask_yaml", "filters/binary_mask.yaml"},
      {"binary_mask_pgm", "filters/binary_mask.pgm"},
      {"asset_report_json", "reports/asset_report.json"},
    };
    std::vector<robot_map_asset_identity::DigestEntry> entries;
    for (const auto & role : roles) {
      const auto content = role.first + "-smoke-content";
      write(bundle / role.second, content);
      entries.push_back({role.first, content});
    }
    write(bundle / "poses.yaml", "poses: []\n");
    digest = robot_map_asset_identity::canonical_map_asset_digest(entries);
    robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps);
    epoch = registry.bind({"B10", "F2", "smoke_map"}, digest).asset_epoch;
    std::ostringstream manifest;
    manifest << "{\"schema\":\"njrh.map_manifest.v2\",\"asset_epoch\":" << epoch
             << ",\"asset_digest_algorithm\":\"sha256\","
             << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
             << "\"asset_digest\":\"" << digest << "\",\"building_id\":\"B10\","
             << "\"floor_id\":\"F2\",\"map_id\":\"smoke_map\","
             << "\"safe_map_name\":\"delivery\"}";
    write(bundle / "manifest.json", manifest.str());
  }

  ~StartupMapFixture()
  {
    std::error_code ignored;
    fs::remove_all(root, ignored);
  }

  static void write(const fs::path & path, const std::string & text)
  {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);
    file << text;
    if (!file) {throw std::runtime_error("cannot write isolated smoke fixture");}
  }

  fs::path root;
  fs::path maps;
  fs::path bundle;
  std::string digest;
  std::uint64_t epoch{0U};
};

enum class StartupScenario {kCold, kHot, kUnknown, kForeignAck, kColdComplete, kSettledMapThenMaskFailure, kDelayedMapFailure};

class ScopedStartupRosRuntime
{
public:
  ScopedStartupRosRuntime()
  {
    const auto domain = std::getenv("ROS_DOMAIN_ID");
    const auto localhost = std::getenv("ROS_LOCALHOST_ONLY");
    if (!domain || std::string(domain) != "213" ||
      !localhost || std::string(localhost) != "1")
    {
      throw std::runtime_error("smoke requires ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1");
    }
    rclcpp::init(0, nullptr);
  }

  ~ScopedStartupRosRuntime()
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }
};

class StartupActionHarness
{
private:
  // Initialize the context before constructing any executor/node, and tear it
  // down only after every ROS object (including the real action worker) is gone.
  ScopedStartupRosRuntime runtime_;

public:
  explicit StartupActionHarness(const StartupScenario scenario)
  : scenario_(scenario), executor_(rclcpp::ExecutorOptions(), 6)
  {
    ports_ = std::make_shared<rclcpp::Node>("floor_startup_smoke_ports");
    navigator_ = std::make_shared<rclcpp::Node>("bt_navigator");
    controller_ = std::make_shared<rclcpp::Node>("controller_server");
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    hold_pub_ = ports_->create_publisher<robot_interfaces::msg::MotionInterlockState>(
      "/safety/motion_interlock_state", state_qos);
    pause_pub_ = ports_->create_publisher<robot_interfaces::msg::CorrectionPauseState>(
      "/localization/correction_pause_state", state_qos);
    health_pub_ = ports_->create_publisher<robot_interfaces::msg::LocalizationHealth>(
      "/localization/floor_health", state_qos);
    wheel_pub_ = ports_->create_publisher<nav_msgs::msg::Odometry>(
      "/wheel/odom", rclcpp::SensorDataQoS());
    local_pub_ = ports_->create_publisher<nav_msgs::msg::Odometry>(
      "/local_state/odometry", rclcpp::SensorDataQoS());

    hold_service_ = ports_->create_service<Hold>("/safety/set_motion_hold",
      [this](const std::shared_ptr<Hold::Request> request, std::shared_ptr<Hold::Response> response) {
        if (scenario_ == StartupScenario::kColdComplete &&
          request->operation == Hold::Request::OP_RELEASE)
        {
          try {
            const auto context = YAML::LoadFile((assets.root / "context.json").string());
            const auto handoff = YAML::LoadFile(request_path().string());
            release_after_durable_commit_.store(bridge_committed_ &&
              context["state"].as<std::string>() == "ready" &&
              context["confirmed"].as<bool>() &&
              context["transaction_id"].as<std::string>() == request->transaction_id &&
              context["asset_digest"].as<std::string>() == assets.digest &&
              handoff["state"].as<std::string>() == "committed");
          } catch (const std::exception &) {
            release_after_durable_commit_.store(false);
          }
        }
        hold_state_.hold_active = request->operation == Hold::Request::OP_ACQUIRE;
        final_hold_active_.store(hold_state_.hold_active);
        hold_state_.motion_blocked = hold_state_.hold_active;
        hold_state_.hold_keys = hold_state_.hold_active ?
          std::vector<std::string>{request->owner + ":" + request->transaction_id} :
          std::vector<std::string>{};
        response->success = true;
        response->applied_sequence = request->command_sequence;
        response->state = hold_state_;
      });
    pause_service_ = ports_->create_service<Pause>(
      "/robot_localization_bridge/set_correction_pause_lease",
      [this](const std::shared_ptr<Pause::Request> request, std::shared_ptr<Pause::Response> response) {
        pause_state_.paused = request->operation == Pause::Request::OP_ACQUIRE;
        final_pause_active_.store(pause_state_.paused);
        pause_state_.lease_keys = pause_state_.paused ?
          std::vector<std::string>{request->owner + ":" + request->transaction_id} :
          std::vector<std::string>{};
        response->success = true;
        response->applied_sequence = request->command_sequence;
        response->state = pause_state_;
      });
    begin_service_ = ports_->create_service<Begin>(
      "/robot_localization_bridge/begin_floor_transition",
      [this](const std::shared_ptr<Begin::Request> request, std::shared_ptr<Begin::Response> response) {
        if (request->operation == Begin::Request::OP_BEGIN) {
          {
            std::lock_guard<std::mutex> lock(observations_mutex_);
            observed_begin_ = *request;
            begin_was_held_ = hold_state_.hold_active && pause_state_.paused;
          }
          ++begin_calls_;
        }
        if (scenario_ == StartupScenario::kColdComplete ||
          scenario_ == StartupScenario::kSettledMapThenMaskFailure ||
          scenario_ == StartupScenario::kDelayedMapFailure)
        {
          response->applied_sequence = request->command_sequence;
          response->accepted_asset_epoch = assets.epoch;
          response->explicit_relocalization_sequence = triggered_ ? 7U : 6U;
          if (!exact_target(*request)) {
            response->result_code = Begin::Response::RESULT_IDENTITY_MISMATCH;
            response->message = "smoke bridge rejected foreign target";
            return;
          }
          if (request->operation == Begin::Request::OP_BEGIN) {
            bridge_begun_ = hold_state_.hold_active && pause_state_.paused;
            response->success = bridge_begun_;
            response->runtime_context_valid = false;
            return;
          }
          if (request->operation == Begin::Request::OP_COMMIT) {
            ++commit_calls_;
            bridge_committed_ = bridge_begun_ && triggered_ && amcl_ready_ &&
              runtime_ready_ack_.load() && global_clear_seen_ && local_clear_seen_ &&
              post_clear_costmaps_published_ && hold_state_.hold_active && !pause_state_.paused;
            response->success = bridge_committed_;
            response->runtime_context_valid = bridge_committed_;
            response->safe_for_goal_start = bridge_committed_;
            return;
          }
          response->success = true;
          response->runtime_context_valid = false;
          return;
        }
        // Controlled endpoint stop: prove actual BEGIN dispatch but do not
        // claim localization/costmap/COMMIT success this smoke does not cover.
        response->success = false;
        response->result_code = Begin::Response::RESULT_CONFLICT;
        response->runtime_context_valid = true;
        response->applied_sequence = request->command_sequence;
        response->message = "SMOKE_BEGIN_OBSERVED_NO_MUTATION";
      });
    bt_state_service_ = navigator_->create_service<lifecycle_msgs::srv::GetState>(
      "/bt_navigator/get_state", [this](
        const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
        std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        ++bt_probes_;
        response->current_state.id = nav_runtime_started_.load() ? 3U :
          (scenario_ == StartupScenario::kUnknown ? 0U : 1U);
      });
    controller_state_service_ = controller_->create_service<lifecycle_msgs::srv::GetState>(
      "/controller_server/get_state", [this](
        const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
        std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        ++controller_probes_;
        response->current_state.id = nav_runtime_started_.load() ? 3U : 2U;
      });
    if (scenario_ == StartupScenario::kHot || scenario_ == StartupScenario::kColdComplete) {
      status_pub_ = ports_->create_publisher<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", state_qos);
    }
    if (scenario_ == StartupScenario::kHot) {start_nav_runtime();}
    if (scenario_ == StartupScenario::kColdComplete ||
      scenario_ == StartupScenario::kSettledMapThenMaskFailure ||
      scenario_ == StartupScenario::kDelayedMapFailure)
    {create_target_runtime_ports();}

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("maps_root", assets.maps.string()),
      rclcpp::Parameter("live_floor_switch_enabled", true),
      rclcpp::Parameter("service_timeout_sec", 1.0),
      rclcpp::Parameter("evidence_timeout_sec", 2.0),
      rclcpp::Parameter("nav_idle_bootstrap_grace_sec", 0.3),
      rclcpp::Parameter("stopped_stable_duration_sec", 0.15),
      rclcpp::Parameter("startup_handoff_timeout_sec", 1.5),
      rclcpp::Parameter("startup_ready_timeout_sec", 3.0),
      rclcpp::Parameter("localizer_apply_timeout_sec", 3.0),
      rclcpp::Parameter("localization_trigger_timeout_sec", 3.0),
      rclcpp::Parameter("speed_filter_enabled", scenario_ == StartupScenario::kColdComplete),
      rclcpp::Parameter("motion_hold_sequence_state_file", (assets.root / "sequence").string()),
      rclcpp::Parameter("runtime_map_context_file", (assets.root / "context.json").string()),
      rclcpp::Parameter("startup_handoff_file", request_path().string()),
      rclcpp::Parameter("startup_handoff_ack_file", ack_path().string()),
    });
    floor_ = std::make_shared<FloorManagerNode>(options);
    client_ = rclcpp_action::create_client<FloorSwitch>(ports_, "/floor_manager/floor_switch");
    timer_ = ports_->create_wall_timer(40ms, [this]() {publish_inputs_and_startup_ack();});
    executor_.add_node(ports_);
    executor_.add_node(navigator_);
    executor_.add_node(controller_);
    executor_.add_node(floor_);
    spin_ = std::thread([this]() {executor_.spin();});
  }

  ~StartupActionHarness()
  {
    allow_map_response_.store(true);
    if (goal_) {
      try {
        auto canceled = client_->async_cancel_goal(goal_);
        (void)canceled.wait_for(500ms);
      } catch (const std::exception &) {}
    }
    executor_.cancel();
    if (spin_.joinable()) {spin_.join();}
    floor_.reset();
  }

  std::shared_ptr<FloorSwitch::Result> run(const std::string & transaction = "startup-smoke-tx")
  {
    if (!client_->wait_for_action_server(3s)) {return nullptr;}
    ack_written_.store(false);
    // Let the real subscriptions acquire fresh stopped odom/source evidence.
    std::this_thread::sleep_for(500ms);
    FloorSwitch::Goal goal;
    goal.transaction_id = transaction;
    goal.building_id = "B10";
    goal.floor_id = "F2";
    goal.map_id = "smoke_map";
    goal.expected_asset_epoch = assets.epoch;
    goal.expected_asset_digest = assets.digest;
    auto sent = client_->async_send_goal(goal);
    if (sent.wait_for(3s) != std::future_status::ready) {return nullptr;}
    goal_ = sent.get();
    if (!goal_) {return nullptr;}
    auto result = client_->async_get_result(goal_);
    if (result.wait_for(12s) != std::future_status::ready) {return nullptr;}
    const auto wrapped = result.get();
    result_code = wrapped.code;
    goal_.reset();
    return wrapped.result;
  }

  Begin::Request begin_request() const
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return observed_begin_;
  }

  bool begin_was_held() const
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return begin_was_held_;
  }

  fs::path request_path() const {return assets.root / "handoff.json";}
  fs::path ack_path() const {return assets.root / "handoff_ack.json";}
  StartupMapFixture assets;
  std::atomic_int begin_calls_{0};
  std::atomic_int bt_probes_{0};
  std::atomic_int controller_probes_{0};
  std::atomic_bool ack_written_{false};
  std::atomic_bool runtime_ready_ack_{false};
  std::atomic_int commit_calls_{0};
  std::atomic_int map_load_calls_{0};
  std::atomic_int mask_load_calls_{0};
  std::atomic_int mask_transition_calls_{0};
  std::atomic_int apply_calls_{0};
  std::atomic_int trigger_calls_{0};
  std::atomic_int clear_calls_{0};
  std::atomic_bool final_hold_active_{false};
  std::atomic_bool final_pause_active_{false};
  std::atomic_bool release_after_durable_commit_{false};
  std::atomic_bool allow_map_response_{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};

private:
  template<typename Request>
  bool exact_target(const Request & request) const
  {
    return request.transaction_id.rfind("startup-smoke-tx", 0U) == 0U && request.building_id == "B10" &&
      request.floor_id == "F2" && request.map_id == "smoke_map" &&
      request.asset_epoch == assets.epoch && request.asset_digest == assets.digest;
  }

  void start_nav_runtime()
  {
    if (nav_server_) {return;}
    nav_server_ = rclcpp_action::create_server<Nav>(navigator_, "/navigate_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal>) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>>) {});
    nav_runtime_started_.store(true);
  }

  void create_target_runtime_ports()
  {
    asset_pub_ = ports_->create_publisher<robot_interfaces::msg::LocalizerAssetState>(
      "/global_localization/asset_state", rclcpp::QoS(1).reliable().transient_local());
    global_costmap_pub_ = ports_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(1).reliable());
    local_costmap_pub_ = ports_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", rclcpp::QoS(1).reliable());
    map_callback_group_ = ports_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    map_service_ = ports_->create_service<nav2_msgs::srv::LoadMap>("/map_server/load_map",
      [this](const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
        ++map_load_calls_;
        if (scenario_ == StartupScenario::kDelayedMapFailure) {
          while (!allow_map_response_.load() && rclcpp::ok()) {
            std::this_thread::sleep_for(20ms);
          }
          response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
          return;
        }
        map_loaded_ = bridge_begun_ && hold_state_.hold_active && pause_state_.paused &&
          request->map_url == (assets.bundle / "nav/delivery.yaml").string();
        response->result = map_loaded_ ? nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS :
          nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      }, rmw_qos_profile_services_default, map_callback_group_);
    for (std::size_t index = 0U; index < 2U; ++index) {
      const auto stem = index == 0U ? std::string("keepout") : std::string("speed");
      const auto owner = "/" + stem + "_filter_mask_server";
      mask_get_services_.push_back(ports_->create_service<lifecycle_msgs::srv::GetState>(
        owner + "/get_state", [this, index](
          const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
          std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
          response->current_state.id = mask_states_[index];
        }));
      mask_change_services_.push_back(ports_->create_service<lifecycle_msgs::srv::ChangeState>(
        owner + "/change_state", [this, index](
          const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
          std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response) {
          ++mask_transition_calls_;
          if (scenario_ == StartupScenario::kSettledMapThenMaskFailure) {
            response->success = false;
            return;
          }
          if (mask_states_[index] == 1U && request->transition.id ==
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)
          {
            mask_states_[index] = 2U;
            response->success = true;
          } else if (mask_states_[index] == 2U && request->transition.id ==
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)
          {
            mask_states_[index] = 3U;
            response->success = true;
          }
        }));
      mask_load_services_.push_back(ports_->create_service<nav2_msgs::srv::LoadMap>(
        owner + "/load_map", [this, index, stem](
          const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
          std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
          ++mask_load_calls_;
          masks_loaded_[index] = map_loaded_ && mask_states_[index] == 3U &&
            hold_state_.hold_active && pause_state_.paused &&
            request->map_url == (assets.bundle / "filters" / (stem + "_mask.yaml")).string();
          response->result = masks_loaded_[index] ? nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS :
            nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
        }));
    }
    apply_service_ = ports_->create_service<robot_interfaces::srv::ApplyFloorAssets>(
      "/global_localization/apply_floor_assets", [this](
        const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Request> request,
        std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> response) {
        ++apply_calls_;
        applied_ = exact_target(*request) && masks_loaded_[0] && masks_loaded_[1] &&
          request->nav_map_yaml == (assets.bundle / "nav/delivery.yaml").string() &&
          request->localizer_map_png == (assets.bundle / "localizer/delivery.png").string() &&
          request->localizer_params_yaml == (assets.bundle / "localizer/delivery.yaml").string();
        response->success = applied_;
        response->reloaded = applied_;
        response->transaction_id = request->transaction_id;
        response->building_id = request->building_id;
        response->floor_id = request->floor_id;
        response->map_id = request->map_id;
        response->asset_epoch = request->asset_epoch;
        response->asset_digest = request->asset_digest;
        response->localizer_generation = 42U;
      });
    trigger_service_ = ports_->create_service<robot_interfaces::srv::TriggerLocalization>(
      "/global_localization/trigger", [this](
        const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request> request,
        std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response) {
        ++trigger_calls_;
        triggered_ = applied_ && !pause_state_.paused && hold_state_.hold_active &&
          request->reason == "floor_switch:B10/F2";
        response->accepted = triggered_;
      });
    global_clear_service_ = ports_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap", [this](
        const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
        std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>) {
        ++clear_calls_;
        global_clear_seen_ = nav_runtime_started_.load() && amcl_ready_;
      });
    local_clear_service_ = ports_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap", [this](
        const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
        std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>) {
        ++clear_calls_;
        local_clear_seen_ = nav_runtime_started_.load() && amcl_ready_;
      });
  }

  void publish_target_runtime_evidence()
  {
    robot_interfaces::msg::LocalizationHealth health;
    health.stamp = ports_->now();
    // Remembered counters do not constitute a healthy source. The new reload
    // and explicit localization must advance both counters during this action.
    health.localizer_generation = applied_ ? 42U : 41U;
    health.explicit_relocalization_sequence = triggered_ ? 7U : 6U;
    if (applied_) {
      robot_interfaces::msg::LocalizerAssetState asset;
      asset.stamp = ports_->now();
      asset.transaction_id = "startup-smoke-tx";
      asset.success = true;
      asset.reloaded = true;
      asset.localizer_ready = true;
      asset.active_identity_valid = true;
      asset.requested_building_id = asset.active_building_id = "B10";
      asset.requested_floor_id = asset.active_floor_id = "F2";
      asset.requested_map_id = asset.active_map_id = "smoke_map";
      asset.requested_asset_epoch = asset.active_asset_epoch = assets.epoch;
      asset.requested_asset_digest = asset.active_asset_digest = assets.digest;
      asset.localizer_generation = 42U;
      asset_pub_->publish(asset);
    }
    if (triggered_) {
      health.building_id = "B10";
      health.floor_id = "F2";
      health.map_id = "smoke_map";
      health.asset_epoch = assets.epoch;
      health.asset_digest = assets.digest;
      health.localizer_ready = true;
      health.bridge_ready = true;
      health.tf_unique = true;
      health.amcl_ready = amcl_ready_;
      health.transition_active = !bridge_committed_;
      health.runtime_context_valid = bridge_committed_;
      ++target_health_ticks_;
    }
    health_pub_->publish(health);
    if (global_clear_seen_ && local_clear_seen_) {
      nav_msgs::msg::OccupancyGrid map;
      map.header.stamp = ports_->now();
      map.header.frame_id = "map";
      map.info.width = map.info.height = 1U;
      map.info.resolution = 0.05F;
      map.info.origin.orientation.w = 1.0;
      map.data = {0};
      global_costmap_pub_->publish(map);
      local_costmap_pub_->publish(map);
      post_clear_costmaps_published_ = true;
    }
  }

  void publish_inputs_and_startup_ack()
  {
    hold_state_.stamp = ports_->now();
    ++hold_state_.generation;
    hold_pub_->publish(hold_state_);
    pause_state_.stamp = ports_->now();
    ++pause_state_.generation;
    pause_pub_->publish(pause_state_);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ports_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    wheel_pub_->publish(odom);
    local_pub_->publish(odom);
    if (nav_runtime_started_.load()) {
      status_pub_->publish(action_msgs::msg::GoalStatusArray{});
    }
    if (scenario_ == StartupScenario::kHot) {
      robot_interfaces::msg::LocalizationHealth health;
      health.stamp = ports_->now();
      health.building_id = "B10";
      health.floor_id = "F1";
      health.map_id = "healthy_source";
      health.asset_epoch = 9U;
      health.asset_digest = "sha256:" + std::string(64U, 'a');
      health.localizer_generation = 1U;
      health.explicit_relocalization_sequence = 1U;
      health.runtime_context_valid = true;
      health.localizer_ready = true;
      health.bridge_ready = true;
      health.tf_unique = true;
      health.amcl_ready = true;
      health_pub_->publish(health);
    }
    if (scenario_ == StartupScenario::kColdComplete) {publish_target_runtime_evidence();}
    const bool ready = scenario_ == StartupScenario::kColdComplete && triggered_ && target_health_ticks_ >= 2U;
    if ((ack_written_.load() && (!ready || runtime_ready_ack_.load())) ||
      !fs::exists(request_path())) {return;}
    try {
      const auto request = YAML::LoadFile(request_path().string());
      if (request["state"].as<std::string>() != "requested") {return;}
      if (ready) {
        const auto context = YAML::LoadFile((assets.root / "context.json").string());
        if (context["state"].as<std::string>() != "floor_switch_pending" ||
          context["transaction_id"].as<std::string>() != "startup-smoke-tx" ||
          context["asset_digest"].as<std::string>() != assets.digest) {return;}
        start_nav_runtime();
        amcl_ready_ = true;
      }
      std::ostringstream ack;
      ack << "{\"schema\":\"njrh.floor_startup_handoff_ack.v1\",\"version\":1,"
          << "\"state\":\"" << (ready ? "runtime_ready" : "adopted")
          << "\",\"failure\":\"\",\"detail\":\"smoke owner prepared exact target\",";
      for (const auto * key : {"transaction_id", "request_nonce", "building_id", "floor_id", "map_id", "asset_digest"}) {
        auto value = request[key].as<std::string>();
        if (scenario_ == StartupScenario::kForeignAck && std::string(key) == "map_id") {
          value += "_foreign";
        }
        ack << '"' << key << "\":\"" << value << "\",";
      }
      ack << "\"asset_epoch\":" << request["asset_epoch"].as<std::uint64_t>();
      if (ready) {
        ack << ",\"localizer_generation\":42,\"explicit_relocalization_sequence\":7";
      }
      ack << '}';
      const auto temporary = assets.root / "ack.tmp";
      StartupMapFixture::write(temporary, ack.str());
      fs::rename(temporary, ack_path());
      ack_written_.store(true);
      if (ready) {runtime_ready_ack_.store(true);}
    } catch (const std::exception &) {
      // An incomplete/unreadable request is never acknowledged; next tick retries.
    }
  }

  StartupScenario scenario_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_;
  rclcpp::Node::SharedPtr ports_, navigator_, controller_;
  std::shared_ptr<FloorManagerNode> floor_;
  rclcpp_action::Client<FloorSwitch>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<FloorSwitch>::SharedPtr goal_;
  rclcpp_action::Server<Nav>::SharedPtr nav_server_;
  rclcpp::Service<Hold>::SharedPtr hold_service_;
  rclcpp::Service<Pause>::SharedPtr pause_service_;
  rclcpp::Service<Begin>::SharedPtr begin_service_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr map_service_;
  std::vector<rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr> mask_load_services_;
  std::vector<rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr> mask_get_services_;
  std::vector<rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr> mask_change_services_;
  rclcpp::Service<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr apply_service_;
  rclcpp::Service<robot_interfaces::srv::TriggerLocalization>::SharedPtr trigger_service_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_service_, local_clear_service_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr bt_state_service_, controller_state_service_;
  rclcpp::Publisher<robot_interfaces::msg::MotionInterlockState>::SharedPtr hold_pub_;
  rclcpp::Publisher<robot_interfaces::msg::CorrectionPauseState>::SharedPtr pause_pub_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizationHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizerAssetState>::SharedPtr asset_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_, local_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_, local_pub_;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  robot_interfaces::msg::MotionInterlockState hold_state_;
  robot_interfaces::msg::CorrectionPauseState pause_state_;
  mutable std::mutex observations_mutex_;
  Begin::Request observed_begin_;
  bool begin_was_held_{false};
  std::atomic_bool nav_runtime_started_{false};
  std::array<std::uint8_t, 2U> mask_states_{{1U, 1U}};
  std::array<bool, 2U> masks_loaded_{{false, false}};
  bool bridge_begun_{false}, bridge_committed_{false}, map_loaded_{false};
  bool applied_{false}, triggered_{false}, amcl_ready_{false};
  bool global_clear_seen_{false}, local_clear_seen_{false}, post_clear_costmaps_published_{false};
  std::size_t target_health_ticks_{0U};
};

TEST(FloorManagerStartupAction, ColdInactiveWithoutHealthySourceAdoptsExactTargetBeforeBegin)
{
  StartupActionHarness harness(StartupScenario::kCold);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(harness.result_code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result->message.find("SMOKE_BEGIN_OBSERVED_NO_MUTATION"), std::string::npos)
    << result->message;
  EXPECT_EQ(harness.begin_calls_.load(), 1);
  EXPECT_TRUE(harness.ack_written_.load());
  EXPECT_TRUE(harness.begin_was_held());
  EXPECT_GT(harness.bt_probes_.load(), 0);
  EXPECT_GT(harness.controller_probes_.load(), 0);
  const auto begin = harness.begin_request();
  EXPECT_TRUE(begin.source_map_id.empty());
  EXPECT_EQ(begin.source_asset_epoch, 0U);
  EXPECT_EQ(begin.map_id, "smoke_map");
  EXPECT_EQ(begin.asset_epoch, harness.assets.epoch);
  EXPECT_EQ(begin.asset_digest, harness.assets.digest);
  EXPECT_EQ(YAML::LoadFile(harness.request_path().string())["state"].as<std::string>(), "failed");
}

TEST(FloorManagerStartupAction, HealthyHotActionIdleUsesExistingPathWithoutStartupHandoff)
{
  StartupActionHarness harness(StartupScenario::kHot);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_NE(result->message.find("SMOKE_BEGIN_OBSERVED_NO_MUTATION"), std::string::npos)
    << result->message;
  EXPECT_EQ(harness.begin_calls_.load(), 1);
  EXPECT_TRUE(harness.begin_was_held());
  EXPECT_EQ(harness.begin_request().source_map_id, "healthy_source");
  EXPECT_FALSE(fs::exists(harness.request_path()));
  EXPECT_FALSE(harness.ack_written_.load());
}

TEST(FloorManagerStartupAction, UnknownLifecycleNeverCreatesHandoffOrDispatchesBegin)
{
  StartupActionHarness harness(StartupScenario::kUnknown);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(harness.result_code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result->message.find("idle"), std::string::npos) << result->message;
  EXPECT_GT(harness.bt_probes_.load(), 0);
  EXPECT_GT(harness.controller_probes_.load(), 0);
  EXPECT_EQ(harness.begin_calls_.load(), 0);
  EXPECT_FALSE(fs::exists(harness.request_path()));
}

TEST(FloorManagerStartupAction, ForeignTargetAcknowledgementCannotAuthorizeBegin)
{
  StartupActionHarness harness(StartupScenario::kForeignAck);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(harness.result_code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(harness.ack_written_.load());
  EXPECT_EQ(harness.begin_calls_.load(), 0);
  EXPECT_NE(result->message.find("acknowledge exact target"), std::string::npos)
    << result->message;
  EXPECT_EQ(YAML::LoadFile(harness.request_path().string())["state"].as<std::string>(), "failed");
}

TEST(FloorManagerStartupAction, SettledTargetFailureReleasesOwnResourcesWithoutClaimingLocalization)
{
  StartupActionHarness harness(StartupScenario::kSettledMapThenMaskFailure);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(harness.result_code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(harness.map_load_calls_.load(), 1);
  EXPECT_EQ(harness.mask_transition_calls_.load(), 1);
  EXPECT_EQ(harness.apply_calls_.load(), 0);
  EXPECT_FALSE(result->runtime_context_valid);
  EXPECT_FALSE(result->recovery_required);
  EXPECT_FALSE(harness.final_hold_active_.load());
  EXPECT_FALSE(harness.final_pause_active_.load());
  const auto context = YAML::LoadFile((harness.assets.root / "context.json").string());
  EXPECT_FALSE(context["confirmed"].as<bool>());
  EXPECT_EQ(context["state"].as<std::string>(), "floor_switch_failed");
  const auto retried = harness.run("startup-smoke-tx-retry");
  ASSERT_NE(retried, nullptr);
  EXPECT_FALSE(retried->success);
  EXPECT_FALSE(retried->recovery_required);
  EXPECT_EQ(harness.begin_calls_.load(), 2);
  EXPECT_EQ(harness.map_load_calls_.load(), 2);
  EXPECT_FALSE(harness.final_hold_active_.load());
  EXPECT_FALSE(harness.final_pause_active_.load());
}

TEST(FloorManagerStartupAction, DelayedTargetRequestReturnsFailureThenAutomaticallyReleasesWhenSettled)
{
  StartupActionHarness harness(StartupScenario::kDelayedMapFailure);
  const auto started = std::chrono::steady_clock::now();
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_TRUE(result->recovery_required);
  EXPECT_TRUE(harness.final_hold_active_.load());
  EXPECT_LT(std::chrono::steady_clock::now() - started, 4s);
  const auto overlap = harness.run("startup-smoke-tx-overlap");
  ASSERT_NE(overlap, nullptr);
  EXPECT_FALSE(overlap->success);
  EXPECT_NE(overlap->message.find("TRANSACTION_CONFLICT"), std::string::npos);
  EXPECT_EQ(harness.begin_calls_.load(), 1);
  harness.allow_map_response_.store(true);
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while ((harness.final_hold_active_.load() || harness.final_pause_active_.load()) &&
    std::chrono::steady_clock::now() < deadline)
  {std::this_thread::sleep_for(20ms);}
  EXPECT_FALSE(harness.final_hold_active_.load());
  EXPECT_FALSE(harness.final_pause_active_.load());
  const auto retried = harness.run("startup-smoke-tx-retry");
  ASSERT_NE(retried, nullptr);
  EXPECT_FALSE(retried->success);
  EXPECT_FALSE(retried->recovery_required);
  EXPECT_EQ(harness.begin_calls_.load(), 2);
  EXPECT_FALSE(harness.final_hold_active_.load());
}

TEST(FloorManagerStartupAction, ColdExactTargetCompletesLocalizationCostmapsCommitAndHoldRelease)
{
  StartupActionHarness harness(StartupScenario::kColdComplete);
  const auto result = harness.run();
  ASSERT_NE(result, nullptr);
  ASSERT_TRUE(result->success) << result->message;
  EXPECT_EQ(harness.result_code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result->runtime_context_valid);
  EXPECT_FALSE(result->recovery_required);
  EXPECT_EQ(result->active_building_id, "B10");
  EXPECT_EQ(result->active_floor_id, "F2");
  EXPECT_EQ(result->active_map_id, "smoke_map");
  EXPECT_EQ(result->asset_epoch, harness.assets.epoch);
  EXPECT_EQ(result->asset_digest, harness.assets.digest);
  EXPECT_EQ(result->explicit_relocalization_sequence, 7U);
  EXPECT_TRUE(harness.begin_request().source_map_id.empty());
  EXPECT_EQ(harness.begin_calls_.load(), 1);
  EXPECT_EQ(harness.map_load_calls_.load(), 1);
  EXPECT_EQ(harness.mask_load_calls_.load(), 2);
  EXPECT_EQ(harness.mask_transition_calls_.load(), 4);
  EXPECT_EQ(harness.apply_calls_.load(), 1);
  EXPECT_EQ(harness.trigger_calls_.load(), 1);
  EXPECT_EQ(harness.clear_calls_.load(), 2);
  EXPECT_EQ(harness.commit_calls_.load(), 1);
  EXPECT_TRUE(harness.runtime_ready_ack_.load());
  EXPECT_FALSE(harness.final_hold_active_.load());
  EXPECT_FALSE(harness.final_pause_active_.load());
  EXPECT_TRUE(harness.release_after_durable_commit_.load());
  const auto context = YAML::LoadFile((harness.assets.root / "context.json").string());
  EXPECT_EQ(context["state"].as<std::string>(), "ready");
  EXPECT_TRUE(context["confirmed"].as<bool>());
  EXPECT_EQ(context["transaction_id"].as<std::string>(), "startup-smoke-tx");
  EXPECT_EQ(context["building_id"].as<std::string>(), "B10");
  EXPECT_EQ(context["floor_id"].as<std::string>(), "F2");
  EXPECT_EQ(context["map_id"].as<std::string>(), "smoke_map");
  EXPECT_EQ(context["asset_epoch"].as<std::uint64_t>(), harness.assets.epoch);
  EXPECT_EQ(context["asset_digest"].as<std::string>(), harness.assets.digest);
  EXPECT_EQ(context["localizer_generation"].as<std::uint64_t>(), 42U);
  EXPECT_EQ(context["explicit_relocalization_sequence"].as<std::uint64_t>(), 7U);
  const auto handoff = YAML::LoadFile(harness.request_path().string());
  EXPECT_EQ(handoff["state"].as<std::string>(), "committed");
  EXPECT_EQ(handoff["explicit_sequence_baseline"].as<std::uint64_t>(), 6U);
  EXPECT_GT(result->explicit_relocalization_sequence,
    handoff["explicit_sequence_baseline"].as<std::uint64_t>());
}
}  // namespace
