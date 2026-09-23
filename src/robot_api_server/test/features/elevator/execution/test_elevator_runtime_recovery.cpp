// Real production RuntimePort + real rclcpp clients. Run ONLY in the private
// network/IPC namespace created by executor_isolation/run_isolated.sh.
// There is deliberately no private-member access or prepare() bypass.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_interfaces/action/floor_switch.hpp>
#include <robot_interfaces/msg/correction_pause_state.hpp>
#include <robot_interfaces/msg/floor_switch_status.hpp>
#include <robot_interfaces/msg/localization_health.hpp>
#include <robot_interfaces/msg/localizer_asset_state.hpp>
#include <robot_interfaces/msg/motion_interlock_state.hpp>
#include <robot_interfaces/msg/operating_mode_state.hpp>
#include <robot_interfaces/srv/set_correction_pause.hpp>
#include <robot_interfaces/srv/set_elevator_navigation_session.hpp>
#include <robot_interfaces/srv/set_mode.hpp>
#include <robot_interfaces/srv/set_motion_hold.hpp>
#include "robot_api_server/features/elevator/execution/elevator_ros_runtime_port.hpp"
#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"

namespace robot_api_server {
namespace {
using namespace std::chrono_literals;
namespace fs = std::filesystem;
using Nav = nav2_msgs::action::NavigateToPose;
using Floor = robot_interfaces::action::FloorSwitch;
using NavHandle = rclcpp_action::ServerGoalHandle<Nav>;
using FloorHandle = rclcpp_action::ServerGoalHandle<Floor>;
using Effect = robot_elevator_manager::ElevatorRuntimeEffect;
using Kind = robot_elevator_manager::ElevatorEffectKind;
using Result = robot_elevator_manager::ElevatorRuntimeResult;
using Hold = robot_interfaces::srv::SetMotionHold;
using Pause = robot_interfaces::srv::SetCorrectionPause;
using Mode = robot_interfaces::srv::SetMode;
using Session = robot_interfaces::srv::SetElevatorNavigationSession;

template<class Predicate>
bool eventually(Predicate predicate, std::chrono::milliseconds budget = 6000ms) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  do {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(5ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

class RuntimeRecovery : public ::testing::Test {
protected:
  const std::string transaction{"isolated-elevator-recovery"};
  fs::path root;
  ElevatorRosRuntimeOptions options;
  robot_elevator_manager::FrozenElevatorRelease release;
  MapManifest source, target;
  rclcpp::Node::SharedPtr server;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread server_thread;
  std::unique_ptr<ElevatorRosRuntimePort> port;
  std::future<Result> work;
  std::mutex mutex;
  std::vector<std::shared_ptr<NavHandle>> nav_handles;
  std::vector<std::shared_ptr<FloorHandle>> floor_handles;
  std::vector<std::string> session_begins, session_ends;
  std::atomic<int> nav_cancels{0}, floor_cancels{0};
  std::atomic<bool> succeed_when_timeout_hold{false};
  std::atomic<bool> cancel_when_timeout_hold{false};
  std::atomic<bool> timeout_succeeded_original{false};
  std::atomic<bool> complete_nav_cancel{true};
  std::atomic<bool> succeed_when_caller_pause_released{false};
  std::atomic<bool> floor_succeeded_after_caller_release{false};
  robot_interfaces::msg::MotionInterlockState interlock;
  robot_interfaces::msg::OperatingModeState mode;
  robot_interfaces::msg::CorrectionPauseState pause;
  robot_interfaces::msg::FloorSwitchStatus floor_status;
  bool target_active{false};
  bool target_pose_available{true};
  bool pause_publication_enabled{true};
  rclcpp_action::Server<Nav>::SharedPtr nav_server;
  rclcpp_action::Server<Floor>::SharedPtr floor_server;
  rclcpp::Service<Hold>::SharedPtr hold_service;
  rclcpp::Service<Pause>::SharedPtr pause_service;
  rclcpp::Service<Mode>::SharedPtr mode_service;
  std::vector<rclcpp::Service<Session>::SharedPtr> sessions;
  rclcpp::Publisher<robot_interfaces::msg::MotionInterlockState>::SharedPtr interlock_pub;
  rclcpp::Publisher<robot_interfaces::msg::OperatingModeState>::SharedPtr mode_pub;
  rclcpp::Publisher<robot_interfaces::msg::CorrectionPauseState>::SharedPtr pause_pub;
  rclcpp::Publisher<robot_interfaces::msg::FloorSwitchStatus>::SharedPtr floor_pub;
  rclcpp::Publisher<robot_interfaces::msg::LocalizerAssetState>::SharedPtr asset_pub;
  rclcpp::Publisher<robot_interfaces::msg::LocalizationHealth>::SharedPtr health_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allowed_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub, odom_pub;
  rclcpp::TimerBase::SharedPtr timer;

  MapManifest map(const std::string & floor) {
    MapManifest manifest;
    manifest.map_id = "map_" + floor;
    manifest.display_name = "Isolated test " + floor;
    manifest.safe_map_name = "test_map";
    manifest.building_id = "B10";
    manifest.floor_id = floor;
    manifest.created_at = "2026-09-22T00:00:00Z";
    manifest.root = root / "maps" / "B10" / floor / "maps" / manifest.map_id;
    fill_manifest_paths(manifest);
    const std::vector<std::pair<fs::path, std::string>> files{
      {manifest.nav_map_yaml, "image: test_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"},
      {manifest.nav_map_pgm, "P5\n1 1\n255\nx"},
      {manifest.localizer_map_png, "isolated-png"},
      {manifest.localizer_params_yaml, "image: test_map.png\n"},
      {manifest.keepout_mask_yaml, "image: keepout_mask.pgm\n"},
      {manifest.keepout_mask_pgm, "keepout"},
      {manifest.speed_mask_yaml, "image: speed_mask.pgm\n"},
      {manifest.speed_mask_pgm, "speed"},
      {manifest.binary_mask_yaml, "image: binary_mask.pgm\n"},
      {manifest.binary_mask_pgm, "binary"},
      {manifest.asset_report_json, "{\"floor\":\"" + floor + "\"}\n"},
      {manifest.poses_yaml, "poses: []\n"},
    };
    for (const auto & [path, contents] : files) {
      fs::create_directories(path.parent_path());
      std::ofstream(path, std::ios::binary) << contents;
    }
    stamp_map_asset_identity(manifest, root / "maps");
    return manifest;
  }

  static void set_key(std::vector<std::string> & keys, const std::string & key, bool on) {
    keys.erase(std::remove(keys.begin(), keys.end(), key), keys.end());
    if (on) {keys.push_back(key);}
  }

  virtual double navigation_timeout_sec() const {return 20.0;}

  void SetUp() override {
    // The runner creates new network/IPC namespaces and sets this guard. A
    // domain ID alone is NOT isolation and must never suffice for this test.
    ASSERT_NE(std::getenv("NJRH_ELEVATOR_TEST_ISOLATED"), nullptr);
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    root = fs::temp_directory_path() / ("elevator-runtime-recovery-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
    source = map("F1"); target = map("F2");
    release.release_id = "isolated-release";
    release.generation = 1U; release.schema_version = 2U;
    release.building_id = "B10"; release.elevator_id = "lift";
    const auto runtime_floor = [](const MapManifest & m) {
      robot_elevator_manager::ElevatorRuntimeFloor f;
      f.floor_id = m.floor_id; f.map_id = m.map_id;
      f.map_asset_epoch = m.asset_epoch; f.map_asset_digest = m.asset_digest;
      f.poses = {{robot_elevator_manager::PoseRole::kHallCall, "hall", 1.0, 0.0, 0.0},
        {robot_elevator_manager::PoseRole::kLanding, "landing", 2.0, 0.0, 0.0},
        {robot_elevator_manager::PoseRole::kCabin, "cabin", 3.0, 0.0, 0.0}};
      return f;
    };
    release.source = runtime_floor(source); release.target = runtime_floor(target);
    options.maps_root = root / "maps";
    options.runtime_map_context_file = root / "context.json";
    options.hold_sequence_state_file = root / "sequence.state";
    options.persistent_recovery_lock_enabled = false;
    options.runtime_idle_probe = [] {return std::array<bool, 3>{true, true, true};};
    options.delayed_side_effect_unknown_probe = [] {return 0U;};
    options.current_map_pose_probe = [this]() -> std::optional<ElevatorMapPose> {
      std::lock_guard<std::mutex> lock(mutex);
      if (target_active && !target_pose_available) {return std::nullopt;}
      ElevatorMapPose p; p.x = 0.0; p.y = 0.0; p.yaw = 0.0;
      p.stamp_sec = server->now().seconds(); p.age_sec = 0.0;
      return p;
    };
    options.endpoint_timeout_sec = 2.0; options.service_timeout_sec = 2.0;
    options.stop_timeout_sec = 2.0; options.stop_settle_sec = 0.05;
    options.navigation_idle_stable_sec = 0.03;
    options.navigation_timeout_sec = navigation_timeout_sec(); options.floor_switch_timeout_sec = 10.0;
    options.target_map_pose_settle_sec = 0.05;
    write_runtime_map_context_file(options.runtime_map_context_file, source, "ready", true, "fixture", 1.0);
    server = std::make_shared<rclcpp::Node>("isolated_elevator_runtime_servers");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    interlock_pub = server->create_publisher<robot_interfaces::msg::MotionInterlockState>(options.motion_interlock_topic, qos);
    mode_pub = server->create_publisher<robot_interfaces::msg::OperatingModeState>(options.operating_mode_topic, qos);
    pause_pub = server->create_publisher<robot_interfaces::msg::CorrectionPauseState>(options.correction_pause_topic, qos);
    floor_pub = server->create_publisher<robot_interfaces::msg::FloorSwitchStatus>(options.floor_switch_status_topic, qos);
    asset_pub = server->create_publisher<robot_interfaces::msg::LocalizerAssetState>(options.localizer_asset_state_topic, qos);
    health_pub = server->create_publisher<robot_interfaces::msg::LocalizationHealth>(options.localization_health_topic, qos);
    allowed_pub = server->create_publisher<std_msgs::msg::Bool>(options.motion_allowed_topic, qos);
    safety_pub = server->create_publisher<std_msgs::msg::String>(options.safety_status_topic, qos);
    wheel_pub = server->create_publisher<nav_msgs::msg::Odometry>(options.wheel_odom_topic, rclcpp::SensorDataQoS());
    odom_pub = server->create_publisher<nav_msgs::msg::Odometry>(options.local_odom_topic, rclcpp::SensorDataQoS());
    nav_server = rclcpp_action::create_server<Nav>(server, options.navigate_to_pose_action,
      [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](auto) {++nav_cancels; return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h) {std::lock_guard<std::mutex> lock(mutex); nav_handles.push_back(h);});
    floor_server = rclcpp_action::create_server<Floor>(server, options.floor_switch_action,
      [](const auto &, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](auto) {++floor_cancels; return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto h) {std::lock_guard<std::mutex> lock(mutex); floor_handles.push_back(h);});
    hold_service = server->create_service<Hold>(options.motion_hold_service,
      [this](Hold::Request::SharedPtr q, Hold::Response::SharedPtr r) {
        std::lock_guard<std::mutex> lock(mutex);
        if (q->reason == "navigation_timeout" && succeed_when_timeout_hold.exchange(false)) {
          if (nav_handles.size() == 1U && nav_handles.front()->is_executing()) {
            nav_handles.front()->succeed(std::make_shared<Nav::Result>());
            if (cancel_when_timeout_hold.load()) {port->request_cancel(transaction);}
            timeout_succeeded_original.store(true);
          }
        }
        set_key(interlock.hold_keys, q->owner + ":" + q->transaction_id, q->operation == Hold::Request::OP_ACQUIRE);
        interlock.hold_active = !interlock.hold_keys.empty();
        interlock.motion_blocked = interlock.hold_active;
        interlock.interlock_effective_motion_blocked = interlock.hold_active;
        ++interlock.generation;
        r->success = true; r->applied_sequence = q->command_sequence; r->state = interlock;
      });
    pause_service = server->create_service<Pause>(options.correction_pause_service,
      [this](Pause::Request::SharedPtr q, Pause::Response::SharedPtr r) {
        std::shared_ptr<FloorHandle> committed_while_reply_pending;
        {
          std::lock_guard<std::mutex> lock(mutex);
          set_key(pause.lease_keys, q->owner + ":" + q->transaction_id, q->operation == Pause::Request::OP_ACQUIRE);
          pause.paused = !pause.lease_keys.empty(); ++pause.generation;
          if (q->operation == Pause::Request::OP_RELEASE &&
            q->owner == "robot_elevator_manager" &&
            succeed_when_caller_pause_released.exchange(false) && !floor_handles.empty())
          {
            committed_while_reply_pending = floor_handles.back();
          }
        }
        // Simulate a real ordering: the caller lease is removed first, allowing
        // the same floor action to commit while its release reply is pending.
        if (committed_while_reply_pending) {
          succeed_floor(committed_while_reply_pending);
          floor_succeeded_after_caller_release.store(true);
        }
        std::lock_guard<std::mutex> lock(mutex);
        r->success = true; r->applied_sequence = q->command_sequence; r->state = pause;
      });
    mode_service = server->create_service<Mode>(options.mode_service,
      [this](Mode::Request::SharedPtr q, Mode::Response::SharedPtr r) {
        std::lock_guard<std::mutex> lock(mutex);
        mode.lease_active = q->operation == Mode::Request::OP_SET;
        mode.mode = mode.lease_active ? q->mode : "NORMAL";
        mode.owner = q->owner; mode.mission_id = q->mission_id; mode.lease_id = q->lease_id;
        r->success = true; r->state = mode;
      });
    for (const auto * id : {"ElevatorHallFollowPath", "ElevatorFollowPath", "ElevatorReverseEntryStagingFollowPath",
        "ElevatorReverseDockingFollowPath", "ElevatorCabinEntryDirectFollowPath", "ElevatorCabinPanelFollowPath"}) {
      sessions.push_back(server->create_service<Session>(options.elevator_controller_session_service_prefix + "/" + id + "/execution_session",
        [this](Session::Request::SharedPtr q, Session::Response::SharedPtr r) {
          std::lock_guard<std::mutex> lock(mutex);
          (q->operation == Session::Request::OP_BEGIN ? session_begins : session_ends).push_back(q->session_id);
          r->accepted = true; r->reset_performed = q->operation == Session::Request::OP_BEGIN;
          r->active_session_id = q->session_id;
        }));
    }
    interlock.execution_mode_contract_valid = true; interlock.normal_source_only = true;
    timer = server->create_wall_timer(20ms, [this] {publish();});
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(server);
    server_thread = std::thread([this] {executor->spin();});
    port = std::make_unique<ElevatorRosRuntimePort>(options);
    const auto prepared = port->prepare(transaction, release);
    ASSERT_TRUE(prepared.success) << prepared.code << ": " << prepared.detail;
  }

  void publish() {
    std::lock_guard<std::mutex> lock(mutex);
    const auto stamp = server->now();
    interlock.stamp = stamp; interlock_pub->publish(interlock);
    mode.stamp = stamp; mode_pub->publish(mode);
    pause.stamp = stamp;
    if (pause_publication_enabled) {pause_pub->publish(pause);}
    floor_status.stamp = stamp; floor_pub->publish(floor_status);
    const auto & m = target_active ? target : source;
    robot_interfaces::msg::LocalizerAssetState a;
    a.stamp = stamp; a.success = true; a.active_identity_valid = true;
    a.active_building_id = m.building_id; a.active_floor_id = m.floor_id; a.active_map_id = m.map_id;
    a.active_asset_epoch = m.asset_epoch; a.active_asset_digest = m.asset_digest;
    a.localizer_generation = 1U; a.localizer_ready = true; asset_pub->publish(a);
    robot_interfaces::msg::LocalizationHealth h;
    h.stamp = stamp; h.building_id = m.building_id; h.floor_id = m.floor_id; h.map_id = m.map_id;
    h.asset_epoch = m.asset_epoch; h.asset_digest = m.asset_digest; h.localizer_generation = 1U;
    h.explicit_relocalization_sequence = 1U;
    h.localizer_ready = h.bridge_ready = h.tf_unique = h.runtime_context_valid = h.amcl_ready = true;
    health_pub->publish(h);
    std_msgs::msg::Bool allowed; allowed.data = !interlock.hold_active; allowed_pub->publish(allowed);
    std_msgs::msg::String safety; safety.data = "OK"; safety_pub->publish(safety);
    nav_msgs::msg::Odometry odom; odom.header.stamp = stamp; odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link"; odom.pose.pose.orientation.w = 1.0;
    wheel_pub->publish(odom); odom_pub->publish(odom);
    for (const auto & goal : nav_handles) {
      if (complete_nav_cancel.load() && goal->is_canceling()) {
        goal->canceled(std::make_shared<Nav::Result>());
      }
    }
    for (const auto & goal : floor_handles) {
      if (goal->is_canceling()) {goal->canceled(std::make_shared<Floor::Result>());}
    }
  }

  Effect effect(Kind kind, bool target_floor = false) const {
    const auto & f = target_floor ? release.target : release.source;
    Effect e; e.effect.kind = kind; e.effect.sequence = 7U; e.effect.transaction_id = transaction;
    e.effect.floor_id = e.floor_id = f.floor_id; e.effect.map_id = e.map_id = f.map_id;
    e.building_id = release.building_id; e.elevator_id = release.elevator_id;
    e.asset_epoch = f.map_asset_epoch; e.asset_digest = f.map_asset_digest;
    if (kind == Kind::kNavigateToPose) {
      e.effect.pose_id = "hall";
      e.effect.navigation_intent = robot_elevator_manager::ElevatorNavigationIntent::kHallCall;
      e.target_pose = f.poses.front();
    }
    return e;
  }
  void start(Effect e) {work = std::async(std::launch::async, [this, e] {return port->apply(e);});}
  std::size_t nav_count() {std::lock_guard<std::mutex> lock(mutex); return nav_handles.size();}
  std::shared_ptr<NavHandle> nav(std::size_t n) {std::lock_guard<std::mutex> lock(mutex); return nav_handles.at(n);}
  std::size_t floor_count() {std::lock_guard<std::mutex> lock(mutex); return floor_handles.size();}
  std::shared_ptr<FloorHandle> floor(std::size_t n) {std::lock_guard<std::mutex> lock(mutex); return floor_handles.at(n);}
  void floor_resources(const std::string & id, bool held) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto key = "robot_floor_manager:" + id;
    set_key(interlock.hold_keys, key, held);
    set_key(pause.lease_keys, key, held);
    pause.paused = !pause.lease_keys.empty();
    interlock.hold_active = !interlock.hold_keys.empty();
    interlock.motion_blocked = interlock.interlock_effective_motion_blocked = interlock.hold_active;
  }
  void failed_floor_status(const std::string & id, bool cleaned) {
    std::lock_guard<std::mutex> lock(mutex);
    floor_status.transaction_id = id;
    floor_status.state = floor_status.stage = cleaned ? "FAILED" : "FAILED_LOCKED";
    floor_status.failure_code = Floor::Result::EVIDENCE_STALE;
    ++floor_status.generation;
  }
  void handoff_floor(const std::shared_ptr<FloorHandle> & h) {
    floor_resources(h->get_goal()->transaction_id, true);
    {
      std::lock_guard<std::mutex> lock(mutex);
      floor_status.transaction_id = h->get_goal()->transaction_id;
      floor_status.state = "SWITCHING";
      floor_status.stage = "CALLER_PAUSE_HANDOFF_READY";
      floor_status.failure_code = 0U;
      ++floor_status.generation;
    }
    auto feedback = std::make_shared<Floor::Feedback>();
    feedback->transaction_id = h->get_goal()->transaction_id;
    feedback->stage = "CALLER_PAUSE_HANDOFF_READY";
    feedback->stage_sequence = 1U;
    feedback->caller_pause_handoff_ready = true;
    h->publish_feedback(feedback);
  }
  void succeed_floor(const std::shared_ptr<FloorHandle> & h) {
    floor_resources(h->get_goal()->transaction_id, false);
    {
      std::lock_guard<std::mutex> lock(mutex);
      target_active = true;
      floor_status.transaction_id = h->get_goal()->transaction_id;
      floor_status.state = floor_status.stage = "COMPLETE";
      floor_status.requested_building_id = floor_status.active_building_id = target.building_id;
      floor_status.requested_floor_id = floor_status.active_floor_id = target.floor_id;
      floor_status.requested_map_id = floor_status.active_map_id = target.map_id;
      floor_status.requested_asset_epoch = floor_status.asset_epoch = target.asset_epoch;
      floor_status.requested_asset_digest = floor_status.asset_digest = target.asset_digest;
      floor_status.active_context_valid = floor_status.nav_map_ready = floor_status.filters_ready = true;
      floor_status.localizer_ready = floor_status.bridge_ready = floor_status.amcl_ready = true;
      floor_status.costmaps_ready = floor_status.nav2_ready = true;
      floor_status.failure_code = 0U;
      ++floor_status.generation;
    }
    write_runtime_map_context_file(options.runtime_map_context_file, target, "ready", true, "fixture switched", 2.0);
    auto result = std::make_shared<Floor::Result>();
    result->success = true; result->runtime_context_valid = true;
    result->active_building_id = target.building_id; result->active_floor_id = target.floor_id;
    result->active_map_id = target.map_id; result->asset_epoch = target.asset_epoch;
    result->asset_digest = target.asset_digest; result->explicit_relocalization_sequence = 2U;
    h->succeed(result);
  }
  void TearDown() override {
    if (port) {port->request_cancel(transaction);}
    if (work.valid()) {
      if (work.wait_for(8s) == std::future_status::ready) {work.get();}
      else {
        // Fail the isolated test process, not a false clean teardown: deleting
        // the port under its still-running apply() would introduce a test UAF.
        std::fputs("FATAL: RuntimePort worker did not exit after cancel\n", stderr);
        std::fflush(stderr);
        std::_Exit(86);
      }
    }
    port.reset();
    if (executor) {executor->cancel();}
    if (server_thread.joinable()) {server_thread.join();}
    executor.reset(); server.reset();
    // Keep failed test artifacts for diagnosis; only remove this exact fixture
    // directory on success, never production report or workspace directories.
    if (!HasFailure() && !root.empty()) {std::error_code ec; fs::remove_all(root, ec);}
  }
};

class TimeoutRuntimeRecovery : public RuntimeRecovery {
protected:
  double navigation_timeout_sec() const override {return 0.2;}
};

TEST_F(TimeoutRuntimeRecovery, SuccessDuringTimeoutHoldMustNotResubmit) {
  succeed_when_timeout_hold.store(true);
  start(effect(Kind::kNavigateToPose));
  ASSERT_TRUE(eventually([&] {return timeout_succeeded_original.load();}));
  ASSERT_TRUE(eventually([&] {
    return nav_count() >= 2U || work.wait_for(0ms) == std::future_status::ready;
  }));
  EXPECT_EQ(nav_count(), 1U) << "original goal SUCCEEDED during timeout hold; must not resend";
  if (work.wait_for(0ms) != std::future_status::ready) {port->request_cancel(transaction);}
  ASSERT_EQ(work.wait_for(6s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_TRUE(result.success) << result.code << ": " << result.detail;
  EXPECT_EQ(nav_cancels.load(), 0);
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(session_begins.size(), 1U); EXPECT_EQ(session_ends.size(), 1U);
}

TEST_F(TimeoutRuntimeRecovery, OperatorCancelWinsOverSuccessDuringTimeoutHold) {
  succeed_when_timeout_hold.store(true);
  cancel_when_timeout_hold.store(true);
  start(effect(Kind::kNavigateToPose));
  ASSERT_TRUE(eventually([&] {return timeout_succeeded_original.load();}));
  ASSERT_EQ(work.wait_for(6s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "ELEVATOR_NAV2_CANCELED");
  EXPECT_EQ(nav_count(), 1U); EXPECT_EQ(nav_cancels.load(), 0);
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(session_begins.size(), 1U); EXPECT_EQ(session_ends.size(), 1U);
}

TEST_F(TimeoutRuntimeRecovery, TimeoutCancellationRetriesOnlyAfterOriginalTerminal) {
  start(effect(Kind::kNavigateToPose));
  ASSERT_TRUE(eventually([&] {return nav_count() == 2U;}));
  EXPECT_FALSE(nav(0)->is_active());
  EXPECT_EQ(nav_cancels.load(), 1);
  nav(1)->succeed(std::make_shared<Nav::Result>());
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_TRUE(result.success) << result.code << ": " << result.detail;
  EXPECT_EQ(nav_count(), 2U); EXPECT_EQ(nav_cancels.load(), 1);
}

TEST_F(TimeoutRuntimeRecovery, TimeoutWithoutTerminalResultDoesNotResubmit) {
  complete_nav_cancel.store(false);
  start(effect(Kind::kNavigateToPose));
  ASSERT_EQ(work.wait_for(6s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "ELEVATOR_NAV2_TERMINAL_UNPROVEN");
  EXPECT_EQ(nav_count(), 1U); EXPECT_EQ(nav_cancels.load(), 1);
  // Let only this fixture's original goal terminate before tearing it down.
  complete_nav_cancel.store(true);
  ASSERT_TRUE(eventually([&] {return !nav(0)->is_active();}));
}

TEST_F(RuntimeRecovery, AbortedNavigationRetriesSameEffectAndCompletes) {
  const auto e = effect(Kind::kNavigateToPose);
  start(e);
  ASSERT_TRUE(eventually([&] {return nav_count() == 1U;}));
  const auto first = nav(0);
  first->abort(std::make_shared<Nav::Result>());
  ASSERT_TRUE(eventually([&] {return nav_count() == 2U;}));
  const auto second = nav(1);
  EXPECT_NE(first->get_goal_id(), second->get_goal_id());
  EXPECT_EQ(first->get_goal()->pose.pose, second->get_goal()->pose.pose);
  EXPECT_EQ(first->get_goal()->behavior_tree, second->get_goal()->behavior_tree);
  EXPECT_EQ(first->get_goal()->pose.header.frame_id, "map");
  second->succeed(std::make_shared<Nav::Result>());
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_TRUE(result.success) << result.code << ": " << result.detail;
  EXPECT_EQ(nav_count(), 2U); EXPECT_EQ(nav_cancels.load(), 0);
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(session_begins.size(), 2U); EXPECT_NE(session_begins[0], session_begins[1]);
  EXPECT_EQ(session_ends.size(), 2U);
}

TEST_F(RuntimeRecovery, CancelDuringRetryWaitDoesNotResubmitNavigation) {
  start(effect(Kind::kNavigateToPose));
  ASSERT_TRUE(eventually([&] {return nav_count() == 1U;}));
  nav(0)->abort(std::make_shared<Nav::Result>());
  ASSERT_TRUE(eventually([&] {std::lock_guard<std::mutex> lock(mutex); return !session_ends.empty();}));
  port->request_cancel(transaction);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.code.find("CANCEL"), std::string::npos) << result.code;
  EXPECT_EQ(nav_count(), 1U); EXPECT_EQ(nav_cancels.load(), 0);
}

TEST_F(RuntimeRecovery, FloorRetryWaitsForCleanupThenNewIdAndVerifyNeverResubmits) {
  auto held = port->apply(effect(Kind::kAcquireSafetyHold));
  ASSERT_TRUE(held.success) << held.detail;
  auto paused = port->apply(effect(Kind::kPauseLocalizationCorrections));
  ASSERT_TRUE(paused.success) << paused.detail;
  ASSERT_TRUE(eventually([&] {return port->poll_health(transaction).success;}));
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  const auto first = floor(0);
  const auto old_id = first->get_goal()->transaction_id;
  floor_resources(old_id, true);
  failed_floor_status(old_id, false);
  auto failure = std::make_shared<Floor::Result>();
  failure->failure_code = Floor::Result::EVIDENCE_STALE;
  failure->recovery_required = true;
  failure->message = "isolated transient failure with deferred cleanup";
  first->abort(failure);
  // Not merely backoff: verify both sides of the old-resource cleanup contract.
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(floor_count(), 1U);
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  floor_resources(old_id, false);
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(floor_count(), 1U); // absence alone is not deferred-cleanup completion
  failed_floor_status(old_id, true);
  ASSERT_TRUE(eventually([&] {return floor_count() == 2U;}));
  const auto second = floor(1);
  EXPECT_NE(old_id, second->get_goal()->transaction_id);
  EXPECT_EQ(first->get_goal()->building_id, second->get_goal()->building_id);
  EXPECT_EQ(first->get_goal()->floor_id, second->get_goal()->floor_id);
  EXPECT_EQ(first->get_goal()->map_id, second->get_goal()->map_id);
  EXPECT_EQ(first->get_goal()->expected_asset_digest, second->get_goal()->expected_asset_digest);
  handoff_floor(second);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  auto begun = work.get();
  ASSERT_TRUE(begun.success) << begun.code << ": " << begun.detail;
  const auto resumed = port->apply(effect(Kind::kResumeLocalizationCorrections, true));
  ASSERT_TRUE(resumed.success) << resumed.code << ": " << resumed.detail;
  start(effect(Kind::kSwitchFloor, true));
  succeed_floor(second);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto switched = work.get();
  ASSERT_TRUE(switched.success) << switched.code << ": " << switched.detail;
  {
    std::lock_guard<std::mutex> lock(mutex); target_pose_available = false;
  }
  start(effect(Kind::kVerifyFloorReady, true));
  std::this_thread::sleep_for(3500ms); // exceed the per-verification pose check
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  EXPECT_EQ(floor_count(), 2U);
  {
    std::lock_guard<std::mutex> lock(mutex); target_pose_available = true;
  }
  ASSERT_EQ(work.wait_for(6s), std::future_status::ready);
  const auto verified = work.get();
  EXPECT_TRUE(verified.success) << verified.code << ": " << verified.detail;
  EXPECT_EQ(floor_count(), 2U); EXPECT_EQ(floor_cancels.load(), 0);
  EXPECT_EQ(nav_count(), 0U); // recovery did not rerun any earlier navigation
}

TEST_F(RuntimeRecovery, CancelWhileFloorCleanupPendingNeverSubmitsAnotherGoal) {
  ASSERT_TRUE(port->apply(effect(Kind::kAcquireSafetyHold)).success);
  ASSERT_TRUE(port->apply(effect(Kind::kPauseLocalizationCorrections)).success);
  ASSERT_TRUE(eventually([&] {return port->poll_health(transaction).success;}));
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  const auto first = floor(0);
  floor_resources(first->get_goal()->transaction_id, true);
  failed_floor_status(first->get_goal()->transaction_id, false);
  auto result = std::make_shared<Floor::Result>();
  result->failure_code = Floor::Result::EVIDENCE_STALE; result->recovery_required = true;
  first->abort(result);
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  port->request_cancel(transaction);
  ASSERT_EQ(work.wait_for(3s), std::future_status::ready);
  const auto canceled = work.get();
  EXPECT_FALSE(canceled.success);
  EXPECT_NE(canceled.code.find("CANCEL"), std::string::npos) << canceled.code;
  EXPECT_EQ(floor_count(), 1U); EXPECT_EQ(floor_cancels.load(), 0);
}

TEST_F(RuntimeRecovery, ExactAttemptStartupOwnerExitEndsCleanupWaitWithoutRetry) {
  ASSERT_TRUE(port->apply(effect(Kind::kAcquireSafetyHold)).success);
  ASSERT_TRUE(port->apply(effect(Kind::kPauseLocalizationCorrections)).success);
  ASSERT_TRUE(eventually([&] {return port->poll_health(transaction).success;}));
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  const auto first = floor(0);
  const auto id = first->get_goal()->transaction_id;
  floor_resources(id, true);
  failed_floor_status(id, false);
  auto result = std::make_shared<Floor::Result>();
  result->failure_code = Floor::Result::EVIDENCE_STALE;
  result->recovery_required = true;
  result->message = "original localization failure";
  first->abort(result);
  const auto owner_exit = [&](const std::string & event_id, const std::string & detail) {
    robot_interfaces::msg::FloorSwitchStatus event;
    event.stamp = server->now(); event.transaction_id = event_id;
    event.state = "FAILED_LOCKED"; event.stage = "RECOVERY_LOCKED";
    event.failure_code = Floor::Result::EVIDENCE_STALE; event.detail = detail;
    // One terminal event, not a heartbeat. The normal timer will immediately
    // publish another status sample without this detail; its proof must persist.
    floor_pub->publish(event);
  };
  owner_exit(id + "-previous-attempt", "STARTUP_OWNER_EXITED; wrong transaction");
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  owner_exit(id, "STARTUP_OWNER_EXITED_WITH_OTHER_REASON; not the contract");
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  owner_exit(id, "STARTUP_OWNER_EXITED; original startup owner exited without settlement");
  ASSERT_EQ(work.wait_for(3s), std::future_status::ready);
  const auto failed = work.get();
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(failed.code, "ELEVATOR_FLOOR_STARTUP_OWNER_EXITED");
  EXPECT_NE(failed.detail.find("original startup owner exited without settlement"), std::string::npos);
  EXPECT_NE(failed.detail.find("original localization failure"), std::string::npos);
  EXPECT_EQ(floor_count(), 1U); EXPECT_EQ(floor_cancels.load(), 0);
  std::lock_guard<std::mutex> lock(mutex);
  const auto key = "robot_floor_manager:" + id;
  EXPECT_NE(std::find(interlock.hold_keys.begin(), interlock.hold_keys.end(), key), interlock.hold_keys.end());
  EXPECT_NE(std::find(pause.lease_keys.begin(), pause.lease_keys.end(), key), pause.lease_keys.end());
}

TEST_F(RuntimeRecovery, CommittedOriginalFloorGoalDuringCallerReleaseResponseIsNotRepeated) {
  ASSERT_TRUE(port->apply(effect(Kind::kAcquireSafetyHold)).success);
  ASSERT_TRUE(port->apply(effect(Kind::kPauseLocalizationCorrections)).success);
  ASSERT_TRUE(eventually([&] {return port->poll_health(transaction).success;}));
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  handoff_floor(floor(0));
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto begin = work.get();
  ASSERT_TRUE(begin.success) << begin.code << ": " << begin.detail;
  succeed_when_caller_pause_released.store(true);
  const auto resumed = port->apply(effect(Kind::kResumeLocalizationCorrections, true));
  EXPECT_TRUE(floor_succeeded_after_caller_release.load());
  ASSERT_TRUE(resumed.success) << resumed.code << ": " << resumed.detail;
  const auto switched = port->apply(effect(Kind::kSwitchFloor, true));
  ASSERT_TRUE(switched.success) << switched.code << ": " << switched.detail;
  const auto verified = port->apply(effect(Kind::kVerifyFloorReady, true));
  EXPECT_TRUE(verified.success) << verified.code << ": " << verified.detail;
  EXPECT_EQ(floor_count(), 1U); EXPECT_EQ(floor_cancels.load(), 0);
}

TEST_F(RuntimeRecovery, FailureAfterCallerHandoffRetriesSwitchInsideSameEffect) {
  ASSERT_TRUE(port->apply(effect(Kind::kAcquireSafetyHold)).success);
  ASSERT_TRUE(port->apply(effect(Kind::kPauseLocalizationCorrections)).success);
  ASSERT_TRUE(eventually([&] {return port->poll_health(transaction).success;}));
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  const auto first = floor(0);
  handoff_floor(first);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(work.get().success);
  const auto released = port->apply(effect(Kind::kResumeLocalizationCorrections, true));
  ASSERT_TRUE(released.success) << released.code << ": " << released.detail;
  start(effect(Kind::kSwitchFloor, true));
  floor_resources(first->get_goal()->transaction_id, false);
  failed_floor_status(first->get_goal()->transaction_id, true);
  auto failure = std::make_shared<Floor::Result>();
  failure->failure_code = Floor::Result::EVIDENCE_STALE;
  failure->recovery_required = false;
  failure->message = "isolated failure after caller pause handoff";
  first->abort(failure);
  ASSERT_TRUE(eventually([&] {return floor_count() == 2U;}));
  const auto second = floor(1);
  EXPECT_NE(first->get_goal()->transaction_id, second->get_goal()->transaction_id);
  const auto caller_key = "robot_elevator_manager:" + transaction;
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_NE(std::find(pause.lease_keys.begin(), pause.lease_keys.end(), caller_key), pause.lease_keys.end());
  }
  handoff_floor(second);
  // The retry is still inside the original kSwitchFloor apply(). The test does
  // not issue a second begin/resume effect to help the implementation recover.
  ASSERT_TRUE(eventually([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return std::find(pause.lease_keys.begin(), pause.lease_keys.end(), caller_key) == pause.lease_keys.end();
  }));
  succeed_floor(second);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto recovered = work.get();
  EXPECT_TRUE(recovered.success) << recovered.code << ": " << recovered.detail;
  EXPECT_EQ(floor_count(), 2U); EXPECT_EQ(floor_cancels.load(), 0);
  EXPECT_EQ(nav_count(), 0U);
  EXPECT_TRUE(port->apply(effect(Kind::kVerifyFloorReady, true)).success);
}

TEST_F(RuntimeRecovery, RetryAllowsPauseStateToCatchUpWithSuccessfulServiceResponse) {
  ASSERT_TRUE(port->apply(effect(Kind::kAcquireSafetyHold)).success);
  {
    std::lock_guard<std::mutex> lock(mutex); pause_publication_enabled = false;
  }
  ASSERT_TRUE(port->apply(effect(Kind::kPauseLocalizationCorrections)).success);
  // Deliberately do NOT poll_health/wait for the state topic here. The real
  // service reply and state publication are independent DDS deliveries.
  start(effect(Kind::kBeginFloorTransition, true));
  ASSERT_TRUE(eventually([&] {return floor_count() == 1U;}));
  const auto first = floor(0);
  auto failure = std::make_shared<Floor::Result>();
  failure->failure_code = Floor::Result::EVIDENCE_STALE;
  failure->recovery_required = false;
  failure->message = "immediate failure with delayed pause-state delivery";
  first->abort(failure);
  std::this_thread::sleep_for(200ms); // controlled mock transport delay
  EXPECT_EQ(work.wait_for(0ms), std::future_status::timeout);
  {
    std::lock_guard<std::mutex> lock(mutex); pause_publication_enabled = true;
  }
  ASSERT_TRUE(eventually([&] {return floor_count() == 2U;}));
  const auto second = floor(1);
  EXPECT_NE(first->get_goal()->transaction_id, second->get_goal()->transaction_id);
  handoff_floor(second);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto begun = work.get();
  ASSERT_TRUE(begun.success) << begun.code << ": " << begun.detail;
  ASSERT_TRUE(port->apply(effect(Kind::kResumeLocalizationCorrections, true)).success);
  start(effect(Kind::kSwitchFloor, true));
  succeed_floor(second);
  ASSERT_EQ(work.wait_for(5s), std::future_status::ready);
  const auto result = work.get();
  EXPECT_TRUE(result.success) << result.code << ": " << result.detail;
  EXPECT_EQ(floor_count(), 2U); EXPECT_EQ(floor_cancels.load(), 0);
}

}  // namespace
}  // namespace robot_api_server
