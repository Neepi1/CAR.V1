#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_floor_manager/executor_runtime_policy.hpp"
#include "robot_floor_manager/floor_asset_snapshot_loader.hpp"
#include "robot_floor_manager/floor_switch_preflight.hpp"
#include "robot_floor_manager/floor_transition_evidence_tracker.hpp"
#include "robot_floor_manager/floor_transition_executor.hpp"
#include "robot_floor_manager/floor_transition_reconciliation.hpp"
#include "robot_floor_manager/navigate_action_graph.hpp"
#include "robot_floor_manager/runtime_map_context_writer.hpp"
#include "robot_interfaces/action/floor_switch.hpp"
#include "robot_interfaces/msg/correction_pause_state.hpp"
#include "robot_interfaces/msg/floor_switch_status.hpp"
#include "robot_interfaces/msg/localizer_asset_state.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_interfaces/msg/motion_interlock_state.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/begin_floor_transition.hpp"
#include "robot_interfaces/srv/set_correction_pause.hpp"
#include "robot_interfaces/srv/set_motion_hold.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "robot_safety/persistent_sequence_allocator.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

struct FloorAssets
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  fs::path root;
  fs::path nav_map_yaml;
  fs::path nav_map_pgm;
  fs::path localizer_map_png;
  fs::path localizer_params_yaml;
  fs::path keepout_mask_yaml;
  fs::path keepout_mask_pgm;
  fs::path speed_mask_yaml;
  fs::path speed_mask_pgm;
  fs::path binary_mask_yaml;
  fs::path binary_mask_pgm;
  fs::path asset_report_json;
  fs::path poses_yaml;
  std::vector<fs::path> filters;
};

std::string join_missing(const std::vector<std::string> & missing)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < missing.size(); ++i) {
    if (i != 0) {
      stream << "; ";
    }
    stream << missing[i];
  }
  return stream.str();
}

std::string lifecycle_state_service_for_load_map(const std::string & load_map_service)
{
  constexpr char suffix[] = "/load_map";
  const std::string suffix_string{suffix};
  if (
    load_map_service.size() > suffix_string.size() &&
    load_map_service.compare(
      load_map_service.size() - suffix_string.size(), suffix_string.size(), suffix_string) == 0)
  {
    return load_map_service.substr(0, load_map_service.size() - suffix_string.size()) + "/get_state";
  }
  return load_map_service + "/get_state";
}

bool cancel_matches_active_floor_switch(
  const bool action_active,
  const std::string & active_transaction_id,
  const std::string & cancel_transaction_id)
{
  return action_active &&
         !cancel_transaction_id.empty() &&
         active_transaction_id == cancel_transaction_id;
}

bool bridge_response_requires_target_epoch(const std::uint8_t operation)
{
  return
    operation ==
    robot_interfaces::srv::BeginFloorTransition::Request::OP_BEGIN ||
    operation ==
    robot_interfaces::srv::BeginFloorTransition::Request::OP_COMMIT;
}

bool bridge_begin_rejection_requires_recovery(
  const bool success,
  const bool runtime_context_valid)
{
  return !success && !runtime_context_valid;
}

bool contains_exact_key(
  const std::vector<std::string> & keys,
  const std::string & expected)
{
  return std::find(keys.cbegin(), keys.cend(), expected) != keys.cend();
}

bool same_runtime_identity(
  const robot_floor_manager::LocalizationHealthEvidence & left,
  const robot_floor_manager::LocalizationHealthEvidence & right)
{
  return
    left.building_id == right.building_id &&
    left.floor_id == right.floor_id &&
    left.map_id == right.map_id &&
    left.asset_epoch == right.asset_epoch &&
    left.asset_digest == right.asset_digest;
}

enum class RuntimeFilterRole
{
  kKeepout,
  kSpeed,
};

std::vector<RuntimeFilterRole> runtime_filter_reload_roles(
  const bool speed_filter_enabled)
{
  std::vector<RuntimeFilterRole> roles{RuntimeFilterRole::kKeepout};
  if (speed_filter_enabled) {
    roles.push_back(RuntimeFilterRole::kSpeed);
  }
  return roles;
}

}  // namespace

class FloorManagerNode
  : public rclcpp::Node,
    private robot_floor_manager::FloorTransitionRuntimePort
{
public:
  using FloorSwitchAction = robot_interfaces::action::FloorSwitch;
  using FloorSwitchGoalHandle = rclcpp_action::ServerGoalHandle<FloorSwitchAction>;

  explicit FloorManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_floor_manager", options)
  {
    maps_root_ = declare_parameter<std::string>("maps_root", "maps_release");
    default_building_id_ = declare_parameter<std::string>("default_building_id", "building_1");
    status_topic_ = declare_parameter<std::string>("status_topic", "/floor_manager/status");
    map_server_load_service_ = declare_parameter<std::string>("map_server_load_service", "/map_server/load_map");
    keepout_mask_load_service_ =
      declare_parameter<std::string>("keepout_mask_load_service", "/keepout_filter_mask_server/load_map");
    speed_mask_load_service_ =
      declare_parameter<std::string>("speed_mask_load_service", "/speed_filter_mask_server/load_map");
    localizer_apply_service_ =
      declare_parameter<std::string>("localizer_apply_service", "/global_localization/apply_floor_assets");
    localization_trigger_service_ =
      declare_parameter<std::string>("localization_trigger_service", "/global_localization/trigger");
    global_costmap_clear_service_ =
      declare_parameter<std::string>("global_costmap_clear_service", "/global_costmap/clear_entirely_global_costmap");
    local_costmap_clear_service_ =
      declare_parameter<std::string>("local_costmap_clear_service", "/local_costmap/clear_entirely_local_costmap");
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 10.0);
    localizer_apply_timeout_sec_ = std::max(
      service_timeout_sec_,
      declare_parameter<double>("localizer_apply_timeout_sec", 30.0));
    localization_trigger_timeout_sec_ =
      declare_parameter<double>("localization_trigger_timeout_sec", 75.0);
    call_map_server_load_ = declare_parameter<bool>("call_map_server_load", true);
    call_filter_mask_load_ = declare_parameter<bool>("call_filter_mask_load", true);
    speed_filter_enabled_ =
      declare_parameter<bool>("speed_filter_enabled", false);
    call_localizer_apply_ = declare_parameter<bool>("call_localizer_apply", true);
    call_localization_trigger_ = declare_parameter<bool>("call_localization_trigger", true);
    clear_costmaps_after_switch_ = declare_parameter<bool>("clear_costmaps_after_switch", true);
    require_filter_assets_ = declare_parameter<bool>("require_filter_assets", true);
    filter_mask_state_timeout_sec_ = declare_parameter<double>("filter_mask_state_timeout_sec", 0.5);
    floor_switch_action_name_ =
      declare_parameter<std::string>("floor_switch_action_name", "/floor_manager/floor_switch");
    transition_status_topic_ =
      declare_parameter<std::string>(
      "transition_status_topic", "/floor_manager/transition_status");
    live_floor_switch_enabled_ =
      declare_parameter<bool>("live_floor_switch_enabled", false);
    motion_hold_service_ =
      declare_parameter<std::string>(
      "motion_hold_service", "/safety/set_motion_hold");
    motion_hold_sequence_state_file_ =
      declare_parameter<std::string>(
      "motion_hold_sequence_state_file",
      "/tmp/njrh_floor_manager_hold_sequence.state");
    try {
      motion_hold_sequence_allocator_ =
        std::make_unique<robot_safety::PersistentSequenceAllocator>(
        fs::path{motion_hold_sequence_state_file_});
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        get_logger(),
        "cannot reserve persistent floor-manager hold sequences from '%s': %s",
        motion_hold_sequence_state_file_.c_str(), exception.what());
      throw;
    }
    motion_interlock_state_topic_ =
      declare_parameter<std::string>(
      "motion_interlock_state_topic", "/safety/motion_interlock_state");
    navigate_to_pose_status_topic_ =
      declare_parameter<std::string>(
      "navigate_to_pose_status_topic", "/navigate_to_pose/_action/status");
    navigate_to_pose_action_name_ =
      declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    wheel_odom_topic_ =
      declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    local_odom_topic_ =
      declare_parameter<std::string>(
      "local_odom_topic", "/local_state/odometry");
    correction_pause_service_ =
      declare_parameter<std::string>(
      "correction_pause_service",
      "/robot_localization_bridge/set_correction_pause_lease");
    correction_pause_state_topic_ =
      declare_parameter<std::string>(
      "correction_pause_state_topic",
      "/localization/correction_pause_state");
    begin_floor_transition_service_ =
      declare_parameter<std::string>(
      "begin_floor_transition_service",
      "/robot_localization_bridge/begin_floor_transition");
    localization_health_topic_ =
      declare_parameter<std::string>(
      "localization_health_topic", "/localization/floor_health");
    localizer_asset_state_topic_ =
      declare_parameter<std::string>(
      "localizer_asset_state_topic", "/global_localization/asset_state");
    global_costmap_topic_ =
      declare_parameter<std::string>(
      "global_costmap_topic", "/global_costmap/costmap");
    local_costmap_topic_ =
      declare_parameter<std::string>(
      "local_costmap_topic", "/local_costmap/costmap");
    runtime_map_context_file_ =
      declare_parameter<std::string>(
      "runtime_map_context_file", "/tmp/njrh_runtime_map_context.json");
    startup_handoff_file_ = declare_parameter<std::string>(
      "startup_handoff_file", "/tmp/njrh_floor_startup_handoff.json");
    startup_handoff_ack_file_ = declare_parameter<std::string>(
      "startup_handoff_ack_file", "/tmp/njrh_floor_startup_handoff_ack.json");
    startup_handoff_timeout_sec_ = declare_parameter<double>("startup_handoff_timeout_sec", 90.0);
    startup_ready_timeout_sec_ = declare_parameter<double>("startup_ready_timeout_sec", 120.0);
    evidence_timeout_sec_ =
      declare_parameter<double>("evidence_timeout_sec", 10.0);
    nav_idle_bootstrap_grace_sec_ = std::clamp(
      declare_parameter<double>("nav_idle_bootstrap_grace_sec", 2.0),
      0.1, std::max(0.1, evidence_timeout_sec_));
    evidence_max_age_sec_ =
      declare_parameter<double>("evidence_max_age_sec", 0.75);
    stopped_stable_duration_sec_ =
      declare_parameter<double>("stopped_stable_duration_sec", 0.30);
    stopped_linear_threshold_mps_ =
      declare_parameter<double>("stopped_linear_threshold_mps", 0.02);
    stopped_angular_threshold_radps_ =
      declare_parameter<double>("stopped_angular_threshold_radps", 0.02);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    nav_cancel_client_ = create_client<action_msgs::srv::CancelGoal>(
      navigate_to_pose_action_name_ + "/_action/cancel_goal", rmw_qos_profile_services_default,
      callback_group_);
    nav_lifecycle_probes_[0].node_name = "/bt_navigator";
    nav_lifecycle_probes_[1].node_name = "/controller_server";
    for (auto & probe : nav_lifecycle_probes_) {
      // Default mutually-exclusive callback group serializes probe bookkeeping.
      probe.client = create_client<lifecycle_msgs::srv::GetState>(probe.node_name + "/get_state");
    }

    keepout_mask_state_service_ = lifecycle_state_service_for_load_map(keepout_mask_load_service_);
    speed_mask_state_service_ = lifecycle_state_service_for_load_map(speed_mask_load_service_);

    map_load_client_ = create_client<nav2_msgs::srv::LoadMap>(map_server_load_service_, rmw_qos_profile_services_default, callback_group_);
    keepout_mask_load_client_ =
      create_client<nav2_msgs::srv::LoadMap>(keepout_mask_load_service_, rmw_qos_profile_services_default, callback_group_);
    speed_mask_load_client_ =
      create_client<nav2_msgs::srv::LoadMap>(speed_mask_load_service_, rmw_qos_profile_services_default, callback_group_);
    keepout_mask_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
      keepout_mask_state_service_, rmw_qos_profile_services_default, callback_group_);
    speed_mask_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
      speed_mask_state_service_, rmw_qos_profile_services_default, callback_group_);
    localizer_apply_client_ = create_client<robot_interfaces::srv::ApplyFloorAssets>(
      localizer_apply_service_, rmw_qos_profile_services_default, callback_group_);
    localization_trigger_client_ = create_client<robot_interfaces::srv::TriggerLocalization>(
      localization_trigger_service_, rmw_qos_profile_services_default, callback_group_);
    global_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      global_costmap_clear_service_, rmw_qos_profile_services_default, callback_group_);
    local_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      local_costmap_clear_service_, rmw_qos_profile_services_default, callback_group_);
    motion_hold_client_ = create_client<robot_interfaces::srv::SetMotionHold>(
      motion_hold_service_, rmw_qos_profile_services_default, callback_group_);
    correction_pause_client_ =
      create_client<robot_interfaces::srv::SetCorrectionPause>(
      correction_pause_service_, rmw_qos_profile_services_default, callback_group_);
    begin_floor_transition_client_ =
      create_client<robot_interfaces::srv::BeginFloorTransition>(
      begin_floor_transition_service_, rmw_qos_profile_services_default, callback_group_);
    const auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    motion_interlock_state_sub_ =
      create_subscription<robot_interfaces::msg::MotionInterlockState>(
      motion_interlock_state_topic_, state_qos,
      [this](const robot_interfaces::msg::MotionInterlockState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_motion_interlock(
          message->hold_active, message->hold_keys, steady_now_sec());
        last_motion_interlock_hold_active_ = message->hold_active;
        last_motion_interlock_hold_keys_ = message->hold_keys;
        ++motion_interlock_state_generation_;
        evidence_changed_.notify_all();
      });
    correction_pause_state_sub_ =
      create_subscription<robot_interfaces::msg::CorrectionPauseState>(
      correction_pause_state_topic_, state_qos,
      [this](const robot_interfaces::msg::CorrectionPauseState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_correction_pause(
          message->paused, message->lease_keys, steady_now_sec());
        last_correction_pause_active_ = message->paused;
        last_correction_pause_keys_ = message->lease_keys;
        ++correction_pause_state_generation_;
        evidence_changed_.notify_all();
      });
    localization_health_sub_ =
      create_subscription<robot_interfaces::msg::LocalizationHealth>(
      localization_health_topic_, state_qos,
      [this](const robot_interfaces::msg::LocalizationHealth::SharedPtr message) {
        robot_floor_manager::LocalizationHealthEvidence health;
        health.building_id = message->building_id;
        health.floor_id = message->floor_id;
        health.map_id = message->map_id;
        health.asset_epoch = message->asset_epoch;
        health.asset_digest = message->asset_digest;
        health.localizer_generation = message->localizer_generation;
        health.explicit_relocalization_sequence =
          message->explicit_relocalization_sequence;
        health.localizer_ready = message->localizer_ready;
        health.bridge_ready = message->bridge_ready;
        health.tf_unique = message->tf_unique;
        health.transition_active = message->transition_active;
        health.runtime_context_valid = message->runtime_context_valid;
        health.amcl_ready = message->amcl_ready;
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_localization_health(
          health, steady_now_sec());
        last_localization_health_ = health;
        last_localization_health_received_steady_sec_ = steady_now_sec();
        ++localization_health_generation_;
        evidence_changed_.notify_all();
      });
    localizer_asset_state_sub_ =
      create_subscription<robot_interfaces::msg::LocalizerAssetState>(
      localizer_asset_state_topic_, state_qos,
      [this](const robot_interfaces::msg::LocalizerAssetState::SharedPtr message) {
        robot_floor_manager::LocalizerAssetEvidence state;
        state.transaction_id = message->transaction_id;
        state.applying = message->applying;
        state.success = message->success;
        state.reloaded = message->reloaded;
        state.active_identity_valid = message->active_identity_valid;
        state.active_building_id = message->active_building_id;
        state.active_floor_id = message->active_floor_id;
        state.active_map_id = message->active_map_id;
        state.active_asset_epoch = message->active_asset_epoch;
        state.active_asset_digest = message->active_asset_digest;
        state.localizer_generation = message->localizer_generation;
        state.localizer_ready = message->localizer_ready;
        state.requested_building_id = message->requested_building_id;
        state.requested_floor_id = message->requested_floor_id;
        state.requested_map_id = message->requested_map_id;
        state.requested_asset_epoch = message->requested_asset_epoch;
        state.requested_asset_digest = message->requested_asset_digest;
        state.code = message->code;
        state.detail = message->message;
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_localizer_asset(
          state, steady_now_sec());
        evidence_changed_.notify_all();
      });
    nav_status_sub_ =
      create_subscription<action_msgs::msg::GoalStatusArray>(
      navigate_to_pose_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr message) {
        bool active = false;
        for (const auto & status : message->status_list) {
          if (
            status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
          {
            active = true;
            break;
          }
        }
        const auto observed_at = steady_now_sec();
        nav_goal_active_.store(active);
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_nav_activity(active, observed_at);
        evidence_changed_.notify_all();
      });
    nav_graph_probe_timer_ = create_wall_timer(
      200ms,
      [this]() {
        if (nav_lifecycle_probe_enabled_.load()) {
          probe_nav_lifecycle();
        }
        std::set<std::string> service_names;
        for (const auto & entry : get_service_names_and_types()) {
          service_names.insert(entry.first);
        }
        const bool graph_ready = robot_floor_manager::navigate_action_graph_ready(
          navigate_to_pose_action_name_, service_names,
          count_publishers(navigate_to_pose_status_topic_));
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_nav_graph_ready(
          graph_ready, steady_now_sec());
        // The grace boundary is time-based, so wake an in-flight precondition
        // wait even when the graph remains continuously ready.
        evidence_changed_.notify_all();
      });
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_wheel_odom(
          message->twist.twist.linear.x,
          message->twist.twist.linear.y,
          message->twist.twist.angular.z,
          steady_now_sec());
        evidence_changed_.notify_all();
      });
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_local_odom(
          message->twist.twist.linear.x,
          message->twist.twist.linear.y,
          message->twist.twist.angular.z,
          steady_now_sec());
        evidence_changed_.notify_all();
      });
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_global_costmap(
          ++global_costmap_receive_sequence_, steady_now_sec());
        evidence_changed_.notify_all();
      });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_local_costmap(
          ++local_costmap_receive_sequence_, steady_now_sec());
        evidence_changed_.notify_all();
      });

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10).transient_local());
    transition_status_pub_ =
      create_publisher<robot_interfaces::msg::FloorSwitchStatus>(
      transition_status_topic_, rclcpp::QoS(1).reliable().transient_local());

    switch_service_ = create_service<robot_interfaces::srv::SwitchFloor>(
      "/floor_manager/switch_floor",
      std::bind(&FloorManagerNode::on_switch_floor, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    floor_switch_action_server_ = rclcpp_action::create_server<FloorSwitchAction>(
      this,
      floor_switch_action_name_,
      std::bind(
        &FloorManagerNode::on_floor_switch_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &FloorManagerNode::on_floor_switch_cancel, this,
        std::placeholders::_1),
      std::bind(
        &FloorManagerNode::on_floor_switch_accepted, this,
        std::placeholders::_1));

    publish_status("idle");
    publish_transition_status(
      "", "IDLE", "IDLE", 0U, "strict live floor switching is disabled by default");
  }

  ~FloorManagerNode() override
  {
    shutting_down_.store(true);
    evidence_changed_.notify_all();
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (floor_switch_worker_.joinable()) {
      floor_switch_worker_.join();
    }
  }

private:
  static double steady_now_sec()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void publish_transition_status(
    const std::string & transaction_id,
    const std::string & state,
    const std::string & stage,
    const std::uint16_t failure_code,
    const std::string & detail,
    const robot_floor_manager::FloorTransitionRequest * request = nullptr)
  {
    robot_interfaces::msg::FloorSwitchStatus status;
    status.stamp = now();
    status.transaction_id = transaction_id;
    status.state = state;
    status.stage = stage;
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      status.generation = ++transition_status_generation_;
      status.selected_building_id = selected_building_id_;
      status.selected_floor_id = selected_floor_id_;
      status.selected_map_id = selected_map_id_;
    }
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      const bool terminal_runtime_ready =
        runtime_context_confirmed_ && !motion_hold_acquired_.load();
      status.active_context_valid = terminal_runtime_ready;
      status.asset_epoch = published_asset_epoch_;
      status.asset_digest = published_asset_digest_;
      status.nav_map_ready = nav_map_ready_;
      status.filters_ready = filters_ready_;
      status.localizer_ready = localizer_ready_;
      status.bridge_ready = bridge_ready_;
      status.amcl_ready = amcl_ready_;
      status.costmaps_ready = costmaps_ready_;
      status.nav2_ready =
        terminal_runtime_ready &&
        nav_map_ready_ &&
        filters_ready_ &&
        localizer_ready_ &&
        bridge_ready_ &&
        amcl_ready_ &&
        costmaps_ready_;
      if (terminal_runtime_ready) {
        status.active_building_id = status.selected_building_id;
        status.active_floor_id = status.selected_floor_id;
        status.active_map_id = status.selected_map_id;
      }
    }
    status.failure_code = failure_code;
    status.detail = detail;
    if (request != nullptr) {
      status.requested_building_id = request->building_id;
      status.requested_floor_id = request->floor_id;
      status.requested_map_id = request->map_id;
      status.requested_asset_epoch = request->expected_asset_epoch;
      status.requested_asset_digest = request->expected_asset_digest;
      status.pending_building_id = request->building_id;
      status.pending_floor_id = request->floor_id;
      status.pending_map_id = request->map_id;
    }
    transition_status_pub_->publish(status);
  }

  rclcpp_action::GoalResponse on_floor_switch_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FloorSwitchAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_floor_switch_cancel(
    const std::shared_ptr<FloorSwitchGoalHandle> goal_handle)
  {
    const auto transaction_id = goal_handle->get_goal()->transaction_id;
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      if (!cancel_matches_active_floor_switch(
          floor_switch_action_active_,
          active_floor_switch_transaction_,
          transaction_id))
      {
        return rclcpp_action::CancelResponse::REJECT;
      }
    }
    cancel_requested_.store(true);
    // Stop future startup steps immediately; already dispatched RPCs still
    // settle through their original clients and the existing safety cleanup.
    finish_startup_handoff("failed", "floor-switch cancellation accepted", transaction_id);
    evidence_changed_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_floor_switch_accepted(
    const std::shared_ptr<FloorSwitchGoalHandle> goal_handle)
  {
    if (live_floor_switch_enabled_) {
      start_live_floor_switch(goal_handle);
      return;
    }
    try {
      execute_floor_switch_preflight(goal_handle);
    } catch (const std::exception & exc) {
      const auto transaction_id = goal_handle->get_goal()->transaction_id;
      release_floor_switch_action(transaction_id);
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kInternalError);
      result->message = "INTERNAL_ERROR: floor-switch preflight exception: " +
        std::string(exc.what());
      result->runtime_context_valid = false;
      result->recovery_required = false;
      try {
        goal_handle->abort(result);
      } catch (const std::exception & terminal_exc) {
        RCLCPP_ERROR(
          get_logger(), "failed to abort floor-switch action after exception: %s",
          terminal_exc.what());
      }
    } catch (...) {
      const auto transaction_id = goal_handle->get_goal()->transaction_id;
      release_floor_switch_action(transaction_id);
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kInternalError);
      result->message = "INTERNAL_ERROR: unknown floor-switch preflight exception";
      result->runtime_context_valid = false;
      result->recovery_required = false;
      try {
        goal_handle->abort(result);
      } catch (const std::exception & terminal_exc) {
        RCLCPP_ERROR(
          get_logger(), "failed to abort floor-switch action after unknown exception: %s",
          terminal_exc.what());
      }
    }
  }

  void start_live_floor_switch(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    bool transaction_conflict = false;
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      transaction_conflict = floor_switch_action_active_ || switching_ ||
        deferred_failure_cleanup_.load();
      if (!transaction_conflict) {
        // Reset the probe while holding the same mutex used by cancellation.
        // A cancel arriving after this point cannot be erased by startup.
        cancel_requested_.store(false);
        floor_switch_action_active_ = true;
        active_floor_switch_transaction_ = goal->transaction_id;
      }
    }
    if (transaction_conflict) {
      const auto code =
        robot_floor_manager::FloorSwitchFailureCode::kTransactionConflict;
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(code);
      result->message =
        std::string(robot_floor_manager::to_string(code)) +
        ": another floor transaction, unresolved request, or owned-resource cleanup is active";
      result->runtime_context_valid = false;
      publish_transition_status(
        goal->transaction_id, "BLOCKED",
        robot_floor_manager::to_string(code),
        result->failure_code, result->message);
      goal_handle->abort(result);
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (floor_switch_worker_.joinable()) {
        floor_switch_worker_.join();
      }
      floor_switch_worker_ = std::thread(
        [this, goal_handle]() {
          execute_live_floor_switch(goal_handle);
          reconcile_deferred_failure();
        });
    } catch (const std::exception & exception) {
      release_floor_switch_action(goal->transaction_id);
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kInternalError);
      result->message =
        "INTERNAL_ERROR: failed to start floor-switch worker: " +
        std::string(exception.what());
      result->runtime_context_valid = true;
      result->recovery_required = false;
      goal_handle->abort(result);
    }
  }

  static robot_floor_manager::FloorTransitionRequest action_request(
    const FloorSwitchAction::Goal & goal)
  {
    return {
      goal.transaction_id,
      goal.building_id,
      goal.floor_id,
      goal.map_id,
      goal.expected_asset_epoch,
      goal.expected_asset_digest,
    };
  }

  static FloorAssets assets_from_snapshot(
    const robot_floor_manager::FloorAssetSnapshot & snapshot)
  {
    FloorAssets assets;
    assets.building_id = snapshot.building_id;
    assets.floor_id = snapshot.floor_id;
    assets.map_id = snapshot.map_id;
    assets.asset_epoch = snapshot.asset_epoch;
    assets.asset_digest = snapshot.asset_digest;
    assets.root = snapshot.paths.root;
    assets.nav_map_yaml = snapshot.paths.nav_map_yaml;
    assets.nav_map_pgm = snapshot.paths.nav_map_pgm;
    assets.localizer_map_png = snapshot.paths.localizer_map_png;
    assets.localizer_params_yaml = snapshot.paths.localizer_params_yaml;
    assets.keepout_mask_yaml = snapshot.paths.keepout_mask_yaml;
    assets.keepout_mask_pgm = snapshot.paths.keepout_mask_pgm;
    assets.speed_mask_yaml = snapshot.paths.speed_mask_yaml;
    assets.speed_mask_pgm = snapshot.paths.speed_mask_pgm;
    assets.binary_mask_yaml = snapshot.paths.binary_mask_yaml;
    assets.binary_mask_pgm = snapshot.paths.binary_mask_pgm;
    assets.asset_report_json = snapshot.paths.asset_report_json;
    assets.poses_yaml = snapshot.paths.poses_yaml;
    assets.filters = {
      assets.keepout_mask_yaml,
      assets.keepout_mask_pgm,
      assets.speed_mask_yaml,
      assets.speed_mask_pgm,
      assets.binary_mask_yaml,
      assets.binary_mask_pgm,
    };
    return assets;
  }

  void execute_live_floor_switch(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle)
  {
    const auto request = action_request(*goal_handle->get_goal());
    try {
      publish_transition_status(
        request.transaction_id, "PREFLIGHT", "VERIFY_EXACT_ASSET",
        0U, "verifying immutable target asset snapshot", &request);
      auto feedback = std::make_shared<FloorSwitchAction::Feedback>();
      feedback->stage = "VERIFY_EXACT_ASSET";
      feedback->progress = 0.01F;
      feedback->detail = "verifying immutable target asset snapshot";
      feedback->asset_epoch = request.expected_asset_epoch;
      feedback->transaction_id = request.transaction_id;
      feedback->stage_sequence = 0U;
      feedback->caller_pause_handoff_ready = false;
      goal_handle->publish_feedback(feedback);

      const robot_floor_manager::FloorAssetSnapshotRequest snapshot_request{
        fs::path(maps_root_),
        request.building_id,
        request.floor_id,
        request.map_id,
        request.expected_asset_epoch,
        request.expected_asset_digest,
      };
      const auto snapshot_result =
        robot_floor_manager::FloorAssetSnapshotLoader{}.load(snapshot_request);
      if (!snapshot_result.ok()) {
        finish_asset_preflight_failure(
          goal_handle, request, snapshot_result);
        return;
      }
      if (goal_handle->is_canceling() || shutting_down_.load()) {
        finish_cancelled_before_mutation(goal_handle, request);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.begin(
          {
            request.transaction_id,
            request.building_id,
            request.floor_id,
            request.map_id,
            request.expected_asset_epoch,
            request.expected_asset_digest,
          },
          steady_now_sec());
        active_snapshot_ = *snapshot_result.snapshot;
        source_runtime_context_.reset();
        const auto source_observed_at = steady_now_sec();
        if (
          last_localization_health_.has_value() &&
          last_localization_health_->runtime_context_valid &&
          !last_localization_health_->transition_active &&
          last_localization_health_->localizer_ready &&
          last_localization_health_->bridge_ready &&
          last_localization_health_->tf_unique &&
          last_localization_health_received_steady_sec_ >= 0.0 &&
          source_observed_at >= last_localization_health_received_steady_sec_ &&
          source_observed_at - last_localization_health_received_steady_sec_ <=
          evidence_max_age_sec_)
        {
          source_runtime_context_ = last_localization_health_;
        }
        bridge_begin_established_ = false;
        bridge_begin_submitted_ = false;
        bridge_begin_outcome_unknown_ = false;
        target_effect_dispatched_ = false;
        target_requests_.clear();
        startup_target_trigger_dispatched_ = false;
        tracked_floor_transaction_ = request.transaction_id;
        {
          std::lock_guard<std::mutex> handoff_lock(startup_handoff_mutex_);
          startup_handoff_.reset();
        }
        motion_hold_command_submitted_.store(false);
        motion_hold_outcome_unknown_.store(false);
        correction_pause_command_submitted_.store(false);
        correction_pause_outcome_unknown_.store(false);
        motion_hold_acquired_.store(false);
        floor_pause_acquired_ = false;
        localizer_generation_ = 0U;
        explicit_relocalization_sequence_ = 0U;
      }

      robot_floor_manager::FloorTransitionExecutor executor(*this);
      nav_lifecycle_probe_enabled_.store(true);
      const auto execution = executor.run(
        request,
        *snapshot_result.snapshot,
        [this, goal_handle]() {
          return shutting_down_.load() || goal_handle->is_canceling();
        },
        [this, &request, &goal_handle](
          const robot_floor_manager::FloorTransitionOutput & output)
        {
          publish_live_progress(goal_handle, request, output);
        });
      nav_lifecycle_probe_enabled_.store(false);
      finish_startup_handoff(execution.success ? "committed" : "failed", execution.message,
        request.transaction_id);
      finish_live_floor_switch(goal_handle, request, execution);
    } catch (const std::exception & exception) {
      nav_lifecycle_probe_enabled_.store(false);
      finish_startup_handoff("failed", exception.what(), request.transaction_id);
      const bool cleanup_released = emergency_lock_after_exception(request, exception.what());
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kInternalError);
      result->message =
        "INTERNAL_ERROR: live floor-switch exception: " +
        std::string(exception.what());
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        result->runtime_context_valid = runtime_context_confirmed_;
      }
      result->recovery_required = !cleanup_released;
      publish_transition_status(
        request.transaction_id, "FAILED", "INTERNAL_ERROR",
        result->failure_code, result->message, &request);
      release_floor_switch_action(request.transaction_id);
      goal_handle->abort(result);
    } catch (...) {
      nav_lifecycle_probe_enabled_.store(false);
      finish_startup_handoff("failed", "unknown exception", request.transaction_id);
      const bool cleanup_released = emergency_lock_after_exception(request, "unknown exception");
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->success = false;
      result->failure_code = static_cast<std::uint16_t>(
        robot_floor_manager::FloorSwitchFailureCode::kInternalError);
      result->message = "INTERNAL_ERROR: unknown live floor-switch exception";
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        result->runtime_context_valid = runtime_context_confirmed_;
      }
      result->recovery_required = !cleanup_released;
      publish_transition_status(
        request.transaction_id, "FAILED", "INTERNAL_ERROR",
        result->failure_code, result->message, &request);
      release_floor_switch_action(request.transaction_id);
      goal_handle->abort(result);
    }
  }

  static robot_floor_manager::FloorSwitchFailureCode snapshot_failure_code(
    const robot_floor_manager::FloorAssetSnapshotError error)
  {
    using Error = robot_floor_manager::FloorAssetSnapshotError;
    using Code = robot_floor_manager::FloorSwitchFailureCode;
    switch (error) {
      case Error::kInvalidRequest: return Code::kInvalidGoal;
      case Error::kRegistryUnavailable:
      case Error::kIdentityNotFound:
        return Code::kAssetNotFound;
      case Error::kIdentityMismatch:
      case Error::kAssetChanged:
        return Code::kAssetIdentityMismatch;
      case Error::kDigestMismatch:
        return Code::kAssetDigestMismatch;
      case Error::kInvalidManifest:
      case Error::kInvalidLayout:
      case Error::kUnsafePath:
      case Error::kUnsafeAsset:
      case Error::kIoError:
        return Code::kAssetBundleInvalid;
      case Error::kNone:
        break;
    }
    return Code::kInternalError;
  }

  void finish_asset_preflight_failure(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle,
    const robot_floor_manager::FloorTransitionRequest & request,
    const robot_floor_manager::FloorAssetSnapshotResult & snapshot_result)
  {
    const auto code = snapshot_failure_code(snapshot_result.error);
    auto result = std::make_shared<FloorSwitchAction::Result>();
    result->success = false;
    result->failure_code = static_cast<std::uint16_t>(code);
    result->message =
      std::string(robot_floor_manager::to_string(code)) + ": " +
      snapshot_result.message;
    result->runtime_context_valid = true;
    result->recovery_required = false;
    publish_transition_status(
      request.transaction_id, "BLOCKED",
      robot_floor_manager::to_string(code),
      result->failure_code, result->message, &request);
    release_floor_switch_action(request.transaction_id);
    goal_handle->abort(result);
  }

  void finish_cancelled_before_mutation(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle,
    const robot_floor_manager::FloorTransitionRequest & request)
  {
    const auto code =
      robot_floor_manager::FloorSwitchFailureCode::kCancelledBeforeMutation;
    auto result = std::make_shared<FloorSwitchAction::Result>();
    result->success = false;
    result->failure_code = static_cast<std::uint16_t>(code);
    result->message =
      std::string(robot_floor_manager::to_string(code)) +
      ": no runtime asset was changed";
    result->runtime_context_valid = true;
    result->recovery_required = false;
    publish_transition_status(
      request.transaction_id, "CANCELED",
      robot_floor_manager::to_string(code),
      result->failure_code, result->message, &request);
    release_floor_switch_action(request.transaction_id);
    goal_handle->canceled(result);
  }

  static robot_floor_manager::FloorSwitchFailureCode execution_failure_code(
    const std::string & failure)
  {
    using Code = robot_floor_manager::FloorSwitchFailureCode;
    if (failure == "CANCELLED") {
      return Code::kCancelledBeforeMutation;
    }
    if (failure == "NAV_MAP_LOAD_SERVICE_UNAVAILABLE" ||
      failure == "FILTER_LOAD_SERVICE_UNAVAILABLE")
    {
      return Code::kRuntimeContextUnproven;
    }
    if (failure == "NAV_MAP_LOAD_INVALID_ASSET" ||
      failure == "FILTER_LOAD_INVALID_ASSET")
    {
      return Code::kAssetBundleInvalid;
    }
    if (failure.find("MOTION_HOLD") != std::string::npos) {
      return Code::kMotionHoldUnproven;
    }
    if (failure.find("NAV_IDLE") != std::string::npos) {
      return Code::kNavIdleUnproven;
    }
    if (failure.find("STOPPED") != std::string::npos) {
      return Code::kStoppedUnproven;
    }
    if (failure.find("STALE") != std::string::npos ||
      failure.find("TIMEOUT") != std::string::npos)
    {
      return Code::kEvidenceStale;
    }
    if (failure.find("LOCALIZER") != std::string::npos) {
      return Code::kLocalizerReloadUnproven;
    }
    if (failure.find("ASSET_IDENTITY") != std::string::npos) {
      return Code::kAssetIdentityMismatch;
    }
    if (
      failure.find("BRIDGE") != std::string::npos ||
      failure.find("CONTEXT") != std::string::npos ||
      failure.find("STARTUP_") != std::string::npos ||
      failure.find("COSTMAP") != std::string::npos ||
      failure.find("PAUSE") != std::string::npos)
    {
      return Code::kRuntimeContextUnproven;
    }
    return Code::kInternalError;
  }

  void publish_live_progress(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle,
    const robot_floor_manager::FloorTransitionRequest & request,
    const robot_floor_manager::FloorTransitionOutput & output)
  {
    const auto stage =
      robot_floor_manager::floor_transition_feedback_stage(output);
    auto feedback = std::make_shared<FloorSwitchAction::Feedback>();
    feedback->stage = stage;
    feedback->progress =
      robot_floor_manager::floor_transition_feedback_progress(output.state);
    feedback->detail = output.message;
    feedback->asset_epoch = request.expected_asset_epoch;
    feedback->transaction_id = request.transaction_id;
    feedback->stage_sequence = output.effect.sequence;
    feedback->caller_pause_handoff_ready =
      robot_floor_manager::floor_transition_caller_pause_handoff_ready(output);
    goal_handle->publish_feedback(feedback);
    publish_transition_status(
      request.transaction_id,
      output.state == robot_floor_manager::FloorTransitionState::kFailureCleanup ?
      "FAILURE_CLEANUP" : "RUNNING",
      stage, 0U, output.message, &request);
  }

  void finish_live_floor_switch(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle,
    const robot_floor_manager::FloorTransitionRequest & request,
    const robot_floor_manager::FloorTransitionExecutionResult & execution)
  {
    auto result = std::make_shared<FloorSwitchAction::Result>();
    result->success = execution.success;
    result->message = execution.message;
    result->active_building_id = execution.active_building_id;
    result->active_floor_id = execution.active_floor_id;
    result->active_map_id = execution.active_map_id;
    result->asset_epoch = execution.asset_epoch;
    result->asset_digest = execution.asset_digest;
    result->explicit_relocalization_sequence =
      execution.explicit_relocalization_sequence;
    result->runtime_context_valid = execution.runtime_context_valid;
    result->recovery_required = execution.recovery_required;

    if (execution.success) {
      result->failure_code = 0U;
      {
        std::lock_guard<std::mutex> lock(floor_state_mutex_);
        selected_building_id_ = execution.active_building_id;
        selected_floor_id_ = execution.active_floor_id;
        selected_map_id_ = execution.active_map_id;
      }
      publish_transition_status(
        request.transaction_id, "COMPLETE", "COMPLETE",
        0U, execution.message, &request);
      release_floor_switch_action(request.transaction_id);
      goal_handle->succeed(result);
      return;
    }

    const auto code =
      execution.failure_code == "CANCELLED" &&
      execution.recovery_required ?
      robot_floor_manager::FloorSwitchFailureCode::kRuntimeContextUnproven :
      execution_failure_code(execution.failure_code);
    result->failure_code = static_cast<std::uint16_t>(code);
    if (result->message.empty()) {
      result->message =
        std::string(robot_floor_manager::to_string(code)) + ": " +
        execution.failure_code;
    }
    const auto disposition =
      robot_floor_manager::floor_transition_failure_disposition(execution);
    deferred_failure_code_ = result->failure_code;
    if (startup_owner_exited()) {
      result->message = "STARTUP_OWNER_EXITED; " + result->message;
    }
    publish_transition_status(
      request.transaction_id,
      disposition.state,
      disposition.stage,
      result->failure_code, result->message, &request);
    release_floor_switch_action(request.transaction_id);
    if (disposition.canceled) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void release_floor_switch_action(const std::string & transaction_id)
  {
    std::lock_guard<std::mutex> lock(floor_state_mutex_);
    if (floor_switch_action_active_ &&
      active_floor_switch_transaction_ == transaction_id)
    {
      floor_switch_action_active_ = false;
      active_floor_switch_transaction_.clear();
    }
  }

  void execute_floor_switch_preflight(
    const std::shared_ptr<FloorSwitchGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    robot_floor_manager::FloorSwitchPreflightInput input;
    input.request.transaction_id = goal->transaction_id;
    input.request.building_id = goal->building_id;
    input.request.floor_id = goal->floor_id;
    input.request.map_id = goal->map_id;
    input.request.expected_asset_epoch = goal->expected_asset_epoch;
    input.request.expected_asset_digest = goal->expected_asset_digest;
    input.live_floor_switch_enabled = live_floor_switch_enabled_;

    bool transaction_conflict = false;
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      transaction_conflict = floor_switch_action_active_ || switching_ ||
        deferred_failure_cleanup_.load();
      if (!transaction_conflict) {
        floor_switch_action_active_ = true;
        active_floor_switch_transaction_ = input.request.transaction_id;
      }
    }
    if (transaction_conflict) {
      const auto code = robot_floor_manager::FloorSwitchFailureCode::kTransactionConflict;
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->failure_code = static_cast<std::uint16_t>(code);
      result->message =
        std::string(robot_floor_manager::to_string(code)) +
        ": another floor transaction or legacy selection is active";
      result->runtime_context_valid = false;
      publish_transition_status(
        input.request.transaction_id, "BLOCKED", robot_floor_manager::to_string(code),
        result->failure_code, result->message, &input.request);
      goal_handle->abort(result);
      return;
    }

    auto feedback = std::make_shared<FloorSwitchAction::Feedback>();
    feedback->stage = "PREFLIGHT";
    feedback->progress = 0.0F;
    feedback->detail = "validating a non-mutating strict floor-switch request";
    feedback->asset_epoch = 0U;
    feedback->transaction_id = input.request.transaction_id;
    feedback->stage_sequence = 0U;
    feedback->caller_pause_handoff_ready = false;
    goal_handle->publish_feedback(feedback);
    publish_transition_status(
      input.request.transaction_id, "PREFLIGHT", "VALIDATE_GOAL", 0U,
      feedback->detail, &input.request);

    if (goal_handle->is_canceling()) {
      const auto code = robot_floor_manager::FloorSwitchFailureCode::kCancelledBeforeMutation;
      auto result = std::make_shared<FloorSwitchAction::Result>();
      result->failure_code = static_cast<std::uint16_t>(code);
      result->message =
        std::string(robot_floor_manager::to_string(code)) + ": no runtime asset was changed";
      result->runtime_context_valid = false;
      result->recovery_required = false;
      publish_transition_status(
        input.request.transaction_id, "CANCELED", robot_floor_manager::to_string(code),
        result->failure_code, result->message, &input.request);
      release_floor_switch_action(input.request.transaction_id);
      goal_handle->canceled(result);
      return;
    }

    auto decision = robot_floor_manager::evaluate_floor_switch_preflight(input);
    if (decision.ready) {
      decision.ready = false;
      decision.failure_code = robot_floor_manager::FloorSwitchFailureCode::kInternalError;
      decision.code = robot_floor_manager::to_string(decision.failure_code);
      decision.detail =
        "preflight-only adapter contains no map, filter, localizer, bridge, or costmap mutation port";
    }

    auto result = std::make_shared<FloorSwitchAction::Result>();
    result->success = false;
    result->failure_code = static_cast<std::uint16_t>(decision.failure_code);
    result->message = decision.code + ": " + decision.detail;
    result->asset_epoch = 0U;
    result->explicit_relocalization_sequence = 0U;
    result->runtime_context_valid = false;
    result->recovery_required = false;
    if (goal_handle->is_canceling()) {
      const auto code = robot_floor_manager::FloorSwitchFailureCode::kCancelledBeforeMutation;
      result->failure_code = static_cast<std::uint16_t>(code);
      result->message =
        std::string(robot_floor_manager::to_string(code)) +
        ": cancellation arrived before any runtime mutation";
      publish_transition_status(
        input.request.transaction_id, "CANCELED", robot_floor_manager::to_string(code),
        result->failure_code, result->message, &input.request);
      release_floor_switch_action(input.request.transaction_id);
      goal_handle->canceled(result);
      return;
    }
    publish_transition_status(
      input.request.transaction_id, "BLOCKED", decision.code,
      result->failure_code, result->message, &input.request);
    release_floor_switch_action(input.request.transaction_id);
    goal_handle->abort(result);
  }

  std::chrono::nanoseconds service_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  }

  std::chrono::nanoseconds localization_trigger_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        std::max(service_timeout_sec_, localization_trigger_timeout_sec_)));
  }

  std::chrono::nanoseconds localizer_apply_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        std::max(service_timeout_sec_, localizer_apply_timeout_sec_)));
  }

  std::chrono::nanoseconds filter_mask_state_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(filter_mask_state_timeout_sec_));
  }

  std::chrono::nanoseconds evidence_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(evidence_timeout_sec_));
  }

  bool wait_for_evidence(
    const std::function<robot_floor_manager::FloorTransitionEvidence()> & evaluate,
    const std::function<bool(
      const robot_floor_manager::FloorTransitionEvidence &)> & proven,
    robot_floor_manager::FloorTransitionEvidence & evidence)
  {
    const auto deadline = std::chrono::steady_clock::now() + evidence_timeout();
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (!shutting_down_.load() && !cancel_requested_.load()) {
      evidence = evaluate();
      if (proven(evidence)) {
        return true;
      }
      if (evidence_changed_.wait_until(lock, deadline) ==
        std::cv_status::timeout)
      {
        evidence = evaluate();
        return proven(evidence);
      }
    }
    return false;
  }

  bool wait_for_condition(const std::function<bool()> & proven)
  {
    const auto deadline = std::chrono::steady_clock::now() + evidence_timeout();
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    while (!shutting_down_.load() && !cancel_requested_.load()) {
      if (proven()) {
        return true;
      }
      if (evidence_changed_.wait_until(lock, deadline) ==
        std::cv_status::timeout)
      {
        return proven();
      }
    }
    return false;
  }

  bool set_motion_hold(
    const std::string & transaction_id,
    const std::uint8_t operation,
    std::string & error)
  {
    if (!wait_for_service(motion_hold_client_, motion_hold_service_, error)) {
      return false;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::SetMotionHold::Request>();
    request->owner = "robot_floor_manager";
    request->transaction_id = transaction_id;
    request->reason = "atomic_floor_switch";
    request->operation = operation;
    if (!motion_hold_sequence_allocator_) {
      error =
        "persistent floor-manager hold sequence allocator is unavailable";
      return false;
    }
    const auto sequence = motion_hold_sequence_allocator_->next();
    if (!sequence.has_value()) {
      error =
        "persistent floor-manager hold command sequence block is exhausted";
      return false;
    }
    request->command_sequence = *sequence;
    std::shared_ptr<robot_interfaces::srv::SetMotionHold::Response> response;
    try {
      motion_hold_command_submitted_.store(true);
      motion_hold_outcome_unknown_.store(true);
      auto future = motion_hold_client_->async_send_request(request);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        error =
          "timed out changing floor-manager motion hold; submitted outcome is unknown";
        return false;
      }
      response = future.get();
      motion_hold_outcome_unknown_.store(false);
    } catch (const std::exception & exception) {
      error =
        "floor-manager motion-hold response is unknown after submission: " +
        std::string(exception.what());
      return false;
    }
    std::uint64_t state_generation_after_response = 0U;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      evidence_tracker_.observe_motion_interlock(
        response->state.hold_active,
        response->state.hold_keys,
        steady_now_sec());
      state_generation_after_response = motion_interlock_state_generation_;
      evidence_changed_.notify_all();
    }
    if (!response->success) {
      if (
        response->result_code ==
        robot_interfaces::srv::SetMotionHold::Response::RESULT_STALE_COMMAND)
      {
        motion_hold_sequence_allocator_->synchronize(
          response->applied_sequence);
      }
      error = "robot_safety rejected floor-manager hold: " + response->message;
      return false;
    }
    if (response->applied_sequence != request->command_sequence) {
      error =
        "robot_safety did not prove the exact floor-manager hold sequence";
      return false;
    }
    const auto key = "robot_floor_manager:" + transaction_id;
    const bool present =
      contains_exact_key(response->state.hold_keys, key);
    const bool acquiring =
      operation == robot_interfaces::srv::SetMotionHold::Request::OP_ACQUIRE;
    if (present != acquiring) {
      error = acquiring ?
        "motion-hold response did not contain exact floor-manager key" :
        "motion-hold response retained exact floor-manager key";
      return false;
    }
    if (!wait_for_motion_hold_state(
        transaction_id, acquiring, state_generation_after_response, error))
    {
      return false;
    }
    motion_hold_acquired_.store(acquiring);
    if (!acquiring) {
      motion_hold_command_submitted_.store(false);
    }
    return true;
  }

  bool wait_for_motion_hold_state(
    const std::string & transaction_id,
    const bool expected_present,
    const std::uint64_t generation_after_response,
    std::string & error)
  {
    const auto key = "robot_floor_manager:" + transaction_id;
    const auto deadline = std::chrono::steady_clock::now() + evidence_timeout();
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    const auto proven =
      [this, &key, expected_present, generation_after_response]() {
        const bool present =
          contains_exact_key(last_motion_interlock_hold_keys_, key);
        return
          motion_interlock_state_generation_ > generation_after_response &&
          present == expected_present &&
          (!expected_present || last_motion_interlock_hold_active_);
      };
    while (!shutting_down_.load()) {
      if (proven()) {
        return true;
      }
      if (evidence_changed_.wait_until(lock, deadline) ==
        std::cv_status::timeout)
      {
        break;
      }
    }
    error = expected_present ?
      "fresh MotionInterlockState did not prove the exact floor-manager hold" :
      "fresh MotionInterlockState did not prove the exact floor-manager hold absent";
    return false;
  }

  bool set_correction_pause(
    const std::string & transaction_id,
    const std::uint8_t operation,
    std::string & error)
  {
    if (!wait_for_service(
        correction_pause_client_, correction_pause_service_, error))
    {
      return false;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::SetCorrectionPause::Request>();
    request->owner = "robot_floor_manager";
    request->transaction_id = transaction_id;
    request->reason = "atomic_floor_switch";
    request->operation = operation;
    if (!motion_hold_sequence_allocator_) {
      error =
        "persistent floor-manager command sequence allocator is unavailable";
      return false;
    }
    const auto sequence = motion_hold_sequence_allocator_->next();
    if (!sequence.has_value()) {
      error =
        "persistent floor-manager command sequence block is exhausted";
      return false;
    }
    request->command_sequence = *sequence;
    std::shared_ptr<robot_interfaces::srv::SetCorrectionPause::Response> response;
    try {
      correction_pause_command_submitted_.store(true);
      correction_pause_outcome_unknown_.store(true);
      auto future = correction_pause_client_->async_send_request(request);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        error =
          "timed out changing floor-manager correction pause; submitted outcome is unknown";
        return false;
      }
      response = future.get();
      correction_pause_outcome_unknown_.store(false);
    } catch (const std::exception & exception) {
      error =
        "floor-manager correction-pause response is unknown after submission: " +
        std::string(exception.what());
      return false;
    }
    std::uint64_t state_generation_after_response = 0U;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      evidence_tracker_.observe_correction_pause(
        response->state.paused,
        response->state.lease_keys,
        steady_now_sec());
      state_generation_after_response = correction_pause_state_generation_;
      evidence_changed_.notify_all();
    }
    if (!response->success) {
      if (
        response->result_code ==
        robot_interfaces::srv::SetCorrectionPause::Response::RESULT_STALE_COMMAND)
      {
        motion_hold_sequence_allocator_->synchronize(
          response->applied_sequence);
      }
      error =
        "localization bridge rejected floor-manager correction pause: " +
        response->message;
      return false;
    }
    if (response->applied_sequence != request->command_sequence) {
      error =
        "localization bridge did not prove the exact correction-pause sequence";
      return false;
    }
    const auto key = "robot_floor_manager:" + transaction_id;
    const bool present =
      contains_exact_key(response->state.lease_keys, key);
    const bool acquiring =
      operation ==
      robot_interfaces::srv::SetCorrectionPause::Request::OP_ACQUIRE;
    if (present != acquiring) {
      error = acquiring ?
        "correction-pause response did not contain exact floor-manager key" :
        "correction-pause response retained exact floor-manager key";
      return false;
    }
    if (!wait_for_correction_pause_state(
        transaction_id, acquiring, state_generation_after_response, error))
    {
      return false;
    }
    floor_pause_acquired_ = acquiring;
    if (!acquiring) {
      correction_pause_command_submitted_.store(false);
    }
    return true;
  }

  bool wait_for_correction_pause_state(
    const std::string & transaction_id,
    const bool expected_present,
    const std::uint64_t generation_after_response,
    std::string & error)
  {
    const auto key = "robot_floor_manager:" + transaction_id;
    const auto deadline = std::chrono::steady_clock::now() + evidence_timeout();
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    const auto proven =
      [this, &key, expected_present, generation_after_response]() {
        const bool present =
          contains_exact_key(last_correction_pause_keys_, key);
        return
          correction_pause_state_generation_ > generation_after_response &&
          present == expected_present &&
          (!expected_present || last_correction_pause_active_);
      };
    while (!shutting_down_.load()) {
      if (proven()) {
        return true;
      }
      if (evidence_changed_.wait_until(lock, deadline) ==
        std::cv_status::timeout)
      {
        break;
      }
    }
    error = expected_present ?
      "fresh CorrectionPauseState did not prove the exact floor-manager lease" :
      "fresh CorrectionPauseState did not prove the exact floor-manager lease absent";
    return false;
  }

  std::shared_ptr<robot_interfaces::srv::BeginFloorTransition::Response>
  call_bridge_transition(
    const robot_floor_manager::FloorTransitionEffect & effect,
    const std::uint8_t operation,
    std::string & error)
  {
    if (!wait_for_service(
        begin_floor_transition_client_,
        begin_floor_transition_service_, error))
    {
      return nullptr;
    }
    auto request =
      std::make_shared<robot_interfaces::srv::BeginFloorTransition::Request>();
    request->transaction_id = effect.transaction_id;
    request->building_id = effect.building_id;
    request->floor_id = effect.floor_id;
    request->map_id = effect.map_id;
    request->asset_epoch = effect.expected_asset_epoch;
    request->asset_digest = effect.expected_asset_digest;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      if (source_runtime_context_.has_value()) {
        request->source_building_id =
          source_runtime_context_->building_id;
        request->source_floor_id = source_runtime_context_->floor_id;
        request->source_map_id = source_runtime_context_->map_id;
        request->source_asset_epoch =
          source_runtime_context_->asset_epoch;
        request->source_asset_digest =
          source_runtime_context_->asset_digest;
      }
    }
    request->operation = operation;
    if (!motion_hold_sequence_allocator_) {
      error =
        "persistent floor-manager command sequence allocator is unavailable";
      return nullptr;
    }
    const auto sequence = motion_hold_sequence_allocator_->next();
    if (!sequence.has_value()) {
      error =
        "persistent floor-manager command sequence block is exhausted";
      return nullptr;
    }
    request->command_sequence = *sequence;
    std::shared_ptr<robot_interfaces::srv::BeginFloorTransition::Response> response;
    try {
      if (
        operation ==
        robot_interfaces::srv::BeginFloorTransition::Request::OP_BEGIN)
      {
        bridge_begin_submitted_ = true;
        bridge_begin_outcome_unknown_ = true;
      }
      auto future =
        begin_floor_transition_client_->async_send_request(request);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        error =
          "timed out calling bridge floor-transition fence; submitted outcome is unknown";
        return nullptr;
      }
      response = future.get();
      if (
        operation ==
        robot_interfaces::srv::BeginFloorTransition::Request::OP_BEGIN)
      {
        bridge_begin_established_ = response->success;
        bridge_begin_outcome_unknown_ =
          bridge_begin_rejection_requires_recovery(
          response->success, response->runtime_context_valid);
        if (!response->success && response->runtime_context_valid) {
          bridge_begin_submitted_ = false;
        }
      }
    } catch (const std::exception & exception) {
      error =
        "bridge floor-transition response is unknown after submission: " +
        std::string(exception.what());
      return nullptr;
    }
    if (!response->success) {
      if (
        response->result_code ==
        robot_interfaces::srv::BeginFloorTransition::Response::RESULT_STALE_COMMAND)
      {
        motion_hold_sequence_allocator_->synchronize(
          response->applied_sequence);
      }
      error = "bridge floor-transition fence rejected request: " +
        response->message;
      return nullptr;
    }
    if (response->applied_sequence != request->command_sequence) {
      error =
        "bridge floor-transition response did not prove the exact command sequence";
      return nullptr;
    }
    // Only ABORT_PREMUTATION can restore the source identity. Ordinary ABORT
    // retains an invalid context. BEGIN and COMMIT must echo the target epoch.
    if (
      bridge_response_requires_target_epoch(operation) &&
      response->accepted_asset_epoch != effect.expected_asset_epoch)
    {
      error = "bridge floor-transition response epoch mismatch";
      return nullptr;
    }
    return response;
  }

  bool wait_for_source_runtime_context_after(
    const std::uint64_t generation_after_response,
    const std::uint64_t restored_explicit_sequence,
    std::string & error)
  {
    const auto deadline = std::chrono::steady_clock::now() + evidence_timeout();
    std::unique_lock<std::mutex> lock(evidence_mutex_);
    if (!source_runtime_context_.has_value()) {
      error =
        "source runtime identity was not fresh and valid before bridge BEGIN";
      return false;
    }
    const auto proven =
      [this, generation_after_response, restored_explicit_sequence]() {
        const auto observed_at = steady_now_sec();
        return
          localization_health_generation_ > generation_after_response &&
          last_localization_health_.has_value() &&
          last_localization_health_received_steady_sec_ >= 0.0 &&
          observed_at >= last_localization_health_received_steady_sec_ &&
          observed_at - last_localization_health_received_steady_sec_ <=
          evidence_max_age_sec_ &&
          same_runtime_identity(
            *last_localization_health_, *source_runtime_context_) &&
          // BEGIN may follow a newer source fix than the cached preflight
          // sample. Compare against the restore response, not that old sample.
          last_localization_health_->explicit_relocalization_sequence ==
          restored_explicit_sequence &&
          last_localization_health_->runtime_context_valid &&
          !last_localization_health_->transition_active &&
          last_localization_health_->localizer_ready &&
          last_localization_health_->bridge_ready &&
          last_localization_health_->tf_unique;
      };
    while (!shutting_down_.load()) {
      if (proven()) {
        // Persist the same source generation that was just proven, rather than
        // the possibly older sample captured before pause acquisition/BEGIN.
        source_runtime_context_ = last_localization_health_;
        return true;
      }
      if (evidence_changed_.wait_until(lock, deadline) ==
        std::cv_status::timeout)
      {
        break;
      }
    }
    error =
      "fresh LocalizationHealth did not prove the exact source runtime context restored";
    return false;
  }

  bool abort_bridge_and_prove_source(
    const robot_floor_manager::FloorTransitionEffect & effect,
    std::string & error)
  {
    const auto response = call_bridge_transition(
      effect,
      robot_interfaces::srv::BeginFloorTransition::Request::OP_ABORT_PREMUTATION,
      error);
    if (!response) {
      return false;
    }
    if (!response->runtime_context_valid) {
      error =
        "bridge pre-mutation ABORT did not restore a valid source runtime context";
      return false;
    }
    std::uint64_t health_generation_after_response = 0U;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      health_generation_after_response = localization_health_generation_;
    }
    if (!wait_for_source_runtime_context_after(
        health_generation_after_response,
        response->explicit_relocalization_sequence, error))
    {
      return false;
    }
    bridge_begin_outcome_unknown_ = false;
    return true;
  }

  bool write_runtime_context(
    const robot_floor_manager::FloorTransitionEffect & effect,
    const std::string & state,
    const bool confirmed,
    const std::string & message,
    std::string & error)
  {
    robot_floor_manager::RuntimeMapContextRecord record;
    record.state = state;
    record.confirmed = confirmed;
    record.message = message;
    record.transaction_id = effect.transaction_id;
    record.building_id = effect.building_id;
    record.floor_id = effect.floor_id;
    record.map_id = effect.map_id;
    record.asset_epoch = effect.expected_asset_epoch;
    record.asset_digest = effect.expected_asset_digest;
    record.localizer_generation = localizer_generation_;
    record.explicit_relocalization_sequence =
      explicit_relocalization_sequence_;
    record.updated_at_sec =
      static_cast<double>(now().nanoseconds()) * 1.0e-9;
    return robot_floor_manager::AtomicRuntimeMapContextWriter{}.write(
      fs::path(runtime_map_context_file_), record, error);
  }

  bool write_source_runtime_context(
    const robot_floor_manager::FloorTransitionEffect & effect,
    const std::string & message,
    std::string & error)
  {
    std::optional<robot_floor_manager::LocalizationHealthEvidence> source;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      source = source_runtime_context_;
    }
    if (!source.has_value()) {
      error = "source runtime identity is unavailable for durable recovery";
      return false;
    }

    robot_floor_manager::RuntimeMapContextRecord record;
    record.state = "ready";
    record.confirmed = true;
    record.message = message;
    record.transaction_id = effect.transaction_id;
    record.building_id = source->building_id;
    record.floor_id = source->floor_id;
    record.map_id = source->map_id;
    record.asset_epoch = source->asset_epoch;
    record.asset_digest = source->asset_digest;
    record.localizer_generation = source->localizer_generation;
    record.explicit_relocalization_sequence =
      source->explicit_relocalization_sequence;
    record.updated_at_sec =
      static_cast<double>(now().nanoseconds()) * 1.0e-9;
    return robot_floor_manager::AtomicRuntimeMapContextWriter{}.write(
      fs::path(runtime_map_context_file_), record, error);
  }

  template<typename ClientT>
  bool wait_for_service(const ClientT & client, const std::string & name, std::string & error)
  {
    if (client->wait_for_service(service_timeout())) {
      return true;
    }
    error = "service unavailable: " + name;
    return false;
  }

  bool validate_floor_assets(
    const std::string & building_id,
    const std::string & floor_id,
    FloorAssets & assets,
    std::string & error) const
  {
    assets.building_id = building_id.empty() ? default_building_id_ : building_id;
    assets.floor_id = floor_id;
    const auto floor_root = fs::path(maps_root_) / assets.building_id / assets.floor_id;
    const auto current_root = floor_root / "current";
    assets.root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor_root;
    assets.nav_map_yaml = assets.root / "nav" / "nav_map.yaml";
    assets.nav_map_pgm = assets.root / "nav" / "nav_map.pgm";
    assets.localizer_map_png = assets.root / "localizer" / "localizer_map.png";
    assets.localizer_params_yaml = assets.root / "localizer" / "localizer_params.yaml";
    assets.keepout_mask_yaml = assets.root / "filters" / "keepout_mask.yaml";
    assets.keepout_mask_pgm = assets.root / "filters" / "keepout_mask.pgm";
    assets.speed_mask_yaml = assets.root / "filters" / "speed_mask.yaml";
    assets.speed_mask_pgm = assets.root / "filters" / "speed_mask.pgm";
    assets.binary_mask_yaml = assets.root / "filters" / "binary_mask.yaml";
    assets.binary_mask_pgm = assets.root / "filters" / "binary_mask.pgm";
    assets.asset_report_json = assets.root / "reports" / "asset_report.json";
    assets.poses_yaml = assets.root / "poses.yaml";
    assets.filters = {
      assets.keepout_mask_yaml,
      assets.keepout_mask_pgm,
      assets.speed_mask_yaml,
      assets.speed_mask_pgm,
      assets.binary_mask_yaml,
      assets.binary_mask_pgm,
    };

    if (assets.floor_id.empty()) {
      error = "floor_id is required";
      return false;
    }

    std::vector<fs::path> required = {
      assets.nav_map_yaml,
      assets.nav_map_pgm,
      assets.localizer_map_png,
      assets.localizer_params_yaml,
      assets.asset_report_json,
      assets.poses_yaml,
    };
    if (require_filter_assets_) {
      required.insert(required.end(), assets.filters.begin(), assets.filters.end());
    }

    std::vector<std::string> missing;
    for (const auto & path : required) {
      if (!fs::exists(path)) {
        missing.push_back(path.string());
      }
    }
    if (!missing.empty()) {
      error = "floor asset validation failed: " + join_missing(missing);
      return false;
    }
    return true;
  }

  bool validate_exact_floor_assets(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id,
    const std::uint64_t expected_asset_epoch,
    const std::string & expected_asset_digest,
    FloorAssets & assets,
    std::string & error_code,
    std::string & error) const
  {
    const robot_floor_manager::FloorAssetSnapshotRequest request{
      fs::path(maps_root_),
      building_id.empty() ? default_building_id_ : building_id,
      floor_id,
      map_id,
      expected_asset_epoch,
      expected_asset_digest,
    };
    const auto result =
      robot_floor_manager::FloorAssetSnapshotLoader{}.load(request);
    if (!result.ok()) {
      error_code =
        robot_floor_manager::floor_asset_snapshot_error_name(result.error);
      error = result.message;
      return false;
    }

    const auto & snapshot = *result.snapshot;
    assets.building_id = snapshot.building_id;
    assets.floor_id = snapshot.floor_id;
    assets.map_id = snapshot.map_id;
    assets.asset_epoch = snapshot.asset_epoch;
    assets.asset_digest = snapshot.asset_digest;
    assets.root = snapshot.paths.root;
    assets.nav_map_yaml = snapshot.paths.nav_map_yaml;
    assets.nav_map_pgm = snapshot.paths.nav_map_pgm;
    assets.localizer_map_png = snapshot.paths.localizer_map_png;
    assets.localizer_params_yaml = snapshot.paths.localizer_params_yaml;
    assets.keepout_mask_yaml = snapshot.paths.keepout_mask_yaml;
    assets.keepout_mask_pgm = snapshot.paths.keepout_mask_pgm;
    assets.speed_mask_yaml = snapshot.paths.speed_mask_yaml;
    assets.speed_mask_pgm = snapshot.paths.speed_mask_pgm;
    assets.binary_mask_yaml = snapshot.paths.binary_mask_yaml;
    assets.binary_mask_pgm = snapshot.paths.binary_mask_pgm;
    assets.asset_report_json = snapshot.paths.asset_report_json;
    assets.poses_yaml = snapshot.paths.poses_yaml;
    assets.filters = {
      assets.keepout_mask_yaml,
      assets.keepout_mask_pgm,
      assets.speed_mask_yaml,
      assets.speed_mask_pgm,
      assets.binary_mask_yaml,
      assets.binary_mask_pgm,
    };
    error_code = "OK";
    error.clear();
    return true;
  }

  bool filter_mask_server_is_active(
    const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr & client,
    const std::string & service_name,
    const std::string & label)
  {
    if (!client->wait_for_service(filter_mask_state_timeout())) {
      RCLCPP_WARN(
        get_logger(),
        "%s lifecycle state service unavailable (%s); deferring filter mask reload to Nav2 startup",
        label.c_str(), service_name.c_str());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(filter_mask_state_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(),
        "timed out querying %s lifecycle state (%s); deferring filter mask reload to Nav2 startup",
        label.c_str(), service_name.c_str());
      return false;
    }

    const auto response = future.get();
    if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(),
        "%s is %s [%u]; deferring filter mask reload to Nav2 startup",
        label.c_str(), response->current_state.label.c_str(), response->current_state.id);
      return false;
    }
    return true;
  }

  bool filter_mask_servers_are_active()
  {
    for (const auto role : runtime_filter_reload_roles(speed_filter_enabled_)) {
      if (role == RuntimeFilterRole::kKeepout) {
        if (!filter_mask_server_is_active(
            keepout_mask_state_client_,
            keepout_mask_state_service_,
            "keepout_filter_mask_server"))
        {
          return false;
        }
        continue;
      }
      if (!filter_mask_server_is_active(
          speed_mask_state_client_,
          speed_mask_state_service_,
          "speed_filter_mask_server"))
      {
        return false;
      }
    }
    return true;
  }

  static bool localizer_apply_failed_before_mutation(const std::string & code)
  {
    return code.rfind("ASSET_", 0U) == 0U ||
      code == "TRANSACTION_ID_INVALID" || code == "COMPONENT_CONFIG_INVALID" ||
      code == "COMPONENT_MANAGER_UNAVAILABLE" || code == "COMPONENT_LIST_TIMEOUT" ||
      code == "LOCALIZER_OPERATION_BUSY" || code == "LOCALIZER_COMPONENT_AMBIGUOUS" ||
      code == "LOCALIZER_PARAMETER_CAPTURE_INVALID" || code == "LOCALIZER_PARAMETER_LIST_TIMEOUT" ||
      code == "LOCALIZER_PARAMETER_GET_TIMEOUT" || code == "LOCALIZER_PNG_INVALID" ||
      code == "NAV_MAP_YAML_INVALID" || code == "LOCALIZER_YAML_INVALID" ||
      code == "LOCALIZER_IMAGE_MISMATCH";
  }

  template<typename Response>
  static bool target_response_settled(const std::shared_ptr<Response> & response)
  {
    return response != nullptr;
  }

  static bool target_response_settled(
    const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> & response)
  {
    if (!response) {return false;}
    if (response->accepted) {return true;}
    // The wrapper can return before its nested Isaac or force-accept request.
    // In particular "not_dispatched" describes Isaac, not the prior bridge arm.
    if (response->message.find("BRIDGE_FORCE_ACCEPT_TIMEOUT") != std::string::npos ||
      response->message.find("BRIDGE_FORCE_ACCEPT_FAILED") != std::string::npos)
    {return false;}
    return response->message.find("dispatch_state=not_dispatched") != std::string::npos;
  }

  static bool target_response_settled(
    const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> & response)
  {
    if (!response) {return false;}
    if (response->success || localizer_apply_failed_before_mutation(response->code)) {return true;}
    return response->rollback_succeeded && response->code.find("TIMEOUT") == std::string::npos &&
      response->code.find("EXCEPTION") == std::string::npos &&
      response->code.find("UNKNOWN") == std::string::npos;
  }

  // Keep the real response future alive across a timeout. "Was dispatched"
  // determines whether source restoration is legal; it must not mean "still
  // executing forever". A callback exception does not prove server completion.
  template<typename Client, typename Request>
  auto dispatch_target_request(
    const std::shared_ptr<Client> & client,
    const std::shared_ptr<Request> & request,
    std::shared_ptr<std::atomic_bool> * settlement = nullptr)
  {
    auto settled = std::make_shared<std::atomic_bool>(false);
    target_requests_.push_back({settled, client, {}});
    if (settlement) {*settlement = settled;}
    target_effect_dispatched_ = true;
    auto future = client->async_send_request(request,
      [](typename Client::SharedFuture) {});
    const auto observed = future.future;
    target_requests_.back().response_settled = [observed]() {
        if (observed.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
          return false;
        }
        try {
          return target_response_settled(observed.get());
        } catch (const std::exception &) {
          // Retain this exact unresolved request, not a historic failure bit.
          return false;
        }
      };
    return future;
  }

  bool target_requests_settled() const
  {
    return std::all_of(target_requests_.begin(), target_requests_.end(),
      [](const auto & request) {
        return request.settled->load() ||
          (request.response_settled && request.response_settled());
      });
  }

  bool startup_effects_settled()
  {
    std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
    if (!startup_handoff_) {return true;}
    const auto ack = robot_floor_manager::read_floor_startup_handoff_ack(
      startup_handoff_ack_file_, *startup_handoff_);
    // A failed request causes the startup owner to tear down its work. Neither
    // its earlier adopted nor runtime_ready ACK proves that teardown is over.
    return ack && ack->state == "failed" &&
      ack->cleanup_completed == std::optional<bool>(true) &&
      ack->effects_settled == std::optional<bool>(true);
  }

  bool startup_owner_exited()
  {
    std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
    if (!startup_handoff_) {return false;}
    const auto ack = robot_floor_manager::read_floor_startup_handoff_ack(
      startup_handoff_ack_file_, *startup_handoff_);
    return ack && ack->state == "failed" &&
      ack->cleanup_completed == std::optional<bool>(true) &&
      ack->owner_available == std::optional<bool>(false);
  }

  void reconcile_deferred_failure()
  {
    bool owner_exit_reported = false;
    while (deferred_failure_cleanup_.load() && !shutting_down_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if (!owner_exit_reported && startup_owner_exited()) {
        owner_exit_reported = true;
        publish_transition_status(deferred_failure_effect_.transaction_id,
          "FAILED_LOCKED", "RECOVERY_LOCKED", deferred_failure_code_,
          "STARTUP_OWNER_EXITED; " + deferred_failure_effect_.detail +
          "; startup owner cannot consume another handoff; cleanup evidence remains separate");
      }
      if (!target_requests_settled() || !startup_effects_settled() ||
        shutting_down_.load()) {continue;}
      const auto effect = deferred_failure_effect_;
      const auto snapshot = deferred_failure_snapshot_;
      try {
        const auto cleanup = perform(effect, snapshot);
        if (cleanup.success && cleanup.evidence.failure_resources_released) {
          publish_transition_status(effect.transaction_id, "FAILED", "FAILED", deferred_failure_code_,
            (owner_exit_reported ? std::string("STARTUP_OWNER_EXITED; ") : std::string{}) +
            effect.detail + "; late requests settled and owned resources released");
          return;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "floor failure resource cleanup not yet proven: %s", error.what());
      }
      // The action result is already returned. Retry only its exact cleanup,
      // without holding a ROS callback or restarting the failed floor switch.
      for (int interval = 0; interval < 8 && !shutting_down_.load(); ++interval) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  }

  enum class MapLoadFailure {kOther, kServiceUnavailable, kTimeout, kInvalidAsset};

  static std::string map_load_failure_code(
    const std::string & prefix, const MapLoadFailure failure)
  {
    switch (failure) {
      case MapLoadFailure::kServiceUnavailable: return prefix + "_SERVICE_UNAVAILABLE";
      case MapLoadFailure::kTimeout: return prefix + "_TIMEOUT";
      case MapLoadFailure::kInvalidAsset: return prefix + "_INVALID_ASSET";
      case MapLoadFailure::kOther: return prefix + "_FAILED";
    }
    return prefix + "_FAILED";
  }

  bool load_map_with_client(
    const rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr & client,
    const std::string & service_name,
    const fs::path & map_yaml,
    const std::string & label,
    std::string & error,
    MapLoadFailure & failure)
  {
    failure = MapLoadFailure::kOther;
    if (!wait_for_service(client, service_name, error)) {
      failure = MapLoadFailure::kServiceUnavailable;
      return false;
    }
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_yaml.string();
    auto future = dispatch_target_request(client, request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      failure = MapLoadFailure::kTimeout;
      error = "timed out loading " + label + ": " + map_yaml.string();
      return false;
    }
    const auto response = future.get();
    if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      using Response = nav2_msgs::srv::LoadMap::Response;
      if (response->result == Response::RESULT_MAP_DOES_NOT_EXIST ||
        response->result == Response::RESULT_INVALID_MAP_DATA ||
        response->result == Response::RESULT_INVALID_MAP_METADATA)
      {
        failure = MapLoadFailure::kInvalidAsset;
      }
      error = service_name + " rejected " + label + " with result code " + std::to_string(response->result);
      return false;
    }
    return true;
  }

  bool load_nav_map(
    const FloorAssets & assets, std::string & error, MapLoadFailure & failure)
  {
    failure = MapLoadFailure::kOther;
    if (!call_map_server_load_) {
      error = "live Nav2 map reload is disabled by configuration";
      return false;
    }
    return load_map_with_client(
      map_load_client_, map_server_load_service_, assets.nav_map_yaml, "Nav2 map", error, failure);
  }

  bool load_filter_masks(
    const FloorAssets & assets, std::string & error, MapLoadFailure & failure)
  {
    failure = MapLoadFailure::kOther;
    if (!call_filter_mask_load_) {
      error = "live filter-mask reload is disabled by configuration";
      return false;
    }
    if (has_startup_handoff() &&
      (!prepare_mask_lifecycle(keepout_mask_state_service_, error) ||
      (speed_filter_enabled_ && !prepare_mask_lifecycle(speed_mask_state_service_, error))))
    {
      return false;
    }
    if (!filter_mask_servers_are_active()) {
      error =
        "live filter-mask reload requires active keepout and speed mask servers";
      return false;
    }
    if (!load_map_with_client(
        keepout_mask_load_client_, keepout_mask_load_service_, assets.keepout_mask_yaml,
        "keepout mask", error, failure))
    {
      return false;
    }
    if (!speed_filter_enabled_) {
      return true;
    }
    return load_map_with_client(
      speed_mask_load_client_, speed_mask_load_service_, assets.speed_mask_yaml,
      "speed mask", error, failure);
  }

  bool apply_localizer_assets(
    const std::string & transaction_id,
    const FloorAssets & assets,
    const std::uint64_t baseline_generation,
    std::uint64_t & accepted_generation,
    std::string & error)
  {
    if (!call_localizer_apply_) {
      error = "live localizer reload is disabled by configuration";
      return false;
    }
    if (!wait_for_service(localizer_apply_client_, localizer_apply_service_, error)) {
      return false;
    }
    auto request = std::make_shared<robot_interfaces::srv::ApplyFloorAssets::Request>();
    request->transaction_id = transaction_id;
    request->building_id = assets.building_id;
    request->floor_id = assets.floor_id;
    request->map_id = assets.map_id;
    request->asset_epoch = assets.asset_epoch;
    request->asset_digest = assets.asset_digest;
    request->nav_map_yaml = assets.nav_map_yaml.string();
    request->localizer_map_png = assets.localizer_map_png.string();
    request->localizer_params_yaml = assets.localizer_params_yaml.string();
    std::shared_ptr<std::atomic_bool> request_settled;
    auto future = dispatch_target_request(localizer_apply_client_, request, &request_settled);
    const auto total_timeout = localizer_apply_timeout();
    const auto deadline = std::chrono::steady_clock::now() + total_timeout;
    std::string rpc_error;

    while (!shutting_down_.load() && !cancel_requested_.load()) {
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          const auto response = future.get();
          if (!response->success) {
            error =
              "global localization rejected floor assets [" + response->code +
              "]: " + response->message;
            return false;
          }
          if (
            !response->reloaded ||
            response->transaction_id != transaction_id ||
            response->building_id != assets.building_id ||
            response->floor_id != assets.floor_id ||
            response->map_id != assets.map_id ||
            response->asset_epoch != assets.asset_epoch ||
            response->asset_digest != assets.asset_digest ||
            response->localizer_generation <= baseline_generation)
          {
            error =
              "global localization response did not prove an exact new target reload";
            return false;
          }
          accepted_generation = response->localizer_generation;
          return true;
        } catch (const std::exception & exception) {
          rpc_error =
            std::string("global localization floor-asset response failed: ") +
            exception.what();
        }
      }

      const auto now = std::chrono::steady_clock::now();
      robot_floor_manager::LocalizerApplyOutcome outcome;
      {
        std::unique_lock<std::mutex> lock(evidence_mutex_);
        outcome = evidence_tracker_.localizer_apply_outcome(
          baseline_generation, steady_now_sec(), evidence_max_age_sec_);
        if (!outcome.terminal && now < deadline) {
          const auto poll_deadline = std::min(
            deadline, now + std::chrono::milliseconds(50));
          evidence_changed_.wait_until(lock, poll_deadline);
          continue;
        }
      }

      if (outcome.terminal) {
        // Exact typed terminal state is completion evidence even when the RPC
        // response was lost; it is not merely a timeout or a local cancellation.
        if (outcome.success || localizer_apply_failed_before_mutation(outcome.code)) {
          request_settled->store(true);
        }
        if (!outcome.success) {
          error = "global localization rejected floor assets from exact typed state";
          if (!outcome.code.empty()) {
            error += " [" + outcome.code + "]";
          }
          if (!outcome.detail.empty()) {
            error += ": " + outcome.detail;
          }
          return false;
        }
        accepted_generation = outcome.localizer_generation;
        RCLCPP_WARN(
          get_logger(),
          "localizer apply RPC response was delayed or lost; reconciled exact "
          "transaction=%s generation=%llu from typed asset state",
          transaction_id.c_str(),
          static_cast<unsigned long long>(accepted_generation));
        return true;
      }

      if (now >= deadline) {
        break;
      }
    }

    if (shutting_down_.load() || cancel_requested_.load()) {
      error = "cancelled while applying localizer floor assets";
      return false;
    }
    error =
      "localizer floor-asset transaction produced neither an exact RPC response "
      "nor exact terminal typed state within " +
      std::to_string(std::chrono::duration<double>(total_timeout).count()) + "s";
    if (!rpc_error.empty()) {
      error += "; " + rpc_error;
    }
    return false;
  }

  bool trigger_localization(const FloorAssets & assets, std::string & error)
  {
    if (!call_localization_trigger_) {
      error = "explicit target localization is disabled by configuration";
      return false;
    }

    const auto total_timeout = localization_trigger_timeout();
    const auto deadline = std::chrono::steady_clock::now() + total_timeout;
    std::size_t attempt = 0U;

    while (!shutting_down_.load() && !cancel_requested_.load()) {
      const auto before_service_wait = std::chrono::steady_clock::now();
      if (before_service_wait >= deadline) {
        break;
      }
      const auto service_wait = std::min(
        service_timeout(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          deadline - before_service_wait));
      if (!localization_trigger_client_->wait_for_service(service_wait)) {
        if (shutting_down_.load() || cancel_requested_.load()) {
          error = "cancelled while waiting for global localization trigger service";
          return false;
        }
        RCLCPP_WARN(
          get_logger(),
          "global localization trigger service unavailable inside active floor transaction; "
          "retrying within the %.3fs total budget",
          std::chrono::duration<double>(total_timeout).count());
        continue;
      }

      ++attempt;
      auto request = std::make_shared<robot_interfaces::srv::TriggerLocalization::Request>();
      request->reason = "floor_switch:" + assets.building_id + "/" + assets.floor_id;
      startup_target_trigger_dispatched_ = true;
      auto future = dispatch_target_request(localization_trigger_client_, request);

      bool response_ready = false;
      while (!shutting_down_.load() && !cancel_requested_.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
          break;
        }
        const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
          deadline - now);
        const auto poll = std::min(
          remaining,
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::milliseconds(50)));
        if (future.wait_for(poll) == std::future_status::ready) {
          response_ready = true;
          break;
        }
      }

      if (shutting_down_.load() || cancel_requested_.load()) {
        error = "cancelled while waiting for global localization transaction response";
        return false;
      }
      if (!response_ready) {
        break;
      }

      std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response;
      try {
        response = future.get();
      } catch (const std::exception & exception) {
        error = std::string("global localization trigger response failed: ") + exception.what();
        return false;
      }
      if (response->accepted) {
        if (attempt > 1U) {
          RCLCPP_INFO(
            get_logger(),
            "global localization accepted inside floor transaction after %zu attempts",
            attempt);
        }
        error.clear();
        return true;
      }

      error = "global localization trigger rejected: " + response->message;
      if (!target_response_settled(response)) {
        // Do not start a second trigger while a nested Isaac/bridge request
        // belonging to the first one still has an unproven outcome.
        return false;
      }
      const auto classification =
        robot_floor_manager::classify_explicit_localization_failure(error);
      if (!classification.retryable) {
        return false;
      }

      const auto now = std::chrono::steady_clock::now();
      const double remaining_sec = now < deadline ?
        std::chrono::duration<double>(deadline - now).count() : 0.0;
      RCLCPP_WARN(
        get_logger(),
        "retryable explicit localization failure kept inside floor transaction: "
        "attempt=%zu failure_code=%s remaining_sec=%.3f",
        attempt,
        classification.failure_code.c_str(),
        remaining_sec);

      const auto retry_at = std::min(
        deadline,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(100));
      while (
        !shutting_down_.load() && !cancel_requested_.load() &&
        std::chrono::steady_clock::now() < retry_at)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    if (shutting_down_.load() || cancel_requested_.load()) {
      error = "cancelled while retrying global localization transaction";
      return false;
    }
    error = "global localization transaction did not produce an accepted result within the " +
      std::to_string(std::max(service_timeout_sec_, localization_trigger_timeout_sec_)) +
      "s total retry budget after " + std::to_string(attempt) + " attempts" +
      (error.empty() ? std::string{} : "; last_error=" + error);
    return false;
  }

  bool clear_costmap(
    const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client,
    const std::string & service_name,
    std::string & error)
  {
    if (!wait_for_service(client, service_name, error)) {
      return false;
    }
    const auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    const auto future = dispatch_target_request(client, request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error = "timed out clearing costmap service: " + service_name;
      return false;
    }
    return true;
  }

  static bool same_snapshot(
    const robot_floor_manager::FloorAssetSnapshot & left,
    const robot_floor_manager::FloorAssetSnapshot & right)
  {
    const auto & a = left.fingerprints;
    const auto & b = right.fingerprints;
    return
      left.building_id == right.building_id &&
      left.floor_id == right.floor_id &&
      left.map_id == right.map_id &&
      left.asset_epoch == right.asset_epoch &&
      left.asset_digest == right.asset_digest &&
      left.paths.root == right.paths.root &&
      a.manifest_json == b.manifest_json &&
      a.nav_map_yaml == b.nav_map_yaml &&
      a.nav_map_pgm == b.nav_map_pgm &&
      a.localizer_map_png == b.localizer_map_png &&
      a.localizer_params_yaml == b.localizer_params_yaml &&
      a.keepout_mask_yaml == b.keepout_mask_yaml &&
      a.keepout_mask_pgm == b.keepout_mask_pgm &&
      a.speed_mask_yaml == b.speed_mask_yaml &&
      a.speed_mask_pgm == b.speed_mask_pgm &&
      a.binary_mask_yaml == b.binary_mask_yaml &&
      a.binary_mask_pgm == b.binary_mask_pgm &&
      a.asset_report_json == b.asset_report_json &&
      a.poses_yaml == b.poses_yaml;
  }

  bool revalidate_snapshot(
    const robot_floor_manager::FloorAssetSnapshot & snapshot,
    std::string & error) const
  {
    const auto verified = robot_floor_manager::FloorAssetSnapshotLoader{}.load(
      {
        fs::path(maps_root_),
        snapshot.building_id,
        snapshot.floor_id,
        snapshot.map_id,
        snapshot.asset_epoch,
        snapshot.asset_digest,
      });
    if (!verified.ok()) {
      error = std::string(
        robot_floor_manager::floor_asset_snapshot_error_name(
          verified.error)) + ": " + verified.message;
      return false;
    }
    if (!same_snapshot(snapshot, *verified.snapshot)) {
      error =
        "immutable target asset fingerprint changed during floor transaction";
      return false;
    }
    return true;
  }

  static robot_floor_manager::FloorTransitionEvidence target_evidence(
    const robot_floor_manager::FloorTransitionEffect & effect)
  {
    robot_floor_manager::FloorTransitionEvidence evidence;
    evidence.active_building_id = effect.building_id;
    evidence.active_floor_id = effect.floor_id;
    evidence.active_map_id = effect.map_id;
    evidence.asset_epoch = effect.expected_asset_epoch;
    evidence.asset_digest = effect.expected_asset_digest;
    return evidence;
  }

  robot_floor_manager::FloorTransitionEvidence source_evidence()
  {
    robot_floor_manager::FloorTransitionEvidence evidence;
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (!source_runtime_context_.has_value()) {
      evidence.runtime_context_invalid = true;
      return evidence;
    }
    const auto & source = *source_runtime_context_;
    evidence.active_building_id = source.building_id;
    evidence.active_floor_id = source.floor_id;
    evidence.active_map_id = source.map_id;
    evidence.asset_epoch = source.asset_epoch;
    evidence.asset_digest = source.asset_digest;
    evidence.explicit_relocalization_sequence =
      source.explicit_relocalization_sequence;
    evidence.bridge_ready = source.bridge_ready;
    evidence.amcl_ready = source.amcl_ready;
    evidence.runtime_context_invalid = false;
    evidence.runtime_context_valid = true;
    evidence.safe_for_goal_start = true;
    return evidence;
  }

  static robot_floor_manager::FloorTransitionEffectResult effect_failure(
    const std::string & code,
    const std::string & detail,
    const robot_floor_manager::FloorTransitionEvidence & evidence = {})
  {
    robot_floor_manager::FloorTransitionEffectResult result;
    result.failure_code = code;
    result.detail = detail;
    result.evidence = evidence;
    return result;
  }

  static robot_floor_manager::FloorTransitionEffectResult effect_success(
    const std::string & detail,
    const robot_floor_manager::FloorTransitionEvidence & evidence)
  {
    robot_floor_manager::FloorTransitionEffectResult result;
    result.success = true;
    result.detail = detail;
    result.evidence = evidence;
    return result;
  }

  bool nav_lifecycle_owner_unique(const std::string & expected_node)
  {
    try {
      const auto names = get_node_names();
      if (std::count(names.begin(), names.end(), expected_node) != 1) {return false;}
      const auto service = expected_node + "/get_state";
      std::size_t owners = 0U;
      for (const auto & name : names) {
        const auto separator = name.find_last_of('/');
        const auto node_name = name.substr(separator + 1U);
        const auto node_namespace = separator == 0U ? "/" : name.substr(0U, separator);
        const auto services = get_service_names_and_types_by_node(node_name, node_namespace);
        const auto found = services.find(service);
        if (found == services.end()) {continue;}
        if (name != expected_node || found->second !=
          std::vector<std::string>{"lifecycle_msgs/srv/GetState"}) {return false;}
        ++owners;
      }
      return owners == 1U;
    } catch (const std::exception &) {
      return false;
    }
  }

  void probe_nav_lifecycle()
  {
    for (std::size_t index = 0; index < nav_lifecycle_probes_.size(); ++index) {
      auto & probe = nav_lifecycle_probes_[index];
      const auto endpoint = index == 0U ? robot_floor_manager::NavLifecycleEndpoint::kBtNavigator :
        robot_floor_manager::NavLifecycleEndpoint::kControllerServer;
      const auto observed = steady_now_sec();
      if (probe.in_flight && observed - probe.requested_at > 0.75) {
        probe.client->remove_pending_request(probe.pending_id);
        probe.in_flight = false;
        ++probe.generation;
      }
      if (probe.in_flight) {continue;}
      if (!nav_lifecycle_owner_unique(probe.node_name) ||
        !probe.client->service_is_ready())
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_nav_lifecycle_state(endpoint, 0U, observed);
        continue;
      }
      probe.in_flight = true;
      probe.requested_at = observed;
      const auto generation = ++probe.generation;
      try {
        auto pending = probe.client->async_send_request(
          std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
          [this, index, endpoint, generation](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
            auto & current = nav_lifecycle_probes_[index];
            if (current.generation != generation) {return;}
            current.in_flight = false;
            std::uint8_t state = 0U;
            try {
              if (steady_now_sec() - current.requested_at <= 0.75 &&
                nav_lifecycle_owner_unique(current.node_name)) {
                state = future.get()->current_state.id;
              }
            } catch (const std::exception &) {}
            std::lock_guard<std::mutex> lock(evidence_mutex_);
            evidence_tracker_.observe_nav_lifecycle_state(endpoint, state, steady_now_sec());
            evidence_changed_.notify_all();
          });
        probe.pending_id = pending.request_id;
      } catch (const std::exception &) {
        probe.in_flight = false;
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        evidence_tracker_.observe_nav_lifecycle_state(endpoint, 0U, observed);
      }
    }
  }

  bool wait_startup_handoff(const std::string & state, double timeout, std::string & error)
  {
    robot_floor_manager::RuntimeMapContextRecord expected;
    {
      std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
      if (!startup_handoff_) {error = "startup handoff request missing"; return false;}
      expected = *startup_handoff_;
    }
    const auto deadline = steady_now_sec() + std::max(0.1, timeout);
    while (!shutting_down_.load() && !cancel_requested_.load() && steady_now_sec() < deadline) {
      const auto ack = robot_floor_manager::read_floor_startup_handoff_ack(
        startup_handoff_ack_file_, expected);
      if (ack) {
        if (ack->state == "failed" || !ack->failure.empty()) {
          error = "target startup preparation failed: " + ack->failure + ": " + ack->detail;
          return false;
        }
        if (ack->state == state) {
          if (state == "runtime_ready" &&
            (ack->explicit_relocalization_sequence != explicit_relocalization_sequence_ ||
            ack->localizer_generation != localizer_generation_))
          {
            error = "startup ready acknowledgement used a different localization generation";
            return false;
          }
          return true;
        }
      }
      std::unique_lock<std::mutex> lock(evidence_mutex_);
      evidence_changed_.wait_for(lock, 100ms);
    }
    error = "startup did not acknowledge exact target state=" + state +
      " before timeout/cancel; motion remains held";
    return false;
  }

  bool begin_startup_handoff(
    const robot_floor_manager::FloorTransitionEffect & effect, const FloorAssets & assets,
    std::string & error)
  {
    robot_floor_manager::RuntimeMapContextRecord record;
    record.startup_handoff = true;
    record.state = "requested";
    record.transaction_id = effect.transaction_id;
    record.building_id = effect.building_id;
    record.floor_id = effect.floor_id;
    record.map_id = effect.map_id;
    record.asset_epoch = effect.expected_asset_epoch;
    record.asset_digest = effect.expected_asset_digest;
    record.request_nonce = "floor_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    record.updated_at_sec = now().seconds();
    record.speed_filter_enabled = speed_filter_enabled_;
    record.asset_root = assets.root.string();
    record.nav_map_yaml = assets.nav_map_yaml.string();
    record.localizer_map_png = assets.localizer_map_png.string();
    record.localizer_params_yaml = assets.localizer_params_yaml.string();
    record.keepout_mask_yaml = assets.keepout_mask_yaml.string();
    record.speed_mask_yaml = assets.speed_mask_yaml.string();
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      if (last_localization_health_) {
        record.explicit_sequence_baseline = last_localization_health_->explicit_relocalization_sequence;
      }
    }
    {
      std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
      if (cancel_requested_.load()) {error = "cancelled before startup handoff"; return false;}
      startup_handoff_ = record;
      if (!robot_floor_manager::AtomicRuntimeMapContextWriter{}.write(
          startup_handoff_file_, record, error)) {return false;}
    }
    return wait_startup_handoff("adopted", startup_handoff_timeout_sec_, error);
  }

  bool has_startup_handoff()
  {
    std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
    return startup_handoff_.has_value();
  }

  bool finish_startup_handoff(
    const std::string & state, const std::string & detail,
    const std::string & expected_transaction = "")
  {
    std::lock_guard<std::mutex> lock(startup_handoff_mutex_);
    if (!startup_handoff_) {return true;}
    if (!expected_transaction.empty() && startup_handoff_->transaction_id != expected_transaction) {
      return true;
    }
    if (state == "committed" && cancel_requested_.load()) {return false;}
    startup_handoff_->state = state;
    startup_handoff_->message = detail;
    startup_handoff_->updated_at_sec = now().seconds();
    std::string error;
    if (!robot_floor_manager::AtomicRuntimeMapContextWriter{}.write(
        startup_handoff_file_, *startup_handoff_, error))
    {
      RCLCPP_ERROR(get_logger(), "failed to publish startup handoff terminal: %s", error.c_str());
      return false;
    }
    return true;
  }

  bool prove_stopped_idle_hold(robot_floor_manager::FloorTransitionEvidence & evidence)
  {
    return wait_for_evidence(
      [this]() {
        return evidence_tracker_.preconditions(
          steady_now_sec(), evidence_max_age_sec_, stopped_stable_duration_sec_,
          stopped_linear_threshold_mps_, stopped_angular_threshold_radps_,
          nav_idle_bootstrap_grace_sec_);
      },
      [](const auto & value) {return value.motion_hold_active && value.nav_idle && value.stopped;},
      evidence);
  }

  bool prepare_mask_lifecycle(const std::string & state_service, std::string & error)
  {
    auto state_client = create_client<lifecycle_msgs::srv::GetState>(
      state_service, rmw_qos_profile_services_default, callback_group_);
    const auto change_service = state_service.substr(0, state_service.size() - std::string("get_state").size()) + "change_state";
    auto change_client = create_client<lifecycle_msgs::srv::ChangeState>(
      change_service, rmw_qos_profile_services_default, callback_group_);
    for (int step = 0; step < 3; ++step) {
      if (!wait_for_service(state_client, state_service, error)) {return false;}
      auto state_future = state_client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (state_future.wait_for(service_timeout()) != std::future_status::ready) {
        error = "mask GetState timeout: " + state_service; return false;
      }
      const auto state = state_future.get()->current_state.id;
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {return true;}
      auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
      } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
      } else {
        error = "mask lifecycle is not stable: " + state_service; return false;
      }
      if (!wait_for_service(change_client, change_service, error)) {return false;}
      // A configure/activate may still execute after a client timeout.
      auto changed = dispatch_target_request(change_client, request);
      if (changed.wait_for(service_timeout()) != std::future_status::ready || !changed.get()->success) {
        error = "mask lifecycle change not acknowledged: " + change_service; return false;
      }
    }
    error = "mask lifecycle did not become active: " + state_service;
    return false;
  }

  robot_floor_manager::FloorTransitionEffectResult perform(
    const robot_floor_manager::FloorTransitionEffect & effect,
    const robot_floor_manager::FloorAssetSnapshot & snapshot) override
  {
    const auto canceled =
      [this]() {
        return shutting_down_.load() || cancel_requested_.load();
      };
    FloorAssets assets = assets_from_snapshot(snapshot);
    std::string error;

    switch (effect.kind) {
      case robot_floor_manager::FloorTransitionEffectKind::kVerifyPreconditions:
      {
        if (!set_motion_hold(
            effect.transaction_id,
            robot_interfaces::srv::SetMotionHold::Request::OP_ACQUIRE,
            error))
        {
          return effect_failure("MOTION_HOLD_UNPROVEN", error);
        }
        // The hold already exists even if the subsequent Nav/odom proof times
        // out or cancellation arrives. Record ownership before waiting so
        // pre-BEGIN cleanup releases exactly this transaction's key.
        motion_hold_acquired_.store(true);
        if (nav_goal_active_.load()) {
          if (!wait_for_service(nav_cancel_client_, navigate_to_pose_action_name_ + "/_action/cancel_goal", error)) {
            return effect_failure("NAV_IDLE_UNPROVEN", error);
          }
          auto canceled_goal = nav_cancel_client_->async_send_request(
            std::make_shared<action_msgs::srv::CancelGoal::Request>());
          if (canceled_goal.wait_for(service_timeout()) != std::future_status::ready) {
            return effect_failure("NAV_IDLE_UNPROVEN", "navigation cancellation is still unproven");
          }
          // Cancellation acknowledgement is not idle proof; status below must become terminal.
          (void)canceled_goal.get();
        }
        robot_floor_manager::FloorTransitionEvidence evidence;
        const bool proven = wait_for_evidence(
          [this]() {
            return evidence_tracker_.preconditions(
              steady_now_sec(), evidence_max_age_sec_,
              stopped_stable_duration_sec_,
              stopped_linear_threshold_mps_,
              stopped_angular_threshold_radps_,
              nav_idle_bootstrap_grace_sec_);
          },
          [](const auto & value) {
            return
              value.motion_hold_active &&
              value.nav_idle &&
              value.stopped;
          },
          evidence);
        if (!proven) {
          if (canceled()) {
            return effect_failure("CANCELLED", "cancelled while proving preconditions", evidence);
          }
          if (!evidence.motion_hold_active) {
            return effect_failure(
              "MOTION_HOLD_UNPROVEN",
              "exact floor-manager hold was not fresh and active",
              evidence);
          }
          if (!evidence.nav_idle) {
            return effect_failure(
              "NAV_IDLE_UNPROVEN",
              "NavigateToPose action status did not prove idle",
              evidence);
          }
          return effect_failure(
            "STOPPED_UNPROVEN",
            "fresh wheel and local odometry did not remain stopped",
            evidence);
        }
        bool inactive = false;
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          inactive = evidence_tracker_.nav_runtime_inactive(steady_now_sec(), evidence_max_age_sec_);
        }
        if (inactive && !begin_startup_handoff(effect, assets, error)) {
          return effect_failure("STARTUP_HANDOFF_UNPROVEN", error, evidence);
        }
        if (inactive && !prove_stopped_idle_hold(evidence)) {
          return effect_failure("NAV_IDLE_UNPROVEN",
            "fresh hold, idle and dual-odom stop proof lost during startup handoff", evidence);
        }
        nav_lifecycle_probe_enabled_.store(false);
        return effect_success(
          "motion hold, Nav2 idle, and dual-odom stop proven",
          evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kAcquireCorrectionPause:
      {
        if (!set_correction_pause(
            effect.transaction_id,
            robot_interfaces::srv::SetCorrectionPause::Request::OP_ACQUIRE,
            error))
        {
          return effect_failure("CORRECTION_PAUSE_UNPROVEN", error);
        }
        floor_pause_acquired_ = true;
        robot_floor_manager::FloorTransitionEvidence evidence;
        if (!wait_for_evidence(
            [this]() {
              return evidence_tracker_.floor_pause(
                steady_now_sec(), evidence_max_age_sec_);
            },
            [](const auto & value) {
              return
                value.floor_pause_owned &&
                value.correction_pause_effective;
            },
            evidence))
        {
          return effect_failure(
            canceled() ? "CANCELLED" : "CORRECTION_PAUSE_UNPROVEN",
            canceled() ? "cancelled while proving floor correction pause" :
            "exact floor-manager correction pause was not proven",
            evidence);
        }
        return effect_success(
          "floor-manager correction pause acquired", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kInvalidateRuntimeContext:
      {
        // A verified source is optional rollback evidence, never a prerequisite
        // for initializing an exact target from an unlocalized startup.
        auto response = call_bridge_transition(
          effect,
          robot_interfaces::srv::BeginFloorTransition::Request::OP_BEGIN,
          error);
        if (!response || response->runtime_context_valid) {
          return effect_failure(
            "BRIDGE_BEGIN_UNPROVEN",
            error.empty() ?
            "bridge BEGIN did not invalidate runtime context" : error);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          bridge_begin_established_ = true;
          runtime_context_confirmed_ = false;
          published_asset_epoch_ = effect.expected_asset_epoch;
          published_asset_digest_ = effect.expected_asset_digest;
          nav_map_ready_ = false;
          filters_ready_ = false;
          localizer_ready_ = false;
          bridge_ready_ = false;
          amcl_ready_ = false;
          costmaps_ready_ = false;
        }
        if (!write_runtime_context(
            effect, "floor_switch_pending", false,
            "source runtime context invalidated by bridge BEGIN", error))
        {
          auto evidence = target_evidence(effect);
          evidence.runtime_context_invalid = true;
          return effect_failure(
            "RUNTIME_CONTEXT_WRITE_FAILED", error, evidence);
        }
        auto evidence = target_evidence(effect);
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          const auto pause = evidence_tracker_.floor_pause(
            steady_now_sec(), evidence_max_age_sec_);
          evidence.floor_pause_owned = pause.floor_pause_owned;
          evidence.correction_pause_effective =
            pause.correction_pause_effective;
        }
        evidence.runtime_context_invalid = true;
        if (!evidence.floor_pause_owned ||
          !evidence.correction_pause_effective)
        {
          return effect_failure(
            "CORRECTION_PAUSE_UNPROVEN",
            "floor pause became stale across bridge BEGIN", evidence);
        }
        return effect_success(
          "bridge BEGIN invalidated source context", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kReportBeginReady:
      {
        auto evidence = target_evidence(effect);
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          const auto pause = evidence_tracker_.floor_pause(
            steady_now_sec(), evidence_max_age_sec_);
          evidence.floor_pause_owned = pause.floor_pause_owned;
          evidence.correction_pause_effective =
            pause.correction_pause_effective;
        }
        evidence.runtime_context_invalid = bridge_begin_established_;
        if (
          !evidence.runtime_context_invalid ||
          !evidence.floor_pause_owned ||
          !evidence.correction_pause_effective)
        {
          return effect_failure(
            "CALLER_HANDOFF_UNPROVEN",
            "caller handoff was not published because BEGIN or the exact "
            "floor pause was no longer proven",
            evidence);
        }
        return effect_success(
          "caller pause handoff barrier published", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kVerifyPauseHandoff:
      {
        robot_floor_manager::FloorTransitionEvidence evidence;
        if (!wait_for_evidence(
            [this]() {
              return evidence_tracker_.pause_handoff(
                steady_now_sec(), evidence_max_age_sec_);
            },
            [](const auto & value) {
              return
                value.caller_pause_released &&
                value.floor_pause_owned &&
                value.correction_pause_effective;
            },
            evidence))
        {
          return effect_failure(
            canceled() ? "CANCELLED" : "PAUSE_HANDOFF_UNPROVEN",
            canceled() ? "cancelled during caller pause handoff" :
            "caller pause was not released while exact floor pause remained",
            evidence);
        }
        evidence.runtime_context_invalid = bridge_begin_established_;
        return effect_success(
          "caller released only its correction pause", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kLoadNavMap:
      {
        if (!revalidate_snapshot(snapshot, error)) {
          return effect_failure("ASSET_IDENTITY_CHANGED", error);
        }
        MapLoadFailure failure;
        if (!load_nav_map(assets, error, failure)) {
          const auto code = map_load_failure_code("NAV_MAP_LOAD", failure);
          return effect_failure(code, code + ": " + error);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          nav_map_ready_ = true;
        }
        return effect_success(
          "target Nav2 map loaded", target_evidence(effect));
      }

      case robot_floor_manager::FloorTransitionEffectKind::kLoadFilters:
      {
        if (!revalidate_snapshot(snapshot, error)) {
          return effect_failure("ASSET_IDENTITY_CHANGED", error);
        }
        MapLoadFailure failure;
        if (!load_filter_masks(assets, error, failure)) {
          const auto code = map_load_failure_code("FILTER_LOAD", failure);
          return effect_failure(code, code + ": " + error);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          filters_ready_ = true;
        }
        return effect_success(
          "target keepout and speed filters loaded",
          target_evidence(effect));
      }

      case robot_floor_manager::FloorTransitionEffectKind::kReloadLocalizer:
      {
        if (!revalidate_snapshot(snapshot, error)) {
          return effect_failure("ASSET_IDENTITY_CHANGED", error);
        }
        std::uint64_t baseline_generation = 0U;
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          baseline_generation =
            evidence_tracker_.begin_localizer_generation();
        }
        std::uint64_t accepted_generation = 0U;
        if (!apply_localizer_assets(
            effect.transaction_id, assets, baseline_generation,
            accepted_generation, error))
        {
          return effect_failure("LOCALIZER_RELOAD_UNPROVEN", error);
        }
        robot_floor_manager::FloorTransitionEvidence evidence;
        if (!wait_for_evidence(
            [this, accepted_generation]() {
              return evidence_tracker_.target_localizer(
                accepted_generation, steady_now_sec(),
                evidence_max_age_sec_);
            },
            [](const auto & value) {
              return value.asset_epoch != 0U;
            },
            evidence))
        {
          return effect_failure(
            canceled() ? "CANCELLED" : "LOCALIZER_RELOAD_UNPROVEN",
            canceled() ? "cancelled while proving localizer reload" :
            "typed localizer state did not echo the exact new reload",
            evidence);
        }
        localizer_generation_ = accepted_generation;
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          localizer_ready_ = true;
        }
        return effect_success(
          "target localizer reload proven by response and typed state",
          evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kReleaseFloorPause:
      {
        if (!set_correction_pause(
            effect.transaction_id,
            robot_interfaces::srv::SetCorrectionPause::Request::OP_RELEASE,
            error))
        {
          return effect_failure("CORRECTION_PAUSE_RELEASE_FAILED", error);
        }
        const bool released = wait_for_condition(
          [this]() {
            return evidence_tracker_.corrections_released(
              steady_now_sec(), evidence_max_age_sec_);
          });
        auto evidence = target_evidence(effect);
        if (!released) {
          return effect_failure(
            canceled() ? "CANCELLED" : "CORRECTION_PAUSE_REMAINS",
            canceled() ? "cancelled while releasing correction pause" :
            "another correction pause remains after floor pause release",
            evidence);
        }
        floor_pause_acquired_ = false;
        return effect_success(
          "all correction pauses released for target localization",
          evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kTriggerExplicitLocalization:
      {
        const bool trigger_call_succeeded = trigger_localization(assets, error);
        robot_floor_manager::FloorTransitionEvidence evidence;
        const bool evidence_proven = wait_for_evidence(
            [this]() {
              return evidence_tracker_.target_localization(
                steady_now_sec(), evidence_max_age_sec_,
                localizer_generation_);
            },
            [](const auto & value) {
              return
                value.asset_epoch != 0U &&
                value.explicit_relocalization_sequence != 0U;
            },
            evidence);
        const auto reconciliation =
          robot_floor_manager::reconcile_explicit_localization(
          trigger_call_succeeded, evidence_proven, canceled(), error);
        if (!reconciliation.success) {
          return effect_failure(
            reconciliation.code,
            reconciliation.detail,
            evidence);
        }
        explicit_relocalization_sequence_ =
          evidence.explicit_relocalization_sequence;
        return effect_success(
          reconciliation.detail, evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kVerifyBridgeReady:
      {
        robot_floor_manager::FloorTransitionEvidence evidence;
        if (has_startup_handoff()) {
          // Pending target TF is usable for initializing Nav2/AMCL, but is not
          // permission to move or to commit. Waiting for AMCL first deadlocks
          // against the startup owner which must create it after this TF.
          if (!wait_for_evidence(
              [this]() {return evidence_tracker_.target_localization(
                  steady_now_sec(), evidence_max_age_sec_, localizer_generation_);},
              [](const auto & value) {return value.bridge_ready &&
                  value.runtime_context_invalid && value.explicit_relocalization_sequence != 0U;},
              evidence) || !wait_startup_handoff("runtime_ready", startup_ready_timeout_sec_, error))
          {
            return effect_failure("STARTUP_TARGET_READY_UNPROVEN",
              error.empty() ? "pending target transform was not proven" : error, evidence);
          }
        }
        if (!wait_for_evidence(
            [this]() {
              return evidence_tracker_.target_localization(
                steady_now_sec(), evidence_max_age_sec_,
                localizer_generation_);
            },
            [](const auto & value) {
              return
                value.bridge_ready &&
                value.amcl_ready &&
                value.runtime_context_invalid &&
                value.explicit_relocalization_sequence != 0U;
            },
            evidence))
        {
          return effect_failure(
            canceled() ? "CANCELLED" : "BRIDGE_READY_UNPROVEN",
            canceled() ? "cancelled while proving bridge readiness" :
            "pending target map->odom was not ready, unique, and published",
            evidence);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          bridge_ready_ = true;
          amcl_ready_ = true;
        }
        return effect_success(
          "pending target bridge readiness proven", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kClearCostmaps:
      {
        if (!clear_costmaps_after_switch_) {
          return effect_failure(
            "COSTMAP_CLEAR_DISABLED",
            "live floor switch requires both typed costmap clears");
        }
        if (!clear_costmap(
            global_clear_client_, global_costmap_clear_service_, error) ||
          !clear_costmap(
            local_clear_client_, local_costmap_clear_service_, error))
        {
          return effect_failure("COSTMAP_CLEAR_FAILED", error);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          evidence_tracker_.mark_costmaps_cleared(steady_now_sec());
        }
        return effect_success(
          "global and local costmap clear services acknowledged",
          target_evidence(effect));
      }

      case robot_floor_manager::FloorTransitionEffectKind::kVerifyFreshCostmaps:
      {
        robot_floor_manager::FloorTransitionEvidence evidence;
        if (!wait_for_evidence(
            [this]() {
              return evidence_tracker_.fresh_costmaps(
                steady_now_sec(), evidence_max_age_sec_,
                localizer_generation_);
            },
            [](const auto & value) {
              return
                value.global_costmap_fresh &&
                value.local_costmap_fresh &&
                value.asset_epoch != 0U;
            },
            evidence))
        {
          return effect_failure(
            canceled() ? "CANCELLED" : "COSTMAP_FRESHNESS_UNPROVEN",
            canceled() ? "cancelled while proving fresh costmaps" :
            "both post-clear target costmaps were not fresh",
            evidence);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          costmaps_ready_ = true;
        }
        return effect_success(
          "fresh global and local target costmaps proven", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kCommitRuntimeContext:
      {
        auto response = call_bridge_transition(
          effect,
          robot_interfaces::srv::BeginFloorTransition::Request::OP_COMMIT,
          error);
        if (
          !response ||
          !response->runtime_context_valid ||
          !response->safe_for_goal_start ||
          response->explicit_relocalization_sequence == 0U ||
          response->explicit_relocalization_sequence !=
          explicit_relocalization_sequence_)
        {
          return effect_failure(
            "BRIDGE_COMMIT_UNPROVEN",
            error.empty() ?
            "bridge COMMIT did not prove exact safe target context" : error);
        }
        if (!write_runtime_context(
            effect, "ready", true,
            "target floor runtime context committed", error))
        {
          return effect_failure("RUNTIME_CONTEXT_WRITE_FAILED", error);
        }
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          runtime_context_confirmed_ = true;
          published_asset_epoch_ = effect.expected_asset_epoch;
          published_asset_digest_ = effect.expected_asset_digest;
        }
        auto evidence = target_evidence(effect);
        evidence.runtime_context_valid = true;
        evidence.safe_for_goal_start = true;
        evidence.bridge_ready = true;
        evidence.amcl_ready = true;
        evidence.explicit_relocalization_sequence =
          explicit_relocalization_sequence_;
        return effect_success(
          "target runtime context durably committed", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kComplete:
      {
        if (has_startup_handoff()) {
          robot_floor_manager::FloorTransitionEvidence stopped_evidence;
          if (!prove_stopped_idle_hold(stopped_evidence)) {
            return effect_failure("NAV_IDLE_UNPROVEN",
              "target runtime started but fresh idle/stop/hold proof is missing", stopped_evidence);
          }
          // Terminal handoff publication is part of success, before allowing motion.
          if (!finish_startup_handoff("committed", "target runtime context committed", effect.transaction_id)) {
            return effect_failure("RUNTIME_CONTEXT_WRITE_FAILED",
              "startup handoff commit could not be durably published; hold retained");
          }
        }
        if (!set_motion_hold(
            effect.transaction_id,
            robot_interfaces::srv::SetMotionHold::Request::OP_RELEASE,
            error))
        {
          return effect_failure("MOTION_HOLD_RELEASE_FAILED", error);
        }
        motion_hold_acquired_.store(false);
        {
          std::lock_guard<std::mutex> lock(floor_state_mutex_);
          selected_building_id_ = effect.building_id;
          selected_floor_id_ = effect.floor_id;
          selected_map_id_ = effect.map_id;
        }
        auto evidence = target_evidence(effect);
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          evidence.runtime_context_valid = runtime_context_confirmed_;
          evidence.safe_for_goal_start = runtime_context_confirmed_;
          evidence.bridge_ready = bridge_ready_;
          evidence.amcl_ready = amcl_ready_;
          evidence.global_costmap_fresh = costmaps_ready_;
          evidence.local_costmap_fresh = costmaps_ready_;
        }
        evidence.explicit_relocalization_sequence =
          explicit_relocalization_sequence_;
        return effect_success(
          "floor-manager motion hold released after commit", evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kHoldAndLock:
      {
        deferred_failure_effect_ = effect;
        deferred_failure_snapshot_ = snapshot;
        deferred_failure_cleanup_.store(true);
        finish_startup_handoff("failed", effect.detail, effect.transaction_id);
        std::string cleanup_detail;
        const auto append_detail =
          [&cleanup_detail](const std::string & detail) {
            if (!detail.empty()) {
              cleanup_detail +=
                (cleanup_detail.empty() ? "" : "; ") + detail;
            }
          };
        const auto release_pre_mutation_resources =
          [this, &effect, &append_detail]() {
            std::string pause_error;
            const bool pause_released = set_correction_pause(
              effect.transaction_id,
              robot_interfaces::srv::SetCorrectionPause::Request::OP_RELEASE,
              pause_error);
            append_detail(pause_error);

            // Do not open a motion window while pause release is unproven.
            if (!pause_released) {
              return false;
            }

            std::string hold_error;
            const bool hold_released = set_motion_hold(
              effect.transaction_id,
              robot_interfaces::srv::SetMotionHold::Request::OP_RELEASE,
              hold_error);
            append_detail(hold_error);
            return hold_released;
          };
        const auto retain_safety_resources =
          [this, &effect, &append_detail]() {
            std::string hold_error;
            const bool hold_retained = set_motion_hold(
              effect.transaction_id,
              robot_interfaces::srv::SetMotionHold::Request::OP_ACQUIRE,
              hold_error);
            append_detail(hold_error);

            std::string pause_error;
            const bool pause_retained = set_correction_pause(
              effect.transaction_id,
              robot_interfaces::srv::SetCorrectionPause::Request::OP_ACQUIRE,
              pause_error);
            append_detail(pause_error);
            return std::pair<bool, bool>{hold_retained, pause_retained};
          };

        const bool begin_established = bridge_begin_established_;
        const bool begin_outcome_unknown = bridge_begin_outcome_unknown_;
        const auto cleanup_action = robot_floor_manager::select_floor_transition_cleanup(
          begin_established, begin_outcome_unknown, target_effect_dispatched_);
        if (cleanup_action ==
          robot_floor_manager::FloorTransitionCleanupAction::kReleaseUnchangedSource)
        {
          // Always issue higher-sequence exact RELEASE commands. An earlier
          // ACQUIRE may have been accepted even when its response timed out,
          // so local "acquired" booleans are not cleanup evidence.
          if (startup_effects_settled() && release_pre_mutation_resources()) {
            deferred_failure_cleanup_.store(false);
            auto evidence = target_evidence(effect);
            evidence.runtime_context_invalid = false;
            evidence.failure_resources_released = true;
            return effect_success(
              "pre-mutation exact leases released and proven absent by fresh state",
              evidence);
          }

          const auto retained = retain_safety_resources();
          auto evidence = target_evidence(effect);
          evidence.motion_hold_active = retained.first;
          evidence.runtime_context_invalid = true;
          if (retained.first) {
            std::string context_error;
            if (!write_runtime_context(
                effect, "floor_switch_cleanup_pending", false,
                "pre-mutation cleanup was not proven; safety hold retained",
                context_error))
            {
              append_detail(context_error);
            }
            return effect_success(
              "pre-mutation cleanup was not proven; higher-sequence safety "
              "hold retained and explicit recovery is required" +
              (retained.second ? std::string{} :
              std::string{"; correction pause retention is unproven"}),
              evidence);
          }
          return effect_success(
            cleanup_detail.empty() ?
            "exact lease release and safety-hold retention were both unproven" :
            cleanup_detail,
            evidence);
        }

        std::string abort_error;
        bool source_restored = false;
        bool bridge_aborted = false;
        if (cleanup_action ==
          robot_floor_manager::FloorTransitionCleanupAction::kRestoreSource)
        {
          // Restoring the bridge is itself a source-ready claim. Do not make
          // it while the startup owner can still tear down that source.
          if (startup_effects_settled()) {
            source_restored = abort_bridge_and_prove_source(effect, abort_error);
          }
        } else {
          // ABORT ends the bridge transaction, but does not prove localization
          // or that outstanding target RPCs have finished.
          const auto response = call_bridge_transition(
            effect,
            robot_interfaces::srv::BeginFloorTransition::Request::OP_ABORT,
            abort_error);
          if (response && response->runtime_context_valid) {
            abort_error = "ordinary ABORT unexpectedly reported a valid runtime context";
          }
          bridge_aborted = response && !response->runtime_context_valid;
          append_detail("target effect dispatched or outcome unknown; source restore skipped");
        }
        append_detail(abort_error);

        // Once exact pre-mutation ABORT and a later source-identity LocalizationHealth
        // sample prove that the source runtime is valid again, persist that
        // source identity and release this transaction's resources. This is
        // a recoverable failed transaction, not a permanent vehicle lock.
        if (source_restored && startup_effects_settled()) {
          std::string source_context_error;
          const bool source_context_written = write_source_runtime_context(
            effect,
            "floor switch failed before target dispatch; exact source runtime restored",
            source_context_error);
          append_detail(source_context_error);
          if (source_context_written && release_pre_mutation_resources()) {
            deferred_failure_cleanup_.store(false);
            bridge_begin_established_ = false;
            bridge_begin_submitted_ = false;
            bridge_begin_outcome_unknown_ = false;
            auto evidence = source_evidence();
            {
              std::lock_guard<std::mutex> lock(floor_state_mutex_);
              selected_building_id_ = evidence.active_building_id;
              selected_floor_id_ = evidence.active_floor_id;
              selected_map_id_ = evidence.active_map_id;
            }
            {
              std::lock_guard<std::mutex> lock(evidence_mutex_);
              runtime_context_confirmed_ = true;
              published_asset_epoch_ = evidence.asset_epoch;
              published_asset_digest_ = evidence.asset_digest;
              localizer_ready_ = source_runtime_context_->localizer_ready;
              bridge_ready_ = evidence.bridge_ready;
              amcl_ready_ = evidence.amcl_ready;
            }
            evidence.motion_hold_active = false;
            evidence.floor_pause_owned = false;
            evidence.correction_pause_effective = false;
            evidence.runtime_context_invalid = false;
            evidence.failure_resources_released = true;
            return effect_success(
              "floor switch failed before target dispatch; source restored and "
              "transaction resources released; retry is allowed",
              evidence);
          }
        }

        // A completed unsuccessful mutation is not an in-flight mutation.
        // End only this transaction's occupancy while leaving localization
        // invalid. Normal navigation must still obtain a valid map/pose itself.
        if (target_requests_settled() && startup_effects_settled()) {
          if (!bridge_aborted) {
            std::string terminal_error;
            const auto response = call_bridge_transition(effect,
              robot_interfaces::srv::BeginFloorTransition::Request::OP_ABORT,
              terminal_error);
            bridge_aborted = response && !response->runtime_context_valid;
            append_detail(terminal_error);
          }
          std::string context_error;
          if (bridge_aborted && write_runtime_context(effect, "floor_switch_failed", false,
              "floor switch failed; transaction ended, localization must be re-established",
              context_error) && release_pre_mutation_resources())
          {
            {
              std::lock_guard<std::mutex> lock(evidence_mutex_);
              runtime_context_confirmed_ = false;
              bridge_ready_ = false;
              amcl_ready_ = false;
            }
            bridge_begin_established_ = false;
            bridge_begin_submitted_ = false;
            bridge_begin_outcome_unknown_ = false;
            deferred_failure_cleanup_.store(false);
            auto evidence = target_evidence(effect);
            evidence.motion_hold_active = false;
            evidence.floor_pause_owned = false;
            evidence.correction_pause_effective = false;
            evidence.runtime_context_valid = false;
            evidence.runtime_context_invalid = true;
            evidence.failure_resources_released = true;
            return effect_success(
              "floor switch failed; all target requests settled, bridge transaction ended "
              "and owned resources released; localization remains invalid", evidence);
          }
          append_detail(context_error);
        } else {
          append_detail("target request is still pending; exact cleanup will resume after its response");
        }

        // Only a real unresolved request or unacknowledged cleanup retains the
        // transaction's safety protection; the already-failed action returns.
        const auto retained = retain_safety_resources();
        {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          runtime_context_confirmed_ = false;
        }
        std::string context_error;
        if (!write_runtime_context(
            effect, "floor_switch_cleanup_pending", false,
            source_restored ?
            "bridge source context restored but floor recovery lock retained" :
            (target_effect_dispatched_ ?
            "target request or exact cleanup still pending" :
            "source recovery unproven; floor recovery lock retained"),
            context_error))
        {
          append_detail(context_error);
        }
        auto evidence = target_evidence(effect);
        evidence.motion_hold_active = retained.first;
        evidence.runtime_context_invalid = true;
        return effect_success(
          "failed floor switch returned; its pending requests or owned-resource cleanup "
          "will be reconciled without restarting the transaction" +
          (retained.second ? std::string{} :
          std::string{"; correction pause retention is unproven"}),
          evidence);
      }

      case robot_floor_manager::FloorTransitionEffectKind::kNone:
        break;
    }
    return effect_failure(
      "INTERNAL_ERROR", "unsupported floor-transition effect");
  }

  bool emergency_lock_after_exception(
    const robot_floor_manager::FloorTransitionRequest & request,
    const std::string & reason) noexcept
  {
    try {
      // A new preflight can throw before this worker owns any runtime effect.
      // Never clean a previous transaction using the new request's identity.
      if (tracked_floor_transaction_ != request.transaction_id) {return true;}
      robot_floor_manager::FloorTransitionEffect effect;
      effect.kind =
        robot_floor_manager::FloorTransitionEffectKind::kHoldAndLock;
      effect.transaction_id = request.transaction_id;
      effect.building_id = request.building_id;
      effect.floor_id = request.floor_id;
      effect.map_id = request.map_id;
      effect.expected_asset_epoch = request.expected_asset_epoch;
      effect.expected_asset_digest = request.expected_asset_digest;
      effect.cleanup_id = request.transaction_id + "-exception-cleanup";
      effect.detail = reason;
      robot_floor_manager::FloorAssetSnapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        if (active_snapshot_.has_value()) {
          snapshot = *active_snapshot_;
        }
      }
      const auto cleanup = perform(effect, snapshot);
      if (!cleanup.success) {
        RCLCPP_ERROR(
          get_logger(),
          "floor-switch exception cleanup remains unproven: %s",
          cleanup.detail.c_str());
      }
      return cleanup.success && cleanup.evidence.failure_resources_released;
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(),
        "floor-switch exception cleanup threw; manual recovery is required");
    }
    return false;
  }

  void on_switch_floor(
    const std::shared_ptr<robot_interfaces::srv::SwitchFloor::Request> request,
    std::shared_ptr<robot_interfaces::srv::SwitchFloor::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      if (switching_ || floor_switch_action_active_ || deferred_failure_cleanup_.load()) {
        response->success = false;
        response->code = "TRANSACTION_CONFLICT";
        response->message = "floor switch or selection already in progress";
        return;
      }
      switching_ = true;
    }

    FloorAssets assets;
    std::string error_code;
    std::string error;
    const auto finish =
      [this, &response](
        const bool success,
        const std::string & code,
        const std::string & message,
        const FloorAssets * proof)
      {
        response->success = success;
        response->code = code;
        response->message = message;
        if (proof != nullptr) {
          response->selected_building_id = proof->building_id;
          response->selected_floor_id = proof->floor_id;
          response->selected_map_id = proof->map_id;
          response->asset_epoch = proof->asset_epoch;
          response->asset_digest = proof->asset_digest;
          response->nav_map_yaml = proof->nav_map_yaml.string();
          response->localizer_map_png = proof->localizer_map_png.string();
          response->localizer_params_yaml = proof->localizer_params_yaml.string();
        }
        {
          std::lock_guard<std::mutex> lock(floor_state_mutex_);
          switching_ = false;
        }
        // This service proves an immutable source bundle only. The API owns
        // current/ activation, so service success must not publish or retain
        // selected/active runtime state that could survive a failed API commit.
        publish_status(success ? "idle" : ("failed:" + message));
      };

    if (request->resume_navigation) {
      finish(
        false,
        "LEGACY_RESUME_NAVIGATION_DISABLED",
        "LEGACY_RESUME_NAVIGATION_DISABLED: use /floor_manager/floor_switch; "
        "the legacy service cannot prove an atomic localizer reload",
        nullptr);
      return;
    }

    if (!validate_exact_floor_assets(
        request->building_id,
        request->floor_id,
        request->map_id,
        request->expected_asset_epoch,
        request->expected_asset_digest,
        assets,
        error_code,
        error))
    {
      finish(false, error_code, error_code + ": " + error, nullptr);
      return;
    }

    const std::string map_key =
      assets.building_id + "/" + assets.floor_id + "/" + assets.map_id;
    finish(
      true,
      "OK",
      "floor source preflight verified; API activation is still required: " +
      map_key,
      &assets);
  }

  std::string maps_root_;
  std::string default_building_id_;
  std::string status_topic_;
  std::string map_server_load_service_;
  std::string keepout_mask_load_service_;
  std::string speed_mask_load_service_;
  std::string keepout_mask_state_service_;
  std::string speed_mask_state_service_;
  std::string localizer_apply_service_;
  std::string localization_trigger_service_;
  std::string global_costmap_clear_service_;
  std::string local_costmap_clear_service_;
  std::string floor_switch_action_name_;
  std::string transition_status_topic_;
  std::string motion_hold_service_;
  std::string motion_hold_sequence_state_file_;
  std::string motion_interlock_state_topic_;
  std::string navigate_to_pose_status_topic_;
  std::string navigate_to_pose_action_name_;
  std::string wheel_odom_topic_;
  std::string local_odom_topic_;
  std::string correction_pause_service_;
  std::string correction_pause_state_topic_;
  std::string begin_floor_transition_service_;
  std::string localization_health_topic_;
  std::string localizer_asset_state_topic_;
  std::string global_costmap_topic_;
  std::string local_costmap_topic_;
  std::string runtime_map_context_file_;
  std::string current_floor_;
  std::string selected_building_id_;
  std::string selected_floor_id_;
  std::string selected_map_id_;
  std::string active_floor_switch_transaction_;
  double service_timeout_sec_{10.0};
  double localizer_apply_timeout_sec_{30.0};
  double localization_trigger_timeout_sec_{75.0};
  double filter_mask_state_timeout_sec_{0.5};
  double evidence_timeout_sec_{10.0};
  double nav_idle_bootstrap_grace_sec_{2.0};
  double evidence_max_age_sec_{0.75};
  double stopped_stable_duration_sec_{0.30};
  double stopped_linear_threshold_mps_{0.02};
  double stopped_angular_threshold_radps_{0.02};
  bool call_map_server_load_{true};
  bool call_filter_mask_load_{true};
  bool speed_filter_enabled_{false};
  bool call_localizer_apply_{true};
  bool call_localization_trigger_{true};
  bool clear_costmaps_after_switch_{true};
  bool require_filter_assets_{true};
  bool switching_{false};
  bool live_floor_switch_enabled_{false};
  bool floor_switch_action_active_{false};
  bool bridge_begin_established_{false};
  bool bridge_begin_submitted_{false};
  bool bridge_begin_outcome_unknown_{false};
  bool target_effect_dispatched_{false};
  bool startup_target_trigger_dispatched_{false};
  struct PendingTargetRequest
  {
    std::shared_ptr<std::atomic_bool> settled;
    std::shared_ptr<void> client;
    std::function<bool()> response_settled;
  };
  std::vector<PendingTargetRequest> target_requests_;
  std::string tracked_floor_transaction_;
  std::atomic_bool deferred_failure_cleanup_{false};
  std::uint16_t deferred_failure_code_{99U};
  robot_floor_manager::FloorTransitionEffect deferred_failure_effect_;
  robot_floor_manager::FloorAssetSnapshot deferred_failure_snapshot_;
  std::mutex startup_handoff_mutex_;
  std::optional<robot_floor_manager::RuntimeMapContextRecord> startup_handoff_;
  std::string startup_handoff_file_;
  std::string startup_handoff_ack_file_;
  double startup_handoff_timeout_sec_{90.0};
  double startup_ready_timeout_sec_{120.0};
  std::atomic_bool nav_lifecycle_probe_enabled_{false};
  std::atomic_bool nav_goal_active_{false};
  struct NavLifecycleProbe
  {
    std::string node_name;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
    bool in_flight{false};
    double requested_at{0.0};
    std::int64_t pending_id{0};
    std::uint64_t generation{0U};
  };
  std::array<NavLifecycleProbe, 2> nav_lifecycle_probes_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr nav_cancel_client_;
  std::atomic_bool motion_hold_command_submitted_{false};
  std::atomic_bool motion_hold_outcome_unknown_{false};
  std::atomic_bool motion_hold_acquired_{false};
  std::unique_ptr<robot_safety::PersistentSequenceAllocator>
    motion_hold_sequence_allocator_;
  std::atomic_bool correction_pause_command_submitted_{false};
  std::atomic_bool correction_pause_outcome_unknown_{false};
  bool floor_pause_acquired_{false};
  bool runtime_context_confirmed_{false};
  bool nav_map_ready_{false};
  bool filters_ready_{false};
  bool localizer_ready_{false};
  bool bridge_ready_{false};
  bool amcl_ready_{false};
  bool costmaps_ready_{false};
  std::uint64_t published_asset_epoch_{0U};
  std::string published_asset_digest_;
  std::uint64_t localizer_generation_{0U};
  std::uint64_t explicit_relocalization_sequence_{0U};
  std::uint64_t global_costmap_receive_sequence_{0U};
  std::uint64_t local_costmap_receive_sequence_{0U};
  std::uint64_t transition_status_generation_{0U};
  std::mutex floor_state_mutex_;
  std::mutex evidence_mutex_;
  std::condition_variable evidence_changed_;
  robot_floor_manager::FloorTransitionEvidenceTracker evidence_tracker_;
  std::uint64_t motion_interlock_state_generation_{0U};
  bool last_motion_interlock_hold_active_{false};
  std::vector<std::string> last_motion_interlock_hold_keys_;
  std::uint64_t correction_pause_state_generation_{0U};
  bool last_correction_pause_active_{false};
  std::vector<std::string> last_correction_pause_keys_;
  std::uint64_t localization_health_generation_{0U};
  double last_localization_health_received_steady_sec_{-1.0};
  std::optional<robot_floor_manager::LocalizationHealthEvidence>
    last_localization_health_;
  std::optional<robot_floor_manager::LocalizationHealthEvidence>
    source_runtime_context_;
  std::optional<robot_floor_manager::FloorAssetSnapshot> active_snapshot_;
  std::atomic_bool cancel_requested_{false};
  std::atomic_bool shutting_down_{false};
  std::mutex worker_mutex_;
  std::thread floor_switch_worker_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::FloorSwitchStatus>::SharedPtr
    transition_status_pub_;
  rclcpp::Service<robot_interfaces::srv::SwitchFloor>::SharedPtr switch_service_;
  rclcpp_action::Server<FloorSwitchAction>::SharedPtr floor_switch_action_server_;
  rclcpp::TimerBase::SharedPtr nav_graph_probe_timer_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr keepout_mask_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr speed_mask_load_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr keepout_mask_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr speed_mask_state_client_;
  rclcpp::Client<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr localizer_apply_client_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr localization_trigger_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<robot_interfaces::srv::SetMotionHold>::SharedPtr
    motion_hold_client_;
  rclcpp::Client<robot_interfaces::srv::SetCorrectionPause>::SharedPtr
    correction_pause_client_;
  rclcpp::Client<robot_interfaces::srv::BeginFloorTransition>::SharedPtr
    begin_floor_transition_client_;
  rclcpp::Subscription<robot_interfaces::msg::MotionInterlockState>::SharedPtr
    motion_interlock_state_sub_;
  rclcpp::Subscription<robot_interfaces::msg::CorrectionPauseState>::SharedPtr
    correction_pause_state_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizationHealth>::SharedPtr
    localization_health_sub_;
  rclcpp::Subscription<robot_interfaces::msg::LocalizerAssetState>::SharedPtr
    localizer_asset_state_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
    nav_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    local_costmap_sub_;
};

#ifndef ROBOT_FLOOR_MANAGER_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<FloorManagerNode>();
    // Floor transitions already run on floor_switch_worker_. Keeping ROS
    // wait-set dispatch single-threaded prevents one executor worker from
    // consuming an action readiness event observed by another worker.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok()) {
      try {
        executor.spin();
        break;
      } catch (const std::runtime_error & exception) {
        if (
          robot_floor_manager::classify_executor_runtime_error(exception) !=
          robot_floor_manager::ExecutorRuntimeErrorDisposition::kRetry)
        {
          throw;
        }
        RCLCPP_ERROR(
          node->get_logger(),
          "continuing after transient action client executor exception: %s",
          exception.what());
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (const std::exception & exception) {
    std::cerr << "robot_floor_manager fatal exception: " << exception.what() << std::endl;
    exit_code = 1;
  } catch (...) {
    std::cerr << "robot_floor_manager unknown fatal exception" << std::endl;
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
#endif
