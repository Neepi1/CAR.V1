#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_floor_manager/floor_switch_preflight.hpp"
#include "robot_interfaces/action/floor_switch.hpp"
#include "robot_interfaces/msg/floor_switch_status.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

struct FloorAssets
{
  std::string building_id;
  std::string floor_id;
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

}  // namespace

class FloorManagerNode : public rclcpp::Node
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
    call_map_server_load_ = declare_parameter<bool>("call_map_server_load", true);
    call_filter_mask_load_ = declare_parameter<bool>("call_filter_mask_load", true);
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

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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

private:
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
    status.active_context_valid = false;
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
    const std::shared_ptr<FloorSwitchGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_floor_switch_accepted(
    const std::shared_ptr<FloorSwitchGoalHandle> goal_handle)
  {
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
      transaction_conflict = floor_switch_action_active_ || switching_;
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

  std::chrono::nanoseconds filter_mask_state_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(filter_mask_state_timeout_sec_));
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
    const bool keepout_active = filter_mask_server_is_active(
      keepout_mask_state_client_, keepout_mask_state_service_, "keepout_filter_mask_server");
    const bool speed_active = filter_mask_server_is_active(
      speed_mask_state_client_, speed_mask_state_service_, "speed_filter_mask_server");
    return keepout_active && speed_active;
  }

  bool load_map_with_client(
    const rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr & client,
    const std::string & service_name,
    const fs::path & map_yaml,
    const std::string & label,
    std::string & error)
  {
    if (!wait_for_service(client, service_name, error)) {
      return false;
    }
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = map_yaml.string();
    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error = "timed out loading " + label + ": " + map_yaml.string();
      return false;
    }
    const auto response = future.get();
    if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      error = service_name + " rejected " + label + " with result code " + std::to_string(response->result);
      return false;
    }
    return true;
  }

  bool load_nav_map(const FloorAssets & assets, std::string & error)
  {
    if (!call_map_server_load_) {
      return true;
    }
    return load_map_with_client(
      map_load_client_, map_server_load_service_, assets.nav_map_yaml, "Nav2 map", error);
  }

  bool load_filter_masks(const FloorAssets & assets, std::string & error)
  {
    if (!call_filter_mask_load_) {
      return true;
    }
    if (!filter_mask_servers_are_active()) {
      RCLCPP_WARN(
        get_logger(),
        "filter mask servers are not active; selected assets remain on disk and will be loaded by Nav2 startup");
      return true;
    }
    if (!load_map_with_client(
        keepout_mask_load_client_, keepout_mask_load_service_, assets.keepout_mask_yaml, "keepout mask", error))
    {
      return false;
    }
    return load_map_with_client(
      speed_mask_load_client_, speed_mask_load_service_, assets.speed_mask_yaml, "speed mask", error);
  }

  bool apply_localizer_assets(const FloorAssets & assets, std::string & error)
  {
    if (!call_localizer_apply_) {
      return true;
    }
    if (!wait_for_service(localizer_apply_client_, localizer_apply_service_, error)) {
      return false;
    }
    auto request = std::make_shared<robot_interfaces::srv::ApplyFloorAssets::Request>();
    request->floor_id = assets.building_id + "/" + assets.floor_id;
    request->nav_map_yaml = assets.nav_map_yaml.string();
    request->localizer_map_png = assets.localizer_map_png.string();
    request->localizer_params_yaml = assets.localizer_params_yaml.string();
    auto future = localizer_apply_client_->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error = "timed out applying localizer floor assets";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      error = "global localization rejected floor assets: " + response->message;
      return false;
    }
    return true;
  }

  bool trigger_localization(const FloorAssets & assets, std::string & error)
  {
    if (!call_localization_trigger_) {
      return true;
    }
    if (!wait_for_service(localization_trigger_client_, localization_trigger_service_, error)) {
      return false;
    }
    auto request = std::make_shared<robot_interfaces::srv::TriggerLocalization::Request>();
    request->reason = "floor_switch:" + assets.building_id + "/" + assets.floor_id;
    auto future = localization_trigger_client_->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error = "timed out triggering global localization";
      return false;
    }
    const auto response = future.get();
    if (!response->accepted) {
      error = "global localization trigger rejected: " + response->message;
      return false;
    }
    return true;
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
    const auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      error = "timed out clearing costmap service: " + service_name;
      return false;
    }
    return true;
  }

  void on_switch_floor(
    const std::shared_ptr<robot_interfaces::srv::SwitchFloor::Request> request,
    std::shared_ptr<robot_interfaces::srv::SwitchFloor::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      if (switching_ || floor_switch_action_active_) {
        response->success = false;
        response->message = "floor switch or selection already in progress";
        return;
      }
      switching_ = true;
    }

    FloorAssets assets;
    std::string error;
    const auto finish = [&](const bool success, const std::string & message) {
      response->success = success;
      response->message = message;
      response->nav_map_yaml = assets.nav_map_yaml.string();
      response->localizer_map_png = assets.localizer_map_png.string();
      response->localizer_params_yaml = assets.localizer_params_yaml.string();
      std::string current_floor;
      {
        std::lock_guard<std::mutex> lock(floor_state_mutex_);
        current_floor = current_floor_;
        switching_ = false;
      }
      publish_status(success ? ("active:" + current_floor) : ("failed:" + message));
    };

    if (request->resume_navigation) {
      finish(
        false,
        "LEGACY_RESUME_NAVIGATION_DISABLED: use /floor_manager/floor_switch; "
        "the legacy service cannot prove an atomic localizer reload");
      return;
    }

    if (!validate_floor_assets(request->building_id, request->floor_id, assets, error)) {
      finish(false, error);
      return;
    }

    const std::string floor_key = assets.building_id + "/" + assets.floor_id;
    publish_status("switching:" + floor_key);

    {
      std::lock_guard<std::mutex> lock(floor_state_mutex_);
      current_floor_ = floor_key;
      selected_building_id_ = assets.building_id;
      selected_floor_id_ = assets.floor_id;
    }
    finish(true, "floor assets selected for next navigation: " + floor_key);
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
  std::string current_floor_;
  std::string selected_building_id_;
  std::string selected_floor_id_;
  std::string selected_map_id_;
  std::string active_floor_switch_transaction_;
  double service_timeout_sec_{10.0};
  double filter_mask_state_timeout_sec_{0.5};
  bool call_map_server_load_{true};
  bool call_filter_mask_load_{true};
  bool call_localizer_apply_{true};
  bool call_localization_trigger_{true};
  bool clear_costmaps_after_switch_{true};
  bool require_filter_assets_{true};
  bool switching_{false};
  bool live_floor_switch_enabled_{false};
  bool floor_switch_action_active_{false};
  std::uint64_t transition_status_generation_{0U};
  std::mutex floor_state_mutex_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<robot_interfaces::msg::FloorSwitchStatus>::SharedPtr
    transition_status_pub_;
  rclcpp::Service<robot_interfaces::srv::SwitchFloor>::SharedPtr switch_service_;
  rclcpp_action::Server<FloorSwitchAction>::SharedPtr floor_switch_action_server_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr keepout_mask_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr speed_mask_load_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr keepout_mask_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr speed_mask_state_client_;
  rclcpp::Client<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr localizer_apply_client_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr localization_trigger_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
};

#ifndef ROBOT_FLOOR_MANAGER_DISABLE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
