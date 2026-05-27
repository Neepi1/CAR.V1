#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

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
  FloorManagerNode()
  : Node("robot_floor_manager")
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
    global_clear_client_ = create_client<std_srvs::srv::Empty>(
      global_costmap_clear_service_, rmw_qos_profile_services_default, callback_group_);
    local_clear_client_ = create_client<std_srvs::srv::Empty>(
      local_costmap_clear_service_, rmw_qos_profile_services_default, callback_group_);

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10).transient_local());

    switch_service_ = create_service<robot_interfaces::srv::SwitchFloor>(
      "/floor_manager/switch_floor",
      std::bind(&FloorManagerNode::on_switch_floor, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    publish_status("idle");
  }

private:
  void publish_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
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
    const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr & client,
    const std::string & service_name,
    std::string & error)
  {
    if (!wait_for_service(client, service_name, error)) {
      RCLCPP_WARN(get_logger(), "%s", error.c_str());
      return true;
    }
    const auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    const auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "timed out clearing costmap service: %s", service_name.c_str());
    }
    return true;
  }

  void on_switch_floor(
    const std::shared_ptr<robot_interfaces::srv::SwitchFloor::Request> request,
    std::shared_ptr<robot_interfaces::srv::SwitchFloor::Response> response)
  {
    if (switching_) {
      response->success = false;
      response->message = "floor switch already in progress";
      return;
    }
    switching_ = true;

    FloorAssets assets;
    std::string error;
    const auto finish = [&](const bool success, const std::string & message) {
      response->success = success;
      response->message = message;
      response->nav_map_yaml = assets.nav_map_yaml.string();
      response->localizer_map_png = assets.localizer_map_png.string();
      response->localizer_params_yaml = assets.localizer_params_yaml.string();
      publish_status(success ? ("active:" + current_floor_) : ("failed:" + message));
      switching_ = false;
    };

    if (!validate_floor_assets(request->building_id, request->floor_id, assets, error)) {
      finish(false, error);
      return;
    }

    const std::string floor_key = assets.building_id + "/" + assets.floor_id;
    const bool pre_nav_resume = request->resume_navigation;
    publish_status("switching:" + floor_key);

    if (!pre_nav_resume) {
      current_floor_ = floor_key;
      finish(true, "floor assets selected for next navigation: " + floor_key);
      return;
    }

    if (!load_nav_map(assets, error)) {
      finish(false, error);
      return;
    }
    if (!pre_nav_resume && !load_filter_masks(assets, error)) {
      finish(false, error);
      return;
    }
    if (!apply_localizer_assets(assets, error)) {
      finish(false, error);
      return;
    }
    if (!trigger_localization(assets, error)) {
      finish(false, error);
      return;
    }
    if (!pre_nav_resume && clear_costmaps_after_switch_) {
      (void)clear_costmap(global_clear_client_, global_costmap_clear_service_, error);
      (void)clear_costmap(local_clear_client_, local_costmap_clear_service_, error);
    }

    current_floor_ = floor_key;
    finish(true, "floor switch complete: " + floor_key);
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
  std::string current_floor_;
  double service_timeout_sec_{10.0};
  double filter_mask_state_timeout_sec_{0.5};
  bool call_map_server_load_{true};
  bool call_filter_mask_load_{true};
  bool call_localizer_apply_{true};
  bool call_localization_trigger_{true};
  bool clear_costmaps_after_switch_{true};
  bool require_filter_assets_{true};
  bool switching_{false};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<robot_interfaces::srv::SwitchFloor>::SharedPtr switch_service_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr keepout_mask_load_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr speed_mask_load_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr keepout_mask_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr speed_mask_state_client_;
  rclcpp::Client<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr localizer_apply_client_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr localization_trigger_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_clear_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_clear_client_;
};

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
