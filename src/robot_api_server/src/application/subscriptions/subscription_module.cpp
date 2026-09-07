#include "robot_api_server/application/subscriptions/subscription_module.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_api_server/application/subscriptions/subscription_api.hpp"
#include "robot_api_server/application/subscriptions/subscription_manager.hpp"

namespace robot_api_server::application::subscriptions
{
namespace
{

SubscriptionModuleConfig normalize_config(SubscriptionModuleConfig config)
{
  config.default_ttl_ms = std::max(1000, config.default_ttl_ms);
  config.max_ttl_ms = std::max(config.default_ttl_ms, config.max_ttl_ms);
  config.scan_max_age_sec = std::max(0.1, config.scan_max_age_sec);
  config.expiry_period = std::max(std::chrono::milliseconds(1), config.expiry_period);
  return config;
}

}  // namespace

class SubscriptionModule::Impl
{
public:
  Impl(
    rclcpp::Node & module_node,
    SubscriptionModuleConfig module_config,
    SubscriptionModulePorts module_ports)
  : node(module_node),
    config(normalize_config(std::move(module_config))),
    ports(std::move(module_ports)),
    manager(
      std::vector<std::string>{"status", "live_map", "scan", "tf", "teleop"},
      [this](const std::string & resource, const bool active) {
        set_resource_active(resource, active);
      })
  {
    expiry_timer = node.create_wall_timer(config.expiry_period, [this]() {manager.expire();});
  }

  std::optional<HttpResponse> handle_http(const HttpRequest & request)
  {
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/acquire") {
      return handle_subscription_update(request.body, "acquire");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/release") {
      return handle_subscription_update(request.body, "release");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/heartbeat") {
      return handle_subscription_update(request.body, "heartbeat");
    }
    return std::nullopt;
  }

  HttpResponse handle_subscription_update(const std::string & body, const std::string & action)
  {
    const auto [client_id, client_id_source] = subscription_client_id_from_body(body);
    auto resources = subscription_resources_from_body(body);
    if (action == "heartbeat" && resources.empty()) {
      resources = manager.resources_for_client(client_id);
    }
    if (action == "heartbeat" && resources.empty()) {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"action\":" << json_string(action) << ","
               << "\"client_id\":" << json_string(client_id) << ","
               << "\"lease_id\":" << json_string(client_id) << ","
               << "\"client_id_source\":" << json_string(client_id_source) << ","
               << "\"refreshed\":false,"
               << "\"ttl_ms\":0,"
               << "\"resources\":[],"
               << "\"subscriptions\":" << manager.snapshot_json() << "}";
      return {200, "application/json", response.str()};
    }
    if (action != "release" && resources.empty()) {
      return {400, "application/json", error_json("resources array is required")};
    }
    if (const auto error = manager.validate_resources(resources)) {
      return {400, "application/json", error_json(*error)};
    }

    int ttl_ms = subscription_ttl_ms_from_body(
      body,
      config.default_ttl_ms,
      config.max_ttl_ms);
    if (action == "acquire" || action == "heartbeat") {
      manager.acquire(client_id, resources, std::chrono::milliseconds(ttl_ms));
    } else if (action == "release") {
      manager.release(client_id, resources);
      ttl_ms = 0;
    } else {
      return {400, "application/json", error_json("unsupported subscription action")};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"action\":" << json_string(action) << ","
             << "\"client_id\":" << json_string(client_id) << ","
             << "\"lease_id\":" << json_string(client_id) << ","
             << "\"client_id_source\":" << json_string(client_id_source) << ","
             << "\"refreshed\":" << ((action == "heartbeat") ? "true" : "false") << ","
             << "\"ttl_ms\":" << ttl_ms << ","
             << "\"resources\":" << resource_list_json(resources) << ","
             << "\"subscriptions\":" << manager.snapshot_json() << "}";
    return {200, "application/json", response.str()};
  }

  void set_resource_active(const std::string & resource, const bool active)
  {
    if (resource == "status") {
      // Status and safety/floor inputs are process-resident. A page lease is
      // compatibility interest only and must never tear down those inputs.
      if (ports.ensure_status_resident) {
        ports.ensure_status_resident();
      }
    } else if (resource == "live_map") {
      if (ports.set_live_map_page_active) {
        ports.set_live_map_page_active(active);
      }
    } else if (resource == "scan") {
      set_scan_subscription_active(active);
    } else if (resource == "tf") {
      // TF is a process-level localization input. Lease transitions only
      // record interest and always converge to the resident subscription.
      if (ports.ensure_tf_resident) {
        ports.ensure_tf_resident();
      }
    } else if (resource == "teleop" && !active) {
      if (ports.clear_teleop_command) {
        ports.clear_teleop_command();
      }
    }
  }

  void set_scan_subscription_active(const bool active)
  {
    {
      std::lock_guard<std::mutex> lifecycle_lock(scan_lifecycle_mutex);
      if (active) {
        if (!scan_subscription) {
          scan_subscription = node.create_subscription<sensor_msgs::msg::LaserScan>(
            config.scan_topic,
            rclcpp::QoS(10),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
              std::lock_guard<std::mutex> state_lock(scan_state_mutex);
              scan_frame = message->header.frame_id;
              scan_range_count = message->ranges.size();
              scan_angle_min = message->angle_min;
              scan_angle_max = message->angle_max;
              scan_received_at = std::chrono::steady_clock::now();
              have_scan = true;
            });
        }
        std::lock_guard<std::mutex> state_lock(scan_state_mutex);
        scan_active = true;
        return;
      }
      scan_subscription.reset();
    }

    std::lock_guard<std::mutex> state_lock(scan_state_mutex);
    scan_active = false;
    have_scan = false;
    scan_frame.clear();
    scan_range_count = 0U;
    scan_angle_min = 0.0;
    scan_angle_max = 0.0;
    scan_received_at = {};
  }

  SubscriptionScanSnapshot scan_snapshot() const
  {
    std::lock_guard<std::mutex> lock(scan_state_mutex);
    SubscriptionScanSnapshot snapshot;
    snapshot.active = scan_active;
    snapshot.available = have_scan;
    snapshot.frame_id = scan_frame;
    snapshot.range_count = scan_range_count;
    snapshot.angle_min = scan_angle_min;
    snapshot.angle_max = scan_angle_max;
    if (have_scan) {
      snapshot.age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - scan_received_at).count();
      snapshot.fresh = snapshot.age_sec <= config.scan_max_age_sec;
    }
    return snapshot;
  }

  rclcpp::Node & node;
  SubscriptionModuleConfig config;
  SubscriptionModulePorts ports;
  SubscriptionManager manager;
  mutable std::mutex scan_state_mutex;
  std::mutex scan_lifecycle_mutex;
  bool scan_active{false};
  bool have_scan{false};
  std::string scan_frame;
  std::size_t scan_range_count{0U};
  double scan_angle_min{0.0};
  double scan_angle_max{0.0};
  std::chrono::steady_clock::time_point scan_received_at{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
  rclcpp::TimerBase::SharedPtr expiry_timer;
};

SubscriptionModule::SubscriptionModule(
  rclcpp::Node & node,
  SubscriptionModuleConfig config,
  SubscriptionModulePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

SubscriptionModule::~SubscriptionModule() = default;

std::optional<HttpResponse> SubscriptionModule::handle_http(const HttpRequest & request)
{
  return impl_->handle_http(request);
}

void SubscriptionModule::acquire(
  const std::string & client_id,
  const std::vector<std::string> & resources,
  const std::chrono::milliseconds ttl)
{
  impl_->manager.acquire(client_id, resources, ttl);
}

void SubscriptionModule::release(
  const std::string & client_id,
  const std::vector<std::string> & resources)
{
  impl_->manager.release(client_id, resources);
}

std::string SubscriptionModule::snapshot_json() const
{
  return impl_->manager.snapshot_json();
}

SubscriptionScanSnapshot SubscriptionModule::scan_snapshot() const
{
  return impl_->scan_snapshot();
}

int SubscriptionModule::max_ttl_ms() const
{
  return impl_->config.max_ttl_ms;
}

}  // namespace robot_api_server::application::subscriptions
