#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace rclcpp
{
class Node;
}

namespace robot_api_server::application::subscriptions
{

struct SubscriptionModuleConfig
{
  int default_ttl_ms{10000};
  int max_ttl_ms{60000};
  std::string scan_topic{"/scan"};
  double scan_max_age_sec{2.0};
  std::chrono::milliseconds expiry_period{1000};
};

struct SubscriptionModulePorts
{
  std::function<void()> ensure_status_resident;
  std::function<void(bool active)> set_live_map_page_active;
  std::function<void()> ensure_tf_resident;
  std::function<void()> clear_teleop_command;
};

struct SubscriptionScanSnapshot
{
  bool active{false};
  bool available{false};
  bool fresh{false};
  std::string frame_id;
  std::size_t range_count{0U};
  double angle_min{0.0};
  double angle_max{0.0};
  double age_sec{-1.0};
};

// Complete page-subscription application boundary. It owns HTTP lease
// semantics, refcounts, expiry, resource transitions, and the page-scoped
// high-rate LaserScan cache. Cross-domain resources are controlled only
// through narrow ports supplied by the composition root.
class SubscriptionModule
{
public:
  SubscriptionModule(
    rclcpp::Node & node,
    SubscriptionModuleConfig config,
    SubscriptionModulePorts ports);
  ~SubscriptionModule();

  SubscriptionModule(const SubscriptionModule &) = delete;
  SubscriptionModule & operator=(const SubscriptionModule &) = delete;

  std::optional<HttpResponse> handle_http(const HttpRequest & request);
  void acquire(
    const std::string & client_id,
    const std::vector<std::string> & resources,
    std::chrono::milliseconds ttl);
  void release(
    const std::string & client_id,
    const std::vector<std::string> & resources);
  std::string snapshot_json() const;
  SubscriptionScanSnapshot scan_snapshot() const;
  int max_ttl_ms() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::application::subscriptions
