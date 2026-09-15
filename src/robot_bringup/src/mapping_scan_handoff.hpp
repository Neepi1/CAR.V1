#pragma once

#include <chrono>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mapping_startup
{
// One participant observes the real resident state AND endpoint transitions.
// Empty graph on a cold participant is unknown, never proof of disabled scan.
class ScanHandoff
{
public:
  enum class Result { Waiting, Ready, Rejected };
  ScanHandoff(const rclcpp::Node::SharedPtr & node, std::string topic,
    std::string owner, const std::string & service, const std::string & status_topic,
    bool enable)
  : node_(node), topic_(std::move(topic)), owner_(std::move(owner)),
    status_topic_(status_topic), enable_(enable), started_ns_(node->now().nanoseconds())
  {
    client_ = node_->create_client<std_srvs::srv::SetBool>(service);
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](std_msgs::msg::String::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        const auto & wire = info.get_rmw_message_info();
        if (wire.source_timestamp < started_ns_) { return; }
        std::istringstream fields(msg->data);
        std::string field, enabled, registered;
        while (fields >> field) {
          if (field.rfind("worker_scan_enabled=", 0) == 0) { enabled = field.substr(20); }
          if (field.rfind("scan_publisher_registered=", 0) == 0) { registered = field.substr(26); }
        }
        status_.reset();
        if (enabled == registered && (enabled == "true" || enabled == "false")) {
          status_ = enabled == "true";
          status_gid_ = wire.publisher_gid;
          status_at_ = std::chrono::steady_clock::now();
        }
      });
  }

  ~ScanHandoff()
  {
    if (pending_) { client_->remove_pending_request(*pending_); }
  }

  Result tick()
  {
    const auto publishers = node_->get_publishers_info_by_topic(topic_);
    const bool owned = publishers.size() == 1 && publishers.front().node_name() == owner_;
    if (pending_ && pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto response = pending_->get();
      pending_.reset();
      if (!response->success) { return Result::Rejected; }
      acknowledged_ = true;
    }
    // Actual unique ownership can prove success even if the RPC reply is late.
    if (enable_ && owned) { return Result::Ready; }
    // An acknowledged reset plus disappearance on this SAME warmed graph
    // proves release; a new client's initial zero does not.
    if (!enable_ && acknowledged_ && publishers.empty()) { return Result::Ready; }

    bool fresh_disabled = false;
    if (status_ && !*status_ &&
      std::chrono::steady_clock::now() - status_at_ < std::chrono::seconds(3))
    {
      const auto owners = node_->get_publishers_info_by_topic(status_topic_);
      fresh_disabled = owners.size() == 1 && owners.front().node_name() == owner_ &&
        std::memcmp(owners.front().endpoint_gid().data(), status_gid_.data,
          RMW_GID_STORAGE_SIZE) == 0;
    }
    if (!enable_ && fresh_disabled && publishers.empty()) { return Result::Ready; }
    const bool may_request = enable_ ? (fresh_disabled && publishers.empty()) : owned;
    if (!sent_ && may_request && client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enable_;
      pending_.emplace(client_->async_send_request(request));
      sent_ = true;
    }
    return Result::Waiting;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_, owner_, status_topic_;
  bool enable_, sent_{false}, acknowledged_{false};
  int64_t started_ns_;
  std::optional<bool> status_;
  rmw_gid_t status_gid_{};
  std::chrono::steady_clock::time_point status_at_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  std::optional<rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId> pending_;
};
}  // namespace mapping_startup
