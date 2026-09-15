#pragma once

#include "robot_bringup/runtime_flatscan_snapshot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_bringup {
class RuntimeFlatScanMonitor {
public:
  // Called only by the guard sampling thread; the callback group has no executor.
  explicit RuntimeFlatScanMonitor(rclcpp::Node &node, double metadata_max_age_sec = 3.0,
                                 double graph_max_age_sec = 15.0);
  void tick(double monotonic_sec);
  void update_graph(double monotonic_sec, bool valid, int scan_publishers,
                    int flatscan_publishers, int metadata_publishers);
  health::Json snapshot(health::Allocator &allocator) const;
private:
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rapidjson::Document input_;
  flatscan::StampProgress stamp_progress_;
  std::string boot_id_;
  double metadata_max_age_sec_, graph_max_age_sec_, graph_at_{0};
  bool metadata_valid_{false}, graph_valid_{false};
  int scan_publishers_{0}, flatscan_publishers_{0}, metadata_publishers_{0};
  uint64_t graph_sequence_{0};
};
}  // namespace robot_bringup
