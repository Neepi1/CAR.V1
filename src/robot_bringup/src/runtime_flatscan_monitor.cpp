#include "robot_bringup/runtime_flatscan_monitor.hpp"
#include <algorithm>

namespace robot_bringup {
using namespace health;

RuntimeFlatScanMonitor::RuntimeFlatScanMonitor(rclcpp::Node &node, double metadata_max_age_sec,
                                             double graph_max_age_sec)
: boot_id_(read_text("/proc/sys/kernel/random/boot_id")),
  metadata_max_age_sec_(metadata_max_age_sec), graph_max_age_sec_(graph_max_age_sec) {
  if (!std::isfinite(metadata_max_age_sec_) || metadata_max_age_sec_ <= 0 ||
      !std::isfinite(graph_max_age_sec_) || graph_max_age_sec_ <= 0)
    throw std::invalid_argument("FlatScan observer ages must be positive");
  group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options; options.callback_group = group_;
  subscription_ = node.create_subscription<std_msgs::msg::String>(
    "/global_localization/flatscan_input_status", rclcpp::QoS(1).best_effort(),
    [](std_msgs::msg::String::ConstSharedPtr) {}, options);
}

void RuntimeFlatScanMonitor::tick(double monotonic_sec) {
  std_msgs::msg::String msg; rclcpp::MessageInfo info;
  if (!subscription_->take(msg, info)) return;
  // A message can arrive after the caller sampled its clock but before take().
  monotonic_sec = std::max(monotonic_sec, monotonic_now());
  rapidjson::Document next;
  if (msg.data.size() > 4096 || next.Parse(msg.data.c_str()).HasParseError() ||
      !flatscan::valid_metadata(next, monotonic_sec, boot_id_)) {
    metadata_valid_ = false;
    return;
  }
  if (input_.IsObject() && string(field(next, "generation")) == string(field(input_, "generation"))) {
    const auto previous = field(input_, "sequence").GetUint64();
    const auto current = field(next, "sequence").GetUint64();
    if (current == previous) return;  // Replays cannot refresh the evidence timestamp.
    if (current < previous || number(field(next, "emitted_monotonic_sec")) <=
        number(field(input_, "emitted_monotonic_sec")) ||
        field(next, "input_sequence").GetUint64() < field(input_, "input_sequence").GetUint64() ||
        number(field(next, "received_monotonic_sec")) < number(field(input_, "received_monotonic_sec")) ||
        (field(next, "input_sequence").GetUint64() == field(input_, "input_sequence").GetUint64() &&
         number(field(next, "received_monotonic_sec")) != number(field(input_, "received_monotonic_sec")))) {
      metadata_valid_ = false;
      return;
    }
  }
  stamp_progress_.observe(next);
  input_.Swap(next);
  metadata_valid_ = true;
}

void RuntimeFlatScanMonitor::update_graph(double now, bool valid, int scan, int flatscan,
                                         int metadata) {
  graph_at_ = now; ++graph_sequence_; graph_valid_ = valid;
  scan_publishers_ = scan; flatscan_publishers_ = flatscan; metadata_publishers_ = metadata;
}

Json RuntimeFlatScanMonitor::snapshot(Allocator &a) const {
  Json out(rapidjson::kObjectType), graph(rapidjson::kObjectType), input;
  put(out, "schema", "njrh.flatscan_monitor.v1", a);
  put(out, "metadata_valid", metadata_valid_, a);
  put(out, "metadata_max_age_sec", metadata_max_age_sec_, a);
  put(out, "graph_max_age_sec", graph_max_age_sec_, a);
  put_json(out, "stamp_progress", stamp_progress_.snapshot(a), a);
  input.CopyFrom(input_, a); put_json(out, "input", std::move(input), a);
  put(graph, "checked_monotonic_sec", graph_at_, a); put(graph, "sequence", graph_sequence_, a);
  put(graph, "valid", graph_valid_, a);
  put(graph, "scan_publishers", scan_publishers_, a);
  put(graph, "flatscan_publishers", flatscan_publishers_, a);
  put(graph, "metadata_publishers", metadata_publishers_, a);
  put_json(out, "graph", std::move(graph), a);
  return out;
}
}  // namespace robot_bringup
