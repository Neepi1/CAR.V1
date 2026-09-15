#pragma once

#include "robot_bringup/runtime_health_json.hpp"
#include <cstdint>

namespace robot_bringup::flatscan {
using namespace robot_bringup::health;

inline bool token(const std::string &s) {
  return !s.empty() && s.size() <= 160 &&
    s.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") == std::string::npos;
}
inline bool sequence(const Json &v) {
  return v.IsUint64() && v.GetUint64() > 0 && v.GetUint64() <= 9007199254740991ULL;
}
inline bool finite_nonnegative(const Json &v) {
  return std::isfinite(number(v)) && number(v) >= 0;
}
inline bool valid_metadata(const Json &d, double now, const std::string &boot) {
  auto trim_boot = [](std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
  };
  if (string(field(d, "schema")) != "njrh.flatscan_input.v1" ||
      trim_boot(string(field(d, "boot_id"))) != trim_boot(boot) || !token(string(field(d, "generation"))) ||
      !sequence(field(d, "sequence")) || !field(d, "input_supported").IsBool() ||
      !field(d, "available").IsBool() || !field(d, "input_sequence").IsUint64() ||
      !finite_nonnegative(field(d, "header_stamp_sec")) ||
      !finite_nonnegative(field(d, "emitted_monotonic_sec")) ||
      !finite_nonnegative(field(d, "received_monotonic_sec"))) return false;
  const double emitted = number(field(d, "emitted_monotonic_sec"));
  const double received = number(field(d, "received_monotonic_sec"));
  return emitted <= now && received <= emitted &&
    (!boolean(field(d, "available")) || (sequence(field(d, "input_sequence")) && received > 0));
}

class StampProgress {
public:
  void observe(const Json &input) {
    const auto generation = string(field(input, "generation"));
    if (generation != generation_) {
      *this = StampProgress{};
      generation_ = generation;
    }
    if (!boolean(field(input, "available"))) return;
    const auto seq = field(input, "input_sequence").GetUint64();
    const double stamp = number(field(input, "header_stamp_sec"));
    if (seeded_ && seq == input_sequence_) {
      // A timer heartbeat without another receive is not a new stamp sample.
      if (stamp != last_received_stamp_) replay_detected_ = true;
      return;
    }
    if (!std::isfinite(stamp) || stamp <= 0 || (seeded_ && seq < input_sequence_)) {
      replay_detected_ = true;
      return;
    }
    input_sequence_ = seq;
    last_received_stamp_ = stamp;
    if (!seeded_) {
      seeded_ = true;
      watermark_ = stamp;
      return;  // Two distinct advancing observations are needed after a new generation.
    }
    if (stamp <= watermark_) {
      replay_detected_ = true;
      return;
    }
    watermark_ = stamp;
    seen_advancing_ = true;
    replay_detected_ = false;
    last_advance_ = number(field(input, "received_monotonic_sec"));
    advance_sequence_ = seq;
  }
  Json snapshot(Allocator &a) const {
    Json out(rapidjson::kObjectType);
    put(out, "generation", generation_, a);
    put(out, "seen_advancing", seen_advancing_, a);
    put(out, "replay_detected", replay_detected_, a);
    put(out, "header_watermark_sec", watermark_, a);
    put(out, "last_advance_monotonic_sec", last_advance_, a);
    put(out, "last_advance_input_sequence", advance_sequence_, a);
    return out;
  }
private:
  std::string generation_;
  bool seeded_{false}, seen_advancing_{false}, replay_detected_{false};
  uint64_t input_sequence_{0}, advance_sequence_{0};
  double watermark_{0}, last_received_stamp_{0}, last_advance_{0};
};

struct Decision {
  int code{40};
  std::string status{"observer_unavailable"}, source_id, evidence_kind;
  uint64_t evidence_sequence{0};
};

// Only code 50 is a candidate for the owner's existing independent stream probe.
// A valid heartbeat is not evidence that the FlatScan input advanced.
inline Decision evaluate(const Json &root, double now, double wall, const std::string &boot,
                         double max_age, double stream_timeout) {
  Decision out;
  const double written = number(field(root, "updated_monotonic_sec"));
  if (string(field(root, "schema")) != "njrh.runtime_health.v1" ||
      string(field(root, "implementation")) != "cpp" ||
      string(field(root, "boot_id")) != boot || !token(string(field(root, "generation"))) ||
      !sequence(field(root, "sequence")) || !boolean(field(root, "clock_valid")) ||
      !field(root, "sampling_delayed").IsBool() || boolean(field(root, "sampling_delayed")) ||
      !std::isfinite(written) || written > now || now - written > max_age ||
      !std::isfinite(number(field(root, "updated_at"))) ||
      std::abs((wall - number(field(root, "updated_at"))) - (now - written)) > 0.5) return out;

  const auto &monitor = field(root, "flatscan_monitor");
  const auto &input = field(monitor, "input");
  const auto &graph = field(monitor, "graph");
  if (string(field(monitor, "schema")) != "njrh.flatscan_monitor.v1" ||
      !boolean(field(monitor, "metadata_valid")) || !valid_metadata(input, written, boot) ||
      !boolean(field(input, "input_supported")) || !boolean(field(input, "available"))) return out;
  const double metadata_limit = number(field(monitor, "metadata_max_age_sec"));
  const double graph_limit = number(field(monitor, "graph_max_age_sec"));
  const double graph_at = number(field(graph, "checked_monotonic_sec"));
  if (!std::isfinite(metadata_limit) || metadata_limit <= 0 ||
      now - number(field(input, "emitted_monotonic_sec")) > metadata_limit ||
      !boolean(field(graph, "valid")) || !sequence(field(graph, "sequence")) ||
      !std::isfinite(graph_limit) || graph_limit <= 0 || !std::isfinite(graph_at) ||
      graph_at > written || now - graph_at > graph_limit ||
      !field(graph, "scan_publishers").IsInt() || field(graph, "scan_publishers").GetInt() < 0 ||
      !field(graph, "flatscan_publishers").IsInt() || field(graph, "flatscan_publishers").GetInt() < 0 ||
      !field(graph, "metadata_publishers").IsInt() || field(graph, "metadata_publishers").GetInt() != 1) return out;

  const auto &progress = field(monitor, "stamp_progress");
  const double advanced_at = number(field(progress, "last_advance_monotonic_sec"));
  if (string(field(progress, "generation")) != string(field(input, "generation")) ||
      !boolean(field(progress, "seen_advancing")) ||
      !field(progress, "replay_detected").IsBool() || boolean(field(progress, "replay_detected")) ||
      number(field(input, "header_stamp_sec")) <= 0 ||
      number(field(progress, "header_watermark_sec")) != number(field(input, "header_stamp_sec")) ||
      !sequence(field(progress, "last_advance_input_sequence")) ||
      field(progress, "last_advance_input_sequence").GetUint64() > field(input, "input_sequence").GetUint64() ||
      !std::isfinite(advanced_at) || advanced_at <= 0 ||
      advanced_at > number(field(input, "received_monotonic_sec"))) return out;

  out.source_id = string(field(root, "generation")) + ":" + string(field(input, "generation"));
  out.evidence_kind = "input";
  out.evidence_sequence = field(input, "sequence").GetUint64();
  const double input_age = now - number(field(input, "received_monotonic_sec"));
  if (input_age <= stream_timeout) {
    if (now - advanced_at > stream_timeout) return Decision{};
    // Recent advancing frame stamps outweigh a delayed graph cache.
    out.code = 0; out.status = "stream_observed";
  } else {
    out.code = 50; out.status = "stream_fault_candidate";
    if (field(graph, "flatscan_publishers").GetInt() == 0) {
      out.evidence_kind = "graph";
      out.evidence_sequence = field(graph, "sequence").GetUint64();
    }
  }
  return out;
}
}  // namespace robot_bringup::flatscan
