#include "robot_bringup/runtime_health_json.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>

using namespace robot_bringup::health;

namespace {
int emit(const char *status, int code, const Json &d) {
  std::cout << "status=" << status;
  const auto generation = string(field(d, "generation"));
  const auto &sequence = field(d, "sequence");
  if (!generation.empty() && sequence.IsUint64()) {
    std::cout << " evidence_id=" << generation << ':' << sequence.GetUint64();
  } else if (std::isfinite(number(field(d, "updated_at"))) && number(field(d, "updated_at")) > 0) {
    const double stamp = number(field(d, "updated_at"));
    if (stamp < 9007199254.0) std::cout << " evidence_id=legacy:" << static_cast<uint64_t>(stamp * 1e6);
  }
  const auto &odom = field(field(d, "topics"), "/local_state/odometry");
  const auto &watch = field(d, "odom_watch");
  std::cout << " odom_age_sec=" << number(field(odom, "last_age_sec"))
            << " no_update_age_sec=" << number(field(watch, "no_update_age_sec")) << '\n';
  return code;
}
double snapshot_age(const Json &d) {
  return field(d, "updated_monotonic_sec").IsNumber()
    ? monotonic_now() - number(field(d, "updated_monotonic_sec"))
    : wall_now() - number(field(d, "updated_at"));
}
double current_age(const Json &d, const Json &sample) {
  return number(field(sample, "last_age_sec")) + std::max(0.0, snapshot_age(d));
}
int observer_status(const Json &d, double max_age) {
  if (!d.IsObject() || !std::isfinite(number(field(d, "updated_at")))) return 41;
  const auto &watch = field(d, "odom_watch");
  bool cpp = false;
  for (const char *key : {"implementation", "odom_watch", "generation", "sequence",
      "updated_monotonic_sec", "boot_id", "clock_valid", "sampling_delayed",
      "sample_period_sec", "message_count_semantics"}) cpp = cpp || d.HasMember(key);
  const auto &sequence = field(d, "sequence");
  const auto generation = string(field(d, "generation"));
  if (cpp && (string(field(d, "implementation")) != "cpp" ||
       string(field(d, "schema")) != "njrh.runtime_health.v1" || !watch.IsObject() ||
       !sequence.IsUint64() || sequence.GetUint64() == 0 || sequence.GetUint64() > 9007199254740991ULL ||
       generation.empty() || generation.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-") != std::string::npos ||
       !std::isfinite(number(field(d, "updated_monotonic_sec"))) ||
       !field(d, "clock_valid").IsBool() || string(field(d, "boot_id")).empty() ||
       !std::isfinite(number(field(watch, "no_update_age_sec"))) || number(field(watch, "no_update_age_sec")) < 0 ||
       !std::isfinite(number(field(watch, "timeout_sec"))) || number(field(watch, "timeout_sec")) <= 0 ||
       !field(watch, "seen_valid").IsBool())) return 41;
  const double age = snapshot_age(d);
  if (field(d, "updated_monotonic_sec").IsNumber()) {
    if (string(field(d, "boot_id")) != read_text("/proc/sys/kernel/random/boot_id")) return 42;
  }
  if (!std::isfinite(age) || age < -0.25) return 43;
  if (age > max_age) return 42;
  if (cpp && std::abs((wall_now() - number(field(d, "updated_at"))) - age) > 0.5) return 43;
  if (field(d, "clock_valid").IsBool() && !boolean(field(d, "clock_valid"))) return 43;
  if (boolean(field(d, "sampling_delayed"))) return 46;
  return 0;
}
int diagnostic(const Json &d, double legacy_odom_age) {
  const auto &topic = field(field(d, "topics"), "/local_state/odometry");
  const auto &summary = field(d, "summary");
  const bool endpoint = boolean(field(summary, "local_state_endpoint_ready"));
  const bool publishers = number(field(topic, "publishers"), 0) > 0;
  const auto &watch = field(d, "odom_watch");
  if (watch.IsObject()) {
    const double age = number(field(watch, "no_update_age_sec")) + std::max(0.0, snapshot_age(d));
    const double timeout = number(field(watch, "timeout_sec"));
    if (!std::isfinite(age) || age < 0 || !std::isfinite(timeout) || timeout <= 0 ||
        !field(watch, "seen_valid").IsBool()) return emit("observer_invalid", 41, d);
    if (age < timeout) {
      if (!boolean(field(watch, "seen_valid"))) return emit("observer_sampling_warmup", 45, d);
      if (!endpoint || !publishers) return emit("observer_graph_inconsistent", 44, d);
      return emit("ready", 0, d);
    }
    return emit("odom_no_fresh_update", 56, d);
  }
  // One deployment may read the previous writer's snapshot before the full restart.
  const double age = number(field(topic, "last_age_sec"));
  const double received_age = wall_now() - number(field(topic, "last_received_at"));
  const bool fresh = std::isfinite(age) && age >= -0.25 && age <= legacy_odom_age &&
    received_age >= -0.25 && received_age <= std::max(2.0, legacy_odom_age * 3.0);
  if (fresh && (!endpoint || !publishers)) return emit("observer_graph_inconsistent", 44, d);
  if (!endpoint) return emit("endpoint_missing", 50, d);
  if (!publishers) return emit("publisher_missing", 51, d);
  if (!field(topic, "last_received_at").IsNumber()) return emit("odom_unseen", 52, d);
  if (!fresh) return emit("odom_stamp_stale", 53, d);
  return emit("ready", 0, d);
}
}

int main(int argc, char **argv) {
  // This executable deliberately has no rclcpp dependency and no DDS participant.
  if (argc < 4) {
    std::cerr << "usage: runtime_health_check FILE MAX_AGE available|diagnostic|check|topic|tf [args]\n";
    return 41;
  }
  rapidjson::Document d;
  try {
    const auto text = read_text(argv[1]);
    d.Parse(text.c_str());
    if (d.HasParseError()) return emit("observer_invalid", 41, d);
    const double max_age = std::stod(argv[2]);
    if (!std::isfinite(max_age) || max_age <= 0) return 41;
    const int rc = observer_status(d, max_age);
    if (rc) return emit(rc == 42 ? "observer_stale" : rc == 43 ? "observer_clock_invalid" :
      rc == 46 ? "observer_sampling_delayed" : "observer_invalid", rc, d);
    const std::string command = argv[3];
    if (command == "available") return 0;
    if (command == "diagnostic") return diagnostic(d, argc > 4 ? std::stod(argv[4]) : 0.75);
    if (command == "check" && argc == 5) {
      std::string key = argv[4];
      if (key == "local_state_endpoint") key = "local_state_endpoint_ready";
      if (key == "local_state_fastlio_endpoint") key = "local_state_fastlio_endpoint_ready";
      if (key == "localization_bridge_endpoint") key = "localization_bridge_endpoint_ready";
      if (key == "scan_topic_seen") return field(field(field(d, "topics"), "/scan"), "last_received_at").IsNumber() ? 0 : 1;
      if (key == "odom_base_tf_seen" || key == "map_odom_tf_seen")
        return field(field(d, "tf"), key == "odom_base_tf_seen" ? "odom->base_link" : "map->odom").IsObject() ? 0 : 1;
      if (key == "local_odom_fresh" || key == "local_state_topic_ready" || key == "local_state_ready") {
        const double age = current_age(d, field(field(d, "topics"), "/local_state/odometry"));
        const char *limit = std::getenv("NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC");
        if (!std::isfinite(age) || age < -0.25 || age > (limit ? std::stod(limit) : 0.75)) return 1;
      }
      if (key == "local_state_ready" || key == "odom_base_tf_fresh" || key == "map_odom_tf_ready") {
        const bool map = key == "map_odom_tf_ready";
        const double age = current_age(d, field(field(d, "tf"), map ? "map->odom" : "odom->base_link"));
        const char *limit = std::getenv(map ? "NJRH_RUNTIME_HEALTH_MAP_TF_FRESH_SEC" : "NJRH_RUNTIME_HEALTH_TF_FRESH_SEC");
        if (!std::isfinite(age) || age < -0.25 || age > (limit ? std::stod(limit) : map ? 1.0 : 0.25)) return 1;
      }
      const char *topic = nullptr;
      const char *limit_key = "NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC";
      double default_limit = 1.5;
      if (key == "docking_observation_fresh" || key == "docking_sensor_healthy") {
        topic = "/dock/target_observation"; limit_key = "NJRH_RUNTIME_HEALTH_DOCKING_FRESH_SEC";
      } else if (key == "safety_status_fresh") topic = "/safety/status";
      else if (key == "local_scan_fresh") {
        topic = "/scan"; limit_key = "NJRH_RUNTIME_HEALTH_SCAN_FRESH_SEC"; default_limit = 1.0;
      } else if (key == "local_costmap_fresh") topic = "/local_costmap/costmap";
      else if (key == "global_costmap_fresh") topic = "/global_costmap/costmap";
      if (topic) {
        const auto &sample = field(field(d, "topics"), topic);
        // Old snapshots did not always retain these optional samples.
        if (sample.IsObject() || string(field(d, "implementation")) == "cpp") {
          const double age = current_age(d, sample);
          const char *limit = std::getenv(limit_key);
          if (!std::isfinite(age) || age < -0.25 || age > (limit ? std::stod(limit) : default_limit)) return 1;
        }
      }
      return boolean(field(field(d, "summary"), key.c_str())) ? 0 : 1;
    }
    if (command == "topic" && argc == 5) {
      const auto &topic = field(field(d, "topics"), argv[4]);
      return field(topic, "last_received_at").IsNumber() && number(field(topic, "publishers"), 0) > 0 ? 0 : 1;
    }
    if (command == "tf" && argc == 7) {
      auto strip = [](std::string s) { auto i = s.find_first_not_of('/'); return i == std::string::npos ? "" : s.substr(i); };
      const auto edge = strip(argv[4]) + "->" + strip(argv[5]);
      const auto age = current_age(d, field(field(d, "tf"), edge.c_str()));
      return std::isfinite(age) && age >= -0.25 && age <= std::stod(argv[6]) ? 0 : 1;
    }
    return 41;
  } catch (const std::exception &e) {
    std::cerr << "runtime_health_check: " << e.what() << '\n';
    return emit("observer_unavailable", 40, d);
  }
}
