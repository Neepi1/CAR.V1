#include "robot_bringup/runtime_health_json.hpp"
#include "robot_bringup/runtime_health_policy.hpp"
#include "robot_bringup/runtime_amcl_status.hpp"
#include "robot_bringup/runtime_flatscan_monitor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <robot_interfaces/msg/dock_target_observation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <algorithm>
#include <cstdlib>
#include <cctype>
#include <cerrno>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <thread>
#include <poll.h>
#include <unistd.h>

using namespace robot_bringup::health;
namespace {
std::string env(const char *key, const char *fallback) {
  const char *v = std::getenv(key);
  return v ? v : fallback;
}
double setting(const char *key, double fallback) {
  const double v = std::stod(env(key, std::to_string(fallback).c_str()));
  if (!std::isfinite(v) || v <= 0) throw std::runtime_error(std::string(key) + " must be positive");
  return v;
}
bool enabled(const char *key) {
  auto s = env(key, "false");
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s == "true" || s == "1" || s == "yes" || s == "on";
}
std::string strip(std::string s) {
  const auto i = s.find_first_not_of('/');
  return i == std::string::npos ? "" : s.substr(i);
}
double stamp(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }
struct Topic {
  std::string type, frame;
  int publishers{0}, subscribers{0};
  uint64_t count{0};
  double received{NAN}, header{NAN}, received_mono{NAN};
  bool sensor_healthy{false}, valid{false};
  std::string source, reason;
};
struct Edge { double header, received; std::string parent, child; };

bool atomic_write(const std::filesystem::path &path, const std::string &data,
                  const std::function<bool()> &can_commit) {
  if (!can_commit()) return false;
  if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
  std::string pattern = (path.parent_path() / ("." + path.filename().string() + ".XXXXXX.tmp")).string();
  const int fd = mkstemps(pattern.data(), 4);
  if (fd < 0) throw std::runtime_error("cannot create health snapshot temporary file");
  try {
    size_t offset = 0;
    while (offset < data.size()) {
      const auto n = write(fd, data.data() + offset, data.size() - offset);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw std::runtime_error("health snapshot write failed");
      offset += static_cast<size_t>(n);
    }
    if (close(fd) != 0) throw std::runtime_error("health snapshot close failed");
  } catch (...) { close(fd); unlink(pattern.c_str()); throw; }
  if (!can_commit()) { unlink(pattern.c_str()); return false; }
  if (rename(pattern.c_str(), path.c_str()) != 0) {
    unlink(pattern.c_str());
    throw std::runtime_error("health snapshot atomic replacement failed");
  }
  return true;
}

class Guard : public rclcpp::Node {
public:
  explicit Guard(const std::string &path)
  : Node("runtime_health_guard", rclcpp::NodeOptions().start_parameter_services(false)
      .start_parameter_event_publisher(false).enable_rosout(false)),
    output_(path),
    sample_period_(setting("NJRH_RUNTIME_HEALTH_SAMPLE_PERIOD_SEC", 1.0)),
    graph_period_(setting("NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC", 5.0)),
    write_period_(setting("NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC", 1.0)),
    timeout_(setting("NJRH_RUNTIME_HEALTH_ODOM_NO_UPDATE_TIMEOUT_SEC", 3.0)),
    watch_(timeout_) {
    if (sample_period_ >= timeout_) throw std::runtime_error("sample period must be less than no-update timeout");
    if (std::abs(write_period_ - sample_period_) > 1e-6)
      throw std::runtime_error("snapshot and sample periods must match");
    started_ = monotonic_now();
    watch_.reset(started_);
    boot_id_ = read_text("/proc/sys/kernel/random/boot_id");
    generation_ = std::to_string(getpid()) + "-" + std::to_string(static_cast<uint64_t>(started_ * 1e6));
    previous_wall_ = wall_now(); previous_ros_ = now().seconds(); previous_mono_ = started_;
    heavy_ = enabled("NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS");
    messages_ = heavy_ || enabled("NJRH_RUNTIME_HEALTH_OBSERVE_TOPIC_MESSAGES");
    all_tf_ = enabled("NJRH_RUNTIME_HEALTH_OBSERVE_ALL_TF");
    tf_ = all_tf_ || enabled("NJRH_RUNTIME_HEALTH_OBSERVE_TF");
    auto tracked = env("NJRH_RUNTIME_HEALTH_TF_TRACKED_EDGES", "map->odom,odom->base_link");
    std::replace(tracked.begin(), tracked.end(), ';', ',');
    std::istringstream input(tracked);
    for (std::string e; std::getline(input, e, ',');) {
      e.erase(std::remove_if(e.begin(), e.end(), [](unsigned char c) { return std::isspace(c); }), e.end());
      if (e.find("->") != std::string::npos) tracked_.insert(e);
    }
    for (const auto &pair : std::map<std::string, std::string>{
      {"/local_state/odometry", "nav_msgs/msg/Odometry"}, {"/fastlio/base_odometry", "nav_msgs/msg/Odometry"},
      {"/Odometry", "nav_msgs/msg/Odometry"}, {"/safety/status", "std_msgs/msg/String"},
      {"/scan", "sensor_msgs/msg/LaserScan"}, {"/map", "nav_msgs/msg/OccupancyGrid"},
      {"/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid"}, {"/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid"},
      {"/localization_result", "geometry_msgs/msg/PoseWithCovarianceStamped"},
      {"/dock/target_observation", "robot_interfaces/msg/DockTargetObservation"}}) topics_[pair.first].type = pair.second;

    // No executor owns this group. Only sample() takes messages, once per period.
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    observe<nav_msgs::msg::Odometry>("/local_state/odometry", false);
    observe<robot_interfaces::msg::DockTargetObservation>("/dock/target_observation", true);
    if (messages_) {
      observe<nav_msgs::msg::Odometry>("/fastlio/base_odometry", true);
      observe<nav_msgs::msg::Odometry>("/Odometry", false);
      observe<sensor_msgs::msg::LaserScan>("/scan", false);
      observe<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization_result", false);
      observe<std_msgs::msg::String>("/safety/status", true);
    }
    if (heavy_) {
      observe<nav_msgs::msg::OccupancyGrid>("/map", true, true);
      observe<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", true);
      observe<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", true);
    }
    if (tf_) {
      rclcpp::SubscriptionOptions options; options.callback_group = group_;
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100).reliable(),
        [](tf2_msgs::msg::TFMessage::ConstSharedPtr) {}, options);
    }
    if (enabled("NJRH_RUNTIME_MANAGEMENT_ENABLED")) {
      robot_bringup::runtime_amcl::Options options;
      options.refresh_sec = setting("NJRH_AMCL_RUNTIME_STATUS_HEARTBEAT_SEC", 2.0);
      options.ttl_sec = setting("NJRH_AMCL_RUNTIME_STATUS_TTL_SEC", 5.0);
      options.startup_grace_sec = setting("NJRH_AMCL_STARTUP_HEARTBEAT_GRACE_SEC", 45.0);
      amcl_ = std::make_unique<robot_bringup::runtime_amcl::StatusServer>(
        env("NJRH_AMCL_RUNTIME_STATUS_FILE", "/tmp/njrh_amcl_runtime_status.env"), options);
      flatscan_ = std::make_unique<robot_bringup::RuntimeFlatScanMonitor>(*this);
    }
  }
  double period() const { return sample_period_; }
  void wait_until(std::chrono::steady_clock::time_point deadline) {
    if (!amcl_) { std::this_thread::sleep_until(deadline); return; }
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      pollfd descriptor{amcl_->fd(), POLLIN, 0};
      const int result = poll(&descriptor, 1, static_cast<int>(std::max<int64_t>(0, remaining)));
      if (result < 0 && errno != EINTR) throw std::runtime_error("management event wait failed");
      if (result > 0 && (descriptor.revents & POLLIN)) amcl_->handle_requests();
    }
  }
  void sample() {
    const double m = monotonic_now(), r = now().seconds(), w = wall_now();
    sample_started_ = m;
    const double elapsed = m - previous_mono_;
    if (std::abs((r - previous_ros_) - elapsed) > 0.5 ||
        std::abs((w - previous_wall_) - elapsed) > 0.5) {
      watch_.reset(m); clock_recover_until_ = m + timeout_;
    }
    previous_mono_ = m; previous_ros_ = r; previous_wall_ = w;
    sample_amcl(r);
    if (flatscan_) flatscan_->tick(m);
    for (const auto &take : samplers_) take(m, r, w);
    if (tf_sub_) {
      tf2_msgs::msg::TFMessage msg; rclcpp::MessageInfo info;
      for (int i = 0; i < 100 && tf_sub_->take(msg, info); ++i) {
        for (const auto &t : msg.transforms) {
          const auto parent = strip(t.header.frame_id), child = strip(t.child_frame_id);
          const auto key = parent + "->" + child;
          if (all_tf_ || tracked_.empty() || tracked_.count(key)) edges_[key] = {stamp(t.header.stamp), w, parent, child};
        }
      }
    }
    if (m - graph_checked_ >= graph_period_ || graph_checked_ == 0.0) refresh_graph(m);
    write_snapshot();
  }
private:
  void sample_amcl(double ros_now) {
    if (!amcl_) return;
    amcl_->handle_requests();
    const uint64_t generation = amcl_->generation();
    if (amcl_->registered() && generation != amcl_generation_) {
      amcl_generation_ = generation;
      amcl_identity_ = amcl_->amcl_identity();
      rclcpp::SubscriptionOptions options; options.callback_group = group_;
      amcl_lifecycle_ = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        "/" + strip(amcl_->node_name()) + "/transition_event", rclcpp::QoS(1).reliable(),
        [](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr) {}, options);
      amcl_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        amcl_->pose_topic(), rclcpp::QoS(1).best_effort(),
        [](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {}, options);
    }
    rclcpp::MessageInfo info;
    if (amcl_lifecycle_) {
      lifecycle_msgs::msg::TransitionEvent event;
      if (amcl_lifecycle_->take(event, info))
        amcl_->observe_lifecycle(amcl_generation_, amcl_identity_, event.goal_state.id == 3);
    }
    if (amcl_pose_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      if (amcl_pose_->take(pose, info)) amcl_->observe_pose(amcl_generation_, stamp(pose.header.stamp), ros_now);
    }
    amcl_->tick();
  }
  template<class Message>
  void observe(const std::string &name, bool reliable, bool transient = false) {
    rclcpp::QoS qos(1);
    if (reliable) qos.reliable(); else qos.best_effort();
    if (transient) qos.transient_local();
    rclcpp::SubscriptionOptions options; options.callback_group = group_;
    auto sub = create_subscription<Message>(name, qos, [](typename Message::ConstSharedPtr) {}, options);
    samplers_.push_back([this, sub, name](double m, double r, double w) {
      Message msg; rclcpp::MessageInfo info;
      if (!sub->take(msg, info)) return;
      auto &t = topics_.at(name);
      ++t.count; t.received = w; t.received_mono = m;
      if constexpr (!std::is_same_v<Message, std_msgs::msg::String>) {
        t.header = stamp(msg.header.stamp); t.frame = msg.header.frame_id;
      }
      if (name == "/local_state/odometry") watch_.observe(m, r, t.header);
      if constexpr (std::is_same_v<Message, robot_interfaces::msg::DockTargetObservation>) {
        t.sensor_healthy = msg.sensor_healthy; t.valid = msg.valid;
        t.source = msg.source; t.reason = msg.reason;
      }
    });
  }
  void refresh_graph(double m) {
    graph_checked_ = m;
    try {
      nodes_.clear();
      for (const auto &name : get_node_names()) nodes_.insert(name);
      for (auto &[name, t] : topics_) {
        t.publishers = static_cast<int>(count_publishers(name));
        t.subscribers = static_cast<int>(count_subscribers(name));
      }
      services_.clear();
      const auto services = get_service_names_and_types();
      for (const char *name : {"/global_localization/trigger", "/trigger_grid_search_localization",
        "/floor_manager/switch_floor", "/robot_localization_bridge/force_accept_next_localization"})
        services_[name] = services.count(name) > 0;
      auto has = [](const auto &infos, const char *name) {
        return std::any_of(infos.begin(), infos.end(), [name](const auto &i) { return i.node_name() == name; });
      };
      endpoints_["robot_local_state_node"] = nodes_.count("/robot_local_state") > 0;
      endpoints_["robot_local_state_odom_pub"] = has(get_publishers_info_by_topic("/local_state/odometry"), "robot_local_state");
      endpoints_["robot_local_state_fastlio_sub"] = has(get_subscriptions_info_by_topic("/fastlio/base_odometry"), "robot_local_state");
      endpoints_["robot_localization_bridge_node"] = nodes_.count("/robot_localization_bridge") > 0;
      endpoints_["robot_localization_bridge_tf_pub"] = has(get_publishers_info_by_topic("/tf"), "robot_localization_bridge");
      if (flatscan_) flatscan_->update_graph(m, true, count_publishers("/scan"),
        count_publishers("/flatscan"), count_publishers("/global_localization/flatscan_input_status"));
      if (amcl_ && amcl_->registered()) {
        robot_bringup::runtime_amcl::GraphObservation observation;
        observation.available = true;
        observation.node_exists = nodes_.count("/" + strip(amcl_->node_name())) > 0;
        observation.pose_publishers = count_publishers(amcl_->pose_topic());
        observation.scan_publishers = count_publishers(amcl_->scan_topic());
        observation.admission_status_publishers = count_publishers(amcl_->admission_status_topic());
        amcl_->observe_graph(amcl_->generation(), observation);
      }
    } catch (const std::exception &e) {
      nodes_.clear(); endpoints_.clear(); services_.clear();
      for (auto &[name, t] : topics_) { (void)name; t.publishers = t.subscribers = 0; }
      if (flatscan_) flatscan_->update_graph(m, false, 0, 0, 0);
      if (amcl_) amcl_->observe_graph(amcl_->generation(), {});
      std::cerr << "[runtime-health] graph query unavailable: " << e.what() << '\n';
    }
  }
  void write_snapshot() {
    const double m = monotonic_now(), r = now().seconds(), w = wall_now();
    rapidjson::Document d; d.SetObject(); auto &a = d.GetAllocator();
    if (flatscan_) put_json(d, "flatscan_monitor", flatscan_->snapshot(a), a);
    put(d, "schema", "njrh.runtime_health.v1", a); put(d, "implementation", "cpp", a);
    put(d, "updated_at", w, a); put(d, "updated_monotonic_sec", m, a);
    put(d, "boot_id", boot_id_, a); put(d, "generation", generation_, a);
    put(d, "sequence", ++sequence_, a); put(d, "uptime_sec", m - started_, a);
    put(d, "clock_valid", m >= clock_recover_until_, a);
    put(d, "sampling_delayed", m - sample_started_ >= sample_period_, a);
    put(d, "sample_period_sec", sample_period_, a);
    put(d, "message_count_semantics", "sampled_messages_not_publisher_rate", a);
    Json topics(rapidjson::kObjectType);
    auto age = [r, m](const Topic &t) { return std::isfinite(t.header) ? r - t.header : m - t.received_mono; };
    auto fresh = [this, &age](const std::string &name, double limit) {
      const auto &t = topics_.at(name); const double v = age(t);
      return t.publishers > 0 && std::isfinite(v) && v >= -0.25 && v <= limit;
    };
    for (const auto &[name, t] : topics_) {
      Json j(rapidjson::kObjectType);
      put(j, "type", t.type, a); put(j, "publishers", t.publishers, a); put(j, "subscriptions", t.subscribers, a);
      put(j, "last_received_at", t.received, a); put(j, "last_stamp_sec", t.header, a);
      put(j, "last_age_sec", age(t), a); put(j, "last_frame_id", t.frame, a); put(j, "message_count", t.count, a);
      if (name == "/dock/target_observation") {
        put(j, "sensor_healthy", t.sensor_healthy, a); put(j, "valid", t.valid, a);
        put(j, "source", t.source, a); put(j, "reason", t.reason, a);
      }
      put_json(topics, name.c_str(), std::move(j), a);
    }
    put_json(d, "topics", std::move(topics), a);
    Json watch(rapidjson::kObjectType);
    put(watch, "timeout_sec", timeout_, a); put(watch, "no_update_age_sec", watch_.no_update_age(m), a);
    put(watch, "seen_valid", watch_.seen_valid(), a); put(watch, "fault_candidate", watch_.fault(m), a);
    put_json(d, "odom_watch", std::move(watch), a);
    Json tracking(rapidjson::kObjectType), observed(rapidjson::kArrayType);
    observed.PushBack(value("/local_state/odometry", a), a); observed.PushBack(value("/dock/target_observation", a), a);
    put(tracking, "observe_topic_messages", messages_, a); put(tracking, "observe_heavy_topics", heavy_, a);
    put_json(tracking, "always_observed", std::move(observed), a); put_json(d, "topic_tracking", std::move(tracking), a);
    Json tf_tracking(rapidjson::kObjectType), tracked(rapidjson::kArrayType), tf(rapidjson::kObjectType);
    for (const auto &e : tracked_) tracked.PushBack(value(e, a), a);
    put(tf_tracking, "observe_tf", tf_, a); put(tf_tracking, "observe_all_tf", all_tf_, a);
    put_json(tf_tracking, "tracked_edges", std::move(tracked), a); put_json(d, "tf_tracking", std::move(tf_tracking), a);
    for (const auto &[name, e] : edges_) {
      Json j(rapidjson::kObjectType);
      put(j, "parent", e.parent, a); put(j, "child", e.child, a); put(j, "last_stamp_sec", e.header, a);
      put(j, "last_age_sec", r - e.header, a); put(j, "last_received_at", e.received, a);
      put_json(tf, name.c_str(), std::move(j), a);
    }
    put_json(d, "tf", std::move(tf), a);
    auto edge_fresh = [this, r](const char *name, double max_age) {
      auto it = edges_.find(name);
      return it != edges_.end() && r - it->second.header >= -0.25 && r - it->second.header <= max_age;
    };
    Json endpoints(rapidjson::kObjectType), services(rapidjson::kObjectType), nodes(rapidjson::kArrayType), summary(rapidjson::kObjectType);
    for (const auto &[name, present] : endpoints_) put(endpoints, name.c_str(), present, a);
    for (const auto &[name, present] : services_) put(services, name.c_str(), present, a);
    for (const auto &name : nodes_) nodes.PushBack(value(name, a), a);
    put_json(d, "endpoints", std::move(endpoints), a); put_json(d, "services", std::move(services), a);
    put_json(d, "nodes", std::move(nodes), a);
    const bool endpoint = endpoints_["robot_local_state_node"] && endpoints_["robot_local_state_odom_pub"];
    const bool odom_fresh = fresh("/local_state/odometry", setting("NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC", 0.75));
    const bool odom_tf = edge_fresh("odom->base_link", setting("NJRH_RUNTIME_HEALTH_TF_FRESH_SEC", 0.25));
    const bool dock_fresh = fresh("/dock/target_observation", setting("NJRH_RUNTIME_HEALTH_DOCKING_FRESH_SEC", 1.5));
    put(summary, "local_state_endpoint_ready", endpoint, a);
    put(summary, "local_state_fastlio_endpoint_ready", endpoint && endpoints_["robot_local_state_fastlio_sub"], a);
    put(summary, "local_odom_fresh", odom_fresh, a); put(summary, "local_state_topic_ready", endpoint && odom_fresh, a);
    put(summary, "local_state_ready", endpoint && odom_fresh && odom_tf, a); put(summary, "odom_base_tf_fresh", odom_tf, a);
    put(summary, "map_odom_tf_ready", edge_fresh("map->odom", setting("NJRH_RUNTIME_HEALTH_MAP_TF_FRESH_SEC", 1.0)), a);
    put(summary, "localization_bridge_endpoint_ready", endpoints_["robot_localization_bridge_node"] && endpoints_["robot_localization_bridge_tf_pub"] && services_["/robot_localization_bridge/force_accept_next_localization"], a);
    put(summary, "safety_status_fresh", fresh("/safety/status", setting("NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC", 1.5)), a);
    put(summary, "local_scan_fresh", fresh("/scan", setting("NJRH_RUNTIME_HEALTH_SCAN_FRESH_SEC", 1.0)), a);
    put(summary, "local_costmap_fresh", fresh("/local_costmap/costmap", setting("NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC", 1.5)), a);
    put(summary, "global_costmap_fresh", fresh("/global_costmap/costmap", setting("NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC", 1.5)), a);
    put(summary, "map_fresh", topics_["/map"].publishers > 0 && topics_["/map"].count > 0, a);
    put(summary, "docking_observation_fresh", dock_fresh, a);
    put(summary, "docking_sensor_healthy", dock_fresh && topics_["/dock/target_observation"].sensor_healthy, a);
    put(summary, "global_localization_trigger_service", services_["/global_localization/trigger"], a);
    put(summary, "isaac_grid_search_trigger_service", services_["/trigger_grid_search_localization"], a);
    put(summary, "floor_switch_service", services_["/floor_manager/switch_floor"], a);
    put_json(d, "summary", std::move(summary), a);
    const auto can_commit = [this] {
      const double m = monotonic_now();
      const double elapsed = m - previous_mono_;
      return m - sample_started_ < sample_period_ &&
        std::abs((wall_now() - previous_wall_) - elapsed) <= 0.5 &&
        std::abs((now().seconds() - previous_ros_) - elapsed) <= 0.5;
    };
    if (!atomic_write(output_, serialize(d) + "\n", can_commit)) {
      std::cerr << "[runtime-health] discarded delayed/clock-invalid snapshot; no new producer fault evidence\n";
    }
  }
  std::filesystem::path output_;
  double sample_period_, graph_period_, write_period_, timeout_;
  OdomWatch watch_;
  double started_, previous_wall_, previous_ros_, previous_mono_, sample_started_{0}, graph_checked_{0}, clock_recover_until_{0};
  bool heavy_, messages_, tf_, all_tf_;
  std::string boot_id_, generation_;
  uint64_t sequence_{0};
  std::set<std::string> tracked_, nodes_;
  std::map<std::string, Topic> topics_;
  std::map<std::string, Edge> edges_;
  std::map<std::string, bool> endpoints_, services_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::vector<std::function<void(double, double, double)>> samplers_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  std::unique_ptr<robot_bringup::RuntimeFlatScanMonitor> flatscan_;
  std::unique_ptr<robot_bringup::runtime_amcl::StatusServer> amcl_;
  uint64_t amcl_generation_{0};
  robot_bringup::runtime_amcl::ProcessIdentity amcl_identity_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr amcl_lifecycle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_;
};
}

int main(int argc, char **argv) {
  try {
    std::string output = env("NJRH_RUNTIME_HEALTH_FILE", "/tmp/njrh_runtime_health.json");
    bool once = false;
    double once_sec = 1.0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--output" && i + 1 < argc) output = argv[++i];
      else if (arg == "--once") once = true;
      else if (arg == "--once-spin-sec" && i + 1 < argc) once_sec = std::stod(argv[++i]);
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Guard>(output);
    std::cout << "[runtime-health] implementation=cpp sample_period_sec=" << node->period() << " output=" << output << std::endl;
    const double started = monotonic_now();
    while (rclcpp::ok()) {
      const auto next = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(node->period()));
      node->sample();
      if (once && monotonic_now() - started >= once_sec) break;
      node->wait_until(next);
    }
    node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[runtime-health] failed: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
