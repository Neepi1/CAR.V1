// Passive, bounded, serialized capture. No publishers, Action clients or motion services.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <sys/statvfs.h>
#include <thread>
#include <unistd.h>

namespace fs = std::filesystem;
using Steady = std::chrono::steady_clock;
static volatile std::sig_atomic_t signal_seen = 0;
static void signal_handler(int value) {signal_seen = value;}
static int64_t mono_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();}
static int64_t wall_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}

static std::string quote(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;}
    else {out << c;}
  }
  out << '"';
  return out.str();
}
static std::string csv(const std::string & value)
{
  std::string result = "\"";
  for (char c : value) {if (c == '"') {result += '"';} result += c;}
  return result + '"';
}
template<typename T> static std::string hex_gid(const T & data)
{
  std::ostringstream out;
  bool any = false;
  for (unsigned char c : data) {any |= c != 0; out << std::hex << std::setw(2) << std::setfill('0') << int(c);}
  return any ? out.str() : "";
}
static void write_json(const fs::path & path, const std::string & content)
{
  const auto temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::trunc);
  stream.exceptions(std::ios::badbit | std::ios::failbit);
  stream << content << '\n';
  stream.close();
  fs::rename(temporary, path);
}
static uint64_t directory_bytes(const fs::path & path)
{
  uint64_t total = 0;
  for (const auto & item : fs::recursive_directory_iterator(path)) {
    if (item.is_regular_file()) {total += item.file_size();}
  }
  return total;
}

struct Options
{
  fs::path output, manifest;
  double duration = 300., discovery = 3.;
  uint64_t disk = 512 * 1024 * 1024ULL, queue = 16 * 1024 * 1024ULL, free = 128 * 1024 * 1024ULL;
};
static Options options(int argc, char ** argv)
{
  Options out;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "--help") {
      std::cout << "navlite_raw_capture --output NEW_DIR --manifest JSON --duration SEC --disk-budget-mb MB --queue-mb MB [--discovery-sec SEC] [--min-free-mb MB]\n";
      std::exit(0);
    }
    if (i + 1 >= argc) {throw std::invalid_argument("missing argument for " + key);}
    std::string value = argv[++i];
    if (key == "--output") {out.output = fs::absolute(value); continue;}
    if (key == "--manifest") {out.manifest = fs::absolute(value); continue;}
    size_t consumed = 0;
    double number = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(number) || number < 0. || number > 1e8) {
      throw std::invalid_argument("invalid finite nonnegative number for " + key);
    }
    if (key == "--duration") {out.duration = number;}
    else if (key == "--discovery-sec") {out.discovery = number;}
    else if (key == "--disk-budget-mb") {out.disk = uint64_t(number * 1024 * 1024);}
    else if (key == "--queue-mb") {out.queue = uint64_t(number * 1024 * 1024);}
    else if (key == "--min-free-mb") {out.free = uint64_t(number * 1024 * 1024);}
    else {throw std::invalid_argument("unknown option " + key);}
  }
  if (out.output.empty() || out.manifest.empty() || out.duration <= 0 || !out.disk || !out.queue) {
    throw std::invalid_argument("output, manifest, positive duration and budgets required");
  }
  return out;
}

struct Topic
{
  std::string name, role, type, expected, cadence, missing, requested_qos, endpoints = "[]";
  std::vector<std::string> roles;
  std::string binding_origin, subscribers = "[]";
  std::vector<std::string> expected_publishers, expected_subscribers;
  bool critical = false, ever_stale = false;
  rclcpp::QoS qos{rclcpp::KeepLast(32)};
  std::shared_ptr<rclcpp::GenericSubscription> subscription;
  uint64_t received = 0, dropped = 0, last_seq = 0, incompatible = 0, rmw_lost = 0;
  int64_t first_mono = 0, last_mono = 0, max_gap_ns = 0;
  std::atomic<uint64_t> written{0};
};

// Humble's public GenericSubscription callback omits MessageInfo; its documented
// virtual event-dispatch method provides it. No rclcpp private state is accessed.
class IndexedSubscription final : public rclcpp::GenericSubscription
{
public:
  using Callback = std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;
  IndexedSubscription(rclcpp::Node & node, const Topic & topic,
    const rclcpp::SubscriptionOptions & opts, Callback callback)
  : rclcpp::GenericSubscription(node.get_node_base_interface().get(),
      rclcpp::get_typesupport_library(topic.type, "rosidl_typesupport_cpp"),
      topic.name, topic.type, topic.qos,
      [](std::shared_ptr<rclcpp::SerializedMessage>) {}, opts), callback_(std::move(callback)) {}
  void handle_serialized_message(const std::shared_ptr<rclcpp::SerializedMessage> & message,
    const rclcpp::MessageInfo & info) override {callback_(message, info);}
private:
  Callback callback_;
};

struct Sample
{
  size_t topic;
  uint64_t seq, ordinal;
  int64_t mono, wall, ros;
  rmw_message_info_t info;
  std::shared_ptr<rclcpp::SerializedMessage> message;
};

class Capture
{
public:
  explicit Capture(Options opts) : opts_(std::move(opts)), start_mono_(mono_ns())
  {
    const auto cfg = YAML::LoadFile(opts_.manifest.string());
    if (!cfg["topics"] || !cfg["topics"].IsSequence() || cfg["topics"].size() == 0 || cfg["topics"].size() > 64) {
      throw std::invalid_argument("manifest requires 1..64 explicitly selected topics");
    }
    std::set<std::string> names;
    for (const auto & row : cfg["topics"]) {
      auto t = std::make_unique<Topic>();
      t->name = row["name"].as<std::string>();
      if (t->name.empty() || t->name.front() != '/' || !names.insert(t->name).second) {
        throw std::invalid_argument("topics must be absolute and unique");
      }
      t->role = row["role"] ? row["role"].as<std::string>() : "unassigned";
      if (row["roles"]) {for (const auto & role : row["roles"]) {t->roles.push_back(role.as<std::string>());}}
      t->binding_origin = row["binding_origin"] ? row["binding_origin"].as<std::string>() : "";
      if (row["endpoint_expected"]) {
        const auto expected = row["endpoint_expected"];
        if (expected["publisher"]) {for (const auto & n : expected["publisher"]) {t->expected_publishers.push_back(n.as<std::string>());}}
        if (expected["subscriber"]) {for (const auto & n : expected["subscriber"]) {t->expected_subscribers.push_back(n.as<std::string>());}}
      }
      t->expected = row["expected_type"] ? row["expected_type"].as<std::string>() : "";
      t->critical = row["critical"] ? row["critical"].as<bool>() : false;
      t->cadence = row["cadence"] ? row["cadence"].as<std::string>() : "event";
      if (row["qos"]) {t->requested_qos = YAML::Dump(row["qos"]);}
      topics_.push_back(std::move(t));
    }
    if (fs::exists(opts_.output) && !fs::is_empty(opts_.output)) {
      throw std::invalid_argument("output must be new or empty; never overwrite evidence");
    }
    fs::create_directories(opts_.output);
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions node_opts;
    node_opts.context(context_).enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
    node_ = std::make_shared<rclcpp::Node>("navlite_raw_capture_" + std::to_string(getpid()), node_opts);
    rclcpp::ExecutorOptions exec_opts;
    exec_opts.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_opts);
    executor_->add_node(node_);
  }
  ~Capture()
  {
    finish_queue();
    if (worker_.joinable()) {worker_.join();}
    if (context_ && context_->is_valid()) {context_->shutdown("capture destruction");}
  }

  int run()
  {
    try {return run_impl();}
    catch (const std::exception & e) {
      finish_queue();
      if (worker_.joinable()) {worker_.join();}
      writer_error_ = std::string("capture_error: ") + e.what();
      failed_ = true; reason_ = "capture_error";
      if (writer_open_) {try {writer_.close(); writer_open_ = false;} catch (...) {}}
      return finalize();
    }
    catch (...) {
      finish_queue();
      if (worker_.joinable()) {worker_.join();}
      writer_error_ = "non_std_capture_error";
      failed_ = true; reason_ = "capture_error";
      if (writer_open_) {try {writer_.close(); writer_open_ = false;} catch (...) {}}
      return finalize();
    }
  }
  int run_impl()
  {
    health_.open(opts_.output / "health.jsonl", std::ios::out);
    health_.exceptions(std::ios::badbit | std::ios::failbit);
    const auto discovery_end = Steady::now() + std::chrono::duration<double>(opts_.discovery);
    while (!signal_seen && Steady::now() < discovery_end) {executor_->spin_once(std::chrono::milliseconds(50));}
    graph(true);
    if (signal_seen) {reason_ = "signal_" + std::to_string(signal_seen); return finalize();}
    if (check_disk()) {return finalize();}
    rosbag2_storage::StorageOptions storage;
    storage.uri = (opts_.output / "bag").string();
    storage.storage_id = "sqlite3";
    storage.max_cache_size = 0;  // one bounded queue, never a second hidden cache
    writer_.open(storage, rosbag2_cpp::ConverterOptions{"", ""});
    writer_open_ = true;
    index_.open(opts_.output / "receive_index.csv", std::ios::out);
    index_.exceptions(std::ios::badbit | std::ios::failbit);
    index_ << "seq,topic,topic_ordinal,bag_timestamp_ns,recv_monotonic_ns,recv_wall_ns,recv_ros_ns,publisher_gid,rmw_source_timestamp_ns,rmw_received_timestamp_ns,serialized_bytes,rmw_publication_sequence,rmw_reception_sequence\n";
    for (size_t i = 0; i < topics_.size(); ++i) {subscribe(i);}
    worker_ = std::thread([this] {write_loop();});
    record_start_ = mono_ns();
    int64_t next_health = record_start_, next_graph = record_start_ + 5000000000LL;
    const int64_t deadline = record_start_ + int64_t(opts_.duration * 1e9);
    while (!signal_seen && !failed_.load() && context_->is_valid() && mono_ns() < deadline) {
      executor_->spin_once(std::chrono::milliseconds(50));
      const auto now = mono_ns();
      if (now >= next_health) {
        next_health = now + 1000000000LL;
        health(false);
        if (check_disk()) {break;}
      }
      if (now >= next_graph) {next_graph = now + 5000000000LL; graph(false);}
    }
    if (signal_seen) {reason_ = "signal_" + std::to_string(signal_seen);}
    else if (failed_) {reason_ = "writer_error";}
    else if (!context_->is_valid()) {reason_ = "context_shutdown";}
    // No callback is running after the only executor's spin_once has returned.
    for (auto & t : topics_) {t->subscription.reset();}
    executor_->remove_node(node_);
    finish_queue();
    worker_.join();
    if (failed_) {reason_ = "writer_error";}
    graph(false);
    return finalize();
  }

private:
  void graph(bool initial)
  {
    bool changed = initial;
    for (auto & ptr : topics_) {
      auto & t = *ptr;
      const auto pubs = node_->get_publishers_info_by_topic(t.name);
      std::set<std::string> types;
      std::ostringstream endpoints;
      endpoints << '[';
      bool first = true, all_transient = !pubs.empty(), all_reliable = !pubs.empty();
      for (const auto & pub : pubs) {
        types.insert(pub.topic_type());
        const auto q = pub.qos_profile().get_rmw_qos_profile();
        all_transient &= q.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        all_reliable &= q.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        if (!first) {endpoints << ',';} first = false;
        const auto node_name = (pub.node_namespace() == "/" ? "/" : pub.node_namespace() + "/") + pub.node_name();
        endpoints << "{\"node\":" << quote(node_name)
                  << ",\"type\":" << quote(pub.topic_type()) << ",\"gid\":" << quote(hex_gid(pub.endpoint_gid()))
                  << ",\"reliability\":" << int(q.reliability) << ",\"durability\":" << int(q.durability)
                  << ",\"history\":" << int(q.history) << ",\"depth\":" << q.depth
                  << ",\"deadline_sec\":" << q.deadline.sec << ",\"deadline_nsec\":" << q.deadline.nsec
                  << ",\"lifespan_sec\":" << q.lifespan.sec << ",\"lifespan_nsec\":" << q.lifespan.nsec
                  << ",\"liveliness\":" << int(q.liveliness)
                  << ",\"liveliness_lease_sec\":" << q.liveliness_lease_duration.sec
                  << ",\"liveliness_lease_nsec\":" << q.liveliness_lease_duration.nsec << '}';
      }
      endpoints << ']';
      if (t.endpoints != endpoints.str()) {changed = true; t.endpoints = endpoints.str();}
      std::ostringstream subscribers;
      subscribers << '[';
      bool first_sub = true;
      for (const auto & sub : node_->get_subscriptions_info_by_topic(t.name)) {
        if (sub.node_name() == node_->get_name()) {continue;}
        if (!first_sub) {subscribers << ',';} first_sub = false;
        const auto q = sub.qos_profile().get_rmw_qos_profile();
        const auto node_name = (sub.node_namespace() == "/" ? "/" : sub.node_namespace() + "/") + sub.node_name();
        subscribers << "{\"node\":" << quote(node_name) << ",\"type\":" << quote(sub.topic_type())
                    << ",\"gid\":" << quote(hex_gid(sub.endpoint_gid()))
                    << ",\"reliability\":" << int(q.reliability) << ",\"durability\":" << int(q.durability)
                    << ",\"history\":" << int(q.history) << ",\"depth\":" << q.depth << '}';
      }
      subscribers << ']';
      if (t.subscribers != subscribers.str()) {changed = true; t.subscribers = subscribers.str();}
      if (!initial) {continue;}
      if (types.size() > 1) {t.missing = "ambiguous_types"; continue;}
      t.type = types.empty() ? t.expected : *types.begin();
      if (t.type.empty()) {t.missing = "type_unavailable"; continue;}
      if (!t.expected.empty() && t.expected != t.type) {t.missing = "expected_type_mismatch"; continue;}
      if (t.type == "sensor_msgs/msg/PointCloud2" || t.type == "sensor_msgs/msg/Image" ||
        t.type == "sensor_msgs/msg/CompressedImage" || t.type == "visualization_msgs/msg/MarkerArray") {
        t.missing = "large_topic_not_supported_by_lite_profile"; continue;
      }
      t.qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
      if (all_transient || t.name == "/tf_static") {t.qos.transient_local();}
      // In this installed Fast DDS, reliable retained publishers did not deliver
      // historical samples to a best-effort late joiner (isolated red test).
      if ((all_transient && all_reliable) || (pubs.empty() && t.name == "/tf_static")) {t.qos.reliable();}
      if (!t.requested_qos.empty()) {
        const auto q = YAML::Load(t.requested_qos);
        if (q["reliability"]) {
          const auto value = q["reliability"].as<std::string>();
          if (value == "reliable") {t.qos.reliable();}
          else if (value == "best_effort") {t.qos.best_effort();}
          else {throw std::invalid_argument("unknown reliability");}
        }
        if (q["durability"]) {
          const auto value = q["durability"].as<std::string>();
          if (value == "transient_local") {t.qos.transient_local();}
          else if (value == "volatile") {t.qos.durability_volatile();}
          else {throw std::invalid_argument("unknown durability");}
        }
        if (q["depth"]) {
          const auto depth = q["depth"].as<size_t>();
          if (!depth || depth > 1024) {throw std::invalid_argument("QoS depth must be 1..1024");}
          t.qos.keep_last(depth);
        }
      }
    }
    if (changed) {
      write_json(opts_.output / "topic_map.json", topic_map());
      health_ << "{\"kind\":\"graph_change\",\"monotonic_ns\":" << mono_ns() << ",\"map\":" << topic_map() << "}\n";
    }
  }
  std::string topic_map() const
  {
    std::ostringstream out;
    out << "{\"schema\":1,\"type_evidence\":\"graph_or_explicit_expected_type\",\"topics\":[";
    bool first = true;
    for (const auto & ptr : topics_) {
      const auto & t = *ptr;
      if (!first) {out << ',';} first = false;
      const auto q = t.qos.get_rmw_qos_profile();
      out << "{\"name\":" << quote(t.name) << ",\"role\":" << quote(t.role) << ",\"type\":" << quote(t.type)
          << ",\"binding_origin\":" << quote(t.binding_origin) << ",\"roles\":[";
      for (size_t i = 0; i < t.roles.size(); ++i) {if (i) {out << ',';} out << quote(t.roles[i]);}
      out << ']'
          << ",\"expected_type\":" << quote(t.expected) << ",\"critical\":" << (t.critical ? "true" : "false")
          << ",\"cadence\":" << quote(t.cadence) << ",\"missing_reason\":" << quote(t.missing)
          << ",\"publishers\":" << t.endpoints << ",\"subscribers\":" << t.subscribers
          << ",\"endpoint_expected\":{\"publisher\":[";
      for (size_t i = 0; i < t.expected_publishers.size(); ++i) {if (i) {out << ',';} out << quote(t.expected_publishers[i]);}
      out << "],\"subscriber\":[";
      for (size_t i = 0; i < t.expected_subscribers.size(); ++i) {if (i) {out << ',';} out << quote(t.expected_subscribers[i]);}
      out << "]},\"binding_check\":\"compare_expected_with_observed_endpoints_offline\""
          << ",\"subscription_qos\":{\"reliability\":" << int(q.reliability)
          << ",\"durability\":" << int(q.durability) << ",\"depth\":" << q.depth << "}}";
    }
    return out.str() + "]}";
  }
  void subscribe(size_t index)
  {
    auto & t = *topics_[index];
    if (!t.missing.empty()) {return;}
    try {
      rosbag2_storage::TopicMetadata meta;
      meta.name = t.name; meta.type = t.type; meta.serialization_format = "cdr";
      writer_.create_topic(meta);
      rclcpp::SubscriptionOptions opts;
      opts.use_default_callbacks = false;
      opts.event_callbacks.incompatible_qos_callback = [&t](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {t.incompatible += std::max(0, info.total_count_change);};
      opts.event_callbacks.message_lost_callback = [&t](rclcpp::QOSMessageLostInfo & info) {t.rmw_lost += info.total_count_change;};
      t.subscription = std::make_shared<IndexedSubscription>(*node_, t, opts,
        [this, index](std::shared_ptr<rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info) {
          receive(index, std::move(message), info);
        });
      node_->get_node_topics_interface()->add_subscription(t.subscription, nullptr);
    } catch (const std::exception & e) {t.missing = std::string("subscription_error: ") + e.what();}
  }
  void receive(size_t index, std::shared_ptr<rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info)
  {
    auto & t = *topics_[index];
    const auto now = mono_ns(), wall = wall_ns();
    ++seq_; ++t.received; t.last_seq = seq_;
    if (t.last_mono) {t.max_gap_ns = std::max(t.max_gap_ns, now - t.last_mono);}
    else {t.first_mono = now;}
    t.last_mono = now;
    const auto bytes = message->size();
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_done_ || bytes > opts_.queue || queue_bytes_ > opts_.queue - bytes || queue_.size() >= 8192) {
      ++t.dropped; ++dropped_; if (!first_drop_seq_) {first_drop_seq_ = seq_;} last_drop_seq_ = seq_; return;
    }
    queue_.push_back(Sample{index, seq_, t.received, now, wall, node_->now().nanoseconds(), info.get_rmw_message_info(), std::move(message)});
    queue_bytes_ += bytes;
    peak_bytes_ = std::max(peak_bytes_, queue_bytes_);
    peak_messages_ = std::max(peak_messages_, queue_.size());
    ready_.notify_one();
  }
  void finish_queue()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_done_ = true; ready_.notify_all();
  }
  void write_loop() noexcept
  {
    try {
      int64_t last_flush = mono_ns();
      while (true) {
        Sample sample;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          ready_.wait(lock, [this] {return queue_done_ || !queue_.empty();});
          if (queue_.empty()) {break;}
          sample = std::move(queue_.front()); queue_.pop_front();
          queue_bytes_ -= sample.message->size();
        }
        auto & t = *topics_[sample.topic];
        const auto bytes = sample.message->size();
        // Unique join key; wall/mono/source clocks remain separate in the index.
        const int64_t bag_stamp = std::max(last_bag_stamp_ + 1, sample.wall);
        last_bag_stamp_ = bag_stamp;
        writer_.write(sample.message, t.name, t.type, rclcpp::Time(bag_stamp, RCL_SYSTEM_TIME));
        index_ << sample.seq << ',' << csv(t.name) << ',' << sample.ordinal << ',' << bag_stamp << ','
               << sample.mono << ',' << sample.wall << ',' << sample.ros << ',' << hex_gid(sample.info.publisher_gid.data) << ',';
        if (sample.info.source_timestamp > 0) {index_ << sample.info.source_timestamp;}
        index_ << ',';
        if (sample.info.received_timestamp > 0) {index_ << sample.info.received_timestamp;}
        index_ << ',' << bytes << ',';
        if (sample.info.publication_sequence_number != RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {index_ << sample.info.publication_sequence_number;}
        index_ << ',';
        if (sample.info.reception_sequence_number != RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {index_ << sample.info.reception_sequence_number;}
        index_ << '\n';
        ++t.written; ++written_;
        if (mono_ns() - last_flush > 1000000000LL) {index_.flush(); last_flush = mono_ns();}
      }
      writer_.close(); writer_open_ = false; index_.close();
    } catch (const std::exception & e) {writer_error_ = e.what(); failed_ = true;}
    catch (...) {writer_error_ = "non_std_writer_exception"; failed_ = true;}
    if (writer_open_) {try {writer_.close(); writer_open_ = false;} catch (...) {failed_ = true;}}
  }
  bool check_disk()
  {
    struct statvfs info {};
    if (statvfs(opts_.output.c_str(), &info) != 0) {throw std::runtime_error("statvfs failed");}
    const auto used = directory_bytes(opts_.output);
    const uint64_t free = uint64_t(info.f_bavail) * info.f_frsize;
    // Queue drain + SQLite metadata headroom. Budget is sampled, not a filesystem quota.
    const uint64_t reserve = opts_.queue + 1024 * 1024;
    if (free < opts_.free + reserve) {reason_ = "disk_low"; return true;}
    if (used + reserve >= opts_.disk) {reason_ = "disk_budget"; return true;}
    return false;
  }
  std::string coverage(bool final)
  {
    const auto now = mono_ns();
    std::ostringstream out;
    out << '{'; bool first = true;
    bool missing = false;
    for (auto & ptr : topics_) {
      auto & t = *ptr;
      const auto age = t.last_mono ? double(now - t.last_mono) / 1e9 : -1.;
      const bool stale = t.cadence == "continuous" && record_start_ && now - record_start_ > 2000000000LL && (!t.last_mono || age > 2.);
      t.ever_stale |= stale;
      missing |= t.critical && (!t.received || !t.missing.empty() || t.ever_stale);
      if (!first) {out << ',';} first = false;
      out << quote(t.name) << ":{\"messages\":" << t.received << ",\"received\":" << t.received << ",\"written\":" << t.written.load()
          << ",\"dropped\":" << t.dropped << ",\"first_monotonic_ns\":" << t.first_mono
          << ",\"last_monotonic_ns\":" << t.last_mono << ",\"max_gap_ns\":" << t.max_gap_ns
          << ",\"last_age_sec\":" << (age < 0 ? "null" : std::to_string(age))
          << ",\"stale\":" << (stale ? "true" : "false") << ",\"ever_stale\":" << (t.ever_stale ? "true" : "false")
          << ",\"matched_publishers\":";
      if (t.subscription) {out << t.subscription->get_publisher_count();} else {out << "null";}
      out << ",\"incompatible_qos_events\":" << t.incompatible << ",\"rmw_reported_lost\":" << t.rmw_lost
          << ",\"dds_or_source_loss\":\"unknown\",\"missing_reason\":" << quote(t.missing) << '}';
    }
    if (final) {missing_ = missing;}
    current_missing_ = missing;
    out << '}'; return out.str();
  }
  std::string performance() const
  {
    struct rusage usage {};
    getrusage(RUSAGE_SELF, &usage);
    const double cpu = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 + usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    const double elapsed = double(mono_ns() - start_mono_) / 1e9;
    std::ostringstream out;
    out << "{\"elapsed_sec\":" << elapsed << ",\"cpu_sec\":" << cpu
        << ",\"cpu_percent_one_core\":" << 100 * cpu / std::max(.001, elapsed)
        << ",\"max_rss_kib\":" << usage.ru_maxrss << ",\"block_in\":" << usage.ru_inblock
        << ",\"block_out\":" << usage.ru_oublock << '}';
    return out.str();
  }
  void health(bool final)
  {
    const auto states = coverage(final);
    const int64_t now = mono_ns(), wall = wall_ns();
    if (last_health_mono_ && std::abs((wall - last_health_wall_) - (now - last_health_mono_)) > 200000000LL) {
      ++clock_jumps_;
      health_ << "{\"kind\":\"wall_clock_jump\",\"monotonic_ns\":" << now << ",\"wall_ns\":" << wall << "}\n";
    }
    last_health_mono_ = now; last_health_wall_ = wall;
    uint64_t queued_bytes; size_t queued_messages;
    {std::lock_guard<std::mutex> lock(queue_mutex_); queued_bytes = queue_bytes_; queued_messages = queue_.size();}
    health_ << "{\"kind\":\"health\",\"monotonic_ns\":" << now << ",\"wall_ns\":" << wall
            << ",\"topics\":" << states << ",\"performance\":" << performance()
            << ",\"queue_bytes\":" << queued_bytes << ",\"queue_messages\":" << queued_messages << "}\n";
    health_.flush();
    const std::string status = current_missing_ || dropped_ || failed_ ? "INCOMPLETE" : "READY";
    if (status != last_status_) {std::cout << status << " native_capture (recorder evidence only)\n" << std::flush; last_status_ = status;}
    save_quality(final);
  }
  bool save_quality(bool final)
  {
    const auto states = coverage(final);
    const bool incomplete = current_missing_ || dropped_ || failed_ || reason_ != "duration";
    std::vector<std::string> reasons;
    for (const auto & t : topics_) {
      if (t->critical && !t->received) {reasons.push_back("no_messages:" + t->name);}
      if (t->critical && !t->missing.empty()) {reasons.push_back(t->missing + ":" + t->name);}
      if (t->critical && t->ever_stale) {reasons.push_back("continuous_gap:" + t->name);}
    }
    if (dropped_) {reasons.push_back("recorder_queue_drops");}
    const bool failed = failed_.load();
    if (failed) {reasons.push_back("writer_or_capture_error");}
    if (reason_ != "duration") {reasons.push_back(reason_);}
    std::ostringstream out;
    out << "{\"schema\":1,\"status\":" << quote(final ? "finished" : "recording")
        << ",\"ready\":" << (incomplete ? "false" : "true") << ",\"incomplete_reasons\":[";
    for (size_t i = 0; i < reasons.size(); ++i) {if (i) {out << ',';} out << quote(reasons[i]);}
    out << "],\"reason\":" << quote(final ? reason_ : "recording") << ",\"incomplete\":" << (incomplete ? "true" : "false")
        << ",\"topics\":" << states << ",\"received\":" << seq_ << ",\"written\":" << written_.load()
        << ",\"dropped\":" << dropped_ << ",\"queue_drops\":" << dropped_ << ",\"first_drop_seq\":" << first_drop_seq_ << ",\"last_drop_seq\":" << last_drop_seq_
        << ",\"peak_queue_bytes\":" << peak_bytes_ << ",\"peak_queue_messages\":" << peak_messages_
        << ",\"writer_error\":" << quote(failed ? writer_error_ : "") << ",\"clock_jumps\":" << clock_jumps_
        << ",\"performance\":" << performance() << ",\"source_clock_verified\":false,\"source_headers\":\"decode_original_CDR_offline\""
        << ",\"bag_join\":\"topic+bag_timestamp_ns; key is unique synthetic wall-based storage time, not source time\""
        << ",\"dds_or_source_loss\":\"unknown\"}";
    write_json(opts_.output / "quality.json", out.str());
    return incomplete;
  }
  int finalize()
  {
    health(true);
    const bool incomplete = save_quality(true);
    write_json(opts_.output / "topic_map.json", topic_map());
    std::cout << "native_complete=" << opts_.output << " incomplete=" << incomplete << " reason=" << reason_ << '\n';
    return incomplete ? 3 : 0;
  }

  Options opts_;
  int64_t start_mono_, record_start_ = 0, last_bag_stamp_ = 0, last_health_mono_ = 0, last_health_wall_ = 0;
  std::vector<std::unique_ptr<Topic>> topics_;
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rosbag2_cpp::Writer writer_;
  std::ofstream index_, health_;
  std::deque<Sample> queue_;
  std::mutex queue_mutex_;
  std::condition_variable ready_;
  std::thread worker_;
  std::atomic<bool> failed_{false};
  std::atomic<uint64_t> written_{0};
  bool queue_done_ = false, writer_open_ = false, missing_ = false, current_missing_ = false;
  uint64_t queue_bytes_ = 0, peak_bytes_ = 0, seq_ = 0, dropped_ = 0, first_drop_seq_ = 0, last_drop_seq_ = 0, clock_jumps_ = 0;
  size_t peak_messages_ = 0;
  std::string reason_ = "duration", writer_error_, last_status_;
};

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
  try {Capture capture(options(argc, argv)); return capture.run();}
  catch (const std::exception & e) {std::cerr << "NATIVE_CAPTURE_ERROR " << e.what() << '\n'; return 2;}
  catch (...) {std::cerr << "NATIVE_CAPTURE_ERROR non_std_exception\n"; return 2;}
}
