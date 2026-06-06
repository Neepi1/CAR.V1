#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <iostream>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

double parse_double(const char * value, const char * name)
{
  char * end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid ") + name + ": " + value);
  }
  return parsed;
}

int parse_int(const char * value, const char * name)
{
  char * end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    throw std::runtime_error(std::string("invalid ") + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

rclcpp::QoS qos_profile(rmw_qos_reliability_policy_t reliability,
                        rmw_qos_durability_policy_t durability,
                        size_t depth = 1)
{
  rclcpp::QoS qos(depth);
  if (reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  if (durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  return qos;
}

std::vector<rclcpp::QoS> default_qos_profiles()
{
  return {
    qos_profile(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE),
    qos_profile(RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE),
  };
}

void spin_slice(const rclcpp::Node::SharedPtr & node)
{
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
}

std::string full_node_name(const std::string & name, const std::string & ns)
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
}

bool node_name_matches(const std::string & observed, const std::string & expected)
{
  if (observed == expected) {
    return true;
  }
  if (!expected.empty() && expected.front() == '/' && observed == expected.substr(1)) {
    return true;
  }
  if (!observed.empty() && observed.front() == '/' && observed.substr(1) == expected) {
    return true;
  }
  return false;
}

std::string first_topic_type(const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
  const auto topics = node->get_topic_names_and_types();
  const auto iter = topics.find(topic);
  if (iter == topics.end() || iter->second.empty()) {
    return {};
  }
  return iter->second.front();
}

std::string trim(std::string value)
{
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [&](char c) {
      return !is_space(static_cast<unsigned char>(c));
    }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [&](char c) {
      return !is_space(static_cast<unsigned char>(c));
    }).base(), value.end());
  if (value.size() >= 2 &&
    ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
  {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::string dirname_of(const std::string & path)
{
  const auto pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return ".";
  }
  if (pos == 0) {
    return path.substr(0, 1);
  }
  return path.substr(0, pos);
}

bool is_absolute_path(const std::string & path)
{
  return !path.empty() && (path.front() == '/' || path.find(':') != std::string::npos);
}

struct ExpectedMapInfo
{
  std::string image;
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

std::map<std::string, std::string> parse_simple_yaml_values(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open yaml");
  }
  std::map<std::string, std::string> values;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto key = trim(line.substr(0, colon));
    const auto value = trim(line.substr(colon + 1));
    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }
  return values;
}

std::string next_pgm_token(std::ifstream & input)
{
  std::string token;
  char c = '\0';
  while (input.get(c)) {
    if (c == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!token.empty()) {
        return token;
      }
      continue;
    }
    token.push_back(c);
  }
  throw std::runtime_error("unexpected pgm eof");
}

std::pair<int, int> pgm_size(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open pgm");
  }
  const auto magic = next_pgm_token(input);
  if (magic != "P2" && magic != "P5") {
    throw std::runtime_error("not a pgm");
  }
  const int width = std::stoi(next_pgm_token(input));
  const int height = std::stoi(next_pgm_token(input));
  return {width, height};
}

ExpectedMapInfo load_expected_map_info(const std::string & map_yaml)
{
  const auto values = parse_simple_yaml_values(map_yaml);
  const auto image_iter = values.find("image");
  const auto resolution_iter = values.find("resolution");
  const auto origin_iter = values.find("origin");
  if (image_iter == values.end() || resolution_iter == values.end() || origin_iter == values.end()) {
    throw std::runtime_error("missing image/resolution/origin");
  }

  ExpectedMapInfo info;
  info.image = image_iter->second;
  if (!is_absolute_path(info.image)) {
    info.image = dirname_of(map_yaml) + "/" + info.image;
  }
  info.resolution = std::stod(resolution_iter->second);

  std::regex number_re("[-+]?\\d+(?:\\.\\d+)?(?:[eE][-+]?\\d+)?");
  std::sregex_iterator iter(origin_iter->second.begin(), origin_iter->second.end(), number_re);
  std::sregex_iterator end;
  std::vector<double> origin_values;
  for (; iter != end; ++iter) {
    origin_values.push_back(std::stod(iter->str()));
  }
  if (origin_values.size() < 2) {
    throw std::runtime_error("invalid origin");
  }
  info.origin_x = origin_values[0];
  info.origin_y = origin_values[1];

  const auto size = pgm_size(info.image);
  info.width = size.first;
  info.height = size.second;
  return info;
}

bool wait_for_service(const rclcpp::Node::SharedPtr & node, const std::string & service_name,
                      double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    const auto services = node->get_service_names_and_types();
    const auto iter = services.find(service_name);
    if (iter != services.end() && !iter->second.empty()) {
      std::cerr << "[runtime-overlay] service ready: " << service_name << "\n";
      return true;
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for service: " << service_name << "\n";
  return false;
}

bool wait_for_node(const rclcpp::Node::SharedPtr & node, const std::string & expected_node,
                   double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    for (const auto & name : node->get_node_names()) {
      if (node_name_matches(name, expected_node)) {
        std::cerr << "[runtime-overlay] node ready: " << expected_node << "\n";
        return true;
      }
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for node: " << expected_node << "\n";
  return false;
}

bool wait_for_topic_publisher(const rclcpp::Node::SharedPtr & node, const std::string & topic,
                              double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    if (node->count_publishers(topic) > 0) {
      std::cerr << "[runtime-overlay] topic publisher ready: " << topic << "\n";
      return true;
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for topic publisher: " << topic << "\n";
  return false;
}

bool wait_for_publisher_from_node(const rclcpp::Node::SharedPtr & node, const std::string & topic,
                                  const std::string & node_name, double timeout_sec)
{
  const std::string expected = node_name.empty() || node_name.front() != '/' ? node_name.substr(0) :
    node_name.substr(1);
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    for (const auto & info : node->get_publishers_info_by_topic(topic)) {
      if (info.node_name() == expected) {
        std::cerr << "[runtime-overlay] publisher ready: " << expected << " on " << topic << "\n";
        return true;
      }
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for publisher " << expected << " on " << topic
            << "\n";
  return false;
}

bool wait_for_topic_message(const rclcpp::Node::SharedPtr & node, const std::string & topic,
                            double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::string topic_type;
  while (rclcpp::ok() && Clock::now() < deadline) {
    topic_type = first_topic_type(node, topic);
    if (!topic_type.empty()) {
      break;
    }
    spin_slice(node);
  }
  if (topic_type.empty()) {
    std::cerr << "[runtime-overlay] timed out waiting for topic type: " << topic << "\n";
    return false;
  }

  std::atomic_bool received{false};
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
  for (const auto & qos : default_qos_profiles()) {
    subscriptions.push_back(node->create_generic_subscription(
      topic, topic_type, qos,
      [&received](std::shared_ptr<rclcpp::SerializedMessage>) { received.store(true); }));
  }

  while (rclcpp::ok() && Clock::now() < deadline && !received.load()) {
    spin_slice(node);
  }

  if (!received.load()) {
    std::cerr << "[runtime-overlay] timed out waiting for topic message: " << topic << "\n";
    return false;
  }
  std::cerr << "[runtime-overlay] topic message ready: " << topic << "\n";
  return true;
}

template<typename MsgT>
bool wait_for_fresh_stamped_topic_typed(const rclcpp::Node::SharedPtr & node,
                                        const std::string & topic,
                                        double timeout_sec,
                                        double max_age_sec,
                                        double max_future_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::atomic_bool received{false};
  double last_age = 0.0;
  bool have_age = false;
  std::vector<typename rclcpp::Subscription<MsgT>::SharedPtr> subscriptions;

  auto on_message = [&](const typename MsgT::SharedPtr msg) {
      const rclcpp::Time stamp(msg->header.stamp);
      const double age = (node->get_clock()->now() - stamp).seconds();
      last_age = age;
      have_age = true;
      if (age <= max_age_sec && age >= -max_future_sec) {
        received.store(true);
      }
    };

  for (const auto & qos : default_qos_profiles()) {
    subscriptions.push_back(node->create_subscription<MsgT>(topic, qos, on_message));
  }

  while (rclcpp::ok() && Clock::now() < deadline && !received.load()) {
    spin_slice(node);
  }

  if (received.load()) {
    std::cerr << "[runtime-overlay] fresh stamped topic ready: " << topic
              << " age=" << last_age << "s\n";
    return true;
  }
  if (have_age) {
    std::cerr << "[runtime-overlay] timed out waiting for fresh stamped topic: " << topic
              << " last_age=" << last_age << "s allowed=[-" << max_future_sec << ","
              << max_age_sec << "]\n";
  } else {
    std::cerr << "[runtime-overlay] timed out waiting for fresh stamped topic: " << topic
              << " (no stamped message received)\n";
  }
  return false;
}

bool wait_for_fresh_stamped_topic(const rclcpp::Node::SharedPtr & node, const std::string & topic,
                                  double timeout_sec, double max_age_sec, double max_future_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::string topic_type;
  while (rclcpp::ok() && Clock::now() < deadline) {
    topic_type = first_topic_type(node, topic);
    if (!topic_type.empty()) {
      break;
    }
    spin_slice(node);
  }

  if (topic_type == "sensor_msgs/msg/PointCloud2") {
    return wait_for_fresh_stamped_topic_typed<sensor_msgs::msg::PointCloud2>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }
  if (topic_type == "sensor_msgs/msg/LaserScan") {
    return wait_for_fresh_stamped_topic_typed<sensor_msgs::msg::LaserScan>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }
  if (topic_type == "sensor_msgs/msg/Imu") {
    return wait_for_fresh_stamped_topic_typed<sensor_msgs::msg::Imu>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }
  if (topic_type == "nav_msgs/msg/Odometry") {
    return wait_for_fresh_stamped_topic_typed<nav_msgs::msg::Odometry>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }
  if (topic_type == "nav_msgs/msg/OccupancyGrid") {
    return wait_for_fresh_stamped_topic_typed<nav_msgs::msg::OccupancyGrid>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }
  if (topic_type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
    return wait_for_fresh_stamped_topic_typed<geometry_msgs::msg::PoseWithCovarianceStamped>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }

  std::cerr << "[runtime-overlay] unsupported stamped topic type for freshness check: "
            << topic << " type=" << (topic_type.empty() ? "<unknown>" : topic_type) << "\n";
  return false;
}

bool wait_for_tf(const rclcpp::Node::SharedPtr & node, const std::string & target,
                 const std::string & source, double timeout_sec)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, false);
  (void)listener;
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::string last_error;

  while (rclcpp::ok() && Clock::now() < deadline) {
    spin_slice(node);
    try {
      buffer->lookupTransform(target, source, tf2::TimePointZero);
      std::cerr << "[runtime-overlay] TF ready: " << target << "->" << source << "\n";
      return true;
    } catch (const tf2::TransformException & exc) {
      last_error = exc.what();
    }
  }

  std::cerr << "[runtime-overlay] timed out waiting for TF " << target << "->" << source;
  if (!last_error.empty()) {
    std::cerr << ": " << last_error;
  }
  std::cerr << "\n";
  return false;
}

bool wait_for_fresh_tf(const rclcpp::Node::SharedPtr & node, const std::string & target,
                       const std::string & source, double timeout_sec, double max_age_sec)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, false);
  (void)listener;

  std::atomic_bool direct_ready{false};
  double direct_last_age = 0.0;
  auto on_tf = [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      const auto now = node->get_clock()->now();
      for (const auto & transform : msg->transforms) {
        if (transform.header.frame_id != target || transform.child_frame_id != source) {
          continue;
        }
        const double age = (now - rclcpp::Time(transform.header.stamp)).seconds();
        direct_last_age = age;
        if (age <= max_age_sec) {
          direct_ready.store(true);
        }
      }
    };
  auto sub_best_effort = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", qos_profile(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE,
    100), on_tf);
  auto sub_reliable = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", qos_profile(RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE,
    100), on_tf);

  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::string last_error;
  double last_age = 0.0;
  bool have_age = false;

  while (rclcpp::ok() && Clock::now() < deadline) {
    spin_slice(node);
    if (direct_ready.load()) {
      std::cerr << "[runtime-overlay] fresh TF ready: " << target << "->" << source
                << " age=" << direct_last_age << "s source=/tf\n";
      return true;
    }
    try {
      const auto transform = buffer->lookupTransform(
        target, source, tf2::TimePointZero);
      const double age =
        (node->get_clock()->now() - rclcpp::Time(transform.header.stamp)).seconds();
      last_age = age;
      have_age = true;
      if (age <= max_age_sec) {
        std::cerr << "[runtime-overlay] fresh TF ready: " << target << "->" << source
                  << " age=" << age << "s source=tf2_buffer\n";
        return true;
      }
      last_error = "stale transform";
    } catch (const tf2::TransformException & exc) {
      last_error = exc.what();
    }
  }

  std::cerr << "[runtime-overlay] timed out waiting for fresh TF " << target << "->" << source;
  if (have_age) {
    std::cerr << ": age=" << last_age << "s max=" << max_age_sec << "s";
  } else if (!last_error.empty()) {
    std::cerr << ": " << last_error;
  }
  std::cerr << "\n";
  return false;
}

bool wait_for_transformable_obstacle_points(const rclcpp::Node::SharedPtr & node,
                                            double timeout_sec, int required_good)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node, false);
  (void)listener;

  const char * warmup_env = std::getenv("NJRH_LOCAL_COSTMAP_TF_BUFFER_WARMUP_SEC");
  const double requested_warmup = warmup_env ? std::max(0.0, std::atof(warmup_env)) : 1.5;
  const double warmup_sec = std::min(requested_warmup, timeout_sec * 0.6);
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  const auto warmup_deadline = Clock::now() + std::chrono::duration<double>(warmup_sec);

  while (rclcpp::ok() && Clock::now() < warmup_deadline) {
    spin_slice(node);
  }

  int seen = 0;
  int good = 0;
  std::string last_error;
  auto on_cloud = [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      ++seen;
      if (msg->header.frame_id.empty()) {
        last_error = "empty cloud frame_id";
        return;
      }
      try {
        buffer->lookupTransform(
          "odom", msg->header.frame_id, rclcpp::Time(msg->header.stamp));
        ++good;
      } catch (const tf2::TransformException & exc) {
        last_error = exc.what();
      }
    };

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/obstacle_points",
    qos_profile(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE),
    on_cloud);

  while (rclcpp::ok() && Clock::now() < deadline && good < required_good) {
    spin_slice(node);
  }

  if (good >= required_good) {
    std::cerr << "[runtime-overlay] transformable obstacle observations ready: "
              << good << "/" << seen << " fresh clouds are TF-valid "
              << "(tf_buffer_warmup=" << warmup_sec << "s)\n";
    return true;
  }

  std::cerr << "[runtime-overlay] transformable obstacle observations not ready: good="
            << good << " seen=" << seen << " tf_buffer_warmup=" << warmup_sec
            << "s last_error=" << (last_error.empty() ? "none" : last_error) << "\n";
  return false;
}

bool wait_for_local_state_endpoint(const rclcpp::Node::SharedPtr & node, double timeout_sec,
                                   const std::string & mode)
{
  const bool require_fastlio_sub = mode == "fastlio";
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    bool has_node = false;
    for (const auto & name : node->get_node_names()) {
      if (node_name_matches(name, "/robot_local_state")) {
        has_node = true;
        break;
      }
    }
    bool has_odom_pub = false;
    for (const auto & info : node->get_publishers_info_by_topic("/local_state/odometry")) {
      if (info.node_name() == "robot_local_state") {
        has_odom_pub = true;
        break;
      }
    }
    bool has_fastlio_sub = false;
    for (const auto & info : node->get_subscriptions_info_by_topic("/fastlio/base_odometry")) {
      if (info.node_name() == "robot_local_state") {
        has_fastlio_sub = true;
        break;
      }
    }
    if (has_node && has_odom_pub && (has_fastlio_sub || !require_fastlio_sub)) {
      std::cerr << "[runtime-overlay] robot_local_state endpoint ready";
      if (require_fastlio_sub) {
        std::cerr << " (fastlio)";
      }
      std::cerr << "\n";
      return true;
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] robot_local_state endpoint not ready: expected "
            << "/robot_local_state node, /local_state/odometry publisher";
  if (require_fastlio_sub) {
    std::cerr << ", /fastlio/base_odometry subscription";
  }
  std::cerr << "\n";
  return false;
}

bool wait_for_lifecycle_active(const rclcpp::Node::SharedPtr & node,
                               const std::string & lifecycle_node,
                               double timeout_sec)
{
  std::string service_node = lifecycle_node;
  while (!service_node.empty() && service_node.back() == '/') {
    service_node.pop_back();
  }
  const std::string service_name = service_node + "/get_state";
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(service_name);
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::string last_state = "unavailable";

  while (rclcpp::ok() && Clock::now() < deadline) {
    if (!client->wait_for_service(200ms)) {
      spin_slice(node);
      continue;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(node, future, 800ms);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      continue;
    }
    const auto response = future.get();
    last_state = response->current_state.label + " [" +
      std::to_string(response->current_state.id) + "]";
    if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
      response->current_state.label == "active")
    {
      std::cerr << "[runtime-overlay] lifecycle node active: " << lifecycle_node << "\n";
      return true;
    }
  }

  std::cerr << "[runtime-overlay] lifecycle node not active: " << lifecycle_node
            << " state=" << last_state << "\n";
  return false;
}

bool wait_for_occupancy_grid(const rclcpp::Node::SharedPtr & node, const std::string & topic,
                             double timeout_sec, int min_width, int min_height)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::atomic_bool received{false};
  nav_msgs::msg::OccupancyGrid last_msg;
  bool have_msg = false;

  auto on_msg = [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      have_msg = true;
      last_msg = *msg;
      if (static_cast<int>(msg->info.width) >= min_width &&
        static_cast<int>(msg->info.height) >= min_height)
      {
        received.store(true);
      }
    };
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic,
    qos_profile(RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    on_msg);

  while (rclcpp::ok() && Clock::now() < deadline && !received.load()) {
    spin_slice(node);
  }

  if (received.load()) {
    std::cerr << "[runtime-overlay] " << topic << " ready: "
              << last_msg.info.width << "x" << last_msg.info.height << " @ "
              << last_msg.info.resolution << ", origin=("
              << last_msg.info.origin.position.x << ", "
              << last_msg.info.origin.position.y << ")\n";
    return true;
  }

  if (have_msg) {
    std::cerr << "[runtime-overlay] timed out waiting for " << topic
              << " OccupancyGrid >= " << min_width << "x" << min_height
              << " last=" << last_msg.info.width << "x" << last_msg.info.height << "\n";
  } else {
    std::cerr << "[runtime-overlay] timed out waiting for " << topic << " OccupancyGrid\n";
  }
  return false;
}

bool wait_for_map_topic_matches_yaml(const rclcpp::Node::SharedPtr & node,
                                     const std::string & map_yaml,
                                     double timeout_sec)
{
  ExpectedMapInfo expected;
  try {
    expected = load_expected_map_info(map_yaml);
  } catch (const std::exception & exc) {
    std::cerr << "[runtime-overlay] failed to inspect requested map yaml " << map_yaml
              << ": " << exc.what() << "\n";
    return false;
  }

  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::atomic_bool matched{false};
  bool have_msg = false;
  nav_msgs::msg::OccupancyGrid last_msg;

  auto on_msg = [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      have_msg = true;
      last_msg = *msg;
      const auto & info = msg->info;
      const auto & origin = info.origin.position;
      const bool matches =
        static_cast<int>(info.width) == expected.width &&
        static_cast<int>(info.height) == expected.height &&
        std::abs(static_cast<double>(info.resolution) - expected.resolution) <= 1e-6 &&
        std::abs(origin.x - expected.origin_x) <= 1e-3 &&
        std::abs(origin.y - expected.origin_y) <= 1e-3;
      if (matches) {
        matched.store(true);
      }
    };
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    qos_profile(RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    on_msg);

  while (rclcpp::ok() && Clock::now() < deadline && !matched.load()) {
    spin_slice(node);
  }

  if (matched.load()) {
    std::cerr << "[runtime-overlay] /map matches requested map yaml: " << map_yaml << "\n";
    return true;
  }
  if (have_msg) {
    std::cerr << "[runtime-overlay] /map does not match requested map yaml: " << map_yaml
              << " expected=" << expected.width << "x" << expected.height
              << " got=" << last_msg.info.width << "x" << last_msg.info.height << "\n";
  }
  return false;
}

void print_usage()
{
  std::cerr
    << "usage: runtime_readiness_probe <command> [args]\n"
    << "commands:\n"
    << "  service <service_name> <timeout_sec>\n"
    << "  node <absolute_node_name> <timeout_sec>\n"
    << "  topic-publisher <topic> <timeout_sec>\n"
    << "  publisher-from-node <topic> <node_name> <timeout_sec>\n"
    << "  topic <topic> <timeout_sec>\n"
    << "  fresh-header-topic <topic> <timeout_sec> <max_age_sec> <max_future_sec>\n"
    << "  tf <target_frame> <source_frame> <timeout_sec>\n"
    << "  fresh-tf <target_frame> <source_frame> <timeout_sec> <max_age_sec>\n"
    << "  transformable-obstacle-points <timeout_sec> <required_good>\n"
    << "  local-state-endpoint <timeout_sec> <mode>\n"
    << "  lifecycle-active <node_name> <timeout_sec>\n"
    << "  occupancy-grid <topic> <timeout_sec> <min_width> <min_height>\n"
    << "  map-topic-matches-yaml <map_yaml> <timeout_sec>\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    print_usage();
    return 2;
  }

  const std::string command = argv[1];
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("runtime_readiness_probe");
    bool ok = false;

    if (command == "service" && argc == 4) {
      ok = wait_for_service(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "node" && argc == 4) {
      ok = wait_for_node(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "topic-publisher" && argc == 4) {
      ok = wait_for_topic_publisher(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "publisher-from-node" && argc == 5) {
      ok = wait_for_publisher_from_node(node, argv[2], argv[3], parse_double(argv[4],
        "timeout_sec"));
    } else if (command == "topic" && argc == 4) {
      ok = wait_for_topic_message(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "fresh-header-topic" && argc == 6) {
      ok = wait_for_fresh_stamped_topic(
        node, argv[2], parse_double(argv[3], "timeout_sec"),
        parse_double(argv[4], "max_age_sec"), parse_double(argv[5], "max_future_sec"));
    } else if (command == "tf" && argc == 5) {
      ok = wait_for_tf(node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"));
    } else if (command == "fresh-tf" && argc == 6) {
      ok = wait_for_fresh_tf(
        node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"),
        parse_double(argv[5], "max_age_sec"));
    } else if (command == "transformable-obstacle-points" && argc == 4) {
      ok = wait_for_transformable_obstacle_points(
        node, parse_double(argv[2], "timeout_sec"), std::max(1, parse_int(argv[3],
        "required_good")));
    } else if (command == "local-state-endpoint" && argc == 4) {
      std::string mode = argv[3];
      std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
      ok = wait_for_local_state_endpoint(node, parse_double(argv[2], "timeout_sec"), mode);
    } else if (command == "lifecycle-active" && argc == 4) {
      ok = wait_for_lifecycle_active(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "occupancy-grid" && argc == 6) {
      ok = wait_for_occupancy_grid(
        node, argv[2], parse_double(argv[3], "timeout_sec"),
        parse_int(argv[4], "min_width"), parse_int(argv[5], "min_height"));
    } else if (command == "map-topic-matches-yaml" && argc == 4) {
      ok = wait_for_map_topic_matches_yaml(node, argv[2], parse_double(argv[3],
        "timeout_sec"));
    } else {
      print_usage();
      rclcpp::shutdown();
      return 2;
    }

    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception & exc) {
    std::cerr << "[runtime-overlay] runtime_readiness_probe error: " << exc.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
