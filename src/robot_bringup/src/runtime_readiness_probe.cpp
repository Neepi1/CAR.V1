#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
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
#include "robot_interfaces/msg/dock_target_observation.hpp"
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

bool wait_for_topic_publisher_count(
  const rclcpp::Node::SharedPtr & node,
  const std::string & topic,
  std::size_t expected_count,
  double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    const auto actual_count = node->count_publishers(topic);
    if (actual_count == expected_count) {
      std::cerr << "[runtime-overlay] exact publisher count ready: " << topic
                << " count=" << actual_count << "\n";
      return true;
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for exact publisher count: " << topic
            << " expected=" << expected_count
            << " actual=" << node->count_publishers(topic) << "\n";
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

bool wait_for_exact_publisher_owner(
  const rclcpp::Node::SharedPtr & node,
  const std::string & topic,
  const std::string & node_name,
  std::size_t expected_count,
  double timeout_sec)
{
  const std::string expected = node_name.empty() || node_name.front() != '/' ? node_name :
    node_name.substr(1);
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && Clock::now() < deadline) {
    const auto publishers = node->get_publishers_info_by_topic(topic);
    const auto owner_count = static_cast<std::size_t>(std::count_if(
      publishers.begin(), publishers.end(), [&expected](const auto & info) {
        return info.node_name() == expected;
      }));
    if (publishers.size() == expected_count && owner_count == expected_count) {
      std::cerr << "[runtime-overlay] exact publisher owner ready: " << topic
                << " owner=" << expected << " count=" << expected_count << "\n";
      return true;
    }
    spin_slice(node);
  }
  std::cerr << "[runtime-overlay] timed out waiting for exact publisher owner: " << topic
            << " owner=" << expected << " expected_count=" << expected_count;
  const auto publishers = node->get_publishers_info_by_topic(topic);
  std::cerr << " actual_count=" << publishers.size() << " actual_owners=[";
  for (std::size_t index = 0; index < publishers.size(); ++index) {
    if (index != 0U) {
      std::cerr << ",";
    }
    const auto & publisher = publishers[index];
    std::cerr << publisher.node_namespace();
    if (publisher.node_namespace().empty() || publisher.node_namespace().back() != '/') {
      std::cerr << "/";
    }
    std::cerr << publisher.node_name();
  }
  std::cerr << "]\n";
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
  if (topic_type == "robot_interfaces/msg/DockTargetObservation") {
    return wait_for_fresh_stamped_topic_typed<robot_interfaces::msg::DockTargetObservation>(
      node, topic, std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count()),
      max_age_sec, max_future_sec);
  }

  std::cerr << "[runtime-overlay] unsupported stamped topic type for freshness check: "
            << topic << " type=" << (topic_type.empty() ? "<unknown>" : topic_type) << "\n";
  return false;
}

bool wait_for_ranger_chassis(const rclcpp::Node::SharedPtr & node, double timeout_sec,
                             double odom_max_age_sec, double odom_max_future_sec)
{
  constexpr const char * kRangerNode = "ranger_base_node";
  constexpr const char * kWheelOdomTopic = "/wheel/odom";
  constexpr const char * kMotionStateTopic = "/motion_state";
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);

  bool wheel_publisher_ready = false;
  bool motion_publisher_ready = false;
  bool have_odom_age = false;
  double last_odom_age = 0.0;
  std::atomic_bool fresh_wheel_odom_received{false};
  std::atomic_bool motion_state_received{false};
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions;
  std::vector<rclcpp::GenericSubscription::SharedPtr> motion_subscriptions;

  auto on_odom = [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const rclcpp::Time stamp(msg->header.stamp);
      last_odom_age = (node->get_clock()->now() - stamp).seconds();
      have_odom_age = true;
      if (last_odom_age <= odom_max_age_sec && last_odom_age >= -odom_max_future_sec) {
        fresh_wheel_odom_received.store(true);
      }
    };
  for (const auto & qos : default_qos_profiles()) {
    odom_subscriptions.push_back(
      node->create_subscription<nav_msgs::msg::Odometry>(kWheelOdomTopic, qos, on_odom));
  }

  auto publisher_is_ready = [&](const char * topic) {
      for (const auto & info : node->get_publishers_info_by_topic(topic)) {
        if (info.node_name() == kRangerNode) {
          return true;
        }
      }
      return false;
    };

  while (rclcpp::ok() && Clock::now() < deadline) {
    wheel_publisher_ready = wheel_publisher_ready || publisher_is_ready(kWheelOdomTopic);
    motion_publisher_ready = motion_publisher_ready || publisher_is_ready(kMotionStateTopic);

    if (motion_subscriptions.empty()) {
      const auto motion_type = first_topic_type(node, kMotionStateTopic);
      if (!motion_type.empty()) {
        for (const auto & qos : default_qos_profiles()) {
          motion_subscriptions.push_back(node->create_generic_subscription(
            kMotionStateTopic, motion_type, qos,
            [&motion_state_received](std::shared_ptr<rclcpp::SerializedMessage>) {
              motion_state_received.store(true);
            }));
        }
      }
    }

    spin_slice(node);
    if (wheel_publisher_ready && fresh_wheel_odom_received.load() && motion_publisher_ready &&
      motion_state_received.load())
    {
      std::cerr << "[runtime-overlay] ranger chassis ready: publisher(" << kWheelOdomTopic
                << ")=" << kRangerNode << " fresh_odom_age=" << last_odom_age
                << "s publisher(" << kMotionStateTopic << ")=" << kRangerNode
                << " motion_message=true\n";
      return true;
    }
  }

  std::cerr << "[runtime-overlay] timed out waiting for ranger chassis: wheel_publisher="
            << wheel_publisher_ready << " wheel_odom_fresh=" << fresh_wheel_odom_received.load()
            << " motion_publisher=" << motion_publisher_ready
            << " motion_message=" << motion_state_received.load();
  if (have_odom_age) {
    std::cerr << " last_odom_age=" << last_odom_age << "s allowed=[-" << odom_max_future_sec
              << "," << odom_max_age_sec << "]";
  } else {
    std::cerr << " last_odom_age=<none>";
  }
  std::cerr << "\n";
  return false;
}

bool wait_for_imu_bias_filter(const rclcpp::Node::SharedPtr & node,
                              const std::string & corrected_imu_topic,
                              const std::string & bias_topic,
                              double timeout_sec)
{
  constexpr const char * kBiasFilterNode = "imu_gyro_bias_filter";
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  bool corrected_publisher_ready = false;
  bool bias_publisher_ready = false;
  std::atomic_bool corrected_message_received{false};
  std::atomic_bool bias_message_received{false};
  std::vector<rclcpp::GenericSubscription::SharedPtr> corrected_subscriptions;
  std::vector<rclcpp::GenericSubscription::SharedPtr> bias_subscriptions;

  auto publisher_is_ready = [&](const std::string & topic) {
      for (const auto & info : node->get_publishers_info_by_topic(topic)) {
        if (info.node_name() == kBiasFilterNode) {
          return true;
        }
      }
      return false;
    };
  auto ensure_subscriptions = [&](const std::string & topic,
                                  std::vector<rclcpp::GenericSubscription::SharedPtr> & subscriptions,
                                  std::atomic_bool & received) {
      if (!subscriptions.empty()) {
        return;
      }
      const auto topic_type = first_topic_type(node, topic);
      if (topic_type.empty()) {
        return;
      }
      for (const auto & qos : default_qos_profiles()) {
        subscriptions.push_back(node->create_generic_subscription(
          topic, topic_type, qos,
          [&received](std::shared_ptr<rclcpp::SerializedMessage>) { received.store(true); }));
      }
    };

  while (rclcpp::ok() && Clock::now() < deadline) {
    corrected_publisher_ready = corrected_publisher_ready ||
      publisher_is_ready(corrected_imu_topic);
    bias_publisher_ready = bias_publisher_ready || publisher_is_ready(bias_topic);
    ensure_subscriptions(
      corrected_imu_topic, corrected_subscriptions, corrected_message_received);
    ensure_subscriptions(bias_topic, bias_subscriptions, bias_message_received);
    spin_slice(node);

    if (corrected_publisher_ready && corrected_message_received.load() &&
      bias_publisher_ready && bias_message_received.load())
    {
      std::cerr << "[runtime-overlay] IMU bias filter ready: publisher("
                << corrected_imu_topic << ")=" << kBiasFilterNode
                << " corrected_message=true publisher(" << bias_topic << ")="
                << kBiasFilterNode << " bias_message=true\n";
      return true;
    }
  }

  std::cerr << "[runtime-overlay] timed out waiting for IMU bias filter: corrected_publisher="
            << corrected_publisher_ready
            << " corrected_message=" << corrected_message_received.load()
            << " bias_publisher=" << bias_publisher_ready
            << " bias_message=" << bias_message_received.load() << "\n";
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

bool wait_for_stable_local_state(const rclcpp::Node::SharedPtr & node,
                                 double timeout_sec,
                                 int required_consecutive_good,
                                 double odom_max_age_sec,
                                 double odom_max_future_sec,
                                 double tf_max_age_sec)
{
  constexpr const char * kOdomTopic = "/local_state/odometry";
  constexpr const char * kTfTopic = "/tf";
  constexpr const char * kOdomFrame = "odom";
  constexpr const char * kBaseFrame = "base_link";

  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  const int required_good = std::max(1, required_consecutive_good);
  int consecutive_good = 0;
  int accepted_samples = 0;
  bool have_odom = false;
  bool have_tf = false;
  bool odom_valid = false;
  bool tf_valid = false;
  std::int64_t latest_odom_stamp_ns = 0;
  std::int64_t latest_tf_stamp_ns = 0;
  std::int64_t last_accepted_odom_stamp_ns = 0;
  std::int64_t last_accepted_tf_stamp_ns = 0;
  double last_odom_age = 0.0;
  double last_tf_age = 0.0;
  std::string last_error{"waiting for local_state observations"};

  auto on_odom = [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const rclcpp::Time odom_stamp(msg->header.stamp);
      const std::int64_t odom_stamp_ns = odom_stamp.nanoseconds();
      if (have_odom && odom_stamp_ns <= latest_odom_stamp_ns) {
        return;
      }
      have_odom = true;
      latest_odom_stamp_ns = odom_stamp_ns;
      last_odom_age = (node->get_clock()->now() - odom_stamp).seconds();

      if (msg->header.frame_id != kOdomFrame || msg->child_frame_id != kBaseFrame) {
        odom_valid = false;
        consecutive_good = 0;
        last_error = "local_state odometry frame contract mismatch";
        return;
      }
      if (last_odom_age > odom_max_age_sec || last_odom_age < -odom_max_future_sec) {
        odom_valid = false;
        consecutive_good = 0;
        last_error = "local_state odometry timestamp is stale or future-dated";
        return;
      }
      odom_valid = true;
    };

  auto on_tf = [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      for (const auto & transform : msg->transforms) {
        if (transform.header.frame_id != kOdomFrame ||
          transform.child_frame_id != kBaseFrame)
        {
          continue;
        }
        const rclcpp::Time tf_stamp(transform.header.stamp);
        const std::int64_t tf_stamp_ns = tf_stamp.nanoseconds();
        if (have_tf && tf_stamp_ns <= latest_tf_stamp_ns) {
          continue;
        }
        have_tf = true;
        latest_tf_stamp_ns = tf_stamp_ns;
        last_tf_age = (node->get_clock()->now() - tf_stamp).seconds();
        if (last_tf_age > tf_max_age_sec || last_tf_age < -odom_max_future_sec) {
          tf_valid = false;
          consecutive_good = 0;
          last_error = "odom->base_link timestamp is stale or future-dated";
          continue;
        }
        tf_valid = true;
      }
    };

  const auto observation_qos = qos_profile(
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    10);
  auto odom_subscription =
    node->create_subscription<nav_msgs::msg::Odometry>(
    kOdomTopic, observation_qos, on_odom);
  auto tf_subscription =
    node->create_subscription<tf2_msgs::msg::TFMessage>(
    kTfTopic, observation_qos, on_tf);
  (void)odom_subscription;
  (void)tf_subscription;

  while (rclcpp::ok() && Clock::now() < deadline && consecutive_good < required_good) {
    spin_slice(node);
    const auto now = node->get_clock()->now();
    if (have_odom) {
      last_odom_age =
        (now - rclcpp::Time(latest_odom_stamp_ns, node->get_clock()->get_clock_type())).seconds();
      if (last_odom_age > odom_max_age_sec || last_odom_age < -odom_max_future_sec) {
        odom_valid = false;
        consecutive_good = 0;
        last_error = "local_state odometry timestamp is stale or future-dated";
      }
    }
    if (have_tf) {
      last_tf_age =
        (now - rclcpp::Time(latest_tf_stamp_ns, node->get_clock()->get_clock_type())).seconds();
      if (last_tf_age > tf_max_age_sec || last_tf_age < -odom_max_future_sec) {
        tf_valid = false;
        consecutive_good = 0;
        last_error = "odom->base_link timestamp is stale or future-dated";
      }
    }

    if (!odom_valid || !tf_valid) {
      continue;
    }
    if (latest_odom_stamp_ns <= last_accepted_odom_stamp_ns ||
      latest_tf_stamp_ns <= last_accepted_tf_stamp_ns)
    {
      last_error = "waiting for both local_state timestamps to advance";
      continue;
    }

    last_accepted_odom_stamp_ns = latest_odom_stamp_ns;
    last_accepted_tf_stamp_ns = latest_tf_stamp_ns;
    ++consecutive_good;
    ++accepted_samples;
    last_error.clear();
  }

  if (consecutive_good >= required_good) {
    std::cerr << "[runtime-overlay] stable local_state ready: consecutive_good="
              << consecutive_good << "/" << required_good
              << " accepted_samples=" << accepted_samples
              << " odom_age=" << last_odom_age << "s"
              << " tf_age=" << last_tf_age << "s\n";
    return true;
  }

  std::cerr << "[runtime-overlay] timed out waiting for stable local_state: consecutive_good="
            << consecutive_good << "/" << required_good
            << " accepted_samples=" << accepted_samples
            << " have_odom=" << have_odom
            << " have_tf=" << have_tf;
  if (have_odom) {
    std::cerr << " last_odom_age=" << last_odom_age << "s";
  }
  if (have_tf) {
    std::cerr << " last_tf_age=" << last_tf_age << "s";
  }
  std::cerr << " last_error=" << (last_error.empty() ? "none" : last_error) << "\n";
  return false;
}

bool wait_for_mapping_preflight(const rclcpp::Node::SharedPtr & node,
                                const std::string & scan_topic,
                                const std::string & scan_owner_node,
                                const std::string & local_odom_topic,
                                const std::string & reference_odom_topic,
                                const std::string & local_state_mode,
                                double tf_timeout_sec,
                                double scan_timeout_sec,
                                double odom_timeout_sec,
                                double scan_max_age_sec,
                                double odom_max_age_sec,
                                double max_future_sec,
                                double max_odom_diff_m)
{
  constexpr const char * kBaseFrame = "base_link";
  constexpr const char * kLidarLevelFrame = "lidar_level_link";

  const auto started_at = Clock::now();
  const auto tf_deadline = started_at + std::chrono::duration<double>(std::max(0.1, tf_timeout_sec));
  const auto scan_deadline =
    started_at + std::chrono::duration<double>(std::max(0.1, scan_timeout_sec));
  const auto odom_deadline =
    started_at + std::chrono::duration<double>(std::max(0.1, odom_timeout_sec));

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  (void)tf_listener;

  bool scan_received = false;
  bool local_odom_received = false;
  bool reference_odom_received = false;
  std::int64_t latest_scan_stamp_ns = 0;
  std::int64_t latest_local_odom_stamp_ns = 0;
  double local_x = 0.0;
  double local_y = 0.0;
  double reference_x = 0.0;
  double reference_y = 0.0;

  const auto observation_qos = qos_profile(
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    10);
  auto scan_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, observation_qos,
    [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      scan_received = true;
      latest_scan_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    });
  auto local_odom_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic, observation_qos,
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      local_odom_received = true;
      latest_local_odom_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      local_x = msg->pose.pose.position.x;
      local_y = msg->pose.pose.position.y;
    });
  auto reference_odom_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
    reference_odom_topic, observation_qos,
    [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      reference_odom_received = true;
      reference_x = msg->pose.pose.position.x;
      reference_y = msg->pose.pose.position.y;
    });
  (void)scan_subscription;
  (void)local_odom_subscription;
  (void)reference_odom_subscription;

  bool tf_ready = false;
  bool scan_owner_ready = false;
  bool scan_fresh = false;
  bool local_state_endpoint_ready = false;
  bool local_odom_fresh = false;
  bool reference_odom_publisher_ready = false;
  double scan_age_sec = 0.0;
  double local_odom_age_sec = 0.0;
  double odom_diff_m = 0.0;
  const bool require_fastlio_subscription = local_state_mode == "fastlio";

  while (rclcpp::ok()) {
    spin_slice(node);
    const auto steady_now = Clock::now();
    const auto ros_now = node->get_clock()->now();

    if (!tf_ready) {
      try {
        tf_buffer->lookupTransform(kBaseFrame, kLidarLevelFrame, tf2::TimePointZero);
        tf_ready = true;
      } catch (const tf2::TransformException &) {
        tf_ready = false;
      }
    }

    scan_owner_ready = false;
    for (const auto & info : node->get_publishers_info_by_topic(scan_topic)) {
      if (node_name_matches(info.node_name(), scan_owner_node)) {
        scan_owner_ready = true;
        break;
      }
    }

    bool has_local_state_node = false;
    for (const auto & name : node->get_node_names()) {
      if (node_name_matches(name, "/robot_local_state")) {
        has_local_state_node = true;
        break;
      }
    }
    bool has_local_state_odom_publisher = false;
    for (const auto & info : node->get_publishers_info_by_topic(local_odom_topic)) {
      if (info.node_name() == "robot_local_state") {
        has_local_state_odom_publisher = true;
        break;
      }
    }
    bool has_fastlio_subscription = false;
    if (require_fastlio_subscription) {
      for (const auto & info : node->get_subscriptions_info_by_topic("/fastlio/base_odometry")) {
        if (info.node_name() == "robot_local_state") {
          has_fastlio_subscription = true;
          break;
        }
      }
    }
    local_state_endpoint_ready = has_local_state_node && has_local_state_odom_publisher &&
      (!require_fastlio_subscription || has_fastlio_subscription);
    reference_odom_publisher_ready = node->count_publishers(reference_odom_topic) > 0;

    if (scan_received) {
      scan_age_sec =
        (ros_now - rclcpp::Time(latest_scan_stamp_ns, node->get_clock()->get_clock_type())).seconds();
      scan_fresh = std::isfinite(scan_age_sec) && scan_age_sec <= scan_max_age_sec &&
        scan_age_sec >= -max_future_sec;
    }
    if (local_odom_received) {
      local_odom_age_sec =
        (ros_now - rclcpp::Time(
          latest_local_odom_stamp_ns, node->get_clock()->get_clock_type())).seconds();
      local_odom_fresh = std::isfinite(local_odom_age_sec) &&
        local_odom_age_sec <= odom_max_age_sec && local_odom_age_sec >= -max_future_sec;
    }

    if (local_odom_received && reference_odom_received) {
      if (!std::isfinite(local_x) || !std::isfinite(local_y) ||
        !std::isfinite(reference_x) || !std::isfinite(reference_y))
      {
        std::cerr << "[runtime-overlay] mapping preflight failed: non-finite odometry pose"
                  << " local_topic=" << local_odom_topic
                  << " reference_topic=" << reference_odom_topic << "\n";
        return false;
      }
      odom_diff_m = std::hypot(local_x - reference_x, local_y - reference_y);
      if (odom_diff_m > max_odom_diff_m) {
        std::cerr << "[runtime-overlay] mapping preflight failed: local odometry differs from "
                  << "reference by " << odom_diff_m << "m, max=" << max_odom_diff_m << "m"
                  << " local_topic=" << local_odom_topic
                  << " reference_topic=" << reference_odom_topic << "\n";
        return false;
      }
    }

    const bool odom_sane = local_odom_received && reference_odom_received &&
      odom_diff_m <= max_odom_diff_m;
    if (tf_ready && scan_owner_ready && scan_fresh && local_state_endpoint_ready &&
      local_odom_fresh && reference_odom_publisher_ready && odom_sane)
    {
      std::cerr << "[runtime-overlay] mapping preflight ready: tf=" << kBaseFrame << "->"
                << kLidarLevelFrame << " scan_topic=" << scan_topic
                << " scan_owner=" << scan_owner_node << " scan_age=" << scan_age_sec << "s"
                << " local_odom_topic=" << local_odom_topic
                << " local_odom_age=" << local_odom_age_sec << "s"
                << " reference_odom_topic=" << reference_odom_topic
                << " odom_diff=" << odom_diff_m << "m\n";
      return true;
    }

    const bool tf_timed_out = steady_now >= tf_deadline && !tf_ready;
    const bool scan_timed_out = steady_now >= scan_deadline &&
      (!scan_owner_ready || !scan_fresh);
    const bool odom_timed_out = steady_now >= odom_deadline &&
      (!local_state_endpoint_ready || !local_odom_fresh ||
      !reference_odom_publisher_ready || !odom_sane);
    if (tf_timed_out || scan_timed_out || odom_timed_out) {
      std::cerr << "[runtime-overlay] mapping preflight not ready:"
                << " tf_ready=" << tf_ready
                << " scan_owner_ready=" << scan_owner_ready
                << " scan_received=" << scan_received
                << " scan_fresh=" << scan_fresh
                << " local_state_endpoint_ready=" << local_state_endpoint_ready
                << " local_odom_received=" << local_odom_received
                << " local_odom_fresh=" << local_odom_fresh
                << " reference_publisher_ready=" << reference_odom_publisher_ready
                << " reference_odom_received=" << reference_odom_received;
      if (scan_received) {
        std::cerr << " scan_age=" << scan_age_sec << "s";
      }
      if (local_odom_received) {
        std::cerr << " local_odom_age=" << local_odom_age_sec << "s";
      }
      if (local_odom_received && reference_odom_received) {
        std::cerr << " odom_diff=" << odom_diff_m << "m";
      }
      std::cerr << "\n";
      return false;
    }
  }

  std::cerr << "[runtime-overlay] mapping preflight interrupted before readiness\n";
  return false;
}

bool wait_for_stamped_scan_tf(const rclcpp::Node::SharedPtr & node,
                              const std::string & scan_topic,
                              const std::string & tf_topic,
                              const std::string & target_frame,
                              double timeout_sec,
                              int required_consecutive_good)
{
  struct ScanObservation
  {
    std::int64_t stamp_ns;
    std::string frame_id;
  };

  const int required_good = std::max(1, required_consecutive_good);
  constexpr std::size_t kMaximumObservations = 64;
  const auto deadline = Clock::now() + std::chrono::duration<double>(
    std::max(0.1, timeout_sec));
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  std::deque<ScanObservation> observations;
  std::uint64_t scan_count = 0;
  std::uint64_t dynamic_tf_count = 0;
  std::uint64_t static_tf_count = 0;
  int best_consecutive_good = 0;
  std::string last_scan_frame;
  std::string last_error{"waiting for scan and TF observations"};

  const auto scan_qos = qos_profile(
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    10);
  const auto dynamic_tf_qos = qos_profile(
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    100);
  const auto static_tf_qos = qos_profile(
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    10);

  auto on_tf = [&](const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static) {
      for (const auto & transform : msg->transforms) {
        try {
          if (tf_buffer->setTransform(transform, "stamped_scan_tf_probe", is_static)) {
            if (is_static) {
              ++static_tf_count;
            } else {
              ++dynamic_tf_count;
            }
          }
        } catch (const tf2::TransformException & exc) {
          last_error = exc.what();
        }
      }
    };

  auto scan_subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, scan_qos,
    [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      ++scan_count;
      last_scan_frame = msg->header.frame_id;
      const auto stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (msg->header.frame_id.empty() || stamp_ns <= 0) {
        last_error = "scan header has an empty frame or zero timestamp";
        return;
      }
      observations.push_back(ScanObservation{stamp_ns, msg->header.frame_id});
      while (observations.size() > kMaximumObservations) {
        observations.pop_front();
      }
    });
  auto dynamic_tf_subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic, dynamic_tf_qos,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {on_tf(msg, false);});
  auto static_tf_subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", static_tf_qos,
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {on_tf(msg, true);});
  (void)scan_subscription;
  (void)dynamic_tf_subscription;
  (void)static_tf_subscription;

  while (rclcpp::ok() && Clock::now() < deadline) {
    spin_slice(node);
    int consecutive_good = 0;
    for (const auto & observation : observations) {
      std::string transform_error;
      const tf2::TimePoint stamp{
        std::chrono::nanoseconds(observation.stamp_ns)};
      const bool transformable = static_cast<const tf2::BufferCore &>(*tf_buffer).canTransform(
        target_frame, observation.frame_id, stamp, &transform_error);
      if (transformable) {
        ++consecutive_good;
        best_consecutive_good = std::max(best_consecutive_good, consecutive_good);
        if (consecutive_good >= required_good) {
          std::cerr << "[runtime-overlay] original-stamp scan TF ready: consecutive_good="
                    << consecutive_good << "/" << required_good
                    << " scan_topic=" << scan_topic
                    << " tf_topic=" << tf_topic
                    << " target_frame=" << target_frame
                    << " scan_frame=" << observation.frame_id
                    << " scans_seen=" << scan_count
                    << " dynamic_tf_seen=" << dynamic_tf_count
                    << " static_tf_seen=" << static_tf_count << "\n";
          return true;
        }
      } else {
        consecutive_good = 0;
        if (!transform_error.empty()) {
          last_error = transform_error;
        }
      }
    }
  }

  std::cerr << "[runtime-overlay] original-stamp scan TF not ready: consecutive_good="
            << best_consecutive_good << "/" << required_good
            << " scan_topic=" << scan_topic
            << " tf_topic=" << tf_topic
            << " target_frame=" << target_frame
            << " last_scan_frame=" << (last_scan_frame.empty() ? "none" : last_scan_frame)
            << " scans_seen=" << scan_count
            << " dynamic_tf_seen=" << dynamic_tf_count
            << " static_tf_seen=" << static_tf_count
            << " last_error=" << last_error << "\n";
  return false;
}

bool wait_for_transformable_scan(const rclcpp::Node::SharedPtr & node,
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
  auto on_scan = [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      ++seen;
      if (msg->header.frame_id.empty()) {
        last_error = "empty scan frame_id";
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

  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    qos_profile(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE),
    on_scan);

  while (rclcpp::ok() && Clock::now() < deadline && good < required_good) {
    spin_slice(node);
  }

  if (good >= required_good) {
    std::cerr << "[runtime-overlay] transformable scan observations ready: "
              << good << "/" << seen << " scans are TF-valid "
              << "(tf_buffer_warmup=" << warmup_sec << "s)\n";
    return true;
  }

  std::cerr << "[runtime-overlay] transformable scan observations not ready: good="
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

template<typename DurationT>
double remaining_timeout_sec(const std::chrono::time_point<Clock, DurationT> & deadline)
{
  return std::max(0.1, std::chrono::duration<double>(deadline - Clock::now()).count());
}

bool wait_for_bridge_odom_status(const rclcpp::Node::SharedPtr & node, double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  std::atomic_bool has_odom{false};
  std::atomic_bool status_received{false};
  std::string last_status;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
  auto on_status = [&](const std_msgs::msg::String::SharedPtr msg) {
      status_received.store(true);
      last_status = msg->data;
      if (msg->data.find("\"has_odom\":true") != std::string::npos) {
        has_odom.store(true);
      }
    };
  for (const auto & qos : default_qos_profiles()) {
    subscriptions.push_back(
      node->create_subscription<std_msgs::msg::String>(
        "/localization/bridge_status", qos, on_status));
  }

  while (rclcpp::ok() && Clock::now() < deadline && !has_odom.load()) {
    spin_slice(node);
  }
  if (has_odom.load()) {
    std::cerr << "[runtime-overlay] localization bridge odom ready: has_odom=true\n";
    return true;
  }
  std::cerr << "[runtime-overlay] timed out waiting for localization bridge odom: status_received="
            << status_received.load() << " has_odom=false";
  if (status_received.load()) {
    std::cerr << " last_status_size=" << last_status.size();
  }
  std::cerr << "\n";
  return false;
}

bool wait_for_localization_prestart(const rclcpp::Node::SharedPtr & node, double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  if (!wait_for_service(
      node, "/trigger_grid_search_localization", remaining_timeout_sec(deadline)))
  {
    return false;
  }
  if (!wait_for_bridge_odom_status(node, remaining_timeout_sec(deadline))) {
    return false;
  }
  std::cerr << "[runtime-overlay] localization prestart ready: isaac_service=true bridge_odom=true\n";
  return true;
}

bool wait_for_localization_stack(const rclcpp::Node::SharedPtr & node,
                                 const std::string & map_yaml,
                                 const std::string & flatscan_topic,
                                 double timeout_sec)
{
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_sec);
  if (!wait_for_service(
      node, "/global_localization/trigger", remaining_timeout_sec(deadline)))
  {
    return false;
  }
  if (!wait_for_service(
      node, "/trigger_grid_search_localization", remaining_timeout_sec(deadline)))
  {
    return false;
  }
  if (!wait_for_map_topic_matches_yaml(node, map_yaml, remaining_timeout_sec(deadline))) {
    return false;
  }
  if (!wait_for_publisher_from_node(
      node, flatscan_topic, "laser_scan_to_flatscan", remaining_timeout_sec(deadline)))
  {
    return false;
  }
  std::cerr << "[runtime-overlay] localization stack ready: services=true map=" << map_yaml
            << " flatscan_publisher=laser_scan_to_flatscan topic=" << flatscan_topic << "\n";
  return true;
}

bool wait_for_global_costmap(const rclcpp::Node::SharedPtr & node,
                             double lifecycle_timeout_sec,
                             double publisher_timeout_sec)
{
  if (!wait_for_lifecycle_active(
      node, "/global_costmap/global_costmap", lifecycle_timeout_sec))
  {
    return false;
  }
  if (!wait_for_topic_publisher(node, "/global_costmap/costmap", publisher_timeout_sec)) {
    return false;
  }
  std::cerr << "[runtime-overlay] global costmap ready: lifecycle=active publisher=true\n";
  return true;
}

[[noreturn]] void exit_probe(int code)
{
  std::cout.flush();
  std::cerr.flush();
  // This probe is a short-lived startup helper. On the Jetson runtime, Fast DDS
  // shutdown can occasionally block after a readiness condition has already
  // been observed, which freezes the parent startup script. Let the OS reclaim
  // DDS resources instead of making runtime readiness depend on clean shutdown.
  std::_Exit(code);
}

void print_usage()
{
  std::cerr
    << "usage: runtime_readiness_probe <command> [args]\n"
    << "commands:\n"
    << "  service <service_name> <timeout_sec>\n"
    << "  node <absolute_node_name> <timeout_sec>\n"
    << "  topic-publisher <topic> <timeout_sec>\n"
    << "  publisher-count <topic> <expected_count> <timeout_sec>\n"
    << "  publisher-from-node <topic> <node_name> <timeout_sec>\n"
    << "  exact-publisher-owner <topic> <node_name> <expected_count> <timeout_sec>\n"
    << "  topic <topic> <timeout_sec>\n"
    << "  fresh-header-topic <topic> <timeout_sec> <max_age_sec> <max_future_sec>\n"
    << "  ranger-chassis <timeout_sec> <odom_max_age_sec> <odom_max_future_sec>\n"
    << "  imu-bias-filter <corrected_imu_topic> <bias_topic> <timeout_sec>\n"
    << "  tf <target_frame> <source_frame> <timeout_sec>\n"
    << "  fresh-tf <target_frame> <source_frame> <timeout_sec> <max_age_sec>\n"
    << "  stable-local-state <timeout_sec> <required_consecutive_good> "
       "<odom_max_age_sec> <odom_max_future_sec> <tf_max_age_sec>\n"
    << "  mapping-preflight <scan_topic> <scan_owner_node> <local_odom_topic> "
       "<reference_odom_topic> <local_state_mode> <tf_timeout_sec> <scan_timeout_sec> "
       "<odom_timeout_sec> <scan_max_age_sec> <odom_max_age_sec> <max_future_sec> "
       "<max_odom_diff_m>\n"
    << "  stamped-scan-tf <scan_topic> <tf_topic> <target_frame> <timeout_sec> "
       "<required_consecutive_good>\n"
    << "  transformable-scan <timeout_sec> <required_good>\n"
    << "  local-state-endpoint <timeout_sec> <mode>\n"
    << "  lifecycle-active <node_name> <timeout_sec>\n"
    << "  occupancy-grid <topic> <timeout_sec> <min_width> <min_height>\n"
    << "  map-topic-matches-yaml <map_yaml> <timeout_sec>\n"
    << "  localization-prestart <timeout_sec>\n"
    << "  localization-stack <map_yaml> <flatscan_topic> <timeout_sec>\n"
    << "  global-costmap <lifecycle_timeout_sec> <publisher_timeout_sec>\n";
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
    } else if (command == "publisher-count" && argc == 5) {
      ok = wait_for_topic_publisher_count(
        node, argv[2],
        static_cast<std::size_t>(std::max(0, parse_int(argv[3], "expected_count"))),
        parse_double(argv[4], "timeout_sec"));
    } else if (command == "publisher-from-node" && argc == 5) {
      ok = wait_for_publisher_from_node(node, argv[2], argv[3], parse_double(argv[4],
        "timeout_sec"));
    } else if (command == "exact-publisher-owner" && argc == 6) {
      ok = wait_for_exact_publisher_owner(
        node, argv[2], argv[3],
        static_cast<std::size_t>(std::max(0, parse_int(argv[4], "expected_count"))),
        parse_double(argv[5], "timeout_sec"));
    } else if (command == "topic" && argc == 4) {
      ok = wait_for_topic_message(node, argv[2], parse_double(argv[3], "timeout_sec"));
    } else if (command == "fresh-header-topic" && argc == 6) {
      ok = wait_for_fresh_stamped_topic(
        node, argv[2], parse_double(argv[3], "timeout_sec"),
        parse_double(argv[4], "max_age_sec"), parse_double(argv[5], "max_future_sec"));
    } else if (command == "ranger-chassis" && argc == 5) {
      ok = wait_for_ranger_chassis(
        node, parse_double(argv[2], "timeout_sec"),
        parse_double(argv[3], "odom_max_age_sec"),
        parse_double(argv[4], "odom_max_future_sec"));
    } else if (command == "imu-bias-filter" && argc == 5) {
      ok = wait_for_imu_bias_filter(
        node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"));
    } else if (command == "tf" && argc == 5) {
      ok = wait_for_tf(node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"));
    } else if (command == "fresh-tf" && argc == 6) {
      ok = wait_for_fresh_tf(
        node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"),
        parse_double(argv[5], "max_age_sec"));
    } else if (command == "stable-local-state" && argc == 7) {
      ok = wait_for_stable_local_state(
        node,
        parse_double(argv[2], "timeout_sec"),
        std::max(1, parse_int(argv[3], "required_consecutive_good")),
        parse_double(argv[4], "odom_max_age_sec"),
        parse_double(argv[5], "odom_max_future_sec"),
        parse_double(argv[6], "tf_max_age_sec"));
    } else if (command == "mapping-preflight" && argc == 14) {
      std::string mode = argv[6];
      std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
      ok = wait_for_mapping_preflight(
        node,
        argv[2],
        argv[3],
        argv[4],
        argv[5],
        mode,
        parse_double(argv[7], "tf_timeout_sec"),
        parse_double(argv[8], "scan_timeout_sec"),
        parse_double(argv[9], "odom_timeout_sec"),
        parse_double(argv[10], "scan_max_age_sec"),
        parse_double(argv[11], "odom_max_age_sec"),
        parse_double(argv[12], "max_future_sec"),
        parse_double(argv[13], "max_odom_diff_m"));
    } else if (command == "stamped-scan-tf" && argc == 7) {
      ok = wait_for_stamped_scan_tf(
        node, argv[2], argv[3], argv[4], parse_double(argv[5], "timeout_sec"),
        std::max(1, parse_int(argv[6], "required_consecutive_good")));
    } else if (command == "transformable-scan" && argc == 4) {
      ok = wait_for_transformable_scan(
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
    } else if (command == "localization-prestart" && argc == 3) {
      ok = wait_for_localization_prestart(node, parse_double(argv[2], "timeout_sec"));
    } else if (command == "localization-stack" && argc == 5) {
      ok = wait_for_localization_stack(
        node, argv[2], argv[3], parse_double(argv[4], "timeout_sec"));
    } else if (command == "global-costmap" && argc == 4) {
      ok = wait_for_global_costmap(
        node, parse_double(argv[2], "lifecycle_timeout_sec"),
        parse_double(argv[3], "publisher_timeout_sec"));
    } else {
      print_usage();
      exit_probe(2);
    }

    exit_probe(ok ? 0 : 1);
  } catch (const std::exception & exc) {
    std::cerr << "[runtime-overlay] runtime_readiness_probe error: " << exc.what() << "\n";
    exit_probe(2);
  }
}
