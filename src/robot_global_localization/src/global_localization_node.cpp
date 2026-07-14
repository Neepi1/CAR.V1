#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("robot_global_localization"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<bool>("mock_mode", false);
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<std::string>("pose_topic", "/global_localization/pose");
    declare_parameter<std::string>("health_topic", "/global_localization/health");
    declare_parameter<std::string>("default_floor_id", "floor_1");
    declare_parameter<std::string>(
      "grid_search_trigger_service", "/trigger_grid_search_localization");
    declare_parameter<double>("service_timeout_sec", 10.0);
    declare_parameter<double>("service_call_timeout_sec", 10.0);
    declare_parameter<double>("result_wait_timeout_sec", 20.0);
    declare_parameter<double>("bridge_accept_timeout_sec", 12.0);
    declare_parameter<double>("map_to_odom_wait_timeout_sec", 8.0);
    declare_parameter<double>("map_to_odom_max_age_ms", 1000.0);
    declare_parameter<bool>("localizer_input_freshness_enabled", true);
    declare_parameter<std::string>("localizer_input_topic", "/flatscan");
    declare_parameter<double>("localizer_input_wait_timeout_sec", 1.0);
    declare_parameter<double>("localizer_input_max_age_sec", 0.5);
    declare_parameter<double>("localizer_input_min_fov_deg", 115.0);
    declare_parameter<int>("localizer_input_required_consecutive_good", 2);
    declare_parameter<double>("result_allowed_pretrigger_age_sec", 1.0);
    declare_parameter<bool>("require_grid_search_trigger", true);
    declare_parameter<bool>("require_bridge_acceptance", true);
    declare_parameter<std::string>("localization_result_topic", "/localization_result");
    declare_parameter<std::string>("bridge_status_topic", "/localization/bridge_status");
    declare_parameter<std::string>(
      "bridge_force_accept_service", "/robot_localization_bridge/force_accept_next_localization");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");

    active_floor_id_ = get_parameter("default_floor_id").as_string();
    grid_search_trigger_service_ = get_parameter("grid_search_trigger_service").as_string();
    service_timeout_sec_ = get_parameter("service_timeout_sec").as_double();
    service_call_timeout_sec_ = get_parameter("service_call_timeout_sec").as_double();
    result_wait_timeout_sec_ = get_parameter("result_wait_timeout_sec").as_double();
    bridge_accept_timeout_sec_ = get_parameter("bridge_accept_timeout_sec").as_double();
    map_to_odom_wait_timeout_sec_ = get_parameter("map_to_odom_wait_timeout_sec").as_double();
    map_to_odom_max_age_ms_ = get_parameter("map_to_odom_max_age_ms").as_double();
    localizer_input_freshness_enabled_ =
      get_parameter("localizer_input_freshness_enabled").as_bool();
    localizer_input_topic_ = get_parameter("localizer_input_topic").as_string();
    localizer_input_wait_timeout_sec_ =
      get_parameter("localizer_input_wait_timeout_sec").as_double();
    localizer_input_max_age_sec_ = get_parameter("localizer_input_max_age_sec").as_double();
    localizer_input_min_fov_deg_ = get_parameter("localizer_input_min_fov_deg").as_double();
    localizer_input_required_consecutive_good_ = std::max(
      1, static_cast<int>(get_parameter("localizer_input_required_consecutive_good").as_int()));
    result_allowed_pretrigger_age_sec_ =
      get_parameter("result_allowed_pretrigger_age_sec").as_double();
    require_grid_search_trigger_ = get_parameter("require_grid_search_trigger").as_bool();
    require_bridge_acceptance_ = get_parameter("require_bridge_acceptance").as_bool();
    mock_mode_ = get_parameter("mock_mode").as_bool();
    localization_result_topic_ = get_parameter("localization_result_topic").as_string();
    bridge_status_topic_ = get_parameter("bridge_status_topic").as_string();
    bridge_force_accept_service_ = get_parameter("bridge_force_accept_service").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::QoS(10));
    health_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("health_topic").as_string(), rclcpp::QoS(10));
    grid_search_trigger_client_ = create_client<std_srvs::srv::Empty>(
      grid_search_trigger_service_, rmw_qos_profile_services_default, callback_group_);
    bridge_force_accept_client_ = create_client<std_srvs::srv::Trigger>(
      bridge_force_accept_service_, rmw_qos_profile_services_default, callback_group_);
    localization_result_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_result_topic_,
      rclcpp::QoS(20),
      std::bind(&GlobalLocalizationNode::on_localization_result, this, std::placeholders::_1));
    bridge_status_sub_ = create_subscription<std_msgs::msg::String>(
      bridge_status_topic_,
      rclcpp::QoS(10),
      std::bind(&GlobalLocalizationNode::on_bridge_status, this, std::placeholders::_1));

    trigger_srv_ = create_service<robot_interfaces::srv::TriggerLocalization>(
      "/global_localization/trigger",
      std::bind(&GlobalLocalizationNode::on_trigger, this, std::placeholders::_1,
      std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);
    apply_floor_srv_ = create_service<robot_interfaces::srv::ApplyFloorAssets>(
      "/global_localization/apply_floor_assets",
      std::bind(&GlobalLocalizationNode::on_apply_floor, this, std::placeholders::_1,
      std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalLocalizationNode::on_timer, this),
      callback_group_);
  }

private:
  struct LocalizationResultSnapshot
  {
    bool available{false};
    std::uint64_t seq{0U};
    double received_sec{0.0};
    double header_stamp_sec{0.0};
    std::string frame_id;
  };

  struct LocalizerInputSnapshot
  {
    bool available{false};
    std::uint64_t seq{0U};
    double received_sec{0.0};
    double header_stamp_sec{0.0};
    double fov_deg{0.0};
    std::size_t point_count{0U};
    std::string topic_type;
  };

  struct BridgeStatusSnapshot
  {
    bool available{false};
    double received_sec{0.0};
    std::uint64_t accepted_result_count{0U};
    std::uint64_t rejected_result_count{0U};
    std::uint64_t last_explicit_relocalization_sequence{0U};
    bool has_map_to_odom{false};
    bool safe_for_goal_start{false};
    bool correction_active{false};
    double map_to_odom_age_ms{-1.0};
    double map_odom_publish_gap_ms{-1.0};
    double remaining_translation_error_m{-1.0};
    double remaining_yaw_error_rad{-1.0};
    std::uint64_t force_accept_ignored_pretrigger_result_count{0U};
    std::string owner;
    std::string gate_mode;
    std::string last_accept_reason;
    std::string last_reject_reason;
    std::string last_force_accept_ignored_reason;
    std::string last_explicit_relocalization_source;
    std::string raw;
  };

  void on_trigger(
    const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request> request,
    std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response)
  {
    const double service_call_timeout_sec =
      positive_or_default(service_call_timeout_sec_, std::min(service_timeout_sec_, 5.0));
    const double result_wait_timeout_sec =
      positive_or_default(result_wait_timeout_sec_, std::max(service_timeout_sec_, 20.0));
    const double bridge_accept_timeout_sec =
      positive_or_default(bridge_accept_timeout_sec_, 12.0);
    const double map_to_odom_wait_timeout_sec =
      positive_or_default(map_to_odom_wait_timeout_sec_, 8.0);

    const auto service_timeout = std::chrono::duration<double>(service_call_timeout_sec);
    if (!grid_search_trigger_client_->wait_for_service(service_timeout)) {
      response->accepted = !require_grid_search_trigger_;
      response->message =
        "failure_code=ISAAC_SERVICE_TIMEOUT service unavailable: " + grid_search_trigger_service_;
      return;
    }

    std::string input_detail;
    if (!wait_for_fresh_localizer_input(input_detail)) {
      response->accepted = false;
      response->message = input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    const auto pre_arm_input = localizer_input_snapshot();
    std::string force_accept_detail;
    const bool force_accept_armed =
      arm_bridge_force_accept(request->reason, service_call_timeout_sec, force_accept_detail);
    const double force_accept_ready_sec = now().seconds();

    if (force_accept_armed) {
      std::string post_arm_input_detail;
      if (!wait_for_localizer_input_after_arm(
          pre_arm_input, force_accept_ready_sec, post_arm_input_detail))
      {
        input_detail += "; " + post_arm_input_detail + "; continuing with the latest fresh input";
      } else {
        input_detail += "; " + post_arm_input_detail;
      }
    }

    const auto trigger_started_sec = now().seconds();
    const auto initial_result = localization_result_snapshot();
    const auto initial_bridge = bridge_status_snapshot();

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = grid_search_trigger_client_->async_send_request(empty_request);
    last_trigger_status_ = "grid_search_trigger_pending: " + request->reason;

    std::string direct_service_detail;
    const auto service_deadline = steady_deadline(service_call_timeout_sec);
    while (std::chrono::steady_clock::now() <= service_deadline) {
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          (void)future.get();
          direct_service_detail = "isaac direct service returned";
          break;
        } catch (const std::exception & exc) {
          response->accepted = false;
          response->message =
            std::string("failure_code=ISAAC_SERVICE_TIMEOUT direct service failed: ") + exc.what();
          last_trigger_status_ = response->message;
          return;
        }
      }
      if (
        localization_result_observed_after(initial_result, trigger_started_sec) ||
        bridge_processed_after(initial_bridge, trigger_started_sec))
      {
        direct_service_detail = "isaac direct service response pending; localization result already observed";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (direct_service_detail.empty()) {
      response->accepted = false;
      response->message =
        "failure_code=ISAAC_SERVICE_TIMEOUT direct service did not return or produce localization_result within " +
        std::to_string(service_call_timeout_sec) + "s";
      last_trigger_status_ = response->message;
      return;
    }

    if (!wait_for_localization_result_or_bridge_processing(
        initial_result, initial_bridge, trigger_started_sec, result_wait_timeout_sec))
    {
      response->accepted = false;
      response->message =
        "failure_code=LOCALIZATION_RESULT_TIMEOUT no localization_result or bridge_status update within " +
        std::to_string(result_wait_timeout_sec) + "s; " + force_accept_detail + "; " +
        direct_service_detail + "; " + input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string bridge_detail;
    if (require_bridge_acceptance_ && !wait_for_bridge_acceptance(
        initial_bridge,
        trigger_started_sec,
        bridge_accept_timeout_sec,
        bridge_detail))
    {
      response->accepted = false;
      response->message =
        bridge_detail + "; " + force_accept_detail + "; " + direct_service_detail + "; " +
        input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string map_to_odom_detail;
    if (!wait_for_map_to_odom(map_to_odom_wait_timeout_sec, map_to_odom_detail)) {
      response->accepted = false;
      response->message =
        map_to_odom_detail + "; " + force_accept_detail + "; " + direct_service_detail + "; " +
        input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    response->accepted = true;
    std::ostringstream out;
    out << "triggered relocalization accepted"
        << " explicit_trigger=" << (force_accept_armed ? "true" : "false")
        << "; " << force_accept_detail
        << "; " << direct_service_detail
        << "; " << input_detail
        << "; " << bridge_detail
        << "; " << map_to_odom_detail
        << "; reason=" << request->reason;
    response->message = out.str();
    last_trigger_status_ = response->message;
    RCLCPP_INFO(get_logger(), "%s", last_trigger_status_.c_str());
  }

  void on_apply_floor(
    const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Request> request,
    std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> response)
  {
    std::string missing;
    append_missing(missing, "nav_map_yaml", request->nav_map_yaml);
    append_missing(missing, "localizer_map_png", request->localizer_map_png);
    append_missing(missing, "localizer_params_yaml", request->localizer_params_yaml);
    if (!missing.empty()) {
      response->success = false;
      response->message = "missing floor assets: " + missing;
      return;
    }

    active_floor_id_ = request->floor_id;
    active_nav_map_yaml_ = request->nav_map_yaml;
    active_localizer_map_png_ = request->localizer_map_png;
    active_localizer_params_yaml_ = request->localizer_params_yaml;
    response->success = true;
    response->message = "applied floor " + request->floor_id + ": " + request->nav_map_yaml;
  }

  void on_timer()
  {
    const bool trigger_ready = grid_search_trigger_client_->service_is_ready();
    if (mock_mode_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      const int64_t now_ns = get_clock()->now().nanoseconds();
      pose.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      pose.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
      pose.header.frame_id = "map";
      pose_pub_->publish(pose);
      std_msgs::msg::String health;
      health.data = "mock_localizer_ready floor=" + active_floor_id_;
      health_pub_->publish(health);
      return;
    }

    std_msgs::msg::String health;
    const std::string status = trigger_ready ? "localizer_ready" :
      "localizer_waiting_for_grid_search";
    health.data = status + " floor=" + active_floor_id_ +
      " trigger_status=" + last_trigger_status_;
    health_pub_->publish(health);
  }

  void on_localization_result(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    localization_result_.available = true;
    localization_result_.seq++;
    localization_result_.received_sec = now().seconds();
    localization_result_.header_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
    localization_result_.frame_id = msg->header.frame_id;
  }

  void on_bridge_status(const std_msgs::msg::String::SharedPtr msg)
  {
    BridgeStatusSnapshot snapshot;
    snapshot.available = true;
    snapshot.received_sec = now().seconds();
    snapshot.raw = msg->data;
    snapshot.accepted_result_count = json_uint_value(msg->data, "accepted_result_count", 0U);
    snapshot.rejected_result_count = json_uint_value(msg->data, "rejected_result_count", 0U);
    snapshot.force_accept_ignored_pretrigger_result_count =
      json_uint_value(msg->data, "force_accept_ignored_pretrigger_result_count", 0U);
    snapshot.last_explicit_relocalization_sequence =
      json_uint_value(msg->data, "last_explicit_relocalization_sequence", 0U);
    snapshot.has_map_to_odom = json_bool_value(msg->data, "has_map_to_odom", false);
    snapshot.safe_for_goal_start = json_bool_value(msg->data, "safe_for_goal_start", false);
    snapshot.correction_active = json_bool_value(msg->data, "correction_active", false);
    snapshot.map_to_odom_age_ms = json_double_value(msg->data, "map_to_odom_age_ms", -1.0);
    snapshot.map_odom_publish_gap_ms = json_double_value(msg->data, "map_odom_publish_gap_ms", -1.0);
    snapshot.remaining_translation_error_m =
      json_double_value(msg->data, "remaining_translation_error_m", -1.0);
    snapshot.remaining_yaw_error_rad = json_double_value(msg->data, "remaining_yaw_error_rad", -1.0);
    snapshot.owner = json_string_value(msg->data, "map_to_odom_publisher_owner");
    snapshot.gate_mode = json_string_value(msg->data, "gate_mode");
    snapshot.last_accept_reason = json_string_value(msg->data, "last_accept_reason");
    snapshot.last_reject_reason = json_string_value(msg->data, "last_reject_reason");
    snapshot.last_force_accept_ignored_reason =
      json_string_value(msg->data, "last_force_accept_ignored_reason");
    snapshot.last_explicit_relocalization_source =
      json_string_value(msg->data, "last_explicit_relocalization_source");

    std::lock_guard<std::mutex> lock(state_mutex_);
    bridge_status_ = snapshot;
  }

  static double localizer_input_fov_deg(
    const isaac_ros_pointcloud_interfaces::msg::FlatScan & msg)
  {
    double min_angle = std::numeric_limits<double>::infinity();
    double max_angle = -std::numeric_limits<double>::infinity();
    for (const float angle : msg.angles) {
      if (!std::isfinite(angle)) {
        continue;
      }
      min_angle = std::min(min_angle, static_cast<double>(angle));
      max_angle = std::max(max_angle, static_cast<double>(angle));
    }
    if (!std::isfinite(min_angle) || !std::isfinite(max_angle) || max_angle < min_angle) {
      return 0.0;
    }
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return (max_angle - min_angle) * kRadiansToDegrees;
  }

  void on_localizer_input(
    const isaac_ros_pointcloud_interfaces::msg::FlatScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    localizer_input_.available = true;
    localizer_input_.seq++;
    localizer_input_.received_sec = now().seconds();
    localizer_input_.header_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
    localizer_input_.fov_deg = localizer_input_fov_deg(*msg);
    localizer_input_.point_count = msg->angles.size();
  }

  static void append_missing(std::string & missing, const std::string & name,
                             const std::string & path)
  {
    if (!path.empty() && std::filesystem::exists(path)) {
      return;
    }
    if (!missing.empty()) {
      missing += "; ";
    }
    missing += name + "=" + path;
  }

  static double positive_or_default(const double value, const double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  static std::chrono::steady_clock::time_point steady_deadline(const double timeout_sec)
  {
    return std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(timeout_sec));
  }

  static std::uint64_t json_uint_value(
    const std::string & data,
    const std::string & key,
    const std::uint64_t fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    try {
      return static_cast<std::uint64_t>(std::stoull(data.substr(colon + 1)));
    } catch (...) {
      return fallback;
    }
  }

  static double json_double_value(
    const std::string & data,
    const std::string & key,
    const double fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    try {
      return std::stod(data.substr(colon + 1));
    } catch (...) {
      return fallback;
    }
  }

  static bool json_bool_value(
    const std::string & data,
    const std::string & key,
    const bool fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    const auto value_pos = data.find_first_not_of(" \t\r\n", colon + 1);
    if (value_pos == std::string::npos) {
      return fallback;
    }
    if (data.compare(value_pos, 4, "true") == 0) {
      return true;
    }
    if (data.compare(value_pos, 5, "false") == 0) {
      return false;
    }
    return fallback;
  }

  static std::string json_string_value(const std::string & data, const std::string & key)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return "";
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return "";
    }
    auto quote = data.find('"', colon + 1);
    if (quote == std::string::npos) {
      return "";
    }
    std::string value;
    bool escaped = false;
    for (auto index = quote + 1; index < data.size(); ++index) {
      const char c = data[index];
      if (escaped) {
        value += c;
        escaped = false;
        continue;
      }
      if (c == '\\') {
        escaped = true;
        continue;
      }
      if (c == '"') {
        return value;
      }
      value += c;
    }
    return "";
  }

  LocalizationResultSnapshot localization_result_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return localization_result_;
  }

  BridgeStatusSnapshot bridge_status_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return bridge_status_;
  }

  LocalizerInputSnapshot localizer_input_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return localizer_input_;
  }

  bool ensure_localizer_input_subscription(std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "localizer input freshness gate disabled";
      return true;
    }
    if (localizer_input_sub_) {
      return true;
    }

    try {
      localizer_input_sub_ =
        create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>(
        localizer_input_topic_,
        rclcpp::QoS(10),
        std::bind(&GlobalLocalizationNode::on_localizer_input, this, std::placeholders::_1));
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        localizer_input_.topic_type = "isaac_ros_pointcloud_interfaces/msg/FlatScan";
      }
      detail = "localizer input subscription ready topic=" + localizer_input_topic_ +
               " type=isaac_ros_pointcloud_interfaces/msg/FlatScan";
      return true;
    } catch (const std::exception & exc) {
      detail = "localizer input subscription failed topic=" + localizer_input_topic_ +
               " type=isaac_ros_pointcloud_interfaces/msg/FlatScan: " + exc.what();
      return false;
    }
  }

  bool wait_for_fresh_localizer_input(std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "localizer input freshness gate disabled";
      return true;
    }

    const double timeout_sec = positive_or_default(localizer_input_wait_timeout_sec_, 1.0);
    const double max_age_sec = positive_or_default(localizer_input_max_age_sec_, 0.5);
    const auto deadline = steady_deadline(timeout_sec);
    std::string subscription_detail;

    while (std::chrono::steady_clock::now() <= deadline) {
      if (ensure_localizer_input_subscription(subscription_detail)) {
        const auto snapshot = localizer_input_snapshot();
        if (snapshot.available) {
          const double age_sec = now().seconds() - snapshot.received_sec;
          if (
            age_sec >= 0.0 && age_sec <= max_age_sec &&
            snapshot.fov_deg >= localizer_input_min_fov_deg_)
          {
            detail = "localizer input fresh topic=" + localizer_input_topic_ +
                     " type=" + snapshot.topic_type +
                     " age_sec=" + std::to_string(age_sec) +
                     " fov_deg=" + std::to_string(snapshot.fov_deg) +
                     " points=" + std::to_string(snapshot.point_count);
            return true;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto snapshot = localizer_input_snapshot();
    detail = "failure_code=LOCALIZER_INPUT_NOT_FRESH topic=" + localizer_input_topic_ +
             " timeout_sec=" + std::to_string(timeout_sec) +
             " max_age_sec=" + std::to_string(max_age_sec) +
             " min_fov_deg=" + std::to_string(localizer_input_min_fov_deg_) +
             " available=" + bool_string(snapshot.available) +
             " seq=" + std::to_string(snapshot.seq) +
             " fov_deg=" + std::to_string(snapshot.fov_deg) +
             " last_age_sec=" +
             (snapshot.available ? std::to_string(now().seconds() - snapshot.received_sec) : "-1") +
             " subscription_detail=" + subscription_detail;
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    return false;
  }

  bool wait_for_localizer_input_after_arm(
    const LocalizerInputSnapshot & baseline,
    const double armed_sec,
    std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "post-arm localizer input gate disabled";
      return true;
    }

    const double timeout_sec = positive_or_default(localizer_input_wait_timeout_sec_, 1.0);
    const double max_age_sec = positive_or_default(localizer_input_max_age_sec_, 0.5);
    const double min_header_stamp_sec =
      armed_sec - positive_or_default(result_allowed_pretrigger_age_sec_, 1.0);
    const auto deadline = steady_deadline(timeout_sec);
    std::string subscription_detail;
    const int required_consecutive_good = std::max(
      1, localizer_input_required_consecutive_good_);
    int consecutive_good = 0;
    std::uint64_t last_examined_seq = baseline.seq;

    while (std::chrono::steady_clock::now() <= deadline) {
      if (ensure_localizer_input_subscription(subscription_detail)) {
        const auto snapshot = localizer_input_snapshot();
        if (snapshot.seq == last_examined_seq) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        last_examined_seq = snapshot.seq;
        const double age_sec = now().seconds() - snapshot.received_sec;
        const bool sample_good =
          snapshot.available &&
          snapshot.seq > baseline.seq &&
          snapshot.received_sec >= armed_sec &&
          snapshot.header_stamp_sec >= min_header_stamp_sec &&
          snapshot.fov_deg >= localizer_input_min_fov_deg_ &&
          age_sec >= 0.0 && age_sec <= max_age_sec;
        if (sample_good) {
          ++consecutive_good;
          if (consecutive_good >= required_consecutive_good) {
            detail = "post-arm localizer input ready topic=" + localizer_input_topic_ +
                     " seq=" + std::to_string(snapshot.seq) +
                     " age_sec=" + std::to_string(age_sec) +
                     " fov_deg=" + std::to_string(snapshot.fov_deg) +
                     " points=" + std::to_string(snapshot.point_count) +
                     " consecutive_good=" + std::to_string(consecutive_good);
            return true;
          }
        } else {
          consecutive_good = 0;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto snapshot = localizer_input_snapshot();
    detail = "post-arm localizer input did not advance topic=" + localizer_input_topic_ +
             " baseline_seq=" + std::to_string(baseline.seq) +
             " current_seq=" + std::to_string(snapshot.seq) +
             " armed_sec=" + std::to_string(armed_sec) +
             " min_header_stamp_sec=" + std::to_string(min_header_stamp_sec) +
             " last_header_stamp_sec=" + std::to_string(snapshot.header_stamp_sec) +
             " last_received_sec=" + std::to_string(snapshot.received_sec) +
             " fov_deg=" + std::to_string(snapshot.fov_deg) +
             " min_fov_deg=" + std::to_string(localizer_input_min_fov_deg_) +
             " consecutive_good=" + std::to_string(consecutive_good) +
             " required_consecutive_good=" + std::to_string(required_consecutive_good);
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    return false;
  }

  bool localization_result_is_fresh_for_trigger(
    const LocalizationResultSnapshot & snapshot,
    const double trigger_started_sec) const
  {
    const double allowed_pretrigger_age_sec =
      positive_or_default(result_allowed_pretrigger_age_sec_, 1.0);
    return snapshot.header_stamp_sec >= trigger_started_sec - allowed_pretrigger_age_sec;
  }

  bool localization_result_observed_after(
    const LocalizationResultSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = localization_result_snapshot();
    return snapshot.available &&
           snapshot.seq > initial.seq &&
           snapshot.received_sec >= trigger_started_sec &&
           localization_result_is_fresh_for_trigger(snapshot, trigger_started_sec);
  }

  bool bridge_processed_after(
    const BridgeStatusSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = bridge_status_snapshot();
    return snapshot.available &&
           snapshot.received_sec >= trigger_started_sec &&
           (snapshot.accepted_result_count > initial.accepted_result_count ||
            snapshot.rejected_result_count > initial.rejected_result_count ||
            snapshot.force_accept_ignored_pretrigger_result_count >
            initial.force_accept_ignored_pretrigger_result_count);
  }

  bool arm_bridge_force_accept(
    const std::string & reason,
    const double timeout_sec,
    std::string & detail)
  {
    const auto timeout = std::chrono::duration<double>(std::min(timeout_sec, 2.0));
    if (!bridge_force_accept_client_->wait_for_service(timeout)) {
      detail = "bridge force-accept unavailable for explicit_trigger reason=" + reason;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = bridge_force_accept_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = "bridge force-accept timed out for explicit_trigger reason=" + reason;
      return false;
    }
    try {
      const auto response = future.get();
      if (!response->success) {
        detail = "bridge force-accept rejected explicit_trigger reason=" + reason +
                 ": " + response->message;
        return false;
      }
      detail = "bridge force-accept armed explicit_trigger=true reason=" + reason +
               ": " + response->message;
      return true;
    } catch (const std::exception & exc) {
      detail = std::string("bridge force-accept failed explicit_trigger reason=") + reason +
               ": " + exc.what();
      return false;
    }
  }

  bool wait_for_localization_result_or_bridge_processing(
    const LocalizationResultSnapshot & initial_result,
    const BridgeStatusSnapshot & initial_bridge,
    const double trigger_started_sec,
    const double timeout_sec)
  {
    const auto deadline = steady_deadline(timeout_sec);
    while (std::chrono::steady_clock::now() <= deadline) {
      if (
        localization_result_observed_after(initial_result, trigger_started_sec) ||
        bridge_processed_after(initial_bridge, trigger_started_sec))
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  bool wait_for_bridge_acceptance(
    const BridgeStatusSnapshot & initial,
    const double trigger_started_sec,
    const double timeout_sec,
    std::string & detail)
  {
    auto active_deadline = steady_deadline(timeout_sec);
    bool saw_nonfresh_bridge_accept = false;
    std::uint64_t last_amcl_observe_only_reject_count = initial.rejected_result_count;
    std::uint64_t last_pretrigger_ignored_count =
      initial.force_accept_ignored_pretrigger_result_count;
    BridgeStatusSnapshot latest;
    while (
      std::chrono::steady_clock::now() <= active_deadline)
    {
      latest = bridge_status_snapshot();
      if (latest.available && latest.received_sec >= trigger_started_sec) {
        if (
          bridge_explicit_trigger_accept_observed(initial, latest) &&
          latest.has_map_to_odom)
        {
          if (map_to_odom_ready(latest))
          {
            detail = "bridge accepted explicit triggered relocalization"
                     " gate_mode=" + latest.gate_mode +
                     " accept_reason=" + latest.last_accept_reason +
                     " explicit_sequence=" +
                     std::to_string(latest.last_explicit_relocalization_sequence) +
                     " explicit_source=" + latest.last_explicit_relocalization_source +
                     " has_map_to_odom=true age_ms=" +
                     std::to_string(latest.map_to_odom_age_ms) +
                     " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms) +
                     " safe_for_goal_start=" + bool_string(latest.safe_for_goal_start) +
                     " correction_active=" + bool_string(latest.correction_active) +
                     " remaining_translation_error_m=" +
                     std::to_string(latest.remaining_translation_error_m) +
                     " remaining_yaw_error_rad=" +
                     std::to_string(latest.remaining_yaw_error_rad);
            return true;
          }
          saw_nonfresh_bridge_accept = true;
          detail = "bridge accepted explicit triggered relocalization but map->odom is not fresh yet"
                   " owner=" + latest.owner +
                   " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                   " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms) +
                   " explicit_sequence=" +
                   std::to_string(latest.last_explicit_relocalization_sequence) +
                   " last_reject_reason=" + latest.last_reject_reason;
        }
        if (
          latest.force_accept_ignored_pretrigger_result_count >
          last_pretrigger_ignored_count)
        {
          last_pretrigger_ignored_count =
            latest.force_accept_ignored_pretrigger_result_count;
          detail =
            "failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED bridge ignored pre-force-accept stale localization_result; "
            "a new Isaac trigger is required; ignored_reason=" +
            latest.last_force_accept_ignored_reason;
          RCLCPP_WARN(get_logger(), "%s", detail.c_str());
          return false;
        }
        if (latest.rejected_result_count > initial.rejected_result_count) {
          if (bridge_reject_is_transient_triggered_stale(latest.last_reject_reason)) {
            detail =
              "failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED "
              "bridge rejected stale triggered localization_result; a new Isaac trigger is "
              "required; last_reject_reason=" +
              latest.last_reject_reason;
            RCLCPP_WARN(get_logger(), "%s", detail.c_str());
            return false;
          }
          if (bridge_reject_is_expected_amcl_observe_only(latest.last_reject_reason)) {
            if (latest.rejected_result_count > last_amcl_observe_only_reject_count) {
              last_amcl_observe_only_reject_count = latest.rejected_result_count;
              detail = "bridge ignoring AMCL observe-only reject while waiting for fresh "
                       "triggered localization_result: " + latest.last_reject_reason;
              RCLCPP_INFO(get_logger(), "%s", detail.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
          }
          detail = "failure_code=BRIDGE_REJECTED_RESULT last_reject_reason=" +
                   latest.last_reject_reason;
          return false;
        }
        if (
          latest.accepted_result_count > initial.accepted_result_count &&
          latest.has_map_to_odom)
        {
          if (map_to_odom_ready(latest))
          {
            detail = "bridge accepted result gate_mode=" + latest.gate_mode +
                     " accept_reason=" + latest.last_accept_reason +
                     " has_map_to_odom=true age_ms=" +
                     std::to_string(latest.map_to_odom_age_ms) +
                     " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
            return true;
          }
          saw_nonfresh_bridge_accept = true;
          detail = "bridge accepted count advanced but map->odom is not fresh yet owner=" +
                   latest.owner + " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                   " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = saw_nonfresh_bridge_accept ?
      "failure_code=BRIDGE_ACCEPT_TIMEOUT bridge accepted a result but map->odom did not become fresh" :
      "failure_code=BRIDGE_ACCEPT_TIMEOUT bridge did not accept localization_result";
    if (latest.available) {
      detail += " last_reject_reason=" + latest.last_reject_reason;
      if (latest.last_reject_reason.find("tf_history_missing") != std::string::npos) {
        detail = "failure_code=TF_HISTORY_MISSING " + latest.last_reject_reason;
      }
    }
    return false;
  }

  bool bridge_reject_is_transient_triggered_stale(const std::string & reason) const
  {
    return reason.find("isaac_triggered_pose_stale_ms") != std::string::npos &&
           reason.find("gate_mode=triggered") != std::string::npos;
  }

  bool bridge_reject_is_expected_amcl_observe_only(const std::string & reason) const
  {
    return (
      reason.find("AMCL_ROBOT_MOVING_OBSERVE_ONLY") != std::string::npos ||
      reason.find("amcl_suppressed_after_isaac_triggered") != std::string::npos ||
      reason.find("AMCL_CORRECTION_TOO_LARGE") != std::string::npos) &&
      reason.find("amcl") != std::string::npos;
  }

  bool bridge_explicit_trigger_accept_observed(
    const BridgeStatusSnapshot & initial,
    const BridgeStatusSnapshot & latest) const
  {
    return latest.last_explicit_relocalization_sequence >
           initial.last_explicit_relocalization_sequence ||
           (
      latest.accepted_result_count > initial.accepted_result_count &&
      latest.last_accept_reason == "EXPLICIT_TRIGGERED_RELOCALIZATION");
  }

  static std::string bool_string(const bool value)
  {
    return value ? "true" : "false";
  }

  bool map_to_odom_tf_available()
  {
    try {
      const rclcpp::Time latest_tf_time(0, 0, get_clock()->get_clock_type());
      const auto timeout = rclcpp::Duration::from_seconds(0.02);
      (void)tf_buffer_.lookupTransform(map_frame_, odom_frame_, latest_tf_time, timeout);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  bool map_to_odom_bridge_publish_healthy(const BridgeStatusSnapshot & latest) const
  {
    if (latest.map_to_odom_age_ms >= 0.0 && latest.map_to_odom_age_ms <= map_to_odom_max_age_ms_) {
      return true;
    }
    return latest.map_odom_publish_gap_ms >= 0.0 &&
           latest.map_odom_publish_gap_ms <= map_to_odom_max_age_ms_;
  }

  bool map_to_odom_ready(const BridgeStatusSnapshot & latest)
  {
    return latest.available &&
           latest.has_map_to_odom &&
           latest.owner == "robot_localization_bridge" &&
           map_to_odom_bridge_publish_healthy(latest) &&
           map_to_odom_tf_available();
  }

  bool wait_for_map_to_odom(const double timeout_sec, std::string & detail)
  {
    const auto deadline = steady_deadline(timeout_sec);
    BridgeStatusSnapshot latest;
    while (std::chrono::steady_clock::now() <= deadline) {
      latest = bridge_status_snapshot();
      if (
        latest.available &&
        latest.has_map_to_odom &&
        latest.owner != "robot_localization_bridge")
      {
        detail = "failure_code=MAP_TO_ODOM_WRONG_OWNER owner=" + latest.owner;
        return false;
      }
      if (map_to_odom_ready(latest))
      {
        detail = "map->odom ready owner=robot_localization_bridge age_ms=" +
                 std::to_string(latest.map_to_odom_age_ms) +
                 " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = "failure_code=MAP_TO_ODOM_TIMEOUT map->odom unavailable or stale";
    if (latest.available) {
      detail += " has_map_to_odom=" + std::string(latest.has_map_to_odom ? "true" : "false") +
                " owner=" + latest.owner +
                " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
    }
    return false;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr grid_search_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bridge_force_accept_client_;
  rclcpp::Subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>::SharedPtr
    localizer_input_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_status_sub_;
  rclcpp::Service<robot_interfaces::srv::TriggerLocalization>::SharedPtr trigger_srv_;
  rclcpp::Service<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr apply_floor_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex state_mutex_;

  std::string active_floor_id_;
  std::string active_nav_map_yaml_;
  std::string active_localizer_map_png_;
  std::string active_localizer_params_yaml_;
  std::string grid_search_trigger_service_;
  std::string localization_result_topic_;
  std::string bridge_status_topic_;
  std::string bridge_force_accept_service_;
  std::string localizer_input_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  double service_timeout_sec_{10.0};
  double service_call_timeout_sec_{10.0};
  double result_wait_timeout_sec_{20.0};
  double bridge_accept_timeout_sec_{12.0};
  double map_to_odom_wait_timeout_sec_{8.0};
  double map_to_odom_max_age_ms_{1000.0};
  double localizer_input_wait_timeout_sec_{1.0};
  double localizer_input_max_age_sec_{0.5};
  double localizer_input_min_fov_deg_{115.0};
  int localizer_input_required_consecutive_good_{2};
  double result_allowed_pretrigger_age_sec_{1.0};
  bool localizer_input_freshness_enabled_{true};
  bool require_grid_search_trigger_{true};
  bool require_bridge_acceptance_{true};
  bool mock_mode_{false};
  std::string last_trigger_status_{"idle"};
  LocalizerInputSnapshot localizer_input_;
  LocalizationResultSnapshot localization_result_;
  BridgeStatusSnapshot bridge_status_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
