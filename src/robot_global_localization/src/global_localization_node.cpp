#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
    declare_parameter<double>("bridge_accept_timeout_sec", 8.0);
    declare_parameter<double>("map_to_odom_wait_timeout_sec", 8.0);
    declare_parameter<double>("map_to_odom_max_age_ms", 1000.0);
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

  struct BridgeStatusSnapshot
  {
    bool available{false};
    double received_sec{0.0};
    std::uint64_t accepted_result_count{0U};
    std::uint64_t rejected_result_count{0U};
    bool has_map_to_odom{false};
    double map_to_odom_age_ms{-1.0};
    std::string owner;
    std::string gate_mode;
    std::string last_accept_reason;
    std::string last_reject_reason;
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
      positive_or_default(bridge_accept_timeout_sec_, 8.0);
    const double map_to_odom_wait_timeout_sec =
      positive_or_default(map_to_odom_wait_timeout_sec_, 8.0);

    const auto trigger_started_sec = now().seconds();
    const auto initial_result = localization_result_snapshot();
    const auto initial_bridge = bridge_status_snapshot();
    const auto service_timeout = std::chrono::duration<double>(service_call_timeout_sec);
    if (!grid_search_trigger_client_->wait_for_service(service_timeout)) {
      response->accepted = !require_grid_search_trigger_;
      response->message =
        "failure_code=ISAAC_SERVICE_TIMEOUT service unavailable: " + grid_search_trigger_service_;
      return;
    }

    std::string force_accept_detail;
    const bool force_accept_armed =
      arm_bridge_force_accept(request->reason, service_call_timeout_sec, force_accept_detail);

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
        direct_service_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string bridge_detail;
    if (require_bridge_acceptance_ && !wait_for_bridge_acceptance(
        initial_bridge, trigger_started_sec, bridge_accept_timeout_sec, bridge_detail))
    {
      response->accepted = false;
      response->message = bridge_detail + "; " + force_accept_detail + "; " + direct_service_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string map_to_odom_detail;
    if (!wait_for_map_to_odom(map_to_odom_wait_timeout_sec, map_to_odom_detail)) {
      response->accepted = false;
      response->message = map_to_odom_detail + "; " + force_accept_detail + "; " + direct_service_detail;
      last_trigger_status_ = response->message;
      return;
    }

    response->accepted = true;
    std::ostringstream out;
    out << "triggered relocalization accepted"
        << " explicit_trigger=" << (force_accept_armed ? "true" : "false")
        << "; " << force_accept_detail
        << "; " << direct_service_detail
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
    snapshot.has_map_to_odom = json_bool_value(msg->data, "has_map_to_odom", false);
    snapshot.map_to_odom_age_ms = json_double_value(msg->data, "map_to_odom_age_ms", -1.0);
    snapshot.owner = json_string_value(msg->data, "map_to_odom_publisher_owner");
    snapshot.gate_mode = json_string_value(msg->data, "gate_mode");
    snapshot.last_accept_reason = json_string_value(msg->data, "last_accept_reason");
    snapshot.last_reject_reason = json_string_value(msg->data, "last_reject_reason");

    std::lock_guard<std::mutex> lock(state_mutex_);
    bridge_status_ = snapshot;
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

  bool localization_result_observed_after(
    const LocalizationResultSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = localization_result_snapshot();
    return snapshot.available &&
           snapshot.seq > initial.seq &&
           snapshot.received_sec >= trigger_started_sec;
  }

  bool bridge_processed_after(
    const BridgeStatusSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = bridge_status_snapshot();
    return snapshot.available &&
           snapshot.received_sec >= trigger_started_sec &&
           (snapshot.accepted_result_count > initial.accepted_result_count ||
            snapshot.rejected_result_count > initial.rejected_result_count);
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
    const auto deadline = steady_deadline(timeout_sec);
    BridgeStatusSnapshot latest;
    while (std::chrono::steady_clock::now() <= deadline) {
      latest = bridge_status_snapshot();
      if (latest.available && latest.received_sec >= trigger_started_sec) {
        if (latest.rejected_result_count > initial.rejected_result_count) {
          detail = "failure_code=BRIDGE_REJECTED_RESULT last_reject_reason=" +
                   latest.last_reject_reason;
          return false;
        }
        if (
          latest.accepted_result_count > initial.accepted_result_count &&
          latest.has_map_to_odom)
        {
          detail = "bridge accepted result gate_mode=" + latest.gate_mode +
                   " accept_reason=" + latest.last_accept_reason +
                   " has_map_to_odom=true";
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = "failure_code=BRIDGE_ACCEPT_TIMEOUT bridge did not accept localization_result";
    if (latest.available) {
      detail += " last_reject_reason=" + latest.last_reject_reason;
      if (latest.last_reject_reason.find("tf_history_missing") != std::string::npos) {
        detail = "failure_code=TF_HISTORY_MISSING " + latest.last_reject_reason;
      }
    }
    return false;
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
      if (
        latest.available &&
        latest.has_map_to_odom &&
        latest.owner == "robot_localization_bridge" &&
        latest.map_to_odom_age_ms >= 0.0 &&
        latest.map_to_odom_age_ms <= map_to_odom_max_age_ms_ &&
        map_to_odom_tf_available())
      {
        detail = "map->odom ready owner=robot_localization_bridge age_ms=" +
                 std::to_string(latest.map_to_odom_age_ms);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = "failure_code=MAP_TO_ODOM_TIMEOUT map->odom unavailable or stale";
    if (latest.available) {
      detail += " has_map_to_odom=" + std::string(latest.has_map_to_odom ? "true" : "false") +
                " owner=" + latest.owner +
                " age_ms=" + std::to_string(latest.map_to_odom_age_ms);
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
  std::string map_frame_;
  std::string odom_frame_;
  double service_timeout_sec_{10.0};
  double service_call_timeout_sec_{10.0};
  double result_wait_timeout_sec_{20.0};
  double bridge_accept_timeout_sec_{8.0};
  double map_to_odom_wait_timeout_sec_{8.0};
  double map_to_odom_max_age_ms_{1000.0};
  bool require_grid_search_trigger_{true};
  bool require_bridge_acceptance_{true};
  bool mock_mode_{false};
  std::string last_trigger_status_{"idle"};
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
