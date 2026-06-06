#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

namespace
{

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("robot_global_localization")
  {
    declare_parameter<bool>("mock_mode", false);
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<std::string>("pose_topic", "/global_localization/pose");
    declare_parameter<std::string>("health_topic", "/global_localization/health");
    declare_parameter<std::string>("default_floor_id", "floor_1");
    declare_parameter<std::string>(
      "grid_search_trigger_service", "/trigger_grid_search_localization");
    declare_parameter<double>("service_timeout_sec", 10.0);
    declare_parameter<bool>("require_grid_search_trigger", true);

    active_floor_id_ = get_parameter("default_floor_id").as_string();
    grid_search_trigger_service_ = get_parameter("grid_search_trigger_service").as_string();
    service_timeout_sec_ = get_parameter("service_timeout_sec").as_double();
    require_grid_search_trigger_ = get_parameter("require_grid_search_trigger").as_bool();
    mock_mode_ = get_parameter("mock_mode").as_bool();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::QoS(10));
    health_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("health_topic").as_string(), rclcpp::QoS(10));
    grid_search_trigger_client_ = create_client<std_srvs::srv::Empty>(
      grid_search_trigger_service_, rmw_qos_profile_services_default, callback_group_);

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
  void on_trigger(
    const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request> request,
    std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response)
  {
    const auto wait_time = std::chrono::duration<double>(
      std::min(service_timeout_sec_, 1.0));
    if (!grid_search_trigger_client_->wait_for_service(wait_time)) {
      response->accepted = !require_grid_search_trigger_;
      response->message = "service unavailable: " + grid_search_trigger_service_;
      return;
    }

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    grid_search_trigger_client_->async_send_request(
      empty_request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        try {
          (void)future.get();
          last_trigger_status_ = "grid_search_trigger_dispatched";
          RCLCPP_INFO(get_logger(), "%s", last_trigger_status_.c_str());
        } catch (const std::exception & exc) {
          last_trigger_status_ = std::string("grid_search_trigger_failed: ") + exc.what();
          RCLCPP_ERROR(get_logger(), "%s", last_trigger_status_.c_str());
        }
      });

    last_trigger_status_ = "grid_search_trigger_pending: " + request->reason;
    response->accepted = true;
    response->message = "grid search localization trigger dispatched: " + request->reason;
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

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr grid_search_trigger_client_;
  rclcpp::Service<robot_interfaces::srv::TriggerLocalization>::SharedPtr trigger_srv_;
  rclcpp::Service<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr apply_floor_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string active_floor_id_;
  std::string active_nav_map_yaml_;
  std::string active_localizer_map_png_;
  std::string active_localizer_params_yaml_;
  std::string grid_search_trigger_service_;
  double service_timeout_sec_{10.0};
  bool require_grid_search_trigger_{true};
  bool mock_mode_{false};
  std::string last_trigger_status_{"idle"};
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
