#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "robot_api_server/features/localization/tf_pose_utils.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{

using namespace std::chrono_literals;

class NavigationTerminalRuntimeModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    NavigationTerminalControl & terminal_control,
    NavigationTerminalRuntimeConfig config,
    NavigationTerminalRuntimePorts ports)
  : node_(node),
    logger_(node.get_logger()),
    terminal_control_(terminal_control),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
    if (!ports_.running || !ports_.cancel_requested || !ports_.safety_hard_blocked ||
      !ports_.verify_final_pose || !ports_.update_final_pose || !ports_.set_job_phase ||
      !ports_.publish_motion_mode || !ports_.current_robot_pose || !ports_.pose_in_frame ||
      !ports_.dock_contact_blocked)
    {
      throw std::invalid_argument("navigation terminal runtime requires every integration port");
    }

    command_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      config_.command_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    if (!config_.reverse_enable_topic.empty()) {
      reverse_enable_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
        config_.reverse_enable_topic, rclcpp::QoS(1));
    }
    if (config_.speed_limit_enabled && !config_.speed_limit_topic.empty()) {
      speed_limit_pub_ = node_.create_publisher<nav2_msgs::msg::SpeedLimit>(
        config_.speed_limit_topic,
        rclcpp::QoS(1).reliable().transient_local());
    }

    mode_controller_status_sub_ = node_.create_subscription<std_msgs::msg::String>(
      config_.mode_controller_status_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_mode_controller_status(msg->data);
      });
    if ((config_.yaw_actual_stop_check_enabled || config_.terminal_settle_enabled) &&
      !config_.actual_stop_odom_topic.empty())
    {
      actual_stop_odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.actual_stop_odom_topic,
        rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          handle_actual_stop_odom(msg);
        });
    }
    local_costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      config_.local_costmap_topic,
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        handle_local_costmap(msg);
      });
    rosout_sub_ = node_.create_subscription<rcl_interfaces::msg::Log>(
      "/rosout",
      rclcpp::QoS(100),
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        handle_rosout_message(msg);
      });
  }

  void publish_command(const geometry_msgs::msg::Twist & command)
  {
    command_pub_->publish(command);
  }

  void publish_zero_burst()
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < config_.zero_command_count; ++i) {
      publish_command(zero);
      std::this_thread::sleep_for(40ms);
    }
  }

  void publish_speed_limit_for_goal(const StoredPose & target)
  {
    if (!config_.speed_limit_enabled || !speed_limit_pub_) {
      return;
    }
    const auto pose = ports_.current_robot_pose();
    if (!pose.available || pose.frame_id != config_.map_frame ||
      pose.age_sec > config_.robot_pose_freshness_sec)
    {
      return;
    }
    const double distance_m = std::hypot(target.x - pose.x, target.y - pose.y);
    publish_speed_limit_value(terminal_control_.speed_limit_for_distance(distance_m));
  }

  void clear_speed_limit()
  {
    publish_speed_limit_value(config_.speed_limit_far_mps);
  }

  void update_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context)
  {
    if (!config_.navigation_reverse_permit_enabled || !reverse_enable_pub_) {
      clear_reverse_permit(permit_active, context + ":disabled");
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady < next_refresh_at) {
      return;
    }
    next_refresh_at = now_steady +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.reverse_permit_refresh_period_sec));

    const auto pose = ports_.current_robot_pose();
    const bool pose_usable = pose.available && pose.frame_id == config_.map_frame &&
      pose.age_sec <= config_.robot_pose_freshness_sec;
    const double distance_m = pose_usable ?
      std::hypot(target.x - pose.x, target.y - pose.y) : -1.0;
    const double distance_limit_m = permit_active ?
      config_.reverse_permit_exit_distance_m : config_.reverse_permit_enter_distance_m;
    const bool should_enable = pose_usable && distance_m <= distance_limit_m;

    if (!should_enable) {
      clear_reverse_permit(
        permit_active,
        context + (pose_usable ? ":outside_window" : ":pose_unavailable"));
      return;
    }

    publish_navigation_reverse_permit(true);
    if (!permit_active) {
      RCLCPP_INFO(
        logger_,
        "navigation terminal reverse permit enabled context=%s distance_m=%.3f enter_m=%.3f exit_m=%.3f",
        context.c_str(),
        distance_m,
        config_.reverse_permit_enter_distance_m,
        config_.reverse_permit_exit_distance_m);
    }
    permit_active = true;
  }

  void clear_reverse_permit(bool & permit_active, const std::string & context)
  {
    if (!permit_active) {
      return;
    }
    publish_navigation_reverse_permit(false);
    permit_active = false;
    RCLCPP_INFO(
      logger_,
      "navigation terminal reverse permit cleared context=%s",
      context.c_str());
  }

  ModeControllerStatusSnapshot mode_controller_status_snapshot() const
  {
    ModeControllerStatusSnapshot snapshot;
    std::lock_guard<std::mutex> lock(mode_controller_status_mutex_);
    if (latest_mode_controller_status_.empty()) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.raw = latest_mode_controller_status_;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_mode_controller_status_received_at_).count();
    if (const auto actual = json_object_value(snapshot.raw, "actual_motion_mode")) {
      snapshot.actual_available = json_bool_value(*actual, "available", false);
      snapshot.actual_fresh = json_bool_value(*actual, "fresh", false);
      snapshot.actual_motion_mode_code = static_cast<int>(
        json_number_value(*actual, "code").value_or(255.0));
    }
    snapshot.mode_aligned = json_bool_value(
      snapshot.raw,
      "mode_aligned",
      json_bool_value(snapshot.raw, "motion_mode_matched", false));
    return snapshot;
  }

  void reset_yaw_actual_stop_stability()
  {
    if (!config_.yaw_actual_stop_check_enabled) {
      return;
    }
    std::lock_guard<std::mutex> lock(yaw_actual_stop_mutex_);
    yaw_actual_wz_stable_sample_count_ = 0;
  }

  bool wait_for_yaw_actual_stop(const std::string & context, std::string & detail) const
  {
    if (!config_.yaw_actual_stop_check_enabled) {
      detail = "disabled";
      return true;
    }
    const auto timeout = std::chrono::milliseconds(config_.yaw_actual_stop_timeout_ms);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      if (yaw_actual_wz_stable_snapshot(detail)) {
        return true;
      }
      if (timeout.count() <= 0) {
        break;
      }
      std::this_thread::sleep_for(20ms);
    } while (std::chrono::steady_clock::now() < deadline && ports_.running());

    (void)context;
    return yaw_actual_wz_stable_snapshot(detail);
  }

  void reset_actual_stop_stability()
  {
    if (!config_.terminal_settle_enabled) {
      return;
    }
    std::lock_guard<std::mutex> lock(terminal_actual_stop_mutex_);
    terminal_actual_stop_stable_since_ = {};
  }

  bool wait_for_actual_stop(
    const std::string & context,
    std::string & detail,
    const bool require_dual_ackermann_mode) const
  {
    if (!config_.terminal_settle_enabled) {
      detail = "disabled";
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.terminal_settle_timeout_sec));
    do {
      if (actual_stop_stable_snapshot(detail, require_dual_ackermann_mode)) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    } while (std::chrono::steady_clock::now() < deadline && ports_.running());

    const bool stable = actual_stop_stable_snapshot(detail, require_dual_ackermann_mode);
    if (!stable) {
      detail = context + ": " + detail;
    }
    return stable;
  }

  bool wait_for_actual_stop(const std::string & context, std::string & detail) const
  {
    return wait_for_actual_stop(
      context, detail, config_.terminal_settle_require_dual_ackermann_mode);
  }

  std::uint64_t local_costmap_update_count() const
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    return local_costmap_update_count_;
  }

  std::uint64_t local_costmap_message_filter_drop_count() const
  {
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    return local_costmap_message_filter_drop_count_;
  }

  std::string last_local_costmap_message_filter_drop_text() const
  {
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    return last_local_costmap_message_filter_drop_text_;
  }

  TerminalTimePoint terminal_now() const
  {
    return std::chrono::steady_clock::now();
  }

  void terminal_sleep_for(const std::chrono::milliseconds duration)
  {
    std::this_thread::sleep_for(duration);
  }

  bool terminal_cancel_requested(const std::uint64_t job_id, std::string & detail)
  {
    return ports_.cancel_requested(job_id, detail);
  }

  bool terminal_safety_hard_blocked(std::string & detail)
  {
    return ports_.safety_hard_blocked(detail);
  }

  FinalPoseCheck terminal_verify_final_pose(
    const StoredPose & target,
    const bool require_fresh_pose)
  {
    return ports_.verify_final_pose(target, require_fresh_pose);
  }

  void terminal_update_final_pose(
    const std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason)
  {
    ports_.update_final_pose(job_id, check, reason);
  }

  void terminal_set_job_phase(
    const std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail)
  {
    ports_.set_job_phase(job_id, phase, detail);
  }

  void terminal_publish_motion_mode(const std::string & mode)
  {
    ports_.publish_motion_mode(mode);
  }

  bool terminal_reverse_permit_available() const
  {
    return static_cast<bool>(reverse_enable_pub_);
  }

  void terminal_publish_reverse_permit(const bool enabled)
  {
    if (!reverse_enable_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled && config_.post_nav2_reverse_permit_enabled;
    reverse_enable_pub_->publish(msg);
  }

  TerminalCostmapContext terminal_costmap_context(const TerminalTimePoint now_steady)
  {
    TerminalCostmapContext context;
    {
      std::lock_guard<std::mutex> lock(local_costmap_mutex_);
      context.grid = latest_local_costmap_;
      context.grid_received_at = latest_local_costmap_received_at_;
    }
    if (context.grid) {
      context.grid_frame = normalized_frame_id(context.grid->header.frame_id);
    }
    if (context.grid_frame == config_.base_frame || context.grid_frame == "base_footprint") {
      context.robot_pose_available = true;
      context.robot_pose_received_at = now_steady;
    } else {
      const auto pose = ports_.pose_in_frame(context.grid_frame);
      if (pose.available) {
        context.robot_pose_available = true;
        context.robot_x = pose.x;
        context.robot_y = pose.y;
        context.robot_yaw = pose.yaw;
        context.robot_pose_received_at = pose.received_at;
      }
    }
    return context;
  }

  RobotPoseSnapshot terminal_current_robot_pose()
  {
    return ports_.current_robot_pose();
  }

  bool terminal_dock_contact_blocked(std::string & detail)
  {
    return ports_.dock_contact_blocked(detail);
  }

  void terminal_warn(const std::string & warning)
  {
    RCLCPP_WARN(logger_, "%s", warning.c_str());
  }

private:
  void publish_speed_limit_value(const double speed_limit_mps)
  {
    if (!speed_limit_pub_) {
      return;
    }
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = config_.map_frame;
    msg.percentage = false;
    msg.speed_limit = std::max(0.0, speed_limit_mps);
    speed_limit_pub_->publish(msg);
  }

  void publish_navigation_reverse_permit(const bool enabled)
  {
    if (!reverse_enable_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled && config_.navigation_reverse_permit_enabled;
    reverse_enable_pub_->publish(msg);
  }

  void handle_mode_controller_status(const std::string & status)
  {
    std::lock_guard<std::mutex> lock(mode_controller_status_mutex_);
    latest_mode_controller_status_ = status;
    latest_mode_controller_status_received_at_ = std::chrono::steady_clock::now();
  }

  void handle_actual_stop_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double wz = msg->twist.twist.angular.z;
    {
      std::lock_guard<std::mutex> lock(yaw_actual_stop_mutex_);
      latest_yaw_actual_wz_radps_ = wz;
      latest_yaw_actual_wz_received_at_ = now;
      if (std::isfinite(wz) && std::abs(wz) <= config_.yaw_actual_wz_threshold_radps) {
        ++yaw_actual_wz_stable_sample_count_;
      } else {
        yaw_actual_wz_stable_sample_count_ = 0;
      }
    }

    if (config_.terminal_settle_enabled) {
      const bool finite = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
      const double linear_speed = finite ?
        std::hypot(vx, vy) : std::numeric_limits<double>::infinity();
      const bool stopped = finite &&
        linear_speed <= config_.terminal_settle_linear_speed_threshold_mps &&
        std::abs(wz) <= config_.terminal_settle_angular_speed_threshold_radps;
      std::lock_guard<std::mutex> lock(terminal_actual_stop_mutex_);
      latest_terminal_actual_vx_mps_ = vx;
      latest_terminal_actual_vy_mps_ = vy;
      latest_terminal_actual_wz_radps_ = wz;
      latest_terminal_actual_stop_received_at_ = now;
      if (stopped) {
        if (terminal_actual_stop_stable_since_.time_since_epoch().count() == 0) {
          terminal_actual_stop_stable_since_ = now;
        }
      } else {
        terminal_actual_stop_stable_since_ = {};
      }
    }
  }

  bool yaw_actual_wz_stable_snapshot(std::string & detail) const
  {
    if (!config_.yaw_actual_stop_check_enabled) {
      detail = "disabled";
      return true;
    }
    std::lock_guard<std::mutex> lock(yaw_actual_stop_mutex_);
    if (latest_yaw_actual_wz_received_at_.time_since_epoch().count() == 0) {
      detail = "no odom sample";
      return false;
    }
    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_yaw_actual_wz_received_at_).count();
    const bool fresh = age_sec <= config_.yaw_actual_wz_max_age_sec;
    const bool stable = fresh &&
      yaw_actual_wz_stable_sample_count_ >= config_.yaw_actual_wz_stable_samples;
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "wz=" << latest_yaw_actual_wz_radps_
        << " stable_samples=" << yaw_actual_wz_stable_sample_count_
        << "/" << config_.yaw_actual_wz_stable_samples
        << " age_sec=" << age_sec
        << " threshold=" << config_.yaw_actual_wz_threshold_radps;
    detail = out.str();
    return stable;
  }

  bool actual_stop_stable_snapshot(
    std::string & detail,
    const bool require_dual_ackermann_mode) const
  {
    if (!config_.terminal_settle_enabled) {
      detail = "disabled";
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    double odom_age_sec = -1.0;
    double stable_duration_sec = 0.0;
    bool have_odom = false;
    {
      std::lock_guard<std::mutex> lock(terminal_actual_stop_mutex_);
      have_odom = latest_terminal_actual_stop_received_at_.time_since_epoch().count() != 0;
      vx = latest_terminal_actual_vx_mps_;
      vy = latest_terminal_actual_vy_mps_;
      wz = latest_terminal_actual_wz_radps_;
      if (have_odom) {
        odom_age_sec = std::chrono::duration<double>(
          now - latest_terminal_actual_stop_received_at_).count();
      }
      if (terminal_actual_stop_stable_since_.time_since_epoch().count() != 0) {
        stable_duration_sec = std::chrono::duration<double>(
          now - terminal_actual_stop_stable_since_).count();
      }
    }

    const double linear_speed = std::hypot(vx, vy);
    const bool odom_fresh = have_odom &&
      odom_age_sec <= config_.terminal_settle_odom_max_age_sec;
    const bool odom_stable = odom_fresh && std::isfinite(linear_speed) && std::isfinite(wz) &&
      linear_speed <= config_.terminal_settle_linear_speed_threshold_mps &&
      std::abs(wz) <= config_.terminal_settle_angular_speed_threshold_radps &&
      stable_duration_sec >= config_.terminal_settle_stable_duration_sec;

    const auto mode_status = mode_controller_status_snapshot();
    const bool mode_fresh = mode_status.available && mode_status.actual_available &&
      mode_status.actual_fresh && mode_status.age_sec >= 0.0 &&
      mode_status.age_sec <= config_.terminal_settle_mode_status_max_age_sec;
    const bool mode_exited = !require_dual_ackermann_mode ||
      (mode_fresh && mode_status.actual_motion_mode_code == 0 && mode_status.mode_aligned);

    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "linear_speed=" << linear_speed
        << " wz=" << wz
        << " stable_duration_sec=" << stable_duration_sec
        << "/" << config_.terminal_settle_stable_duration_sec
        << " odom_age_sec=" << odom_age_sec
        << " mode_required=" << (require_dual_ackermann_mode ? "true" : "false")
        << " mode_available=" << (mode_status.available ? "true" : "false")
        << " mode_fresh=" << (mode_fresh ? "true" : "false")
        << " actual_mode=" << mode_status.actual_motion_mode_code
        << " mode_aligned=" << (mode_status.mode_aligned ? "true" : "false")
        << " odom_stable=" << (odom_stable ? "true" : "false")
        << " mode_exited=" << (mode_exited ? "true" : "false");
    detail = out.str();
    return odom_stable && mode_exited;
  }

  void handle_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    latest_local_costmap_ = msg;
    ++local_costmap_update_count_;
    latest_local_costmap_received_at_ = std::chrono::steady_clock::now();
  }

  void handle_rosout_message(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    const std::string combined = msg->name + ": " + msg->msg;
    if (combined.find("Message Filter dropping") == std::string::npos) {
      return;
    }
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    ++message_filter_drop_count_;
    last_message_filter_drop_text_ = combined;
    if (combined.find("local_costmap") != std::string::npos ||
      combined.find("controller_server") != std::string::npos)
    {
      ++local_costmap_message_filter_drop_count_;
      last_local_costmap_message_filter_drop_text_ = combined;
    }
  }

  rclcpp::Node & node_;
  rclcpp::Logger logger_;
  NavigationTerminalControl & terminal_control_;
  NavigationTerminalRuntimeConfig config_;
  NavigationTerminalRuntimePorts ports_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_enable_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_controller_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr actual_stop_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  mutable std::mutex mode_controller_status_mutex_;
  std::string latest_mode_controller_status_;
  std::chrono::steady_clock::time_point latest_mode_controller_status_received_at_{};
  mutable std::mutex yaw_actual_stop_mutex_;
  double latest_yaw_actual_wz_radps_{0.0};
  int yaw_actual_wz_stable_sample_count_{0};
  std::chrono::steady_clock::time_point latest_yaw_actual_wz_received_at_{};
  mutable std::mutex terminal_actual_stop_mutex_;
  double latest_terminal_actual_vx_mps_{0.0};
  double latest_terminal_actual_vy_mps_{0.0};
  double latest_terminal_actual_wz_radps_{0.0};
  std::chrono::steady_clock::time_point latest_terminal_actual_stop_received_at_{};
  std::chrono::steady_clock::time_point terminal_actual_stop_stable_since_{};
  mutable std::mutex local_costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_local_costmap_;
  std::uint64_t local_costmap_update_count_{0U};
  std::chrono::steady_clock::time_point latest_local_costmap_received_at_{};
  mutable std::mutex rosout_mutex_;
  std::uint64_t message_filter_drop_count_{0U};
  std::uint64_t local_costmap_message_filter_drop_count_{0U};
  std::string last_message_filter_drop_text_;
  std::string last_local_costmap_message_filter_drop_text_;
};

NavigationTerminalRuntimeModule::NavigationTerminalRuntimeModule(
  rclcpp::Node & node,
  NavigationTerminalControl & terminal_control,
  NavigationTerminalRuntimeConfig config,
  NavigationTerminalRuntimePorts ports)
: impl_(std::make_unique<Impl>(
      node, terminal_control, std::move(config), std::move(ports)))
{
}

NavigationTerminalRuntimeModule::~NavigationTerminalRuntimeModule() = default;

void NavigationTerminalRuntimeModule::publish_command(
  const geometry_msgs::msg::Twist & command)
{
  impl_->publish_command(command);
}

void NavigationTerminalRuntimeModule::publish_zero_burst()
{
  impl_->publish_zero_burst();
}

void NavigationTerminalRuntimeModule::publish_speed_limit_for_goal(const StoredPose & target)
{
  impl_->publish_speed_limit_for_goal(target);
}

void NavigationTerminalRuntimeModule::clear_speed_limit()
{
  impl_->clear_speed_limit();
}

void NavigationTerminalRuntimeModule::update_reverse_permit_for_goal(
  const StoredPose & target,
  bool & permit_active,
  std::chrono::steady_clock::time_point & next_refresh_at,
  const std::string & context)
{
  impl_->update_reverse_permit_for_goal(target, permit_active, next_refresh_at, context);
}

void NavigationTerminalRuntimeModule::clear_reverse_permit(
  bool & permit_active,
  const std::string & context)
{
  impl_->clear_reverse_permit(permit_active, context);
}

ModeControllerStatusSnapshot NavigationTerminalRuntimeModule::mode_controller_status_snapshot()
const
{
  return impl_->mode_controller_status_snapshot();
}

void NavigationTerminalRuntimeModule::reset_yaw_actual_stop_stability()
{
  impl_->reset_yaw_actual_stop_stability();
}

bool NavigationTerminalRuntimeModule::wait_for_yaw_actual_stop(
  const std::string & context,
  std::string & detail) const
{
  return impl_->wait_for_yaw_actual_stop(context, detail);
}

void NavigationTerminalRuntimeModule::reset_actual_stop_stability()
{
  impl_->reset_actual_stop_stability();
}

bool NavigationTerminalRuntimeModule::wait_for_actual_stop(
  const std::string & context,
  std::string & detail,
  const bool require_dual_ackermann_mode) const
{
  return impl_->wait_for_actual_stop(context, detail, require_dual_ackermann_mode);
}

bool NavigationTerminalRuntimeModule::wait_for_actual_stop(
  const std::string & context,
  std::string & detail) const
{
  return impl_->wait_for_actual_stop(context, detail);
}

std::uint64_t NavigationTerminalRuntimeModule::local_costmap_update_count() const
{
  return impl_->local_costmap_update_count();
}

std::uint64_t NavigationTerminalRuntimeModule::local_costmap_message_filter_drop_count() const
{
  return impl_->local_costmap_message_filter_drop_count();
}

std::string NavigationTerminalRuntimeModule::last_local_costmap_message_filter_drop_text() const
{
  return impl_->last_local_costmap_message_filter_drop_text();
}

TerminalTimePoint NavigationTerminalRuntimeModule::terminal_now() const
{
  return impl_->terminal_now();
}

void NavigationTerminalRuntimeModule::terminal_sleep_for(
  const std::chrono::milliseconds duration)
{
  impl_->terminal_sleep_for(duration);
}

bool NavigationTerminalRuntimeModule::terminal_cancel_requested(
  const std::uint64_t job_id,
  std::string & detail)
{
  return impl_->terminal_cancel_requested(job_id, detail);
}

bool NavigationTerminalRuntimeModule::terminal_safety_hard_blocked(std::string & detail)
{
  return impl_->terminal_safety_hard_blocked(detail);
}

FinalPoseCheck NavigationTerminalRuntimeModule::terminal_verify_final_pose(
  const StoredPose & target,
  const bool require_fresh_pose)
{
  return impl_->terminal_verify_final_pose(target, require_fresh_pose);
}

void NavigationTerminalRuntimeModule::terminal_update_final_pose(
  const std::uint64_t job_id,
  const FinalPoseCheck & check,
  const std::string & reason)
{
  impl_->terminal_update_final_pose(job_id, check, reason);
}

void NavigationTerminalRuntimeModule::terminal_set_job_phase(
  const std::uint64_t job_id,
  const std::string & phase,
  const std::string & detail)
{
  impl_->terminal_set_job_phase(job_id, phase, detail);
}

void NavigationTerminalRuntimeModule::terminal_publish_command(
  const geometry_msgs::msg::Twist & command)
{
  impl_->publish_command(command);
}

void NavigationTerminalRuntimeModule::terminal_publish_motion_mode(const std::string & mode)
{
  impl_->terminal_publish_motion_mode(mode);
}

bool NavigationTerminalRuntimeModule::terminal_reverse_permit_available() const
{
  return impl_->terminal_reverse_permit_available();
}

void NavigationTerminalRuntimeModule::terminal_publish_reverse_permit(const bool enabled)
{
  impl_->terminal_publish_reverse_permit(enabled);
}

void NavigationTerminalRuntimeModule::terminal_reset_actual_stop_stability()
{
  impl_->reset_actual_stop_stability();
}

bool NavigationTerminalRuntimeModule::terminal_wait_actual_stop(
  const std::string & context,
  std::string & detail)
{
  return impl_->wait_for_actual_stop(context, detail);
}

TerminalCostmapContext NavigationTerminalRuntimeModule::terminal_costmap_context(
  const TerminalTimePoint now)
{
  return impl_->terminal_costmap_context(now);
}

RobotPoseSnapshot NavigationTerminalRuntimeModule::terminal_current_robot_pose()
{
  return impl_->terminal_current_robot_pose();
}

bool NavigationTerminalRuntimeModule::terminal_dock_contact_blocked(std::string & detail)
{
  return impl_->terminal_dock_contact_blocked(detail);
}

void NavigationTerminalRuntimeModule::terminal_reset_yaw_actual_stop_stability()
{
  impl_->reset_yaw_actual_stop_stability();
}

bool NavigationTerminalRuntimeModule::terminal_wait_yaw_actual_stop(
  const std::string & context,
  std::string & detail)
{
  return impl_->wait_for_yaw_actual_stop(context, detail);
}

void NavigationTerminalRuntimeModule::terminal_warn(const std::string & warning)
{
  impl_->terminal_warn(warning);
}

}  // namespace robot_api_server::features::navigation
