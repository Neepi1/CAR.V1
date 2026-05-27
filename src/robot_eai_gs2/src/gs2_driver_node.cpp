#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "CYdLidar.h"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

namespace robot_eai_gs2
{

class Gs2DriverNode final : public rclcpp::Node
{
public:
  Gs2DriverNode()
  : Node("gs2_driver_node")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/gs2");
    serial_baudrate_ = declare_parameter<int>("serial_baudrate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gs2_link");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/dock/gs2_scan");
    point_cloud_topic_ = declare_parameter<std::string>("point_cloud_topic", "/dock/gs2_points");
    publish_point_cloud_ = declare_parameter<bool>("publish_point_cloud", true);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    fail_on_initialize_error_ = declare_parameter<bool>("fail_on_initialize_error", true);
    use_ros_time_stamp_ = declare_parameter<bool>("use_ros_time_stamp", true);
    invalid_range_is_inf_ = declare_parameter<bool>("invalid_range_is_inf", false);
    point_cloud_preservative_ = declare_parameter<bool>("point_cloud_preservative", false);
    angle_min_deg_ = declare_parameter<double>("angle_min_deg", -50.0);
    angle_max_deg_ = declare_parameter<double>("angle_max_deg", 50.0);
    range_min_m_ = declare_parameter<double>("range_min_m", 0.025);
    range_max_m_ = declare_parameter<double>("range_max_m", 0.30);
    sdk_range_scale_ = declare_parameter<double>("sdk_range_scale", 0.001);
    scan_frequency_hz_ = declare_parameter<double>("scan_frequency_hz", 8.0);
    fixed_resolution_ = declare_parameter<bool>("fixed_resolution", false);
    reversion_ = declare_parameter<bool>("reversion", false);
    inverted_ = declare_parameter<bool>("inverted", false);
    auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);
    single_channel_ = declare_parameter<bool>("single_channel", false);
    intensity_ = declare_parameter<bool>("intensity", true);
    tof_lidar_ = declare_parameter<bool>("tof_lidar", false);

    if (sdk_range_scale_ <= 0.0) {
      throw std::runtime_error("sdk_range_scale must be positive");
    }
    if (range_max_m_ <= range_min_m_) {
      throw std::runtime_error("range_max_m must be greater than range_min_m");
    }
    if (angle_max_deg_ <= angle_min_deg_) {
      throw std::runtime_error("angle_max_deg must be greater than angle_min_deg");
    }

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
    if (publish_point_cloud_) {
      point_cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud>(point_cloud_topic_, rclcpp::SensorDataQoS());
    }

    start_service_ = create_service<std_srvs::srv::Empty>(
      "start_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!initialized_ && !initializeLocked()) {
          RCLCPP_ERROR(get_logger(), "GS2 initialize failed on %s", serial_port_.c_str());
          return;
        }
        scanning_ = laser_.turnOn();
        RCLCPP_INFO(get_logger(), "GS2 scan %s", scanning_ ? "started" : "failed to start");
      });

    stop_service_ = create_service<std_srvs::srv::Empty>(
      "stop_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (initialized_) {
          laser_.turnOff();
        }
        scanning_ = false;
        RCLCPP_INFO(get_logger(), "GS2 scan stopped");
      });

    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      initialized_ = initializeLocked();
      if (initialized_ && auto_start_) {
        scanning_ = laser_.turnOn();
      }
    }

    if (!initialized_ && fail_on_initialize_error_) {
      throw std::runtime_error(
        "GS2 initialize failed: cannot open " + serial_port_ + " at " +
        std::to_string(serial_baudrate_) +
        ". Check serial_port, baudrate, permissions, and whether another process owns it.");
    }

    running_ = true;
    worker_ = std::thread(&Gs2DriverNode::pollLoop, this);

    if (initialized_) {
      RCLCPP_INFO(
        get_logger(),
        "GS2 driver ready: port=%s baud=%d frame=%s scan=%s range=[%.3f, %.3f]m angles=[%.1f, %.1f]deg",
        serial_port_.c_str(), serial_baudrate_, frame_id_.c_str(), scan_topic_.c_str(),
        range_min_m_, range_max_m_, angle_min_deg_, angle_max_deg_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "GS2 driver is not initialized: port=%s baud=%d. Call start_scan after fixing the device.",
        serial_port_.c_str(), serial_baudrate_);
    }
  }

  ~Gs2DriverNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    std::lock_guard<std::mutex> lock(driver_mutex_);
    if (initialized_) {
      laser_.turnOff();
      laser_.disconnecting();
    }
  }

private:
  bool initializeLocked()
  {
    laser_.setSerialPort(serial_port_);
    laser_.setSerialBaudrate(serial_baudrate_);
    laser_.setFixedResolution(fixed_resolution_);
    laser_.setReversion(reversion_);
    laser_.setInverted(inverted_);
    laser_.setAutoReconnect(auto_reconnect_);
    laser_.setSingleChannel(single_channel_);
    laser_.setLidarType(tof_lidar_ ? TYPE_TOF : TYPE_TRIANGLE);
    laser_.setMinAngle(static_cast<float>(angle_min_deg_));
    laser_.setMaxAngle(static_cast<float>(angle_max_deg_));
    laser_.setMinRange(static_cast<float>(range_min_m_ / sdk_range_scale_));
    laser_.setMaxRange(static_cast<float>(range_max_m_ / sdk_range_scale_));
    laser_.setScanFrequency(static_cast<float>(scan_frequency_hz_));
    laser_.setIntensity(intensity_);
    laser_.setIgnoreArray({});
    return laser_.initialize();
  }

  void pollLoop()
  {
    while (rclcpp::ok() && running_) {
      if (!initialized_ || !scanning_) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      LaserScan scan;
      bool hard_error = false;
      bool ok = false;
      {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (initialized_ && scanning_) {
          ok = laser_.doProcessSimple(scan, hard_error);
        }
      }

      if (ok) {
        publishScan(scan);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to read GS2 scan%s", hard_error ? " (hardware error)" : "");
      }
    }
  }

  rclcpp::Time scanStamp(const LaserScan & scan) const
  {
    if (use_ros_time_stamp_) {
      return now();
    }
    return rclcpp::Time(static_cast<int64_t>(scan.stamp), RCL_SYSTEM_TIME);
  }

  void publishScan(const LaserScan & scan)
  {
    if (scan.points.empty()) {
      return;
    }

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = scanStamp(scan);
    msg.header.frame_id = frame_id_;
    msg.angle_min = scan.config.min_angle;
    msg.angle_max = scan.config.max_angle;
    msg.angle_increment = scan.config.angle_increment;
    msg.scan_time = scan.config.scan_time;
    msg.time_increment = scan.config.time_increment;
    msg.range_min = static_cast<float>(range_min_m_);
    msg.range_max = static_cast<float>(range_max_m_);

    if (msg.angle_increment <= 0.0F) {
      msg.angle_increment = static_cast<float>((msg.angle_max - msg.angle_min) / 159.0);
    }
    const auto beam_count = std::max(
      1, static_cast<int>(std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1);
    const float invalid = invalid_range_is_inf_ ? std::numeric_limits<float>::infinity() : 0.0F;
    msg.ranges.assign(static_cast<size_t>(beam_count), invalid);
    msg.intensities.assign(static_cast<size_t>(beam_count), 0.0F);

    sensor_msgs::msg::PointCloud cloud;
    if (publish_point_cloud_) {
      cloud.header = msg.header;
      sensor_msgs::msg::ChannelFloat32 intensity_channel;
      intensity_channel.name = "intensities";
      sensor_msgs::msg::ChannelFloat32 stamp_channel;
      stamp_channel.name = "stamps";
      cloud.channels.push_back(intensity_channel);
      cloud.channels.push_back(stamp_channel);
    }

    for (size_t i = 0; i < scan.points.size(); ++i) {
      const auto & point = scan.points[i];
      const float range_m = static_cast<float>(point.range * sdk_range_scale_);
      if (range_m >= msg.range_min && range_m <= msg.range_max) {
        const int index = static_cast<int>(
          std::lround((point.angle - msg.angle_min) / msg.angle_increment));
        if (index >= 0 && index < beam_count) {
          msg.ranges[static_cast<size_t>(index)] = range_m;
          msg.intensities[static_cast<size_t>(index)] = point.intensity;
        }
      }

      if (publish_point_cloud_ &&
        (point_cloud_preservative_ || (range_m >= msg.range_min && range_m <= msg.range_max)))
      {
        geometry_msgs::msg::Point32 ros_point;
        ros_point.x = range_m * std::cos(point.angle);
        ros_point.y = range_m * std::sin(point.angle);
        ros_point.z = 0.0F;
        cloud.points.push_back(ros_point);
        cloud.channels[0].values.push_back(point.intensity);
        cloud.channels[1].values.push_back(static_cast<float>(i) * msg.time_increment);
      }
    }

    scan_pub_->publish(msg);
    if (publish_point_cloud_ && point_cloud_pub_) {
      point_cloud_pub_->publish(cloud);
    }
  }

  std::string serial_port_;
  int serial_baudrate_{921600};
  std::string frame_id_;
  std::string scan_topic_;
  std::string point_cloud_topic_;
  bool publish_point_cloud_{true};
  bool auto_start_{true};
  bool fail_on_initialize_error_{true};
  bool use_ros_time_stamp_{true};
  bool invalid_range_is_inf_{false};
  bool point_cloud_preservative_{false};
  double angle_min_deg_{-50.0};
  double angle_max_deg_{50.0};
  double range_min_m_{0.025};
  double range_max_m_{0.30};
  double sdk_range_scale_{0.001};
  double scan_frequency_hz_{8.0};
  bool fixed_resolution_{false};
  bool reversion_{false};
  bool inverted_{false};
  bool auto_reconnect_{true};
  bool single_channel_{false};
  bool intensity_{true};
  bool tof_lidar_{false};

  CYdLidar laser_;
  std::mutex driver_mutex_;
  std::atomic_bool running_{false};
  std::atomic_bool initialized_{false};
  std::atomic_bool scanning_{false};
  std::thread worker_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
};

}  // namespace robot_eai_gs2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_eai_gs2::Gs2DriverNode>());
  rclcpp::shutdown();
  return 0;
}
