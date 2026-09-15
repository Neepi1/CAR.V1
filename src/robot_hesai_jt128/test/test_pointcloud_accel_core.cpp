#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_hesai_jt128/pointcloud_accel_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_hesai_jt128
{
namespace
{

using Cloud = sensor_msgs::msg::PointCloud2;
using Scan = sensor_msgs::msg::LaserScan;
using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
constexpr double kAngleIncrement = kPi / 720.0;
constexpr char kFrame[] = "pointcloud_core_test_lidar";

sensor_msgs::msg::PointField field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

void store_float(Cloud & cloud, std::size_t point, std::size_t offset, float value)
{
  std::memcpy(cloud.data.data() + point * cloud.point_step + offset, &value, sizeof(value));
}

float load_float(const Cloud & cloud, std::size_t point, std::size_t offset)
{
  float value{};
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(value));
  return value;
}

// Four synthetic returns: two visible, one inside the unchanged mask, one above
// the slice. Extra ring/time fields and padding exercise the full-field trunk.
Cloud make_cloud(std::int32_t seconds, float first_x, float first_y)
{
  Cloud cloud;
  cloud.header.frame_id = "pointcloud_core_test_vendor";
  cloud.header.stamp.sec = seconds;
  cloud.header.stamp.nanosec = 123456789U;
  cloud.height = 1U;
  cloud.width = 4U;
  cloud.point_step = 32U;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  using F = sensor_msgs::msg::PointField;
  cloud.fields = {
    field("x", 0U, F::FLOAT32), field("y", 4U, F::FLOAT32),
    field("z", 8U, F::FLOAT32), field("intensity", 16U, F::FLOAT32),
    field("ring", 20U, F::UINT16), field("timestamp", 24U, F::FLOAT64)};
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0U; i < cloud.data.size(); ++i) {
    cloud.data[i] = static_cast<std::uint8_t>((i * 17U + 5U) % 251U);
  }
  const float xyz[4][3] = {
    {first_x, first_y, 0.10F}, {0.10F, 0.30F, 0.10F},
    {-3.0F, 0.5F, 2.0F}, {-2.0F, 3.0F, 0.20F}};
  for (std::size_t i = 0U; i < 4U; ++i) {
    store_float(cloud, i, 0U, xyz[i][0]);
    store_float(cloud, i, 4U, xyz[i][1]);
    store_float(cloud, i, 8U, xyz[i][2]);
    store_float(cloud, i, 16U, 10.0F + static_cast<float>(i));
  }
  return cloud;
}

Cloud expected_trunk(const Cloud & input)
{
  Cloud expected = input;
  expected.header.frame_id = kFrame;
  for (std::size_t i = 0U; i < input.width; ++i) {
    store_float(expected, i, 0U, load_float(input, i, 4U));
    store_float(expected, i, 4U, -load_float(input, i, 0U));
  }
  return expected;
}

void expect_scan(const Scan & scan, const Cloud & input)
{
  EXPECT_EQ(scan.header.stamp, input.header.stamp);
  EXPECT_EQ(scan.header.frame_id, kFrame);
  EXPECT_FLOAT_EQ(scan.scan_time, 1.0F / 15.0F);
  ASSERT_EQ(scan.ranges.size(), 1440U);
  const auto expected = expected_trunk(input);
  std::vector<float> ranges(1440U, std::numeric_limits<float>::infinity());
  // Only the two visible endpoints survive the existing height/mask rules.
  for (const std::size_t point : {0U, 3U}) {
    const double x = load_float(expected, point, 0U);
    const double y = load_float(expected, point, 4U);
    const auto bin = static_cast<std::size_t>((std::atan2(y, x) + kPi) / kAngleIncrement);
    ASSERT_LT(bin, ranges.size());
    ranges[bin] = static_cast<float>(std::hypot(x, y));
  }
  for (std::size_t i = 0U; i < ranges.size(); ++i) {
    if (std::isfinite(ranges[i])) {
      EXPECT_NEAR(scan.ranges[i], ranges[i], 1.0e-6F) << "bin=" << i;
    } else {
      EXPECT_TRUE(std::isinf(scan.ranges[i])) << "unexpected return in bin=" << i;
    }
  }
}

class PointCloudAccelCoreIntegration : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // The test runner must select an isolated non-production DDS domain.
    // TFListener creates its own node; therefore these remaps must be GLOBAL,
    // not only NodeOptions attached to the node passed into AccelCore.
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    ASSERT_NE(domain, nullptr) << "Run only with an explicit isolated ROS_DOMAIN_ID";
    ASSERT_GT(std::strtol(domain, nullptr, 10), 0L);
    ASSERT_FALSE(rclcpp::ok()) << "Dedicated test executable requires its own ROS context";
    const char * args[] = {
      "test_pointcloud_accel_core", "--ros-args",
      "-r", "/tf:=/pointcloud_core_test/tf",
      "-r", "/tf_static:=/pointcloud_core_test/tf_static",
      "-r", "/rosout:=/pointcloud_core_test/rosout",
      "-r", "/parameter_events:=/pointcloud_core_test/parameter_events"};
    rclcpp::init(static_cast<int>(sizeof(args) / sizeof(args[0])), args);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    static std::atomic<unsigned int> sequence{0U};
    prefix_ = "/pointcloud_core_test/case_" + std::to_string(++sequence);
  }

  void start(bool missing_mask_tf = false)
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false).start_parameter_event_publisher(false).enable_rosout(false);
    options.parameter_overrides({
      rclcpp::Parameter("input_topic", prefix_ + "/unused_input"),
      rclcpp::Parameter("output_topic", prefix_ + "/trunk"),
      rclcpp::Parameter("output_frame_id", std::string{kFrame}),
      rclcpp::Parameter("accel_profile", "ipc_worker"),
      rclcpp::Parameter("status_topic", prefix_ + "/status"),
      rclcpp::Parameter("accel_status_topic", prefix_ + "/accel_status"),
      rclcpp::Parameter("status_publish_period_sec", 0.10),
      rclcpp::Parameter("scan_output_topic", prefix_ + "/scan"),
      rclcpp::Parameter("flatscan_output_topic", prefix_ + "/unused_flatscan"),
      rclcpp::Parameter("local_worker_stamp_odom_topic", prefix_ + "/unused_odom"),
      rclcpp::Parameter("scan_worker_frame_id", std::string{kFrame}),
      rclcpp::Parameter("scan_worker_self_mask_frame_id", missing_mask_tf ?
        "pointcloud_core_test_absent_mask_frame" : std::string{kFrame}),
      rclcpp::Parameter("scan_worker_self_mask_enabled", true),
      rclcpp::Parameter("scan_worker_rate_hz", 15.0),
      rclcpp::Parameter("scan_worker_angle_min", -kPi),
      rclcpp::Parameter("scan_worker_angle_max", kPi),
      rclcpp::Parameter("scan_worker_angle_increment", kAngleIncrement),
      rclcpp::Parameter("rotation_matrix", std::vector<double>{
        0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0})});
    producer_ = std::make_shared<rclcpp::Node>("unit_under_test", prefix_, options);
    rclcpp::NodeOptions observer_options;
    observer_options.start_parameter_services(false).start_parameter_event_publisher(false).
      enable_rosout(false);
    observer_ = std::make_shared<rclcpp::Node>("observer", prefix_, observer_options);
    trunk_subscription_ = observer_->create_subscription<Cloud>(
      prefix_ + "/trunk", rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr message) {trunks_.push_back(std::move(message));});
    scan_subscription_ = observer_->create_subscription<Scan>(
      prefix_ + "/scan", rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr message) {scans_.push_back(std::move(message));});
    status_subscription_ = observer_->create_subscription<std_msgs::msg::String>(
      prefix_ + "/accel_status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr message) {status_ = message->data;});
    core_ = std::make_unique<PointCloudAccelCore>(*producer_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(producer_);
    executor_->add_node(observer_);
    ASSERT_TRUE(wait_for([this] {
      return trunk_subscription_->get_publisher_count() == 1U &&
             scan_subscription_->get_publisher_count() == 1U &&
             producer_->count_subscribers(prefix_ + "/scan") == 1U;
    })) << "Private DDS endpoints did not discover one another";
  }

  bool wait_for(const std::function<bool()> & predicate)
  {
    const auto deadline = SteadyClock::now() + std::chrono::seconds(8);
    while (SteadyClock::now() < deadline && rclcpp::ok()) {
      executor_->spin_some(std::chrono::milliseconds(5));
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  void TearDown() override
  {
    core_.reset();  // Joins scan worker before releasing the node or ROS context.
    if (executor_) {
      executor_->cancel();
      executor_.reset();
    }
    status_subscription_.reset();
    scan_subscription_.reset();
    trunk_subscription_.reset();
    observer_.reset();
    producer_.reset();
  }

  std::string prefix_;
  rclcpp::Node::SharedPtr producer_, observer_;
  std::unique_ptr<PointCloudAccelCore> core_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<Cloud>::SharedPtr trunk_subscription_;
  rclcpp::Subscription<Scan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  std::vector<Cloud::ConstSharedPtr> trunks_;
  std::vector<Scan::ConstSharedPtr> scans_;
  std::string status_;
};

TEST_F(PointCloudAccelCoreIntegration, PreservesTrunkFieldsAndStampsAndScansNormalizedFrames)
{
  ASSERT_NO_FATAL_FAILURE(start());
  const auto first = make_cloud(123, 1.0F, 2.0F);
  core_->process_pointcloud2(std::make_unique<Cloud>(first));
  ASSERT_TRUE(wait_for([this] {return trunks_.size() == 1U && scans_.size() == 1U;}));
  EXPECT_EQ(*trunks_.front(), expected_trunk(first));
  expect_scan(*scans_.front(), first);
  const auto first_trunk = trunks_.front();
  const auto first_scan = scans_.front();
  const Cloud retained_trunk = *first_trunk;
  const Scan retained_scan = *first_scan;

  const auto second = make_cloud(124, 4.0F, 1.0F);
  core_->process_pointcloud2(Cloud{second});
  ASSERT_TRUE(wait_for([this] {return trunks_.size() == 2U && scans_.size() == 2U;}));
  EXPECT_EQ(*trunks_.back(), expected_trunk(second));
  expect_scan(*scans_.back(), second);
  EXPECT_EQ(*first_trunk, retained_trunk);
  EXPECT_EQ(*first_scan, retained_scan);

  // The first old normalized buffer becomes reusable only after frame two.
  // Frames three and four exercise both sides of the core's reuse cycle.
  const auto third = make_cloud(125, -1.0F, 4.0F);
  core_->process_pointcloud2(std::make_unique<Cloud>(third));
  ASSERT_TRUE(wait_for([this] {return trunks_.size() == 3U && scans_.size() == 3U;}));
  EXPECT_EQ(*trunks_.back(), expected_trunk(third));
  expect_scan(*scans_.back(), third);
  EXPECT_EQ(*first_trunk, retained_trunk);
  EXPECT_EQ(*first_scan, retained_scan);

  const auto fourth = make_cloud(126, 3.0F, -2.0F);
  core_->process_pointcloud2(Cloud{fourth});
  ASSERT_TRUE(wait_for([this] {return trunks_.size() == 4U && scans_.size() == 4U;}));
  EXPECT_EQ(*trunks_.back(), expected_trunk(fourth));
  expect_scan(*scans_.back(), fourth);
  EXPECT_EQ(*first_trunk, retained_trunk);
  EXPECT_EQ(*first_scan, retained_scan);
}

TEST_F(PointCloudAccelCoreIntegration, MissingMaskTransformWithholdsScanButPreservesTrunk)
{
  ASSERT_NO_FATAL_FAILURE(start(true));
  const auto input = make_cloud(125, 1.0F, 2.0F);
  core_->process_pointcloud2(std::make_unique<Cloud>(input));
  ASSERT_TRUE(wait_for([this] {
    const std::string key = "scan_self_mask_tf_unavailable_count=";
    const auto position = status_.find(key);
    return trunks_.size() == 1U && position != std::string::npos &&
           std::strtoull(status_.c_str() + position + key.size(), nullptr, 10) > 0U;
  })) << "Expected real scan-worker missing-TF diagnostic, not only absence of messages";
  EXPECT_EQ(*trunks_.front(), expected_trunk(input));
  EXPECT_TRUE(scans_.empty());
}

}  // namespace
}  // namespace robot_hesai_jt128
