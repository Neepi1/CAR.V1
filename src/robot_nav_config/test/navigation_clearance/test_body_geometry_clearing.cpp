// Synthetic observations through public APIs; real installed Humble ObstacleLayer.
// Execute only in the project's network/device-isolated test container.
// This does not replay scans, publish commands, or activate a costmap update thread.
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

class BodyGeometryClearingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE,
      "-r", "__node:=local_costmap", "-r", "__ns:=/local_costmap"});
    map_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    map_->set_parameter(rclcpp::Parameter("plugins",
      std::vector<std::string>{"obstacle_layer"}));
    map_->set_parameter(rclcpp::Parameter("filters", std::vector<std::string>{}));
    // Replace only the sensor seam: no real /scan subscription or scan-to-cloud
    // path. Native marking, footprint transformation, raster clearing and merge
    // are exercised below by the normal layered-costmap update entry point.
    const std::string sources = "obstacle_layer.observation_sources";
    if (!map_->has_parameter(sources)) {
      map_->declare_parameter(sources, std::string{});
    }
    ASSERT_TRUE(map_->set_parameter(rclcpp::Parameter(sources, "")).successful);
    ASSERT_EQ(map_->on_configure(rclcpp_lifecycle::State{}),
      nav2_util::CallbackReturn::SUCCESS);
    configured_ = true;
    ASSERT_EQ(map_->getGlobalFrameID(), "odom");
    ASSERT_TRUE(map_->get_parameter("obstacle_layer.footprint_clearing_enabled").as_bool());
    layered_ = map_->getLayeredCostmap();
    ASSERT_EQ(layered_->getPlugins()->size(), 1U);
    obstacle_ = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(
      layered_->getPlugins()->front());
    ASSERT_TRUE(obstacle_);
    layered_->resizeMap(200, 200, 0.05, -5.0, -5.0);
    ASSERT_EQ(map_->getRobotFootprint().size(), 4U);
    // Candidate config, not a hardcoded replacement footprint in the fixture.
    for (const auto & point : map_->getRobotFootprint()) {
      // Native parser stores float precision, despite the Point double fields.
      ASSERT_NEAR(std::abs(point.x), 0.50, 1e-6);
      ASSERT_NEAR(std::abs(point.y), 0.39, 1e-6);
    }
  }

  void TearDown() override
  {
    if (obstacle_) {obstacle_->clearStaticObservations(true, true);}
    if (configured_) {map_->on_cleanup(rclcpp_lifecycle::State{});}
    obstacle_.reset();
    map_.reset();
  }

  void observe(std::initializer_list<std::pair<double, double>> points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = static_cast<float>(point.first);
      *y = static_cast<float>(point.second);
      *z = 0.1F;
      ++x; ++y; ++z;
    }
    geometry_msgs::msg::Point origin;
    origin.z = 0.1;
    nav2_costmap_2d::Observation observation(origin, cloud, 5.5, 0.25, 8.0, 0.20);
    obstacle_->clearStaticObservations(true, true);
    obstacle_->addStaticObservation(observation, true, false);
  }

  unsigned char cost(double x, double y)
  {
    unsigned int mx = 0, my = 0;
    if (!map_->getCostmap()->worldToMap(x, y, mx, my)) {
      ADD_FAILURE() << "fixture point outside map: " << x << ',' << y;
      return nav2_costmap_2d::NO_INFORMATION;
    }
    return map_->getCostmap()->getCost(mx, my);
  }

  bool configured_{false};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> obstacle_;
  nav2_costmap_2d::LayeredCostmap * layered_{nullptr};
};

TEST_F(BodyGeometryClearingTest, EnlargedCurrentBodyClearsInsideButPreservesOutside)
{
  // Cell centres in the newly added body band, not ambiguous polygon edges.
  observe({{0.425, 0.025}, {0.025, 0.325}, {0.575, 0.025}, {0.025, 0.475}});
  layered_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0.425, 0.025), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.025, 0.325), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.575, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.025, 0.475), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Isolated control proves the inside observations were valid and that native
  // footprint clearing (not source range/height rejection) removed them.
  ASSERT_TRUE(map_->set_parameter(rclcpp::Parameter(
    "obstacle_layer.footprint_clearing_enabled", false)).successful);
  layered_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0.425, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.025, 0.325), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(map_->set_parameter(rclcpp::Parameter(
    "obstacle_layer.footprint_clearing_enabled", true)).successful);
  layered_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0.425, 0.025), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.025, 0.325), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(BodyGeometryClearingTest, ClearingRectangleRotatesWithCurrentPose)
{
  observe({{0.475, 0.025}, {0.025, 0.475}});
  layered_->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0.475, 0.025), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(0.025, 0.475), nav2_costmap_2d::LETHAL_OBSTACLE);
  layered_->updateMap(0.0, 0.0, std::acos(-1.0) / 2.0);
  EXPECT_EQ(cost(0.475, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(0.025, 0.475), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(BodyGeometryClearingTest, CurrentClearingDoesNotEraseFuturePoseEdgeObstacle)
{
  // This point is outside the current body's clearing polygon but on the front
  // edge of a future body centred at (1.025, 0.025), away from grid boundaries.
  // No future-pose clearing is performed.
  observe({{1.525, 0.025}});
  layered_->updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(cost(1.525, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(
    map_->getCostmap());
  EXPECT_LT(checker.footprintCostAtPose(0.0, 0.0, 0.0, map_->getRobotFootprint()),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(checker.footprintCostAtPose(1.025, 0.025, 0.0, map_->getRobotFootprint()),
    nav2_costmap_2d::LETHAL_OBSTACLE);
}

}  // namespace
