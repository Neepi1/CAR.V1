// Device/network-isolated tests only. No goals, vehicle commands or live inputs.
// The configured geometry and native inflation/MPPI critic are the test subjects;
// only the external obstacle observations are replaced with synthetic cells.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"

namespace
{
constexpr double kHalfX = 0.47;  // User-confirmed physical envelope in base_link.
constexpr double kHalfY = 0.36;
constexpr double kPadding = 0.03;
constexpr double kPoseX = 1.025;
constexpr double kPoseY = 1.025;
constexpr double kPi = 3.14159265358979323846;

const char * config_file()
{
  const char * override_file = std::getenv("NJRH_BODY_GEOMETRY_CONFIG");
  return override_file ? override_file : NAV_CONFIG_FILE;
}

class ObservedCells : public nav2_costmap_2d::Layer
{
public:
  std::vector<std::pair<double, double>> points;
  void reset() override {}
  bool isClearable() override {return true;}
  void updateBounds(double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    *min_x = std::min(*min_x, -5.0); *min_y = std::min(*min_y, -5.0);
    *max_x = std::max(*max_x, 5.0); *max_y = std::max(*max_y, 5.0);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int, int, int, int) override
  {
    // Observed test area is free except listed occupied samples. Do not change
    // default_value_: LayeredCostmap::isTrackingUnknown() derives its policy
    // directly from that value in the installed Humble implementation.
    std::fill(grid.getCharMap(), grid.getCharMap() +
      grid.getSizeInCellsX() * grid.getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
    for (const auto & point : points) {
      unsigned int mx, my;
      if (grid.worldToMap(point.first, point.second, mx, my)) {
        grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
};

class BodyGeometry : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    const std::string name = GetParam() + "_costmap";
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", config_file(),
      "-r", "__node:=" + name, "-r", "__ns:=/" + name});
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    inflation_name = GetParam() == "local" ? "local_inflation_layer" : "global_inflation_layer";
    // Remove live sensor/map/keepout inputs, NOT footprint, padding, resolution,
    // unknown-space policy or native inflation parameters from the YAML.
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{inflation_name}));
    map->set_parameter(rclcpp::Parameter("filters", std::vector<std::string>{}));
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
    configured = true;
    layered = map->getLayeredCostmap();
    observed = std::make_shared<ObservedCells>();
    layered->getPlugins()->insert(layered->getPlugins()->begin(), observed);
    const double resolution = map->get_parameter("resolution").as_double();
    layered->resizeMap(200, 200, resolution, -5, -5);
    rclcpp::NodeOptions controller_options;
    controller_options.arguments({"--ros-args", "--params-file", config_file()});
    controller = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "controller_server", controller_options);
    controller->declare_parameter("FollowPath.critics", std::vector<std::string>{});
    controller->set_parameter(rclcpp::Parameter(
      "FollowPath.critics", std::vector<std::string>{"ObstaclesCritic"}));
    handler = std::make_unique<mppi::ParametersHandler>(controller);
    manager = std::make_unique<mppi::CriticManager>();
    manager->on_configure(controller, "FollowPath", map, handler.get());
  }
  void TearDown() override
  {
    manager.reset(); handler.reset(); controller.reset();
    if (configured) {map->on_cleanup(rclcpp_lifecycle::State{});}
  }
  void update()
  {
    // Simulated current pose is away from the future trajectory being scored.
    layered->updateMap(0, 0, 0);
  }
  void body_patch(double bx, double by, double yaw)
  {
    observed->points.clear();
    // A small occupied patch straddling the new padded perimeter. Rasterization
    // uses native 5 cm cells, rather than an idealized point-in-polygon test.
    for (double dx : {-0.035, 0.0, 0.035}) {
      for (double dy : {-0.035, 0.0, 0.035}) {
        observed->points.emplace_back(
          kPoseX + std::cos(yaw) * (bx + dx) - std::sin(yaw) * (by + dy),
          kPoseY + std::sin(yaw) * (bx + dx) + std::cos(yaw) * (by + dy));
      }
    }
    update();
  }
  unsigned char cost(double x, double y)
  {
    unsigned int mx, my;
    if (!map->getCostmap()->worldToMap(x, y, mx, my)) {
      throw std::runtime_error("test sample outside synthetic costmap");
    }
    return map->getCostmap()->getCost(mx, my);
  }
  bool fails(double x, double y, double yaw)
  {
    mppi::models::State state; state.reset(1, 48);
    state.pose.pose.orientation.w = 1;
    mppi::models::Trajectories trajectories; trajectories.reset(1, 48);
    trajectories.x.fill(x); trajectories.y.fill(y); trajectories.yaws.fill(yaw);
    mppi::models::Path path;
    path.x = {0.0F, 4.0F}; path.y = {0.0F, 0.0F}; path.yaws = {0.0F, 0.0F};
    xt::xtensor<float, 1> costs = xt::zeros<float>({1});
    float dt = 1.0F / 15;
    mppi::CriticData data{state, trajectories, path, costs, dt, false,
      nullptr, nullptr, std::nullopt, std::nullopt};
    manager->evalTrajectoriesScores(data);
    EXPECT_TRUE(std::isfinite(costs(0)));
    return data.fail_flag;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  nav2_costmap_2d::LayeredCostmap * layered{};
  std::shared_ptr<ObservedCells> observed;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> controller;
  std::unique_ptr<mppi::ParametersHandler> handler;
  std::unique_ptr<mppi::CriticManager> manager;
  std::string inflation_name;
  bool configured{false};
};

TEST_P(BodyGeometry, ConfiguredFootprintCoversPhysicalBodyWithRetainedPadding)
{
  std::vector<geometry_msgs::msg::Point> raw;
  ASSERT_TRUE(nav2_costmap_2d::makeFootprintFromString(
    map->get_parameter("footprint").as_string(), raw));
  ASSERT_EQ(raw.size(), 4U);
  for (const auto & point : raw) {
    // Humble parses footprint coordinates through float before storing double.
    EXPECT_NEAR(std::abs(point.x), kHalfX, 1e-6);
    EXPECT_NEAR(std::abs(point.y), kHalfY, 1e-6);
  }
  EXPECT_DOUBLE_EQ(map->get_parameter("footprint_padding").as_double(), kPadding);
  for (const auto & point : map->getRobotFootprint()) {
    EXPECT_NEAR(std::abs(point.x), kHalfX + kPadding, 1e-6);
    EXPECT_NEAR(std::abs(point.y), kHalfY + kPadding, 1e-6);
  }
  EXPECT_DOUBLE_EQ(map->get_parameter(inflation_name + ".inflation_radius").as_double(), 0.75);
  EXPECT_TRUE(controller->get_parameter("FollowPath.ObstaclesCritic.consider_footprint").as_bool());
  EXPECT_DOUBLE_EQ(controller->get_parameter("FollowPath.ObstaclesCritic.inflation_radius").as_double(), 0.75);
}

TEST_P(BodyGeometry, NativeCriticRejectsFrontRearSideAndRotatedCornerEvenWhenCenterIsNotLethal)
{
  const std::vector<std::pair<double, double>> edges{
    {0.49, 0}, {-0.49, 0}, {0, 0.38}, {0, -0.38},
    {0.49, 0.38}, {0.49, -0.38}, {-0.49, 0.38}, {-0.49, -0.38}};
  for (const double yaw : {0.0, kPi / 4, kPi / 2, -kPi / 2}) {
    for (const auto & edge : edges) {
      SCOPED_TRACE(::testing::Message() << "map=" << GetParam() << " yaw=" << yaw
        << " edge=" << edge.first << "," << edge.second);
      body_patch(edge.first, edge.second, yaw);
      ASSERT_LT(cost(kPoseX, kPoseY), nav2_costmap_2d::LETHAL_OBSTACLE);
      EXPECT_TRUE(fails(kPoseX, kPoseY, yaw));
      EXPECT_FALSE(fails(-1.5, -1.5, yaw));
    }
  }
}

TEST_P(BodyGeometry, ClearSpaceAndRemovedObstacleRemainTraversable)
{
  update();
  for (const double yaw : {0.0, kPi / 4, kPi / 2}) {
    EXPECT_FALSE(fails(kPoseX, kPoseY, yaw));
  }
  body_patch(0.49, 0, 0);
  ASSERT_TRUE(fails(kPoseX, kPoseY, 0));
  observed->points.clear();
  update();
  EXPECT_FALSE(fails(kPoseX, kPoseY, 0));
}

TEST_P(BodyGeometry, NativeUnknownAndOutOfBoundsPolicyIsNotChanged)
{
  update();
  auto * grid = map->getCostmap();
  // Entire footprint and its center are unknown, avoiding partial-perimeter
  // ambiguity. This deliberately tests installed Humble's existing behavior,
  // not a newly invented fail-closed policy for this configuration correction.
  for (double x = 0.2; x <= 1.8; x += 0.025) {
    for (double y = 0.2; y <= 1.8; y += 0.025) {
      unsigned int mx, my;
      ASSERT_TRUE(grid->worldToMap(x, y, mx, my));
      grid->setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
    }
  }
  const bool tracks_unknown = layered->isTrackingUnknown();
  EXPECT_EQ(tracks_unknown, map->get_parameter("track_unknown_space").as_bool());
  EXPECT_EQ(fails(kPoseX, kPoseY, 0), !tracks_unknown);
  EXPECT_EQ(fails(20, 20, 0), !tracks_unknown);
}

INSTANTIATE_TEST_SUITE_P(ProductionYaml, BodyGeometry,
  ::testing::Values(std::string("local"), std::string("global")));
}  // namespace
