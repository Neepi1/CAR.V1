// Synthetic inputs, real installed Humble costmap filters and MPPI critics.
// Run only in a private network/device-isolated container (no robot goals).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <gtest/gtest.h>
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"
#include "robot_nav_config/chassis_dynamics/motion_model.hpp"

namespace
{
// Replace only the sensor boundary; layer composition/inflation remain native.
class SyntheticObstacles : public nav2_costmap_2d::Layer
{
public:
  bool wall{false};
  void reset() override {}
  bool isClearable() override {return true;}
  void updateBounds(double x, double y, double, double * a, double * b,
    double * c, double * d) override
  {
    *a = std::min(*a, x - 5); *b = std::min(*b, y - 5);
    *c = std::max(*c, x + 5); *d = std::max(*d, y + 5);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int, int, int, int) override
  {
    if (!wall) {return;}
    for (double x = -3.0; x < 3.0; x += 0.05) {
      unsigned int mx, my;
      if (grid.worldToMap(x, 2.025, mx, my)) {grid.setCost(mx, my, 254);}
    }
  }
};

class KeepoutTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE,
      "-r", "__node:=local_costmap", "-r", "__ns:=/local_costmap"});
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"local_inflation_layer"}));
    if (std::getenv("NJRH_TEST_UNINFLATED_KEEPOUT")) {
      map->set_parameter(rclcpp::Parameter("filters", std::vector<std::string>{"keepout_filter"}));
    }
    ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
    ASSERT_EQ(map->getGlobalFrameID(), "odom");
    layered = map->getLayeredCostmap();
    source = std::make_shared<SyntheticObstacles>();
    layered->getPlugins()->insert(layered->getPlugins()->begin(), source);
    layered->resizeMap(200, 200, 0.05, -5, -5);
    // Activate only the fixture's filters; updateMap is stepped deterministically.
    for (auto & filter : *layered->getFilters()) {filter->activate();}
    transform(0, 0, 0);
    executor->add_node(map->get_node_base_interface());
    publisher = std::make_shared<rclcpp::Node>("synthetic_keepout");
    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    info_pub = publisher->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      "/costmap_filter_info/keepout", qos);
    mask_pub = publisher->create_publisher<nav_msgs::msg::OccupancyGrid>("/keepout_filter_mask", qos);
    nav2_msgs::msg::CostmapFilterInfo info;
    info.type = 0; info.base = 0; info.multiplier = 1;
    info.filter_mask_topic = "/keepout_filter_mask";
    info_pub->publish(info);
    mask.header.frame_id = "map";
    mask.info.resolution = 0.05;
    mask.info.width = mask.info.height = 200;
    mask.info.origin.position.x = mask.info.origin.position.y = -5;
    mask.info.origin.orientation.w = 1;
    mask.data.assign(40000, 0);
    mask_pub->publish(mask);
    auto filter = std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(layered->getFilters()->at(0));
    ASSERT_TRUE(filter);
    ASSERT_TRUE(wait([&] {return filter->isActive();}));
    layered->updateMap(0, 0, 0);
  }
  void TearDown() override
  {
    executor->remove_node(map->get_node_base_interface());
    for (auto & filter : *layered->getFilters()) {filter->deactivate();}
    map->on_cleanup(rclcpp_lifecycle::State{});
  }
  template<class F> bool wait(F ready)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      executor->spin_some();
      if (ready()) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }
  void transform(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom"; tf.child_frame_id = "map";
    tf.transform.translation.x = x; tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2);
    tf.transform.rotation.w = std::cos(yaw / 2);
    ASSERT_TRUE(map->getTfBuffer()->setTransform(tf, "fixture", true));
  }
  void cell(double x, double y, int8_t value)
  {
    const auto mx = static_cast<unsigned int>((x + 5) / 0.05);
    const auto my = static_cast<unsigned int>((y + 5) / 0.05);
    mask.data.at(my * 200 + mx) = value;
  }
  unsigned char cost(double x, double y)
  {
    unsigned int mx, my;
    if (!map->getCostmap()->worldToMap(x, y, mx, my)) {throw std::runtime_error("outside fixture");}
    return map->getCostmap()->getCost(mx, my);
  }
  bool publish_until(double x, double y, unsigned char value)
  {
    mask.header.stamp = publisher->now();
    mask_pub->publish(mask);
    return wait([&] {layered->updateMap(0, 0, 0); return cost(x, y) == value;});
  }
  std::vector<unsigned char> grid()
  {
    auto * cm = map->getCostmap();
    return {cm->getCharMap(), cm->getCharMap() + 40000};
  }
  bool collision(double x, double y, double yaw)
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE});
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    node->declare_parameter("FollowPath.critics", std::vector<std::string>{});
    node->set_parameter(rclcpp::Parameter("FollowPath.critics", std::vector<std::string>{"ObstaclesCritic"}));
    mppi::ParametersHandler handler(node);
    mppi::CriticManager critic;
    critic.on_configure(node, "FollowPath", map, &handler);
    mppi::models::State state; state.reset(1, 48);
    mppi::models::Trajectories paths; paths.reset(1, 48);
    paths.x.fill(x); paths.y.fill(y); paths.yaws.fill(yaw);
    mppi::models::Path path; path.x = {0.0F, 4.0F}; path.y = {0.0F, 0.0F};
    path.yaws = {0.0F, 0.0F};
    xt::xtensor<float, 1> costs = xt::zeros<float>({1});
    float dt = 1.0F / 15;
    mppi::CriticData data{state, paths, path, costs, dt, false, nullptr, nullptr,
      std::nullopt, std::nullopt};
    critic.evalTrajectoriesScores(data);
    EXPECT_TRUE(std::isfinite(costs(0)));
    return data.fail_flag;
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  nav2_costmap_2d::LayeredCostmap * layered{};
  std::shared_ptr<SyntheticObstacles> source;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<rclcpp::Node> publisher;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr info_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub;
  nav_msgs::msg::OccupancyGrid mask;
};

TEST_F(KeepoutTest, NativeMppiSeesBodyEdgeWhenCenterIsNotLethal)
{
  // Use the loaded padded long edge, not the superseded 0.39/0.28 fixture.
  double front = 0;
  for (const auto & point : map->getRobotFootprint()) {
    front = std::max(front, std::abs(point.x));
  }
  // Centre the pose within a cell: float yaw must not move a perimeter vertex
  // across an exact cell boundary and make a single-cell test ambiguous.
  const double edge = front + 0.025;
  cell(edge, 0.025, 100);
  ASSERT_TRUE(publish_until(edge, 0.025, 254));
  EXPECT_LT(cost(0.025, 0.025), 254);
  EXPECT_TRUE(collision(0.025, 0.025, 0));
  EXPECT_FALSE(collision(-0.8, 0, 0));
  // The long side rotates with the robot, not with a circular approximation.
  mask.data.assign(40000, 0);
  cell(0.025, edge, 100);
  ASSERT_TRUE(publish_until(0.025, edge, 254));
  EXPECT_FALSE(collision(0.025, 0.025, 0));
  EXPECT_TRUE(collision(0.025, 0.025, M_PI_2));
}

TEST_F(KeepoutTest, MaskAddMoveDeleteAndFrameTransformLeaveNoGhostCosts)
{
  source->wall = true;
  layered->updateMap(0, 0, 0);
  const auto empty = grid();
  cell(0.375, 0.025, 100);
  ASSERT_TRUE(publish_until(0.375, 0.025, 254));
  EXPECT_GT(cost(0.025, 0.025), 0);
  mask.data.assign(40000, 0);
  cell(-1.025, 0.025, 100);
  ASSERT_TRUE(publish_until(-1.025, 0.025, 254));
  EXPECT_EQ(cost(0.375, 0.025), 0);
  transform(1, 0, M_PI_2);
  layered->updateMap(0, 0, 0);
  EXPECT_EQ(cost(0.975, -1.025), 254);
  EXPECT_EQ(cost(-1.025, 0.025), 0);
  mask.data.assign(40000, 0);
  ASSERT_TRUE(publish_until(0.975, -1.025, 0));
  EXPECT_EQ(grid(), empty);
}

TEST_F(KeepoutTest, NeutralMaskPreservesObstacleInflationAndMeasuresOverhead)
{
  source->wall = true;
  auto filters = *layered->getFilters();
  std::vector<unsigned char> baseline;
  for (bool enabled : {false, true}) {
    *layered->getFilters() = enabled ? filters : std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>{};
    std::vector<double> times;
    for (int i = 0; i < 50; ++i) {
      const auto start = std::chrono::steady_clock::now();
      layered->updateMap(0, 0, 0);
      times.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count());
    }
    if (!enabled) {baseline = grid();} else {EXPECT_EQ(grid(), baseline);}
    std::sort(times.begin(), times.end());
    std::cout << "KEEPOUT_UPDATE filters=" << enabled << " median_ms=" << times[25]
      << " p96_ms=" << times[47] << " max_ms=" << times.back() << std::endl;
  }
  source->wall = false;
  layered->updateMap(0, 0, 0);
  EXPECT_EQ(cost(0.025, 2.025), 0);
}

TEST(NativeConstraintCritic, ScoresFiniteAtRestAndPenalizesRealViolationsWithRangerModel)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  node->declare_parameter("FollowPath.critics", std::vector<std::string>{});
  node->set_parameter(rclcpp::Parameter("FollowPath.critics", std::vector<std::string>{"ConstraintCritic"}));
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("constraint_map");
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
  mppi::ParametersHandler handler(node);
  mppi::CriticManager critic;
  critic.on_configure(node, "FollowPath", map, &handler);
  auto model = std::make_shared<robot_nav_config::chassis_dynamics::RangerMotionModel>(
    &handler, "FollowPath", robot_nav_config::chassis_dynamics::Parameters{},
    1.0 / 15, 0.55, 0.95, 0.90, 1.10);
  mppi::models::State state; state.reset(7, 48);
  const float vx[] = {0, 1e-8F, 0.4F, 0.4F, 0.4F, 1.4F, -0.2F};
  const float wz[] = {0, 0, 0, 0.2F, 1.0F, 0, 0};
  for (int row = 0; row < 7; ++row) {
    for (int t = 0; t < 48; ++t) {state.vx(row, t) = vx[row]; state.wz(row, t) = wz[row];}
  }
  mppi::models::Trajectories trajectories; trajectories.reset(7, 48);
  mppi::models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({7});
  float dt = 1.0F / 15;
  mppi::CriticData data{state, trajectories, path, costs, dt, false, nullptr, model,
    std::nullopt, std::nullopt};
  critic.evalTrajectoriesScores(data);
  for (int row = 0; row < 7; ++row) {EXPECT_TRUE(std::isfinite(costs(row))) << row;}
  for (int row = 0; row < 4; ++row) {EXPECT_FLOAT_EQ(costs(row), 0) << row;}
  for (int row = 4; row < 7; ++row) {EXPECT_GT(costs(row), 0) << row;}
  EXPECT_FALSE(data.fail_flag);
  map->on_cleanup(rclcpp_lifecycle::State{});
}
}  // namespace
