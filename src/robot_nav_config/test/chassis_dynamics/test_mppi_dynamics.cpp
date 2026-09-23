#include <chrono>
#include <iostream>
#include <gtest/gtest.h>
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "robot_nav_config/chassis_dynamics/motion_model.hpp"
#include "robot_nav_config/chassis_dynamics/output_filter.hpp"

namespace
{
class PredictionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("response_test");
    node->declare_parameter("FollowPath.AckermannConstraints.min_turning_r", 0.81);
    handler = std::make_unique<mppi::ParametersHandler>(node);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<mppi::ParametersHandler> handler;
};
}

TEST_F(PredictionTest, NativeAndMeasuredModelHaveDifferentBrakingTrajectories)
{
  mppi::AckermannMotionModel native(handler.get(), "FollowPath");
  robot_nav_config::chassis_dynamics::RangerMotionModel measured(handler.get(), "FollowPath",
    {}, 1.0 / 15, 0.55, 0.95, 0.90, 1.10);
  mppi::models::State instant, actual;
  instant.reset(1, 48);
  actual.reset(1, 48);
  instant.vx(0, 0) = actual.vx(0, 0) = 1.2;
  native.predict(instant);
  measured.predict(actual);
  const double old_distance = xt::sum(instant.vx)() / 15;
  const double new_distance = xt::sum(actual.vx)() / 15;
  EXPECT_LT(old_distance, 0.1);
  EXPECT_GT(new_distance, 0.8); // Includes the separately modeled smoother ramp.
  EXPECT_LT(new_distance, 1.05);
  EXPECT_GT(actual.vx(0, 2), 1.0);
  EXPECT_LT(actual.vx(0, 47), 0.001);
}

TEST_F(PredictionTest, RolloutUsesFeedbackAndDoesNotMutateCommandsOrAccumulateAcrossCalls)
{
  robot_nav_config::chassis_dynamics::RangerMotionModel model(handler.get(), "FollowPath",
    {}, 1.0 / 15, 0.55, 0.95, 0.90, 1.10);
  mppi::models::State state;
  state.reset(2, 48);
  state.cvx.fill(0.6);
  state.cwz.fill(0.15);
  const auto commands = state.cvx;
  model.predict(state);
  const auto predicted = state.vx;
  model.predict(state);
  EXPECT_TRUE(xt::all(xt::equal(predicted, state.vx)));
  EXPECT_TRUE(xt::all(xt::equal(commands, state.cvx)));
  robot_nav_config::chassis_dynamics::PredictionInput input;
  input.smoother_valid = true;
  input.smoother_linear = 0.6;
  input.issued = {{-0.5, 0.6, 0}};
  model.set_input(input, 1.0 / 15, 0, 1.2, 0.7);
  model.predict(state);
  EXPECT_GT(state.vx(0, 2), predicted(0, 2));
}

TEST_F(PredictionTest, ProductionBatchRuntime)
{
  robot_nav_config::chassis_dynamics::RangerMotionModel model(handler.get(), "FollowPath",
    {}, 1.0 / 15, 0.55, 0.95, 0.90, 1.10);
  mppi::models::State state;
  state.reset(1200, 48);
  state.cvx.fill(0.6);
  state.cwz.fill(0.15);
  std::vector<double> timings;
  for (int i = 0; i < 30; ++i) {
    const auto start = std::chrono::steady_clock::now();
    model.predict(state);
    timings.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count());
  }
  std::sort(timings.begin(), timings.end());
  std::cout << "DYNAMICS_1200x48 median_ms=" << timings[15]
    << " p97_ms=" << timings[29] << std::endl;
}

TEST_F(PredictionTest, NativeObstacleCriticSeesResidualBrakingTravel)
{
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("braking_obstacle_map");
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"local_inflation_layer"}));
  map->declare_parameter("local_inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
  map->declare_parameter("local_inflation_layer.inflation_radius", 0.6);
  map->declare_parameter("local_inflation_layer.cost_scaling_factor", 6.0);
  map->set_parameter(rclcpp::Parameter("footprint", "[[0.36,0.25],[0.36,-0.25],[-0.36,-0.25],[-0.36,0.25]]"));
  map->set_parameter(rclcpp::Parameter("footprint_padding", 0.03));
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
  auto * grid = map->getCostmap();
  grid->resizeMap(200, 200, 0.05, -5, -5);
  grid->resetMap(0, 0, 200, 200);
  for (unsigned int x = 114; x < 130; ++x) {
    for (unsigned int y = 80; y < 120; ++y) {grid->setCost(x, y, 254);}
  }
  node->declare_parameter("FollowPath.critics", std::vector<std::string>{"ObstaclesCritic"});
  node->declare_parameter("FollowPath.ObstaclesCritic.inflation_radius", 0.6);
  node->declare_parameter("FollowPath.ObstaclesCritic.cost_scaling_factor", 6.0);
  node->declare_parameter("FollowPath.ObstaclesCritic.inflation_layer_name", "local_inflation_layer");
  node->declare_parameter("FollowPath.ObstaclesCritic.consider_footprint", true);
  mppi::CriticManager critic;
  critic.on_configure(node, "FollowPath", map, handler.get());
  robot_nav_config::chassis_dynamics::RangerMotionModel measured(handler.get(), "FollowPath",
    {}, 1.0 / 15, 0.55, 0.95, 0.90, 1.10);
  mppi::AckermannMotionModel native(handler.get(), "FollowPath");
  const auto collides = [&](mppi::MotionModel & model) {
    mppi::models::State state;
    state.reset(1, 48);
    state.vx(0, 0) = 1.2;
    state.pose.pose.orientation.w = 1;
    model.predict(state);
    mppi::models::Trajectories trajectory;
    trajectory.reset(1, 48);
    float dt = 1.0F / 15;
    float x = 0;
    for (std::size_t t = 0; t < 48; ++t) {x += state.vx(0, t) * dt; trajectory.x(0, t) = x;}
    mppi::models::Path path;
    path.x = {0.0F, 4.0F}; path.y = {0.0F, 0.0F}; path.yaws = {0.0F, 0.0F};
    xt::xtensor<float, 1> costs = xt::zeros<float>({1u});
    mppi::CriticData data{state, trajectory, path, costs, dt, false,
      nullptr, nullptr, std::nullopt, std::nullopt};
    critic.evalTrajectoriesScores(data);
    return data.fail_flag;
  };
  EXPECT_FALSE(collides(native));
  EXPECT_TRUE(collides(measured));
  // End this isolated test context before destroying Costmap2DROS's internal
  // executor: cancel-before-spin can otherwise race in a short-lived fixture.
  rclcpp::shutdown();
  map->on_cleanup(rclcpp_lifecycle::State{});
}

TEST_F(PredictionTest, NativeOutputFilterCanOvershootAlreadyBoundedControls)
{
  // Native-library baseline: the adapter must not inherit this filter overshoot.
  mppi::models::ControlSequence sequence;
  sequence.reset(48);
  sequence.vx.fill(0.2F);
  sequence.vx(4) = 0;
  std::array<mppi::models::Control, 4> history{};
  for (auto & command : history) {command.vx = 0.2F;}
  mppi::models::OptimizerSettings settings;
  settings.time_steps = 48;
  mppi::utils::savitskyGolayFilter(sequence, history, settings);
  EXPECT_GT(sequence.vx(0), 0.2F);
  EXPECT_LT(sequence.vx(0), 0.24F);
}

TEST_F(PredictionTest, FilteredForwardSequenceAndHistoryCannotBecomeReverse)
{
  mppi::AckermannMotionModel model(handler.get(), "FollowPath");
  for (const bool shift : {false, true}) {
    mppi::models::ControlSequence sequence;
    sequence.reset(48);
    std::array<mppi::models::Control, 4> history{};
    history[0].vx = 0.2F;
    mppi::models::OptimizerSettings settings;
    settings.time_steps = 48;
    settings.shift_control_sequence = shift;
    settings.constraints.vx_min = 0;
    settings.constraints.vx_max = 1.2F;
    settings.constraints.wz = 0.7F;
    robot_nav_config::chassis_dynamics::filter_control_sequence(sequence, history, settings, model);
    EXPECT_GE(xt::amin(sequence.vx)(), 0.0F);
    EXPECT_GE(history.back().vx, 0.0F);
    EXPECT_FLOAT_EQ(history.back().vx, sequence.vx(shift ? 1 : 0));
  }
}

TEST_F(PredictionTest, FilterProjectionHonorsLimitsRadiusAndIssuedHistory)
{
  mppi::AckermannMotionModel model(handler.get(), "FollowPath");
  for (const unsigned int steps : {8u, 48u}) {
    for (const bool shift : {false, true}) {
      for (const float minimum : {0.0F, -0.2F}) {
        mppi::models::ControlSequence sequence;
        sequence.reset(steps);
        for (unsigned int t = 0; t < steps; ++t) {
          sequence.vx(t) = t % 3 == 0 ? minimum : 0.2F;
          sequence.wz(t) = (t % 2 == 0 ? 1 : -1) * sequence.vx(t) / 0.81F;
        }
        std::array<mppi::models::Control, 4> history{};
        for (unsigned int t = 0; t < 4; ++t) {
          history[t].vx = 0.3F + t * 0.1F;
          history[t].wz = 0.1F + t * 0.01F;
        }
        const auto previous = history; // Real older commands may exceed a new limit.
        mppi::models::OptimizerSettings settings;
        settings.time_steps = steps;
        settings.shift_control_sequence = shift;
        settings.constraints.vx_min = minimum;
        settings.constraints.vx_max = 0.12F;
        settings.constraints.wz = 0.08F;
        robot_nav_config::chassis_dynamics::filter_control_sequence(sequence, history, settings, model);
        for (unsigned int t = 0; t < steps; ++t) {
          EXPECT_GE(sequence.vx(t), minimum);
          EXPECT_LE(sequence.vx(t), 0.12F);
          EXPECT_LE(std::abs(sequence.wz(t)), 0.08F);
          EXPECT_LE(std::abs(sequence.wz(t)), std::abs(sequence.vx(t)) / 0.81F + 1e-7F);
          EXPECT_FLOAT_EQ(sequence.vy(t), 0.0F);
          if (sequence.vx(t) == 0) {EXPECT_FLOAT_EQ(sequence.wz(t), 0.0F);}
        }
        for (unsigned int t = 0; t < 3; ++t) {
          EXPECT_FLOAT_EQ(history[t].vx, previous[t + 1].vx);
          EXPECT_FLOAT_EQ(history[t].wz, previous[t + 1].wz);
        }
        const unsigned int selected = shift ? 1 : 0;
        EXPECT_FLOAT_EQ(history.back().vx, sequence.vx(selected));
        EXPECT_FLOAT_EQ(history.back().wz, sequence.wz(selected));
        EXPECT_FLOAT_EQ(history.back().vy, 0.0F);
      }
    }
  }
}

TEST_F(PredictionTest, LegalSmallControlsAreNotSuppressedOrGivenReversePermission)
{
  mppi::AckermannMotionModel model(handler.get(), "FollowPath");
  for (const float v : {0.001F, -0.001F}) {
    mppi::models::ControlSequence sequence;
    sequence.reset(48);
    sequence.vx.fill(v);
    sequence.wz.fill(v / 2);
    std::array<mppi::models::Control, 4> history{};
    for (auto & command : history) {command = {v, 0, v / 2};}
    mppi::models::OptimizerSettings settings;
    settings.time_steps = 48;
    settings.shift_control_sequence = true;
    settings.constraints.vx_min = v < 0 ? -0.2F : 0.0F;
    settings.constraints.vx_max = 1.2F;
    settings.constraints.wz = 0.7F;
    robot_nav_config::chassis_dynamics::filter_control_sequence(sequence, history, settings, model);
    EXPECT_NEAR(sequence.vx(1), v, 1e-8F);
    EXPECT_NEAR(sequence.wz(1), v / 2, 1e-8F);
  }
}

TEST_F(PredictionTest, ProductionPluginLoadsHotUpdatesEnforcesSpeedLimitsAndPreservesNoControlFailure)
{
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("response_map");
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"local_inflation_layer"}));
  map->declare_parameter("local_inflation_layer.plugin", "nav2_costmap_2d::InflationLayer");
  map->declare_parameter("local_inflation_layer.inflation_radius", 0.6);
  map->declare_parameter("local_inflation_layer.cost_scaling_factor", 6.0);
  map->set_parameter(rclcpp::Parameter("footprint", "[[0.36,0.25],[0.36,-0.25],[-0.36,-0.25],[-0.36,0.25]]"));
  map->set_parameter(rclcpp::Parameter("footprint_padding", 0.03));
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State{}), nav2_util::CallbackReturn::SUCCESS);
  map->getCostmap()->resizeMap(200, 200, 0.05, -5, -5);
  map->getCostmap()->resetMap(0, 0, 200, 200);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", NAV_CONFIG_FILE});
  auto controller_node = std::make_shared<nav2_util::LifecycleNode>("controller_server", "", options);
  auto tf = std::make_shared<tf2_ros::Buffer>(controller_node->get_clock());
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(controller_node, "goal_checker", map);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance("robot_nav_config::RangerMPPIController");
  controller->configure(controller_node, "FollowPath", tf, map);
  controller->activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = map->getGlobalFrameID();
  path.header.stamp = controller_node->now();
  for (int i = 0; i <= 60; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = i * 0.05;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
  controller->setPlan(path);
  geometry_msgs::msg::Twist velocity;
  std::vector<double> times;
  for (int i = 0; i < 35; ++i) {
    if (i == 10) {controller->setPlan(path);}
    if (i == 15) {controller->setSpeedLimit(0.2, false);}
    if (i == 25) {controller->setSpeedLimit(10.0, true);}
    if (i == 30) {controller->setSpeedLimit(100.0, true);}
    const auto start = std::chrono::steady_clock::now();
    const auto command = controller->computeVelocityCommands(path.poses.front(), velocity, &checker);
    times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
    EXPECT_TRUE(std::isfinite(command.twist.linear.x));
    EXPECT_GE(command.twist.linear.x, 0.0);
    const double maximum = i >= 30 || i < 15 ? 1.2 : (i >= 25 ? 0.12 : 0.2);
    EXPECT_LE(command.twist.linear.x, maximum + 1e-7);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
    EXPECT_LE(std::abs(command.twist.angular.z), std::abs(command.twist.linear.x) / 0.81 + 1e-7);
    velocity = command.twist;
  }
  EXPECT_GT(velocity.linear.x, 0);
  std::sort(times.begin(), times.end());
  std::cout << "DYNAMICS_FULL_MPPI median_ms=" << times[times.size() / 2]
    << " max_ms=" << times.back() << std::endl;
  // Real native critic/fallback, not a mocked exception or a production goal.
  auto * grid = map->getCostmap();
  for (unsigned int x = 0; x < grid->getSizeInCellsX(); ++x) {
    for (unsigned int y = 0; y < grid->getSizeInCellsY(); ++y) {grid->setCost(x, y, 254);}
  }
  try {
    controller->computeVelocityCommands(path.poses.front(), velocity, &checker);
    ADD_FAILURE() << "A fully lethal map must preserve native no-control failure";
  } catch (const std::runtime_error & error) {
    EXPECT_STREQ(error.what(), "Optimizer fail to compute path");
  }
  // Same client/controller, next compute call, no new plan and no retained
  // command through the failure. Clearing this synthetic map is test-only.
  grid->resetMap(0, 0, grid->getSizeInCellsX(), grid->getSizeInCellsY());
  bool nonzero_recovered = false;
  for (int n = 0; n < 8; ++n) {
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = controller->computeVelocityCommands(
      path.poses.front(), geometry_msgs::msg::Twist{}, &checker));
    EXPECT_TRUE(std::isfinite(command.twist.linear.x));
    nonzero_recovered |= command.twist.linear.x != 0 || command.twist.angular.z != 0;
  }
  EXPECT_TRUE(nonzero_recovered);
  controller->deactivate();
  controller->cleanup();
  controller.reset();
  rclcpp::shutdown();
  map->on_cleanup(rclcpp_lifecycle::State{});
}

namespace
{
class CountingCritic : public mppi::critics::CriticFunction
{
public:
  int calls{0};
  void initialize() override {}
  void score(mppi::CriticData &) override {++calls;}
};
class InspectCriticManager : public mppi::CriticManager
{
public:
  CountingCritic * install()
  {
    auto critic = std::make_unique<CountingCritic>();
    auto * result = critic.get();
    critics_.push_back(std::move(critic));
    return result;
  }
};
}

TEST(InstalledMppiLibrary, AlreadyFailedDataSkipsScoringUntilCallerClearsFlag)
{
  InspectCriticManager manager;
  auto * counter = manager.install();
  mppi::models::State state;
  mppi::models::Trajectories trajectory;
  mppi::models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1u});
  float dt = 0.05F;
  mppi::CriticData data{state, trajectory, path, costs, dt, true,
    nullptr, nullptr, std::nullopt, std::nullopt};
  manager.evalTrajectoriesScores(data);  // Calls the actual installed library.
  EXPECT_EQ(counter->calls, 0);
  data.fail_flag = false;  // Synthetic test data only, not a production change.
  manager.evalTrajectoriesScores(data);
  EXPECT_EQ(counter->calls, 1);
}
