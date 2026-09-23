// Run only in the private ROS namespace used by test_navlite_mppi_evidence.py.
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("navlite_load_map");
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  if (map->on_configure(rclcpp_lifecycle::State{}) != nav2_util::CallbackReturn::SUCCESS) {
    throw std::runtime_error("isolated costmap configure failed");
  }
  auto node = std::make_shared<nav2_util::LifecycleNode>("navlite_controller_load");
  node->declare_parameter("FollowPath.primary_controller",
    "robot_nav_config::RangerMPPIController");
  node->declare_parameter("FollowPath.critics", std::vector<std::string>{"GoalCritic"});
  node->declare_parameter("controller_frequency", 15.0);
  node->declare_parameter("FollowPath.model_dt", 1.0 / 15.0);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance("robot_nav_config::GoalScopedRotationShimController");
  controller->configure(node, "FollowPath", tf, map);
  controller->activate();
  // Leave a loader proof: the test runner verifies the candidate, not the installed library.
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("libgoal_scoped_rotation_shim_controller.so") != std::string::npos ||
      line.find("libranger_dynamics_mppi_controller.so") != std::string::npos)
    {
      std::cout << "CANDIDATE_MAP " << line << '\n';
    }
  }
  controller->deactivate();
  controller->cleanup();
  controller.reset();
  rclcpp::shutdown();
  map->on_cleanup(rclcpp_lifecycle::State{});
  std::cout << "PASS outer controller + primary plugin lifecycle; no goal or command sent\n";
}
