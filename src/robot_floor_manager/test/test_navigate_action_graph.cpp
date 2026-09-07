#include <set>
#include <string>

#include <gtest/gtest.h>

#include "robot_floor_manager/navigate_action_graph.hpp"

TEST(NavigateActionGraph, IsReadyWithCompleteActionGraphAndStatusPublisher)
{
  const std::set<std::string> services{
    "/navigate_to_pose/_action/send_goal",
    "/navigate_to_pose/_action/get_result",
    "/navigate_to_pose/_action/cancel_goal",
  };

  EXPECT_TRUE(robot_floor_manager::navigate_action_graph_ready(
      "/navigate_to_pose", services, 1U));
}

TEST(NavigateActionGraph, RejectsIncompleteActionGraphOrMissingStatusPublisher)
{
  const std::set<std::string> incomplete_services{
    "/navigate_to_pose/_action/send_goal",
    "/navigate_to_pose/_action/get_result",
  };
  const std::set<std::string> complete_services{
    "/navigate_to_pose/_action/send_goal",
    "/navigate_to_pose/_action/get_result",
    "/navigate_to_pose/_action/cancel_goal",
  };

  EXPECT_FALSE(robot_floor_manager::navigate_action_graph_ready(
      "/navigate_to_pose", incomplete_services, 1U));
  EXPECT_FALSE(robot_floor_manager::navigate_action_graph_ready(
      "/navigate_to_pose", complete_services, 0U));
}
