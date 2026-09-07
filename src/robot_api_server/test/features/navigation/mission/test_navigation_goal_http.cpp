#include <optional>
#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/mission/navigation_goal_http.hpp"

namespace navigation = robot_api_server::features::navigation;

TEST(NavigationGoalHttp, ErrorPayloadPreservesOptionalCodeAndDockSnapshot)
{
  const auto response = navigation::navigation_goal_error_response(
    409,
    "blocked \"now\"",
    true,
    "charging",
    R"({"state":"docked"})",
    "DOCKED_CONTACT_BLOCK");

  EXPECT_EQ(response.status, 409);
  EXPECT_EQ(response.content_type, "application/json");
  EXPECT_NE(response.body.find("\"accepted\":false"), std::string::npos);
  EXPECT_NE(response.body.find("\"error\":\"blocked \\\"now\\\"\""), std::string::npos);
  EXPECT_NE(response.body.find("\"code\":\"DOCKED_CONTACT_BLOCK\""), std::string::npos);
  EXPECT_NE(response.body.find("\"pre_navigation_undock\":true"), std::string::npos);
  EXPECT_NE(response.body.find("\"pre_navigation_dock_check\":{\"state\":\"docked\"}"),
    std::string::npos);

  const auto without_code = navigation::navigation_goal_error_response(
    400, "bad", false, "", "{}");
  EXPECT_EQ(without_code.body.find("\"code\":"), std::string::npos);
}

TEST(NavigationGoalHttp, AcceptedPayloadKeepsAppContractAndOptionalFloorIdentity)
{
  navigation::NavigationGoalAcceptedPayload payload;
  payload.navigation_goal_id = 7U;
  payload.action = "/navigate_to_pose";
  payload.source = "poses_yaml";
  payload.goal_completion_policy = "pose_required";
  payload.native_nav2_goal_completion = true;
  payload.api_final_yaw_align_enabled = true;
  payload.nav2_rotation_shim_enabled = true;
  payload.frame_id = "map";
  payload.pose_id = "delivery-1";
  payload.building_id = "B15";
  payload.floor_id = "F1";
  payload.pre_navigation_undock = false;
  payload.pre_navigation_undock_detail = "not docked";
  payload.pre_navigation_dock_check_json = R"({"state":"clear"})";
  payload.pre_navigation_relocalization_detail = "normal path disabled";
  payload.goal.x = 1.25;
  payload.goal.y = -2.5;
  payload.goal.yaw = 0.125;

  const auto response = navigation::navigation_goal_accepted_response(payload);
  EXPECT_EQ(response.status, 202);
  EXPECT_NE(response.body.find("\"navigation_goal_id\":7"), std::string::npos);
  EXPECT_NE(response.body.find("\"building_id\":\"B15\""), std::string::npos);
  EXPECT_NE(response.body.find("\"floor_id\":\"F1\""), std::string::npos);
  EXPECT_NE(response.body.find("\"navigation_normal_path_relocalization_enabled\":false"),
    std::string::npos);
  EXPECT_NE(response.body.find("\"ordinary_navigation_triggered_relocalization\":false"),
    std::string::npos);
  EXPECT_NE(response.body.find("\"goal\":{\"x\":1.250000,\"y\":-2.500000,\"yaw\":0.125000}"),
    std::string::npos);

  payload.building_id.reset();
  payload.floor_id.reset();
  const auto direct = navigation::navigation_goal_accepted_response(payload);
  EXPECT_EQ(direct.body.find("\"building_id\":"), std::string::npos);
  EXPECT_EQ(direct.body.find("\"floor_id\":"), std::string::npos);
}
