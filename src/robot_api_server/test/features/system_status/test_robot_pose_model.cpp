#include <gtest/gtest.h>

#include "robot_api_server/features/system_status/robot_pose_model.hpp"

namespace robot_api_server
{
namespace
{

TEST(RobotPoseModel, FloorContextFailureHasMachineReadableCodeAndState)
{
  const auto payload = floor_context_not_ready_robot_pose_json(
    "map",
    "base_link",
    "floor_switch_failed_locked",
    "bridge target evidence is not proven");

  EXPECT_NE(payload.find("\"code\":\"FLOOR_CONTEXT_NOT_READY\""), std::string::npos);
  EXPECT_NE(
    payload.find("\"runtime_state\":\"floor_switch_failed_locked\""),
    std::string::npos);
  EXPECT_NE(payload.find("bridge target evidence is not proven"), std::string::npos);
  EXPECT_NE(payload.find("\"error\":\"no fresh map-frame robot pose\""), std::string::npos);
}

}  // namespace
}  // namespace robot_api_server
