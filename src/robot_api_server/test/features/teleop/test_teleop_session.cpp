#include <gtest/gtest.h>

#include <chrono>

#include "robot_api_server/features/teleop/teleop_session.hpp"

namespace robot_api_server::features::teleop
{
namespace
{

TEST(TeleopPayload, ClampsVelocityAndBuildsExistingAck)
{
  const TeleopCommandConfig config{
    1.0,
    0.55,
    true,
    "/cmd_vel_api",
  };

  const auto result = evaluate_teleop_payload(
    R"({"type":"cmd_vel","linear_x":1.8,"angular_z":-0.8})",
    config);

  EXPECT_EQ(result.kind, TeleopMessageKind::Velocity);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.velocity.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result.velocity.angular_z, -0.55);
  EXPECT_EQ(
    result.response_json,
    R"({"ok":true,"type":"cmd_vel_ack","linear_x":1,"angular_z":-0.55,"allow_reverse":true,"cmd_topic":"/cmd_vel_api"})");
}

TEST(TeleopPayload, PreservesLegacyAliasPriorityAndDefaultMessageType)
{
  const TeleopCommandConfig config{1.0, 0.55, false, "/cmd_vel_api"};

  const auto result = evaluate_teleop_payload(
    R"({"linearX":0.7,"vx":0.2,"linear":{"x":0.4},"angularZ":0.9,"wz":0.1})",
    config);

  EXPECT_EQ(result.kind, TeleopMessageKind::Velocity);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.velocity.linear_x, 0.7);
  EXPECT_DOUBLE_EQ(result.velocity.angular_z, 0.55);
  EXPECT_EQ(
    result.response_json,
    R"({"ok":true,"type":"cmd_vel_ack","linear_x":0.7,"angular_z":0.55,"allow_reverse":false,"cmd_topic":"/cmd_vel_api"})");
}

TEST(TeleopPayload, DisallowsReverseAndAcceptsNestedTwistShape)
{
  const TeleopCommandConfig config{1.0, 0.55, false, "/cmd_vel_api"};

  const auto result = evaluate_teleop_payload(
    R"({"linear":{"x":-0.4},"angular":{"z":-0.25}})",
    config);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.velocity.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.velocity.angular_z, -0.25);
}

TEST(TeleopPayload, StopKeepsExistingAcknowledgement)
{
  const TeleopCommandConfig config{1.0, 0.55, true, "/cmd_vel_api"};

  const auto result = evaluate_teleop_payload(R"({"type":"stop"})", config);

  EXPECT_EQ(result.kind, TeleopMessageKind::Stop);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.response_json, R"({"ok":true,"type":"teleop_stopped"})");
}

TEST(TeleopPayload, RejectsUnsupportedMessageTypeWithoutVelocity)
{
  const TeleopCommandConfig config{1.0, 0.55, true, "/cmd_vel_api"};

  const auto result = evaluate_teleop_payload(R"({"type":"drive"})", config);

  EXPECT_EQ(result.kind, TeleopMessageKind::Unsupported);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.response_json,
    R"({"ok":false,"error":"unsupported teleop message type: drive"})");
}

TEST(TeleopSession, RepeatsFreshVelocityForAnActiveSession)
{
  using namespace std::chrono_literals;
  TeleopSessionState state;
  const auto stored_at = TeleopSessionState::Clock::time_point{1s};
  state.session_started();
  state.store_velocity(TeleopVelocity{0.3, -0.2}, stored_at);

  const auto decision = state.repeat_decision(stored_at + 200ms, 500ms);

  EXPECT_EQ(decision.action, TeleopRepeatAction::PublishVelocity);
  EXPECT_DOUBLE_EQ(decision.velocity.linear_x, 0.3);
  EXPECT_DOUBLE_EQ(decision.velocity.angular_z, -0.2);
}

TEST(TeleopSession, WatchdogPublishesZeroOnlyOnceForStaleVelocity)
{
  using namespace std::chrono_literals;
  TeleopSessionState state;
  const auto stored_at = TeleopSessionState::Clock::time_point{1s};
  state.session_started();
  state.store_velocity(TeleopVelocity{0.3, 0.2}, stored_at);

  const auto first = state.repeat_decision(stored_at + 501ms, 500ms);
  const auto second = state.repeat_decision(stored_at + 700ms, 500ms);

  EXPECT_EQ(first.action, TeleopRepeatAction::PublishZero);
  EXPECT_EQ(second.action, TeleopRepeatAction::None);
}

TEST(TeleopSession, OnlyLastDisconnectRequestsStopPublication)
{
  using namespace std::chrono_literals;
  TeleopSessionState state;
  state.session_started();
  state.session_started();
  state.store_velocity(
    TeleopVelocity{0.2, 0.1},
    TeleopSessionState::Clock::time_point{1s});

  EXPECT_FALSE(state.session_stopped());
  EXPECT_TRUE(state.session_active());
  EXPECT_FALSE(state.idle());

  EXPECT_TRUE(state.session_stopped());
  EXPECT_FALSE(state.session_active());
  EXPECT_TRUE(state.idle());
}

TEST(TeleopSession, ExplicitClearLeavesSessionOpenWithoutRepeatingAnotherZero)
{
  using namespace std::chrono_literals;
  TeleopSessionState state;
  const auto stored_at = TeleopSessionState::Clock::time_point{1s};
  state.session_started();
  state.store_velocity(TeleopVelocity{0.2, 0.1}, stored_at);

  state.clear_velocity();
  const auto decision = state.repeat_decision(stored_at + 100ms, 500ms);

  EXPECT_TRUE(state.session_active());
  EXPECT_FALSE(state.idle());
  EXPECT_EQ(decision.action, TeleopRepeatAction::None);
}

}  // namespace
}  // namespace robot_api_server::features::teleop
