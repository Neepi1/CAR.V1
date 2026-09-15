#include <gtest/gtest.h>

#ifndef NJRH_DYNAMICS_BASELINE
#include "robot_nav_config/chassis_dynamics/response_model.hpp"
#endif

TEST(RangerResponse, ZeroRequestPredictsMeasuredBrakingTravelNotInstantStop)
{
  constexpr double dt = 0.02;
  double travel = 0.0;
#ifndef NJRH_DYNAMICS_BASELINE
  robot_nav_config::chassis_dynamics::ResponseModel model;
  model.reset(1.2, 0.0, 1.2, 0.0);
#endif
  for (int i = 0; i < 150; ++i) {
#ifdef NJRH_DYNAMICS_BASELINE
    // Installed Humble MotionModel::predict copies the requested velocity to
    // the next state. This is the explicit baseline, not the candidate model.
    const double velocity = 0.0;
#else
    const double velocity = model.advance(0.0, 0.0, dt).linear;
#endif
    travel += velocity * dt;
  }
  EXPECT_GT(travel, 0.53);  // June 30 held-out CAN stopping integral ~0.583 m.
  EXPECT_LT(travel, 0.64);
}

#ifndef NJRH_DYNAMICS_BASELINE
TEST(RangerResponse, AccelerationAndSteeringHaveMeasuredDelayAndRise)
{
  using robot_nav_config::chassis_dynamics::ResponseModel;
  ResponseModel linear;
  linear.reset(0, 0, 0, 0);
  double t10 = -1, t90 = -1;
  for (int i = 0; i < 160; ++i) {
    const auto state = linear.advance(1.2, 0, 0.02);
    if (state.linear >= 0.12 && t10 < 0) {t10 = (i + 1) * 0.02;}
    if (state.linear >= 1.08 && t90 < 0) {t90 = (i + 1) * 0.02;}
  }
  EXPECT_NEAR(t90 - t10, 1.55, 0.05);
  ResponseModel steering;
  steering.reset(0.2, 0, 0.2, 0);
  for (int i = 0; i < 5; ++i) {EXPECT_DOUBLE_EQ(steering.advance(0.2, 0.15, 0.02).steering, 0);}
  const auto early = steering.advance(0.2, 0.15, 0.02);
  EXPECT_LT(early.steering, steering.steering_for(0.2, 0.15));
  for (int i = 0; i < 50; ++i) {steering.advance(0.2, 0.15, 0.02);}
  EXPECT_NEAR(steering.advance(0.2, 0.15, 0.02).angular, 0.15, 0.001);
}

TEST(RangerResponse, SignedSteeringMatchesExistingDualAckermannGeometry)
{
  robot_nav_config::chassis_dynamics::ResponseModel model;
  EXPECT_NEAR(model.steering_for(0.2, 0.15), 0.21552193, 1e-7);
  EXPECT_NEAR(model.steering_for(0.2, -0.15), -0.21552193, 1e-7);
  for (double v : {0.2, 0.6, 1.2}) {
    for (double w : {-0.15, 0.0, 0.15}) {
      EXPECT_NEAR(model.yaw_rate(v, model.steering_for(v, w)), w, 1e-9);
    }
  }
}

TEST(RangerResponse, MeasuredIssuedHistoryIsUsedOnlyBeforeFutureCommandsArrive)
{
  robot_nav_config::chassis_dynamics::ResponseModel model;
  model.reset(0.6, 0, 0.6, 0);
  model.remember({-0.2, 0, 0}); // Actual safety stop already reached the driver.
  EXPECT_LT(model.advance(0.6, 0, 0.02).linear, 0.6);
  for (int i = 0; i < 100; ++i) {model.advance(0.6, 0, 0.02);}
  EXPECT_NEAR(model.advance(0.6, 0, 0.02).linear, 0.6, 1e-4);
}
#endif
