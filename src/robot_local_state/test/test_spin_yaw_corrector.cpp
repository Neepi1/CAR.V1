#include <cmath>

#include <gtest/gtest.h>

#include "robot_local_state/spin_yaw_corrector.hpp"

namespace
{

using robot_local_state::SpinYawCorrector;
using robot_local_state::SpinYawCorrectorConfig;

TEST(SpinYawCorrector, PreservesNormalWheelYawAcrossWrap)
{
  SpinYawCorrector corrector(SpinYawCorrectorConfig{});

  const auto first = corrector.correct_wheel(0.0, 0.0, 3.13, 0.0, 0.0);
  const auto second = corrector.correct_wheel(0.1, 0.0, -3.13, 0.0, 0.1);

  EXPECT_NEAR(first.yaw, 3.13, 1.0e-9);
  EXPECT_NEAR(second.yaw - first.yaw, 2.0 * M_PI - 6.26, 1.0e-6);
  EXPECT_NEAR(second.x, 0.1, 1.0e-9);
}

TEST(SpinYawCorrector, RemovesWheelStartupLeadAndKeepsImuStopTail)
{
  SpinYawCorrectorConfig config;
  config.imu_stop_stable_sec = 0.10;
  config.imu_max_integration_dt_sec = 0.20;
  SpinYawCorrector corrector(config);

  corrector.correct_wheel(0.0, 0.0, 0.0, 0.0, 0.0);
  corrector.observe_motion_mode(2, 0.10);
  corrector.observe_command(0.0, 0.0, 0.6, 0.11);
  corrector.observe_imu(0.0, 0.10);

  // The chassis reports 0.1rad before the body IMU observes any rotation.
  const auto startup = corrector.correct_wheel(0.04, -0.02, 0.10, 0.6, 0.20);
  EXPECT_TRUE(startup.correction_active);
  EXPECT_NEAR(startup.yaw, 0.0, 1.0e-9);
  EXPECT_NEAR(startup.x, 0.0, 1.0e-9);
  EXPECT_NEAR(startup.y, 0.0, 1.0e-9);

  corrector.observe_imu(0.0, 0.20);
  corrector.observe_imu(0.6, 0.30);
  corrector.observe_imu(0.6, 0.40);
  const auto moving = corrector.correct_wheel(0.08, -0.03, 0.20, 0.6, 0.40);
  EXPECT_NEAR(moving.yaw, 0.09, 1.0e-9);

  corrector.observe_command(0.0, 0.0, 0.0, 0.41);
  corrector.observe_imu(0.3, 0.50);
  corrector.observe_imu(0.0, 0.60);
  corrector.observe_imu(0.0, 0.71);

  // Finalization happens on a wheel sample so the persistent offset is based
  // on the latest raw wheel pose and cannot jump on the next normal sample.
  const auto settled = corrector.correct_wheel(0.10, -0.04, 0.25, 0.0, 0.72);
  EXPECT_FALSE(settled.correction_active);
  EXPECT_NEAR(settled.yaw, 0.15, 1.0e-9);
  EXPECT_NEAR(settled.x, 0.0, 1.0e-9);
  EXPECT_NEAR(settled.y, 0.0, 1.0e-9);

  corrector.observe_motion_mode(0, 0.73);
  const auto resumed = corrector.correct_wheel(0.11, -0.04, 0.26, 0.0, 0.80);
  EXPECT_NEAR(resumed.yaw, 0.16, 1.0e-9);
  EXPECT_NEAR(resumed.x, 0.01 * std::cos(-0.10), 1.0e-9);
  EXPECT_NEAR(resumed.y, 0.01 * std::sin(-0.10), 1.0e-9);
  EXPECT_EQ(corrector.status(0.80).completed_spin_count, 1u);
}

TEST(SpinYawCorrector, RotatesPostSpinTranslationIntoCorrectedOdomFrame)
{
  SpinYawCorrectorConfig config;
  config.imu_stop_stable_sec = 0.10;
  config.imu_max_integration_dt_sec = 0.20;
  SpinYawCorrector corrector(config);

  corrector.correct_wheel(0.0, 0.0, 0.0, 0.0, 0.0);
  corrector.observe_motion_mode(2, 0.10);
  corrector.observe_command(0.0, 0.0, 0.6, 0.11);
  corrector.observe_imu(0.0, 0.10);
  corrector.correct_wheel(0.0, 0.0, 0.20, 0.6, 0.20);

  // The body turns 0.10rad according to the IMU while wheel odom reports
  // 0.20rad. Freezing spin x/y makes the persistent yaw correction -0.10rad.
  corrector.observe_imu(0.0, 0.20);
  corrector.observe_imu(0.5, 0.30);
  corrector.observe_imu(0.5, 0.40);
  corrector.observe_command(0.0, 0.0, 0.0, 0.41);
  corrector.observe_imu(0.0, 0.50);
  corrector.observe_imu(0.0, 0.61);
  const auto settled = corrector.correct_wheel(0.0, 0.0, 0.20, 0.0, 0.62);

  ASSERT_FALSE(settled.correction_active);
  ASSERT_NEAR(settled.yaw, 0.10, 1.0e-9);

  // A subsequent 1m raw-wheel displacement must be transformed by the same
  // rigid rotation as yaw. Adding a constant x/y offset is not an SE(2)
  // transform and would incorrectly return (1, 0) here.
  corrector.observe_motion_mode(0, 0.63);
  const auto resumed = corrector.correct_wheel(1.0, 0.0, 0.20, 0.0, 0.70);
  EXPECT_NEAR(resumed.x, std::cos(-0.10), 1.0e-9);
  EXPECT_NEAR(resumed.y, std::sin(-0.10), 1.0e-9);
  EXPECT_NEAR(resumed.yaw, 0.10, 1.0e-9);
}

TEST(SpinYawCorrector, FallsBackContinuouslyWhenImuIsMissing)
{
  SpinYawCorrectorConfig config;
  config.imu_timeout_sec = 0.20;
  SpinYawCorrector corrector(config);

  corrector.correct_wheel(0.0, 0.0, 0.0, 0.0, 0.0);
  corrector.observe_motion_mode(2, 0.10);
  corrector.observe_command(0.0, 0.0, 0.4, 0.11);
  const auto held = corrector.correct_wheel(0.0, 0.0, 0.10, 0.4, 0.20);
  EXPECT_TRUE(held.correction_active);
  EXPECT_NEAR(held.yaw, 0.0, 1.0e-9);

  const auto fallback = corrector.correct_wheel(0.0, 0.0, 0.20, 0.4, 0.45);
  EXPECT_FALSE(fallback.correction_active);
  EXPECT_NEAR(fallback.yaw, held.yaw, 1.0e-9);

  const auto resumed = corrector.correct_wheel(0.0, 0.0, 0.25, 0.4, 0.55);
  EXPECT_NEAR(resumed.yaw, 0.05, 1.0e-9);
  EXPECT_EQ(corrector.status(0.55).imu_fallback_count, 1u);
}

}  // namespace
