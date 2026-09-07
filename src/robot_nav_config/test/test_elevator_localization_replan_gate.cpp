#include <gtest/gtest.h>

#include "robot_nav_config/elevator_localization_replan_gate.hpp"

namespace {

using robot_nav_config::ElevatorLocalizationReplanAction;
using robot_nav_config::ElevatorLocalizationReplanGate;
using robot_nav_config::ElevatorLocalizationReplanObservation;

TEST(ElevatorLocalizationReplanGate, IgnoresNormalSmallCorrectionNoise) {
  ElevatorLocalizationReplanGate gate;
  EXPECT_EQ(gate.observe({1.0, 2.0, 0.1}, 0.0),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_EQ(gate.observe({1.04, 2.02, 0.14}, 1.0),
            ElevatorLocalizationReplanAction::kTrack);
}

TEST(ElevatorLocalizationReplanGate,
     TracksContinuouslyAcrossMaterialMapCorrection) {
  ElevatorLocalizationReplanGate gate;
  ASSERT_EQ(gate.observe({0.0, 0.0, 0.0}, 0.0),
            ElevatorLocalizationReplanAction::kTrack);

  EXPECT_EQ(gate.observe({0.20, 0.0, 0.02}, 0.10),
            ElevatorLocalizationReplanAction::kTrack)
      << "a map-frame correction must update the remaining pose error without "
         "pausing the active elevator motion";
  EXPECT_EQ(gate.observe({0.31, 0.0, 0.04}, 0.40),
            ElevatorLocalizationReplanAction::kTrack)
      << "a correction burst must not create a controller-owned zero interval";
  EXPECT_TRUE(gate.consume_correction_event());
  EXPECT_FALSE(gate.consume_correction_event());
}

TEST(ElevatorLocalizationReplanGate,
     CoalescesACorrectionBurstIntoOneDiagnostic) {
  ElevatorLocalizationReplanGate gate;
  ASSERT_EQ(gate.observe({0.0, 0.0, 0.0}, 0.0),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_EQ(gate.observe({0.20, 0.0, 0.02}, 0.10),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_TRUE(gate.consume_correction_event());
  EXPECT_EQ(gate.observe({0.31, 0.0, 0.04}, 0.40),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_FALSE(gate.consume_correction_event());
  EXPECT_EQ(gate.observe({0.31, 0.0, 0.04}, 1.01),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_EQ(gate.observe({0.50, 0.0, 0.04}, 1.20),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_TRUE(gate.consume_correction_event());
}

TEST(ElevatorLocalizationReplanGate, InvalidObservationDoesNotAuthorizeMotion) {
  ElevatorLocalizationReplanGate gate;
  EXPECT_EQ(gate.observe({NAN, 0.0, 0.0}, 0.0),
            ElevatorLocalizationReplanAction::kHold);
}

TEST(ElevatorLocalizationReplanGate,
     DisabledDiagnosticStillTracksContinuously) {
  robot_nav_config::ElevatorLocalizationReplanParameters parameters;
  parameters.enabled = false;
  ElevatorLocalizationReplanGate gate(parameters);
  EXPECT_EQ(gate.observe({0.0, 0.0, 0.0}, 0.0),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_EQ(gate.observe({1.0, 0.0, 1.0}, 0.1),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_FALSE(gate.consume_correction_event());
}

TEST(ElevatorLocalizationReplanGate, HandlesYawWrapWithoutFalseEvent) {
  ElevatorLocalizationReplanGate gate;
  ASSERT_EQ(gate.observe({0.0, 0.0, 3.13}, 0.0),
            ElevatorLocalizationReplanAction::kTrack);
  EXPECT_EQ(gate.observe({0.0, 0.0, -3.13}, 1.0),
            ElevatorLocalizationReplanAction::kTrack);
}

} // namespace
