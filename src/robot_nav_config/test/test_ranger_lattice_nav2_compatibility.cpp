#include <cmath>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/utils.hpp"

#ifndef RANGER_LATTICE_TEST_PATH
#error "RANGER_LATTICE_TEST_PATH must point to the generated lattice artifact"
#endif

TEST(RangerLatticeNav2Compatibility, ParsesPinnedArtifactWithHumbleTypes)
{
  std::ifstream input(RANGER_LATTICE_TEST_PATH);
  ASSERT_TRUE(input.is_open());

  nlohmann::json document;
  input >> document;

  nav2_smac_planner::LatticeMetadata metadata;
  nav2_smac_planner::fromJsonToMetaData(document.at("lattice_metadata"), metadata);
  EXPECT_EQ(metadata.motion_model, "ackermann");
  EXPECT_NEAR(metadata.min_turning_radius, 0.81, 1.0e-6);
  EXPECT_NEAR(metadata.grid_resolution, 0.05, 1.0e-6);
  EXPECT_EQ(metadata.number_of_headings, 16u);
  EXPECT_EQ(metadata.number_of_trajectories, 104u);
  ASSERT_EQ(metadata.heading_angles.size(), 16u);

  const auto & primitives_json = document.at("primitives");
  ASSERT_EQ(primitives_json.size(), metadata.number_of_trajectories);

  std::size_t parsed_pose_count = 0;
  std::size_t spin_count = 0;
  for (std::size_t index = 0; index < primitives_json.size(); ++index) {
    nav2_smac_planner::MotionPrimitive primitive;
    nav2_smac_planner::fromJsonToMotionPrimitive(primitives_json.at(index), primitive);
    EXPECT_EQ(primitive.trajectory_id, index);
    EXPECT_GE(primitive.start_angle, 0.0f);
    EXPECT_GE(primitive.end_angle, 0.0f);
    EXPECT_LT(primitive.start_angle, metadata.number_of_headings);
    EXPECT_LT(primitive.end_angle, metadata.number_of_headings);
    EXPECT_NEAR(primitive.start_angle, std::round(primitive.start_angle), 1.0e-7f);
    EXPECT_NEAR(primitive.end_angle, std::round(primitive.end_angle), 1.0e-7f);
    EXPECT_FALSE(primitive.poses.empty());
    if (std::abs(primitive.trajectory_length) < 1.0e-7f) {
      ++spin_count;
      const auto start_bin = static_cast<unsigned int>(std::lround(primitive.start_angle));
      const auto end_bin = static_cast<unsigned int>(std::lround(primitive.end_angle));
      const auto bin_delta =
        (end_bin + metadata.number_of_headings - start_bin) %
        metadata.number_of_headings;
      EXPECT_TRUE(bin_delta == 1u || bin_delta == metadata.number_of_headings - 1u);
      for (const auto & pose : primitive.poses) {
        EXPECT_NEAR(pose._x, 0.0f, 1.0e-7f);
        EXPECT_NEAR(pose._y, 0.0f, 1.0e-7f);
      }
    } else if (std::abs(primitive.arc_length) > 1.0e-7f) {
      EXPECT_GE(std::abs(primitive.turning_radius), 0.81f);
    }
    parsed_pose_count += primitive.poses.size();
  }
  EXPECT_EQ(spin_count, 32u);
  EXPECT_GT(parsed_pose_count, primitives_json.size());
}
