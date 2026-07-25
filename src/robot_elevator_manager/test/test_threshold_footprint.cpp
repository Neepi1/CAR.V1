#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "robot_elevator_manager/threshold_footprint.hpp"

namespace robot_elevator_manager
{
namespace
{

DoorThreshold threshold()
{
  return DoorThreshold{{0.0, -0.6}, {0.0, 0.6}, {1.0, 0.0}, 0.05};
}

TEST(ThresholdFootprint, RequiresEveryVertexInsideTheCabinClearance)
{
  const std::vector<Point2> footprint{
    {0.20, -0.30},
    {0.60, -0.30},
    {0.60, 0.30},
    {0.20, 0.30},
  };

  const auto result = classify_footprint(threshold(), footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kInside);
  EXPECT_GE(result.minimum_cabin_distance_m, 0.05);
}

TEST(ThresholdFootprint, RequiresEveryVertexOutsideTheHallClearance)
{
  const std::vector<Point2> footprint{
    {-0.60, -0.30},
    {-0.20, -0.30},
    {-0.20, 0.30},
    {-0.60, 0.30},
  };

  const auto result = classify_footprint(threshold(), footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kOutside);
  EXPECT_LE(result.maximum_cabin_distance_m, -0.05);
}

TEST(ThresholdFootprint, TreatsAFootprintCrossingTheThresholdAsStraddling)
{
  const std::vector<Point2> footprint{
    {-0.20, -0.30},
    {0.20, -0.30},
    {0.20, 0.30},
    {-0.20, 0.30},
  };

  const auto result = classify_footprint(threshold(), footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kStraddling);
  EXPECT_LT(result.minimum_cabin_distance_m, 0.0);
  EXPECT_GT(result.maximum_cabin_distance_m, 0.0);
}

TEST(ThresholdFootprint, TreatsClearanceBandContactAsStraddling)
{
  const std::vector<Point2> footprint{
    {0.01, -0.30},
    {0.04, -0.30},
    {0.04, 0.30},
    {0.01, 0.30},
  };

  const auto result = classify_footprint(threshold(), footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kStraddling);
}

TEST(ThresholdFootprint, RequiresFullFootprintInsideDoorJambClearance)
{
  const std::vector<Point2> footprint{
    {0.20, 0.45},
    {0.60, 0.45},
    {0.60, 0.65},
    {0.20, 0.65},
  };

  const auto result = classify_footprint(threshold(), footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.within_jamb_clearance);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kStraddling);
}

TEST(ThresholdFootprint, UsesCabinReferenceInsteadOfThresholdEndpointOrder)
{
  auto reversed = threshold();
  std::swap(reversed.left, reversed.right);
  const std::vector<Point2> footprint{
    {0.20, -0.30},
    {0.60, -0.30},
    {0.60, 0.30},
    {0.20, 0.30},
  };

  const auto result = classify_footprint(reversed, footprint);

  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.occupancy, ThresholdOccupancy::kInside);
}

TEST(ThresholdFootprint, FailsClosedForInvalidGeometryOrFootprint)
{
  auto degenerate = threshold();
  degenerate.right = degenerate.left;
  const std::vector<Point2> triangle{{0.2, 0.0}, {0.3, 0.1}, {0.3, -0.1}};

  const auto bad_threshold = classify_footprint(degenerate, triangle);
  EXPECT_FALSE(bad_threshold.ok);
  EXPECT_EQ(bad_threshold.occupancy, ThresholdOccupancy::kStraddling);

  const auto too_few_points = classify_footprint(
    threshold(), std::vector<Point2>{{0.2, 0.0}, {0.3, 0.1}});
  EXPECT_FALSE(too_few_points.ok);
  EXPECT_EQ(too_few_points.occupancy, ThresholdOccupancy::kStraddling);

  const auto degenerate_footprint = classify_footprint(
    threshold(), std::vector<Point2>{{0.2, 0.0}, {0.3, 0.0}, {0.4, 0.0}});
  EXPECT_FALSE(degenerate_footprint.ok);
  EXPECT_EQ(degenerate_footprint.occupancy, ThresholdOccupancy::kStraddling);
}

}  // namespace
}  // namespace robot_elevator_manager
