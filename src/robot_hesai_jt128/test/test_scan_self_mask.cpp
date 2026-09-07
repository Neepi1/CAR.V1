#include <gtest/gtest.h>

#include "robot_hesai_jt128/scan_self_mask.hpp"

namespace robot_hesai_jt128
{
namespace
{

TEST(ScanSelfMask, RejectsReturnsInsidePaddedRobotFootprint)
{
  const ScanSelfMask mask{};
  EXPECT_TRUE(mask.contains(0.13, -0.10));
  EXPECT_TRUE(mask.contains(-0.39, 0.28));
}

TEST(ScanSelfMask, PreservesReturnsOutsidePaddedRobotFootprint)
{
  const ScanSelfMask mask{};
  EXPECT_FALSE(mask.contains(0.40, 0.0));
  EXPECT_FALSE(mask.contains(0.0, -0.29));
  EXPECT_FALSE(mask.contains(2.0, 0.0));
}

TEST(ScanSelfMask, DisabledMaskPreservesAllReturns)
{
  ScanSelfMask mask{};
  mask.enabled = false;
  EXPECT_FALSE(mask.contains(0.0, 0.0));
}

TEST(ScanSelfMask, DetectsInvalidBounds)
{
  ScanSelfMask mask{};
  mask.min_x = 0.5;
  mask.max_x = -0.5;
  EXPECT_FALSE(mask.valid());
}

}  // namespace
}  // namespace robot_hesai_jt128
