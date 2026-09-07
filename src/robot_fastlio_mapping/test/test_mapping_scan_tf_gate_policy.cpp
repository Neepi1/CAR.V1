#include <chrono>

#include "gtest/gtest.h"
#include "robot_fastlio_mapping/mapping_scan_tf_gate_policy.hpp"

namespace
{

using namespace std::chrono_literals;
using robot_fastlio_mapping::MappingScanTfGatePolicy;
using robot_fastlio_mapping::ScanTfGateDecision;

TEST(MappingScanTfGatePolicy, HoldsEarlyScanUntilOriginalStampTfHasSettled)
{
  const MappingScanTfGatePolicy policy(2s, 20ms);
  const auto start = MappingScanTfGatePolicy::TimePoint{};
  MappingScanTfGatePolicy::Timing timing{start, std::nullopt};

  EXPECT_EQ(policy.evaluate(timing, start + 500ms, false), ScanTfGateDecision::wait);
  EXPECT_EQ(policy.evaluate(timing, start + 700ms, true), ScanTfGateDecision::wait);
  EXPECT_EQ(policy.evaluate(timing, start + 719ms, true), ScanTfGateDecision::wait);
  EXPECT_EQ(policy.evaluate(timing, start + 720ms, true), ScanTfGateDecision::release);
}

TEST(MappingScanTfGatePolicy, ReleasesImmediatelyWhenTfPredatesScanArrival)
{
  const MappingScanTfGatePolicy policy(2s, 20ms);
  const auto start = MappingScanTfGatePolicy::TimePoint{};
  MappingScanTfGatePolicy::Timing timing{start, std::nullopt};
  policy.mark_tf_ready_before_enqueue(timing);

  EXPECT_EQ(policy.evaluate(timing, start, true), ScanTfGateDecision::release);
}

TEST(MappingScanTfGatePolicy, DropsOnlyAfterBoundedWait)
{
  const MappingScanTfGatePolicy policy(2s, 20ms);
  const auto start = MappingScanTfGatePolicy::TimePoint{};
  MappingScanTfGatePolicy::Timing timing{start, std::nullopt};

  EXPECT_EQ(policy.evaluate(timing, start + 1999ms, false), ScanTfGateDecision::wait);
  EXPECT_EQ(
    policy.evaluate(timing, start + 2000ms, false),
    ScanTfGateDecision::drop_timeout);
}

}  // namespace
