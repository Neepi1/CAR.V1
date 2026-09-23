#include <gtest/gtest.h>

#include <stdexcept>
#include <limits>

#include "robot_api_server/features/elevator/execution/elevator_floor_retry_policy.hpp"

namespace robot_api_server
{

TEST(ElevatorFloorRetryPolicy, RetryUsesANewTransactionRatherThanRevivingAbortedOuter)
{
  EXPECT_NE(make_floor_retry_transaction_id("elevator-task-01", 41001U), "elevator-task-01");
}

TEST(ElevatorFloorRetryPolicy, InvalidOuterOrUnreservedSequenceCannotCreateRetry)
{
  EXPECT_THROW(make_floor_retry_transaction_id("task", 0U), std::invalid_argument);
  for (const auto & invalid : {"", ".", "..", "task/child", "task:child", "task\\child"}) {
    EXPECT_THROW(floor_attempt_prefix(invalid), std::invalid_argument);
  }
  EXPECT_THROW(floor_attempt_prefix(std::string(129U, 'x')), std::invalid_argument);
}

TEST(ElevatorFloorRetryPolicy, IdentifiesLegacyAndRetryTransactionsWithoutPrefixAliasing)
{
  const std::string outer = "elevator-task-01";
  EXPECT_TRUE(floor_transaction_belongs_to(outer, outer));
  EXPECT_TRUE(floor_transaction_belongs_to(outer, make_floor_retry_transaction_id(outer, 41U)));
  EXPECT_FALSE(floor_transaction_belongs_to(outer, outer + "-floor-retry-41"));
  EXPECT_FALSE(floor_transaction_belongs_to(outer, make_floor_retry_transaction_id(outer + "0", 41U)));
}

TEST(ElevatorFloorRetryPolicy, OnlyRecognizesFloorManagerResourcesOfThisOuterTransaction)
{
  const std::string outer = "elevator-task-01";
  const auto attempt = make_floor_retry_transaction_id(outer, 41001U);
  EXPECT_TRUE(floor_resource_belongs_to(outer, "robot_floor_manager:" + outer));
  EXPECT_TRUE(floor_resource_belongs_to(outer, "robot_floor_manager:" + attempt));
  EXPECT_FALSE(floor_resource_belongs_to(outer, "robot_api_server:" + attempt));
  EXPECT_FALSE(floor_resource_belongs_to(outer, "foreign_robot_floor_manager:" + attempt));
  EXPECT_FALSE(floor_resource_belongs_to(outer, "robot_floor_manager:" + outer + "-other"));
}

TEST(ElevatorFloorRetryPolicy, OnlyKnownRecoverableFailureCodesPermitConsideringRetry)
{
  for (const auto code : {11U, 30U, 31U, 32U, 33U, 34U, 40U}) {
    EXPECT_TRUE(floor_failure_code_retryable(code)) << code;
  }
  for (const auto code : {0U, 10U, 20U, 21U, 22U, 23U, 41U, 90U, 99U, 65535U}) {
    EXPECT_FALSE(floor_failure_code_retryable(code)) << code;
  }
}

TEST(ElevatorFloorRetryPolicy, CleanupRequiresFreshResourcesAndAnyDeferredCleanupCompletion)
{
  EXPECT_TRUE(floor_attempt_cleanup_ready(false, false, true));
  EXPECT_TRUE(floor_attempt_cleanup_ready(true, true, true));
  EXPECT_FALSE(floor_attempt_cleanup_ready(false, false, false));
  EXPECT_FALSE(floor_attempt_cleanup_ready(true, true, false));
  // FAILED_LOCKED or a different attempt is not cleanup completion.
  EXPECT_FALSE(floor_attempt_cleanup_ready(true, false, true));
}

TEST(ElevatorFloorRetryPolicy, MaximumOuterAndSequenceRemainPathSafeAndDistinct)
{
  const std::string outer(128U, 'z');
  const auto first_retry = make_floor_retry_transaction_id(outer, 1U);
  const auto last_retry = make_floor_retry_transaction_id(
    outer, std::numeric_limits<std::uint64_t>::max());
  EXPECT_NE(first_retry, last_retry);
  EXPECT_LE(last_retry.size(), 128U);
  EXPECT_EQ(last_retry.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-"),
    std::string::npos);
  EXPECT_TRUE(floor_transaction_belongs_to(outer, first_retry));
  EXPECT_TRUE(floor_transaction_belongs_to(outer, last_retry));
  EXPECT_TRUE(floor_transaction_belongs_to(outer, outer));
  EXPECT_EQ(floor_attempt_prefix(outer), floor_attempt_prefix(outer));
  EXPECT_NE(floor_attempt_prefix(outer), floor_attempt_prefix(std::string(127U, 'z')));
}

TEST(ElevatorFloorRetryPolicy, MalformedRetrySuffixCannotClaimOwnership)
{
  const std::string outer = "elevator-task-01";
  const auto prefix = floor_attempt_prefix(outer);
  for (const auto & suffix : {
      "", "0", "01", "-1", "+1", "1more", "1-2", "1:2", "1/2", " 1", "1 ",
      "18446744073709551616", "99999999999999999999999999999999999999999999"})
  {
    EXPECT_FALSE(floor_transaction_belongs_to(outer, prefix + suffix)) << suffix;
    EXPECT_FALSE(floor_resource_belongs_to(outer, "robot_floor_manager:" + prefix + suffix)) << suffix;
  }
  EXPECT_FALSE(floor_transaction_belongs_to("", ""));
  EXPECT_FALSE(floor_transaction_belongs_to("bad:outer", "bad:outer"));
  EXPECT_FALSE(floor_resource_belongs_to(outer, "robot_floor_manager:"));
  EXPECT_FALSE(floor_resource_belongs_to(outer, "robot_floor_manager::" + outer));
  EXPECT_FALSE(floor_resource_belongs_to(outer, prefix + "1"));
}

TEST(ElevatorFloorRetryPolicy, EveryOtherWireFailureCodeRemainsNonRetryable)
{
  for (std::uint32_t value = 0U; value <= std::numeric_limits<std::uint16_t>::max(); ++value) {
    const bool expected = value == 11U || (value >= 30U && value <= 34U) || value == 40U;
    EXPECT_EQ(floor_failure_code_retryable(static_cast<std::uint16_t>(value)), expected) << value;
  }
}

TEST(ElevatorFloorRetryPolicy, CleanupTruthTableNeverInfersAbsentResources)
{
  for (const bool requires_recovery : {false, true}) {
    for (const bool same_attempt_complete : {false, true}) {
      for (const bool resources_absent : {false, true}) {
        const bool expected = resources_absent && (!requires_recovery || same_attempt_complete);
        EXPECT_EQ(floor_attempt_cleanup_ready(
            requires_recovery, same_attempt_complete, resources_absent), expected);
      }
    }
  }
}

}  // namespace robot_api_server
