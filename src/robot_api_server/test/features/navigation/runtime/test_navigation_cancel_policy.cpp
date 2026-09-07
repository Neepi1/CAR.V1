#include <gtest/gtest.h>

#include "robot_api_server/features/navigation/runtime/navigation_cancel_policy.hpp"

namespace
{

using robot_api_server::cancel_all_response_proves_idle;
using robot_api_server::navigation_cancel_job_succeeded;
using robot_api_server::navigation_stack_stop_allowed;

TEST(NavigationCancelPolicy, AcceptedEmptyCancelAllResponseProvesIdle)
{
  EXPECT_TRUE(cancel_all_response_proves_idle(true, 0U));
  EXPECT_FALSE(cancel_all_response_proves_idle(true, 1U));
  EXPECT_FALSE(cancel_all_response_proves_idle(false, 0U));
}

TEST(NavigationCancelPolicy, RuntimeStopRequiresCancellationTerminalProof)
{
  EXPECT_FALSE(navigation_stack_stop_allowed(false));
  EXPECT_TRUE(navigation_stack_stop_allowed(true));

  EXPECT_FALSE(navigation_cancel_job_succeeded(false, true, true));
  EXPECT_FALSE(navigation_cancel_job_succeeded(true, true, false));
  EXPECT_TRUE(navigation_cancel_job_succeeded(true, true, true));
  EXPECT_TRUE(navigation_cancel_job_succeeded(true, false, false));
}

}  // namespace
