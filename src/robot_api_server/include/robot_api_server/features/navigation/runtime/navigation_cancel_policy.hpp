#pragma once

#include <cstddef>

namespace robot_api_server
{

inline bool cancel_all_response_proves_idle(
  const bool response_accepted,
  const std::size_t goals_canceling_count) noexcept
{
  return response_accepted && goals_canceling_count == 0U;
}

inline bool navigation_stack_stop_allowed(
  const bool cancellation_terminal_proven) noexcept
{
  return cancellation_terminal_proven;
}

inline bool navigation_cancel_job_succeeded(
  const bool cancellation_terminal_proven,
  const bool stop_stack_requested,
  const bool stop_stack_ok) noexcept
{
  return cancellation_terminal_proven &&
         (!stop_stack_requested || stop_stack_ok);
}

}  // namespace robot_api_server
