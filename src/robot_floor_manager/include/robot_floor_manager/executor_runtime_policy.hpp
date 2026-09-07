#ifndef ROBOT_FLOOR_MANAGER__EXECUTOR_RUNTIME_POLICY_HPP_
#define ROBOT_FLOOR_MANAGER__EXECUTOR_RUNTIME_POLICY_HPP_

#include <exception>

namespace robot_floor_manager
{

enum class ExecutorRuntimeErrorDisposition
{
  kFatal,
  kRetry,
};

ExecutorRuntimeErrorDisposition classify_executor_runtime_error(
  const std::exception & exception);

}  // namespace robot_floor_manager

#endif  // ROBOT_FLOOR_MANAGER__EXECUTOR_RUNTIME_POLICY_HPP_
