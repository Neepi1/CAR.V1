#include "robot_floor_manager/executor_runtime_policy.hpp"

#include <string>

namespace robot_floor_manager
{

ExecutorRuntimeErrorDisposition classify_executor_runtime_error(
  const std::exception & exception)
{
  const std::string message = exception.what();
  if (
    message.find("Taking data from action client but no ready event") !=
    std::string::npos)
  {
    return ExecutorRuntimeErrorDisposition::kRetry;
  }
  return ExecutorRuntimeErrorDisposition::kFatal;
}

}  // namespace robot_floor_manager
