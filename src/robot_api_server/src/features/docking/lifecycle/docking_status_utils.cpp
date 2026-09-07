#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"

namespace robot_api_server
{
namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::string leading_status_code(const std::string & status)
{
  const auto begin = status.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = status.find_first_of(" \t\r\n", begin);
  return status.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
}

bool status_code_has_failure_marker(const std::string & code)
{
  return code == "failed" || code == "failure" ||
         code == "timeout" || code == "not_found" || code == "rejected" ||
         starts_with(code, "failed_") || starts_with(code, "failure_") ||
         starts_with(code, "timeout_") || starts_with(code, "not_found_") ||
         starts_with(code, "rejected_") ||
         code.find("_failed") != std::string::npos ||
         code.find("_timeout") != std::string::npos ||
         code.find("_not_found") != std::string::npos ||
         code.find("_rejected") != std::string::npos;
}

}  // namespace

bool docking_status_is_success(const std::string & status)
{
  const auto code = leading_status_code(status);
  return starts_with(code, "docked") || starts_with(code, "charging");
}

bool docking_status_is_failure(const std::string & status)
{
  // /docking/status is a state code followed by diagnostic key=value fields.
  // Only the leading code is authoritative. A diagnostic such as
  // contact_stop_feedback_timeout=false (or even =true while zero commands
  // continue) must never turn the running contact_stopping state into a
  // terminal docking failure.
  const auto code = leading_status_code(status);
  return status_code_has_failure_marker(code) ||
         starts_with(status, "dock alignment outside hard limit");
}

bool docking_status_is_undocking(const std::string & status)
{
  return starts_with(leading_status_code(status), "undocking");
}

bool docking_status_is_undocked(const std::string & status)
{
  return starts_with(leading_status_code(status), "undocked");
}

bool docking_status_is_undock_failed(const std::string & status)
{
  return starts_with(leading_status_code(status), "undock_failed");
}

bool docking_status_is_stopped(const std::string & status)
{
  return starts_with(leading_status_code(status), "stopped");
}

}  // namespace robot_api_server
