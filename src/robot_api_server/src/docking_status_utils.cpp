#include "robot_api_server/docking_status_utils.hpp"

namespace robot_api_server
{
namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

bool docking_status_is_success(const std::string & status)
{
  return starts_with(status, "docked") || starts_with(status, "charging");
}

bool docking_status_is_failure(const std::string & status)
{
  return status.find("failed") != std::string::npos || status.find("timeout") != std::string::npos ||
    status.find("not_found") != std::string::npos || status.find("rejected") != std::string::npos ||
    status.find("outside hard limit") != std::string::npos || docking_status_is_undock_failed(status);
}

bool docking_status_is_undocking(const std::string & status)
{
  return starts_with(status, "undocking");
}

bool docking_status_is_undocked(const std::string & status)
{
  return starts_with(status, "undocked");
}

bool docking_status_is_undock_failed(const std::string & status)
{
  return starts_with(status, "undock_failed");
}

bool docking_status_is_stopped(const std::string & status)
{
  return status.find("stopped") != std::string::npos;
}

}  // namespace robot_api_server
