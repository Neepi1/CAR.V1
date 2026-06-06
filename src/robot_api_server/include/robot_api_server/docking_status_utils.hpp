#pragma once

#include <string>

namespace robot_api_server
{

bool docking_status_is_success(const std::string & status);

bool docking_status_is_failure(const std::string & status);

bool docking_status_is_undocking(const std::string & status);

bool docking_status_is_undocked(const std::string & status);

bool docking_status_is_undock_failed(const std::string & status);

bool docking_status_is_stopped(const std::string & status);

}  // namespace robot_api_server
