#pragma once

#include <string>
#include <utility>
#include <vector>

namespace robot_api_server
{

bool safe_client_id(const std::string & client_id);
int subscription_ttl_ms_from_body(
  const std::string & body,
  int default_ttl_ms,
  int max_ttl_ms);
std::vector<std::string> subscription_resources_from_body(const std::string & body);
std::pair<std::string, std::string> subscription_client_id_from_body(const std::string & body);
std::string resource_list_json(const std::vector<std::string> & resources);

}  // namespace robot_api_server
