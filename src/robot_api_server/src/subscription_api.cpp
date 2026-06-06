#include "robot_api_server/subscription_api.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <sstream>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

bool safe_client_id(const std::string & client_id)
{
  if (client_id.empty() || client_id.size() > 128U) {
    return false;
  }
  return std::all_of(client_id.begin(), client_id.end(), [](const unsigned char c) {
    return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':';
  });
}

int subscription_ttl_ms_from_body(
  const std::string & body,
  const int default_ttl_ms,
  const int max_ttl_ms)
{
  const auto ttl = json_number_value(body, "ttl_ms");
  if (!ttl || !std::isfinite(*ttl)) {
    return default_ttl_ms;
  }
  return std::clamp(static_cast<int>(*ttl), 1000, max_ttl_ms);
}

std::vector<std::string> subscription_resources_from_body(const std::string & body)
{
  auto resources = json_string_array_value(body, "resources");
  if (resources.empty()) {
    const auto resource = json_string_value(body, "resource");
    if (resource) {
      resources.push_back(*resource);
    }
  }
  std::sort(resources.begin(), resources.end());
  resources.erase(std::unique(resources.begin(), resources.end()), resources.end());
  return resources;
}

std::pair<std::string, std::string> subscription_client_id_from_body(const std::string & body)
{
  const std::array<std::string, 6> keys = {
    "client_id", "clientId", "lease_id", "leaseId", "subscription_id", "subscriptionId"};
  for (const auto & key : keys) {
    const auto value = json_string_value(body, key);
    if (value && safe_client_id(*value)) {
      return {*value, key};
    }
  }
  return {"http:compat-default", "fallback"};
}

std::string resource_list_json(const std::vector<std::string> & resources)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < resources.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    out << json_string(resources[i]);
  }
  out << "]";
  return out.str();
}

}  // namespace robot_api_server
