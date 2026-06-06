#include "robot_api_server/subscription_manager.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

SubscriptionManager::SubscriptionManager(
  std::vector<std::string> supported_resources,
  TransitionCallback transition_callback)
: supported_resources_(std::move(supported_resources)),
  transition_callback_(std::move(transition_callback))
{
}

bool SubscriptionManager::supported(const std::string & resource) const
{
  return std::find(supported_resources_.begin(), supported_resources_.end(), resource) !=
         supported_resources_.end();
}

std::optional<std::string> SubscriptionManager::validate_resources(
  const std::vector<std::string> & resources) const
{
  for (const auto & resource : resources) {
    if (!supported(resource)) {
      return "unsupported subscription resource: " + resource;
    }
  }
  return std::nullopt;
}

void SubscriptionManager::acquire(
  const std::string & client_id,
  const std::vector<std::string> & resources,
  const std::chrono::milliseconds ttl)
{
  std::vector<std::pair<std::string, bool>> transitions;
  const auto expires_at = Clock::now() + ttl;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & resource : resources) {
      auto & clients = leases_[resource];
      const bool was_empty = clients.empty();
      clients[client_id] = expires_at;
      if (was_empty) {
        transitions.emplace_back(resource, true);
      }
    }
  }
  apply_transitions(transitions);
}

void SubscriptionManager::release(
  const std::string & client_id,
  const std::vector<std::string> & requested_resources)
{
  std::vector<std::pair<std::string, bool>> transitions;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto resources = requested_resources.empty() ? resources_for_client_locked(client_id) : requested_resources;
    for (const auto & resource : resources) {
      auto it = leases_.find(resource);
      if (it == leases_.end()) {
        continue;
      }
      if (it->second.erase(client_id) > 0U && it->second.empty()) {
        transitions.emplace_back(resource, false);
      }
    }
  }
  apply_transitions(transitions);
}

std::vector<std::string> SubscriptionManager::resources_for_client(const std::string & client_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return resources_for_client_locked(client_id);
}

void SubscriptionManager::expire()
{
  std::vector<std::pair<std::string, bool>> transitions;
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [resource, clients] : leases_) {
      const bool was_empty = clients.empty();
      for (auto it = clients.begin(); it != clients.end();) {
        if (it->second <= now) {
          it = clients.erase(it);
        } else {
          ++it;
        }
      }
      if (!was_empty && clients.empty()) {
        transitions.emplace_back(resource, false);
      }
    }
  }
  apply_transitions(transitions);
}

bool SubscriptionManager::active(const std::string & resource) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = leases_.find(resource);
  return it != leases_.end() && !it->second.empty();
}

std::size_t SubscriptionManager::ref_count(const std::string & resource) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = leases_.find(resource);
  return it == leases_.end() ? 0U : it->second.size();
}

std::string SubscriptionManager::snapshot_json() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream out;
  out << "{\"resources\":{";
  for (std::size_t i = 0; i < supported_resources_.size(); ++i) {
    const auto & resource = supported_resources_[i];
    const auto it = leases_.find(resource);
    const std::size_t count = it == leases_.end() ? 0U : it->second.size();
    if (i > 0U) {
      out << ",";
    }
    out << json_string(resource) << ":{\"active\":" << (count > 0U ? "true" : "false")
        << ",\"ref_count\":" << count << "}";
  }
  out << "}}";
  return out.str();
}

std::vector<std::string> SubscriptionManager::resources_for_client_locked(
  const std::string & client_id) const
{
  std::vector<std::string> resources;
  for (const auto & [resource, clients] : leases_) {
    if (clients.find(client_id) != clients.end()) {
      resources.push_back(resource);
    }
  }
  return resources;
}

void SubscriptionManager::apply_transitions(
  const std::vector<std::pair<std::string, bool>> & transitions)
{
  for (const auto & transition : transitions) {
    transition_callback_(transition.first, transition.second);
  }
}

}  // namespace robot_api_server
