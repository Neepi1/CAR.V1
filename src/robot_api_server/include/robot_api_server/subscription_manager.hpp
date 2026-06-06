#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robot_api_server
{

class SubscriptionManager
{
public:
  using Clock = std::chrono::steady_clock;
  using TransitionCallback = std::function<void(const std::string &, bool)>;

  SubscriptionManager(std::vector<std::string> supported_resources, TransitionCallback transition_callback);

  bool supported(const std::string & resource) const;
  std::optional<std::string> validate_resources(const std::vector<std::string> & resources) const;

  void acquire(
    const std::string & client_id,
    const std::vector<std::string> & resources,
    std::chrono::milliseconds ttl);

  void release(const std::string & client_id, const std::vector<std::string> & requested_resources);
  std::vector<std::string> resources_for_client(const std::string & client_id) const;
  void expire();
  bool active(const std::string & resource) const;
  std::size_t ref_count(const std::string & resource) const;
  std::string snapshot_json() const;

private:
  std::vector<std::string> resources_for_client_locked(const std::string & client_id) const;
  void apply_transitions(const std::vector<std::pair<std::string, bool>> & transitions);

  std::vector<std::string> supported_resources_;
  TransitionCallback transition_callback_;
  mutable std::mutex mutex_;
  std::map<std::string, std::map<std::string, Clock::time_point>> leases_;
};

}  // namespace robot_api_server
