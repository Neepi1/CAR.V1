#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "robot_interfaces/msg/localization_trigger_status.hpp"

namespace robot_api_server::features::localization
{

// Both the service callback and late status can settle the same evidence once.
// Destruction and UNKNOWN are deliberately not evidence of completion.
class TriggerEvidence
{
public:
  using Status = robot_interfaces::msg::LocalizationTriggerStatus;
  TriggerEvidence(std::string id, std::function<void()> started, std::function<void()> resolved)
  : resolved_(std::move(resolved))
  {
    status_.request_id = std::move(id);
    status_.state = Status::UNKNOWN;
    status_.code = "LOCALIZATION_OUTCOME_UNKNOWN";
    started();
  }

  void observe(const Status & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status.request_id != status_.request_id || status_.outcome_known) {
      return;
    }
    const bool valid_terminal = status.outcome_known &&
      (status.state == Status::FAILED ||
      (status.state == Status::SUCCEEDED && status.accepted_explicit_sequence >
      status.baseline_explicit_sequence));
    if (status.outcome_known && !valid_terminal) {
      return;
    }
    status_ = status;
    if (valid_terminal) {
      resolved_();
    }
  }

  Status snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

private:
  mutable std::mutex mutex_;
  Status status_;
  std::function<void()> resolved_;
};

class TriggerEvidenceRegistry
{
public:
  void insert(const std::shared_ptr<TriggerEvidence> & evidence)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.emplace(evidence->snapshot().request_id, evidence);
  }

  void observe(const TriggerEvidence::Status & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = pending_.find(status.request_id);
    if (found == pending_.end()) {
      return;
    }
    found->second->observe(status);
    if (found->second->snapshot().outcome_known) {
      pending_.erase(found);
    }
  }

private:
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<TriggerEvidence>> pending_;
};

}  // namespace robot_api_server::features::localization
