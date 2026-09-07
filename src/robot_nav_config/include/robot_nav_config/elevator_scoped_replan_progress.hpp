#pragma once

#include <cstdint>
#include <optional>
#include <unordered_set>

namespace robot_nav_config {

// Action-local admission for evidence-driven route revisions.
//
// A committed route is never a lock.  A later blockage with different live
// evidence may replace it before any historical rejoin point is reached.  The
// attempted-signature set only suppresses searches for an identical snapshot,
// which prevents A/B/A costmap oscillation from becoming a replan loop.
class ElevatorScopedReplanProgress {
public:
  void reset() noexcept {
    active_route_revision_signature_.reset();
    attempted_signatures_.clear();
  }

  bool should_attempt_replan(const std::uint64_t signature) const noexcept {
    return attempted_signatures_.find(signature) == attempted_signatures_.end();
  }

  void observe_replan_attempt(const std::uint64_t signature) {
    attempted_signatures_.insert(signature);
  }

  void forget_replan_attempt(const std::uint64_t signature) noexcept {
    attempted_signatures_.erase(signature);
  }

  bool try_commit_route_revision(const std::uint64_t signature) noexcept {
    if (attempted_signatures_.find(signature) == attempted_signatures_.end() ||
        (active_route_revision_signature_ &&
         *active_route_revision_signature_ == signature)) {
      return false;
    }
    active_route_revision_signature_ = signature;
    return true;
  }

  std::optional<std::uint64_t>
  active_route_revision_signature() const noexcept {
    return active_route_revision_signature_;
  }

private:
  std::optional<std::uint64_t> active_route_revision_signature_;
  std::unordered_set<std::uint64_t> attempted_signatures_;
};

} // namespace robot_nav_config
