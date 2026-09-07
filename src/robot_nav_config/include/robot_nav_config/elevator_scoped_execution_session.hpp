#pragma once

#include <string>

namespace robot_nav_config {

struct ElevatorScopedExecutionSessionTransition {
  bool accepted{false};
  bool reset_required{false};
  const char *detail{"invalid_session_id"};
};

// Tracks the explicit action-attempt boundary supplied by the elevator
// runtime. It deliberately does not infer identity from goal coordinates,
// path stamps, or elapsed time: a retry to the same pose is still a new
// execution session, while periodic plans inside one session are not.
class ElevatorScopedExecutionSession {
public:
  ElevatorScopedExecutionSessionTransition begin(
      const std::string &session_id) {
    if (session_id.empty()) {
      return {};
    }
    if (session_id == active_session_id_) {
      return {true, false, "session_already_active"};
    }
    active_session_id_ = session_id;
    last_ended_session_id_.clear();
    return {true, true, "new_session_started"};
  }

  ElevatorScopedExecutionSessionTransition end(
      const std::string &session_id) {
    if (session_id.empty()) {
      return {};
    }
    if (session_id == active_session_id_) {
      active_session_id_.clear();
      last_ended_session_id_ = session_id;
      return {true, true, "active_session_ended"};
    }
    if (active_session_id_.empty() &&
        session_id == last_ended_session_id_) {
      return {true, false, "session_already_ended"};
    }
    return {false, false, "stale_or_unknown_session"};
  }

  bool has_active_session() const noexcept {
    return !active_session_id_.empty();
  }

  bool matches(const std::string &session_id) const noexcept {
    return !session_id.empty() && session_id == active_session_id_;
  }

  const std::string &active_session_id() const noexcept {
    return active_session_id_;
  }

  void reset() noexcept {
    active_session_id_.clear();
    last_ended_session_id_.clear();
  }

private:
  std::string active_session_id_;
  std::string last_ended_session_id_;
};

}  // namespace robot_nav_config
