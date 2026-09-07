#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"

#include <utility>

namespace robot_api_server::application::runtime_mode
{

std::string active_runtime_profile(const RuntimeModeSnapshot & snapshot)
{
  if (snapshot.docking_active &&
    (snapshot.docking_state == "FINE_ALIGN" ||
    snapshot.docking_state == "FINE_DOCKING_ENTRY_CHECK" ||
    snapshot.docking_state == "docking_fine"))
  {
    return "docking_fine";
  }
  if (
    snapshot.navigation_state.find("relocal") != std::string::npos ||
    snapshot.navigation_state.find("localization") != std::string::npos ||
    snapshot.docking_state.find("RELOCALIZE") != std::string::npos)
  {
    return "recovery";
  }
  return "normal";
}

void RuntimeModeCoordinator::set_mapping(
  const bool active,
  const std::string & state,
  const std::string & message,
  const bool healthy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  mapping_active_ = active;
  mapping_state_ = state;
  healthy_ = healthy;
  message_ = message;
  if (active) {
    navigation_active_ = false;
    navigation_state_ = "stopped";
    docking_active_ = false;
    docking_state_ = "stopped";
  }
}

void RuntimeModeCoordinator::set_navigation(
  const bool active,
  const std::string & state,
  const std::string & message,
  const bool healthy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  navigation_active_ = active;
  navigation_state_ = state;
  healthy_ = healthy;
  message_ = message;
  if (active) {
    mapping_active_ = false;
    mapping_state_ = "stopped";
  }
}

void RuntimeModeCoordinator::set_docking(
  const bool active,
  const std::string & state,
  const std::string & message,
  const bool healthy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  docking_active_ = active;
  docking_state_ = state;
  healthy_ = healthy;
  message_ = message;
  if (active) {
    mapping_active_ = false;
    mapping_state_ = "stopped";
  }
}

void RuntimeModeCoordinator::accept_docking(
  const std::string & dock_id,
  const std::string & message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  docking_active_ = true;
  docking_state_ = "accepted";
  docking_dock_id_ = dock_id;
  docking_status_.clear();
  healthy_ = true;
  message_ = message;
  mapping_active_ = false;
  mapping_state_ = "stopped";
}

void RuntimeModeCoordinator::set_docking_identity(
  const std::string & dock_id,
  std::optional<std::string> message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  docking_dock_id_ = dock_id;
  if (message) {
    message_ = std::move(*message);
  }
}

void RuntimeModeCoordinator::set_docking_status(const std::string & status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  docking_status_ = status;
}

void RuntimeModeCoordinator::finish_docking(
  const std::string & final_state,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  docking_active_ = false;
  docking_state_ = final_state;
  docking_status_ = detail;
  healthy_ = true;
  message_ = detail;
  if (final_state == "docked" || final_state == "charging") {
    navigation_active_ = false;
    navigation_state_ = "stopped";
  }
}

RuntimeModeSnapshot RuntimeModeCoordinator::snapshot() const
{
  RuntimeModeSnapshot snapshot;
  std::lock_guard<std::mutex> lock(state_mutex_);
  snapshot.mapping_active = mapping_active_;
  snapshot.navigation_active = navigation_active_;
  snapshot.docking_active = docking_active_;
  snapshot.mapping_state = mapping_state_;
  snapshot.navigation_state = navigation_state_;
  snapshot.docking_state = docking_state_;
  snapshot.docking_status = docking_status_;
  snapshot.docking_dock_id = docking_dock_id_;
  snapshot.healthy = healthy_;
  snapshot.message = message_;
  if (!snapshot.healthy) {
    snapshot.mode = "ERROR";
    snapshot.state = "error";
  } else if (snapshot.docking_active) {
    snapshot.mode = "DOCKING";
    snapshot.state = snapshot.docking_state;
  } else if (snapshot.mapping_active) {
    snapshot.mode = "MAPPING_2D";
    snapshot.state = snapshot.mapping_state;
  } else if (snapshot.navigation_active) {
    snapshot.mode = "NAVIGATION";
    snapshot.state = snapshot.navigation_state;
  } else {
    snapshot.mode = "IDLE";
    snapshot.state = "idle";
  }
  return snapshot;
}

bool RuntimeModeCoordinator::try_begin_transition(
  const std::string & owner,
  std::string & conflict_owner)
{
  std::lock_guard<std::mutex> lock(transition_mutex_);
  if (!transition_owner_.empty()) {
    conflict_owner = transition_owner_;
    return false;
  }
  transition_owner_ = owner;
  conflict_owner.clear();
  return true;
}

void RuntimeModeCoordinator::finish_transition(const std::string & owner)
{
  std::lock_guard<std::mutex> lock(transition_mutex_);
  if (transition_owner_ == owner) {
    transition_owner_.clear();
  }
}

std::string RuntimeModeCoordinator::transition_owner() const
{
  std::lock_guard<std::mutex> lock(transition_mutex_);
  return transition_owner_;
}

}  // namespace robot_api_server::application::runtime_mode
