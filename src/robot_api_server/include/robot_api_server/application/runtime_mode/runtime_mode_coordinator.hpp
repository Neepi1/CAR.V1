#pragma once

#include <mutex>
#include <optional>
#include <string>

namespace robot_api_server::application::runtime_mode
{

struct RuntimeModeSnapshot
{
  std::string mode{"IDLE"};
  std::string state{"idle"};
  std::string mapping_state{"stopped"};
  std::string navigation_state{"stopped"};
  std::string docking_state{"stopped"};
  std::string docking_status;
  std::string docking_dock_id;
  std::string message;
  bool mapping_active{false};
  bool navigation_active{false};
  bool docking_active{false};
  bool healthy{true};
};

std::string active_runtime_profile(const RuntimeModeSnapshot & snapshot);

class RuntimeModeCoordinator
{
public:
  void set_mapping(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true);

  void set_navigation(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true);

  void set_docking(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true);

  void accept_docking(
    const std::string & dock_id,
    const std::string & message = "docking accepted");

  void set_docking_identity(
    const std::string & dock_id,
    std::optional<std::string> message = std::nullopt);

  void set_docking_status(const std::string & status);
  void finish_docking(const std::string & final_state, const std::string & detail);

  RuntimeModeSnapshot snapshot() const;

  bool try_begin_transition(const std::string & owner, std::string & conflict_owner);
  void finish_transition(const std::string & owner);
  std::string transition_owner() const;

private:
  mutable std::mutex state_mutex_;
  bool mapping_active_{false};
  bool navigation_active_{false};
  bool docking_active_{false};
  bool healthy_{true};
  std::string mapping_state_{"stopped"};
  std::string navigation_state_{"stopped"};
  std::string docking_state_{"stopped"};
  std::string docking_status_;
  std::string docking_dock_id_;
  std::string message_;

  mutable std::mutex transition_mutex_;
  std::string transition_owner_;
};

}  // namespace robot_api_server::application::runtime_mode
