#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"

namespace robot_api_server::features::docking
{

struct DockingCorrectionPauseConfig
{
  bool enabled{true};
  std::chrono::nanoseconds service_timeout{std::chrono::seconds(8)};
};

struct DockingCorrectionPausePorts
{
  std::function<bool(
      bool,
      std::string &,
      std::chrono::nanoseconds,
      bool)>
  request_correction_pause;
  std::function<features::localization::BridgeStatusSnapshot()> bridge_status_snapshot;
  std::function<RobotPoseSnapshot()> current_robot_pose;
  std::function<void(const std::string &)> warn;
  std::function<void(const std::string &)> error;
};

// Owns docking_fine map->odom correction-pause lifecycle and its canonical
// DockingJob projection. It neither publishes TF nor commands robot motion.
class DockingCorrectionPauseModule
{
public:
  DockingCorrectionPauseModule(
    DockingCorrectionPauseConfig config,
    DockingJobStore & job_store,
    DockingCorrectionPausePorts ports);

  bool set_paused(
    std::uint64_t job_id,
    bool paused,
    const std::string & reason,
    std::string & detail);

  bool release_stale_if_needed(
    const std::string & context,
    std::string & detail);

private:
  static bool phase_owns_fine_pause(const std::string & phase);
  static bool bridge_has_docking_fine_pause(
    const features::localization::BridgeStatusSnapshot & bridge);
  void update_job_state(
    std::uint64_t job_id,
    bool paused,
    const std::string & reason,
    const std::string & detail);

  DockingCorrectionPauseConfig config_;
  DockingJobStore & job_store_;
  DockingCorrectionPausePorts ports_;
};

}  // namespace robot_api_server::features::docking
