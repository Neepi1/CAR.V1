#include "robot_api_server/features/docking/lifecycle/docking_status_module.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <optional>
#include <thread>
#include <utility>

#include "rclcpp/logging.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"

namespace robot_api_server::features::docking
{
namespace
{

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  return value;
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0U) == 0U;
}

std::string post_undock_failure_code_from_settle_failure(const std::string & failure_code)
{
  if (failure_code == "POST_RELOCALIZATION_SETTLE_TIMEOUT") {
    return "POST_UNDOCK_SETTLE_TIMEOUT";
  }
  if (failure_code == "POST_RELOCALIZATION_STABLE_SAMPLE_TIMEOUT") {
    return "POST_UNDOCK_STABLE_SAMPLE_TIMEOUT";
  }
  if (failure_code == "POST_RELOCALIZATION_CORRECTION_ACTIVE") {
    return "POST_UNDOCK_CORRECTION_ACTIVE";
  }
  if (failure_code == "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH") {
    return "POST_UNDOCK_MAP_ODOM_NOT_FRESH";
  }
  if (failure_code == "POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH") {
    return "POST_UNDOCK_ODOM_BASE_NOT_FRESH";
  }
  if (failure_code == "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE" ||
    failure_code == "POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR")
  {
    return "POST_UNDOCK_TF_CHAIN_UNSTABLE";
  }
  if (failure_code == "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED") {
    return "POST_UNDOCK_LOCAL_COSTMAP_NOT_UPDATED";
  }
  if (failure_code == "POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS") {
    return "POST_UNDOCK_LOCAL_COSTMAP_TF_DROPS";
  }
  if (failure_code == "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_GAP") {
    return "POST_UNDOCK_MAP_ODOM_PUBLISH_GAP";
  }
  if (failure_code == "POST_RELOCALIZATION_MAP_ODOM_PUBLISHER_NOT_DECOUPLED") {
    return "POST_UNDOCK_MAP_ODOM_PUBLISHER_NOT_DECOUPLED";
  }
  if (failure_code == "POST_RELOCALIZATION_MAP_ODOM_PUBLISH_SEQUENCE_LAG") {
    return "POST_UNDOCK_MAP_ODOM_PUBLISH_SEQUENCE_LAG";
  }
  if (failure_code == "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER") {
    return "POST_UNDOCK_WRONG_MAP_ODOM_OWNER";
  }
  if (failure_code == "POST_RELOCALIZATION_SEQUENCE_MISMATCH") {
    return "POST_UNDOCK_RELOCALIZATION_SEQUENCE_MISMATCH";
  }
  return failure_code;
}

}  // namespace

class DockingStatusModule::Impl
{
public:
  Impl(
    rclcpp::Logger logger,
    application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
    DockingJobStore & job_store,
    DockingStatusConfig config,
    DockingStatusPorts ports)
  : logger_(std::move(logger)),
    runtime_mode_(runtime_mode),
    docking_job_mutex_(job_store.mutex()),
    docking_job_(job_store.job_unsafe()),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
  }

  ~Impl()
  {
    shutdown();
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lock(relocalization_worker_mutex_);
    if (relocalization_worker_.joinable()) {
      relocalization_worker_.join();
    }
  }

  void handle_status(const std::string & status)
  {
    if (docking_status_is_success(status)) {
      const std::string dock_id = runtime_mode_.snapshot().docking_dock_id;
      ports_.update_dock_contact_latch(true, "docking_status", status, dock_id);
    } else if (docking_status_is_undocked(status)) {
      ports_.update_dock_contact_latch(false, "docking_status", status, "");
    }

    runtime_mode_.set_docking_status(status);

    std::uint64_t post_undock_job_id = 0U;
    std::string post_undock_dock_id;
    bool post_undock_required = false;
    std::uint64_t post_fine_docking_job_id = 0U;
    std::string post_fine_docking_dock_id;
    std::string post_fine_docking_status;
    std::string post_fine_docking_final_state;
    bool post_fine_docking_ok = false;
    bool post_fine_docking_required = false;
    std::uint64_t deferred_pause_release_job_id = 0U;
    std::string deferred_pause_release_reason;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      docking_job_.last_status = status;
      ports_.record_undock_status_observation(docking_job_, status);
      if (docking_job_.state != "running") {
        if (docking_job_.state == "failed" &&
          docking_job_.post_undock_relocalization_required &&
          docking_job_.post_undock_relocalization_requested &&
          !docking_job_.post_undock_relocalization_succeeded)
        {
          runtime_mode_.set_docking(false, "failed", docking_job_.detail);
          return;
        }
        if (docking_status_is_undocking(status)) {
          runtime_mode_.set_docking(true, "undocking", status);
        } else if (docking_status_is_undocked(status)) {
          runtime_mode_.set_docking(false, "undocked", status);
        } else if (docking_status_is_undock_failed(status)) {
          runtime_mode_.set_docking(false, "failed", status);
        } else if (docking_status_is_success(status)) {
          const auto lowered = lower_copy(status);
          runtime_mode_.set_docking(
            false,
            starts_with(lowered, "charging") ? "charging" : "docked",
            status);
        }
        return;
      }
      const auto finish_docking_job_locked_and_defer_pause_release =
        [&](const bool ok, const std::string & final_state, const std::string & finish_detail,
          const std::string & release_reason) {
          const auto finished_job_id = docking_job_.id;
          if (ports_.finish_docking_job_locked(ok, final_state, finish_detail)) {
            deferred_pause_release_job_id = finished_job_id;
            deferred_pause_release_reason = release_reason;
          }
        };
      if (docking_job_.phase == "relocalize_after_fine_docking") {
        runtime_mode_.set_docking(
          true,
          "relocalize_after_fine_docking",
          docking_job_.post_fine_docking_relocalization_detail.empty() ?
          status : docking_job_.post_fine_docking_relocalization_detail);
        return;
      }
      const auto finish_or_relocalize_after_fine_docking =
        [&](const bool ok, const std::string & final_state) {
          if (!ok) {
            const auto code = classify_fine_docking_failure_code(status);
            docking_job_.failure_code = code;
            docking_job_.last_error_code = code;
            docking_job_.last_error_detail = status;
          }
          if (config_.relocalize_after_fine_docking &&
            docking_job_.docking_service_called &&
            !docking_job_.post_fine_docking_relocalization_requested)
          {
            post_fine_docking_job_id = docking_job_.id;
            post_fine_docking_dock_id = docking_job_.dock_id;
            post_fine_docking_status = status;
            post_fine_docking_final_state = final_state;
            post_fine_docking_ok = ok;
            post_fine_docking_required = config_.relocalize_after_fine_docking_required;
            docking_job_.phase = "relocalize_after_fine_docking";
            docking_job_.detail = status;
            docking_job_.post_fine_docking_relocalization_requested = true;
            docking_job_.post_fine_docking_relocalization_required =
              post_fine_docking_required;
            docking_job_.post_fine_docking_relocalization_detail =
              "waiting for fresh localization_result after fine docking";
            runtime_mode_.set_docking(
              true,
              "relocalize_after_fine_docking",
              "fine docking finished; triggering localization before returning to navigation state");
            return;
          }
          finish_docking_job_locked_and_defer_pause_release(
            ok,
            final_state,
            status,
            "fine_docking_status_" + final_state);
        };
      if (docking_status_is_undocked(status)) {
        if (config_.undock_relocalize_after_success &&
          !docking_job_.post_undock_relocalization_requested)
        {
          post_undock_job_id = docking_job_.id;
          post_undock_dock_id = docking_job_.dock_id;
          post_undock_required = docking_job_.resume_navigation;
          docking_job_.phase = "relocalize_after_undock";
          docking_job_.detail = status;
          docking_job_.post_undock_relocalization_requested = true;
          docking_job_.post_undock_relocalization_started = true;
          docking_job_.post_undock_relocalization_required = post_undock_required;
          docking_job_.post_undock_relocalization_detail =
            "waiting for fresh localization_result after undock";
          docking_job_.pending_goal_held_for_post_undock_settle =
            docking_job_.pending_goal_held_for_post_undock_settle || post_undock_required;
          runtime_mode_.set_docking(
            true,
            "relocalize_after_undock",
            "undocked by odometry; triggering localization after undock");
        } else if (docking_job_.phase != "relocalize_after_undock") {
          finish_docking_job_locked_and_defer_pause_release(
            true,
            "undocked",
            status,
            "docking_status_undocked");
        }
      } else if (docking_status_is_undock_failed(status)) {
        finish_docking_job_locked_and_defer_pause_release(
          false,
          "failed",
          status,
          "docking_status_undock_failed");
      } else if (docking_status_is_undocking(status)) {
        docking_job_.phase = "undocking";
        runtime_mode_.set_docking(true, "undocking", status);
      } else if (docking_status_is_success(status)) {
        finish_or_relocalize_after_fine_docking(true, "docked");
      } else if (docking_status_is_failure(status)) {
        finish_or_relocalize_after_fine_docking(false, "failed");
      } else if (docking_status_is_stopped(status)) {
        finish_or_relocalize_after_fine_docking(
          true,
          docking_job_.cancel_requested ? "canceled" : "stopped");
      } else {
        runtime_mode_.set_docking(true, "fine_docking", status);
      }
    }

    if (deferred_pause_release_job_id != 0U) {
      const auto release_reason =
        "docking_job_finished_" + deferred_pause_release_reason;
      const bool queued = ports_.post_deferred_work(
        [this, deferred_pause_release_job_id, release_reason]() {
          std::string release_detail;
          const bool released = ports_.set_global_correction_paused(
            deferred_pause_release_job_id,
            false,
            release_reason,
            release_detail);
          if (!released) {
            RCLCPP_ERROR(
              logger_,
              "deferred docking correction-pause release failed: %s",
              release_detail.c_str());
          }
        });
      if (!queued) {
        RCLCPP_ERROR(
          logger_,
          "could not queue docking correction-pause release reason=%s",
          release_reason.c_str());
      }
    }

    if (post_undock_job_id != 0U) {
      start_relocalization_worker(
        [this, post_undock_job_id, post_undock_dock_id, status, post_undock_required]() {
          complete_post_undock_relocalization(
            post_undock_job_id,
            post_undock_dock_id,
            status,
            post_undock_required);
        });
    }
    if (post_fine_docking_job_id != 0U) {
      start_relocalization_worker(
        [this,
          post_fine_docking_job_id,
          post_fine_docking_dock_id,
          post_fine_docking_status,
          post_fine_docking_ok,
          post_fine_docking_final_state,
          post_fine_docking_required]() {
          complete_post_fine_docking_relocalization(
            post_fine_docking_job_id,
            post_fine_docking_dock_id,
            post_fine_docking_status,
            post_fine_docking_ok,
            post_fine_docking_final_state,
            post_fine_docking_required);
        });
    }
  }

private:
  std::string classify_fine_docking_failure_code(const std::string & status) const
  {
    const auto text = lower_copy(status);
    if (text.find("contact_verify_timeout") != std::string::npos ||
      text.find("contact_verify_failed") != std::string::npos)
    {
      return "FINAL_INSERTION_NO_CONTACT";
    }
    if (text.find("dock_feature_not_found") != std::string::npos ||
      text.find("lost_dock_feature") != std::string::npos)
    {
      return config_.observation_backend == "target_observation" ?
        "DOCK_TARGET_OBSERVATION_TIMEOUT" : "GS2_DOCK_DETECT_TIMEOUT";
    }
    if (text.find("yaw") != std::string::npos ||
      text.find("outside hard limit") != std::string::npos)
    {
      return "FINE_DOCKING_REJECTED_YAW_TOO_LARGE";
    }
    if (text.find("lateral") != std::string::npos ||
      text.find(" y=") != std::string::npos)
    {
      return "FINE_DOCKING_REJECTED_LATERAL_TOO_LARGE";
    }
    if (text.find("timeout") != std::string::npos) {
      return "FINE_DOCKING_TIMEOUT";
    }
    return "FINE_DOCKING_ENTRY_CONDITION_FAILED";
  }

  void start_relocalization_worker(std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(relocalization_worker_mutex_);
    if (relocalization_worker_.joinable()) {
      relocalization_worker_.join();
    }
    relocalization_worker_ = std::thread(std::move(work));
  }

  void update_post_undock_localization_visibility(const std::uint64_t job_id)
  {
    const auto visibility = ports_.localization_visibility();
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id != job_id || docking_job_.state != "running") {
      return;
    }
    docking_job_.amcl_ready = visibility.amcl_ready;
    docking_job_.localization_degraded = visibility.localization_degraded;
    docking_job_.using_triggered_baseline_only = visibility.using_triggered_baseline_only;
  }

  void record_post_undock_navigation_readiness_failure_locked(
    const std::string & failure_code,
    const std::string & detail)
  {
    docking_job_.post_undock_navigation_readiness_failed = true;
    docking_job_.post_undock_navigation_readiness_failure_code = failure_code;
    docking_job_.post_undock_navigation_readiness_detail = detail;
    docking_job_.pending_goal_held_for_post_undock_settle = true;
    docking_job_.pending_goal_released_after_post_undock_settle = false;
  }

  void clear_post_undock_navigation_readiness_failure_locked()
  {
    docking_job_.post_undock_navigation_readiness_failed = false;
    docking_job_.post_undock_navigation_readiness_failure_code.clear();
    docking_job_.post_undock_navigation_readiness_detail.clear();
  }

  void complete_post_fine_docking_relocalization(
    std::uint64_t job_id,
    const std::string & dock_id,
    const std::string & fine_docking_status,
    bool original_ok,
    const std::string & original_final_state,
    bool required_before_final_state);
  void complete_post_undock_relocalization(
    std::uint64_t job_id,
    const std::string & dock_id,
    const std::string & undock_status,
    bool required_before_navigation);

  rclcpp::Logger logger_;
  application::runtime_mode::RuntimeModeCoordinator & runtime_mode_;
  std::mutex & docking_job_mutex_;
  DockingJob & docking_job_;
  DockingStatusConfig config_;
  DockingStatusPorts ports_;
  std::mutex relocalization_worker_mutex_;
  std::thread relocalization_worker_;
};

void DockingStatusModule::Impl::complete_post_fine_docking_relocalization(
  const std::uint64_t job_id,
  const std::string & dock_id,
  const std::string & fine_docking_status,
  const bool original_ok,
  const std::string & original_final_state,
  const bool required_before_final_state)
{
  std::string correction_resume_detail;
  const bool correction_resume_ok = ports_.set_global_correction_paused(
    job_id,
    false,
    "fine_docking_finished",
    correction_resume_detail);
  std::string localization_detail = correction_resume_detail;
  std::uint64_t relocalization_sequence = 0U;
  const std::string reason =
    "docking_after_fine:" + (dock_id.empty() ? std::string("unknown_dock") : dock_id);
  bool relocalized = false;
  if (correction_resume_ok) {
    std::string relocalization_detail;
    relocalized = ports_.trigger_localization_and_wait_for_result(
      reason,
      relocalization_detail,
      config_.docking_relocalize_wait_sec,
      &relocalization_sequence);
    localization_detail += "; " + relocalization_detail;
  }
  if (relocalized) {
    localization_detail += "; post_fine_docking_relocalization_sequence=" +
      std::to_string(relocalization_sequence) + "; settle_record_only=docked_idle";
  }

  std::lock_guard<std::mutex> lock(docking_job_mutex_);
  if (docking_job_.id != job_id || docking_job_.state != "running") {
    return;
  }
  docking_job_.post_fine_docking_relocalization_succeeded = relocalized;
  docking_job_.post_fine_docking_relocalization_detail = localization_detail;
  docking_job_.detail = localization_detail;
  if (relocalized) {
    (void)ports_.finish_docking_job_locked(
      original_ok,
      original_final_state,
      fine_docking_status + "; relocalized after fine docking: " + localization_detail);
    return;
  }
  if (required_before_final_state) {
    (void)ports_.finish_docking_job_locked(
      false,
      "failed",
      fine_docking_status + "; relocalize after fine docking failed: " + localization_detail);
    return;
  }
  (void)ports_.finish_docking_job_locked(
    original_ok,
    original_final_state,
    fine_docking_status + "; relocalize after fine docking failed: " + localization_detail);
}

void DockingStatusModule::Impl::complete_post_undock_relocalization(
  const std::uint64_t job_id,
  const std::string & dock_id,
  const std::string & undock_status,
  const bool required_before_navigation)
{
  std::string localization_detail;
  std::uint64_t relocalization_sequence = 0U;
  const std::string reason =
    "undock_after_success:" + (dock_id.empty() ? std::string("unknown_dock") : dock_id);
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id == job_id && docking_job_.state == "running") {
      docking_job_.post_undock_relocalization_started = true;
      docking_job_.post_undock_relocalization_detail =
        "triggering post-undock relocalization before pending Nav2 goal";
      docking_job_.detail =
        "post-undock relocalization is running before releasing pending Nav2 goal";
    }
  }
  update_post_undock_localization_visibility(job_id);

  std::string stale_pause_detail;
  if (!ports_.release_stale_docking_fine_pause(
      "post_undock_relocalization_before_trigger",
      stale_pause_detail))
  {
    localization_detail = stale_pause_detail;
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id == job_id && docking_job_.state == "running") {
      docking_job_.post_undock_relocalization_accepted = false;
      docking_job_.post_undock_settle_started = false;
      docking_job_.post_undock_settle_complete = false;
      docking_job_.post_undock_settle_failure_reason =
        "POST_UNDOCK_STALE_DOCKING_FINE_PAUSE: " + stale_pause_detail;
      record_post_undock_navigation_readiness_failure_locked(
        "POST_UNDOCK_STALE_DOCKING_FINE_PAUSE",
        stale_pause_detail);
      docking_job_.post_undock_relocalization_detail = stale_pause_detail;
      docking_job_.detail = stale_pause_detail;
      (void)ports_.finish_docking_job_locked(
        true,
        "undocked",
        undock_status + "; undock succeeded; post-undock navigation readiness failed: " +
        stale_pause_detail);
    }
    return;
  }

  const bool relocalized = ports_.trigger_localization_and_wait_for_result(
    reason,
    localization_detail,
    config_.undock_relocalize_wait_sec,
    &relocalization_sequence);
  bool settle_ok = true;
  std::string post_undock_failure_code;
  if (relocalized) {
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.post_undock_relocalization_accepted = true;
        docking_job_.post_undock_settle_started = true;
        docking_job_.post_undock_settle_complete = false;
        docking_job_.post_undock_settle_failure_reason.clear();
        docking_job_.post_undock_relocalization_detail =
          localization_detail + "; waiting for post-undock settle barrier before pending Nav2 goal";
        docking_job_.detail =
          "post-undock relocalization accepted; waiting for Nav2/controller TF settle";
      }
    }
    update_post_undock_localization_visibility(job_id);
    const auto settle = ports_.wait_for_relocalization_settle(
      relocalization_sequence,
      "post_undock",
      "nav2_goal",
      [this, job_id](std::string & cancel_detail) {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id != job_id || !docking_job_.cancel_requested) {
          return false;
        }
        cancel_detail = "CANCELLED_BY_APP: docking canceled during post-undock settle barrier";
        return true;
      });
    settle_ok = settle.ok;
    post_undock_failure_code =
      post_undock_failure_code_from_settle_failure(settle.failure_code);
    localization_detail += "; " +
      (settle.ok ? settle.detail : post_undock_failure_code + ": " + settle.detail);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.post_undock_settle_complete = settle.ok;
        docking_job_.post_undock_settle_failure_reason =
          settle.ok ? std::string() : post_undock_failure_code + ": " + settle.detail;
        docking_job_.post_undock_relocalization_detail = localization_detail;
        docking_job_.detail = settle.ok ?
          "post-undock settle passed; pending Nav2 goal can be released" :
          "post-undock settle failed: " + docking_job_.post_undock_settle_failure_reason;
        if (!settle.ok) {
          record_post_undock_navigation_readiness_failure_locked(
            post_undock_failure_code,
            settle.detail);
        }
      }
    }
    update_post_undock_localization_visibility(job_id);
  } else {
    post_undock_failure_code = "POST_UNDOCK_RELOCALIZATION_FAILED";
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.post_undock_relocalization_accepted = false;
        docking_job_.post_undock_settle_started = false;
        docking_job_.post_undock_settle_complete = false;
        docking_job_.post_undock_settle_failure_reason =
          post_undock_failure_code + ": " + localization_detail;
        record_post_undock_navigation_readiness_failure_locked(
          post_undock_failure_code,
          localization_detail);
      }
    }
    update_post_undock_localization_visibility(job_id);
  }

  std::lock_guard<std::mutex> lock(docking_job_mutex_);
  if (docking_job_.id != job_id || docking_job_.state != "running") {
    return;
  }
  docking_job_.post_undock_relocalization_succeeded = relocalized && settle_ok;
  docking_job_.post_undock_relocalization_detail = localization_detail;
  docking_job_.detail = localization_detail;
  if (relocalized && settle_ok) {
    docking_job_.post_undock_settle_complete = true;
    docking_job_.post_undock_settle_failure_reason.clear();
    clear_post_undock_navigation_readiness_failure_locked();
    (void)ports_.finish_docking_job_locked(
      true,
      "undocked",
      undock_status + "; relocalized after undock: " + localization_detail);
    return;
  }
  if (required_before_navigation) {
    if (docking_job_.post_undock_navigation_readiness_failure_code.empty()) {
      record_post_undock_navigation_readiness_failure_locked(
        post_undock_failure_code.empty() ?
        "POST_UNDOCK_RELOCALIZATION_FAILED" : post_undock_failure_code,
        localization_detail);
    }
    (void)ports_.finish_docking_job_locked(
      true,
      "undocked",
      undock_status +
      "; undock succeeded; post-undock navigation readiness failed, pending Nav2 goal held: " +
      localization_detail);
    return;
  }
  (void)ports_.finish_docking_job_locked(
    true,
    "undocked",
    undock_status + "; relocalize/settle after undock warning: " + localization_detail);
}

DockingStatusModule::DockingStatusModule(
  rclcpp::Logger logger,
  application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
  DockingJobStore & job_store,
  DockingStatusConfig config,
  DockingStatusPorts ports)
: impl_(std::make_unique<Impl>(
      std::move(logger),
      runtime_mode,
      job_store,
      std::move(config),
      std::move(ports)))
{
}

DockingStatusModule::~DockingStatusModule() = default;

void DockingStatusModule::handle_status(const std::string & status)
{
  impl_->handle_status(status);
}

void DockingStatusModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::docking
