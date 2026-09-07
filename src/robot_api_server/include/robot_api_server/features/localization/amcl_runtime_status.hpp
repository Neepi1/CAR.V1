#pragma once

#include <filesystem>
#include <string>

namespace robot_api_server::features::localization
{

struct AmclRuntimeStatus
{
  bool available{false};
  std::string mode;
  std::string state;
  std::string start_result;
  bool ready{false};
  bool degraded{false};
  std::string degraded_reason;
  bool process_alive{false};
  bool scan_admission_alive{false};
  int pose_publisher_count{0};
  int scan_admission_status_publisher_count{0};
  bool seed_succeeded{false};
  bool seed_response_ok{false};
  bool nomotion_probe_used{false};
  bool nomotion_pose_received{false};
  int nomotion_pose_count{0};
  double nomotion_pose_header_age_ms{-1.0};
  bool process_ready{false};
  bool seeded{false};
  bool static_standby{false};
  bool tracking_ready{false};
  bool correction_ready{false};
  bool not_moving_no_update_ok{false};
  std::string stamp;
  double stamp_sec{-1.0};
  double age_ms{-1.0};
  bool stale{true};
};

AmclRuntimeStatus read_amcl_runtime_status(
  const std::filesystem::path & status_file,
  double now_sec,
  double ttl_sec);

}  // namespace robot_api_server::features::localization
