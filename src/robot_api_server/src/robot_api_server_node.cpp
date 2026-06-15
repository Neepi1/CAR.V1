#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <fstream>
#include <future>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/switch_floor.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "robot_api_server/api_time_utils.hpp"
#include "robot_api_server/bms_contact.hpp"
#include "robot_api_server/docking_job_model.hpp"
#include "robot_api_server/docking_status_utils.hpp"
#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/floor_asset_resolver.hpp"
#include "robot_api_server/http_common.hpp"
#include "robot_api_server/localization_result_model.hpp"
#include "robot_api_server/map_catalog.hpp"
#include "robot_api_server/map_asset_io.hpp"
#include "robot_api_server/map_asset_writer.hpp"
#include "robot_api_server/map_manifest_io.hpp"
#include "robot_api_server/navigation_cancel_job_model.hpp"
#include "robot_api_server/poses_io.hpp"
#include "robot_api_server/runtime_map_context_io.hpp"
#include "robot_api_server/runtime_process_utils.hpp"
#include "robot_api_server/runtime_map_lookup.hpp"
#include "robot_api_server/robot_pose_model.hpp"
#include "robot_api_server/semantic_layer_io.hpp"
#include "robot_api_server/storage_models.hpp"
#include "robot_api_server/subscription_api.hpp"
#include "robot_api_server/subscription_manager.hpp"
#include "robot_api_server/tf_pose_utils.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

using robot_api_server::BatteryContactEvaluation;
using robot_api_server::DockingJob;
using robot_api_server::docking_job_json;
using robot_api_server::docking_status_is_failure;
using robot_api_server::docking_status_is_stopped;
using robot_api_server::docking_status_is_success;
using robot_api_server::docking_status_is_undock_failed;
using robot_api_server::docking_status_is_undocked;
using robot_api_server::docking_status_is_undocking;
using robot_api_server::evaluate_battery_charging_contact;
using robot_api_server::content_length_from_headers;
using robot_api_server::copy_file_if_exists;
using robot_api_server::copy_yaml_with_image_if_exists;
using robot_api_server::error_json;
using robot_api_server::encode_grayscale_png;
using robot_api_server::generate_current_pose_id;
using robot_api_server::HttpRequest;
using robot_api_server::HttpResponse;
using robot_api_server::is_pid_directory;
using robot_api_server::json_bool_value;
using robot_api_server::json_nested_number_value;
using robot_api_server::json_number_value;
using robot_api_server::json_object_array_value;
using robot_api_server::json_object_value;
using robot_api_server::json_string;
using robot_api_server::json_string_array_value;
using robot_api_server::json_string_value;
using robot_api_server::keepout_filter_json;
using robot_api_server::keepout_semantic_json_path;
using robot_api_server::keepout_semantic_payload_json;
using robot_api_server::LocalizationResultSnapshot;
using robot_api_server::localization_result_recent_fallback_detail;
using robot_api_server::localization_result_success_detail;
using robot_api_server::localization_result_wait_failure_detail;
using robot_api_server::lower_copy;
using robot_api_server::fill_manifest_paths;
using robot_api_server::find_floor_catalog_pose;
using robot_api_server::fixed_hex;
using robot_api_server::FloorAssetPaths;
using robot_api_server::fnv1a64;
using robot_api_server::MapCatalog;
using robot_api_server::map_manifest_json;
using robot_api_server::map_info_json;
using robot_api_server::map_yaml_text;
using robot_api_server::MapManifest;
using robot_api_server::MapYamlInfo;
using robot_api_server::NavigationCancelJob;
using robot_api_server::navigation_cancel_job_json;
using robot_api_server::no_fresh_map_robot_pose_json;
using robot_api_server::normalized_soc_percent;
using robot_api_server::normalized_frame_id;
using robot_api_server::normalize_angle;
using robot_api_server::occupancy_grid_to_image_pixels;
using robot_api_server::older_nonzero_stamp;
using robot_api_server::poses_json_array;
using robot_api_server::prepare_child_process;
using robot_api_server::process_group_has_live_process;
using robot_api_server::process_pid_is_live;
using robot_api_server::parse_http_request;
using robot_api_server::quaternion_yaw;
using robot_api_server::read_floor_poses;
using robot_api_server::read_binary_file;
using robot_api_server::read_nav_map_info;
using robot_api_server::read_optional_text_file;
using robot_api_server::read_proc_cmdline;
using robot_api_server::read_proc_environ;
using robot_api_server::read_runtime_map_context_file;
using robot_api_server::read_text_file;
using robot_api_server::reason_phrase;
using robot_api_server::resource_list_json;
using robot_api_server::resolve_mapping_2d_png;
using robot_api_server::resolve_floor_asset_paths;
using robot_api_server::RuntimeMapContext;
using robot_api_server::RobotPoseMapIdentity;
using robot_api_server::RobotPoseSnapshot;
using robot_api_server::robot_pose_json;
using robot_api_server::poses_yaml_path;
using robot_api_server::safe_asset_id;
using robot_api_server::safe_map_name;
using robot_api_server::safe_file_stem_from_display_name;
using robot_api_server::safe_pose_id;
using robot_api_server::set_close_on_exec;
using robot_api_server::signal_process_group;
using robot_api_server::stamp_to_seconds;
using robot_api_server::StoredPose;
using robot_api_server::subscription_client_id_from_body;
using robot_api_server::SubscriptionManager;
using robot_api_server::subscription_resources_from_body;
using robot_api_server::subscription_ttl_ms_from_body;
using robot_api_server::trim;
using robot_api_server::utc_timestamp_compact;
using robot_api_server::utc_timestamp_iso8601;
using robot_api_server::valid_display_map_name;
using robot_api_server::wall_time_seconds;
using robot_api_server::websocket_accept_key;
using robot_api_server::WebSocketFrame;
using robot_api_server::write_binary_file;
using robot_api_server::write_asset_report;
using robot_api_server::write_floor_poses;
using robot_api_server::write_map_manifest;
using robot_api_server::write_neutral_filter_assets;
using robot_api_server::write_pgm_file;
using robot_api_server::write_runtime_map_context_file;
using robot_api_server::write_text_file;

bool is_transient_action_client_exception(const std::exception & exc)
{
  const std::string message = exc.what();
  return message.find("Taking data from action client but no ready event") != std::string::npos;
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::string unquote_env_value(std::string value)
{
  value = trim(value);
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1U, value.size() - 2U);
  }
  std::string output;
  output.reserve(value.size());
  bool escaped = false;
  for (const char c : value) {
    if (escaped) {
      output.push_back(c);
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else {
      output.push_back(c);
    }
  }
  if (escaped) {
    output.push_back('\\');
  }
  return output;
}

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

std::optional<double> parse_utc_iso8601_seconds(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  std::tm tm{};
  std::istringstream input(text);
  input >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  if (input.fail()) {
    return std::nullopt;
  }
#if defined(_WIN32)
  const auto epoch = _mkgmtime(&tm);
#else
  const auto epoch = timegm(&tm);
#endif
  if (epoch < 0) {
    return std::nullopt;
  }
  return static_cast<double>(epoch);
}

bool latch_source_is_bms(const std::string & source)
{
  return lower_copy(source) == "bms";
}

bool latch_source_is_docking_evidence(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "docking_job" || normalized == "docking_status";
}

bool latch_source_is_manual_evidence(const std::string & source)
{
  const auto normalized = lower_copy(source);
  return normalized == "manual" ||
    normalized == "manual_confirm" ||
    normalized == "manual_clear";
}

}  // namespace

class RobotApiServerNode : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

public:
  RobotApiServerNode()
  : Node("robot_api_server")
  {
    configure_runtime_permissions();
    host_ = declare_parameter<std::string>("host", "0.0.0.0");
    port_ = declare_parameter<int>("port", 8080);
    api_token_ = declare_parameter<std::string>("api_token", "");
    max_http_connections_ =
      std::clamp(static_cast<int>(declare_parameter<int>("max_http_connections", 16)), 4, 64);
    maps_root_ = declare_parameter<std::string>("maps_root", "/workspaces/njrh-v3/workspace1/maps_release");
    map_catalog_ = std::make_unique<MapCatalog>(
      fs::path(maps_root_),
      [this](const std::string & building_id, const std::string & floor_id) {
        ensure_legacy_floor_map_manifest(building_id, floor_id);
      });
    runtime_maps_dir_ = declare_parameter<std::string>(
      "runtime_maps_dir", "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps");

    safety_estop_topic_ = declare_parameter<std::string>("safety_estop_topic", "/safety/estop");
    safety_status_topic_ = declare_parameter<std::string>("safety_status_topic", "/safety/status");
    safety_motion_allowed_topic_ =
      declare_parameter<std::string>("safety_motion_allowed_topic", "/safety/motion_allowed");
    floor_status_topic_ = declare_parameter<std::string>("floor_status_topic", "/floor_manager/status");
    bms_state_topic_ = declare_parameter<std::string>("bms_state_topic", "/battery_state");
    bms_state_max_age_sec_ = std::max(0.1, declare_parameter<double>("bms_state_max_age_sec", 3.0));
    teleop_stop_on_charging_ = declare_parameter<bool>("teleop_stop_on_charging", true);
    teleop_charging_current_min_a_ =
      std::max(0.0, declare_parameter<double>("teleop_charging_current_min_a", 0.10));
    bms_charging_contact_voltage_min_v_ =
      std::max(0.0, declare_parameter<double>("bms_charging_contact_voltage_min_v", 40.0));
    bms_charging_contact_voltage_max_v_ =
      std::max(bms_charging_contact_voltage_min_v_, declare_parameter<double>(
        "bms_charging_contact_voltage_max_v", 1000.0));
    bms_full_soc_threshold_pct_ =
      std::clamp(declare_parameter<double>("bms_full_soc_threshold_pct", 99.0), 0.0, 100.0);
    bms_full_soc_voltage_contact_enable_ =
      declare_parameter<bool>("bms_full_soc_voltage_contact_enable", true);
    floor_switch_service_ = declare_parameter<std::string>("floor_switch_service", "/floor_manager/switch_floor");
    localization_trigger_service_ =
      declare_parameter<std::string>("localization_trigger_service", "/global_localization/trigger");
    localization_bridge_force_accept_service_ = declare_parameter<std::string>(
      "localization_bridge_force_accept_service",
      "/robot_localization_bridge/force_accept_next_localization");
    localization_result_topic_ =
      declare_parameter<std::string>("localization_result_topic", "/localization_result");
    localization_bridge_status_topic_ =
      declare_parameter<std::string>("localization_bridge_status_topic", "/localization/bridge_status");
    navigate_to_pose_action_ = declare_parameter<std::string>("navigate_to_pose_action", "/navigate_to_pose");
    mapping_2d_start_command_ = declare_parameter<std::string>(
      "mapping_2d_start_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_projected_map.sh");
    mapping_2d_log_file_ =
      declare_parameter<std::string>("mapping_2d_log_file", "/tmp/njrh_mapping2d_slam_toolbox.log");
    mapping_lidar_rps_xps_state_dir_ = declare_parameter<std::string>(
      "mapping_lidar_rps_xps_state_dir",
      "/tmp/njrh_slam2d_lidar_rps_xps");
    navigation_resume_command_ = declare_parameter<std::string>(
      "navigation_resume_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh");
    navigation_resume_log_file_ =
      declare_parameter<std::string>("navigation_resume_log_file", "/tmp/njrh_navigation_resume.log");
    runtime_map_context_file_ =
      declare_parameter<std::string>("runtime_map_context_file", "/tmp/njrh_runtime_map_context.json");
    amcl_runtime_status_file_ =
      declare_parameter<std::string>("amcl_runtime_status_file", "/tmp/njrh_amcl_runtime_status.env");
    amcl_runtime_status_ttl_sec_ =
      std::max(0.0, declare_parameter<double>("amcl_runtime_status_ttl_sec", 5.0));
    last_navigation_map_file_ = declare_parameter<std::string>(
      "last_navigation_map_file",
      "/workspaces/njrh-v3/workspace1/maps_release/last_navigation_map.json");
    navigation_stop_command_ = declare_parameter<std::string>(
      "navigation_stop_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/stop_floor_navigation.sh");
    navigation_stop_log_file_ =
      declare_parameter<std::string>("navigation_stop_log_file", "/tmp/njrh_navigation_stop.log");
    docking_manager_start_command_ = declare_parameter<std::string>(
      "docking_manager_start_command",
      "/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/run_docking_manager.sh");
    docking_manager_log_file_ =
      declare_parameter<std::string>("docking_manager_log_file", "/tmp/njrh_docking_manager.log");
    docking_start_service_ = declare_parameter<std::string>("docking_start_service", "/docking/start");
    docking_stop_service_ = declare_parameter<std::string>("docking_stop_service", "/docking/stop");
    docking_undock_service_ = declare_parameter<std::string>("docking_undock_service", "/docking/undock");
    docking_status_topic_ = declare_parameter<std::string>("docking_status_topic", "/docking/status");
    docking_contact_latch_file_ = declare_parameter<std::string>(
      "docking_contact_latch_file",
      "/workspaces/njrh-v3/workspace1/maps_release/docking_contact_latch.json");
    dock_contact_latch_bms_ttl_sec_ =
      std::max(0.0, declare_parameter<double>("dock_contact_latch_bms_ttl_sec", 300.0));
    dock_contact_latch_bms_require_contact_sec_ =
      std::max(0.0, declare_parameter<double>("dock_contact_latch_bms_require_contact_sec", 2.0));
    dock_contact_latch_bms_clear_no_contact_sec_ =
      std::max(0.0, declare_parameter<double>("dock_contact_latch_bms_clear_no_contact_sec", 3.0));
    dock_contact_latch_allow_bms_stale_auto_undock_ =
      declare_parameter<bool>("dock_contact_latch_allow_bms_stale_auto_undock", false);
    dock_contact_latch_clear_when_live_undocked_no_contact_ =
      declare_parameter<bool>("dock_contact_latch_clear_when_live_undocked_no_contact", true);
    dock_contact_latch_max_age_warn_sec_ =
      std::max(0.0, declare_parameter<double>("dock_contact_latch_max_age_warn_sec", 600.0));
    docking_pre_dock_distance_m_ =
      std::max(0.05, declare_parameter<double>("docking_pre_dock_distance_m", 0.60));
    mapping_2d_live_map_topic_ = declare_parameter<std::string>("mapping_2d_live_map_topic", "/map");
    mapping_2d_live_map_max_age_sec_ =
      std::max(0.1, declare_parameter<double>("mapping_2d_live_map_max_age_sec", 3.0));
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    scan_max_age_sec_ = std::max(0.1, declare_parameter<double>("scan_max_age_sec", 2.0));
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
    tf_map_frame_ = declare_parameter<std::string>("tf_map_frame", "map");
    tf_odom_frame_ = declare_parameter<std::string>("tf_odom_frame", "odom");
    tf_base_frame_ = declare_parameter<std::string>("tf_base_frame", "base_link");
    post_relocalization_static_lidar_frame_ = normalized_frame_id(
      declare_parameter<std::string>("post_relocalization_static_lidar_frame", "lidar_level_link"));
    tf_map_frame_ = normalized_frame_id(tf_map_frame_);
    tf_odom_frame_ = normalized_frame_id(tf_odom_frame_);
    tf_base_frame_ = normalized_frame_id(tf_base_frame_);
    tf_pose_max_age_sec_ = std::max(0.1, declare_parameter<double>("tf_pose_max_age_sec", 2.0));
    robot_pose_freshness_sec_ =
      std::max(0.05, declare_parameter<double>("robot_pose_freshness_sec", 0.5));
    tf_chain_freshness_sec_ =
      std::max(0.05, declare_parameter<double>("tf_chain_freshness_sec", 0.30));
    tf_chain_settle_timeout_sec_ =
      std::max(tf_chain_freshness_sec_, declare_parameter<double>("tf_chain_settle_timeout_sec", 2.0));
    teleop_cmd_topic_ = declare_parameter<std::string>("teleop_cmd_topic", "/cmd_vel_collision_checked");
    teleop_reverse_enable_topic_ =
      declare_parameter<std::string>("teleop_reverse_enable_topic", "/ranger_mini3/teleop_allow_reverse");
    teleop_pose_topic_ = declare_parameter<std::string>("teleop_pose_topic", "/local_state/odometry");
    teleop_max_linear_x_mps_ =
      std::max(0.0, declare_parameter<double>("teleop_max_linear_x_mps", 1.00));
    teleop_max_angular_z_radps_ =
      std::max(0.0, declare_parameter<double>("teleop_max_angular_z_radps", 0.55));
    teleop_allow_reverse_ = declare_parameter<bool>("teleop_allow_reverse", false);
    teleop_require_mapping_active_ = declare_parameter<bool>("teleop_require_mapping_active", true);
    teleop_watchdog_timeout_sec_ =
      std::max(0.1, declare_parameter<double>("teleop_watchdog_timeout_sec", 0.5));
    teleop_socket_idle_timeout_sec_ = std::max(
      teleop_watchdog_timeout_sec_,
      declare_parameter<double>("teleop_socket_idle_timeout_sec", 5.0));
    teleop_repeat_rate_hz_ = std::max(1.0, declare_parameter<double>("teleop_repeat_rate_hz", 20.0));
    subscription_default_ttl_ms_ =
      std::max(1000, static_cast<int>(declare_parameter<int>("subscription_default_ttl_ms", 10000)));
    subscription_max_ttl_ms_ =
      std::max(
        subscription_default_ttl_ms_,
        static_cast<int>(declare_parameter<int>("subscription_max_ttl_ms", 60000)));
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 8.0);
    navigation_cancel_action_wait_sec_ =
      std::clamp(declare_parameter<double>("navigation_cancel_action_wait_sec", 0.75), 0.05, service_timeout_sec_);
    docking_stop_service_wait_sec_ =
      std::clamp(declare_parameter<double>("docking_stop_service_wait_sec", 3.0), 0.5, service_timeout_sec_);
    localization_trigger_service_timeout_sec_ =
      std::max(service_timeout_sec_, declare_parameter<double>("localization_trigger_service_timeout_sec", 15.0));
    localization_bridge_acceptance_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("localization_bridge_acceptance_timeout_sec", 3.0));
    localization_bridge_acceptance_max_distance_m_ =
      std::max(0.05, declare_parameter<double>("localization_bridge_acceptance_max_distance_m", 1.0));
    localization_bridge_acceptance_max_yaw_rad_ =
      std::max(0.01, declare_parameter<double>("localization_bridge_acceptance_max_yaw_rad", 0.35));
    navigation_relocalize_before_goal_ =
      declare_parameter<bool>("navigation_relocalize_before_goal", true);
    navigation_relocalize_before_goal_always_ =
      declare_parameter<bool>("navigation_relocalize_before_goal_always", false);
    navigation_relocalize_before_goal_required_ =
      declare_parameter<bool>("navigation_relocalize_before_goal_required", true);
    navigation_relocalize_wait_sec_ =
      std::max(0.5, declare_parameter<double>("navigation_relocalize_wait_sec", 8.0));
    navigation_goal_result_timeout_sec_ =
      std::max(5.0, declare_parameter<double>("navigation_goal_result_timeout_sec", 600.0));
    navigation_goal_position_success_tolerance_m_ =
      std::clamp(declare_parameter<double>("navigation_goal_position_success_tolerance_m", 0.20), 0.05, 1.0);
    navigation_final_yaw_align_enable_ =
      declare_parameter<bool>("navigation_final_yaw_align_enable", true);
    navigation_final_yaw_tolerance_rad_ =
      std::clamp(declare_parameter<double>("navigation_final_yaw_tolerance_rad", 0.05), 0.01, 1.57);
    navigation_final_yaw_align_trigger_rad_ = std::max(
      navigation_final_yaw_tolerance_rad_,
      std::clamp(declare_parameter<double>("navigation_final_yaw_align_trigger_rad", 0.08), 0.01, 1.57));
    navigation_final_yaw_align_speed_radps_ =
      std::clamp(declare_parameter<double>("navigation_final_yaw_align_speed_radps", 0.25), 0.05, 0.8);
    navigation_final_yaw_align_max_speed_radps_ = std::clamp(
      declare_parameter<double>(
        "navigation_final_yaw_align_max_speed_radps", navigation_final_yaw_align_speed_radps_),
      0.05,
      0.8);
    navigation_final_yaw_align_min_speed_radps_ = std::clamp(
      declare_parameter<double>("navigation_final_yaw_align_min_speed_radps", 0.06),
      0.0,
      navigation_final_yaw_align_max_speed_radps_);
    navigation_final_yaw_align_kp_ =
      std::clamp(declare_parameter<double>("navigation_final_yaw_align_kp", 1.2), 0.1, 5.0);
    navigation_final_yaw_align_timeout_sec_ =
      std::clamp(declare_parameter<double>("navigation_final_yaw_align_timeout_sec", 8.0), 0.5, 20.0);
    navigation_final_yaw_align_max_xy_drift_m_ =
      std::clamp(declare_parameter<double>("navigation_final_yaw_align_max_xy_drift_m", 0.08), 0.01, 0.50);
    navigation_final_yaw_align_require_fresh_pose_ =
      declare_parameter<bool>("navigation_final_yaw_align_require_fresh_pose", true);
    navigation_final_yaw_align_cmd_topic_ =
      declare_parameter<std::string>("navigation_final_yaw_align_cmd_topic", "/cmd_vel_collision_checked");
    if (navigation_final_yaw_align_cmd_topic_ != "/cmd_vel_nav" &&
      navigation_final_yaw_align_cmd_topic_ != "/cmd_vel_collision_checked")
    {
      RCLCPP_WARN(
        get_logger(),
        "navigation_final_yaw_align_cmd_topic=%s is not allowed; using /cmd_vel_collision_checked",
        navigation_final_yaw_align_cmd_topic_.c_str());
      navigation_final_yaw_align_cmd_topic_ = "/cmd_vel_collision_checked";
    }
    navigation_final_yaw_align_bypass_collision_monitor_ = declare_parameter<bool>(
      "navigation_final_yaw_align_bypass_collision_monitor",
      navigation_final_yaw_align_cmd_topic_ == "/cmd_vel_collision_checked");
    navigation_final_yaw_align_bypass_collision_monitor_ =
      navigation_final_yaw_align_cmd_topic_ == "/cmd_vel_collision_checked";
    navigation_final_yaw_align_zero_cmd_count_ = static_cast<int>(
      std::clamp(declare_parameter<int>("navigation_final_yaw_align_zero_cmd_count", 3), 1L, 10L));
    navigation_lifecycle_check_timeout_sec_ =
      std::clamp(declare_parameter<double>("navigation_lifecycle_check_timeout_sec", 0.35), 0.05, 3.0);
    docking_navigation_start_wait_sec_ =
      std::max(service_timeout_sec_, declare_parameter<double>("docking_navigation_start_wait_sec", 45.0));
    docking_predock_nav_timeout_sec_ =
      std::max(5.0, declare_parameter<double>("docking_predock_nav_timeout_sec", 180.0));
    docking_relocalize_before_predock_ =
      declare_parameter<bool>("docking_relocalize_before_predock", true);
    docking_relocalize_after_predock_ =
      declare_parameter<bool>("docking_relocalize_after_predock", true);
    docking_relocalize_after_predock_required_ =
      declare_parameter<bool>("docking_relocalize_after_predock_required", true);
    docking_relocalize_after_fine_docking_ =
      declare_parameter<bool>("docking_relocalize_after_fine_docking", true);
    docking_relocalize_after_fine_docking_required_ =
      declare_parameter<bool>("docking_relocalize_after_fine_docking_required", false);
    docking_validate_predock_pose_after_relocalization_ =
      declare_parameter<bool>("docking_validate_predock_pose_after_relocalization", true);
    docking_predock_pose_max_distance_m_ =
      std::max(0.05, declare_parameter<double>("docking_predock_pose_max_distance_m", 0.35));
    docking_predock_pose_max_yaw_rad_ =
      std::max(0.01, declare_parameter<double>("docking_predock_pose_max_yaw_rad", 0.35));
    docking_manual_predock_distance_check_enable_ =
      declare_parameter<bool>("docking_manual_predock_distance_check_enable", false);
    docking_manual_predock_min_distance_m_ =
      std::clamp(declare_parameter<double>("docking_manual_predock_min_distance_m", 0.50), 0.05, 5.00);
    docking_manual_predock_max_distance_m_ = std::max(
      docking_manual_predock_min_distance_m_ + 0.05,
      declare_parameter<double>("docking_manual_predock_max_distance_m", 1.20));
    docking_manual_predock_max_yaw_error_rad_ =
      std::clamp(declare_parameter<double>("docking_manual_predock_max_yaw_error_rad", 0.80), 0.05, 3.14);
    docking_relocalize_wait_sec_ =
      std::max(0.5, declare_parameter<double>("docking_relocalize_wait_sec", 8.0));
    docking_relocalize_recent_result_max_age_sec_ =
      std::max(0.0, declare_parameter<double>("docking_relocalize_recent_result_max_age_sec", 5.0));
    undock_relocalize_after_success_ =
      declare_parameter<bool>("undock_relocalize_after_success", true);
    undock_relocalize_wait_sec_ =
      std::max(0.5, declare_parameter<double>("undock_relocalize_wait_sec", docking_relocalize_wait_sec_));
    local_costmap_topic_ = declare_parameter<std::string>("local_costmap_topic", "/local_costmap/costmap");
    post_relocalization_settle_enabled_ =
      declare_parameter<bool>("post_relocalization_settle_enabled", true);
    post_relocalization_settle_min_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("post_relocalization_settle_min_ms", 800)));
    post_relocalization_settle_max_ms_ = std::max(
      post_relocalization_settle_min_ms_,
      static_cast<int>(declare_parameter<int>("post_relocalization_settle_max_ms", 3000)));
    post_relocalization_stable_tf_samples_ =
      std::max(1, static_cast<int>(declare_parameter<int>("post_relocalization_stable_tf_samples", 5)));
    post_relocalization_tf_sample_period_ms_ =
      std::max(20, static_cast<int>(declare_parameter<int>("post_relocalization_tf_sample_period_ms", 100)));
    post_relocalization_zero_cmd_ =
      declare_parameter<bool>("post_relocalization_zero_cmd", true);
    post_relocalization_require_local_costmap_update_ =
      declare_parameter<bool>("post_relocalization_require_local_costmap_update", true);
    post_relocalization_required_local_costmap_updates_ =
      std::max(
        0,
        static_cast<int>(declare_parameter<int>("post_relocalization_required_local_costmap_updates", 2)));
    post_relocalization_reject_if_new_message_filter_drop_ =
      declare_parameter<bool>("post_relocalization_reject_if_new_message_filter_drop", true);
    post_relocalization_large_correction_translation_m_ =
      std::max(0.0, declare_parameter<double>("post_relocalization_large_correction_translation_m", 0.5));
    post_relocalization_large_correction_yaw_rad_ =
      std::max(0.0, declare_parameter<double>("post_relocalization_large_correction_yaw_rad", 0.3));
    post_relocalization_large_correction_min_ms_ = std::max(
      post_relocalization_settle_min_ms_,
      static_cast<int>(declare_parameter<int>("post_relocalization_large_correction_min_ms", 1500)));
    docking_cancel_active_goal_before_predock_ =
      declare_parameter<bool>("docking_cancel_active_goal_before_predock", true);
    navigation_auto_undock_timeout_sec_ =
      std::max(service_timeout_sec_, declare_parameter<double>("navigation_auto_undock_timeout_sec", 28.0));
    docking_undock_charging_retry_sec_ =
      std::max(0.0, declare_parameter<double>("docking_undock_charging_retry_sec", 3.0));

    estop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_estop_topic_, rclcpp::QoS(10).transient_local());
    teleop_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(teleop_cmd_topic_, rclcpp::QoS(10));
    navigation_final_yaw_cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(navigation_final_yaw_align_cmd_topic_, rclcpp::QoS(10));
    teleop_reverse_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(teleop_reverse_enable_topic_, rclcpp::QoS(1));
    bms_state_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      bms_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        handle_bms_state(msg);
      });
    docking_status_sub_ = create_subscription<std_msgs::msg::String>(
      docking_status_topic_, rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_docking_status(msg->data);
      });
    localization_result_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_result_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handle_localization_result(msg);
      });
    localization_bridge_status_sub_ = create_subscription<std_msgs::msg::String>(
      localization_bridge_status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_localization_bridge_status(msg);
      });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        handle_local_costmap(msg);
      });
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_static_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        handle_tf_static_message(msg);
      });
    rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100),
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        handle_rosout_message(msg);
      });
    teleop_repeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / teleop_repeat_rate_hz_)),
      [this]() { on_teleop_repeat_timer(); });
    subscription_manager_ = std::make_unique<SubscriptionManager>(
      std::vector<std::string>{"status", "live_map", "scan", "tf", "teleop"},
      [this](const std::string & resource, const bool active) {
        set_subscription_resource_active(resource, active);
      });
    set_status_subscriptions_active(true);
    // TF is a process-level localization input. Keeping it resident avoids
    // reliable /tf DDS endpoint churn from high-rate /api/v1/robot/pose polling.
    set_tf_subscription_active(true);
    subscription_ttl_timer_ = create_wall_timer(
      1s,
      [this]() {
        if (subscription_manager_) {
          subscription_manager_->expire();
        }
      });

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    floor_switch_client_ = create_client<robot_interfaces::srv::SwitchFloor>(
      floor_switch_service_, rmw_qos_profile_services_default, callback_group_);
    localization_trigger_client_ = create_client<robot_interfaces::srv::TriggerLocalization>(
      localization_trigger_service_, rmw_qos_profile_services_default, callback_group_);
    localization_bridge_force_accept_client_ = create_client<std_srvs::srv::Trigger>(
      localization_bridge_force_accept_service_, rmw_qos_profile_services_default, callback_group_);
    for (const auto & node_name : navigation_lifecycle_node_names()) {
      navigation_lifecycle_clients_[node_name] = create_client<lifecycle_msgs::srv::GetState>(
        node_name + "/get_state", rmw_qos_profile_services_default, callback_group_);
    }
    docking_start_client_ = create_client<std_srvs::srv::Trigger>(
      docking_start_service_, rmw_qos_profile_services_default, callback_group_);
    docking_stop_client_ = create_client<std_srvs::srv::Trigger>(
      docking_stop_service_, rmw_qos_profile_services_default, callback_group_);
    docking_undock_client_ = create_client<std_srvs::srv::Trigger>(
      docking_undock_service_, rmw_qos_profile_services_default, callback_group_);
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_);

    start_server();
  }

  ~RobotApiServerNode() override
  {
    stop_server();
    join_navigation_goal_worker();
    join_navigation_cancel_worker();
    join_docking_worker();
    join_docking_relocalization_worker();
  }

private:
  void configure_runtime_permissions() const
  {
    ::umask(0002);
  }

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

  struct BmsChargingContactSnapshot
  {
    bool have_state{false};
    bool have_soc{false};
    bool fresh{false};
    bool contact{false};
    bool contact_stable{false};
    std::string reason{"no_bms_state"};
    double age_sec{-1.0};
    double contact_stable_duration_sec{0.0};
    double no_contact_duration_sec{0.0};
    double soc{0.0};
    double voltage{0.0};
    double current{0.0};
    double temperature{0.0};
    int power_supply_status{sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
    int power_supply_health{sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN};
    int power_supply_technology{sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN};
    bool present{false};
  };

  struct DockContactLatchSnapshot
  {
    bool valid{false};
    bool docked{false};
    bool latched_docked{false};
    std::string source{"none"};
    std::string reason{"not_latched"};
    std::string building_id;
    std::string floor_id;
    std::string map_id;
    std::string dock_id;
    std::string latched_at;
    std::string last_confirmed_at;
    std::string cleared_at;
    std::string clear_reason;
    std::string note;
    std::string updated_at;
    double age_sec{-1.0};
    bool source_bms{false};
    bool stale{false};
    bool contradicted_by_live_state{false};
  };

  struct PreNavigationDockCheck
  {
    RuntimeModeSnapshot runtime;
    BmsChargingContactSnapshot bms;
    DockContactLatchSnapshot dock_latch;
    bool runtime_state_docked{false};
    bool runtime_state_charging{false};
    bool runtime_state_undocking{false};
    bool docking_status_indicates_docked{false};
    bool docking_status_indicates_charging{false};
    bool docking_status_indicates_undocking{false};
    bool dock_latch_indicates_docked{false};
    bool dock_contact_latch_present{false};
    bool dock_contact_latch_latched_docked{false};
    std::string dock_contact_latch_source;
    std::string dock_contact_latch_reason;
    double dock_contact_latch_age_sec{-1.0};
    bool dock_contact_latch_stale{false};
    bool dock_contact_latch_contradicted_by_live_state{false};
    bool dock_contact_latch_auto_cleared{false};
    std::string dock_contact_latch_clear_reason;
    bool live_docking_state_undocked{false};
    bool live_bms_charging_contact_stable{false};
    bool strong_live_docked{false};
    bool latch_valid_for_auto_undock{false};
    bool inferred_docked{false};
    bool final_is_docked_or_charging{false};
    bool final_auto_undock_required{false};
    bool docking_active_not_docked_block{false};
    bool can_auto_undock{false};
    std::string docked_state_class{"UNKNOWN"};
    std::vector<std::string> docked_evidence;
    std::vector<std::string> docked_warnings;
    std::string auto_undock_reason{"not_docked"};
  };

  struct NavigationRelocalizationDecision
  {
    bool requested{false};
    bool required{false};
    std::string detail;
  };

  struct NavigationLifecycleSnapshot
  {
    bool active{false};
    std::string detail;
  };

  struct TriggerServiceObservation
  {
    bool service_called{false};
    bool service_success{false};
    std::string message;
  };

  struct NavigationGoalJob
  {
    std::uint64_t id{0U};
    std::string state{"idle"};
    std::string phase{"idle"};
    std::string pose_id;
    std::string building_id;
    std::string floor_id;
    std::string detail;
    std::string started_at;
    std::string completed_at;
    double target_x{0.0};
    double target_y{0.0};
    double target_yaw{0.0};
    double final_distance_m{-1.0};
    double final_yaw_error_rad{-1.0};
    int nav2_result_code{0};
    bool position_reached{false};
    bool nav2_succeeded{false};
    bool final_yaw_align_requested{false};
    bool final_yaw_align_attempted{false};
    bool final_yaw_align_succeeded{false};
    bool final_yaw_align_blocked{false};
    std::string final_yaw_align_blocked_reason;
    double final_yaw_align_duration_sec{-1.0};
    double final_yaw_align_timeout_sec{-1.0};
    double final_yaw_align_target_yaw_rad{0.0};
    double final_yaw_align_initial_yaw_error_rad{-1.0};
    double final_yaw_align_final_yaw_error_rad{-1.0};
    double final_yaw_align_max_xy_drift_m{-1.0};
    double final_yaw_align_observed_xy_drift_m{-1.0};
    std::string final_yaw_align_cmd_topic;
    bool final_yaw_align_bypass_collision_monitor{true};
    bool final_pose_verified{false};
    std::string final_pose_verify_reason;
    bool cancel_requested{false};
    std::string cancel_reason;
  };

  struct FinalPoseCheck
  {
    RobotPoseSnapshot pose;
    bool pose_available{false};
    bool position_reached{false};
    double distance_m{-1.0};
    double yaw_error_rad{-1.0};
    std::string reason;
  };

  struct FinalYawAlignResult
  {
    bool attempted{false};
    bool succeeded{false};
    bool blocked{false};
    bool canceled{false};
    std::string phase{"position_reached_yaw_aligning"};
    std::string detail;
    std::string blocked_reason;
    double duration_sec{-1.0};
    double initial_yaw_error_rad{-1.0};
    double final_yaw_error_rad{-1.0};
    double observed_xy_drift_m{0.0};
  };

  struct TfChainFreshnessSnapshot
  {
    bool have_map_to_odom{false};
    bool have_odom_to_base{false};
    bool have_map_pose{false};
    double map_to_odom_age_sec{-1.0};
    double odom_to_base_age_sec{-1.0};
    double map_pose_age_sec{-1.0};
    double map_to_odom_stamp_sec{0.0};
    double odom_to_base_stamp_sec{0.0};
  };

  struct BridgeStatusSnapshot
  {
    bool available{false};
    std::string raw;
    std::chrono::steady_clock::time_point received_at{};
    double age_sec{-1.0};
    bool has_map_to_odom{false};
    std::string map_to_odom_publisher_owner{"unknown"};
    double map_to_odom_age_ms{-1.0};
    std::uint64_t last_explicit_relocalization_sequence{0U};
    double last_explicit_relocalization_accept_time{0.0};
    std::string last_explicit_relocalization_source{"none"};
    double last_accepted_correction_translation_m{0.0};
    double last_accepted_correction_yaw_rad{0.0};
    bool amcl_input_enabled{false};
    bool amcl_ready{false};
    bool amcl_degraded{false};
    std::string amcl_degraded_reason;
    std::string amcl_status_source;
    bool amcl_status_file_stale{true};
    double amcl_status_age_ms{-1.0};
    bool amcl_process_ready{false};
    bool amcl_seeded{false};
    bool amcl_seed_response_ok{false};
    bool amcl_nomotion_pose_received{false};
    bool amcl_static_standby{false};
    bool amcl_tracking_ready{false};
    bool amcl_correction_ready{false};
    bool amcl_not_moving_no_update_ok{false};
    bool amcl_scan_admission_enabled{false};
    bool amcl_scan_admission_alive{false};
    bool amcl_message_filter_drop_detected{false};
    std::string amcl_scan_admission_last_error{"none"};
    bool localization_degraded{false};
  };

  struct PostRelocalizationSettleResult
  {
    bool ok{false};
    std::string failure_code{"POST_RELOCALIZATION_SETTLE_TIMEOUT"};
    std::string detail;
    std::uint64_t expected_sequence{0U};
    std::uint64_t observed_sequence{0U};
    int stable_samples{0};
    std::uint64_t local_costmap_updates{0U};
    double elapsed_ms{0.0};
  };

  struct PostRelocalizationSettleState
  {
    bool required{false};
    bool in_progress{false};
    bool complete{true};
    std::string reason{"none"};
    std::string target_stage{"none"};
    std::string failure_reason{"none"};
    std::string detail;
    std::uint64_t expected_sequence{0U};
    int min_ms{0};
    double start_wall_time{0.0};
  };

  std::chrono::nanoseconds service_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(service_timeout_sec_));
  }

  std::chrono::nanoseconds navigation_cancel_action_wait() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(navigation_cancel_action_wait_sec_));
  }

  std::chrono::nanoseconds docking_stop_service_wait() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(docking_stop_service_wait_sec_));
  }

  std::chrono::nanoseconds localization_trigger_service_timeout(const double result_wait_timeout_sec) const
  {
    const double result_wait =
      result_wait_timeout_sec > 0.0 ? result_wait_timeout_sec : docking_relocalize_wait_sec_;
    const double timeout_sec =
      std::max(localization_trigger_service_timeout_sec_, service_timeout_sec_ + result_wait);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));
  }

  void set_mapping_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    mapping_runtime_active_ = active;
    mapping_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      navigation_runtime_active_ = false;
      navigation_runtime_state_ = "stopped";
      docking_runtime_active_ = false;
      docking_runtime_state_ = "stopped";
    }
  }

  void set_navigation_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    navigation_runtime_active_ = active;
    navigation_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }
  }

  void set_docking_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & message = "",
    const bool healthy = true)
  {
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    docking_runtime_active_ = active;
    docking_runtime_state_ = state;
    runtime_healthy_ = healthy;
    runtime_message_ = message;
    if (active) {
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }
  }

  RuntimeModeSnapshot runtime_mode_snapshot() const
  {
    RuntimeModeSnapshot snapshot;
    std::lock_guard<std::mutex> lock(runtime_mode_mutex_);
    snapshot.mapping_active = mapping_runtime_active_;
    snapshot.navigation_active = navigation_runtime_active_;
    snapshot.docking_active = docking_runtime_active_;
    snapshot.mapping_state = mapping_runtime_state_;
    snapshot.navigation_state = navigation_runtime_state_;
    snapshot.docking_state = docking_runtime_state_;
    snapshot.docking_status = docking_runtime_status_;
    snapshot.docking_dock_id = docking_runtime_dock_id_;
    snapshot.healthy = runtime_healthy_;
    snapshot.message = runtime_message_;
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

  void set_subscription_resource_active(const std::string & resource, const bool active)
  {
    if (resource == "status") {
      // Safety and floor health are process-level health inputs, not page-scoped telemetry.
      // Keep them resident so /api/v1/status and safety gates do not depend on App leases.
      set_status_subscriptions_active(true);
    } else if (resource == "live_map") {
      set_live_map_subscription_active(active);
    } else if (resource == "scan") {
      set_scan_subscription_active(active);
    } else if (resource == "tf") {
      // Keep TF resident; page leases only report interest and must not tear it down.
      set_tf_subscription_active(true);
    } else if (resource == "teleop" && !active) {
      clear_teleop_command();
    }
  }

  void clear_live_map_cache()
  {
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    latest_live_map_ = nav_msgs::msg::OccupancyGrid{};
    have_live_map_ = false;
    latest_live_map_received_at_ = {};
  }

  void set_status_subscriptions_active(const bool active)
  {
    std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
    if (active) {
      const auto safety_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      if (!safety_status_sub_) {
        safety_status_sub_ = create_subscription<std_msgs::msg::String>(
          safety_status_topic_, safety_state_qos,
          [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_safety_status_ = msg->data;
          });
      }
      if (!motion_allowed_sub_) {
        motion_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
          safety_motion_allowed_topic_, safety_state_qos,
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_motion_allowed_ = msg->data;
            have_motion_allowed_ = true;
          });
      }
      if (!floor_status_sub_) {
        floor_status_sub_ = create_subscription<std_msgs::msg::String>(
          floor_status_topic_, rclcpp::QoS(10),
          [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            latest_floor_status_ = msg->data;
          });
      }
      return;
    }

    safety_status_sub_.reset();
    motion_allowed_sub_.reset();
    floor_status_sub_.reset();
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    latest_safety_status_ = "UNKNOWN";
    latest_floor_status_ = "UNKNOWN";
    latest_motion_allowed_ = false;
    have_motion_allowed_ = false;
  }

  void reconcile_live_map_subscription_locked()
  {
    const bool required = live_map_page_subscription_active_ || live_map_mapping_cache_active_;
    if (required) {
      if (!live_map_sub_) {
        live_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
          mapping_2d_live_map_topic_, rclcpp::QoS(1).reliable(),
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> map_lock(live_map_mutex_);
            latest_live_map_ = *msg;
            latest_live_map_received_at_ = std::chrono::steady_clock::now();
            have_live_map_ = true;
          });
      }
      return;
    }
    live_map_sub_.reset();
  }

  void set_live_map_subscription_active(const bool active)
  {
    bool should_clear_cache = false;
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      live_map_page_subscription_active_ = active;
      reconcile_live_map_subscription_locked();
      should_clear_cache = !live_map_sub_;
    }
    if (should_clear_cache) {
      clear_live_map_cache();
    }
  }

  void set_mapping_live_map_cache_active(const bool active)
  {
    bool should_clear_cache = false;
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      live_map_mapping_cache_active_ = active;
      reconcile_live_map_subscription_locked();
      should_clear_cache = !live_map_sub_;
    }
    if (should_clear_cache) {
      clear_live_map_cache();
    }
  }

  void set_scan_subscription_active(const bool active)
  {
    {
      std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
      if (active) {
        if (!scan_sub_) {
          scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::QoS(10),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
              std::lock_guard<std::mutex> state_lock(state_mutex_);
              latest_scan_frame_ = msg->header.frame_id;
              latest_scan_range_count_ = msg->ranges.size();
              latest_scan_angle_min_ = msg->angle_min;
              latest_scan_angle_max_ = msg->angle_max;
              latest_scan_received_at_ = std::chrono::steady_clock::now();
              have_scan_ = true;
            });
        }
        return;
      }
      scan_sub_.reset();
    }
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    latest_scan_frame_.clear();
    latest_scan_range_count_ = 0U;
    have_scan_ = false;
  }

  void set_tf_subscription_active(const bool active)
  {
    if (!active) {
      return;
    }
    std::lock_guard<std::mutex> lock(subscription_lifecycle_mutex_);
    if (!tf_sub_) {
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, rclcpp::QoS(100),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          handle_tf_message(msg);
        });
    }
  }

  RobotPoseSnapshot current_robot_pose_snapshot()
  {
    RobotPoseSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_pose_ || latest_pose_frame_ != tf_map_frame_) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.frame_id = latest_pose_frame_;
    snapshot.child_frame_id = tf_base_frame_;
    snapshot.x = latest_pose_x_;
    snapshot.y = latest_pose_y_;
    snapshot.yaw = latest_pose_yaw_;
    snapshot.stamp_sec = latest_pose_stamp_sec_;
    snapshot.age_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_pose_received_at_).count();
    return snapshot;
  }

  TfChainFreshnessSnapshot tf_chain_freshness_snapshot()
  {
    TfChainFreshnessSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    snapshot.have_map_to_odom = have_map_to_odom_;
    snapshot.have_odom_to_base = have_odom_to_base_;
    snapshot.have_map_pose = have_pose_ && latest_pose_frame_ == tf_map_frame_;
    snapshot.map_to_odom_stamp_sec = latest_map_to_odom_stamp_sec_;
    snapshot.odom_to_base_stamp_sec = latest_odom_to_base_stamp_sec_;
    if (have_map_to_odom_) {
      snapshot.map_to_odom_age_sec =
        std::chrono::duration<double>(now - latest_map_to_odom_received_at_).count();
    }
    if (have_odom_to_base_) {
      snapshot.odom_to_base_age_sec =
        std::chrono::duration<double>(now - latest_odom_to_base_received_at_).count();
    }
    if (snapshot.have_map_pose) {
      snapshot.map_pose_age_sec =
        std::chrono::duration<double>(now - latest_pose_received_at_).count();
    }
    return snapshot;
  }

  std::string tf_chain_freshness_detail(const TfChainFreshnessSnapshot & snapshot) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "map_to_odom=" << (snapshot.have_map_to_odom ? "yes" : "no")
        << " age=" << snapshot.map_to_odom_age_sec
        << " odom_to_base=" << (snapshot.have_odom_to_base ? "yes" : "no")
        << " age=" << snapshot.odom_to_base_age_sec
        << " map_pose=" << (snapshot.have_map_pose ? "yes" : "no")
        << " age=" << snapshot.map_pose_age_sec
        << " freshness_limit=" << tf_chain_freshness_sec_;
    return out.str();
  }

  bool tf_chain_is_fresh(const TfChainFreshnessSnapshot & snapshot) const
  {
    return snapshot.have_map_to_odom && snapshot.have_odom_to_base && snapshot.have_map_pose &&
           snapshot.map_to_odom_age_sec >= 0.0 &&
           snapshot.odom_to_base_age_sec >= 0.0 &&
           snapshot.map_pose_age_sec >= 0.0 &&
           snapshot.map_to_odom_age_sec <= tf_chain_freshness_sec_ &&
           snapshot.odom_to_base_age_sec <= tf_chain_freshness_sec_ &&
           snapshot.map_pose_age_sec <= robot_pose_freshness_sec_;
  }

  bool wait_for_fresh_tf_chain(const std::string & reason, std::string & detail)
  {
    set_tf_subscription_active(true);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(tf_chain_settle_timeout_sec_));
    TfChainFreshnessSnapshot snapshot;
    std::string last_detail = "no TF sample yet";
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = tf_chain_freshness_snapshot();
      last_detail = tf_chain_freshness_detail(snapshot);
      if (tf_chain_is_fresh(snapshot)) {
        detail = "fresh TF chain before " + reason + ": " + last_detail;
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    detail = "timed out waiting for fresh TF chain before " + reason + ": " + last_detail;
    return false;
  }

  void update_post_relocalization_settle_state(
    const std::function<void(PostRelocalizationSettleState &)> & update)
  {
    std::lock_guard<std::mutex> lock(post_relocalization_settle_mutex_);
    update(post_relocalization_settle_state_);
  }

  PostRelocalizationSettleState post_relocalization_settle_state_snapshot() const
  {
    std::lock_guard<std::mutex> lock(post_relocalization_settle_mutex_);
    return post_relocalization_settle_state_;
  }

  std::string post_relocalization_settle_state_json() const
  {
    const auto state = post_relocalization_settle_state_snapshot();
    std::ostringstream out;
    out << "{\"required\":" << (state.required ? "true" : "false")
        << ",\"in_progress\":" << (state.in_progress ? "true" : "false")
        << ",\"complete\":" << (state.complete ? "true" : "false")
        << ",\"reason\":" << json_string(state.reason)
        << ",\"target_stage\":" << json_string(state.target_stage)
        << ",\"expected_sequence\":" << state.expected_sequence
        << ",\"start_time\":" << std::fixed << std::setprecision(3) << state.start_wall_time
        << ",\"min_ms\":" << state.min_ms
        << ",\"failure_reason\":" << json_string(state.failure_reason)
        << ",\"detail\":" << json_string(state.detail) << "}";
    return out.str();
  }

  PostRelocalizationSettleResult wait_for_post_relocalization_settle_barrier(
    const std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested = {})
  {
    PostRelocalizationSettleResult result;
    result.expected_sequence = expected_sequence;
    if (!post_relocalization_settle_enabled_) {
      result.ok = true;
      result.failure_code = "NONE";
      result.detail = "post relocalization settle disabled";
      return result;
    }

    if (expected_sequence == 0U) {
      result.failure_code = "POST_RELOCALIZATION_SEQUENCE_MISMATCH";
      result.detail = "post relocalization settle missing expected explicit relocalization sequence";
      return result;
    }

    const auto initial_bridge = bridge_status_snapshot();
    int min_ms = post_relocalization_settle_min_ms_;
    if (std::fabs(initial_bridge.last_accepted_correction_translation_m) >=
      post_relocalization_large_correction_translation_m_ ||
      std::fabs(initial_bridge.last_accepted_correction_yaw_rad) >=
      post_relocalization_large_correction_yaw_rad_)
    {
      min_ms = std::max(min_ms, post_relocalization_large_correction_min_ms_);
    }

    const auto start = std::chrono::steady_clock::now();
    auto settle_start = start;
    auto deadline = start + std::chrono::milliseconds(post_relocalization_settle_max_ms_);
    auto min_deadline = start + std::chrono::milliseconds(min_ms);
    const auto sample_period = std::chrono::milliseconds(post_relocalization_tf_sample_period_ms_);
    const std::uint64_t baseline_costmap_updates = local_costmap_update_count();
    const std::uint64_t baseline_local_costmap_drops = local_costmap_message_filter_drop_count();
    int stable_samples = 0;
    bool sequence_observed = false;
    auto mark_sequence_observed = [&]() {
      if (sequence_observed) {
        return;
      }
      sequence_observed = true;
      settle_start = std::chrono::steady_clock::now();
      deadline = settle_start + std::chrono::milliseconds(post_relocalization_settle_max_ms_);
      min_deadline = settle_start + std::chrono::milliseconds(min_ms);
      stable_samples = 0;
    };
    std::string last_failure_code = "POST_RELOCALIZATION_SETTLE_TIMEOUT";
    std::string last_detail = "waiting for first settle sample";

    update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
      state.required = true;
      state.in_progress = true;
      state.complete = false;
      state.reason = reason;
      state.target_stage = target_next_stage;
      state.failure_reason = "none";
      state.detail = "waiting for post relocalization settle";
      state.expected_sequence = expected_sequence;
      state.min_ms = min_ms;
      state.start_wall_time = wall_time_seconds();
    });

    if (post_relocalization_zero_cmd_) {
      clear_teleop_command();
      publish_teleop_zero_burst();
      publish_final_yaw_align_zero_burst();
    }

    while (std::chrono::steady_clock::now() <= deadline) {
      if (post_relocalization_zero_cmd_) {
        clear_teleop_command();
        publish_teleop_zero_burst();
        publish_final_yaw_align_zero_burst();
      }

      std::string cancel_detail;
      if (cancel_requested && cancel_requested(cancel_detail)) {
        result.failure_code = "CANCELLED_BY_APP";
        result.detail = cancel_detail.empty() ? "post relocalization settle canceled by app" : cancel_detail;
        update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
          state.in_progress = false;
          state.complete = false;
          state.failure_reason = result.failure_code;
          state.detail = result.detail;
        });
        return result;
      }

      const auto bridge = bridge_status_snapshot();
      const auto tf = tf_chain_freshness_snapshot();
      const std::uint64_t costmap_updates =
        local_costmap_update_count() - baseline_costmap_updates;
      result.observed_sequence = bridge.last_explicit_relocalization_sequence;
      result.local_costmap_updates = costmap_updates;

      bool sample_ok = true;
      std::ostringstream sample_detail;
      sample_detail << std::fixed << std::setprecision(3)
                    << "reason=" << reason
                    << " target=" << target_next_stage
                    << " expected_seq=" << expected_sequence
                    << " observed_seq=" << bridge.last_explicit_relocalization_sequence
                    << " bridge_age_sec=" << bridge.age_sec
                    << " map_to_odom_age_ms=" << bridge.map_to_odom_age_ms
                    << " costmap_updates=" << costmap_updates
                    << " stable_samples=" << stable_samples;

      if (!bridge.available) {
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; bridge_status unavailable";
      } else if (bridge.map_to_odom_publisher_owner != "robot_localization_bridge") {
        result.failure_code = "POST_RELOCALIZATION_WRONG_MAP_ODOM_OWNER";
        result.detail = "post relocalization settle rejected wrong map->odom owner: " +
          bridge.map_to_odom_publisher_owner;
        update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
          state.in_progress = false;
          state.complete = false;
          state.failure_reason = result.failure_code;
          state.detail = result.detail;
        });
        return result;
      } else if (bridge.last_explicit_relocalization_sequence != expected_sequence) {
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_SEQUENCE_MISMATCH";
        sample_detail << "; sequence mismatch";
        if (bridge.last_explicit_relocalization_sequence > expected_sequence) {
          result.failure_code = last_failure_code;
          result.detail = sample_detail.str();
          update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
            state.in_progress = false;
            state.complete = false;
            state.failure_reason = result.failure_code;
            state.detail = result.detail;
          });
          return result;
        }
      } else if (!bridge.has_map_to_odom)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; bridge has no map->odom";
      } else if (!tf.have_map_to_odom || tf.map_to_odom_age_sec < 0.0 ||
        tf.map_to_odom_age_sec > tf_chain_freshness_sec_)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_MAP_ODOM_NOT_FRESH";
        sample_detail << "; api tf map->odom not fresh: " << tf_chain_freshness_detail(tf);
      } else if (!tf.have_odom_to_base || tf.odom_to_base_age_sec < 0.0 ||
        tf.odom_to_base_age_sec > tf_chain_freshness_sec_)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_ODOM_BASE_NOT_FRESH";
        sample_detail << "; api tf odom->base_link not fresh: " << tf_chain_freshness_detail(tf);
      } else if (!tf.have_map_pose || tf.map_pose_age_sec < 0.0 ||
        tf.map_pose_age_sec > robot_pose_freshness_sec_)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE";
        sample_detail << "; api map pose not fresh: " << tf_chain_freshness_detail(tf);
      } else if (!base_to_lidar_static_tf_ready()) {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_TF_CHAIN_UNSTABLE";
        sample_detail << "; static " << tf_base_frame_ << "->"
                      << post_relocalization_static_lidar_frame_ << " not observed";
      } else if (post_relocalization_require_local_costmap_update_ &&
        costmap_updates < static_cast<std::uint64_t>(post_relocalization_required_local_costmap_updates_))
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_LOCAL_COSTMAP_NOT_UPDATED";
        sample_detail << "; local_costmap updates below required="
                      << post_relocalization_required_local_costmap_updates_;
      } else if (post_relocalization_reject_if_new_message_filter_drop_ &&
        local_costmap_message_filter_drop_count() > baseline_local_costmap_drops)
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_LOCAL_COSTMAP_TF_DROPS";
        sample_detail << "; local_costmap MessageFilter drop: "
                      << last_local_costmap_message_filter_drop_text();
      } else if (bridge.amcl_scan_admission_enabled &&
        (!bridge.amcl_scan_admission_alive ||
        bridge.amcl_message_filter_drop_detected ||
        lower_copy(bridge.amcl_scan_admission_last_error).find("tf") != std::string::npos ||
        lower_copy(bridge.amcl_scan_admission_last_error).find("transform") != std::string::npos))
      {
        mark_sequence_observed();
        sample_ok = false;
        last_failure_code = "POST_RELOCALIZATION_SCAN_ADMISSION_TF_ERROR";
        sample_detail << "; amcl scan admission not clean: alive="
                      << (bridge.amcl_scan_admission_alive ? "true" : "false")
                      << " error=" << bridge.amcl_scan_admission_last_error;
      } else if (!sequence_observed) {
        mark_sequence_observed();
      }

      if (sample_ok) {
        ++stable_samples;
      } else {
        stable_samples = 0;
      }
      last_detail = sample_detail.str();
      result.stable_samples = stable_samples;
      update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
        state.detail = last_detail;
      });

      if (stable_samples >= post_relocalization_stable_tf_samples_ &&
        std::chrono::steady_clock::now() >= min_deadline)
      {
        result.ok = true;
        result.failure_code = "NONE";
        result.detail = last_detail + "; post relocalization settle passed";
        result.elapsed_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
          state.in_progress = false;
          state.complete = true;
          state.failure_reason = "none";
          state.detail = result.detail;
        });
        return result;
      }

      std::this_thread::sleep_for(sample_period);
    }

    result.failure_code = last_failure_code;
    result.detail = last_detail + "; timed out waiting for post relocalization settle";
    result.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    update_post_relocalization_settle_state([&](PostRelocalizationSettleState & state) {
      state.in_progress = false;
      state.complete = false;
      state.failure_reason = result.failure_code;
      state.detail = result.detail;
    });
    return result;
  }

  RobotPoseSnapshot wait_for_current_robot_pose(
    const bool require_map_frame,
    std::string & error)
  {
    set_tf_subscription_active(true);

    RobotPoseSnapshot snapshot;
    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = current_robot_pose_snapshot();
      if (snapshot.available && snapshot.age_sec <= robot_pose_freshness_sec_) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }

    if (!snapshot.available) {
      error = "no fresh map-frame robot pose";
      return snapshot;
    }
    if (snapshot.age_sec > robot_pose_freshness_sec_) {
      error = "no fresh map-frame robot pose";
      snapshot.available = false;
      return snapshot;
    }
    (void)require_map_frame;
    return snapshot;
  }

  double dock_contact_latch_age_sec(const DockContactLatchSnapshot & snapshot) const
  {
    const auto stamp = !snapshot.latched_at.empty() ? snapshot.latched_at : snapshot.updated_at;
    const auto stamp_seconds = parse_utc_iso8601_seconds(stamp);
    if (!stamp_seconds) {
      return -1.0;
    }
    return std::max(0.0, wall_time_seconds() - *stamp_seconds);
  }

  void refresh_dock_contact_latch_derived_fields(DockContactLatchSnapshot & snapshot) const
  {
    snapshot.source_bms = latch_source_is_bms(snapshot.source);
    snapshot.age_sec = dock_contact_latch_age_sec(snapshot);
    snapshot.stale =
      snapshot.valid &&
      snapshot.latched_docked &&
      snapshot.source_bms &&
      snapshot.age_sec >= 0.0 &&
      dock_contact_latch_bms_ttl_sec_ > 0.0 &&
      snapshot.age_sec > dock_contact_latch_bms_ttl_sec_;
  }

  DockContactLatchSnapshot read_dock_contact_latch() const
  {
    DockContactLatchSnapshot snapshot;
    if (docking_contact_latch_file_.empty()) {
      snapshot.reason = "latch_file_not_configured";
      return snapshot;
    }
    try {
      const auto text = read_optional_text_file(fs::path(docking_contact_latch_file_));
      if (text.empty()) {
        snapshot.reason = "latch_file_missing";
        return snapshot;
      }
      snapshot.valid = true;
      snapshot.latched_docked = json_bool_value(text, "latched_docked", json_bool_value(text, "docked", false));
      snapshot.docked = snapshot.latched_docked;
      snapshot.source = json_string_value(text, "source").value_or("unknown");
      snapshot.reason = json_string_value(text, "reason").value_or("no_reason");
      snapshot.building_id = json_string_value(text, "building_id").value_or("");
      snapshot.floor_id = json_string_value(text, "floor_id").value_or("");
      snapshot.map_id = json_string_value(text, "map_id").value_or("");
      snapshot.dock_id = json_string_value(text, "dock_id").value_or("");
      snapshot.latched_at = json_string_value(text, "latched_at").value_or("");
      snapshot.last_confirmed_at = json_string_value(text, "last_confirmed_at").value_or("");
      snapshot.cleared_at = json_string_value(text, "cleared_at").value_or("");
      snapshot.clear_reason = json_string_value(text, "clear_reason").value_or("");
      snapshot.note = json_string_value(text, "note").value_or("");
      snapshot.updated_at = json_string_value(text, "updated_at").value_or("");
      refresh_dock_contact_latch_derived_fields(snapshot);
      return snapshot;
    } catch (const std::exception & exc) {
      snapshot.reason = std::string("latch_read_failed:") + exc.what();
      return snapshot;
    }
  }

  void update_dock_contact_latch(
    const bool docked,
    const std::string & source,
    const std::string & reason,
    const std::string & dock_id,
    const std::string & building_id = "",
    const std::string & floor_id = "",
    const std::string & map_id = "",
    const std::string & note = "")
  {
    if (docking_contact_latch_file_.empty()) {
      return;
    }
    if (have_last_dock_contact_latch_write_ &&
      docked == last_dock_contact_latch_docked_ &&
      source == last_dock_contact_latch_source_ &&
      reason == last_dock_contact_latch_reason_ &&
      dock_id == last_dock_contact_latch_dock_id_ &&
      note == last_dock_contact_latch_note_)
    {
      return;
    }
    const auto previous = read_dock_contact_latch();
    const auto now_text = utc_timestamp_iso8601();
    const auto latched_at = docked ? (previous.latched_docked && !previous.latched_at.empty() ?
      previous.latched_at : now_text) : previous.latched_at;
    std::ostringstream body;
    body << "{\n"
         << "  \"schema\": \"njrh.docking_contact_latch.v1\",\n"
         << "  \"latched_docked\": " << (docked ? "true" : "false") << ",\n"
         << "  \"docked\": " << (docked ? "true" : "false") << ",\n"
         << "  \"source\": " << json_string(source) << ",\n"
         << "  \"reason\": " << json_string(reason) << ",\n"
         << "  \"building_id\": " << json_string(building_id.empty() ? previous.building_id : building_id) << ",\n"
         << "  \"floor_id\": " << json_string(floor_id.empty() ? previous.floor_id : floor_id) << ",\n"
         << "  \"map_id\": " << json_string(map_id.empty() ? previous.map_id : map_id) << ",\n"
         << "  \"dock_id\": " << json_string(dock_id) << ",\n"
         << "  \"latched_at\": " << json_string(latched_at) << ",\n"
         << "  \"last_confirmed_at\": " << json_string(docked ? now_text : previous.last_confirmed_at) << ",\n"
         << "  \"cleared_at\": " << json_string(docked ? "" : now_text) << ",\n"
         << "  \"clear_reason\": " << json_string(docked ? "" : reason) << ",\n"
         << "  \"note\": " << json_string(note.empty() ? previous.note : note) << ",\n"
         << "  \"updated_at\": " << json_string(now_text) << "\n"
         << "}\n";
    try {
      const auto path = fs::path(docking_contact_latch_file_);
      fs::create_directories(path.parent_path());
      const auto tmp = path.string() + ".tmp";
      write_text_file(fs::path(tmp), body.str());
      fs::rename(fs::path(tmp), path);
      have_last_dock_contact_latch_write_ = true;
      last_dock_contact_latch_docked_ = docked;
      last_dock_contact_latch_source_ = source;
      last_dock_contact_latch_reason_ = reason;
      last_dock_contact_latch_dock_id_ = dock_id;
      last_dock_contact_latch_note_ = note;
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(),
        "failed to write docking contact latch %s: %s",
        docking_contact_latch_file_.c_str(),
        exc.what());
    }
  }

  bool bms_latch_write_allowed_by_runtime()
  {
    const auto runtime = runtime_mode_snapshot();
    const auto docking_state = lower_copy(runtime.docking_state);
    const auto docking_status = lower_copy(runtime.docking_status);
    const bool docking_context =
      runtime.docking_active ||
      docking_state == "docked" ||
      docking_state == "charging" ||
      docking_state == "docking" ||
      docking_state == "undocking" ||
      starts_with(docking_status, "docked") ||
      starts_with(docking_status, "charging") ||
      docking_status_is_undocking(docking_status);
    if (docking_context) {
      return true;
    }
    return !(runtime.navigation_active && navigation_goal_job_running());
  }

  void maybe_update_bms_dock_contact_latch(
    const BatteryContactEvaluation & charging_contact,
    const bool contact_stable,
    const double stable_duration_sec)
  {
    if (!charging_contact.contact || !contact_stable) {
      return;
    }
    if (!bms_latch_write_allowed_by_runtime()) {
      return;
    }
    (void)stable_duration_sec;
    update_dock_contact_latch(
      true,
      "bms",
      "bms_charging_contact:" + charging_contact.reason,
      "",
      "",
      "",
      "",
      "stable_bms_contact");
  }

  void handle_bms_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    const auto charging_contact = battery_charging_contact(*msg);
    bool contact_stable = false;
    double contact_stable_duration_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (std::isfinite(msg->percentage)) {
        latest_bms_soc_ = normalized_soc_percent(msg->percentage);
        have_bms_soc_ = true;
      } else {
        have_bms_soc_ = false;
      }
      latest_bms_voltage_ = static_cast<double>(msg->voltage);
      latest_bms_current_ = static_cast<double>(msg->current);
      latest_bms_temperature_ = static_cast<double>(msg->temperature);
      latest_bms_power_supply_status_ = static_cast<int>(msg->power_supply_status);
      latest_bms_power_supply_health_ = static_cast<int>(msg->power_supply_health);
      latest_bms_power_supply_technology_ = static_cast<int>(msg->power_supply_technology);
      latest_bms_present_ = msg->present;
      latest_bms_charging_contact_ = charging_contact.contact;
      latest_bms_charging_contact_reason_ = charging_contact.reason;
      if (charging_contact.contact) {
        if (!have_latest_bms_contact_started_at_) {
          latest_bms_contact_started_at_ = now;
          have_latest_bms_contact_started_at_ = true;
        }
        have_latest_bms_no_contact_started_at_ = false;
        latest_bms_contact_stable_duration_sec_ =
          std::chrono::duration<double>(now - latest_bms_contact_started_at_).count();
        latest_bms_no_contact_duration_sec_ = 0.0;
      } else {
        if (!have_latest_bms_no_contact_started_at_) {
          latest_bms_no_contact_started_at_ = now;
          have_latest_bms_no_contact_started_at_ = true;
        }
        have_latest_bms_contact_started_at_ = false;
        latest_bms_no_contact_duration_sec_ =
          std::chrono::duration<double>(now - latest_bms_no_contact_started_at_).count();
        latest_bms_contact_stable_duration_sec_ = 0.0;
      }
      contact_stable_duration_sec = latest_bms_contact_stable_duration_sec_;
      contact_stable = charging_contact.contact &&
        contact_stable_duration_sec >= dock_contact_latch_bms_require_contact_sec_;
      latest_bms_received_at_ = now;
      have_bms_state_ = true;
    }
    maybe_update_bms_dock_contact_latch(charging_contact, contact_stable, contact_stable_duration_sec);
    if (charging_contact.contact && teleop_stop_on_charging_ && teleop_session_active()) {
      clear_teleop_command();
    }
  }

  void handle_localization_result(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_localization_result_seq_++;
    latest_localization_result_frame_ = normalized_frame_id(msg->header.frame_id);
    latest_localization_result_x_ = msg->pose.pose.position.x;
    latest_localization_result_y_ = msg->pose.pose.position.y;
    latest_localization_result_yaw_ = quaternion_yaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    latest_localization_result_stamp_sec_ = stamp_to_seconds(msg->header.stamp);
    latest_localization_result_received_at_ = std::chrono::steady_clock::now();
    have_localization_result_ = true;
  }

  LocalizationResultSnapshot localization_result_snapshot()
  {
    LocalizationResultSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_localization_result_) {
      return snapshot;
    }
    snapshot.available = true;
    snapshot.seq = latest_localization_result_seq_;
    snapshot.frame_id = latest_localization_result_frame_;
    snapshot.x = latest_localization_result_x_;
    snapshot.y = latest_localization_result_y_;
    snapshot.yaw = latest_localization_result_yaw_;
    snapshot.stamp_sec = latest_localization_result_stamp_sec_;
    snapshot.received_at = latest_localization_result_received_at_;
    snapshot.age_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_localization_result_received_at_).count();
    return snapshot;
  }

  static std::uint64_t json_uint64_value(
    const std::string & body,
    const std::string & key,
    const std::uint64_t fallback = 0U)
  {
    const auto value = json_number_value(body, key);
    if (!value || !std::isfinite(*value) || *value < 0.0) {
      return fallback;
    }
    return static_cast<std::uint64_t>(*value);
  }

  void handle_localization_bridge_status(const std_msgs::msg::String::SharedPtr msg)
  {
    BridgeStatusSnapshot snapshot;
    snapshot.available = true;
    snapshot.raw = msg->data;
    snapshot.received_at = std::chrono::steady_clock::now();
    snapshot.has_map_to_odom = json_bool_value(msg->data, "has_map_to_odom", false);
    snapshot.map_to_odom_publisher_owner =
      json_string_value(msg->data, "map_to_odom_publisher_owner").value_or("unknown");
    snapshot.map_to_odom_age_ms = json_number_value(msg->data, "map_to_odom_age_ms").value_or(-1.0);
    snapshot.last_explicit_relocalization_sequence =
      json_uint64_value(msg->data, "last_explicit_relocalization_sequence");
    snapshot.last_explicit_relocalization_accept_time =
      json_number_value(msg->data, "last_explicit_relocalization_accept_time").value_or(0.0);
    snapshot.last_explicit_relocalization_source =
      json_string_value(msg->data, "last_explicit_relocalization_source").value_or("none");
    snapshot.last_accepted_correction_translation_m =
      json_number_value(msg->data, "last_accepted_correction_translation_m").value_or(0.0);
    snapshot.last_accepted_correction_yaw_rad =
      json_number_value(msg->data, "last_accepted_correction_yaw_rad").value_or(0.0);
    snapshot.amcl_input_enabled = json_bool_value(msg->data, "amcl_input_enabled", false);
    snapshot.amcl_ready = json_bool_value(msg->data, "amcl_ready", false);
    snapshot.amcl_degraded = json_bool_value(msg->data, "amcl_degraded", false);
    snapshot.amcl_degraded_reason =
      json_string_value(msg->data, "amcl_degraded_reason").value_or("");
    snapshot.amcl_status_source =
      json_string_value(msg->data, "amcl_status_source").value_or("");
    snapshot.amcl_status_file_stale =
      json_bool_value(msg->data, "amcl_status_file_stale", true);
    snapshot.amcl_status_age_ms =
      json_number_value(msg->data, "amcl_status_age_ms").value_or(-1.0);
    snapshot.amcl_process_ready =
      json_bool_value(msg->data, "amcl_process_ready", false);
    snapshot.amcl_seeded = json_bool_value(msg->data, "amcl_seeded", false);
    snapshot.amcl_seed_response_ok =
      json_bool_value(msg->data, "amcl_seed_response_ok", false);
    snapshot.amcl_nomotion_pose_received =
      json_bool_value(msg->data, "amcl_nomotion_pose_received", false);
    snapshot.amcl_static_standby =
      json_bool_value(msg->data, "amcl_static_standby", false);
    snapshot.amcl_tracking_ready =
      json_bool_value(msg->data, "amcl_tracking_ready", false);
    snapshot.amcl_correction_ready =
      json_bool_value(msg->data, "amcl_correction_ready", false);
    snapshot.amcl_not_moving_no_update_ok =
      json_bool_value(msg->data, "amcl_not_moving_no_update_ok", false);
    snapshot.amcl_scan_admission_enabled =
      json_bool_value(msg->data, "amcl_scan_admission_enabled", false);
    snapshot.amcl_scan_admission_alive =
      json_bool_value(msg->data, "amcl_scan_admission_alive", false);
    snapshot.amcl_message_filter_drop_detected =
      json_bool_value(msg->data, "amcl_message_filter_drop_detected", false);
    snapshot.amcl_scan_admission_last_error =
      json_string_value(msg->data, "amcl_scan_admission_last_error").value_or("none");
    snapshot.localization_degraded = json_bool_value(msg->data, "localization_degraded", false);

    std::lock_guard<std::mutex> lock(bridge_status_mutex_);
    latest_bridge_status_ = snapshot;
  }

  BridgeStatusSnapshot bridge_status_snapshot() const
  {
    std::lock_guard<std::mutex> lock(bridge_status_mutex_);
    BridgeStatusSnapshot snapshot = latest_bridge_status_;
    if (snapshot.available) {
      snapshot.age_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - snapshot.received_at).count();
    }
    return snapshot;
  }

  void handle_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    (void)msg;
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    ++local_costmap_update_count_;
    latest_local_costmap_received_at_ = std::chrono::steady_clock::now();
  }

  std::uint64_t local_costmap_update_count() const
  {
    std::lock_guard<std::mutex> lock(local_costmap_mutex_);
    return local_costmap_update_count_;
  }

  void handle_tf_static_message(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & transform : msg->transforms) {
      const auto parent = normalized_frame_id(transform.header.frame_id);
      const auto child = normalized_frame_id(transform.child_frame_id);
      if (parent == tf_base_frame_ && child == post_relocalization_static_lidar_frame_) {
        have_base_to_lidar_static_tf_ = true;
        base_to_lidar_static_tf_received_at_ = std::chrono::steady_clock::now();
      }
    }
  }

  bool base_to_lidar_static_tf_ready()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return have_base_to_lidar_static_tf_;
  }

  void handle_rosout_message(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    const std::string combined = msg->name + ": " + msg->msg;
    if (combined.find("Message Filter dropping") == std::string::npos) {
      return;
    }
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    ++message_filter_drop_count_;
    last_message_filter_drop_text_ = combined;
    if (combined.find("local_costmap") != std::string::npos ||
      combined.find("controller_server") != std::string::npos)
    {
      ++local_costmap_message_filter_drop_count_;
      last_local_costmap_message_filter_drop_text_ = combined;
    }
  }

  std::uint64_t local_costmap_message_filter_drop_count() const
  {
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    return local_costmap_message_filter_drop_count_;
  }

  std::string last_local_costmap_message_filter_drop_text() const
  {
    std::lock_guard<std::mutex> lock(rosout_mutex_);
    return last_local_costmap_message_filter_drop_text_;
  }

  bool wait_for_localization_result_after(
    const std::chrono::steady_clock::time_point & min_received_at,
    const double timeout_sec,
    std::string & detail,
    LocalizationResultSnapshot * accepted_snapshot = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_sec));
    LocalizationResultSnapshot snapshot;
    while (std::chrono::steady_clock::now() <= deadline) {
      snapshot = localization_result_snapshot();
      if (snapshot.available && snapshot.received_at >= min_received_at && snapshot.frame_id == tf_map_frame_) {
        detail = localization_result_success_detail(snapshot);
        if (accepted_snapshot != nullptr) {
          *accepted_snapshot = snapshot;
        }
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }

    detail = localization_result_wait_failure_detail(localization_result_topic_, timeout_sec, snapshot);
    return false;
  }

  bool wait_for_localization_bridge_acceptance(
    const LocalizationResultSnapshot & localization,
    std::string & detail)
  {
    if (localization_bridge_acceptance_timeout_sec_ <= 0.0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(localization_bridge_acceptance_timeout_sec_));
    std::string last_detail = "no fresh map-frame robot pose";
    while (std::chrono::steady_clock::now() <= deadline) {
      const auto pose = current_robot_pose_snapshot();
      if (pose.available && pose.frame_id == tf_map_frame_) {
        const double dx = pose.x - localization.x;
        const double dy = pose.y - localization.y;
        const double distance = std::hypot(dx, dy);
        const double yaw_error = std::fabs(normalize_angle(pose.yaw - localization.yaw));
        std::ostringstream out;
        out << std::fixed << std::setprecision(3)
            << "bridge acceptance distance=" << distance
            << " yaw_error=" << yaw_error
            << " pose_age=" << pose.age_sec;
        last_detail = out.str();
        if (distance <= localization_bridge_acceptance_max_distance_m_ &&
          yaw_error <= localization_bridge_acceptance_max_yaw_rad_)
        {
          detail += "; " + last_detail;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    detail += "; localization_result not accepted by map->odom bridge: " + last_detail;
    return false;
  }

  bool request_localization_bridge_force_accept(
    const std::string & reason,
    std::string & detail,
    const std::chrono::nanoseconds & timeout)
  {
    if (!localization_bridge_force_accept_client_->wait_for_service(timeout)) {
      detail = "service unavailable: " + localization_bridge_force_accept_service_;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = localization_bridge_force_accept_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = "timed out waiting for bridge force-accept service: " +
        localization_bridge_force_accept_service_;
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      detail = "bridge force-accept rejected for " + reason + ": " + response->message;
      return false;
    }
    detail = "bridge force-accept armed for " + reason + ": " + response->message;
    return true;
  }

  bool trigger_localization_and_wait_for_result(
    const std::string & reason,
    std::string & detail,
    const double wait_timeout_sec = -1.0,
    std::uint64_t * accepted_sequence = nullptr)
  {
    const auto trigger_started_at = std::chrono::steady_clock::now();
    if (accepted_sequence != nullptr) {
      *accepted_sequence = 0U;
    }
    const auto bridge_before = bridge_status_snapshot();
    const std::uint64_t previous_explicit_sequence =
      bridge_before.available ? bridge_before.last_explicit_relocalization_sequence : 0U;
    const double timeout_sec = wait_timeout_sec > 0.0 ? wait_timeout_sec : docking_relocalize_wait_sec_;
    const auto trigger_timeout = localization_trigger_service_timeout(timeout_sec);
    if (!localization_trigger_client_->wait_for_service(trigger_timeout)) {
      detail = "service unavailable: " + localization_trigger_service_;
      return false;
    }

    std::string force_accept_detail;
    if (!request_localization_bridge_force_accept(reason, force_accept_detail, trigger_timeout)) {
      detail = force_accept_detail;
      return false;
    }

    auto request = std::make_shared<robot_interfaces::srv::TriggerLocalization::Request>();
    request->reason = reason;
    auto future = localization_trigger_client_->async_send_request(request);
    if (future.wait_for(trigger_timeout) != std::future_status::ready) {
      detail = "timed out waiting for localization trigger";
      return false;
    }

    const auto response = future.get();
    if (!response->accepted) {
      detail = response->message;
      return false;
    }

    std::string result_detail;
    LocalizationResultSnapshot accepted_snapshot;
    if (!wait_for_localization_result_after(trigger_started_at, timeout_sec, result_detail, &accepted_snapshot)) {
      accepted_snapshot = localization_result_snapshot();
      if (accepted_snapshot.available && accepted_snapshot.frame_id == tf_map_frame_ &&
        accepted_snapshot.age_sec <= docking_relocalize_recent_result_max_age_sec_)
      {
      detail = force_accept_detail + "; " + localization_result_recent_fallback_detail(
          response->message,
          localization_result_topic_,
          accepted_snapshot);
        const bool accepted = wait_for_localization_bridge_acceptance(accepted_snapshot, detail);
        if (accepted && accepted_sequence != nullptr) {
          const auto bridge_after = bridge_status_snapshot();
          *accepted_sequence =
            bridge_after.available &&
            bridge_after.last_explicit_relocalization_sequence > previous_explicit_sequence ?
            bridge_after.last_explicit_relocalization_sequence : previous_explicit_sequence + 1U;
        }
        return accepted;
      }
      detail = force_accept_detail + "; " + response->message + "; " + result_detail;
      return false;
    }
    detail = force_accept_detail + "; " + response->message + "; " + result_detail;
    const bool accepted = wait_for_localization_bridge_acceptance(accepted_snapshot, detail);
    if (accepted && accepted_sequence != nullptr) {
      const auto bridge_after = bridge_status_snapshot();
      *accepted_sequence =
        bridge_after.available &&
        bridge_after.last_explicit_relocalization_sequence > previous_explicit_sequence ?
        bridge_after.last_explicit_relocalization_sequence : previous_explicit_sequence + 1U;
    }
    return accepted;
  }

  NavigationRelocalizationDecision navigation_goal_relocalization_decision(const bool force_requested)
  {
    NavigationRelocalizationDecision decision;
    decision.required = navigation_relocalize_before_goal_required_;

    if (force_requested) {
      decision.requested = true;
      decision.detail = "force_relocalize requested by navigation goal";
      return decision;
    }

    if (!navigation_relocalize_before_goal_) {
      decision.detail = "pre-navigation relocalization disabled";
      return decision;
    }

    if (navigation_relocalize_before_goal_always_) {
      decision.requested = true;
      decision.detail = "navigation_relocalize_before_goal_always=true";
      return decision;
    }

    const auto context = read_runtime_map_context();
    if (!context) {
      decision.requested = true;
      decision.detail = "no runtime map context; refreshing localization before Nav2 goal";
      return decision;
    }

    if (!context->confirmed || context->state != "ready") {
      decision.requested = true;
      decision.detail = "runtime map context not ready: " + context->building_id + "/" +
        context->floor_id + "/" + context->map_id + " state=" + context->state;
      return decision;
    }

    const auto pose = current_robot_pose_snapshot();
    if (!pose.available || pose.frame_id != tf_map_frame_) {
      decision.requested = true;
      decision.detail = "no current map-frame pose; refreshing localization before Nav2 goal";
      return decision;
    }

    if (pose.age_sec > robot_pose_freshness_sec_) {
      decision.requested = true;
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "map-frame pose stale age=" << pose.age_sec
          << "s > " << robot_pose_freshness_sec_
          << "s; refreshing localization before Nav2 goal";
      decision.detail = out.str();
      return decision;
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "confirmed runtime map context and fresh map-frame pose age="
        << pose.age_sec
        << "s; skipping pre-navigation relocalization";
    decision.detail = out.str();
    return decision;
  }

  std::vector<std::string> navigation_lifecycle_node_names() const
  {
    return {
      "/controller_server",
      "/planner_server",
      "/bt_navigator",
      "/behavior_server",
      "/local_costmap/local_costmap",
      "/global_costmap/global_costmap",
      "/velocity_smoother",
      "/collision_monitor"};
  }

  std::chrono::nanoseconds navigation_lifecycle_check_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(navigation_lifecycle_check_timeout_sec_));
  }

  NavigationLifecycleSnapshot navigation_lifecycle_snapshot()
  {
    NavigationLifecycleSnapshot snapshot;
    std::vector<std::string> inactive;
    const auto timeout = navigation_lifecycle_check_timeout();
    for (const auto & node_name : navigation_lifecycle_node_names()) {
      auto client_it = navigation_lifecycle_clients_.find(node_name);
      if (client_it == navigation_lifecycle_clients_.end() || !client_it->second) {
        inactive.push_back(node_name + ":client_missing");
        continue;
      }
      auto & client = client_it->second;
      if (!client->wait_for_service(timeout)) {
        inactive.push_back(node_name + ":service_unavailable");
        continue;
      }
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = client->async_send_request(request);
      if (future.wait_for(timeout) != std::future_status::ready) {
        inactive.push_back(node_name + ":state_timeout");
        continue;
      }
      const auto response = future.get();
      if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        inactive.push_back(node_name + ":" + response->current_state.label);
      }
    }

    if (inactive.empty()) {
      snapshot.active = true;
      snapshot.detail = "navigation lifecycle active";
    } else {
      snapshot.active = false;
      std::ostringstream detail;
      detail << "navigation lifecycle inactive";
      for (const auto & item : inactive) {
        detail << "; " << item;
      }
      snapshot.detail = detail.str();
    }
    return snapshot;
  }

  bool validate_current_pose_near_docking_approach(
    const DockingJob & job,
    std::string & detail)
  {
    if (!docking_validate_predock_pose_after_relocalization_) {
      detail = "post-predock pose check disabled";
      return true;
    }

    std::string pose_error;
    const auto pose = wait_for_current_robot_pose(true, pose_error);
    if (!pose.available) {
      detail = "no fresh map-frame pose for post-predock approach check: " + pose_error;
      return false;
    }

    const double dx = pose.x - job.approach_x;
    const double dy = pose.y - job.approach_y;
    const double distance = std::hypot(dx, dy);
    const double yaw_error = std::fabs(normalize_angle(pose.yaw - job.approach_yaw));

    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "post-predock pose check distance=" << distance
        << " yaw_error=" << yaw_error
        << " max_distance=" << docking_predock_pose_max_distance_m_
        << " max_yaw=" << docking_predock_pose_max_yaw_rad_
        << " pose=(" << pose.x << "," << pose.y << "," << pose.yaw << ")"
        << " approach=(" << job.approach_x << "," << job.approach_y << "," << job.approach_yaw << ")";
    detail = out.str();
    return distance <= docking_predock_pose_max_distance_m_ &&
      yaw_error <= docking_predock_pose_max_yaw_rad_;
  }

  bool is_docking_predock_pose_type(const StoredPose & pose) const
  {
    const auto type = lower_copy(pose.type);
    return type == "dock_predock" || type == "predock" || type == "dock_approach";
  }

  std::vector<std::string> docking_predock_pose_id_candidates(const std::string & dock_id) const
  {
    return {
      dock_id + "_predock",
      dock_id + "_pre_dock",
      dock_id + "_approach",
      "predock_" + dock_id,
      "pre_dock_" + dock_id,
      "approach_" + dock_id};
  }

  std::optional<StoredPose> resolve_docking_predock_pose(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & dock_id,
    const StoredPose & dock_pose,
    const std::string & requested_predock_pose_id,
    std::string & source,
    std::string & error,
    int & error_status) const
  {
    source.clear();
    error.clear();
    error_status = 0;

    if (!requested_predock_pose_id.empty()) {
      if (!safe_pose_id(requested_predock_pose_id)) {
        error_status = 400;
        error = "valid predock_pose_id is required";
        return std::nullopt;
      }
      auto pose = find_floor_catalog_pose(*map_catalog_, building_id, floor_id, requested_predock_pose_id);
      if (!pose) {
        error_status = 404;
        error = "predock_pose_id not found in poses.yaml: " + requested_predock_pose_id;
        return std::nullopt;
      }
      source = "manual_predock_explicit";
      return pose;
    }

    for (const auto & candidate_id : docking_predock_pose_id_candidates(dock_id)) {
      if (auto pose = find_floor_catalog_pose(*map_catalog_, building_id, floor_id, candidate_id)) {
        source = "manual_predock_auto_id";
        return pose;
      }
    }

    const auto path = poses_yaml_path(*map_catalog_, building_id, floor_id);
    if (!fs::exists(path)) {
      return std::nullopt;
    }

    const auto poses = read_floor_poses(path);
    std::vector<StoredPose> typed_predocks;
    std::vector<StoredPose> named_predocks;
    const auto dock_name_predock = lower_copy(dock_pose.name + "_predock");
    const auto dock_name_pre_dock = lower_copy(dock_pose.name + "_pre_dock");
    const auto dock_name_approach = lower_copy(dock_pose.name + "_approach");

    for (const auto & pose : poses) {
      if (pose.id == dock_id || !is_docking_predock_pose_type(pose)) {
        continue;
      }
      typed_predocks.push_back(pose);
      const auto pose_id = lower_copy(pose.id);
      const auto pose_name = lower_copy(pose.name);
      bool named_match = false;
      for (const auto & candidate_id : docking_predock_pose_id_candidates(dock_id)) {
        const auto candidate = lower_copy(candidate_id);
        if (pose_id == candidate || pose_name == candidate) {
          named_match = true;
          break;
        }
      }
      if (pose_name == dock_name_predock || pose_name == dock_name_pre_dock || pose_name == dock_name_approach) {
        named_match = true;
      }
      if (named_match) {
        named_predocks.push_back(pose);
      }
    }

    if (named_predocks.size() == 1U) {
      source = "manual_predock_auto_name";
      return named_predocks.front();
    }
    if (named_predocks.size() > 1U) {
      error_status = 409;
      error = "multiple matching dock predock poses found; pass predock_pose_id explicitly";
      return std::nullopt;
    }
    if (typed_predocks.size() == 1U) {
      source = "manual_predock_auto_unique_type";
      return typed_predocks.front();
    }
    if (typed_predocks.size() > 1U) {
      error_status = 409;
      error =
        "multiple dock_predock poses found; pass predock_pose_id or name one " + dock_id + "_predock";
      return std::nullopt;
    }
    return std::nullopt;
  }

  bool validate_manual_docking_predock_pose(
    const StoredPose & dock_pose,
    const StoredPose & predock_pose,
    std::string & error) const
  {
    const double distance = std::hypot(predock_pose.x - dock_pose.x, predock_pose.y - dock_pose.y);
    const double yaw_error = std::fabs(normalize_angle(predock_pose.yaw - dock_pose.yaw));
    const bool distance_ok = !docking_manual_predock_distance_check_enable_ ||
      (distance >= docking_manual_predock_min_distance_m_ &&
      distance <= docking_manual_predock_max_distance_m_);
    const bool yaw_ok = yaw_error <= docking_manual_predock_max_yaw_error_rad_;
    if (distance_ok && yaw_ok) {
      return true;
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "manual predock pose " << predock_pose.id << " failed docking sanity check: distance="
        << distance;
    if (docking_manual_predock_distance_check_enable_) {
      out << " allowed=[" << docking_manual_predock_min_distance_m_
          << "," << docking_manual_predock_max_distance_m_ << "]";
    } else {
      out << " distance_check=disabled";
    }
    out << " yaw_error=" << yaw_error
        << " max_yaw_error=" << docking_manual_predock_max_yaw_error_rad_
        << "; resave the predock point with a heading aligned to the charger";
    error = out.str();
    return false;
  }

  void handle_tf_message(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = std::chrono::steady_clock::now();
    bool saw_direct_map_to_base = false;
    for (const auto & transform : msg->transforms) {
      const auto parent = normalized_frame_id(transform.header.frame_id);
      const auto child = normalized_frame_id(transform.child_frame_id);
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      const double yaw = quaternion_yaw(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      const double stamp_sec = stamp_to_seconds(transform.header.stamp);

      if (parent == tf_map_frame_ && child == tf_base_frame_) {
        latest_pose_frame_ = tf_map_frame_;
        latest_pose_x_ = x;
        latest_pose_y_ = y;
        latest_pose_yaw_ = yaw;
        latest_pose_stamp_sec_ = stamp_sec;
        latest_pose_received_at_ = now;
        have_pose_ = true;
        saw_direct_map_to_base = true;
      } else if (parent == tf_map_frame_ && child == tf_odom_frame_) {
        latest_map_to_odom_x_ = x;
        latest_map_to_odom_y_ = y;
        latest_map_to_odom_yaw_ = yaw;
        latest_map_to_odom_stamp_sec_ = stamp_sec;
        latest_map_to_odom_received_at_ = now;
        have_map_to_odom_ = true;
      } else if (parent == tf_odom_frame_ && child == tf_base_frame_) {
        latest_odom_to_base_x_ = x;
        latest_odom_to_base_y_ = y;
        latest_odom_to_base_yaw_ = yaw;
        latest_odom_to_base_stamp_sec_ = stamp_sec;
        latest_odom_to_base_received_at_ = now;
        have_odom_to_base_ = true;
      }
    }

    if (!saw_direct_map_to_base && have_map_to_odom_ && have_odom_to_base_) {
      const double c = std::cos(latest_map_to_odom_yaw_);
      const double s = std::sin(latest_map_to_odom_yaw_);
      latest_pose_frame_ = tf_map_frame_;
      latest_pose_x_ = latest_map_to_odom_x_ + c * latest_odom_to_base_x_ - s * latest_odom_to_base_y_;
      latest_pose_y_ = latest_map_to_odom_y_ + s * latest_odom_to_base_x_ + c * latest_odom_to_base_y_;
      latest_pose_yaw_ = normalize_angle(latest_map_to_odom_yaw_ + latest_odom_to_base_yaw_);
      latest_pose_stamp_sec_ =
        older_nonzero_stamp(latest_map_to_odom_stamp_sec_, latest_odom_to_base_stamp_sec_);
      latest_pose_received_at_ =
        latest_map_to_odom_received_at_ < latest_odom_to_base_received_at_ ?
        latest_map_to_odom_received_at_ : latest_odom_to_base_received_at_;
      have_pose_ = true;
    } else if (have_odom_to_base_ && !have_pose_) {
      latest_pose_frame_ = tf_odom_frame_;
      latest_pose_x_ = latest_odom_to_base_x_;
      latest_pose_y_ = latest_odom_to_base_y_;
      latest_pose_yaw_ = latest_odom_to_base_yaw_;
      latest_pose_stamp_sec_ = latest_odom_to_base_stamp_sec_;
      latest_pose_received_at_ = latest_odom_to_base_received_at_;
      have_pose_ = true;
    }
  }

  void start_server()
  {
    running_.store(true);
    start_http_workers();
    server_thread_ = std::thread([this]() { serve(); });
  }

  void stop_server()
  {
    running_.store(false);
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    stop_http_workers();
  }

  void start_http_workers()
  {
    http_workers_.clear();
    http_workers_.reserve(static_cast<std::size_t>(max_http_connections_));
    for (int index = 0; index < max_http_connections_; ++index) {
      http_workers_.emplace_back([this]() { http_worker_loop(); });
    }
  }

  void stop_http_workers()
  {
    {
      std::lock_guard<std::mutex> lock(http_queue_mutex_);
      while (!http_client_queue_.empty()) {
        ::close(http_client_queue_.front());
        http_client_queue_.pop_front();
        active_http_connections_.fetch_sub(1, std::memory_order_acq_rel);
      }
    }
    http_queue_cv_.notify_all();
    for (auto & worker : http_workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    http_workers_.clear();
  }

  void enqueue_http_client(const int client_fd)
  {
    {
      std::lock_guard<std::mutex> lock(http_queue_mutex_);
      if (!running_.load()) {
        ::close(client_fd);
        active_http_connections_.fetch_sub(1, std::memory_order_acq_rel);
        return;
      }
      http_client_queue_.push_back(client_fd);
    }
    http_queue_cv_.notify_one();
  }

  void http_worker_loop()
  {
    while (true) {
      int client_fd = -1;
      {
        std::unique_lock<std::mutex> lock(http_queue_mutex_);
        http_queue_cv_.wait(lock, [this]() {
          return !running_.load() || !http_client_queue_.empty();
        });
        if (http_client_queue_.empty()) {
          if (!running_.load()) {
            return;
          }
          continue;
        }
        client_fd = http_client_queue_.front();
        http_client_queue_.pop_front();
      }

      struct ActiveConnectionGuard
      {
        std::atomic<int> & counter;
        ~ActiveConnectionGuard()
        {
          counter.fetch_sub(1, std::memory_order_acq_rel);
        }
      } guard{active_http_connections_};

      if (!running_.load()) {
        ::close(client_fd);
        continue;
      }
      handle_client(client_fd);
    }
  }

  void serve()
  {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create API socket: %s", std::strerror(errno));
      return;
    }
    set_close_on_exec(server_fd_);

    int reuse = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (host_ == "0.0.0.0" || host_.empty()) {
      address.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "invalid API host: %s", host_.c_str());
      return;
    }

    if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to bind API server on %s:%d: %s", host_.c_str(), port_, std::strerror(errno));
      return;
    }
    if (::listen(server_fd_, 64) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to listen on API socket: %s", std::strerror(errno));
      return;
    }

    RCLCPP_INFO(get_logger(), "robot_api_server listening on %s:%d", host_.c_str(), port_);
    while (running_.load()) {
      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&client_address), &client_length);
      if (client_fd < 0) {
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "API accept failed: %s", std::strerror(errno));
        }
        continue;
      }
      set_close_on_exec(client_fd);
      timeval timeout{};
      timeout.tv_sec = 15;
      timeout.tv_usec = 0;
      ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
      if (active_http_connections_.load(std::memory_order_relaxed) >= max_http_connections_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "rejecting HTTP client: active connections reached limit %d",
          max_http_connections_);
        send_response(client_fd, {503, "application/json", error_json("server busy")});
        ::close(client_fd);
        continue;
      }
      active_http_connections_.fetch_add(1, std::memory_order_acq_rel);
      enqueue_http_client(client_fd);
    }
  }

  void handle_client(const int client_fd)
  {
    std::string raw;
    char buffer[4096];
    std::size_t expected_body = 0;

    while (running_.load()) {
      const ssize_t count = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        break;
      }
      raw.append(buffer, static_cast<std::size_t>(count));
      const auto header_end = raw.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        expected_body = content_length_from_headers(raw.substr(0, header_end));
        const auto current_body = raw.size() - header_end - 4;
        if (current_body >= expected_body) {
          break;
        }
      }
      if (raw.size() > 1024 * 1024) {
        send_response(client_fd, {400, "application/json", error_json("request too large")});
        ::close(client_fd);
        return;
      }
    }

    const auto request = parse_http_request(raw);
    if (!request) {
      send_response(client_fd, {400, "application/json", error_json("invalid HTTP request")});
      ::close(client_fd);
      return;
    }

    if (request->method == "GET" && request->path == "/ws/v1/teleop") {
      handle_teleop_websocket(client_fd, *request);
      ::close(client_fd);
      return;
    }

    const auto started_at = std::chrono::steady_clock::now();
    HttpResponse response;
    try {
      response = route(*request);
    } catch (const std::exception & exc) {
      response = {
        500,
        "application/json",
        error_json(std::string("unhandled API exception: ") + exc.what())
      };
    } catch (...) {
      response = {500, "application/json", error_json("unknown unhandled API exception")};
    }
    const auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count();
    log_http_request(*request, response.status, latency_ms);
    send_response(client_fd, response);
    ::close(client_fd);
  }

  void send_response(const int client_fd, const HttpResponse & response)
  {
    std::ostringstream out;
    out << "HTTP/1.1 " << response.status << " " << reason_phrase(response.status) << "\r\n";
    out << "Content-Type: " << response.content_type << "\r\n";
    out << "Content-Length: " << response.body.size() << "\r\n";
    out << "Connection: close\r\n";
    out << "Access-Control-Allow-Origin: *\r\n";
    out << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
    out << "Access-Control-Allow-Headers: Content-Type, X-Robot-Token\r\n";
    out << "\r\n";
    out << response.body;
    const auto text = out.str();
    if (!send_all_text(client_fd, text)) {
      RCLCPP_WARN(get_logger(), "failed to send full HTTP response status=%d bytes=%zu", response.status, text.size());
    }
  }

  void log_http_request(const HttpRequest & request, const int status, const long latency_ms)
  {
    if (status >= 500) {
      RCLCPP_ERROR(
        get_logger(),
        "HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
      return;
    }
    if (status >= 400) {
      RCLCPP_WARN(
        get_logger(),
        "HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
      return;
    }
    if (latency_ms > 2000) {
      RCLCPP_WARN(
        get_logger(),
        "slow HTTP %s %s -> %d in %ld ms",
        request.method.c_str(),
        request.path.c_str(),
        status,
        latency_ms);
    }
  }

  bool token_allowed(const HttpRequest & request) const
  {
    if (api_token_.empty()) {
      return true;
    }
    const auto it = request.headers.find("x-robot-token");
    return it != request.headers.end() && it->second == api_token_;
  }

  HttpResponse route(const HttpRequest & request)
  {
    if (request.method == "OPTIONS") {
      return {200, "application/json", "{\"ok\":true}"};
    }
    if (!token_allowed(request)) {
      return {401, "application/json", error_json("missing or invalid X-Robot-Token")};
    }

    if (request.method == "GET" && request.path == "/api/v1/status") {
      return handle_status();
    }
    if (request.method == "GET" && request.path == "/api/v1/robot/pose") {
      return handle_robot_pose(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps") {
      return handle_maps();
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/semantic_layer") {
      return handle_get_semantic_layer(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/poses") {
      return handle_get_poses(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/filters/keepout") {
      return handle_get_keepout_filter(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/mapping/2d/map") {
      return handle_mapping_2d_map_png(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/openapi") {
      return handle_openapi();
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/acquire") {
      return handle_subscription_update(request.body, "acquire");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/release") {
      return handle_subscription_update(request.body, "release");
    }
    if (request.method == "POST" && request.path == "/api/v1/subscriptions/heartbeat") {
      return handle_subscription_update(request.body, "heartbeat");
    }
    if (request.method == "POST" && request.path == "/api/v1/mapping/2d/start") {
      return handle_start_mapping_2d();
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/stop" || request.path == "/api/v1/mapping/stop")) {
      return handle_stop_mapping_2d();
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/save" || request.path == "/api/v1/mapping/save")) {
      return handle_save_mapping_2d(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/delete") {
      return handle_delete_map(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses") {
      return handle_save_pose(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses/save") {
      return handle_save_pose(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/poses/save_current") {
      return handle_save_current_pose(request.body);
    }
    if (request.method == "PUT" && request.path == "/api/v1/maps/poses/batch") {
      return handle_replace_poses_batch(request.body);
    }
    const std::string pose_item_prefix = "/api/v1/maps/poses/";
    if ((request.method == "PUT" || request.method == "DELETE") && starts_with(request.path, pose_item_prefix)) {
      const auto pose_id = request.path.substr(pose_item_prefix.size());
      if (!safe_pose_id(pose_id)) {
        return {400, "application/json", error_json("valid pose_id path segment is required")};
      }
      if (request.method == "PUT") {
        return handle_save_pose(request.body, std::optional<std::string>(pose_id));
      }
      return handle_delete_pose(request, pose_id);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/filters/keepout/save") {
      return handle_save_keepout_filter(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/safety/stop") {
      return publish_estop(true);
    }
    if (request.method == "POST" && request.path == "/api/v1/safety/resume") {
      return publish_estop(false);
    }
    if (request.method == "POST" && request.path == "/api/v1/floors/switch") {
      return handle_switch_floor(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/localization/trigger") {
      return handle_trigger_localization(request.body);
    }
    if (request.method == "GET" && request.path == "/api/v1/navigation/state") {
      return handle_navigation_state();
    }
    if (request.method == "GET" && request.path == "/api/v1/navigation/pre_goal_check") {
      return handle_navigation_pre_goal_check(request);
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/goal") {
      return handle_navigation_goal(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/cancel") {
      return handle_navigation_cancel(request.body, false);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/navigation/stop" || request.path == "/api/v1/navigation/stop_runtime"))
    {
      return handle_navigation_cancel(request.body, true);
    }
    if (request.method == "GET" && request.path == "/api/v1/docking/state") {
      return handle_docking_state();
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/start") {
      return handle_docking_start(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/undock") {
      return handle_docking_undock(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/confirm_docked") {
      return handle_docking_confirm_docked(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/clear_docked_latch") {
      return handle_docking_clear_latch(request.body);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/docking/cancel" || request.path == "/api/v1/docking/stop")) {
      return handle_docking_cancel(request.body);
    }
    if (starts_with(request.path, "/api/v1/mapping/") || starts_with(request.path, "/api/v1/navigation/")) {
      return not_wired(request.path);
    }
    return {404, "application/json", error_json("endpoint not found: " + request.path)};
  }

  HttpResponse handle_status()
  {
    refresh_mapping_2d_runtime_state();
    refresh_navigation_resume_runtime_state(false);

    std::string safety_status;
    std::string floor_status;
    bool motion_allowed = false;
    bool have_motion_allowed = false;
    bool have_bms_state = false;
    bool have_bms_soc = false;
    double bms_soc = 0.0;
    double bms_voltage = 0.0;
    double bms_current = 0.0;
    double bms_temperature = 0.0;
    double bms_age_sec = -1.0;
    int bms_power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    int bms_power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    int bms_power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    bool bms_present = false;
    bool bms_charging_contact = false;
    std::string bms_charging_contact_reason{"no_bms_state"};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      safety_status = latest_safety_status_;
      floor_status = latest_floor_status_;
      motion_allowed = latest_motion_allowed_;
      have_motion_allowed = have_motion_allowed_;
      have_bms_state = have_bms_state_;
      have_bms_soc = have_bms_soc_;
      bms_soc = latest_bms_soc_;
      bms_voltage = latest_bms_voltage_;
      bms_current = latest_bms_current_;
      bms_temperature = latest_bms_temperature_;
      bms_power_supply_status = latest_bms_power_supply_status_;
      bms_power_supply_health = latest_bms_power_supply_health_;
      bms_power_supply_technology = latest_bms_power_supply_technology_;
      bms_present = latest_bms_present_;
      bms_charging_contact = latest_bms_charging_contact_;
      bms_charging_contact_reason = latest_bms_charging_contact_reason_;
      if (have_bms_state_) {
        bms_age_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_bms_received_at_).count();
      }
    }
    if (!have_bms_state) {
      bms_charging_contact = false;
      bms_charging_contact_reason = "no_bms_state";
    } else if (bms_age_sec > bms_state_max_age_sec_) {
      bms_charging_contact = false;
      bms_charging_contact_reason = "stale_bms_state";
    }
    const bool bms_valid = have_bms_state && have_bms_soc && bms_age_sec <= bms_state_max_age_sec_;
    const auto runtime = runtime_mode_snapshot();
    const auto status_dock_check = pre_navigation_dock_check_snapshot();
    const bool inferred_docked = status_dock_check.inferred_docked;
    const auto status_dock_check_json = pre_navigation_dock_check_json(
      status_dock_check,
      "status",
      "",
      "",
      "",
      "map",
      false);
    const auto localization = localization_result_snapshot();
    const auto amcl_status = read_amcl_runtime_status();
    const auto bridge_status = bridge_status_snapshot();
    const bool amcl_file_authoritative = amcl_status.available && !amcl_status.stale;
    const bool bridge_amcl_available = bridge_status.available && bridge_status.amcl_input_enabled;
    const bool effective_amcl_ready =
      bridge_amcl_available ? bridge_status.amcl_ready :
      (amcl_file_authoritative && amcl_status.ready);
    const bool effective_amcl_degraded =
      bridge_amcl_available ? bridge_status.localization_degraded :
      (amcl_file_authoritative && (amcl_status.degraded || !amcl_status.ready));
    const std::string effective_amcl_degraded_reason =
      bridge_amcl_available ?
      (bridge_status.amcl_degraded_reason.empty() ?
        (bridge_status.localization_degraded ? std::string("AMCL_NOT_READY") : std::string()) :
        bridge_status.amcl_degraded_reason) :
      (effective_amcl_degraded ?
        (amcl_status.degraded_reason.empty() ? std::string("AMCL_NOT_READY") : amcl_status.degraded_reason) :
        std::string());
    const bool localization_degraded =
      effective_amcl_degraded;
    const bool using_triggered_baseline_only =
      (bridge_amcl_available || (amcl_status.available && amcl_status.mode != "disabled")) &&
      !effective_amcl_ready;
    const std::string localization_degraded_reason =
      localization_degraded ? effective_amcl_degraded_reason : std::string();
    bool live_mapping_map_available = false;
    double live_mapping_map_age_sec = 0.0;
    std::uint32_t live_mapping_map_width = 0U;
    std::uint32_t live_mapping_map_height = 0U;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (have_live_map_) {
        live_mapping_map_available = true;
        live_mapping_map_age_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
        live_mapping_map_width = latest_live_map_.info.width;
        live_mapping_map_height = latest_live_map_.info.height;
      }
    }
    std::string navigation_goal_json;
    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      navigation_goal_json = navigation_goal_job_json_locked();
    }

    std::ostringstream body;
    body << "{";
    body << "\"ok\":true,";
    body << "\"api_version\":\"v1\",";
    body << "\"node\":\"robot_api_server\",";
    body << "\"mode\":" << json_string(runtime.mode) << ",";
    body << "\"state\":" << json_string(runtime.state) << ",";
    body << "\"mapping_active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ",";
    body << "\"message\":" << json_string(runtime.message) << ",";
    body << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ",";
    body << "\"localization_degraded_reason\":"
         << json_string(localization_degraded_reason) << ",";
    body << "\"using_triggered_baseline_only\":"
         << (using_triggered_baseline_only ? "true" : "false") << ",";
    body << "\"mapping\":{";
    body << "\"active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.mapping_state) << ",";
    body << "\"map_topic\":" << json_string(mapping_2d_live_map_topic_) << ",";
    body << "\"map_endpoint\":\"/api/v1/mapping/2d/map\",";
    body << "\"live_map_available\":" << (live_mapping_map_available ? "true" : "false") << ",";
    if (live_mapping_map_available) {
      body << "\"live_map_age_sec\":" << live_mapping_map_age_sec << ",";
      body << "\"live_map_width\":" << live_mapping_map_width << ",";
      body << "\"live_map_height\":" << live_mapping_map_height;
    } else {
      body << "\"live_map_age_sec\":null,";
      body << "\"live_map_width\":0,";
      body << "\"live_map_height\":0";
    }
    body << "},";
    body << "\"navigation\":{";
    body << "\"active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.navigation_state) << ",";
    body << "\"action\":" << json_string(navigate_to_pose_action_) << ",";
    body << "\"pre_goal_check_endpoint\":\"/api/v1/navigation/pre_goal_check\",";
    body << "\"blocked_by_docked_contact\":"
         << (status_dock_check.final_auto_undock_required ? "true" : "false") << ",";
    body << "\"normal_motion_blocked_reason\":"
         << json_string(safety_status == "DOCKED_CONTACT_BLOCK" ? "DOCKED_CONTACT_BLOCK" : "") << ",";
    body << "\"pre_navigation_dock_check\":" << status_dock_check_json << ",";
    body << "\"post_relocalization_settle\":" << post_relocalization_settle_state_json() << ",";
    body << "\"goal\":" << navigation_goal_json;
    body << "},";
    body << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"docking\":{";
    body << "\"active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.docking_state) << ",";
    body << "\"charging_contact\":" << (bms_charging_contact ? "true" : "false") << ",";
    body << "\"inferred_docked\":" << (inferred_docked ? "true" : "false") << ",";
    body << "\"dock_id\":" << json_string(runtime.docking_dock_id) << ",";
    body << "\"status_topic\":" << json_string(docking_status_topic_) << ",";
    body << "\"last_status\":" << json_string(runtime.docking_status) << ",";
    body << "\"can_auto_undock\":" << (status_dock_check.can_auto_undock ? "true" : "false") << ",";
    body << "\"auto_undock_reason\":" << json_string(status_dock_check.auto_undock_reason) << ",";
    body << "\"pre_navigation_dock_check\":" << status_dock_check_json;
    body << "},";
    body << "\"safety_status\":" << json_string(safety_status) << ",";
    body << "\"motion_allowed\":" << (motion_allowed ? "true" : "false") << ",";
    body << "\"motion_allowed_valid\":" << (have_motion_allowed ? "true" : "false") << ",";
    body << "\"safety\":{";
    body << "\"status\":" << json_string(safety_status) << ",";
    body << "\"motion_allowed\":" << (motion_allowed ? "true" : "false") << ",";
    body << "\"motion_allowed_valid\":" << (have_motion_allowed ? "true" : "false");
    body << "},";
    body << "\"floor_status\":" << json_string(floor_status) << ",";
    body << "\"localization\":{";
    body << "\"trigger_service\":" << json_string(localization_trigger_service_) << ",";
    body << "\"result_topic\":" << json_string(localization_result_topic_) << ",";
    body << "\"amcl_mode\":" << json_string(amcl_status.mode) << ",";
    body << "\"amcl_state\":" << json_string(amcl_status.state) << ",";
    body << "\"amcl_start_result\":" << json_string(amcl_status.start_result) << ",";
    body << "\"amcl_status_file_stale\":" << (amcl_status.stale ? "true" : "false") << ",";
    body << "\"amcl_status_age_ms\":" << amcl_status.age_ms << ",";
    body << "\"amcl_status_source\":" << json_string(
      bridge_amcl_available ? bridge_status.amcl_status_source :
      (amcl_file_authoritative ? std::string("file") : std::string("stale_file_ignored"))) << ",";
    body << "\"amcl_ready\":" << (effective_amcl_ready ? "true" : "false") << ",";
    body << "\"amcl_degraded\":" << (effective_amcl_degraded ? "true" : "false") << ",";
    body << "\"amcl_degraded_reason\":" << json_string(effective_amcl_degraded_reason) << ",";
    body << "\"amcl_process_alive\":" << (amcl_status.process_alive ? "true" : "false") << ",";
    body << "\"amcl_process_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_process_ready : amcl_status.process_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_seeded\":" << ((
      bridge_amcl_available ? bridge_status.amcl_seeded : amcl_status.seeded
    ) ? "true" : "false") << ",";
    body << "\"amcl_seed_response_ok\":" << ((
      bridge_amcl_available ? bridge_status.amcl_seed_response_ok : amcl_status.seed_response_ok
    ) ? "true" : "false") << ",";
    body << "\"amcl_nomotion_pose_received\":" << ((
      bridge_amcl_available ? bridge_status.amcl_nomotion_pose_received : amcl_status.nomotion_pose_received
    ) ? "true" : "false") << ",";
    body << "\"amcl_static_standby\":" << ((
      bridge_amcl_available ? bridge_status.amcl_static_standby : amcl_status.static_standby
    ) ? "true" : "false") << ",";
    body << "\"amcl_tracking_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_tracking_ready : amcl_status.tracking_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_correction_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_correction_ready : amcl_status.correction_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_not_moving_no_update_ok\":" << ((
      bridge_amcl_available ?
        bridge_status.amcl_not_moving_no_update_ok :
        amcl_status.not_moving_no_update_ok
    ) ? "true" : "false") << ",";
    body << "\"amcl_scan_admission_alive\":"
         << (amcl_status.scan_admission_alive ? "true" : "false") << ",";
    body << "\"amcl_pose_publisher_count\":" << amcl_status.pose_publisher_count << ",";
    body << "\"amcl_scan_admission_status_publisher_count\":"
         << amcl_status.scan_admission_status_publisher_count << ",";
    body << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ",";
    body << "\"localization_degraded_reason\":"
         << json_string(localization_degraded_reason) << ",";
    body << "\"using_triggered_baseline_only\":"
         << (using_triggered_baseline_only ? "true" : "false") << ",";
    body << "\"last_result_available\":" << (localization.available ? "true" : "false") << ",";
    body << "\"last_result_frame\":" << json_string(localization.frame_id) << ",";
    body << "\"last_result_age_sec\":";
    if (localization.available) {
      body << localization.age_sec;
    } else {
      body << "null";
    }
    body << ",\"last_result_seq\":" << localization.seq << ",";
    body << "\"post_relocalization_settle\":" << post_relocalization_settle_state_json();
    body << "},";
    body << "\"bms\":{";
    body << "\"soc\":";
    if (have_bms_soc) {
      body << bms_soc;
    } else {
      body << "null";
    }
    body << ",\"soc_valid\":" << (bms_valid ? "true" : "false") << ",";
    body << "\"source_topic\":" << json_string(bms_state_topic_) << ",";
    body << "\"age_sec\":" << bms_age_sec << ",";
    body << "\"power_supply_status\":" << bms_power_supply_status << ",";
    body << "\"power_supply_health\":" << bms_power_supply_health << ",";
    body << "\"power_supply_technology\":" << bms_power_supply_technology << ",";
    body << "\"present\":" << (bms_present ? "true" : "false") << ",";
    body << "\"charging_contact\":" << (bms_charging_contact ? "true" : "false") << ",";
    body << "\"charging_contact_reason\":" << json_string(bms_charging_contact_reason) << ",";
    body << "\"contact_snapshot\":{" << bms_charging_contact_snapshot_json(status_dock_check.bms) << "},";
    body << "\"voltage\":";
    if (have_bms_state && std::isfinite(bms_voltage)) {
      body << bms_voltage;
    } else {
      body << "null";
    }
    body << ",\"current\":";
    if (have_bms_state && std::isfinite(bms_current)) {
      body << bms_current;
    } else {
      body << "null";
    }
    body << ",\"temperature\":";
    if (have_bms_state && std::isfinite(bms_temperature)) {
      body << bms_temperature;
    } else {
      body << "null";
    }
    body << "},";
    body << "\"subscriptions\":";
    if (subscription_manager_) {
      body << subscription_manager_->snapshot_json();
    } else {
      body << "{\"resources\":{}}";
    }
    body << ",";
    body << "\"http\":{";
    body << "\"active_connections\":" << active_http_connections_.load(std::memory_order_relaxed) << ",";
    body << "\"max_connections\":" << max_http_connections_;
    body << "},";
    body << "\"maps_root\":" << json_string(maps_root_) << ",";
    body << "\"runtime_maps_dir\":" << json_string(runtime_maps_dir_);
    body << "}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_robot_pose(const HttpRequest & request)
  {
    (void)request;
    std::string error;
    const auto pose = wait_for_current_robot_pose(true, error);
    if (!pose.available) {
      (void)error;
      return {503, "application/json", no_fresh_map_robot_pose_json(tf_map_frame_, tf_base_frame_)};
    }

    std::string context_error;
    bool blocked_by_pending_context = false;
    const auto active_map = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
    if (blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(tf_map_frame_, tf_base_frame_, context_error)};
    }
    RobotPoseMapIdentity map_identity;
    if (active_map) {
      map_identity.map_id = active_map->map_id;
      map_identity.floor_id = active_map->floor_id;
      map_identity.building_id = active_map->building_id;
    }
    return {200, "application/json", robot_pose_json(pose, map_identity)};
  }

  HttpResponse handle_openapi()
  {
    const std::string body =
      "{"
      "\"ok\":true,"
      "\"endpoints\":["
      "\"GET /api/v1/status\","
      "\"GET /api/v1/robot/pose\","
      "\"GET /api/v1/maps\","
      "\"GET /api/v1/maps/semantic_layer\","
      "\"GET /api/v1/maps/poses\","
      "\"GET /api/v1/maps/filters/keepout\","
      "\"GET /api/v1/mapping/2d/map\","
      "\"GET /api/v1/openapi\","
      "\"POST /api/v1/maps/poses\","
      "\"PUT /api/v1/maps/poses/{pose_id}\","
      "\"DELETE /api/v1/maps/poses/{pose_id}\","
      "\"PUT /api/v1/maps/poses/batch\","
      "\"POST /api/v1/subscriptions/acquire\","
      "\"POST /api/v1/subscriptions/release\","
      "\"POST /api/v1/subscriptions/heartbeat\","
      "\"POST /api/v1/mapping/2d/start\","
      "\"POST /api/v1/mapping/2d/stop\","
      "\"POST /api/v1/mapping/2d/save\","
      "\"POST /api/v1/mapping/stop\","
      "\"POST /api/v1/mapping/save\","
      "\"POST /api/v1/maps/delete\","
      "\"POST /api/v1/maps/poses/save\","
      "\"POST /api/v1/maps/poses/save_current\","
      "\"POST /api/v1/maps/filters/keepout/save\","
      "\"POST /api/v1/safety/stop\","
      "\"POST /api/v1/safety/resume\","
      "\"POST /api/v1/floors/switch\","
      "\"POST /api/v1/localization/trigger\","
      "\"GET /api/v1/navigation/state\","
      "\"GET /api/v1/navigation/pre_goal_check\","
      "\"POST /api/v1/navigation/goal\","
      "\"POST /api/v1/navigation/cancel\","
      "\"POST /api/v1/navigation/stop\","
      "\"POST /api/v1/navigation/stop_runtime\","
      "\"GET /api/v1/docking/state\","
      "\"POST /api/v1/docking/start\","
      "\"POST /api/v1/docking/undock\","
      "\"POST /api/v1/docking/confirm_docked\","
      "\"POST /api/v1/docking/clear_docked_latch\","
      "\"POST /api/v1/docking/cancel\","
      "\"POST /api/v1/docking/stop\","
      "\"WS /ws/v1/teleop\""
      "],"
      "\"not_wired\":["
      "\"POST /api/v1/mapping/3d/start\","
      "\"POST /api/v1/navigation/start\""
      "]"
      "}";
    return {200, "application/json", body};
  }

  HttpResponse handle_subscription_update(const std::string & body, const std::string & action)
  {
    if (!subscription_manager_) {
      return {503, "application/json", error_json("subscription manager is not initialized")};
    }
    const auto [client_id, client_id_source] = subscription_client_id_from_body(body);
    auto resources = subscription_resources_from_body(body);
    if (action == "heartbeat" && resources.empty()) {
      resources = subscription_manager_->resources_for_client(client_id);
    }
    if (action == "heartbeat" && resources.empty()) {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"action\":" << json_string(action) << ","
               << "\"client_id\":" << json_string(client_id) << ","
               << "\"lease_id\":" << json_string(client_id) << ","
               << "\"client_id_source\":" << json_string(client_id_source) << ","
               << "\"refreshed\":false,"
               << "\"ttl_ms\":0,"
               << "\"resources\":[],"
               << "\"subscriptions\":" << subscription_manager_->snapshot_json() << "}";
      return {200, "application/json", response.str()};
    }
    if (action != "release" && resources.empty()) {
      return {400, "application/json", error_json("resources array is required")};
    }
    if (const auto error = subscription_manager_->validate_resources(resources)) {
      return {400, "application/json", error_json(*error)};
    }

    int ttl_ms = subscription_ttl_ms_from_body(
      body,
      subscription_default_ttl_ms_,
      subscription_max_ttl_ms_);
    if (action == "acquire" || action == "heartbeat") {
      subscription_manager_->acquire(client_id, resources, std::chrono::milliseconds(ttl_ms));
    } else if (action == "release") {
      subscription_manager_->release(client_id, resources);
      ttl_ms = 0;
    } else {
      return {400, "application/json", error_json("unsupported subscription action")};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"action\":" << json_string(action) << ","
             << "\"client_id\":" << json_string(client_id) << ","
             << "\"lease_id\":" << json_string(client_id) << ","
             << "\"client_id_source\":" << json_string(client_id_source) << ","
             << "\"refreshed\":" << ((action == "heartbeat") ? "true" : "false") << ","
             << "\"ttl_ms\":" << ttl_ms << ","
             << "\"resources\":" << resource_list_json(resources) << ","
             << "\"subscriptions\":" << subscription_manager_->snapshot_json() << "}";
    return {200, "application/json", response.str()};
  }

  void write_runtime_map_context(
    const MapManifest & manifest,
    const std::string & state,
    const bool confirmed,
    const std::string & message) const
  {
    if (runtime_map_context_file_.empty()) {
      return;
    }
    write_runtime_map_context_file(
      fs::path(runtime_map_context_file_), manifest, state, confirmed, message, wall_time_seconds());
  }

  std::optional<RuntimeMapContext> read_runtime_map_context() const
  {
    if (runtime_map_context_file_.empty()) {
      return std::nullopt;
    }
    return read_runtime_map_context_file(fs::path(runtime_map_context_file_));
  }

  AmclRuntimeStatus read_amcl_runtime_status() const
  {
    AmclRuntimeStatus status;
    if (amcl_runtime_status_file_.empty()) {
      return status;
    }
    std::ifstream input(amcl_runtime_status_file_);
    if (!input.good()) {
      return status;
    }
    std::map<std::string, std::string> fields;
    std::string line;
    while (std::getline(input, line)) {
      line = trim(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      const auto equal_pos = line.find('=');
      if (equal_pos == std::string::npos) {
        continue;
      }
      const auto key = trim(line.substr(0U, equal_pos));
      if (!key.empty()) {
        fields[key] = unquote_env_value(line.substr(equal_pos + 1U));
      }
    }
    auto string_field = [&](const std::string & key) -> std::string {
        const auto it = fields.find(key);
        return it == fields.end() ? std::string() : it->second;
      };
    auto bool_field = [&](const std::string & key) -> bool {
        const auto value = string_field(key);
        return value == "true" || value == "True" || value == "1";
      };
    auto int_field = [&](const std::string & key) -> int {
        const auto value = string_field(key);
        if (value.empty()) {
          return 0;
        }
        try {
          return std::stoi(value);
        } catch (const std::exception &) {
          return 0;
        }
      };
    auto double_field = [&](const std::string & key, const double fallback = -1.0) -> double {
        const auto value = string_field(key);
        if (value.empty()) {
          return fallback;
        }
        try {
          return std::stod(value);
        } catch (const std::exception &) {
          return fallback;
        }
      };
    status.available = true;
    status.mode = string_field("AMCL_MODE");
    status.state = string_field("AMCL_STATE");
    status.start_result = string_field("AMCL_START_RESULT");
    status.ready = bool_field("AMCL_READY");
    status.degraded = bool_field("AMCL_DEGRADED");
    status.degraded_reason = string_field("AMCL_FAILURE_REASON");
    status.process_alive = bool_field("AMCL_PID_ALIVE");
    status.scan_admission_alive = bool_field("SCAN_ADMISSION_ALIVE");
    status.pose_publisher_count = int_field("AMCL_POSE_PUBLISHER_COUNT");
    status.scan_admission_status_publisher_count = int_field("SCAN_ADMISSION_STATUS_PUBLISHER_COUNT");
    status.seed_succeeded = bool_field("AMCL_SEED_SUCCEEDED");
    status.seed_response_ok = bool_field("AMCL_SEED_RESPONSE_OK");
    status.nomotion_probe_used = bool_field("AMCL_NOMOTION_PROBE_USED");
    status.nomotion_pose_received = bool_field("AMCL_NOMOTION_POSE_RECEIVED");
    status.nomotion_pose_count = int_field("AMCL_NOMOTION_POSE_COUNT");
    status.nomotion_pose_header_age_ms = double_field("AMCL_NOMOTION_POSE_HEADER_AGE_MS");
    status.process_ready = bool_field("AMCL_PROCESS_READY");
    status.seeded = bool_field("AMCL_SEEDED") || status.seed_succeeded || status.seed_response_ok;
    status.static_standby = bool_field("AMCL_STATIC_STANDBY");
    status.tracking_ready = bool_field("AMCL_TRACKING_READY");
    status.correction_ready = bool_field("AMCL_CORRECTION_READY");
    status.not_moving_no_update_ok = bool_field("AMCL_NOT_MOVING_NO_UPDATE_OK");
    status.stamp_sec = double_field("AMCL_STATUS_STAMP_SEC");
    if (status.stamp_sec > 0.0) {
      status.age_ms = std::max(0.0, (wall_time_seconds() - status.stamp_sec) * 1000.0);
      status.stale = status.age_ms > amcl_runtime_status_ttl_sec_ * 1000.0;
    } else {
      status.age_ms = -1.0;
      status.stale = true;
    }
    status.stamp = string_field("TIMESTAMP");
    return status;
  }

  void clear_runtime_map_context() const
  {
    if (runtime_map_context_file_.empty()) {
      return;
    }
    std::error_code ec;
    fs::remove(fs::path(runtime_map_context_file_), ec);
  }

  void write_last_navigation_map_selection(
    const MapManifest & manifest,
    const std::string & reason) const
  {
    if (last_navigation_map_file_.empty()) {
      return;
    }
    std::ostringstream body;
    body << std::fixed << std::setprecision(6)
         << "{\n"
         << "  \"schema\": \"njrh.last_navigation_map.v1\",\n"
         << "  \"reason\": " << json_string(reason) << ",\n"
         << "  \"map_id\": " << json_string(manifest.map_id) << ",\n"
         << "  \"display_name\": " << json_string(manifest.display_name) << ",\n"
         << "  \"building_id\": " << json_string(manifest.building_id) << ",\n"
         << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n"
         << "  \"updated_at\": " << wall_time_seconds() << "\n"
         << "}\n";
    write_text_file(fs::path(last_navigation_map_file_), body.str());
  }

  std::optional<MapManifest> confirmed_runtime_map_manifest(
    std::string & unavailable_reason,
    bool & blocked_by_pending_context)
  {
    blocked_by_pending_context = false;
    const auto runtime = runtime_mode_snapshot();
    const auto context = read_runtime_map_context();
    if (context) {
      if (!context->confirmed) {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "runtime map context is not confirmed yet: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id +
            " state=" + context->state;
        }
        return std::nullopt;
      }
      const auto manifest = map_catalog_->find_map_by_id(context->map_id);
      if (!manifest || manifest->building_id != context->building_id ||
        manifest->floor_id != context->floor_id)
      {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "confirmed runtime map context does not match a valid manifest: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id;
        }
        return std::nullopt;
      }
      return manifest;
    }

    if (runtime.navigation_active || runtime.docking_active) {
      blocked_by_pending_context = true;
      unavailable_reason = "navigation or docking is active but no runtime map context is recorded";
      return std::nullopt;
    }

    if (runtime.mapping_active) {
      return std::nullopt;
    }

    return map_catalog_->unique_active_map_manifest();
  }

  bool validate_map_manifest_assets(const MapManifest & manifest, std::string & error) const
  {
    const std::vector<fs::path> required = {
      manifest.nav_map_yaml,
      manifest.nav_map_pgm,
      manifest.localizer_map_png,
      manifest.localizer_params_yaml,
      manifest.keepout_mask_yaml,
      manifest.keepout_mask_pgm,
      manifest.speed_mask_yaml,
      manifest.speed_mask_pgm,
      manifest.binary_mask_yaml,
      manifest.binary_mask_pgm,
      manifest.asset_report_json,
      manifest.poses_yaml
    };
    for (const auto & path : required) {
      if (!fs::exists(path)) {
        error = "map asset is incomplete, missing: " + path.string();
        return false;
      }
    }
    return true;
  }

  void sync_manifest_to_fixed_entry(
    const MapManifest & manifest,
    const fs::path & fixed_root,
    const bool include_manifest) const
  {
    copy_file_if_exists(manifest.nav_map_pgm, fixed_root / "nav" / "nav_map.pgm");
    copy_yaml_with_image_if_exists(
      manifest.nav_map_yaml, fixed_root / "nav" / "nav_map.yaml", "nav_map.pgm");
    copy_file_if_exists(manifest.localizer_map_png, fixed_root / "localizer" / "localizer_map.png");
    copy_yaml_with_image_if_exists(
      manifest.localizer_params_yaml,
      fixed_root / "localizer" / "localizer_params.yaml",
      "localizer_map.png");
    copy_file_if_exists(manifest.keepout_mask_pgm, fixed_root / "filters" / "keepout_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.keepout_mask_yaml, fixed_root / "filters" / "keepout_mask.yaml", "keepout_mask.pgm");
    copy_file_if_exists(manifest.speed_mask_pgm, fixed_root / "filters" / "speed_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.speed_mask_yaml, fixed_root / "filters" / "speed_mask.yaml", "speed_mask.pgm");
    copy_file_if_exists(manifest.binary_mask_pgm, fixed_root / "filters" / "binary_mask.pgm");
    copy_yaml_with_image_if_exists(
      manifest.binary_mask_yaml, fixed_root / "filters" / "binary_mask.yaml", "binary_mask.pgm");
    copy_file_if_exists(manifest.asset_report_json, fixed_root / "reports" / "asset_report.json");
    copy_file_if_exists(manifest.poses_yaml, fixed_root / "poses.yaml");
    if (include_manifest) {
      write_text_file(fixed_root / "manifest.json", map_manifest_json(manifest));
    }
  }

  void remove_current_map_entry(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = map_catalog_->floor_root_path(building_id, floor_id);
    const auto current_root = map_catalog_->floor_current_root_path(building_id, floor_id);
    if (current_root.filename() != "current" || current_root.parent_path() != floor_root) {
      throw std::runtime_error("refusing unsafe current map reset path: " + current_root.string());
    }

    std::error_code ec;
    fs::remove_all(current_root, ec);
    if (!ec) {
      return;
    }

    const auto remove_error = ec.message();
    std::error_code status_ec;
    const auto status = fs::symlink_status(current_root, status_ec);
    if (status_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " status=" + status_ec.message());
    }
    if (!fs::exists(status)) {
      return;
    }

    // Old dashboard/test runs may leave current/ as a non-empty root-owned directory.
    // Renaming only needs write permission on the floor directory, then the new
    // current/ can be created by the API runtime user.
    fs::path stale_root;
    for (int index = 0; index < 100; ++index) {
      stale_root = floor_root /
        (".stale_current_" + utc_timestamp_compact() + "_" + std::to_string(::getpid()) + "_" +
        std::to_string(index));
      if (!fs::exists(stale_root)) {
        break;
      }
    }

    std::error_code rename_ec;
    fs::rename(current_root, stale_root, rename_ec);
    if (rename_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " rename=" + rename_ec.message());
    }

    std::error_code cleanup_ec;
    fs::remove_all(stale_root, cleanup_ec);
    if (cleanup_ec) {
      RCLCPP_WARN(
        get_logger(),
        "quarantined stale current map entry at %s after reset; cleanup skipped: %s",
        stale_root.string().c_str(), cleanup_ec.message().c_str());
    }
  }

  void activate_map_manifest(MapManifest manifest)
  {
    std::string error;
    if (!validate_map_manifest_assets(manifest, error)) {
      throw std::runtime_error(error);
    }
    for (auto other : map_catalog_->read_floor_map_manifests(manifest.building_id, manifest.floor_id, false)) {
      if (other.map_id == manifest.map_id) {
        continue;
      }
      if (other.active) {
        other.active = false;
        write_map_manifest(other);
      }
    }

    manifest.active = true;
    write_map_manifest(manifest);

    const auto current_root = map_catalog_->floor_current_root_path(manifest.building_id, manifest.floor_id);
    remove_current_map_entry(manifest.building_id, manifest.floor_id);
    sync_manifest_to_fixed_entry(manifest, current_root, true);

    // Keep the historical fixed role files in the floor root as a compatibility shim for older tools.
    sync_manifest_to_fixed_entry(manifest, map_catalog_->floor_root_path(manifest.building_id, manifest.floor_id), false);
  }

  void clear_fixed_floor_entries(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = map_catalog_->floor_root_path(building_id, floor_id);
    std::error_code ec;
    remove_current_map_entry(building_id, floor_id);
    fs::remove_all(floor_root / "nav", ec);
    fs::remove_all(floor_root / "localizer", ec);
    fs::remove_all(floor_root / "filters", ec);
    fs::remove_all(floor_root / "reports", ec);
    fs::remove(floor_root / "poses.yaml", ec);
  }

  void ensure_legacy_floor_map_manifest(
    const std::string & building_id,
    const std::string & floor_id)
  {
    if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
      return;
    }
    const auto maps_root = map_catalog_->floor_maps_root_path(building_id, floor_id);
    if (fs::exists(maps_root) && fs::is_directory(maps_root)) {
      for (const auto & entry : fs::directory_iterator(maps_root)) {
        if (entry.is_directory() && fs::exists(entry.path() / "manifest.json")) {
          return;
        }
      }
    }

    const auto floor_root = map_catalog_->floor_root_path(building_id, floor_id);
    const auto legacy_nav_yaml = floor_root / "nav" / "nav_map.yaml";
    const auto legacy_nav_pgm = floor_root / "nav" / "nav_map.pgm";
    const auto legacy_localizer_png = floor_root / "localizer" / "localizer_map.png";
    const auto legacy_localizer_params = floor_root / "localizer" / "localizer_params.yaml";
    const auto legacy_report = floor_root / "reports" / "asset_report.json";
    const auto legacy_poses = floor_root / "poses.yaml";
    if (!fs::exists(legacy_nav_yaml) || !fs::exists(legacy_nav_pgm) ||
      !fs::exists(legacy_localizer_png) || !fs::exists(legacy_localizer_params))
    {
      return;
    }

    MapManifest manifest;
    manifest.map_id = "legacy_" + fixed_hex(fnv1a64(floor_root.string()), 12);
    manifest.display_name = "legacy_" + floor_id;
    manifest.safe_map_name = safe_file_stem_from_display_name(manifest.display_name);
    manifest.building_id = building_id;
    manifest.floor_id = floor_id;
    manifest.created_at = utc_timestamp_iso8601();
    manifest.active = true;
    manifest.root = map_catalog_->map_root_path(building_id, floor_id, manifest.map_id);
    fill_manifest_paths(manifest);

    copy_file_if_exists(legacy_nav_pgm, manifest.nav_map_pgm);
    copy_yaml_with_image_if_exists(
      legacy_nav_yaml, manifest.nav_map_yaml, manifest.nav_map_pgm.filename().string());
    copy_file_if_exists(legacy_localizer_png, manifest.localizer_map_png);
    copy_yaml_with_image_if_exists(
      legacy_localizer_params,
      manifest.localizer_params_yaml,
      manifest.localizer_map_png.filename().string());
    copy_file_if_exists(floor_root / "filters" / "keepout_mask.pgm", manifest.keepout_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "keepout_mask.yaml",
      manifest.keepout_mask_yaml,
      "keepout_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "speed_mask.pgm", manifest.speed_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "speed_mask.yaml",
      manifest.speed_mask_yaml,
      "speed_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "binary_mask.pgm", manifest.binary_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "binary_mask.yaml",
      manifest.binary_mask_yaml,
      "binary_mask.pgm");
    copy_file_if_exists(legacy_report, manifest.asset_report_json);
    if (!fs::exists(manifest.asset_report_json)) {
      write_text_file(manifest.asset_report_json, "{}\n");
    }
    copy_file_if_exists(legacy_poses, manifest.poses_yaml);
    if (!fs::exists(manifest.poses_yaml)) {
      write_text_file(manifest.poses_yaml, "poses: []\n");
    }
    std::string error;
    if (validate_map_manifest_assets(manifest, error)) {
      write_map_manifest(manifest);
      activate_map_manifest(manifest);
    } else {
      manifest.active = false;
      write_map_manifest(manifest);
      RCLCPP_WARN(get_logger(), "legacy map manifest created but not activated: %s", error.c_str());
    }
  }

  HttpResponse handle_maps()
  {
    std::ostringstream body;
    body << "{\"ok\":true,\"runtime_maps\":[";
    bool first = true;
    if (fs::exists(runtime_maps_dir_)) {
      for (const auto & entry : fs::directory_iterator(runtime_maps_dir_)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
          continue;
        }
        const auto stem = entry.path().stem().string();
        if (stem.size() >= 10 && stem.substr(stem.size() - 10) == ".localizer") {
          continue;
        }
        const auto pgm = entry.path().parent_path() / (stem + ".pgm");
        if (!fs::exists(pgm)) {
          continue;
        }
        if (!first) {
          body << ",";
        }
        first = false;
        const auto map_info = read_nav_map_info(entry.path());
        body << "{\"name\":" << json_string(stem)
             << ",\"yaml\":" << json_string(entry.path().string())
             << ",\"pgm\":" << json_string(pgm.string())
             << ",\"map_info\":" << map_info_json(map_info) << "}";
      }
    }

    const auto manifests = map_catalog_->read_all_map_manifests(true);
    body << "],\"floor_maps\":[";
    first = true;
    for (const auto & manifest : manifests) {
      if (!first) {
        body << ",";
      }
      first = false;
      body << "{\"map_id\":" << json_string(manifest.map_id)
           << ",\"display_name\":" << json_string(manifest.display_name)
           << ",\"map_name\":" << json_string(manifest.display_name)
           << ",\"building_id\":" << json_string(manifest.building_id)
           << ",\"floor_id\":" << json_string(manifest.floor_id)
           << ",\"active\":" << (manifest.active ? "true" : "false")
           << ",\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string())
           << ",\"localizer_map_png\":" << json_string(manifest.localizer_map_png.string())
           << ",\"map_info\":" << map_info_json(read_nav_map_info(manifest.nav_map_yaml))
           << ",\"manifest_json\":" << json_string(manifest.manifest_json.string()) << "}";
    }

    body << "],\"floors\":[";
    first = true;
    if (fs::exists(maps_root_)) {
      for (const auto & building : fs::directory_iterator(maps_root_)) {
        if (!building.is_directory()) {
          continue;
        }
        for (const auto & floor : fs::directory_iterator(building.path())) {
          if (!floor.is_directory()) {
            continue;
          }
          const auto building_id = building.path().filename().string();
          const auto floor_id = floor.path().filename().string();
          const auto active = map_catalog_->active_floor_map(building_id, floor_id);
          const auto current_root = floor.path() / "current";
          const auto root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor.path();
          const auto nav_yaml = root / "nav" / "nav_map.yaml";
          const auto nav_pgm = root / "nav" / "nav_map.pgm";
          const auto localizer_png = root / "localizer" / "localizer_map.png";
          const auto localizer_params = root / "localizer" / "localizer_params.yaml";
          if (!fs::exists(nav_yaml) || !fs::exists(nav_pgm) || !fs::exists(localizer_png) ||
            !fs::exists(localizer_params))
          {
            continue;
          }
          if (!first) {
            body << ",";
          }
          first = false;
          body << "{\"building_id\":" << json_string(building_id)
               << ",\"floor_id\":" << json_string(floor_id)
               << ",\"active_map_id\":" << json_string(active ? active->map_id : "")
               << ",\"active_display_name\":" << json_string(active ? active->display_name : "")
               << ",\"nav_map_yaml\":" << json_string(nav_yaml.string())
               << ",\"nav_map_pgm\":" << json_string(nav_pgm.string())
               << ",\"localizer_map_png\":" << json_string(localizer_png.string())
               << ",\"localizer_params_yaml\":" << json_string(localizer_params.string())
               << ",\"map_info\":" << map_info_json(read_nav_map_info(nav_yaml)) << "}";
        }
      }
    }
    body << "]}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_delete_map(const std::string & body)
  {
    const auto map_id = json_string_value(body, "map_id");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");

    if (!map_id) {
      if (building_id || floor_id) {
        return {
          400,
          "application/json",
          error_json("delete by map_id only; refusing to delete non-empty building/floor assets")
        };
      }
      return {400, "application/json", error_json("map_id is required")};
    }
    if (!safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    const auto manifest = map_catalog_->find_map_by_id(*map_id);
    if (!manifest) {
      return {404, "application/json", error_json("map_id not found: " + *map_id)};
    }

    std::uintmax_t entries_deleted = 0;
    try {
      std::error_code ec;
      entries_deleted = fs::remove_all(manifest->root, ec);
      if (ec) {
        return {500, "application/json", error_json("failed to delete map asset: " + manifest->root.string())};
      }
      if (manifest->active) {
        auto remaining = map_catalog_->read_floor_map_manifests(manifest->building_id, manifest->floor_id, false);
        if (!remaining.empty()) {
          activate_map_manifest(remaining.front());
        } else {
          clear_fixed_floor_entries(manifest->building_id, manifest->floor_id);
        }
      }
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"deleted\":" << (entries_deleted > 0U ? "true" : "false") << ","
             << "\"map_id\":" << json_string(manifest->map_id) << ","
             << "\"display_name\":" << json_string(manifest->display_name) << ","
             << "\"building_id\":" << json_string(manifest->building_id) << ","
             << "\"floor_id\":" << json_string(manifest->floor_id) << ","
             << "\"active_deleted\":" << (manifest->active ? "true" : "false") << ","
             << "\"entries_deleted\":" << entries_deleted << "}";
    return {200, "application/json", response.str()};
  }

  struct ManifestLookupResult
  {
    bool ok{false};
    MapManifest manifest;
    HttpResponse error{400, "application/json", "{}"};
  };

  std::optional<std::string> query_value(const HttpRequest & request, const std::string & key) const
  {
    const auto it = request.query.find(key);
    if (it == request.query.end() || it->second.empty()) {
      return std::nullopt;
    }
    return it->second;
  }

  ManifestLookupResult resolve_map_manifest_from_query(const HttpRequest & request)
  {
    const auto requested_building_id = query_value(request, "building_id");
    const auto requested_floor_id = query_value(request, "floor_id");
    const auto requested_map_id = query_value(request, "map_id");
    const auto requested_map_name = query_value(request, "map_name").value_or(
      query_value(request, "display_name").value_or(""));

    ManifestLookupResult result;
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      result.error = {400, "application/json", error_json("valid building_id is required")};
      return result;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      result.error = {400, "application/json", error_json("valid floor_id is required")};
      return result;
    }

    std::optional<MapManifest> manifest;
    if (requested_map_id) {
      if (!safe_asset_id(*requested_map_id)) {
        result.error = {400, "application/json", error_json("valid map_id is required")};
        return result;
      }
      manifest = map_catalog_->find_map_by_id(*requested_map_id);
      if (!manifest) {
        result.error = {404, "application/json", error_json("map_id not found: " + *requested_map_id)};
        return result;
      }
      if (requested_building_id && *requested_building_id != manifest->building_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return result;
      }
      if (requested_floor_id && *requested_floor_id != manifest->floor_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested floor")};
        return result;
      }
    } else {
      if (!requested_building_id) {
        result.error = {400, "application/json", error_json("valid building_id is required")};
        return result;
      }
      if (!requested_floor_id) {
        result.error = {400, "application/json", error_json("valid floor_id is required")};
        return result;
      }
      if (!requested_map_name.empty()) {
        if (!valid_display_map_name(requested_map_name)) {
          result.error = {400, "application/json", error_json("valid map_name is required")};
          return result;
        }
        std::string error;
        manifest = map_catalog_->find_floor_map_by_name(*requested_building_id, *requested_floor_id, requested_map_name, error);
        if (!manifest) {
          result.error = {
            error.empty() ? 404 : 400,
            "application/json",
            error_json(error.empty() ? "map_name not found: " + requested_map_name : error)
          };
          return result;
        }
      } else {
        manifest = map_catalog_->active_floor_map(*requested_building_id, *requested_floor_id);
        if (!manifest) {
          result.error = {
            404,
            "application/json",
            error_json("active map not found for floor: " + *requested_building_id + "/" + *requested_floor_id)
          };
          return result;
        }
      }
    }

    result.ok = true;
    result.manifest = *manifest;
    return result;
  }

  HttpResponse handle_get_keepout_filter(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }
    try {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"filter\":\"keepout\","
               << "\"keepout\":" << keepout_filter_json(lookup.manifest) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter_json(lookup.manifest) << "}"
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_semantic_layer(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    try {
      const auto poses = read_floor_poses(lookup.manifest.poses_yaml);
      const auto keepout_filter = keepout_filter_json(lookup.manifest);
      const auto keepout_payload = keepout_semantic_payload_json(
        read_optional_text_file(keepout_semantic_json_path(lookup.manifest)));
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"schema\":\"njrh.semantic_layer.v1\","
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
               << "\"poses\":" << poses_json_array(poses) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter << "},"
               << "\"keepout_filter\":" << keepout_filter << ","
               << "\"keepout\":" << keepout_payload
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_poses(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    std::vector<StoredPose> poses;
    try {
      poses = read_floor_poses(lookup.manifest.poses_yaml);
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << std::fixed << std::setprecision(6);
    response << "{\"ok\":true,"
             << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
             << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
             << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
             << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
             << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
             << "\"poses\":" << poses_json_array(poses) << "}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_saved_mapping_2d_map_png(const HttpRequest & request)
  {
    const auto png_path = resolve_mapping_2d_png(
      request, fs::path(runtime_maps_dir_), fs::path(maps_root_));
    if (!png_path) {
      return {
        404,
        "application/json",
        error_json("no saved 2D PNG map is available; save a slam_toolbox 2D map first")
      };
    }

    try {
      return {200, "image/png", read_binary_file(*png_path)};
    } catch (const std::exception &) {
      return {404, "application/json", error_json("failed to open 2D PNG map: " + png_path->string())};
    }
  }

  nav_msgs::msg::OccupancyGrid latest_mapping_map_for_save(double & age_sec)
  {
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    if (!have_live_map_) {
      throw std::runtime_error("no live slam_toolbox /map has been received; start 2D mapping before saving");
    }
    age_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
    return latest_live_map_;
  }

  HttpResponse handle_save_mapping_2d(const std::string & body)
  {
    const auto map_name = json_string_value(body, "map_name");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    if (!map_name || !valid_display_map_name(*map_name)) {
      return {400, "application/json", error_json("valid map_name is required")};
    }
    if (!building_id || !safe_asset_id(*building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (!floor_id || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }

    double map_age_sec = 0.0;
    nav_msgs::msg::OccupancyGrid map;
    bool mapping_was_active = false;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      mapping_2d_process_running_locked();
      mapping_was_active = mapping_2d_active_;
    }
    try {
      map = latest_mapping_map_for_save(map_age_sec);
    } catch (const std::exception & exc) {
      return {404, "application/json", error_json(exc.what())};
    }

    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U || map.data.size() != static_cast<std::size_t>(width) * height) {
      return {503, "application/json", error_json("live slam_toolbox /map has invalid dimensions")};
    }
    set_mapping_runtime_state(true, "saving", "saving live 2D mapping assets");
    clear_runtime_map_context();

    const auto pixels = occupancy_grid_to_image_pixels(map);
    const auto png = encode_grayscale_png(width, height, pixels);
    if (png.empty()) {
      set_mapping_runtime_state(true, "running", "map save failed: PNG encode failed", false);
      return {500, "application/json", error_json("failed to encode live slam_toolbox map as PNG")};
    }

    auto manifest = map_catalog_->make_new_manifest(*building_id, *floor_id, *map_name);
    manifest.active = false;

    const fs::path runtime_dir(runtime_maps_dir_);
    const fs::path runtime_base = runtime_dir / manifest.safe_map_name;
    const fs::path runtime_yaml = runtime_base.string() + ".yaml";
    const fs::path runtime_pgm = runtime_base.string() + ".pgm";
    const fs::path runtime_png = runtime_base.string() + ".png";
    const fs::path runtime_localizer_yaml = runtime_dir / (manifest.safe_map_name + ".localizer.yaml");
    const fs::path runtime_localizer_png = runtime_dir / (manifest.safe_map_name + ".localizer.png");

    try {
      write_pgm_file(runtime_pgm, width, height, pixels);
      write_binary_file(runtime_png, png);
      write_text_file(runtime_yaml, map_yaml_text(runtime_pgm.filename().string(), map));
      write_binary_file(runtime_localizer_png, png);
      write_text_file(runtime_localizer_yaml, map_yaml_text(runtime_localizer_png.filename().string(), map));

      write_pgm_file(manifest.nav_map_pgm, width, height, pixels);
      write_text_file(
        manifest.nav_map_yaml, map_yaml_text(manifest.nav_map_pgm.filename().string(), map));
      write_binary_file(manifest.localizer_map_png, png);
      write_text_file(
        manifest.localizer_params_yaml,
        map_yaml_text(manifest.localizer_map_png.filename().string(), map));
      write_neutral_filter_assets(manifest.root / "filters", map);
      if (!fs::exists(manifest.poses_yaml)) {
        write_text_file(manifest.poses_yaml, "poses: []\n");
      }
      write_asset_report(manifest, map);
      write_map_manifest(manifest);
    } catch (const std::exception & exc) {
      set_mapping_runtime_state(true, "running", std::string("map save failed: ") + exc.what(), false);
      return {500, "application/json", error_json(exc.what())};
    }

    std::size_t stopped_groups = 0U;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      stopped_groups = terminate_mapping_2d_process_groups_locked();
    }
    set_mapping_live_map_cache_active(false);
    clear_live_map_cache();
    set_mapping_runtime_state(false, "stopped", "2D map saved and mapping chain stopped");

    std::ostringstream response;
    response << std::fixed << std::setprecision(3);
    response << "{\"ok\":true,"
             << "\"mapping_active\":false,"
             << "\"mapping_was_active\":" << (mapping_was_active ? "true" : "false") << ","
             << "\"stopped\":" << (stopped_groups > 0U ? "true" : "false") << ","
             << "\"stopped_groups\":" << stopped_groups << ","
             << "\"map_age_sec\":" << map_age_sec << ","
             << "\"map_id\":" << json_string(manifest.map_id) << ","
             << "\"display_name\":" << json_string(manifest.display_name) << ","
             << "\"map_name\":" << json_string(manifest.display_name) << ","
             << "\"safe_map_name\":" << json_string(manifest.safe_map_name) << ","
             << "\"building_id\":" << json_string(*building_id) << ","
             << "\"floor_id\":" << json_string(*floor_id) << ","
             << "\"active\":false,"
             << "\"selected_for_navigation\":false,"
             << "\"requires_manual_navigation_selection\":true,"
             << "\"runtime_map\":{"
             << "\"yaml\":" << json_string(runtime_yaml.string()) << ","
             << "\"pgm\":" << json_string(runtime_pgm.string()) << ","
             << "\"png\":" << json_string(runtime_png.string()) << ","
             << "\"localizer_yaml\":" << json_string(runtime_localizer_yaml.string()) << ","
             << "\"localizer_png\":" << json_string(runtime_localizer_png.string()) << "},"
             << "\"floor_assets\":{"
             << "\"root\":" << json_string(manifest.root.string()) << ","
             << "\"current_root\":" << json_string(map_catalog_->floor_current_root_path(*building_id, *floor_id).string()) << ","
             << "\"manifest_json\":" << json_string(manifest.manifest_json.string()) << ","
             << "\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string()) << ","
             << "\"nav_map_pgm\":" << json_string(manifest.nav_map_pgm.string()) << ","
             << "\"localizer_map_png\":" << json_string(manifest.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":" << json_string(manifest.localizer_params_yaml.string()) << ","
             << "\"asset_report_json\":" << json_string(manifest.asset_report_json.string()) << "}}";
    return {200, "application/json", response.str()};
  }

  bool mapping_2d_process_running_locked()
  {
    if (mapping_2d_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(mapping_2d_pid_, &status, WNOHANG);
    if (wait_result == mapping_2d_pid_) {
      mapping_2d_pid_ = -1;
      mapping_2d_active_ = false;
      set_mapping_runtime_state(false, "stopped", "2D mapping process exited");
      return false;
    }
    if (::kill(mapping_2d_pid_, 0) == 0) {
      return true;
    }
    if (errno == ESRCH) {
      mapping_2d_pid_ = -1;
      mapping_2d_active_ = false;
      set_mapping_runtime_state(false, "stopped", "2D mapping process is not alive");
      return false;
    }
    return true;
  }

  bool recover_mapping_2d_process_locked()
  {
    if (mapping_2d_process_running_locked()) {
      return true;
    }
    if (mapping_2d_pid_ <= 0 && !discover_mapping_2d_process_groups().empty()) {
      if (!mapping_2d_active_ || mapping_2d_started_at_ == std::chrono::steady_clock::time_point{}) {
        mapping_2d_started_at_ = std::chrono::steady_clock::now();
      }
      mapping_2d_active_ = true;
      return true;
    }
    return false;
  }

  bool is_mapping_2d_process_command(const std::string & cmdline) const
  {
    static constexpr std::array<const char *, 4> kMapping2dProcessPatterns{
      "run_projected_map.sh",
      "jt128_slam_toolbox_mapping.launch.py",
      "jt128_2d_mapping.launch.py",
      "run_jt128_2d_mapping.sh"
    };
    return std::any_of(
      kMapping2dProcessPatterns.begin(), kMapping2dProcessPatterns.end(),
      [&cmdline](const char * pattern) {
        return cmdline.find(pattern) != std::string::npos;
      });
  }

  bool is_private_slam2d_fastlio_process(const std::string & cmdline, const std::string & environ) const
  {
    static constexpr std::array<const char *, 3> kFastlioProcessPatterns{
      "ros2 run fast_lio fastlio_mapping",
      "fast_lio/lib/fast_lio/fastlio_mapping",
      "fastlio_mapping --ros-args"
    };
    const bool is_fastlio_process = std::any_of(
      kFastlioProcessPatterns.begin(), kFastlioProcessPatterns.end(),
      [&cmdline](const char * pattern) {
        return cmdline.find(pattern) != std::string::npos;
      });
    return is_fastlio_process &&
           environ.find("NJRH_SLAM2D_PRIVATE_FASTLIO=1") != std::string::npos;
  }

  bool is_mapping_2d_residual_process_command(
    const std::string & cmdline, const std::string & environ) const
  {
    static constexpr std::array<const char *, 4> kMapping2dResidualProcessPatterns{
      "slam_toolbox",
      "fastlio_mapping_odom_bridge.py",
      "/mapping/fastlio_odometry",
      "/tf_slam2d"
    };
    if (is_private_slam2d_fastlio_process(cmdline, environ)) {
      return true;
    }
    return std::any_of(
      kMapping2dResidualProcessPatterns.begin(), kMapping2dResidualProcessPatterns.end(),
      [&cmdline](const char * pattern) {
        return cmdline.find(pattern) != std::string::npos;
      });
  }

  std::set<pid_t> discover_mapping_2d_process_groups() const
  {
    std::set<pid_t> groups;
    const pid_t self_pid = ::getpid();
    if (!fs::exists("/proc")) {
      return groups;
    }
    for (const auto & entry : fs::directory_iterator("/proc")) {
      if (!entry.is_directory() || !is_pid_directory(entry.path())) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
      if (pid <= 1 || pid == self_pid) {
        continue;
      }
      const auto cmdline = read_proc_cmdline(pid);
      if (!is_mapping_2d_process_command(cmdline)) {
        continue;
      }
      const pid_t pgid = ::getpgid(pid);
      groups.insert(pgid > 0 ? pgid : pid);
    }
    return groups;
  }

  std::set<pid_t> discover_mapping_2d_residual_processes() const
  {
    std::set<pid_t> pids;
    const pid_t self_pid = ::getpid();
    if (!fs::exists("/proc")) {
      return pids;
    }
    for (const auto & entry : fs::directory_iterator("/proc")) {
      if (!entry.is_directory() || !is_pid_directory(entry.path())) {
        continue;
      }
      const pid_t pid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
      if (pid <= 1 || pid == self_pid) {
        continue;
      }
      const auto cmdline = read_proc_cmdline(pid);
      const auto environ = read_proc_environ(pid);
      if (is_mapping_2d_residual_process_command(cmdline, environ)) {
        pids.insert(pid);
      }
    }
    return pids;
  }

  std::size_t terminate_mapping_2d_residual_processes() const
  {
    std::set<pid_t> pids = discover_mapping_2d_residual_processes();
    if (pids.empty()) {
      return 0U;
    }
    const std::size_t requested_pids = pids.size();
    for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
      for (const auto pid : pids) {
        ::kill(pid, signal);
      }
      std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);
      for (auto it = pids.begin(); it != pids.end();) {
        if (!process_pid_is_live(*it)) {
          it = pids.erase(it);
        } else {
          ++it;
        }
      }
      if (pids.empty()) {
        break;
      }
    }
    return requested_pids;
  }

  bool is_safe_mapping_lidar_rps_xps_path(const std::string & path) const
  {
    const bool is_rps = path.size() >= 9U && path.compare(path.size() - 9U, 9U, "/rps_cpus") == 0;
    const bool is_xps = path.size() >= 9U && path.compare(path.size() - 9U, 9U, "/xps_cpus") == 0;
    return starts_with(path, "/sys/class/net/") &&
           path.find("/queues/") != std::string::npos &&
           path.find("..") == std::string::npos &&
           (is_rps || is_xps);
  }

  bool is_safe_mapping_lidar_rps_xps_value(const std::string & value) const
  {
    return !value.empty() &&
           std::all_of(value.begin(), value.end(), [](const unsigned char ch) {
             return std::isxdigit(ch) || ch == ',';
           });
  }

  std::size_t restore_mapping_lidar_rps_xps_state() const
  {
    if (mapping_lidar_rps_xps_state_dir_.empty()) {
      return 0U;
    }
    const fs::path state_dir(mapping_lidar_rps_xps_state_dir_);
    const fs::path table_path = state_dir / "rps_xps.tsv";
    if (!fs::exists(table_path) || !fs::is_regular_file(table_path)) {
      return 0U;
    }

    std::size_t restored = 0U;
    std::size_t failures = 0U;
    std::string table_text;
    try {
      table_text = read_text_file(table_path);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(),
        "failed to read mapping LiDAR RPS/XPS state table %s: %s",
        table_path.string().c_str(),
        exc.what());
      return 0U;
    }
    std::istringstream input(table_text);
    std::string line;
    while (std::getline(input, line)) {
      const auto tab = line.find('\t');
      if (tab == std::string::npos) {
        continue;
      }
      const std::string path = trim(line.substr(0, tab));
      const std::string value = trim(line.substr(tab + 1));
      if (!is_safe_mapping_lidar_rps_xps_path(path) || !is_safe_mapping_lidar_rps_xps_value(value)) {
        ++failures;
        RCLCPP_WARN(
          get_logger(),
          "skipping unsafe mapping LiDAR RPS/XPS restore entry: %s",
          path.c_str());
        continue;
      }

      std::ofstream file(path);
      if (!file) {
        ++failures;
        RCLCPP_WARN(
          get_logger(),
          "failed to open mapping LiDAR RPS/XPS restore target: %s",
          path.c_str());
        continue;
      }
      file << value << '\n';
      if (!file) {
        ++failures;
        RCLCPP_WARN(
          get_logger(),
          "failed to write mapping LiDAR RPS/XPS restore target: %s",
          path.c_str());
        continue;
      }
      ++restored;
    }

    if (failures == 0U) {
      std::error_code ec;
      fs::remove_all(state_dir, ec);
      if (ec) {
        RCLCPP_WARN(
          get_logger(),
          "failed to remove mapping LiDAR RPS/XPS state dir %s: %s",
          state_dir.string().c_str(),
          ec.message().c_str());
      }
    }
    if (restored > 0U) {
      RCLCPP_INFO(
        get_logger(),
        "restored %zu mapping LiDAR RPS/XPS queue settings",
        restored);
    }
    return restored;
  }

  std::size_t terminate_mapping_2d_process_groups_locked()
  {
    std::set<pid_t> groups = discover_mapping_2d_process_groups();
    if (mapping_2d_pid_ > 0) {
      const pid_t pgid = ::getpgid(mapping_2d_pid_);
      groups.insert(pgid > 0 ? pgid : mapping_2d_pid_);
    }

    const std::size_t requested_groups = groups.size();
    if (!groups.empty()) {
      for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
        for (const auto pgid : groups) {
          signal_process_group(pgid, signal);
        }
        std::this_thread::sleep_for(signal == SIGKILL ? 200ms : 800ms);

        for (auto it = groups.begin(); it != groups.end();) {
          if (!process_group_has_live_process(*it)) {
            it = groups.erase(it);
          } else {
            ++it;
          }
        }
        if (groups.empty()) {
          break;
        }
      }
    }

    if (mapping_2d_pid_ > 0) {
      int status = 0;
      while (::waitpid(mapping_2d_pid_, &status, WNOHANG) == mapping_2d_pid_) {
      }
    }
    mapping_2d_pid_ = -1;
    mapping_2d_active_ = false;
    set_mapping_runtime_state(false, "stopped", "2D mapping runtime stopped");
    const std::size_t requested_residuals = terminate_mapping_2d_residual_processes();
    restore_mapping_lidar_rps_xps_state();
    return requested_groups + requested_residuals;
  }

  HttpResponse handle_start_mapping_2d()
  {
    if (mapping_2d_start_command_.empty() || !fs::exists(mapping_2d_start_command_)) {
      return {
        503,
        "application/json",
        error_json("2D slam_toolbox start command is not available: " + mapping_2d_start_command_)
      };
    }

    const auto runtime = runtime_mode_snapshot();
    if (runtime.docking_active) {
      return {409, "application/json", error_json("cannot start 2D mapping while docking is active")};
    }
    if (runtime.navigation_active) {
      return {
        409,
        "application/json",
        error_json("cannot start 2D mapping while navigation runtime is active; stop navigation runtime first")
      };
    }
    clear_runtime_map_context();

    std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
    if (mapping_2d_process_running_locked()) {
      set_mapping_live_map_cache_active(true);
      set_mapping_runtime_state(true, "running", "2D mapping chain is already running");
      return {
        202,
        "application/json",
        "{\"ok\":true,\"state\":\"already_running\",\"map_topic\":" + json_string(mapping_2d_live_map_topic_) +
        ",\"map_endpoint\":\"/api/v1/mapping/2d/map\"}"
      };
    }

    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      have_live_map_ = false;
      latest_live_map_ = nav_msgs::msg::OccupancyGrid{};
      latest_live_map_received_at_ = {};
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      return {500, "application/json", error_json("failed to fork 2D slam_toolbox mapping process")};
    }
    if (pid == 0) {
      prepare_child_process(mapping_2d_log_file_);
      ::execl("/bin/bash", "bash", mapping_2d_start_command_.c_str(), static_cast<char *>(nullptr));
      ::_exit(127);
    }

    mapping_2d_pid_ = pid;
    mapping_2d_active_ = true;
    mapping_2d_started_at_ = std::chrono::steady_clock::now();
    set_mapping_live_map_cache_active(true);
    set_mapping_runtime_state(true, "starting", "2D mapping chain start accepted");

    std::ostringstream body;
    body << "{\"ok\":true,\"state\":\"starting\","
         << "\"pid\":" << pid << ","
         << "\"map_topic\":" << json_string(mapping_2d_live_map_topic_) << ","
         << "\"map_endpoint\":\"/api/v1/mapping/2d/map\","
         << "\"log_file\":" << json_string(mapping_2d_log_file_) << "}";
    return {202, "application/json", body.str()};
  }

  HttpResponse handle_stop_mapping_2d()
  {
    std::size_t requested_groups = 0U;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      requested_groups = terminate_mapping_2d_process_groups_locked();
    }
    set_mapping_live_map_cache_active(false);
    clear_live_map_cache();
    set_mapping_runtime_state(false, "stopped", "2D mapping chain stopped");

    std::ostringstream body;
    body << "{\"ok\":true,\"mapping_active\":false,\"stopped\":"
         << (requested_groups > 0U ? "true" : "false")
         << ",\"stopped_groups\":" << requested_groups << "}";
    return {200, "application/json", body.str()};
  }

  bool navigation_resume_process_running_locked()
  {
    if (navigation_resume_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(navigation_resume_pid_, &status, WNOHANG);
    if (wait_result == navigation_resume_pid_) {
      navigation_resume_pid_ = -1;
      return false;
    }
    if (::kill(navigation_resume_pid_, 0) == 0) {
      return true;
    }
    if (errno == ESRCH) {
      navigation_resume_pid_ = -1;
      return false;
    }
    return true;
  }

  void refresh_mapping_2d_runtime_state()
  {
    auto runtime = runtime_mode_snapshot();
    if (!runtime.mapping_active) {
      bool recovered = false;
      {
        std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
        recovered = recover_mapping_2d_process_locked();
      }
      if (!recovered) {
        return;
      }
      set_mapping_runtime_state(true, "starting", "2D mapping process discovered");
      runtime = runtime_mode_snapshot();
    }
    if (runtime.mapping_state == "saving" || runtime.mapping_state == "stopping") {
      return;
    }

    bool process_running = false;
    std::chrono::steady_clock::time_point started_at;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      process_running = recover_mapping_2d_process_locked();
      started_at = mapping_2d_started_at_;
    }
    if (!process_running) {
      set_mapping_live_map_cache_active(false);
      return;
    }
    set_mapping_live_map_cache_active(true);

    bool live_map_ready = false;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (have_live_map_ && latest_live_map_received_at_ >= started_at) {
        const auto age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
        const auto expected_size =
          static_cast<std::size_t>(latest_live_map_.info.width) * latest_live_map_.info.height;
        live_map_ready =
          age <= mapping_2d_live_map_max_age_sec_ &&
          latest_live_map_.info.width > 0U &&
          latest_live_map_.info.height > 0U &&
          latest_live_map_.data.size() == expected_size;
      }
    }

    if (live_map_ready && runtime.mapping_state != "running") {
      set_mapping_runtime_state(true, "running", "live 2D mapping map ready");
    }
  }

  void refresh_navigation_resume_runtime_state(const bool probe_lifecycle = false)
  {
    bool process_exited = false;
    bool process_running = false;
    {
      std::lock_guard<std::mutex> process_lock(navigation_process_mutex_);
      const bool had_process = navigation_resume_pid_ > 0;
      process_running = navigation_resume_process_running_locked();
      process_exited = had_process && !process_running;
    }
    if (process_exited) {
      set_navigation_runtime_state(
        false,
        "failed",
        "navigation runtime process exited before ready; check " + navigation_resume_log_file_,
        false);
      return;
    }
    const auto context = read_runtime_map_context();
    if (context && !context->confirmed && context->state == "failed") {
      set_navigation_runtime_state(
        false,
        "failed",
        context->message.empty() ? "navigation runtime failed before ready" : context->message,
        false);
      return;
    }
    const bool context_ready = context && context->confirmed && context->state == "ready";
    const bool navigate_action_ready =
      navigate_to_pose_client_ && navigate_to_pose_client_->action_server_is_ready();
    if (context_ready && (process_running || navigate_action_ready)) {
      if (probe_lifecycle) {
        const auto lifecycle = navigation_lifecycle_snapshot();
        if (!lifecycle.active) {
          set_navigation_runtime_state(true, "degraded", lifecycle.detail, false);
          return;
        }
      }
      set_navigation_runtime_state(
        true,
        "running",
        context->message.empty() ? "navigation runtime ready" : context->message);
    }
  }

  void terminate_navigation_resume_process_locked()
  {
    if (!navigation_resume_process_running_locked()) {
      navigation_resume_pid_ = -1;
      return;
    }
    const pid_t pgid = ::getpgid(navigation_resume_pid_);
    if (pgid > 0) {
      signal_process_group(pgid, SIGINT);
      std::this_thread::sleep_for(800ms);
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGTERM);
        std::this_thread::sleep_for(800ms);
      }
      if (process_group_has_live_process(pgid)) {
        signal_process_group(pgid, SIGKILL);
      }
    } else {
      ::kill(navigation_resume_pid_, SIGINT);
    }
    int status = 0;
    while (::waitpid(navigation_resume_pid_, &status, WNOHANG) == navigation_resume_pid_) {
    }
    navigation_resume_pid_ = -1;
  }

  bool runtime_context_matches_resume_request(
    const RuntimeMapContext & context,
    const std::string & building_id,
    const std::string & floor_id,
    const MapManifest & selected_map) const
  {
    return context.confirmed && context.state == "ready" &&
           context.map_id == selected_map.map_id &&
           context.building_id == building_id &&
           context.floor_id == floor_id;
  }

  HttpResponse handle_resume_floor_navigation(
    const std::string & building_id,
    const std::string & floor_id,
    const std::optional<MapManifest> & selected_map = std::nullopt)
  {
    FloorAssetPaths assets;
    std::string error;
    if (!resolve_floor_asset_paths(*map_catalog_, building_id, floor_id, assets, error)) {
      return {404, "application/json", error_json(error)};
    }
    if (navigation_resume_command_.empty() || !fs::exists(navigation_resume_command_)) {
      return {
        503,
        "application/json",
        error_json("navigation resume command is not available: " + navigation_resume_command_)
      };
    }

    std::lock_guard<std::mutex> process_lock(navigation_process_mutex_);
    const bool existing_resume_process_running = navigation_resume_process_running_locked();
    const auto existing_context = selected_map ? read_runtime_map_context() : std::nullopt;
    const bool navigate_action_ready =
      navigate_to_pose_client_ && navigate_to_pose_client_->action_server_is_ready();
    const auto lifecycle = navigation_lifecycle_snapshot();
    if (selected_map && existing_context &&
      runtime_context_matches_resume_request(*existing_context, building_id, floor_id, *selected_map))
    {
      if (!existing_resume_process_running && !navigate_action_ready && !lifecycle.active) {
        terminate_navigation_resume_process_locked();
      } else {
        try {
          write_last_navigation_map_selection(*selected_map, "navigation_runtime_reused");
        } catch (const std::exception & exc) {
          return {500, "application/json", error_json(exc.what())};
        }
        set_navigation_runtime_state(
          true,
          lifecycle.active ? "running" : "degraded",
          lifecycle.active ? existing_context->message : lifecycle.detail,
          lifecycle.active);

        std::ostringstream body;
        body << "{\"ok\":true,"
             << "\"state\":\"navigation_runtime_reused\","
             << "\"pid\":" << navigation_resume_pid_ << ","
             << "\"building_id\":" << json_string(building_id) << ","
             << "\"floor_id\":" << json_string(floor_id) << ","
             << "\"map_id\":" << json_string(selected_map->map_id) << ","
             << "\"display_name\":" << json_string(selected_map->display_name) << ","
             << "\"resume_navigation\":true,"
             << "\"reused\":true,"
             << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
             << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":" << json_string(assets.localizer_params_yaml.string()) << ","
             << "\"log_file\":" << json_string(navigation_resume_log_file_) << "}";
        return {200, "application/json", body.str()};
      }
    }

    terminate_navigation_resume_process_locked();
    if (selected_map) {
      try {
        write_last_navigation_map_selection(*selected_map, "navigation_resume_start");
      } catch (const std::exception & exc) {
        return {500, "application/json", error_json(exc.what())};
      }
      write_runtime_map_context(
        *selected_map, "starting", false, "navigation runtime start accepted");
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      return {500, "application/json", error_json("failed to fork navigation resume process")};
    }
    if (pid == 0) {
      prepare_child_process(navigation_resume_log_file_);
      ::setenv("NJRH_AMCL_RUNTIME_STATUS_FILE", amcl_runtime_status_file_.c_str(), 1);
      if (selected_map) {
        ::setenv("NJRH_RUNTIME_MAP_CONTEXT_FILE", runtime_map_context_file_.c_str(), 1);
        ::setenv("NJRH_MAP_ID", selected_map->map_id.c_str(), 1);
        ::setenv("NJRH_MAP_DISPLAY_NAME", selected_map->display_name.c_str(), 1);
        ::setenv("NJRH_MAP_CONTEXT_BUILDING_ID", selected_map->building_id.c_str(), 1);
        ::setenv("NJRH_MAP_CONTEXT_FLOOR_ID", selected_map->floor_id.c_str(), 1);
      }
      ::execl(
        "/bin/bash",
        "bash",
        navigation_resume_command_.c_str(),
        building_id.c_str(),
        floor_id.c_str(),
        static_cast<char *>(nullptr));
      ::_exit(127);
    }

    navigation_resume_pid_ = pid;
    set_navigation_runtime_state(true, "starting", "navigation runtime start accepted");

    std::ostringstream body;
    body << "{\"ok\":true,"
         << "\"state\":\"navigation_resume_starting\","
         << "\"pid\":" << pid << ","
         << "\"building_id\":" << json_string(building_id) << ","
         << "\"floor_id\":" << json_string(floor_id) << ","
         << "\"map_id\":" << json_string(selected_map ? selected_map->map_id : "") << ","
         << "\"display_name\":" << json_string(selected_map ? selected_map->display_name : "") << ","
         << "\"resume_navigation\":true,"
         << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
         << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
         << "\"localizer_params_yaml\":" << json_string(assets.localizer_params_yaml.string()) << ","
         << "\"log_file\":" << json_string(navigation_resume_log_file_) << "}";
    return {202, "application/json", body.str()};
  }

  HttpResponse handle_live_mapping_2d_map_png()
  {
    if (!subscription_manager_ || !subscription_manager_->active("live_map")) {
      return {
        409,
        "application/json",
        error_json("live_map resource is not acquired; call POST /api/v1/subscriptions/acquire first")
      };
    }

    std::chrono::steady_clock::time_point started_at;
    {
      std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
      recover_mapping_2d_process_locked();
      if (!mapping_2d_active_) {
        return {
          409,
          "application/json",
          error_json("2D slam_toolbox mapping is not active; call POST /api/v1/mapping/2d/start first")
        };
      }
      started_at = mapping_2d_started_at_;
    }

    nav_msgs::msg::OccupancyGrid map;
    std::chrono::steady_clock::time_point received_at;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (!have_live_map_ || latest_live_map_received_at_ < started_at) {
        return {
          404,
          "application/json",
          error_json("waiting for live slam_toolbox /map data")
        };
      }
      map = latest_live_map_;
      received_at = latest_live_map_received_at_;
    }

    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - received_at).count();
    if (age > mapping_2d_live_map_max_age_sec_) {
      return {
        503,
        "application/json",
        error_json("live slam_toolbox /map is stale")
      };
    }

    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U || map.data.size() != static_cast<std::size_t>(width) * height) {
      return {503, "application/json", error_json("live slam_toolbox /map has invalid dimensions")};
    }

    const auto pixels = occupancy_grid_to_image_pixels(map);

    const auto png = encode_grayscale_png(width, height, pixels);
    if (png.empty()) {
      return {500, "application/json", error_json("failed to encode live slam_toolbox map as PNG")};
    }
    set_mapping_runtime_state(true, "running", "live 2D mapping map ready");
    return {200, "image/png", png};
  }

  HttpResponse handle_mapping_2d_map_png(const HttpRequest & request)
  {
    const auto source_it = request.query.find("source");
    const bool explicit_saved =
      request.query.find("name") != request.query.end() ||
      (source_it != request.query.end() && source_it->second == "saved");
    if (explicit_saved) {
      return handle_saved_mapping_2d_map_png(request);
    }
    return handle_live_mapping_2d_map_png();
  }

  HttpResponse handle_save_pose(
    const std::string & body,
    const std::optional<std::string> & forced_pose_id = std::nullopt)
  {
    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto body_pose_id = json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      return {400, "application/json", error_json("pose_id in body does not match path pose_id")};
    }

    const auto x = json_number_value(body, "x").value_or(
      json_nested_number_value(body, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(body, "y").value_or(
      json_nested_number_value(body, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(body, "yaw").value_or(
      json_number_value(body, "theta").value_or(
        json_nested_number_value(body, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return {400, "application/json", error_json("finite x, y, and yaw are required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = map_catalog_->find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = map_catalog_->active_floor_map(resolved_building_id, resolved_floor_id);
    }

    const auto floor_root = map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(body, "name").value_or(pose_id);
    pose.type = json_string_value(body, "type").value_or("delivery_point");
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);

    const auto path = manifest ? manifest->poses_yaml :
      poses_yaml_path(*map_catalog_, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      write_floor_poses(path, poses);
      if (manifest && manifest->active) {
        copy_file_if_exists(path, map_catalog_->floor_current_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
        copy_file_if_exists(path, map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
      }

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"pose\":{\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & ex) {
      return {500, "application/json", error_json(ex.what())};
    }
  }

  HttpResponse handle_save_current_pose(const std::string & body)
  {
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }
    if (!manifest) {
      return {
        404,
        "application/json",
        error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
      };
    }

    const auto floor_root = map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto body_pose_id = json_string_value(body, "pose_id").value_or(
      json_string_value(body, "id").value_or(""));
    const std::string pose_type = json_string_value(body, "type").value_or("delivery_point");
    const std::string pose_name = json_string_value(body, "name").value_or(
      body_pose_id.empty() ? pose_type : body_pose_id);
    const std::string pose_id =
      body_pose_id.empty() ? generate_current_pose_id(pose_type, pose_name) : body_pose_id;
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }

    std::string pose_error;
    const auto current_pose = wait_for_current_robot_pose(true, pose_error);
    if (!current_pose.available) {
      (void)pose_error;
      return {503, "application/json", no_fresh_map_robot_pose_json(tf_map_frame_, tf_base_frame_)};
    }
    std::string context_error;
    bool blocked_by_pending_context = false;
    const auto current_context = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
    if (blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(tf_map_frame_, tf_base_frame_, context_error)};
    }
    if (current_context &&
      (current_context->map_id != manifest->map_id ||
      current_context->building_id != resolved_building_id ||
      current_context->floor_id != resolved_floor_id))
    {
      return {
        503,
        "application/json",
        no_fresh_map_robot_pose_json(
          tf_map_frame_,
          tf_base_frame_,
          "requested pose target does not match confirmed runtime map context: " +
          current_context->building_id + "/" + current_context->floor_id + "/" +
          current_context->map_id)
      };
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = pose_name.empty() ? pose_id : pose_name;
    pose.type = pose_type.empty() ? "delivery_point" : pose_type;
    pose.x = current_pose.x;
    pose.y = current_pose.y;
    pose.yaw = current_pose.yaw;

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"source\":\"current_robot_pose\","
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"source_pose\":{"
               << "\"frame_id\":" << json_string(current_pose.frame_id) << ","
               << "\"child_frame_id\":" << json_string(current_pose.child_frame_id) << ","
               << "\"x\":" << current_pose.x << ","
               << "\"y\":" << current_pose.y << ","
               << "\"yaw\":" << current_pose.yaw << ","
               << "\"stamp\":" << current_pose.stamp_sec << ","
               << "\"age_sec\":" << current_pose.age_sec << "},"
               << "\"pose\":{"
               << "\"id\":" << json_string(pose.id) << ","
               << "\"name\":" << json_string(pose.name) << ","
               << "\"type\":" << json_string(pose.type) << ","
               << "\"x\":" << pose.x << ","
               << "\"y\":" << pose.y << ","
               << "\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  std::optional<std::string> request_value(const HttpRequest & request, const std::string & key) const
  {
    const auto query = query_value(request, key);
    if (query) {
      return query;
    }
    return json_string_value(request.body, key);
  }

  bool resolve_pose_target_manifest(
    const std::optional<std::string> & requested_building_id,
    const std::optional<std::string> & requested_floor_id,
    const std::optional<std::string> & map_id,
    std::optional<MapManifest> & manifest,
    std::string & resolved_building_id,
    std::string & resolved_floor_id,
    HttpResponse & error)
  {
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      error = {400, "application/json", error_json("valid map_id is required")};
      return false;
    }

    if (map_id && !map_id->empty()) {
      manifest = map_catalog_->find_map_by_id(*map_id);
      if (!manifest) {
        error = {404, "application/json", error_json("map_id not found: " + *map_id)};
        return false;
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return false;
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested floor")};
        return false;
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
      return true;
    }

    if (!requested_building_id) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (!requested_floor_id) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    resolved_building_id = *requested_building_id;
    resolved_floor_id = *requested_floor_id;
    manifest = map_catalog_->active_floor_map(resolved_building_id, resolved_floor_id);
    return true;
  }

  fs::path pose_target_path(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id) const
  {
    return manifest ? manifest->poses_yaml : poses_yaml_path(*map_catalog_, building_id, floor_id);
  }

  void sync_active_poses_if_needed(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id,
    const fs::path & path) const
  {
    if (!manifest || !manifest->active) {
      return;
    }
    copy_file_if_exists(path, map_catalog_->floor_current_root_path(building_id, floor_id) / "poses.yaml");
    copy_file_if_exists(path, map_catalog_->floor_root_path(building_id, floor_id) / "poses.yaml");
  }

  std::optional<StoredPose> parse_pose_payload(
    const std::string & payload,
    const std::optional<std::string> & forced_pose_id,
    std::string & error) const
  {
    const auto body_pose_id = json_string_value(payload, "pose_id").value_or(
      json_string_value(payload, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (!safe_pose_id(pose_id)) {
      error = "valid pose_id is required";
      return std::nullopt;
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      error = "pose_id in body does not match path pose_id";
      return std::nullopt;
    }

    const auto x = json_number_value(payload, "x").value_or(
      json_nested_number_value(payload, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(payload, "y").value_or(
      json_nested_number_value(payload, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(payload, "yaw").value_or(
      json_number_value(payload, "theta").value_or(
        json_nested_number_value(payload, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      error = "finite x, y, and yaw are required";
      return std::nullopt;
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(payload, "name").value_or(pose_id);
    pose.type = json_string_value(payload, "type").value_or("delivery_point");
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);
    return pose;
  }

  HttpResponse handle_delete_pose(const HttpRequest & request, const std::string & pose_id)
  {
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        request_value(request, "building_id"),
        request_value(request, "floor_id"),
        request_value(request, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      const auto before = poses.size();
      poses.erase(
        std::remove_if(poses.begin(), poses.end(), [&pose_id](const StoredPose & pose) {
          return pose.id == pose_id;
        }),
        poses.end());
      if (poses.size() == before) {
        return {404, "application/json", error_json("pose_id not found in poses.yaml: " + pose_id)};
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"deleted\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose_id) << ","
               << "\"remaining\":" << poses.size() << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_replace_poses_batch(const std::string & body)
  {
    if (body.find("\"poses\"") == std::string::npos) {
      return {400, "application/json", error_json("poses array is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    std::vector<StoredPose> poses;
    std::set<std::string> pose_ids;
    for (const auto & object : json_object_array_value(body, "poses")) {
      std::string parse_error;
      auto pose = parse_pose_payload(object, std::nullopt, parse_error);
      if (!pose) {
        return {400, "application/json", error_json(parse_error)};
      }
      if (!pose_ids.insert(pose->id).second) {
        return {400, "application/json", error_json("duplicate pose_id in batch: " + pose->id)};
      }
      poses.push_back(*pose);
    }

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"replaced\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"count\":" << poses.size() << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_save_keepout_filter(const std::string & body)
  {
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = map_catalog_->find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = map_catalog_->active_floor_map(resolved_building_id, resolved_floor_id);
      if (!manifest) {
        return {
          404,
          "application/json",
          error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
        };
      }
    }

    const auto path = keepout_semantic_json_path(*manifest);
    try {
      write_text_file(path, stripped_body + "\n");
      if (manifest->active) {
        copy_file_if_exists(
          path,
          map_catalog_->floor_current_root_path(resolved_building_id, resolved_floor_id) /
          "filters" / "keepout_semantic_layer.json");
        copy_file_if_exists(
          path,
          map_catalog_->floor_root_path(resolved_building_id, resolved_floor_id) /
          "filters" / "keepout_semantic_layer.json");
      }

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest->map_id) << ","
               << "\"display_name\":" << json_string(manifest->display_name) << ","
               << "\"map_name\":" << json_string(manifest->display_name) << ","
               << "\"active\":" << (manifest->active ? "true" : "false") << ","
               << "\"semantic_json_path\":" << json_string(path.string()) << ","
               << "\"keepout_mask_yaml\":" << json_string(manifest->keepout_mask_yaml.string()) << ","
               << "\"keepout_mask_pgm\":" << json_string(manifest->keepout_mask_pgm.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  static std::string query_string_value(
    const HttpRequest & request,
    const std::string & key,
    const std::string & default_value = "")
  {
    const auto it = request.query.find(key);
    if (it == request.query.end()) {
      return default_value;
    }
    return it->second;
  }

  static std::optional<double> query_number_value(const HttpRequest & request, const std::string & key)
  {
    const auto it = request.query.find(key);
    if (it == request.query.end() || it->second.empty()) {
      return std::nullopt;
    }
    try {
      std::size_t parsed = 0U;
      const double value = std::stod(it->second, &parsed);
      if (parsed != it->second.size()) {
        return std::nullopt;
      }
      return value;
    } catch (...) {
      return std::nullopt;
    }
  }

  HttpResponse handle_navigation_pre_goal_check(const HttpRequest & request)
  {
    const auto pose_id = query_string_value(
      request,
      "pose_id",
      query_string_value(request, "id", ""));
    auto building_id = query_string_value(request, "building_id", "");
    auto floor_id = query_string_value(request, "floor_id", "");
    const auto map_id = query_string_value(request, "map_id", "");
    const auto frame_id = normalized_frame_id(query_string_value(request, "frame_id", "map"));
    const bool by_pose_id = !pose_id.empty();
    const bool has_direct_pose_query =
      request.query.find("x") != request.query.end() ||
      request.query.find("y") != request.query.end() ||
      request.query.find("yaw") != request.query.end() ||
      request.query.find("theta") != request.query.end();

    const auto dock_check = pre_navigation_dock_check_snapshot();
    const auto lifecycle = navigation_lifecycle_snapshot();

    bool pose_requested = by_pose_id;
    bool pose_ok = false;
    std::string pose_status = by_pose_id ? "unresolved" : "not_requested";
    std::string pose_detail;
    std::string target_source;
    StoredPose target;

    if (by_pose_id) {
      pose_requested = true;
      if (!safe_pose_id(pose_id)) {
        pose_status = "invalid_pose_id";
        pose_detail = "valid pose_id is required";
      } else {
        std::optional<MapManifest> manifest;
        try {
          if (!map_id.empty()) {
            manifest = map_catalog_->find_map_by_id(map_id);
            if (!manifest) {
              pose_status = "map_not_found";
              pose_detail = "map_id not found: " + map_id;
            } else {
              building_id = manifest->building_id;
              floor_id = manifest->floor_id;
            }
          } else if (!safe_map_name(building_id) || !safe_map_name(floor_id)) {
            pose_status = "invalid_floor";
            pose_detail = "valid building_id and floor_id are required for pose_id navigation";
          } else {
            manifest = map_catalog_->active_floor_map(building_id, floor_id);
            if (!manifest) {
              pose_status = "active_map_not_found";
              pose_detail = "active map not found for floor: " + building_id + "/" + floor_id;
            }
          }

          if (manifest) {
            const auto pose = find_floor_catalog_pose(*map_catalog_, building_id, floor_id, pose_id);
            if (!pose) {
              pose_status = "pose_not_found";
              pose_detail = "pose_id not found in poses.yaml: " + pose_id;
            } else {
              target = *pose;
              pose_ok = true;
              pose_status = "resolved";
              pose_detail = "pose_id resolved from poses.yaml";
              target_source = "poses_yaml";
            }
          }
        } catch (const std::exception & exc) {
          pose_status = "resolve_exception";
          pose_detail = exc.what();
        }
      }
    } else {
      const auto x = query_number_value(request, "x");
      const auto y = query_number_value(request, "y");
      auto yaw = query_number_value(request, "yaw");
      if (!yaw) {
        yaw = query_number_value(request, "theta");
      }
      if (x || y || yaw) {
        pose_requested = true;
        if (frame_id != "map") {
          pose_status = "invalid_frame";
          pose_detail = "navigation goals must be in map frame";
        } else if (!x || !y || !yaw || !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*yaw)) {
          pose_status = "invalid_direct_pose";
          pose_detail = "finite x, y, and yaw query parameters are required for direct pose precheck";
        } else {
          target.id = "direct";
          target.name = "direct";
          target.type = "direct_goal";
          target.x = *x;
          target.y = *y;
          target.yaw = normalize_angle(*yaw);
          pose_ok = true;
          pose_status = "resolved";
          pose_detail = "direct map-frame pose is valid";
          target_source = "direct_pose";
        }
      }
    }

    std::ostringstream response;
    response << std::fixed << std::setprecision(6)
             << "{\"ok\":true,"
             << "\"read_only\":true,"
             << "\"endpoint\":\"GET /api/v1/navigation/pre_goal_check\","
             << "\"would_auto_undock\":"
             << (dock_check.final_auto_undock_required ? "true" : "false") << ","
             << "\"auto_undock_required\":"
             << (dock_check.final_auto_undock_required ? "true" : "false") << ","
             << "\"can_auto_undock\":" << (dock_check.can_auto_undock ? "true" : "false") << ","
             << "\"pre_navigation_dock_check\":" << pre_navigation_dock_check_json(
               dock_check,
               "navigation_pre_goal_check",
               pose_id,
               building_id,
               floor_id,
               frame_id,
               has_direct_pose_query) << ","
             << "\"navigation_lifecycle\":{"
             << "\"active\":" << (lifecycle.active ? "true" : "false") << ","
             << "\"detail\":" << json_string(lifecycle.detail) << "},"
             << "\"pose_resolution\":{"
             << "\"requested\":" << (pose_requested ? "true" : "false") << ","
             << "\"ok\":" << (pose_ok ? "true" : "false") << ","
             << "\"status\":" << json_string(pose_status) << ","
             << "\"detail\":" << json_string(pose_detail) << ","
             << "\"source\":" << json_string(target_source) << ","
             << "\"pose_id\":" << json_string(pose_id) << ","
             << "\"building_id\":" << json_string(building_id) << ","
             << "\"floor_id\":" << json_string(floor_id) << ","
             << "\"map_id\":" << json_string(map_id) << ","
             << "\"frame_id\":" << json_string(frame_id);
    if (pose_ok) {
      response << ",\"goal\":{\"x\":" << target.x << ",\"y\":" << target.y << ",\"yaw\":" << target.yaw << "}";
    }
    response << "}}";
    return {200, "application/json", response.str()};
  }

  HttpResponse navigation_goal_error_response(
    const int status,
    const std::string & error,
    const bool pre_navigation_undock,
    const std::string & pre_navigation_undock_detail,
    const std::string & pre_navigation_dock_check_json) const
  {
    std::ostringstream response;
    response << "{\"ok\":false,"
             << "\"accepted\":false,"
             << "\"error\":" << json_string(error) << ","
             << "\"pre_navigation_undock\":" << (pre_navigation_undock ? "true" : "false") << ","
             << "\"pre_navigation_undock_detail\":"
             << json_string(pre_navigation_undock_detail) << ","
             << "\"pre_navigation_dock_check\":" << pre_navigation_dock_check_json << "}";
    return {status, "application/json", response.str()};
  }

  HttpResponse handle_navigation_goal(const std::string & body)
  {
    const auto pose_id = json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or(""));
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    const bool by_pose_id = !pose_id.empty();

    StoredPose target;
    std::string target_source = "direct_pose";
    std::string frame_id = normalized_frame_id(json_string_value(body, "frame_id").value_or("map"));
    const bool force_pre_navigation_relocalization =
      json_bool_value(body, "force_relocalize", false) ||
      json_bool_value(body, "force_relocalization", false);
    const auto pre_navigation_dock_check = pre_navigation_dock_check_snapshot();
    RCLCPP_INFO(
      get_logger(),
      "pre_navigation dock gate source=%s pose_id=%s auto_undock_required=%s can_auto_undock=%s reason=%s "
      "bms_contact=%s bms_stable=%s latch_docked=%s latch_source=%s latch_age=%.3f latch_stale=%s "
      "latch_contradicted=%s latch_valid=%s strong_live_docked=%s docking_state=%s docking_status=%s",
      by_pose_id ? "pose_id" : "direct_pose",
      pose_id.c_str(),
      pre_navigation_dock_check.final_auto_undock_required ? "true" : "false",
      pre_navigation_dock_check.can_auto_undock ? "true" : "false",
      pre_navigation_dock_check.auto_undock_reason.c_str(),
      pre_navigation_dock_check.bms.contact ? "true" : "false",
      pre_navigation_dock_check.live_bms_charging_contact_stable ? "true" : "false",
      pre_navigation_dock_check.dock_latch_indicates_docked ? "true" : "false",
      pre_navigation_dock_check.dock_contact_latch_source.c_str(),
      pre_navigation_dock_check.dock_contact_latch_age_sec,
      pre_navigation_dock_check.dock_contact_latch_stale ? "true" : "false",
      pre_navigation_dock_check.dock_contact_latch_contradicted_by_live_state ? "true" : "false",
      pre_navigation_dock_check.latch_valid_for_auto_undock ? "true" : "false",
      pre_navigation_dock_check.strong_live_docked ? "true" : "false",
      pre_navigation_dock_check.runtime.docking_state.c_str(),
      pre_navigation_dock_check.runtime.docking_status.c_str());
    auto pre_navigation_dock_check_payload = [&]() {
        return pre_navigation_dock_check_json(
          pre_navigation_dock_check,
          "navigation_goal",
          pose_id,
          building_id.value_or(""),
          floor_id.value_or(""),
          frame_id,
          !by_pose_id);
      };
    auto navigation_goal_error = [&](
        const int status,
        const std::string & error,
        const bool pre_navigation_undock = false,
        const std::string & pre_navigation_undock_detail = std::string()) {
        return navigation_goal_error_response(
          status,
          error,
          pre_navigation_undock,
          pre_navigation_undock_detail,
          pre_navigation_dock_check_payload());
      };

    if (by_pose_id) {
      if (!building_id || !safe_map_name(*building_id)) {
        return navigation_goal_error(400, "valid building_id is required for pose_id navigation");
      }
      if (!floor_id || !safe_map_name(*floor_id)) {
        return navigation_goal_error(400, "valid floor_id is required for pose_id navigation");
      }
      if (!safe_pose_id(pose_id)) {
        return navigation_goal_error(400, "valid pose_id is required");
      }
      std::optional<StoredPose> pose;
      try {
        pose = find_floor_catalog_pose(*map_catalog_, *building_id, *floor_id, pose_id);
      } catch (const std::exception & ex) {
        return navigation_goal_error(500, ex.what());
      }
      if (!pose) {
        return navigation_goal_error(404, "pose_id not found in poses.yaml: " + pose_id);
      }
      target = *pose;
      target_source = "poses_yaml";
      frame_id = "map";
    } else {
      const auto x = json_number_value(body, "x").value_or(
        json_nested_number_value(body, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
      const auto y = json_number_value(body, "y").value_or(
        json_nested_number_value(body, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
      const auto yaw = json_number_value(body, "yaw").value_or(
        json_number_value(body, "theta").value_or(
          json_nested_number_value(body, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        return navigation_goal_error(400, "pose_id or finite x, y, and yaw are required");
      }
      target.id = "direct";
      target.name = "direct";
      target.type = "direct_goal";
      target.x = x;
      target.y = y;
      target.yaw = normalize_angle(yaw);
    }

    if (frame_id != "map") {
      return navigation_goal_error(400, "navigation goals must be in map frame");
    }

    if (by_pose_id) {
      std::string context_error;
      bool blocked_by_pending_context = false;
      const auto active_map = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
      if (blocked_by_pending_context) {
        return navigation_goal_error(503, context_error);
      }
      if (active_map &&
        (active_map->building_id != *building_id || active_map->floor_id != *floor_id))
      {
        return navigation_goal_error(
          409,
          "requested pose target does not match confirmed runtime map context: target=" +
          *building_id + "/" + *floor_id + " current=" +
          active_map->building_id + "/" + active_map->floor_id + "/" + active_map->map_id);
      }
    }

    std::lock_guard<std::mutex> navigation_goal_start_lock(navigation_goal_start_mutex_);
    if (navigation_goal_job_running()) {
      return navigation_goal_error(409, "navigation goal is already running; cancel it before starting a new goal");
    }
    join_navigation_goal_worker();

    bool pre_navigation_undock = false;
    std::string pre_navigation_undock_detail;
    if (!undock_before_navigation_if_needed(
        pre_navigation_dock_check,
        pre_navigation_undock_detail,
        pre_navigation_undock))
    {
      return navigation_goal_error(
        409,
        "navigation requires successful undock first: " + pre_navigation_undock_detail,
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }

    const auto lifecycle = navigation_lifecycle_snapshot();
    if (!lifecycle.active) {
      set_navigation_runtime_state(true, "degraded", lifecycle.detail, false);
      return navigation_goal_error(
        503,
        lifecycle.detail,
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }

    bool pre_navigation_relocalization_requested = false;
    bool pre_navigation_relocalization_succeeded = false;
    std::string pre_navigation_relocalization_detail;
    std::uint64_t pre_navigation_relocalization_sequence = 0U;
    const auto pre_navigation_relocalization_decision =
      navigation_goal_relocalization_decision(force_pre_navigation_relocalization);
    if (pre_navigation_relocalization_decision.requested && !pre_navigation_undock) {
      pre_navigation_relocalization_requested = true;
      set_navigation_runtime_state(
        true,
        "relocalize_before_navigation",
        pre_navigation_relocalization_decision.detail);
      const std::string reason = by_pose_id ? "navigation_goal:" + pose_id : "navigation_goal:direct";
      pre_navigation_relocalization_succeeded = trigger_localization_and_wait_for_result(
        reason,
        pre_navigation_relocalization_detail,
        navigation_relocalize_wait_sec_,
        &pre_navigation_relocalization_sequence);
      if (!pre_navigation_relocalization_succeeded && navigation_relocalize_before_goal_required_) {
        const auto detail =
          "navigation requires fresh localization before goal: " + pre_navigation_relocalization_detail;
        set_navigation_runtime_state(true, "localization_failed", detail, false);
        return navigation_goal_error(409, detail, pre_navigation_undock, pre_navigation_undock_detail);
      }
      if (pre_navigation_relocalization_succeeded) {
        set_navigation_runtime_state(
          true,
          "post_relocalization_settle",
          "settling after explicit relocalization before Nav2 goal");
        const auto settle = wait_for_post_relocalization_settle_barrier(
          pre_navigation_relocalization_sequence,
          "manual_before_navigation",
          "nav2_goal");
        if (!settle.ok) {
          const auto detail = settle.failure_code + ": " + settle.detail;
          set_navigation_runtime_state(true, "post_relocalization_settle_failed", detail, false);
          return navigation_goal_error(503, detail, pre_navigation_undock, pre_navigation_undock_detail);
        }
        pre_navigation_relocalization_detail += "; " + settle.detail;
      }
    } else if (pre_navigation_undock) {
      pre_navigation_relocalization_detail = "covered by pre-navigation undock relocalization";
      pre_navigation_relocalization_succeeded = true;
    } else {
      pre_navigation_relocalization_detail = pre_navigation_relocalization_decision.detail;
      pre_navigation_relocalization_succeeded = true;
    }

    std::string tf_chain_detail;
    if (!wait_for_fresh_tf_chain("navigation goal", tf_chain_detail)) {
      set_navigation_runtime_state(true, "tf_not_ready", tf_chain_detail, false);
      return navigation_goal_error(
        503,
        tf_chain_detail,
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }
    if (!pre_navigation_relocalization_detail.empty()) {
      pre_navigation_relocalization_detail += "; ";
    }
    pre_navigation_relocalization_detail += tf_chain_detail;

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

    NavigateGoalHandle::SharedPtr goal_handle;
    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      if (!navigate_to_pose_client_->wait_for_action_server(service_timeout())) {
        return navigation_goal_error(
          503,
          "action unavailable: " + navigate_to_pose_action_,
          pre_navigation_undock,
          pre_navigation_undock_detail);
      }
      auto future = navigate_to_pose_client_->async_send_goal(goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        return navigation_goal_error(
          503,
          "timed out sending navigation goal",
          pre_navigation_undock,
          pre_navigation_undock_detail);
      }
      goal_handle = future.get();
    } catch (const std::exception & exc) {
      return navigation_goal_error(
        503,
        std::string("exception sending navigation goal: ") + exc.what(),
        pre_navigation_undock,
        pre_navigation_undock_detail);
    } catch (...) {
      return navigation_goal_error(
        503,
        "unknown exception sending navigation goal",
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }

    if (!goal_handle) {
      return navigation_goal_error(
        500,
        "navigation goal was rejected by Nav2",
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }

    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_ = goal_handle;
      active_nav_goal_pose_id_ = by_pose_id ? pose_id : "";
      active_nav_goal_building_id_ = building_id.value_or("");
      active_nav_goal_floor_id_ = floor_id.value_or("");
    }

    std::uint64_t navigation_job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      navigation_job_id = ++navigation_goal_job_seq_;
      navigation_goal_job_ = NavigationGoalJob{};
      navigation_goal_job_.id = navigation_job_id;
      navigation_goal_job_.state = "running";
      navigation_goal_job_.phase = "accepted";
      navigation_goal_job_.pose_id = by_pose_id ? pose_id : "";
      navigation_goal_job_.building_id = building_id.value_or("");
      navigation_goal_job_.floor_id = floor_id.value_or("");
      navigation_goal_job_.target_x = target.x;
      navigation_goal_job_.target_y = target.y;
      navigation_goal_job_.target_yaw = target.yaw;
      navigation_goal_job_.final_yaw_align_timeout_sec = navigation_final_yaw_align_timeout_sec_;
      navigation_goal_job_.final_yaw_align_target_yaw_rad = target.yaw;
      navigation_goal_job_.final_yaw_align_max_xy_drift_m = navigation_final_yaw_align_max_xy_drift_m_;
      navigation_goal_job_.final_yaw_align_cmd_topic = navigation_final_yaw_align_cmd_topic_;
      navigation_goal_job_.final_yaw_align_bypass_collision_monitor =
        navigation_final_yaw_align_bypass_collision_monitor_;
      navigation_goal_job_.detail = "navigation goal accepted";
      navigation_goal_job_.started_at = utc_timestamp_iso8601();
    }

    try {
      navigation_goal_worker_ = std::thread(
        [this, navigation_job_id, goal_handle, target]() {
          run_navigation_goal_job_guarded(navigation_job_id, goal_handle, target);
        });
    } catch (const std::exception & exc) {
      finish_navigation_goal_job(
        navigation_job_id,
        false,
        "worker_start_failed",
        std::string("failed to start navigation goal worker: ") + exc.what(),
        -1.0,
        -1.0,
        0,
        false,
        false,
        false,
        false,
        false);
      return navigation_goal_error(
        500,
        std::string("failed to start navigation goal worker: ") + exc.what(),
        pre_navigation_undock,
        pre_navigation_undock_detail);
    }

    set_navigation_runtime_state(true, "navigating", "navigation goal accepted");

    std::ostringstream response;
    response << std::fixed << std::setprecision(6)
             << "{\"ok\":true,"
             << "\"accepted\":true,"
             << "\"navigation_goal_id\":" << navigation_job_id << ","
             << "\"action\":" << json_string(navigate_to_pose_action_) << ","
             << "\"source\":" << json_string(target_source) << ","
             << "\"frame_id\":" << json_string(frame_id) << ","
             << "\"pose_id\":" << json_string(by_pose_id ? pose_id : "") << ",";
    if (building_id) {
      response << "\"building_id\":" << json_string(*building_id) << ",";
    }
    if (floor_id) {
      response << "\"floor_id\":" << json_string(*floor_id) << ",";
    }
    response << "\"pre_navigation_undock\":" << (pre_navigation_undock ? "true" : "false") << ","
             << "\"pre_navigation_undock_detail\":" << json_string(pre_navigation_undock_detail) << ","
             << "\"pre_navigation_dock_check\":" << pre_navigation_dock_check_payload() << ","
             << "\"pre_navigation_relocalization_requested\":"
             << (pre_navigation_relocalization_requested ? "true" : "false") << ","
             << "\"pre_navigation_relocalization_succeeded\":"
             << (pre_navigation_relocalization_succeeded ? "true" : "false") << ","
             << "\"pre_navigation_relocalization_detail\":"
             << json_string(pre_navigation_relocalization_detail) << ","
             << "\"goal\":{\"x\":" << target.x << ",\"y\":" << target.y << ",\"yaw\":" << target.yaw << "}}";
    return {202, "application/json", response.str()};
  }

  bool undock_before_navigation_if_needed(
    const PreNavigationDockCheck & dock_check,
    std::string & detail,
    bool & undock_performed)
  {
    undock_performed = false;
    if (!dock_check.final_auto_undock_required) {
      if (dock_check.docking_active_not_docked_block) {
        detail = "docking is active but not docked: " + dock_check.runtime.docking_state;
        return false;
      }
      detail = dock_check.auto_undock_reason;
      return true;
    }

    undock_performed = true;
    if (!dock_check.runtime_state_undocking && !dock_check.docking_status_indicates_undocking &&
      !start_pre_navigation_undock(detail, dock_check.bms.contact))
    {
      return false;
    }
    return wait_for_pre_navigation_undock(detail);
  }

  bool start_pre_navigation_undock(std::string & detail, const bool charging_contact_at_gate)
  {
    clear_teleop_command();
    publish_teleop_zero_burst();

    std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.state == "running") {
        if (docking_job_.phase == "undocking") {
          detail = "undocking already active";
          return true;
        }
        detail = "cannot auto-undock while docking job is running";
        return false;
      }
    }

    join_docking_worker();

    const auto runtime = runtime_mode_snapshot();
    std::string dock_id = runtime.docking_dock_id;
    std::string ensure_detail;
    if (!ensure_docking_manager_running(ensure_detail)) {
      detail = ensure_detail;
      return false;
    }

    DockingJob next_job;
    next_job.state = "running";
    next_job.phase = "undocking";
    next_job.dock_id = dock_id;
    next_job.detail = "auto_undock_before_navigation";
    next_job.last_status = "undocking before navigation accepted";
    next_job.started_at = utc_timestamp_iso8601();
    next_job.resume_navigation = true;
    next_job.api_accepted = true;
    next_job.already_running = false;
    next_job.docking_status_at_request = runtime.docking_status;

    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      job_id = ++docking_job_seq_;
      next_job.id = job_id;
      docking_job_ = next_job;
    }
    set_docking_runtime_state(true, "undocking", "undocking before navigation accepted");
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_dock_id_ = dock_id;
    }

    std::string service_detail;
    TriggerServiceObservation service_observation;
    if (!call_undock_service_with_charging_retry(
        service_detail, charging_contact_at_gate, &service_observation))
    {
      const auto after_status = runtime_mode_snapshot().docking_status;
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id) {
          docking_job_.api_accepted = false;
          docking_job_.docking_service_called = service_observation.service_called;
          docking_job_.docking_service_success = service_observation.service_success;
          docking_job_.docking_service_message = service_observation.message;
          record_undock_status_observation(docking_job_, after_status);
        }
      }
      finish_docking_job(job_id, false, "failed", service_detail);
      detail = service_detail;
      return false;
    }
    const auto after_status = runtime_mode_snapshot().docking_status;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.docking_service_called = service_observation.service_called;
        docking_job_.docking_service_success = service_observation.service_success;
        docking_job_.docking_service_message = service_observation.message;
        docking_job_.detail = service_detail;
        docking_job_.last_status = service_detail;
        record_undock_status_observation(docking_job_, after_status);
        if (docking_job_.docking_service_success && !docking_job_.undock_started_observed) {
          docking_job_.docking_service_warning =
            "service_success_without_undocking_status_observed_yet";
        }
      }
    }
    set_docking_runtime_state(true, "undocking", service_detail);
    detail = service_detail;
    return true;
  }

  bool call_undock_service_with_charging_retry(
    std::string & detail,
    const bool allow_charging_retry,
    TriggerServiceObservation * observation = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(docking_undock_charging_retry_sec_));
    while (true) {
      const auto observed =
        call_docking_trigger_service_observed(docking_undock_client_, docking_undock_service_);
      detail = observed.message;
      if (observation != nullptr) {
        *observation = observed;
      }
      if (observed.service_success) {
        return true;
      }
      const bool retryable_rejection =
        detail.find("not docked and no charging contact") != std::string::npos ||
        detail.find("undock rejected") != std::string::npos;
      if (!allow_charging_retry || !retryable_rejection || std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      set_docking_runtime_state(true, "undocking", "waiting for docking manager charging state before undock");
      std::this_thread::sleep_for(200ms);
    }
  }

  bool wait_for_pre_navigation_undock(std::string & detail)
  {
    const double post_undock_wait_sec =
      undock_relocalize_after_success_ ? undock_relocalize_wait_sec_ : 0.0;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(navigation_auto_undock_timeout_sec_ + post_undock_wait_sec));
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.state == "undocked") {
          if (!undock_relocalize_after_success_) {
            detail = docking_job_.detail.empty() ? "undocked before navigation" : docking_job_.detail;
            return true;
          }
          if (docking_job_.post_undock_relocalization_requested &&
            docking_job_.post_undock_relocalization_succeeded)
          {
            detail = docking_job_.detail.empty() ?
              "undocked and relocalized before navigation" : docking_job_.detail;
            return true;
          }
          detail = "undocked before navigation but post-undock relocalization did not complete: " +
            docking_job_.post_undock_relocalization_detail;
          return false;
        }
        if (docking_job_.state == "failed") {
          detail = docking_job_.detail.empty() ? "undock failed before navigation" : docking_job_.detail;
          return false;
        }
        if (docking_job_.state == "running" && docking_job_.phase == "relocalize_after_undock") {
          detail = docking_job_.post_undock_relocalization_detail.empty() ?
            "waiting for post-undock relocalization" : docking_job_.post_undock_relocalization_detail;
        }
      }
      const auto runtime = runtime_mode_snapshot();
      if ((runtime.docking_state == "undocked" || docking_status_is_undocked(runtime.docking_status)) &&
        !undock_relocalize_after_success_)
      {
        detail = runtime.docking_status.empty() ? "undocked before navigation" : runtime.docking_status;
        return true;
      }
      if (runtime.docking_state == "failed" || docking_status_is_undock_failed(runtime.docking_status) ||
        runtime.docking_status.find("undock_rejected") != std::string::npos) {
        detail = runtime.docking_status.empty() ? "undock failed before navigation" : runtime.docking_status;
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    detail = "timed out waiting for undock before navigation";
    return false;
  }

  bool cancel_active_navigation_goal(std::string & detail)
  {
    request_navigation_goal_cancel("navigation action cancel requested");
    NavigateGoalHandle::SharedPtr active_goal;
    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_goal = active_nav_goal_handle_;
    }

    if (!active_goal) {
      detail = "no active API goal handle cached";
      return false;
    }

    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      auto future = navigate_to_pose_client_->async_cancel_goal(active_goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        detail = "timed out canceling cached goal handle";
        return false;
      }
    } catch (const std::exception & exc) {
      detail = std::string("exception canceling cached goal handle: ") + exc.what();
      return false;
    } catch (...) {
      detail = "unknown exception canceling cached goal handle";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_.reset();
      active_nav_goal_pose_id_.clear();
      active_nav_goal_building_id_.clear();
      active_nav_goal_floor_id_.clear();
    }
    detail = "cached goal handle cancel requested";
    return true;
  }

  bool cancel_navigation_task_for_mode_switch(const std::string & reason, std::string & detail)
  {
    request_navigation_goal_cancel(reason);
    bool action_available = false;
    std::string action_availability_detail;
    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      action_available = navigate_to_pose_client_->wait_for_action_server(navigation_cancel_action_wait());
    } catch (const std::exception & exc) {
      action_availability_detail = std::string("exception waiting for navigation action server: ") + exc.what();
    } catch (...) {
      action_availability_detail = "unknown exception waiting for navigation action server";
    }

    std::string active_goal_detail = "not requested";
    bool active_goal_cancel_requested = false;
    bool cancel_all_requested = false;
    bool cancel_all_ok = true;
    std::string cancel_all_detail = "not requested";

    if (action_available) {
      active_goal_cancel_requested = cancel_active_navigation_goal(active_goal_detail);
      cancel_all_requested = true;
      try {
        std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
        auto cancel_all_future = navigate_to_pose_client_->async_cancel_all_goals();
        if (cancel_all_future.wait_for(service_timeout()) == std::future_status::ready) {
          cancel_all_detail = "cancel-all requested";
        } else {
          cancel_all_ok = false;
          cancel_all_detail = "timed out canceling navigation goals for mode switch";
        }
      } catch (const std::exception & exc) {
        cancel_all_ok = false;
        cancel_all_detail = std::string("exception canceling navigation goals for mode switch: ") + exc.what();
      } catch (...) {
        cancel_all_ok = false;
        cancel_all_detail = "unknown exception canceling navigation goals for mode switch";
      }
    } else {
      const bool goal_running = navigation_goal_job_running();
      cancel_all_ok = !goal_running;
      active_goal_detail = action_availability_detail.empty()
        ? "action server unavailable: " + navigate_to_pose_action_
        : action_availability_detail;
      cancel_all_detail = goal_running
        ? "navigation goal is still running but action server is unavailable"
        : "no running navigation goal; action server unavailable";
    }

    clear_teleop_command();
    publish_teleop_zero_burst();
    publish_final_yaw_align_zero_burst();

    std::ostringstream out;
    out << reason
        << ": action_available=" << (action_available ? "true" : "false")
        << ", active_goal_cancel_requested=" << (active_goal_cancel_requested ? "true" : "false")
        << ", active_goal_detail=" << active_goal_detail
        << ", cancel_all_requested=" << (cancel_all_requested ? "true" : "false")
        << ", cancel_all_detail=" << cancel_all_detail;
    detail = out.str();
    return cancel_all_ok;
  }

  std::string navigation_cancel_job_json_locked() const
  {
    return navigation_cancel_job_json(navigation_cancel_job_);
  }

  void set_navigation_cancel_job_phase(const std::uint64_t job_id, const std::string & phase)
  {
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    if (navigation_cancel_job_.id == job_id && navigation_cancel_job_.state == "running") {
      navigation_cancel_job_.phase = phase;
    }
  }

  void finish_navigation_cancel_job(
    const std::uint64_t job_id,
    const bool ok,
    const bool action_available,
    const bool active_goal_cancel_requested,
    const bool cancel_all_requested,
    const bool cancel_all_ok,
    const bool stop_stack_ok,
    const std::string & detail,
    const std::string & cancel_all_detail,
    const std::string & stop_stack_detail)
  {
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    if (navigation_cancel_job_.id != job_id) {
      return;
    }
    navigation_cancel_job_.state = ok ? "succeeded" : "failed";
    navigation_cancel_job_.phase = "finished";
    navigation_cancel_job_.ok = ok;
    navigation_cancel_job_.action_available = action_available;
    navigation_cancel_job_.active_goal_cancel_requested = active_goal_cancel_requested;
    navigation_cancel_job_.cancel_all_requested = cancel_all_requested;
    navigation_cancel_job_.cancel_all_ok = cancel_all_ok;
    navigation_cancel_job_.stop_stack_ok = stop_stack_ok;
    navigation_cancel_job_.detail = detail;
    navigation_cancel_job_.cancel_all_detail = cancel_all_detail;
    navigation_cancel_job_.stop_stack_detail = stop_stack_detail;
    navigation_cancel_job_.finished_at = utc_timestamp_iso8601();
    if (ok && navigation_cancel_job_.stop_stack) {
      set_navigation_runtime_state(false, "stopped", "navigation runtime stopped");
    } else if (ok) {
      set_navigation_runtime_state(true, "ready", "navigation goal canceled; navigation stack remains active");
    } else {
      const auto message = stop_stack_detail.empty() ? detail : stop_stack_detail;
      set_navigation_runtime_state(true, "error", message, false);
    }
  }

  void run_navigation_cancel_job(const std::uint64_t job_id, const bool stop_stack)
  {
    bool action_available = false;
    std::string action_availability_detail;
    set_navigation_cancel_job_phase(job_id, "wait_for_nav2_action");
    try {
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      action_available = navigate_to_pose_client_->wait_for_action_server(navigation_cancel_action_wait());
    } catch (const std::exception & exc) {
      action_availability_detail = std::string("exception waiting for navigation action server: ") + exc.what();
    } catch (...) {
      action_availability_detail = "unknown exception waiting for navigation action server";
    }
    std::string active_goal_detail;
    bool active_goal_cancel_requested = false;
    bool cancel_all_requested = false;
    bool cancel_all_ok = true;
    std::string cancel_all_detail = "not requested";

    if (action_available) {
      set_navigation_cancel_job_phase(job_id, "cancel_cached_goal");
      active_goal_cancel_requested = cancel_active_navigation_goal(active_goal_detail);
      cancel_all_requested = true;
      set_navigation_cancel_job_phase(job_id, "cancel_all_goals");
      try {
        std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
        auto cancel_all_future = navigate_to_pose_client_->async_cancel_all_goals();
        if (cancel_all_future.wait_for(service_timeout()) == std::future_status::ready) {
          cancel_all_detail = "cancel-all requested";
        } else {
          cancel_all_ok = false;
          cancel_all_detail = "timed out canceling navigation goals";
        }
      } catch (const std::exception & exc) {
        cancel_all_ok = false;
        cancel_all_detail = std::string("exception canceling navigation goals: ") + exc.what();
      } catch (...) {
        cancel_all_ok = false;
        cancel_all_detail = "unknown exception canceling navigation goals";
      }
    } else {
      active_goal_detail = action_availability_detail.empty()
        ? "action server unavailable: " + navigate_to_pose_action_
        : action_availability_detail;
      cancel_all_ok = false;
      cancel_all_detail = active_goal_detail;
    }

    set_navigation_cancel_job_phase(job_id, "publish_zero_velocity");
    clear_teleop_command();
    publish_teleop_zero_burst();
    publish_final_yaw_align_zero_burst();

    std::string stop_stack_detail = "not requested";
    bool stop_stack_ok = true;
    if (stop_stack) {
      set_navigation_cancel_job_phase(job_id, "stop_navigation_stack");
      stop_stack_ok = stop_navigation_runtime_stack(stop_stack_detail);
      publish_teleop_zero_burst();
      publish_final_yaw_align_zero_burst();
    }

    const bool ok = stop_stack ? stop_stack_ok : cancel_all_ok;
    finish_navigation_cancel_job(
      job_id,
      ok,
      action_available,
      active_goal_cancel_requested,
      cancel_all_requested,
      cancel_all_ok,
      stop_stack_ok,
      active_goal_detail,
      cancel_all_detail,
      stop_stack_detail);
  }

  void run_navigation_cancel_job_guarded(const std::uint64_t job_id, const bool stop_stack)
  {
    try {
      run_navigation_cancel_job(job_id, stop_stack);
    } catch (const std::exception & exc) {
      const std::string detail = std::string("navigation cancel worker exception: ") + exc.what();
      finish_navigation_cancel_job(
        job_id,
        false,
        false,
        false,
        false,
        false,
        false,
        detail,
        detail,
        "not completed");
    } catch (...) {
      const std::string detail = "navigation cancel worker unknown exception";
      finish_navigation_cancel_job(
        job_id,
        false,
        false,
        false,
        false,
        false,
        false,
        detail,
        detail,
        "not completed");
    }
  }

  std::string navigation_goal_job_json_locked() const
  {
    const auto & job = navigation_goal_job_;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"id\":" << job.id
        << ",\"state\":" << json_string(job.state)
        << ",\"phase\":" << json_string(job.phase)
        << ",\"pose_id\":" << json_string(job.pose_id)
        << ",\"building_id\":" << json_string(job.building_id)
        << ",\"floor_id\":" << json_string(job.floor_id)
        << ",\"detail\":" << json_string(job.detail)
        << ",\"started_at\":" << json_string(job.started_at)
        << ",\"completed_at\":" << json_string(job.completed_at)
        << ",\"target\":{\"x\":" << job.target_x
        << ",\"y\":" << job.target_y
        << ",\"yaw\":" << job.target_yaw << "}"
        << ",\"final_distance_m\":" << job.final_distance_m
        << ",\"final_yaw_error_rad\":" << job.final_yaw_error_rad
        << ",\"nav2_result_code\":" << job.nav2_result_code
        << ",\"nav2_succeeded\":" << (job.nav2_succeeded ? "true" : "false")
        << ",\"position_reached\":" << (job.position_reached ? "true" : "false")
        << ",\"final_yaw_align_requested\":" << (job.final_yaw_align_requested ? "true" : "false")
        << ",\"final_yaw_align_attempted\":" << (job.final_yaw_align_attempted ? "true" : "false")
        << ",\"final_yaw_align_succeeded\":" << (job.final_yaw_align_succeeded ? "true" : "false")
        << ",\"final_yaw_align_blocked\":" << (job.final_yaw_align_blocked ? "true" : "false")
        << ",\"final_yaw_align_blocked_reason\":" << json_string(job.final_yaw_align_blocked_reason)
        << ",\"final_yaw_align_duration_sec\":" << job.final_yaw_align_duration_sec
        << ",\"final_yaw_align_timeout_sec\":" << job.final_yaw_align_timeout_sec
        << ",\"final_yaw_align_target_yaw_rad\":" << job.final_yaw_align_target_yaw_rad
        << ",\"final_yaw_align_initial_yaw_error_rad\":" << job.final_yaw_align_initial_yaw_error_rad
        << ",\"final_yaw_align_final_yaw_error_rad\":" << job.final_yaw_align_final_yaw_error_rad
        << ",\"final_yaw_align_max_xy_drift_m\":" << job.final_yaw_align_max_xy_drift_m
        << ",\"final_yaw_align_observed_xy_drift_m\":" << job.final_yaw_align_observed_xy_drift_m
        << ",\"final_yaw_align_cmd_topic\":" << json_string(job.final_yaw_align_cmd_topic)
        << ",\"final_yaw_align_bypass_collision_monitor\":"
        << (job.final_yaw_align_bypass_collision_monitor ? "true" : "false")
        << ",\"final_pose_verified\":" << (job.final_pose_verified ? "true" : "false")
        << ",\"final_pose_verify_reason\":" << json_string(job.final_pose_verify_reason)
        << "}";
    return out.str();
  }

  void join_navigation_goal_worker()
  {
    if (navigation_goal_worker_.joinable()) {
      navigation_goal_worker_.join();
    }
  }

  bool navigation_goal_job_running()
  {
    std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
    return navigation_goal_job_.state == "running";
  }

  void finish_navigation_goal_job(
    const std::uint64_t job_id,
    const bool succeeded,
    const std::string & phase,
    const std::string & detail,
    const double final_distance,
    const double final_yaw_error,
    const int nav2_result_code,
    const bool nav2_succeeded,
    const bool position_reached,
    const bool final_yaw_align_requested,
    const bool final_yaw_align_succeeded,
    const bool final_yaw_align_blocked)
  {
    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      if (navigation_goal_job_.id != job_id) {
        return;
      }
      const bool canceled = phase == "canceled";
      navigation_goal_job_.state = canceled ? "canceled" : (succeeded ? "succeeded" : "failed");
      navigation_goal_job_.phase = phase;
      navigation_goal_job_.detail = detail;
      navigation_goal_job_.completed_at = utc_timestamp_iso8601();
      navigation_goal_job_.final_distance_m = final_distance;
      navigation_goal_job_.final_yaw_error_rad = final_yaw_error;
      navigation_goal_job_.nav2_result_code = nav2_result_code;
      navigation_goal_job_.nav2_succeeded = nav2_succeeded;
      navigation_goal_job_.position_reached = position_reached;
      navigation_goal_job_.final_yaw_align_requested = final_yaw_align_requested;
      navigation_goal_job_.final_yaw_align_succeeded = final_yaw_align_succeeded;
      navigation_goal_job_.final_yaw_align_blocked = final_yaw_align_blocked;
    }
    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_.reset();
      active_nav_goal_pose_id_.clear();
      active_nav_goal_building_id_.clear();
      active_nav_goal_floor_id_.clear();
    }
    if (!runtime_mode_snapshot().navigation_active) {
      return;
    }
    if (succeeded) {
      set_navigation_runtime_state(true, "ready", detail);
    } else if (phase == "canceled") {
      set_navigation_runtime_state(true, "ready", detail);
    } else {
      set_navigation_runtime_state(true, "failed", detail, false);
    }
  }

  bool motion_allowed_snapshot(std::string & detail)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_motion_allowed_) {
      detail = "motion_allowed unavailable";
      return true;
    }
    detail = latest_motion_allowed_ ? "motion allowed" : "motion not allowed";
    return latest_motion_allowed_;
  }

  bool safety_motion_hard_blocked_snapshot(std::string & detail)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_motion_allowed_) {
      detail = "motion_allowed unavailable; relying on robot_safety arbitration";
      return false;
    }
    if (latest_motion_allowed_) {
      detail = "motion allowed";
      return false;
    }
    if (latest_safety_status_ == "COMMAND_STALE") {
      detail = "motion_allowed false because command stream is stale; final yaw command may refresh robot_safety";
      return false;
    }
    detail = "motion not allowed by robot_safety: " + latest_safety_status_;
    return true;
  }

  void publish_final_yaw_align_command(const geometry_msgs::msg::Twist & twist)
  {
    if (navigation_final_yaw_cmd_pub_) {
      navigation_final_yaw_cmd_pub_->publish(twist);
    }
  }

  void publish_final_yaw_align_zero_burst()
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < navigation_final_yaw_align_zero_cmd_count_; ++i) {
      publish_final_yaw_align_command(zero);
      std::this_thread::sleep_for(40ms);
    }
  }

  bool request_navigation_goal_cancel(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
    if (navigation_goal_job_.state != "running") {
      return false;
    }
    navigation_goal_job_.cancel_requested = true;
    navigation_goal_job_.cancel_reason = reason;
    return true;
  }

  bool navigation_goal_cancel_requested(const std::uint64_t job_id, std::string & detail)
  {
    std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
    if (navigation_goal_job_.id != job_id) {
      detail = "navigation goal job was superseded";
      return true;
    }
    if (!navigation_goal_job_.cancel_requested) {
      return false;
    }
    detail = navigation_goal_job_.cancel_reason.empty() ?
      "navigation goal cancel requested" :
      navigation_goal_job_.cancel_reason;
    return true;
  }

  FinalPoseCheck verify_navigation_final_pose(const StoredPose & target, const bool require_fresh_pose)
  {
    FinalPoseCheck check;
    std::string pose_error;
    check.pose = require_fresh_pose ? wait_for_current_robot_pose(true, pose_error) : current_robot_pose_snapshot();
    if (!check.pose.available || check.pose.frame_id != tf_map_frame_) {
      check.reason = pose_error.empty() ? "no fresh map-frame robot pose" : pose_error;
      return check;
    }
    if (require_fresh_pose && check.pose.age_sec > robot_pose_freshness_sec_) {
      check.pose.available = false;
      check.reason = "map-frame robot pose is stale";
      return check;
    }
    check.pose_available = true;
    check.distance_m = std::hypot(check.pose.x - target.x, check.pose.y - target.y);
    check.yaw_error_rad = std::fabs(normalize_angle(target.yaw - check.pose.yaw));
    check.position_reached = check.distance_m <= navigation_goal_position_success_tolerance_m_;
    std::ostringstream reason;
    reason << std::fixed << std::setprecision(3)
           << "final distance=" << check.distance_m
           << " position_tolerance=" << navigation_goal_position_success_tolerance_m_
           << " yaw_error=" << check.yaw_error_rad
           << " yaw_tolerance=" << navigation_final_yaw_tolerance_rad_
           << " yaw_trigger=" << navigation_final_yaw_align_trigger_rad_;
    check.reason = reason.str();
    return check;
  }

  void update_navigation_goal_final_pose_fields(
    const std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
    if (navigation_goal_job_.id != job_id) {
      return;
    }
    navigation_goal_job_.final_distance_m = check.distance_m;
    navigation_goal_job_.final_yaw_error_rad = check.yaw_error_rad;
    navigation_goal_job_.position_reached = check.position_reached;
    navigation_goal_job_.final_pose_verify_reason = reason.empty() ? check.reason : reason;
  }

  void update_navigation_goal_final_yaw_fields(
    const std::uint64_t job_id,
    const FinalYawAlignResult & result)
  {
    std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
    if (navigation_goal_job_.id != job_id) {
      return;
    }
    navigation_goal_job_.final_yaw_align_attempted = result.attempted;
    navigation_goal_job_.final_yaw_align_succeeded = result.succeeded;
    navigation_goal_job_.final_yaw_align_blocked = result.blocked;
    navigation_goal_job_.final_yaw_align_blocked_reason = result.blocked_reason;
    navigation_goal_job_.final_yaw_align_duration_sec = result.duration_sec;
    navigation_goal_job_.final_yaw_align_initial_yaw_error_rad = result.initial_yaw_error_rad;
    navigation_goal_job_.final_yaw_align_final_yaw_error_rad = result.final_yaw_error_rad;
    navigation_goal_job_.final_yaw_align_observed_xy_drift_m = result.observed_xy_drift_m;
  }

  FinalYawAlignResult run_final_yaw_align(
    const std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check)
  {
    FinalYawAlignResult result;
    result.initial_yaw_error_rad = initial_check.yaw_error_rad;
    result.final_yaw_error_rad = initial_check.yaw_error_rad;
    result.attempted = true;

    const auto initial_dock_check = pre_navigation_dock_check_snapshot();
    if (initial_dock_check.final_auto_undock_required) {
      result.blocked = true;
      result.phase = "blocked_by_docked_contact";
      result.blocked_reason = "DOCKED_OR_CHARGING_CONTACT";
      result.detail = "final yaw alignment blocked by dock/contact gate: " +
        initial_dock_check.auto_undock_reason;
      publish_final_yaw_align_zero_burst();
      return result;
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(navigation_final_yaw_align_timeout_sec_));
    const auto started = std::chrono::steady_clock::now();
    const auto tick = std::chrono::milliseconds(67);

    while (std::chrono::steady_clock::now() < deadline) {
      std::string cancel_detail;
      if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
        result.canceled = true;
        result.blocked = true;
        result.phase = "canceled";
        result.blocked_reason = "canceled";
        result.detail = "final yaw alignment canceled: " + cancel_detail;
        break;
      }

      std::string safety_detail;
      if (safety_motion_hard_blocked_snapshot(safety_detail)) {
        result.blocked = true;
        result.phase = "blocked_by_safety";
        result.blocked_reason = safety_detail;
        result.detail = "final yaw alignment blocked by safety: " + safety_detail;
        break;
      }

      const auto dock_check = pre_navigation_dock_check_snapshot();
      if (dock_check.final_auto_undock_required) {
        result.blocked = true;
        result.phase = "blocked_by_docked_contact";
        result.blocked_reason = "DOCKED_OR_CHARGING_CONTACT";
        result.detail = "final yaw alignment stopped by dock/contact gate: " +
          dock_check.auto_undock_reason;
        break;
      }

      auto pose = current_robot_pose_snapshot();
      if (!pose.available || pose.frame_id != tf_map_frame_) {
        result.blocked = true;
        result.phase = "failed_final_yaw_align";
        result.blocked_reason = "no map-frame robot pose";
        result.detail = "final yaw alignment has no fresh pose: " + result.blocked_reason;
        break;
      }
      if (navigation_final_yaw_align_require_fresh_pose_ && pose.age_sec > robot_pose_freshness_sec_) {
        result.blocked = true;
        result.phase = "failed_final_yaw_align";
        result.blocked_reason = "stale map-frame robot pose";
        result.detail = "final yaw alignment has no fresh pose: " + result.blocked_reason;
        break;
      }

      const double xy_drift = std::hypot(pose.x - initial_check.pose.x, pose.y - initial_check.pose.y);
      result.observed_xy_drift_m = std::max(result.observed_xy_drift_m, xy_drift);
      if (xy_drift > navigation_final_yaw_align_max_xy_drift_m_) {
        result.blocked = true;
        result.phase = "failed_final_pose_verify";
        result.blocked_reason = "max_xy_drift_exceeded";
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
               << "final yaw alignment stopped because xy drift=" << xy_drift
               << " max=" << navigation_final_yaw_align_max_xy_drift_m_;
        result.detail = detail.str();
        break;
      }

      const double signed_error = normalize_angle(target.yaw - pose.yaw);
      result.final_yaw_error_rad = std::fabs(signed_error);
      if (result.final_yaw_error_rad <= navigation_final_yaw_tolerance_rad_) {
        result.succeeded = true;
        result.phase = "final_pose_verifying";
        result.detail = "final yaw aligned";
        break;
      }

      const double command_speed = std::clamp(
        std::fabs(navigation_final_yaw_align_kp_ * signed_error),
        navigation_final_yaw_align_min_speed_radps_,
        navigation_final_yaw_align_max_speed_radps_);
      geometry_msgs::msg::Twist twist;
      twist.angular.z = std::copysign(command_speed, signed_error);
      publish_final_yaw_align_command(twist);
      std::this_thread::sleep_for(tick);
    }

    result.duration_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    publish_final_yaw_align_zero_burst();
    if (!result.succeeded && !result.blocked) {
      result.blocked = true;
      result.phase = "failed_final_yaw_align";
      result.blocked_reason = "timeout";
      result.detail = "final yaw alignment timed out";
    }
    return result;
  }

  void run_navigation_goal_job_guarded(
    const std::uint64_t job_id,
    const NavigateGoalHandle::SharedPtr goal_handle,
    const StoredPose target)
  {
    try {
      run_navigation_goal_job(job_id, goal_handle, target);
    } catch (const std::exception & exc) {
      finish_navigation_goal_job(
        job_id,
        false,
        "exception",
        std::string("navigation goal worker exception: ") + exc.what(),
        -1.0,
        -1.0,
        0,
        false,
        false,
        false,
        false,
        false);
    } catch (...) {
      finish_navigation_goal_job(
        job_id,
        false,
        "exception",
        "navigation goal worker unknown exception",
        -1.0,
        -1.0,
        0,
        false,
        false,
        false,
        false,
        false);
    }
  }

  void run_navigation_goal_job(
    const std::uint64_t job_id,
    const NavigateGoalHandle::SharedPtr goal_handle,
    const StoredPose target)
  {
    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      if (navigation_goal_job_.id == job_id) {
        navigation_goal_job_.phase = "waiting_for_nav2_result";
        navigation_goal_job_.detail = "waiting for Nav2 result";
      }
    }

    auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(navigation_goal_result_timeout_sec_));
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      std::string cancel_detail;
      if (navigation_goal_cancel_requested(job_id, cancel_detail)) {
        std::string action_cancel_detail;
        cancel_active_navigation_goal(action_cancel_detail);
        publish_final_yaw_align_zero_burst();
        finish_navigation_goal_job(
          job_id,
          false,
          "canceled",
          "navigation goal canceled while waiting for Nav2 result: " + cancel_detail + "; " + action_cancel_detail,
          -1.0,
          -1.0,
          0,
          false,
          false,
          false,
          false,
          false);
        return;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        std::string cancel_detail;
        cancel_active_navigation_goal(cancel_detail);
        finish_navigation_goal_job(
          job_id,
          false,
          "timeout",
          "timed out waiting for navigation result; " + cancel_detail,
          -1.0,
          -1.0,
          0,
          false,
          false,
          false,
          false,
          false);
        return;
      }
    }

    const auto result = result_future.get();
    const bool nav2_succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    const int result_code = static_cast<int>(result.code);

    std::string cancel_detail;
    if (navigation_goal_cancel_requested(job_id, cancel_detail) ||
      result.code == rclcpp_action::ResultCode::CANCELED)
    {
      finish_navigation_goal_job(
        job_id,
        false,
        "canceled",
        cancel_detail.empty() ? "navigation goal canceled" : "navigation goal canceled: " + cancel_detail,
        -1.0,
        -1.0,
        result_code,
        nav2_succeeded,
        false,
        false,
        false,
        false);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      if (navigation_goal_job_.id == job_id) {
        navigation_goal_job_.phase = "position_reached_verifying";
        navigation_goal_job_.detail = "verifying fresh final map->base_link pose after Nav2 result";
      }
    }

    auto pose_check = verify_navigation_final_pose(target, true);
    double distance = pose_check.distance_m;
    double yaw_error = pose_check.yaw_error_rad;
    bool position_reached = pose_check.position_reached;
    update_navigation_goal_final_pose_fields(job_id, pose_check, pose_check.reason);

    if (!position_reached) {
      std::ostringstream detail;
      if (nav2_succeeded) {
        detail << "navigation reported success but final position is outside tolerance";
      } else {
        detail << "navigation failed with result code " << result_code;
      }
      if (pose_check.pose_available) {
        detail << "; final distance=" << distance << " tolerance="
               << navigation_goal_position_success_tolerance_m_ << " yaw_error=" << yaw_error;
      } else {
        detail << "; " << pose_check.reason;
      }
      finish_navigation_goal_job(
        job_id,
        false,
        "failed_position",
        detail.str(),
        distance,
        yaw_error,
        result_code,
        nav2_succeeded,
        false,
        false,
        false,
        false);
      return;
    }

    bool yaw_align_requested = false;
    bool yaw_align_attempted = false;
    bool yaw_align_succeeded = false;
    bool yaw_align_blocked = false;
    std::string yaw_blocked_reason;
    std::string yaw_detail = "final yaw already within tolerance";
    bool final_pose_verified = true;
    std::string final_pose_verify_reason = "position and yaw are within final tolerance";
    std::string final_phase = "final_pose_verified";

    if (yaw_error > navigation_final_yaw_align_trigger_rad_) {
      yaw_align_requested = true;
      {
        std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
        if (navigation_goal_job_.id == job_id) {
          navigation_goal_job_.phase = "position_reached_yaw_aligning";
          navigation_goal_job_.detail = "position reached; aligning final yaw";
          navigation_goal_job_.final_distance_m = distance;
          navigation_goal_job_.final_yaw_error_rad = yaw_error;
          navigation_goal_job_.position_reached = true;
          navigation_goal_job_.final_yaw_align_requested = true;
          navigation_goal_job_.final_yaw_align_initial_yaw_error_rad = yaw_error;
        }
      }

      if (!navigation_final_yaw_align_enable_) {
        yaw_align_blocked = true;
        yaw_blocked_reason = "disabled";
        yaw_detail = "final yaw alignment disabled";
        final_pose_verified = false;
        final_pose_verify_reason = yaw_detail;
        final_phase = "position_reached_yaw_warning";
      } else {
        const auto align_result = run_final_yaw_align(job_id, target, pose_check);
        yaw_align_attempted = align_result.attempted;
        yaw_align_succeeded = align_result.succeeded;
        yaw_align_blocked = align_result.blocked && !align_result.succeeded;
        yaw_blocked_reason = align_result.blocked_reason;
        yaw_detail = align_result.detail;
        update_navigation_goal_final_yaw_fields(job_id, align_result);
        if (align_result.canceled) {
          finish_navigation_goal_job(
            job_id,
            false,
            "canceled",
            yaw_detail,
            distance,
            yaw_error,
            result_code,
            nav2_succeeded,
            true,
            yaw_align_requested,
            yaw_align_succeeded,
            yaw_align_blocked);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
          if (navigation_goal_job_.id == job_id) {
            navigation_goal_job_.phase = "final_pose_verifying";
            navigation_goal_job_.detail = "verifying final pose after yaw alignment";
          }
        }
        auto final_check = verify_navigation_final_pose(target, true);
        distance = final_check.distance_m;
        yaw_error = final_check.yaw_error_rad;
        position_reached = final_check.position_reached;
        update_navigation_goal_final_pose_fields(job_id, final_check, final_check.reason);
        if (align_result.blocked_reason == "max_xy_drift_exceeded") {
          std::ostringstream detail;
          detail << align_result.detail << "; final pose verify: " << final_check.reason;
          finish_navigation_goal_job(
            job_id,
            false,
            "failed_final_pose_verify",
            detail.str(),
            distance,
            yaw_error,
            result_code,
            nav2_succeeded,
            position_reached,
            yaw_align_requested,
            yaw_align_succeeded,
            yaw_align_blocked);
          return;
        }
        if (!position_reached) {
          std::ostringstream detail;
          detail << "navigation final pose check failed after yaw alignment";
          if (final_check.pose_available) {
            detail << "; final distance=" << distance << " tolerance="
                   << navigation_goal_position_success_tolerance_m_ << " yaw_error=" << yaw_error;
          } else {
            detail << "; " << final_check.reason;
          }
          finish_navigation_goal_job(
            job_id,
            false,
            "failed_final_pose_verify",
            detail.str(),
            distance,
            yaw_error,
            result_code,
            nav2_succeeded,
            false,
            yaw_align_requested,
            yaw_align_succeeded,
            yaw_align_blocked);
          return;
        }

        if (yaw_error <= navigation_final_yaw_tolerance_rad_) {
          final_pose_verified = true;
          final_pose_verify_reason = "position reached and final yaw is within tolerance";
          final_phase = "final_pose_verified";
        } else {
          final_pose_verified = false;
          final_pose_verify_reason = yaw_detail.empty() ?
            "position reached but final yaw remains outside tolerance" :
            yaw_detail;
          final_phase = "position_reached_yaw_warning";
        }
      }
    } else if (yaw_error > navigation_final_yaw_tolerance_rad_) {
      final_pose_verified = true;
      final_pose_verify_reason = "position reached; yaw error is within final yaw trigger deadband";
      final_phase = "final_pose_verified";
      yaw_detail = final_pose_verify_reason;
    }

    std::ostringstream detail;
    if (yaw_align_requested && !yaw_align_succeeded) {
      detail << "navigation position reached; final yaw alignment warning: " << yaw_detail;
    } else {
      detail << "navigation goal reached";
      if (yaw_align_requested) {
        detail << "; " << yaw_detail;
      }
    }
    if (distance >= 0.0) {
      detail << "; final distance=" << distance;
    }
    if (yaw_error >= 0.0) {
      detail << " yaw_error=" << yaw_error;
    }
    {
      std::lock_guard<std::mutex> lock(navigation_goal_job_mutex_);
      if (navigation_goal_job_.id == job_id) {
        navigation_goal_job_.final_pose_verified = final_pose_verified;
        navigation_goal_job_.final_pose_verify_reason = final_pose_verify_reason;
        navigation_goal_job_.final_yaw_align_requested = yaw_align_requested;
        navigation_goal_job_.final_yaw_align_attempted = yaw_align_attempted;
        navigation_goal_job_.final_yaw_align_succeeded = yaw_align_succeeded;
        navigation_goal_job_.final_yaw_align_blocked = yaw_align_blocked;
        navigation_goal_job_.final_yaw_align_blocked_reason = yaw_blocked_reason;
        navigation_goal_job_.final_yaw_align_final_yaw_error_rad = yaw_error;
      }
    }

    finish_navigation_goal_job(
      job_id,
      true,
      final_phase,
      detail.str(),
      distance,
      yaw_error,
      result_code,
      nav2_succeeded,
      position_reached,
      yaw_align_requested,
      yaw_align_succeeded,
      yaw_align_blocked);
  }

  HttpResponse handle_navigation_state()
  {
    refresh_navigation_resume_runtime_state(false);

    const auto runtime = runtime_mode_snapshot();
    const auto dock_check = pre_navigation_dock_check_snapshot();
    const auto dock_check_json = pre_navigation_dock_check_json(
      dock_check,
      "navigation_state",
      "",
      "",
      "",
      "map",
      false);
    const auto amcl_status = read_amcl_runtime_status();
    const auto bridge_status = bridge_status_snapshot();
    const bool amcl_file_authoritative = amcl_status.available && !amcl_status.stale;
    const bool bridge_amcl_available = bridge_status.available && bridge_status.amcl_input_enabled;
    const bool effective_amcl_ready =
      bridge_amcl_available ? bridge_status.amcl_ready :
      (amcl_file_authoritative && amcl_status.ready);
    const bool effective_amcl_degraded =
      bridge_amcl_available ? bridge_status.localization_degraded :
      (amcl_file_authoritative && (amcl_status.degraded || !amcl_status.ready));
    const std::string effective_amcl_degraded_reason =
      bridge_amcl_available ?
      (bridge_status.amcl_degraded_reason.empty() ?
        (bridge_status.localization_degraded ? std::string("AMCL_NOT_READY") : std::string()) :
        bridge_status.amcl_degraded_reason) :
      (effective_amcl_degraded ?
        (amcl_status.degraded_reason.empty() ? std::string("AMCL_NOT_READY") : amcl_status.degraded_reason) :
        std::string());
    const bool localization_degraded =
      effective_amcl_degraded;
    const bool using_triggered_baseline_only =
      (bridge_amcl_available || (amcl_status.available && amcl_status.mode != "disabled")) &&
      !effective_amcl_ready;
    const std::string localization_degraded_reason =
      localization_degraded ? effective_amcl_degraded_reason : std::string();
    std::string safety_status;
    bool motion_allowed = false;
    bool have_motion_allowed = false;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      safety_status = latest_safety_status_;
      motion_allowed = latest_motion_allowed_;
      have_motion_allowed = have_motion_allowed_;
    }
    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    std::lock_guard<std::mutex> goal_lock(navigation_goal_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"mode\":" << json_string(runtime.mode) << ","
             << "\"state\":" << json_string(runtime.navigation_state) << ","
             << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ","
             << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ","
             << "\"message\":" << json_string(runtime.message) << ","
             << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ","
             << "\"localization_degraded_reason\":" << json_string(localization_degraded_reason) << ","
             << "\"using_triggered_baseline_only\":"
             << (using_triggered_baseline_only ? "true" : "false") << ","
             << "\"amcl_mode\":" << json_string(amcl_status.mode) << ","
             << "\"amcl_state\":" << json_string(amcl_status.state) << ","
             << "\"amcl_start_result\":" << json_string(amcl_status.start_result) << ","
             << "\"amcl_status_file_stale\":" << (amcl_status.stale ? "true" : "false") << ","
             << "\"amcl_status_age_ms\":" << amcl_status.age_ms << ","
             << "\"amcl_status_source\":" << json_string(
               bridge_amcl_available ? bridge_status.amcl_status_source :
               (amcl_file_authoritative ? std::string("file") : std::string("stale_file_ignored"))) << ","
             << "\"amcl_ready\":" << (effective_amcl_ready ? "true" : "false") << ","
             << "\"amcl_degraded\":" << (effective_amcl_degraded ? "true" : "false") << ","
             << "\"amcl_degraded_reason\":" << json_string(effective_amcl_degraded_reason) << ","
             << "\"amcl_process_alive\":" << (amcl_status.process_alive ? "true" : "false") << ","
             << "\"amcl_process_ready\":" << ((
               bridge_amcl_available ? bridge_status.amcl_process_ready : amcl_status.process_ready
             ) ? "true" : "false") << ","
             << "\"amcl_seeded\":" << ((
               bridge_amcl_available ? bridge_status.amcl_seeded : amcl_status.seeded
             ) ? "true" : "false") << ","
             << "\"amcl_seed_response_ok\":" << ((
               bridge_amcl_available ? bridge_status.amcl_seed_response_ok : amcl_status.seed_response_ok
             ) ? "true" : "false") << ","
             << "\"amcl_nomotion_pose_received\":" << ((
               bridge_amcl_available ?
                 bridge_status.amcl_nomotion_pose_received :
                 amcl_status.nomotion_pose_received
             ) ? "true" : "false") << ","
             << "\"amcl_static_standby\":" << ((
               bridge_amcl_available ? bridge_status.amcl_static_standby : amcl_status.static_standby
             ) ? "true" : "false") << ","
             << "\"amcl_tracking_ready\":" << ((
               bridge_amcl_available ? bridge_status.amcl_tracking_ready : amcl_status.tracking_ready
             ) ? "true" : "false") << ","
             << "\"amcl_correction_ready\":" << ((
               bridge_amcl_available ? bridge_status.amcl_correction_ready : amcl_status.correction_ready
             ) ? "true" : "false") << ","
             << "\"amcl_not_moving_no_update_ok\":" << ((
               bridge_amcl_available ?
                 bridge_status.amcl_not_moving_no_update_ok :
                 amcl_status.not_moving_no_update_ok
             ) ? "true" : "false") << ","
             << "\"amcl_scan_admission_alive\":"
             << (amcl_status.scan_admission_alive ? "true" : "false") << ","
             << "\"amcl_pose_publisher_count\":" << amcl_status.pose_publisher_count << ","
             << "\"amcl_scan_admission_status_publisher_count\":"
             << amcl_status.scan_admission_status_publisher_count << ","
             << "\"pre_navigation_dock_check\":" << dock_check_json << ","
             << "\"blocked_by_docked_contact\":"
             << (dock_check.final_auto_undock_required ? "true" : "false") << ","
             << "\"normal_motion_blocked_reason\":"
             << json_string(safety_status == "DOCKED_CONTACT_BLOCK" ? "DOCKED_CONTACT_BLOCK" : "") << ","
             << "\"safety\":{"
              << "\"status\":" << json_string(safety_status) << ","
              << "\"motion_allowed\":" << (motion_allowed ? "true" : "false") << ","
              << "\"motion_allowed_valid\":" << (have_motion_allowed ? "true" : "false") << "},"
              << "\"post_relocalization_settle\":" << post_relocalization_settle_state_json() << ","
              << "\"navigation_goal\":" << navigation_goal_job_json_locked() << ","
             << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
    return {200, "application/json", response.str()};
  }

  void join_navigation_cancel_worker()
  {
    if (navigation_cancel_worker_.joinable()) {
      navigation_cancel_worker_.join();
    }
  }

  HttpResponse handle_navigation_cancel(const std::string & body, const bool force_stop_stack)
  {
    const auto reason = json_string_value(body, "reason").value_or("");
    const bool stop_stack = force_stop_stack || json_bool_value(body, "stop_stack", false);

    clear_teleop_command();
    publish_teleop_zero_burst();
    request_navigation_goal_cancel(reason.empty() ? "app_navigation_cancel" : reason);
    publish_final_yaw_align_zero_burst();
    set_navigation_runtime_state(
      true,
      stop_stack ? "stopping" : "canceling",
      stop_stack ? "navigation runtime stop accepted" : "navigation cancel accepted");

    std::lock_guard<std::mutex> start_lock(navigation_cancel_start_mutex_);

    {
      std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
      if (navigation_cancel_job_.state == "running") {
        std::ostringstream response;
        response << "{\"ok\":true,"
                 << "\"accepted\":true,"
                 << "\"already_running\":true,"
                 << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
    }

    join_navigation_cancel_worker();

    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
      job_id = ++navigation_cancel_job_seq_;
      navigation_cancel_job_ = NavigationCancelJob{};
      navigation_cancel_job_.id = job_id;
      navigation_cancel_job_.state = "running";
      navigation_cancel_job_.phase = "accepted";
      navigation_cancel_job_.reason = reason;
      navigation_cancel_job_.stop_stack = stop_stack;
      navigation_cancel_job_.started_at = utc_timestamp_iso8601();
      navigation_cancel_job_.zero_velocity_published = true;
    }

    try {
      navigation_cancel_worker_ = std::thread(
        [this, job_id, stop_stack]() {
          run_navigation_cancel_job_guarded(job_id, stop_stack);
        });
    } catch (const std::exception & exc) {
      finish_navigation_cancel_job(
        job_id,
        false,
        false,
        false,
        false,
        false,
        false,
        std::string("failed to start navigation cancel worker: ") + exc.what(),
        "not requested",
        "not requested");
      return {
        500,
        "application/json",
        error_json(std::string("failed to start navigation cancel worker: ") + exc.what())};
    }

    std::lock_guard<std::mutex> lock(navigation_cancel_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"accepted\":true,"
             << "\"cancel_requested\":true,"
             << "\"stop_stack\":" << (stop_stack ? "true" : "false") << ","
             << "\"action\":" << json_string(navigate_to_pose_action_) << ","
             << "\"navigation_cancel\":" << navigation_cancel_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  bool stop_navigation_runtime_stack(std::string & detail)
  {
    if (navigation_stop_command_.empty() || !fs::exists(navigation_stop_command_)) {
      detail = "navigation stop command is not available: " + navigation_stop_command_;
      return false;
    }

    {
      std::lock_guard<std::mutex> process_lock(navigation_process_mutex_);
      terminate_navigation_resume_process_locked();
    }

    const pid_t pid = ::fork();
    if (pid < 0) {
      detail = "failed to fork navigation stop process";
      return false;
    }
    if (pid == 0) {
      prepare_child_process(navigation_stop_log_file_);
      ::execl("/bin/bash", "bash", navigation_stop_command_.c_str(), static_cast<char *>(nullptr));
      ::_exit(127);
    }

    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    bool exited = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const pid_t wait_result = ::waitpid(pid, &status, WNOHANG);
      if (wait_result == pid) {
        exited = true;
        break;
      }
      if (wait_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        detail = "failed waiting for navigation stop process";
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!exited) {
      ::kill(-pid, SIGKILL);
      ::waitpid(pid, &status, 0);
      detail = "timed out waiting for navigation stop command; log_file=" + navigation_stop_log_file_;
      return false;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      std::ostringstream out;
      out << "navigation stop command failed";
      if (WIFEXITED(status)) {
        out << " with exit code " << WEXITSTATUS(status);
      }
      out << "; log_file=" << navigation_stop_log_file_;
      detail = out.str();
      return false;
    }
    detail = "navigation runtime stack stopped; log_file=" + navigation_stop_log_file_;
    return true;
  }

  HttpResponse publish_estop(const bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
    return {202, "application/json", std::string("{\"ok\":true,\"estop\":") + (active ? "true" : "false") + "}"};
  }

  HttpResponse handle_switch_floor(const std::string & body)
  {
    auto floor_id = json_string_value(body, "floor_id");
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const bool resume_navigation = json_bool_value(body, "resume_navigation", false);

    std::optional<MapManifest> selected_map;
    if (map_id && !map_id->empty()) {
      selected_map = map_catalog_->find_map_by_id(*map_id);
      if (!selected_map) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (!safe_asset_id(building_id) || (floor_id && !safe_asset_id(*floor_id))) {
        return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
      }
      if (json_string_value(body, "building_id") && building_id != selected_map->building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (floor_id && *floor_id != selected_map->floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      building_id = selected_map->building_id;
      floor_id = selected_map->floor_id;
    } else {
      if (!floor_id || floor_id->empty()) {
        return {400, "application/json", error_json("floor_id is required")};
      }
      if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
        return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
      }
      if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = map_catalog_->find_floor_map_by_name(building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {404, "application/json", error_json("map_name not found on requested floor: " + *map_name)};
        }
      } else {
        selected_map = map_catalog_->active_floor_map(building_id, *floor_id);
      }
    }

    if (selected_map) {
      try {
        activate_map_manifest(*selected_map);
      } catch (const std::exception & exc) {
        return {500, "application/json", error_json(exc.what())};
      }
    }

    if (resume_navigation) {
      return handle_resume_floor_navigation(building_id, *floor_id, selected_map);
    }

    if (!floor_switch_client_->wait_for_service(service_timeout())) {
      return {503, "application/json", error_json("service unavailable: " + floor_switch_service_)};
    }
    auto request = std::make_shared<robot_interfaces::srv::SwitchFloor::Request>();
    request->building_id = building_id;
    request->floor_id = *floor_id;
    request->resume_navigation = resume_navigation;

    auto future = floor_switch_client_->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      return {503, "application/json", error_json("timed out waiting for floor switch")};
    }
    const auto response = future.get();
    std::ostringstream out;
    out << "{\"ok\":" << (response->success ? "true" : "false")
        << ",\"message\":" << json_string(response->message)
        << ",\"map_id\":" << json_string(selected_map ? selected_map->map_id : "")
        << ",\"display_name\":" << json_string(selected_map ? selected_map->display_name : "")
        << ",\"nav_map_yaml\":" << json_string(response->nav_map_yaml)
        << ",\"localizer_map_png\":" << json_string(response->localizer_map_png)
        << ",\"localizer_params_yaml\":" << json_string(response->localizer_params_yaml) << "}";
    return {response->success ? 200 : 500, "application/json", out.str()};
  }

  HttpResponse handle_trigger_localization(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("robot_api_server");
    const auto wait_timeout = json_number_value(body, "wait_timeout_sec").value_or(docking_relocalize_wait_sec_);
    std::string detail;
    std::uint64_t relocalization_sequence = 0U;
    bool ok = trigger_localization_and_wait_for_result(
      reason,
      detail,
      wait_timeout,
      &relocalization_sequence);
    if (ok) {
      const auto settle = wait_for_post_relocalization_settle_barrier(
        relocalization_sequence,
        "manual_before_navigation",
        "navigation_resume");
      if (!settle.ok) {
        ok = false;
        detail += "; " + settle.failure_code + ": " + settle.detail;
      } else {
        detail += "; " + settle.detail;
      }
    }
    std::ostringstream out;
    out << "{\"ok\":" << (ok ? "true" : "false")
        << ",\"message\":" << json_string(detail)
        << ",\"last_explicit_relocalization_sequence\":" << relocalization_sequence
        << ",\"post_relocalization_settle\":" << post_relocalization_settle_state_json()
        << "}";
    return {ok ? 200 : 503, "application/json", out.str()};
  }

  std::chrono::nanoseconds docking_navigation_start_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(docking_navigation_start_wait_sec_));
  }

  bool docking_manager_process_running_locked()
  {
    if (docking_manager_pid_ <= 0) {
      return false;
    }
    int status = 0;
    const pid_t wait_result = ::waitpid(docking_manager_pid_, &status, WNOHANG);
    if (wait_result == docking_manager_pid_) {
      docking_manager_pid_ = -1;
      return false;
    }
    if (::kill(docking_manager_pid_, 0) == 0 || errno == EPERM) {
      return true;
    }
    docking_manager_pid_ = -1;
    return false;
  }

  bool ensure_docking_manager_running(std::string & detail)
  {
    if (docking_start_client_->wait_for_service(500ms)) {
      detail = "docking service already available";
      return true;
    }
    if (docking_manager_start_command_.empty() || !fs::exists(docking_manager_start_command_)) {
      detail = "docking manager start command is not available: " + docking_manager_start_command_;
      return false;
    }

    {
      std::lock_guard<std::mutex> process_lock(docking_manager_process_mutex_);
      if (!docking_manager_process_running_locked()) {
        const pid_t pid = ::fork();
        if (pid < 0) {
          detail = "failed to fork docking manager process";
          return false;
        }
        if (pid == 0) {
          prepare_child_process(docking_manager_log_file_);
          ::execl("/bin/bash", "bash", docking_manager_start_command_.c_str(), static_cast<char *>(nullptr));
          ::_exit(127);
        }
        docking_manager_pid_ = pid;
      }
    }

    const auto deadline = std::chrono::steady_clock::now() + service_timeout();
    while (std::chrono::steady_clock::now() < deadline) {
      if (docking_start_client_->wait_for_service(500ms)) {
        detail = "docking manager ready; log_file=" + docking_manager_log_file_;
        return true;
      }
    }
    detail = "timed out waiting for docking service; log_file=" + docking_manager_log_file_;
    return false;
  }

  bool call_docking_trigger_service(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & service_name,
    std::string & detail)
  {
    const auto observed = call_docking_trigger_service_observed(client, service_name);
    detail = observed.message;
    return observed.service_success;
  }

  TriggerServiceObservation call_docking_trigger_service_observed(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & service_name)
  {
    if (!client->wait_for_service(service_timeout())) {
      return {false, false, "service unavailable: " + service_name};
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      return {false, false, "timed out waiting for service: " + service_name};
    }
    const auto response = future.get();
    return {true, response->success, response->message};
  }

  static std::optional<int> status_int_value(const std::string & status, const std::string & key)
  {
    const std::string needle = key + "=";
    const auto pos = status.find(needle);
    if (pos == std::string::npos) {
      return std::nullopt;
    }
    const auto start = pos + needle.size();
    std::size_t end = start;
    while (end < status.size() && (std::isdigit(static_cast<unsigned char>(status[end])) ||
      status[end] == '-' || status[end] == '+'))
    {
      ++end;
    }
    if (end == start) {
      return std::nullopt;
    }
    try {
      return std::stoi(status.substr(start, end - start));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  static std::string status_failure_reason(const std::string & status)
  {
    if (!docking_status_is_undock_failed(status)) {
      return "";
    }
    const auto pos = status.find(" failure_reason=");
    if (pos != std::string::npos) {
      const auto start = pos + std::string(" failure_reason=").size();
      const auto end = status.find(' ', start);
      return status.substr(start, end == std::string::npos ? std::string::npos : end - start);
    }
    const auto end = status.find(' ');
    return status.substr(0, end == std::string::npos ? std::string::npos : end);
  }

  static bool status_indicates_undock_started(const std::string & status)
  {
    return docking_status_is_undocking(status) || docking_status_is_undocked(status) ||
      docking_status_is_undock_failed(status);
  }

  void record_undock_status_observation(DockingJob & job, const std::string & status) const
  {
    if (status.empty()) {
      return;
    }
    job.docking_status_after_request = status;
    if (status_indicates_undock_started(status)) {
      job.undock_started_observed = true;
      if (job.docking_service_success) {
        job.docking_service_warning.clear();
      }
    }
    if (const auto count = status_int_value(status, "cmd_count")) {
      job.undock_cmd_count_observed = *count;
    }
    if (docking_status_is_undock_failed(status)) {
      job.undock_failure_reason = status_failure_reason(status);
    }
  }

  void handle_docking_status(const std::string & status)
  {
    if (docking_status_is_success(status)) {
      std::string dock_id;
      {
        std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
        dock_id = docking_runtime_dock_id_;
      }
      update_dock_contact_latch(true, "docking_status", status, dock_id);
    } else if (docking_status_is_undocked(status)) {
      update_dock_contact_latch(false, "docking_status", status, "");
    }

    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_status_ = status;
    }

    std::uint64_t post_undock_job_id = 0U;
    std::string post_undock_dock_id;
    bool post_undock_required = false;
    std::uint64_t post_fine_docking_job_id = 0U;
    std::string post_fine_docking_dock_id;
    std::string post_fine_docking_status;
    std::string post_fine_docking_final_state;
    bool post_fine_docking_ok = false;
    bool post_fine_docking_required = false;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      docking_job_.last_status = status;
      record_undock_status_observation(docking_job_, status);
      if (docking_job_.state != "running") {
        if (docking_job_.state == "failed" && docking_job_.post_undock_relocalization_required &&
          docking_job_.post_undock_relocalization_requested &&
          !docking_job_.post_undock_relocalization_succeeded)
        {
          set_docking_runtime_state(false, "failed", docking_job_.detail);
          return;
        }
        if (docking_status_is_undocking(status)) {
          set_docking_runtime_state(true, "undocking", status);
        } else if (docking_status_is_undocked(status)) {
          set_docking_runtime_state(false, "undocked", status);
        } else if (docking_status_is_undock_failed(status)) {
          set_docking_runtime_state(false, "failed", status);
        } else if (docking_status_is_success(status)) {
          const auto lowered = lower_copy(status);
          set_docking_runtime_state(
            false,
            starts_with(lowered, "charging") ? "charging" : "docked",
            status);
        }
        return;
      }
      if (docking_job_.phase == "relocalize_after_fine_docking") {
        set_docking_runtime_state(
          true,
          "relocalize_after_fine_docking",
          docking_job_.post_fine_docking_relocalization_detail.empty() ?
            status : docking_job_.post_fine_docking_relocalization_detail);
        return;
      }
      const auto finish_or_relocalize_after_fine_docking =
        [&](const bool ok, const std::string & final_state) {
          if (docking_relocalize_after_fine_docking_ &&
            docking_job_.docking_service_called &&
            !docking_job_.post_fine_docking_relocalization_requested)
          {
            post_fine_docking_job_id = docking_job_.id;
            post_fine_docking_dock_id = docking_job_.dock_id;
            post_fine_docking_status = status;
            post_fine_docking_final_state = final_state;
            post_fine_docking_ok = ok;
            post_fine_docking_required = docking_relocalize_after_fine_docking_required_;
            docking_job_.phase = "relocalize_after_fine_docking";
            docking_job_.detail = status;
            docking_job_.post_fine_docking_relocalization_requested = true;
            docking_job_.post_fine_docking_relocalization_required = post_fine_docking_required;
            docking_job_.post_fine_docking_relocalization_detail =
              "waiting for fresh localization_result after fine docking";
            set_docking_runtime_state(
              true,
              "relocalize_after_fine_docking",
              "fine docking finished; triggering localization before returning to navigation state");
            return;
          }
          finish_docking_job_locked(ok, final_state, status);
        };
      if (docking_status_is_undocked(status)) {
        if (undock_relocalize_after_success_ && !docking_job_.post_undock_relocalization_requested) {
          post_undock_job_id = docking_job_.id;
          post_undock_dock_id = docking_job_.dock_id;
          post_undock_required = docking_job_.resume_navigation;
          docking_job_.phase = "relocalize_after_undock";
          docking_job_.detail = status;
          docking_job_.post_undock_relocalization_requested = true;
          docking_job_.post_undock_relocalization_required = post_undock_required;
          docking_job_.post_undock_relocalization_detail = "waiting for fresh localization_result after undock";
          set_docking_runtime_state(
            true,
            "relocalize_after_undock",
            "undocked by odometry; triggering localization after undock");
        } else if (docking_job_.phase != "relocalize_after_undock") {
          finish_docking_job_locked(true, "undocked", status);
        }
      } else if (docking_status_is_undock_failed(status)) {
        finish_docking_job_locked(false, "failed", status);
      } else if (docking_status_is_undocking(status)) {
        docking_job_.phase = "undocking";
        set_docking_runtime_state(true, "undocking", status);
      } else if (docking_status_is_success(status)) {
        finish_or_relocalize_after_fine_docking(true, "docked");
      } else if (docking_status_is_failure(status)) {
        finish_or_relocalize_after_fine_docking(false, "failed");
      } else if (docking_status_is_stopped(status)) {
        finish_or_relocalize_after_fine_docking(
          true,
          docking_job_.cancel_requested ? "canceled" : "stopped");
      } else {
        set_docking_runtime_state(true, "fine_docking", status);
      }
    }

    if (post_undock_job_id != 0U) {
      start_docking_relocalization_worker(
        [this, post_undock_job_id, post_undock_dock_id, status, post_undock_required]() {
          complete_post_undock_relocalization(
            post_undock_job_id,
            post_undock_dock_id,
            status,
            post_undock_required);
        });
    }
    if (post_fine_docking_job_id != 0U) {
      start_docking_relocalization_worker(
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

  void join_docking_relocalization_worker()
  {
    std::lock_guard<std::mutex> lock(docking_relocalization_worker_mutex_);
    if (docking_relocalization_worker_.joinable()) {
      docking_relocalization_worker_.join();
    }
  }

  void start_docking_relocalization_worker(std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(docking_relocalization_worker_mutex_);
    if (docking_relocalization_worker_.joinable()) {
      docking_relocalization_worker_.join();
    }
    docking_relocalization_worker_ = std::thread(std::move(work));
  }

  void complete_post_fine_docking_relocalization(
    const std::uint64_t job_id,
    const std::string & dock_id,
    const std::string & fine_docking_status,
    const bool original_ok,
    const std::string & original_final_state,
    const bool required_before_final_state)
  {
    std::string localization_detail;
    std::uint64_t relocalization_sequence = 0U;
    const std::string reason =
      "docking_after_fine:" + (dock_id.empty() ? std::string("unknown_dock") : dock_id);
    const bool relocalized = trigger_localization_and_wait_for_result(
      reason,
      localization_detail,
      docking_relocalize_wait_sec_,
      &relocalization_sequence);
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
      finish_docking_job_locked(
        original_ok,
        original_final_state,
        fine_docking_status + "; relocalized after fine docking: " + localization_detail);
      return;
    }
    if (required_before_final_state) {
      finish_docking_job_locked(
        false,
        "failed",
        fine_docking_status + "; relocalize after fine docking failed: " + localization_detail);
      return;
    }
    finish_docking_job_locked(
      original_ok,
      original_final_state,
      fine_docking_status + "; relocalize after fine docking failed: " + localization_detail);
  }

  void complete_post_undock_relocalization(
    const std::uint64_t job_id,
    const std::string & dock_id,
    const std::string & undock_status,
    const bool required_before_navigation)
  {
    std::string localization_detail;
    std::uint64_t relocalization_sequence = 0U;
    const std::string reason =
      "undock_after_success:" + (dock_id.empty() ? std::string("unknown_dock") : dock_id);
    const bool relocalized = trigger_localization_and_wait_for_result(
      reason,
      localization_detail,
      undock_relocalize_wait_sec_,
      &relocalization_sequence);
    bool settle_ok = true;
    if (relocalized) {
      const auto settle = wait_for_post_relocalization_settle_barrier(
        relocalization_sequence,
        "post_undock",
        "nav2_goal",
        [this, job_id](std::string & cancel_detail) {
          if (!docking_cancel_requested(job_id)) {
            return false;
          }
          cancel_detail = "CANCELLED_BY_APP: docking canceled during post-undock settle barrier";
          return true;
        });
      settle_ok = settle.ok;
      localization_detail += "; " +
        (settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail);
    }

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id != job_id || docking_job_.state != "running") {
      return;
    }
    docking_job_.post_undock_relocalization_succeeded = relocalized && settle_ok;
    docking_job_.post_undock_relocalization_detail = localization_detail;
    docking_job_.detail = localization_detail;
    if (relocalized && settle_ok) {
      finish_docking_job_locked(
        true,
        "undocked",
        undock_status + "; relocalized after undock: " + localization_detail);
      return;
    }
    if (required_before_navigation) {
      finish_docking_job_locked(
        false,
        "failed",
        undock_status + "; relocalize/settle after undock failed before navigation: " + localization_detail);
      return;
    }
    finish_docking_job_locked(
      true,
      "undocked",
      undock_status + "; relocalize/settle after undock warning: " + localization_detail);
  }

  std::string docking_job_json_locked() const
  {
    return docking_job_json(docking_job_);
  }

  void set_docking_job_phase(const std::uint64_t job_id, const std::string & phase)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id == job_id && docking_job_.state == "running") {
      docking_job_.phase = phase;
    }
  }

  bool docking_cancel_requested(const std::uint64_t job_id)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    return docking_job_.id == job_id && docking_job_.cancel_requested;
  }

  void finish_docking_job_locked(const bool ok, const std::string & final_state, const std::string & detail)
  {
    if (ok && (final_state == "docked" || final_state == "charging")) {
      update_dock_contact_latch(true, "docking_job", detail, docking_job_.dock_id);
    } else if (ok && final_state == "undocked") {
      update_dock_contact_latch(false, "docking_job", detail, docking_job_.dock_id);
    }
    docking_job_.state = final_state;
    docking_job_.phase = "finished";
    docking_job_.ok = ok;
    docking_job_.detail = detail;
    docking_job_.finished_at = utc_timestamp_iso8601();
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_active_ = false;
      docking_runtime_state_ = final_state;
      docking_runtime_status_ = detail;
      runtime_healthy_ = true;
      runtime_message_ = detail;
      if (final_state == "docked") {
        navigation_runtime_active_ = false;
        navigation_runtime_state_ = "stopped";
      }
    }
  }

  void finish_docking_job(const std::uint64_t job_id, const bool ok, const std::string & final_state, const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id != job_id) {
      return;
    }
    finish_docking_job_locked(ok, final_state, detail);
  }

  void mark_docking_nav_goal_sent(
    const std::uint64_t job_id,
    const NavigateGoalHandle::SharedPtr & goal_handle,
    const std::string & building_id,
    const std::string & floor_id)
  {
    {
      std::lock_guard<std::mutex> lock(active_nav_goal_mutex_);
      active_nav_goal_handle_ = goal_handle;
      active_nav_goal_pose_id_ = "";
      active_nav_goal_building_id_ = building_id;
      active_nav_goal_floor_id_ = floor_id;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.nav_goal_sent = true;
      }
    }
  }

  void run_docking_job(const std::uint64_t job_id)
  {
    DockingJob job;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id != job_id) {
        return;
      }
      job = docking_job_;
    }

    if (job.resume_navigation) {
      bool action_available = false;
      {
        std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
        action_available = navigate_to_pose_client_->wait_for_action_server(500ms);
      }
      if (!action_available) {
        set_docking_job_phase(job_id, "resume_navigation_stack");
        std::optional<MapManifest> selected_map;
        if (!job.map_id.empty()) {
          selected_map = map_catalog_->find_map_by_id(job.map_id);
        } else {
          selected_map = map_catalog_->active_floor_map(job.building_id, job.floor_id);
        }
        const auto resume_response = handle_resume_floor_navigation(job.building_id, job.floor_id, selected_map);
        if (resume_response.status >= 400) {
          finish_docking_job(job_id, false, "failed", "failed to start navigation runtime: " + resume_response.body);
          return;
        }
      }
    }

    set_docking_job_phase(job_id, "wait_for_nav2_action");
    const auto action_deadline = std::chrono::steady_clock::now() + docking_navigation_start_timeout();
    bool action_available = false;
    while (std::chrono::steady_clock::now() < action_deadline) {
      if (docking_cancel_requested(job_id)) {
        finish_docking_job(job_id, true, "canceled", "docking canceled before approach navigation");
        return;
      }
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      if (navigate_to_pose_client_->wait_for_action_server(500ms)) {
        action_available = true;
        break;
      }
    }
    if (!action_available) {
      finish_docking_job(job_id, false, "failed", "action unavailable: " + navigate_to_pose_action_);
      return;
    }

    if (docking_cancel_active_goal_before_predock_) {
      set_docking_job_phase(job_id, "cancel_active_navigation_goal");
      std::string cancel_detail;
      const bool cancel_requested = cancel_active_navigation_goal(cancel_detail);
      clear_teleop_command();
      publish_teleop_zero_burst();
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.active_navigation_cancel_requested = cancel_requested;
          docking_job_.active_navigation_cancel_detail = cancel_detail;
          docking_job_.detail = cancel_detail;
        }
      }
    }

    if (docking_relocalize_before_predock_) {
      set_docking_job_phase(job_id, "relocalize_before_predock");
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.relocalization_requested = true;
          docking_job_.relocalization_detail = "waiting for fresh localization_result";
        }
      }
      set_docking_runtime_state(true, "relocalizing", "triggering localization before predock navigation");
      std::string localization_detail;
      std::uint64_t relocalization_sequence = 0U;
      const bool relocalized = trigger_localization_and_wait_for_result(
        "docking_start_before_predock:" + job.dock_id,
        localization_detail,
        -1.0,
        &relocalization_sequence);
      bool settle_ok = true;
      if (relocalized) {
        set_docking_job_phase(job_id, "settle_before_predock");
        set_docking_runtime_state(
          true,
          "post_relocalization_settle",
          "settling after relocalization before predock Nav2 goal");
        const auto settle = wait_for_post_relocalization_settle_barrier(
          relocalization_sequence,
          "before_predock",
          "nav2_goal",
          [this, job_id](std::string & cancel_detail) {
            if (!docking_cancel_requested(job_id)) {
              return false;
            }
            cancel_detail = "CANCELLED_BY_APP: docking canceled during before-predock settle barrier";
            return true;
          });
        settle_ok = settle.ok;
        localization_detail += "; " +
          (settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail);
      }
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.relocalization_succeeded = relocalized && settle_ok;
          docking_job_.relocalization_detail = localization_detail;
          docking_job_.detail = localization_detail;
        }
      }
      if (!relocalized || !settle_ok) {
        finish_docking_job(
          job_id,
          false,
          "failed",
          "relocalize/settle before predock failed: " + localization_detail);
        return;
      }
    }

    {
      std::string tf_chain_detail;
      if (!wait_for_fresh_tf_chain("docking predock navigation", tf_chain_detail)) {
        finish_docking_job(job_id, false, "failed", tf_chain_detail);
        return;
      }
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        if (!docking_job_.detail.empty()) {
          docking_job_.detail += "; ";
        }
        docking_job_.detail += tf_chain_detail;
      }
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = job.approach_x;
    goal.pose.pose.position.y = job.approach_y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(job.approach_yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(job.approach_yaw * 0.5);

    NavigateGoalHandle::SharedPtr goal_handle;
    try {
      set_docking_job_phase(job_id, "send_approach_goal");
      std::lock_guard<std::mutex> action_lock(navigate_action_mutex_);
      auto future = navigate_to_pose_client_->async_send_goal(goal);
      if (future.wait_for(service_timeout()) != std::future_status::ready) {
        finish_docking_job(job_id, false, "failed", "timed out sending predock navigation goal");
        return;
      }
      goal_handle = future.get();
    } catch (const std::exception & exc) {
      finish_docking_job(job_id, false, "failed", std::string("exception sending predock goal: ") + exc.what());
      return;
    } catch (...) {
      finish_docking_job(job_id, false, "failed", "unknown exception sending predock goal");
      return;
    }
    if (!goal_handle) {
      finish_docking_job(job_id, false, "failed", "predock navigation goal rejected by Nav2");
      return;
    }
    mark_docking_nav_goal_sent(job_id, goal_handle, job.building_id, job.floor_id);
    set_navigation_runtime_state(true, "navigating", "docking predock navigation accepted");
    set_docking_runtime_state(true, "nav_to_predock", "navigating to docking approach pose");

    auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
    set_docking_job_phase(job_id, "nav_to_predock");
    const auto predock_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(docking_predock_nav_timeout_sec_));
    while (result_future.wait_for(200ms) != std::future_status::ready) {
      if (docking_cancel_requested(job_id)) {
        std::string cancel_detail;
        cancel_active_navigation_goal(cancel_detail);
        finish_docking_job(job_id, true, "canceled", cancel_detail);
        return;
      }
      if (std::chrono::steady_clock::now() > predock_deadline) {
        std::string cancel_detail;
        cancel_active_navigation_goal(cancel_detail);
        finish_docking_job(job_id, false, "failed", "timed out navigating to predock pose; " + cancel_detail);
        return;
      }
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      std::ostringstream detail;
      detail << "predock navigation failed with result code " << static_cast<int>(result.code);
      finish_docking_job(job_id, false, "failed", detail.str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.nav_goal_succeeded = true;
      }
    }
    set_navigation_runtime_state(true, "ready", "predock navigation reached");

    if (docking_relocalize_after_predock_) {
      set_docking_job_phase(job_id, "relocalize_after_predock");
      clear_teleop_command();
      publish_teleop_zero_burst();
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.post_predock_relocalization_requested = true;
          docking_job_.post_predock_relocalization_required = docking_relocalize_after_predock_required_;
          docking_job_.post_predock_relocalization_detail =
            "waiting for fresh localization_result after predock navigation";
          docking_job_.detail = docking_job_.post_predock_relocalization_detail;
        }
      }
      set_docking_runtime_state(
        true,
        "relocalize_after_predock",
        "predock reached; triggering localization before fine docking");
      std::string localization_detail;
      std::uint64_t relocalization_sequence = 0U;
      const bool relocalized = trigger_localization_and_wait_for_result(
        "docking_after_predock:" + job.dock_id,
        localization_detail,
        -1.0,
        &relocalization_sequence);
      bool settle_ok = true;
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.post_predock_relocalization_succeeded = relocalized;
          docking_job_.post_predock_relocalization_detail = localization_detail;
          docking_job_.detail = localization_detail;
        }
      }
      if (!relocalized && docking_relocalize_after_predock_required_) {
        finish_docking_job(
          job_id,
          false,
          "failed",
            "relocalize after predock failed before fine docking: " + localization_detail);
        return;
      }
      if (relocalized) {
        set_docking_job_phase(job_id, "settle_after_predock");
        set_docking_runtime_state(
          true,
          "post_relocalization_settle",
          "settling after predock relocalization before fine docking");
        const auto settle = wait_for_post_relocalization_settle_barrier(
          relocalization_sequence,
          "after_predock",
          "fine_docking",
          [this, job_id](std::string & cancel_detail) {
            if (!docking_cancel_requested(job_id)) {
              return false;
            }
            cancel_detail = "CANCELLED_BY_APP: docking canceled during after-predock settle barrier";
            return true;
          });
        settle_ok = settle.ok;
        localization_detail += "; " +
          (settle.ok ? settle.detail : settle.failure_code + ": " + settle.detail);
        {
          std::lock_guard<std::mutex> lock(docking_job_mutex_);
          if (docking_job_.id == job_id && docking_job_.state == "running") {
            docking_job_.post_predock_relocalization_succeeded = settle_ok;
            docking_job_.post_predock_relocalization_detail = localization_detail;
            docking_job_.detail = localization_detail;
          }
        }
        if (!settle_ok) {
          finish_docking_job(
            job_id,
            false,
            "failed",
            "post-predock relocalization settle failed before fine docking: " + localization_detail);
          return;
        }
      }
      if (relocalized && docking_validate_predock_pose_after_relocalization_) {
        std::string pose_check_detail;
        if (!validate_current_pose_near_docking_approach(job, pose_check_detail)) {
          finish_docking_job(
            job_id,
            false,
            "failed",
            "post-predock localization pose is outside approach tolerance; refusing fine docking: " +
              pose_check_detail);
          return;
        }
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id && docking_job_.state == "running") {
          docking_job_.detail += "; " + pose_check_detail;
          docking_job_.post_predock_relocalization_detail += "; " + pose_check_detail;
        }
      }
    }

    {
      std::string tf_chain_detail;
      if (!wait_for_fresh_tf_chain("docking fine docking", tf_chain_detail)) {
        finish_docking_job(job_id, false, "failed", tf_chain_detail);
        return;
      }
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        if (!docking_job_.detail.empty()) {
          docking_job_.detail += "; ";
        }
        docking_job_.detail += tf_chain_detail;
      }
    }

    set_docking_job_phase(job_id, "start_fine_docking");
    if (docking_cancel_requested(job_id)) {
      finish_docking_job(job_id, true, "canceled", "docking canceled before GS2 fine docking");
      return;
    }
    std::string ensure_detail;
    if (!ensure_docking_manager_running(ensure_detail)) {
      finish_docking_job(job_id, false, "failed", ensure_detail);
      return;
    }
    if (docking_cancel_requested(job_id)) {
      finish_docking_job(job_id, true, "canceled", "docking canceled before GS2 fine docking start");
      return;
    }
    std::string service_detail;
    if (!call_docking_trigger_service(docking_start_client_, docking_start_service_, service_detail)) {
      finish_docking_job(job_id, false, "failed", service_detail);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.docking_service_called = true;
        docking_job_.phase = "fine_docking_active";
        docking_job_.detail = service_detail;
      }
    }
    set_docking_runtime_state(true, "fine_docking", service_detail);
  }

  void run_docking_job_guarded(const std::uint64_t job_id)
  {
    try {
      run_docking_job(job_id);
    } catch (const std::exception & exc) {
      finish_docking_job(
        job_id,
        false,
        "failed",
        std::string("docking worker exception: ") + exc.what());
    } catch (...) {
      finish_docking_job(job_id, false, "failed", "docking worker unknown exception");
    }
  }

  HttpResponse handle_docking_start(const std::string & body)
  {
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    auto floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const auto dock_id = json_string_value(body, "dock_id").value_or(
      json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or("")));
    const auto requested_predock_pose_id = json_string_value(body, "predock_pose_id").value_or(
      json_string_value(body, "approach_pose_id").value_or(""));
    const bool resume_navigation = json_bool_value(body, "resume_navigation", true);
    const double approach_distance = std::clamp(
      json_number_value(body, "approach_distance_m").value_or(docking_pre_dock_distance_m_),
      0.10,
      2.00);

    if (!floor_id || floor_id->empty()) {
      return {400, "application/json", error_json("floor_id is required")};
    }
    if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
    }
    if (!safe_pose_id(dock_id)) {
      return {400, "application/json", error_json("valid dock_id is required")};
    }

    std::optional<MapManifest> selected_map;
    try {
      if (map_id && !map_id->empty()) {
        selected_map = map_catalog_->find_map_by_id(*map_id);
        if (!selected_map) {
          return {404, "application/json", error_json("map_id not found: " + *map_id)};
        }
        building_id = selected_map->building_id;
        floor_id = selected_map->floor_id;
        activate_map_manifest(*selected_map);
      } else if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = map_catalog_->find_floor_map_by_name(building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {404, "application/json", error_json("map_name not found on requested floor: " + *map_name)};
        }
        activate_map_manifest(*selected_map);
      } else {
        selected_map = map_catalog_->active_floor_map(building_id, *floor_id);
      }
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    const auto pose = find_floor_catalog_pose(*map_catalog_, building_id, *floor_id, dock_id);
    if (!pose) {
      return {404, "application/json", error_json("dock_id not found in poses.yaml: " + dock_id)};
    }

    std::string predock_source;
    std::string predock_error;
    int predock_error_status = 0;
    const auto predock_pose = resolve_docking_predock_pose(
      building_id,
      *floor_id,
      dock_id,
      *pose,
      requested_predock_pose_id,
      predock_source,
      predock_error,
      predock_error_status);
    if (!predock_error.empty()) {
      return {
        predock_error_status == 0 ? 409 : predock_error_status,
        "application/json",
        error_json(predock_error)};
    }
    if (predock_pose && !validate_manual_docking_predock_pose(*pose, *predock_pose, predock_error)) {
      return {409, "application/json", error_json(predock_error)};
    }

    DockingJob next_job;
    next_job.id = 0U;
    next_job.state = "running";
    next_job.phase = "accepted";
    next_job.building_id = building_id;
    next_job.floor_id = *floor_id;
    next_job.map_id = selected_map ? selected_map->map_id : "";
    next_job.dock_id = dock_id;
    next_job.dock_name = pose->name;
    next_job.dock_type = pose->type;
    next_job.started_at = utc_timestamp_iso8601();
    next_job.resume_navigation = resume_navigation;
    next_job.dock_x = pose->x;
    next_job.dock_y = pose->y;
    next_job.dock_yaw = normalize_angle(pose->yaw);
    if (predock_pose) {
      next_job.predock_pose_id = predock_pose->id;
      next_job.approach_source = predock_source;
      next_job.approach_x = predock_pose->x;
      next_job.approach_y = predock_pose->y;
      next_job.approach_yaw = normalize_angle(predock_pose->yaw);
      next_job.approach_distance_m =
        std::hypot(next_job.dock_x - next_job.approach_x, next_job.dock_y - next_job.approach_y);
    } else {
      next_job.approach_source = "computed_from_dock_pose";
      next_job.approach_distance_m = approach_distance;
      next_job.approach_x = next_job.dock_x - std::cos(next_job.dock_yaw) * approach_distance;
      next_job.approach_y = next_job.dock_y - std::sin(next_job.dock_yaw) * approach_distance;
      next_job.approach_yaw = next_job.dock_yaw;
    }

    std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.state == "running") {
        std::ostringstream response;
        response << "{\"ok\":true,\"accepted\":true,\"already_running\":true,"
                 << "\"docking\":" << docking_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
    }
    join_docking_worker();
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      job_id = ++docking_job_seq_;
      next_job.id = job_id;
      docking_job_ = next_job;
    }
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_active_ = true;
      docking_runtime_state_ = "accepted";
      docking_runtime_dock_id_ = dock_id;
      docking_runtime_status_.clear();
      runtime_healthy_ = true;
      runtime_message_ = "docking accepted";
      mapping_runtime_active_ = false;
      mapping_runtime_state_ = "stopped";
    }

    try {
      docking_worker_ = std::thread([this, job_id]() { run_docking_job_guarded(job_id); });
    } catch (const std::exception & exc) {
      finish_docking_job(job_id, false, "failed", std::string("failed to start docking worker: ") + exc.what());
      return {
        500,
        "application/json",
        error_json(std::string("failed to start docking worker: ") + exc.what())};
    }

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"docking\":" << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_docking_cancel(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("app_docking_cancel");
    clear_teleop_command();
    publish_teleop_zero_burst();
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      docking_job_.cancel_requested = true;
      docking_job_.detail = reason;
      job_id = docking_job_.id;
    }
    std::string nav_detail;
    cancel_active_navigation_goal(nav_detail);
    std::string stop_detail;
    if (docking_stop_client_->wait_for_service(docking_stop_service_wait())) {
      call_docking_trigger_service(docking_stop_client_, docking_stop_service_, stop_detail);
    } else {
      stop_detail = "docking stop service not available";
    }
    publish_teleop_zero_burst();
    if (job_id != 0U) {
      finish_docking_job(job_id, true, "canceled", reason + "; " + nav_detail + "; " + stop_detail);
    } else {
      set_docking_runtime_state(false, "canceled", reason + "; " + stop_detail);
    }
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"navigation_cancel_detail\":"
             << json_string(nav_detail) << ",\"docking_stop_detail\":" << json_string(stop_detail)
             << ",\"docking\":" << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_docking_confirm_docked(const std::string & body)
  {
    auto building_id = json_string_value(body, "building_id").value_or("");
    auto floor_id = json_string_value(body, "floor_id").value_or("");
    auto map_id = json_string_value(body, "map_id").value_or("");
    const auto dock_id = json_string_value(body, "dock_id").value_or("");
    const auto note = json_string_value(body, "note").value_or("");
    const auto reason = json_string_value(body, "reason").value_or("manual_confirm");
    if (!building_id.empty() && !safe_asset_id(building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (!floor_id.empty() && !safe_asset_id(floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (!map_id.empty() && !safe_asset_id(map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }
    if (!dock_id.empty() && !safe_pose_id(dock_id)) {
      return {400, "application/json", error_json("dock_id must be a safe pose id when provided")};
    }
    if (auto context = read_runtime_map_context()) {
      if (building_id.empty()) {
        building_id = context->building_id;
      }
      if (floor_id.empty()) {
        floor_id = context->floor_id;
      }
      if (map_id.empty()) {
        map_id = context->map_id;
      }
    }
    update_dock_contact_latch(
      true,
      "manual_confirm",
      reason,
      dock_id,
      building_id,
      floor_id,
      map_id,
      note);
    const auto latch = read_dock_contact_latch();
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"maintenance_only\":true,"
             << "\"sent_velocity\":false,"
             << "\"latch\":" << dock_contact_latch_snapshot_json(latch) << "}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_docking_clear_latch(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("manual_clear");
    const auto note = json_string_value(body, "note").value_or("");
    update_dock_contact_latch(false, "manual_clear", reason, "", "", "", "", note);
    const auto latch = read_dock_contact_latch();
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"maintenance_only\":true,"
             << "\"sent_velocity\":false,"
             << "\"latch\":" << dock_contact_latch_snapshot_json(latch) << "}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_docking_undock(const std::string & body)
  {
    const auto reason = json_string_value(body, "reason").value_or("app_manual_undock");
    auto dock_id = json_string_value(body, "dock_id").value_or("");
    if (!dock_id.empty() && !safe_pose_id(dock_id)) {
      return {400, "application/json", error_json("dock_id must be a safe pose id when provided")};
    }

    clear_teleop_command();
    publish_teleop_zero_burst();

    std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.state == "running") {
        if (docking_job_.phase == "undocking") {
          std::ostringstream response;
          response << "{\"ok\":true,\"accepted\":true,\"already_running\":true,"
                   << "\"api_accepted\":true,"
                   << "\"docking_service_called\":false,"
                   << "\"docking_service_success\":false,"
                   << "\"docking_service_message\":"
                   << json_string("already running; no new /docking/undock service call") << ","
                   << "\"docking_status_at_request\":" << json_string(docking_job_.last_status) << ","
                   << "\"docking_status_after_request\":" << json_string(docking_job_.last_status) << ","
                   << "\"undock_started_observed\":"
                   << (docking_job_.undock_started_observed ? "true" : "false") << ","
                   << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
                   << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
                   << "\"docking\":" << docking_job_json_locked() << "}";
          return {202, "application/json", response.str()};
        }
        return {409, "application/json", error_json("cannot undock while docking job is running")};
      }
      if (dock_id.empty()) {
        dock_id = docking_job_.dock_id;
      }
    }

    join_docking_worker();

    const auto runtime = runtime_mode_snapshot();
    if (dock_id.empty()) {
      dock_id = runtime.docking_dock_id;
    }
    const auto charging_contact_snapshot = bms_charging_contact_snapshot();
    const bool charging_contact = charging_contact_snapshot.contact;
    const auto dock_check = pre_navigation_dock_check_snapshot();
    const bool docked_state = runtime.docking_state == "docked" ||
      docking_status_is_success(runtime.docking_status) ||
      dock_check.dock_latch_indicates_docked;
    if (!docked_state && !charging_contact) {
      return {
        409,
        "application/json",
        error_json("undock requires docked state or live charging contact")};
    }

    std::string ensure_detail;
    if (!ensure_docking_manager_running(ensure_detail)) {
      return {500, "application/json", error_json(ensure_detail)};
    }

    DockingJob next_job;
    next_job.state = "running";
    next_job.phase = "undocking";
    next_job.dock_id = dock_id;
    next_job.detail = reason;
    next_job.last_status = "undocking accepted";
    next_job.started_at = utc_timestamp_iso8601();
    next_job.resume_navigation = false;
    next_job.api_accepted = true;
    next_job.already_running = false;
    next_job.docking_status_at_request = runtime.docking_status;

    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      job_id = ++docking_job_seq_;
      next_job.id = job_id;
      docking_job_ = next_job;
    }
    set_docking_runtime_state(true, "undocking", "undocking accepted");
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mode_mutex_);
      docking_runtime_dock_id_ = dock_id;
      runtime_message_ = "undocking accepted";
    }

    std::string service_detail;
    TriggerServiceObservation service_observation;
    if (!call_undock_service_with_charging_retry(service_detail, charging_contact, &service_observation)) {
      const auto after_status = runtime_mode_snapshot().docking_status;
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        if (docking_job_.id == job_id) {
          docking_job_.api_accepted = false;
          docking_job_.docking_service_called = service_observation.service_called;
          docking_job_.docking_service_success = service_observation.service_success;
          docking_job_.docking_service_message = service_observation.message;
          record_undock_status_observation(docking_job_, after_status);
        }
      }
      finish_docking_job(job_id, false, "failed", service_detail);
      std::string docking_json;
      {
        std::lock_guard<std::mutex> lock(docking_job_mutex_);
        docking_json = docking_job_json_locked();
      }
      std::ostringstream response;
      response << "{\"ok\":false,\"accepted\":false,\"api_accepted\":false,"
               << "\"already_running\":false,"
               << "\"docking_service_called\":" << (service_observation.service_called ? "true" : "false") << ","
               << "\"docking_service_success\":" << (service_observation.service_success ? "true" : "false") << ","
               << "\"docking_service_message\":" << json_string(service_observation.message) << ","
               << "\"docking_status_at_request\":" << json_string(runtime.docking_status) << ","
               << "\"docking_status_after_request\":" << json_string(after_status) << ","
               << "\"error\":" << json_string(service_detail) << ","
               << "\"docking\":" << docking_json << "}";
      return {409, "application/json", response.str()};
    }
    const auto after_status = runtime_mode_snapshot().docking_status;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id && docking_job_.state == "running") {
        docking_job_.docking_service_called = service_observation.service_called;
        docking_job_.docking_service_success = service_observation.service_success;
        docking_job_.docking_service_message = service_observation.message;
        docking_job_.detail = service_detail;
        docking_job_.last_status = service_detail;
        record_undock_status_observation(docking_job_, after_status);
        if (docking_job_.docking_service_success && !docking_job_.undock_started_observed) {
          docking_job_.docking_service_warning =
            "service_success_without_undocking_status_observed_yet";
        }
      }
    }
    set_docking_runtime_state(true, "undocking", service_detail);

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,"
             << "\"api_accepted\":true,"
             << "\"already_running\":false,"
             << "\"docking_service_called\":" << (docking_job_.docking_service_called ? "true" : "false") << ","
             << "\"docking_service_success\":" << (docking_job_.docking_service_success ? "true" : "false") << ","
             << "\"docking_service_message\":" << json_string(docking_job_.docking_service_message) << ","
             << "\"docking_status_at_request\":" << json_string(docking_job_.docking_status_at_request) << ","
             << "\"docking_status_after_request\":" << json_string(docking_job_.docking_status_after_request) << ","
             << "\"undock_started_observed\":"
             << (docking_job_.undock_started_observed ? "true" : "false") << ","
             << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
             << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
             << "\"docking_service_warning\":" << json_string(docking_job_.docking_service_warning) << ","
             << "\"docking\":" << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_docking_state()
  {
    const auto runtime = runtime_mode_snapshot();
    const auto charging_contact = bms_charging_contact_snapshot();
    const auto dock_check = pre_navigation_dock_check_snapshot();
    const auto dock_check_json = pre_navigation_dock_check_json(
      dock_check,
      "docking_state",
      "",
      "",
      "",
      "map",
      false);
    const bool inferred_docked = dock_check.inferred_docked;
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"mode\":" << json_string(runtime.mode) << ","
             << "\"state\":" << json_string(runtime.docking_state) << ","
             << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ","
             << "\"charging_contact\":" << (charging_contact.contact ? "true" : "false") << ","
             << "\"charging_contact_reason\":" << json_string(charging_contact.reason) << ","
             << "\"inferred_docked\":" << (inferred_docked ? "true" : "false") << ","
             << "\"can_undock\":" << (dock_check.final_is_docked_or_charging ? "true" : "false") << ","
             << "\"can_auto_undock\":" << (dock_check.can_auto_undock ? "true" : "false") << ","
             << "\"auto_undock_reason\":" << json_string(dock_check.auto_undock_reason) << ","
             << "\"last_status\":" << json_string(runtime.docking_status) << ","
             << "\"api_accepted\":" << (docking_job_.api_accepted ? "true" : "false") << ","
             << "\"already_running\":" << (docking_job_.already_running ? "true" : "false") << ","
             << "\"docking_service_called\":"
             << (docking_job_.docking_service_called ? "true" : "false") << ","
             << "\"docking_service_success\":"
             << (docking_job_.docking_service_success ? "true" : "false") << ","
             << "\"docking_service_message\":" << json_string(docking_job_.docking_service_message) << ","
             << "\"docking_status_at_request\":" << json_string(docking_job_.docking_status_at_request) << ","
             << "\"docking_status_after_request\":" << json_string(docking_job_.docking_status_after_request) << ","
             << "\"undock_started_observed\":"
             << (docking_job_.undock_started_observed ? "true" : "false") << ","
             << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
             << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
             << "\"docking_service_warning\":" << json_string(docking_job_.docking_service_warning) << ","
             << "\"pre_navigation_dock_check\":" << dock_check_json << ","
             << "\"docking\":" << docking_job_json_locked() << "}";
    return {200, "application/json", response.str()};
  }

  void join_docking_worker()
  {
    if (docking_worker_.joinable()) {
      docking_worker_.join();
    }
  }

  HttpResponse not_wired(const std::string & endpoint)
  {
    return {
      501,
      "application/json",
      "{\"ok\":false,\"error\":\"endpoint is reserved but not wired to a ROS-native service/action yet\","
      "\"endpoint\":" + json_string(endpoint) + "}"
    };
  }

  bool send_all_bytes(const int client_fd, const void * data, const std::size_t length)
  {
    const char * cursor = static_cast<const char *>(data);
    std::size_t sent = 0;
    while (sent < length) {
      const ssize_t count = ::send(client_fd, cursor + sent, length - sent, MSG_NOSIGNAL);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        return false;
      }
      sent += static_cast<std::size_t>(count);
    }
    return true;
  }

  bool send_all_text(const int client_fd, const std::string & text)
  {
    return send_all_bytes(client_fd, text.data(), text.size());
  }

  bool recv_exact(const int client_fd, void * data, const std::size_t length)
  {
    char * cursor = static_cast<char *>(data);
    std::size_t received = 0;
    while (received < length && running_.load()) {
      const ssize_t count = ::recv(client_fd, cursor + received, length - received, 0);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        return false;
      }
      received += static_cast<std::size_t>(count);
    }
    return received == length;
  }

  void set_socket_receive_timeout(const int client_fd, const double timeout_sec) const
  {
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(timeout_sec);
    timeout.tv_usec = static_cast<suseconds_t>(
      std::max(0.0, timeout_sec - static_cast<double>(timeout.tv_sec)) * 1000000.0);
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  bool websocket_headers_valid(const HttpRequest & request) const
  {
    const auto upgrade_it = request.headers.find("upgrade");
    const auto connection_it = request.headers.find("connection");
    const auto key_it = request.headers.find("sec-websocket-key");
    if (upgrade_it == request.headers.end() || lower_copy(upgrade_it->second) != "websocket") {
      return false;
    }
    if (connection_it == request.headers.end()) {
      return false;
    }
    if (lower_copy(connection_it->second).find("upgrade") == std::string::npos) {
      return false;
    }
    return key_it != request.headers.end() && !key_it->second.empty();
  }

  void send_websocket_handshake(const int client_fd, const HttpRequest & request)
  {
    const auto key = request.headers.at("sec-websocket-key");
    std::ostringstream out;
    out << "HTTP/1.1 101 Switching Protocols\r\n";
    out << "Upgrade: websocket\r\n";
    out << "Connection: Upgrade\r\n";
    out << "Sec-WebSocket-Accept: " << websocket_accept_key(key) << "\r\n";
    out << "Access-Control-Allow-Origin: *\r\n";
    out << "\r\n";
    send_all_text(client_fd, out.str());
  }

  bool send_websocket_frame(
    const int client_fd,
    const std::string & payload,
    const std::uint8_t opcode = 0x1)
  {
    std::vector<std::uint8_t> header;
    header.push_back(static_cast<std::uint8_t>(0x80U | (opcode & 0x0FU)));
    if (payload.size() <= 125U) {
      header.push_back(static_cast<std::uint8_t>(payload.size()));
    } else if (payload.size() <= 65535U) {
      header.push_back(126U);
      header.push_back(static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
      header.push_back(static_cast<std::uint8_t>(payload.size() & 0xFFU));
    } else {
      return false;
    }
    if (!send_all_bytes(client_fd, header.data(), header.size())) {
      return false;
    }
    if (!payload.empty() && !send_all_bytes(client_fd, payload.data(), payload.size())) {
      return false;
    }
    return true;
  }

  std::optional<WebSocketFrame> read_websocket_frame(const int client_fd)
  {
    std::uint8_t header[2]{};
    if (!recv_exact(client_fd, header, sizeof(header))) {
      return std::nullopt;
    }

    const std::uint8_t opcode = header[0] & 0x0FU;
    const bool masked = (header[1] & 0x80U) != 0U;
    std::uint64_t payload_length = header[1] & 0x7FU;
    if (payload_length == 126U) {
      std::uint8_t extended[2]{};
      if (!recv_exact(client_fd, extended, sizeof(extended))) {
        return std::nullopt;
      }
      payload_length = (static_cast<std::uint64_t>(extended[0]) << 8U) |
        static_cast<std::uint64_t>(extended[1]);
    } else if (payload_length == 127U) {
      std::uint8_t extended[8]{};
      if (!recv_exact(client_fd, extended, sizeof(extended))) {
        return std::nullopt;
      }
      payload_length = 0;
      for (const std::uint8_t byte : extended) {
        payload_length = (payload_length << 8U) | static_cast<std::uint64_t>(byte);
      }
    }
    if (payload_length > 4096U) {
      return std::nullopt;
    }

    std::uint8_t mask[4]{};
    if (masked && !recv_exact(client_fd, mask, sizeof(mask))) {
      return std::nullopt;
    }

    std::string payload;
    payload.resize(static_cast<std::size_t>(payload_length));
    if (payload_length > 0U && !recv_exact(client_fd, payload.data(), payload.size())) {
      return std::nullopt;
    }
    if (masked) {
      for (std::size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<char>(static_cast<std::uint8_t>(payload[i]) ^ mask[i % 4U]);
      }
    }
    return WebSocketFrame{opcode, payload};
  }

  void publish_teleop_zero()
  {
    geometry_msgs::msg::Twist twist;
    teleop_cmd_pub_->publish(twist);
  }

  void publish_teleop_zero_burst()
  {
    for (int i = 0; i < 8; ++i) {
      publish_teleop_zero();
      std::this_thread::sleep_for(50ms);
    }
  }

  void publish_teleop_reverse_enable(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled && teleop_allow_reverse_;
    teleop_reverse_enable_pub_->publish(msg);
  }

  bool battery_indicates_charging(const sensor_msgs::msg::BatteryState & msg) const
  {
    return battery_charging_contact(msg).contact;
  }

  BatteryContactEvaluation battery_charging_contact(const sensor_msgs::msg::BatteryState & msg) const
  {
    return evaluate_battery_charging_contact(
      msg,
      teleop_charging_current_min_a_,
      bms_charging_contact_voltage_min_v_,
      bms_charging_contact_voltage_max_v_,
      bms_full_soc_voltage_contact_enable_,
      bms_full_soc_threshold_pct_);
  }

  static std::string json_nullable_number(const bool valid, const double value)
  {
    if (!valid || !std::isfinite(value)) {
      return "null";
    }
    std::ostringstream out;
    out << value;
    return out.str();
  }

  static std::string json_string_array_fragment(const std::vector<std::string> & values)
  {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        out << ",";
      }
      out << json_string(values[i]);
    }
    out << "]";
    return out.str();
  }

  BmsChargingContactSnapshot bms_charging_contact_snapshot()
  {
    BmsChargingContactSnapshot snapshot;
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot.have_state = have_bms_state_;
    if (!have_bms_state_) {
      snapshot.reason = "no_bms_state";
      return snapshot;
    }
    snapshot.have_soc = have_bms_soc_;
    snapshot.soc = latest_bms_soc_;
    snapshot.voltage = latest_bms_voltage_;
    snapshot.current = latest_bms_current_;
    snapshot.temperature = latest_bms_temperature_;
    snapshot.power_supply_status = latest_bms_power_supply_status_;
    snapshot.power_supply_health = latest_bms_power_supply_health_;
    snapshot.power_supply_technology = latest_bms_power_supply_technology_;
    snapshot.present = latest_bms_present_;
    snapshot.age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_bms_received_at_).count();
    snapshot.fresh = snapshot.age_sec <= bms_state_max_age_sec_;
    snapshot.contact_stable_duration_sec = latest_bms_contact_stable_duration_sec_;
    snapshot.no_contact_duration_sec = latest_bms_no_contact_duration_sec_;
    if (!snapshot.fresh) {
      snapshot.reason = "stale_bms_state";
      return snapshot;
    }
    snapshot.contact = latest_bms_charging_contact_;
    snapshot.contact_stable =
      snapshot.contact &&
      snapshot.contact_stable_duration_sec >= dock_contact_latch_bms_require_contact_sec_;
    snapshot.reason = latest_bms_charging_contact_reason_;
    return snapshot;
  }

  PreNavigationDockCheck pre_navigation_dock_check_snapshot()
  {
    PreNavigationDockCheck check;
    check.runtime = runtime_mode_snapshot();
    check.bms = bms_charging_contact_snapshot();
    check.dock_latch = read_dock_contact_latch();

    const auto docking_state = lower_copy(check.runtime.docking_state);
    const auto docking_status = lower_copy(check.runtime.docking_status);
    check.runtime_state_docked = docking_state == "docked";
    check.runtime_state_charging = docking_state == "charging";
    check.runtime_state_undocking = docking_state == "undocking";
    check.live_docking_state_undocked =
      docking_state == "undocked" || docking_status_is_undocked(docking_status);
    check.docking_status_indicates_docked = starts_with(docking_status, "docked");
    check.docking_status_indicates_charging = starts_with(docking_status, "charging");
    check.docking_status_indicates_undocking = docking_status_is_undocking(docking_status);
    check.dock_latch_indicates_docked = check.dock_latch.valid && check.dock_latch.docked;
    check.dock_contact_latch_present = check.dock_latch.valid;
    check.dock_contact_latch_latched_docked = check.dock_latch_indicates_docked;
    check.dock_contact_latch_source = check.dock_latch.source;
    check.dock_contact_latch_reason = check.dock_latch.reason;
    check.dock_contact_latch_age_sec = check.dock_latch.age_sec;
    check.dock_contact_latch_stale = check.dock_latch.stale;
    check.live_bms_charging_contact_stable = check.bms.have_state && check.bms.fresh && check.bms.contact_stable;
    const bool live_bms_no_contact_stable =
      check.bms.have_state &&
      check.bms.fresh &&
      !check.bms.contact &&
      check.bms.no_contact_duration_sec >= dock_contact_latch_bms_clear_no_contact_sec_;
    const bool live_not_docked_or_charging =
      !check.runtime_state_docked &&
      !check.runtime_state_charging &&
      !check.docking_status_indicates_docked &&
      !check.docking_status_indicates_charging &&
      !check.bms.contact;
    check.dock_contact_latch_contradicted_by_live_state =
      dock_contact_latch_clear_when_live_undocked_no_contact_ &&
      check.dock_latch_indicates_docked &&
      check.dock_latch.source_bms &&
      check.live_docking_state_undocked &&
      live_not_docked_or_charging &&
      live_bms_no_contact_stable;
    if (check.dock_contact_latch_contradicted_by_live_state) {
      check.dock_contact_latch_auto_cleared = true;
      check.dock_contact_latch_clear_reason = "stale_bms_latch_cleared_live_undocked_no_contact";
      update_dock_contact_latch(
        false,
        "auto_clear",
        check.dock_contact_latch_clear_reason,
        check.dock_latch.dock_id,
        check.dock_latch.building_id,
        check.dock_latch.floor_id,
        check.dock_latch.map_id,
        "source=bms live_undocked_no_contact");
      check.dock_latch = read_dock_contact_latch();
      check.dock_latch.contradicted_by_live_state = true;
      check.dock_latch_indicates_docked = check.dock_latch.valid && check.dock_latch.docked;
      check.dock_contact_latch_present = check.dock_latch.valid;
      check.dock_contact_latch_latched_docked = check.dock_latch_indicates_docked;
      check.dock_contact_latch_source = check.dock_latch.source;
      check.dock_contact_latch_reason = check.dock_latch.reason;
      check.dock_contact_latch_age_sec = check.dock_latch.age_sec;
      check.dock_contact_latch_stale = check.dock_latch.stale;
    }
    const bool latch_source_valid_for_auto_undock =
      latch_source_is_docking_evidence(check.dock_latch.source) ||
      latch_source_is_manual_evidence(check.dock_latch.source) ||
      (check.dock_latch.source_bms && dock_contact_latch_allow_bms_stale_auto_undock_);
    check.latch_valid_for_auto_undock =
      check.dock_latch_indicates_docked &&
      !check.dock_contact_latch_stale &&
      !check.dock_contact_latch_contradicted_by_live_state &&
      latch_source_valid_for_auto_undock;
    check.strong_live_docked =
      check.runtime_state_docked ||
      check.runtime_state_charging ||
      check.docking_status_indicates_docked ||
      check.docking_status_indicates_charging ||
      check.live_bms_charging_contact_stable;
    if (check.runtime_state_docked) {
      check.docked_evidence.push_back("runtime_state:docked");
    }
    if (check.runtime_state_charging) {
      check.docked_evidence.push_back("runtime_state:charging");
    }
    if (check.docking_status_indicates_docked) {
      check.docked_evidence.push_back("docking_status:docked");
    }
    if (check.docking_status_indicates_charging) {
      check.docked_evidence.push_back("docking_status:charging");
    }
    if (check.live_bms_charging_contact_stable) {
      check.docked_evidence.push_back("bms:" + check.bms.reason);
    } else if (check.bms.contact) {
      check.docked_warnings.push_back(
        "bms_contact_unstable:" + check.bms.reason);
    }
    if (check.latch_valid_for_auto_undock) {
      check.docked_evidence.push_back("latch:" + check.dock_latch.source + ":" + check.dock_latch.reason);
    } else if (check.dock_latch_indicates_docked) {
      check.docked_warnings.push_back(
        "dock_latch_ignored:" + check.dock_latch.source + ":" + check.dock_latch.reason);
    }
    if (!check.bms.have_state) {
      check.docked_warnings.push_back("no_bms_state");
    } else if (!check.bms.fresh) {
      check.docked_warnings.push_back("stale_bms_state");
    } else if (!check.bms.contact) {
      check.docked_warnings.push_back("bms_contact_false:" + check.bms.reason);
    }
    if (!check.dock_latch.valid) {
      check.docked_warnings.push_back("dock_latch_unavailable:" + check.dock_latch.reason);
    } else if (check.dock_contact_latch_stale) {
      check.docked_warnings.push_back("stale_bms_dock_latch");
    } else if (
      check.dock_latch.source_bms &&
      check.dock_latch.age_sec >= 0.0 &&
      dock_contact_latch_max_age_warn_sec_ > 0.0 &&
      check.dock_latch.age_sec > dock_contact_latch_max_age_warn_sec_)
    {
      check.docked_warnings.push_back("old_bms_dock_latch");
    }
    if (check.dock_contact_latch_auto_cleared) {
      check.docked_warnings.push_back(check.dock_contact_latch_clear_reason);
    }
    check.inferred_docked =
      !check.runtime_state_docked && (check.strong_live_docked || check.latch_valid_for_auto_undock);
    check.final_is_docked_or_charging = check.strong_live_docked || check.latch_valid_for_auto_undock;
    check.final_auto_undock_required =
      check.final_is_docked_or_charging ||
      check.runtime_state_undocking ||
      check.docking_status_indicates_undocking;
    check.docking_active_not_docked_block =
      check.runtime.docking_active &&
      !check.final_is_docked_or_charging &&
      !check.runtime_state_undocking &&
      !check.docking_status_indicates_undocking;
    check.can_auto_undock = check.final_auto_undock_required && !check.docking_active_not_docked_block;

    if (check.strong_live_docked) {
      check.docked_state_class = "DOCKED_CONFIRMED";
    } else if (check.latch_valid_for_auto_undock) {
      check.docked_state_class = "DOCKED_LATCHED";
    } else if (check.docking_active_not_docked_block) {
      check.docked_state_class = "UNKNOWN";
    } else if (check.live_docking_state_undocked) {
      check.docked_state_class = "NOT_DOCKED";
    } else {
      check.docked_state_class = "UNKNOWN";
    }

    if (check.runtime_state_undocking || check.docking_status_indicates_undocking) {
      check.auto_undock_reason = "undocking_already_active";
    } else if (check.runtime_state_docked) {
      check.auto_undock_reason = "runtime_docking_state_docked";
    } else if (check.runtime_state_charging) {
      check.auto_undock_reason = "runtime_docking_state_charging";
    } else if (check.docking_status_indicates_charging) {
      check.auto_undock_reason = "docking_status_charging";
    } else if (check.docking_status_indicates_docked) {
      check.auto_undock_reason = "docking_status_docked";
    } else if (check.live_bms_charging_contact_stable) {
      check.auto_undock_reason = "bms_charging_contact:" + check.bms.reason;
    } else if (check.latch_valid_for_auto_undock) {
      check.auto_undock_reason = "dock_contact_latch:" + check.dock_latch.source + ":" + check.dock_latch.reason;
    } else if (check.dock_contact_latch_auto_cleared) {
      check.auto_undock_reason = check.dock_contact_latch_clear_reason;
    } else if (check.dock_contact_latch_stale) {
      check.auto_undock_reason = "stale_bms_latch_ignored";
    } else if (check.docking_active_not_docked_block) {
      check.auto_undock_reason = "docking_active_not_docked:" + check.runtime.docking_state;
    } else {
      check.auto_undock_reason = "not_docked";
    }
    return check;
  }

  std::string bms_charging_contact_snapshot_json(const BmsChargingContactSnapshot & bms) const
  {
    std::ostringstream body;
    body << "\"have_state\":" << (bms.have_state ? "true" : "false") << ","
         << "\"fresh\":" << (bms.fresh ? "true" : "false") << ","
         << "\"age_sec\":" << json_nullable_number(bms.have_state, bms.age_sec) << ","
         << "\"contact\":" << (bms.contact ? "true" : "false") << ","
         << "\"contact_stable\":" << (bms.contact_stable ? "true" : "false") << ","
         << "\"contact_stable_duration_sec\":"
         << json_nullable_number(bms.have_state, bms.contact_stable_duration_sec) << ","
         << "\"no_contact_duration_sec\":"
         << json_nullable_number(bms.have_state, bms.no_contact_duration_sec) << ","
         << "\"reason\":" << json_string(bms.reason) << ","
         << "\"soc\":" << json_nullable_number(bms.have_soc, bms.soc) << ","
         << "\"soc_valid\":" << (bms.have_soc && bms.fresh ? "true" : "false") << ","
         << "\"voltage\":" << json_nullable_number(bms.have_state, bms.voltage) << ","
         << "\"current\":" << json_nullable_number(bms.have_state, bms.current) << ","
         << "\"temperature\":" << json_nullable_number(bms.have_state, bms.temperature) << ","
         << "\"present\":" << (bms.present ? "true" : "false") << ","
         << "\"power_supply_status\":" << bms.power_supply_status << ","
         << "\"power_supply_health\":" << bms.power_supply_health << ","
         << "\"power_supply_technology\":" << bms.power_supply_technology;
    return body.str();
  }

  std::string dock_contact_latch_snapshot_json(const DockContactLatchSnapshot & latch) const
  {
    std::ostringstream body;
    body << "{"
         << "\"valid\":" << (latch.valid ? "true" : "false") << ","
         << "\"latched_docked\":" << (latch.latched_docked ? "true" : "false") << ","
         << "\"docked\":" << (latch.docked ? "true" : "false") << ","
         << "\"source\":" << json_string(latch.source) << ","
         << "\"reason\":" << json_string(latch.reason) << ","
         << "\"building_id\":" << json_string(latch.building_id) << ","
         << "\"floor_id\":" << json_string(latch.floor_id) << ","
         << "\"map_id\":" << json_string(latch.map_id) << ","
         << "\"dock_id\":" << json_string(latch.dock_id) << ","
         << "\"latched_at\":" << json_string(latch.latched_at) << ","
         << "\"last_confirmed_at\":" << json_string(latch.last_confirmed_at) << ","
         << "\"cleared_at\":" << json_string(latch.cleared_at) << ","
         << "\"clear_reason\":" << json_string(latch.clear_reason) << ","
         << "\"note\":" << json_string(latch.note) << ","
         << "\"age_sec\":" << json_nullable_number(latch.age_sec >= 0.0, latch.age_sec) << ","
         << "\"source_bms\":" << (latch.source_bms ? "true" : "false") << ","
         << "\"stale\":" << (latch.stale ? "true" : "false") << ","
         << "\"contradicted_by_live_state\":"
         << (latch.contradicted_by_live_state ? "true" : "false") << ","
         << "\"updated_at\":" << json_string(latch.updated_at) << "}";
    return body.str();
  }

  std::string pre_navigation_dock_check_json(
    const PreNavigationDockCheck & check,
    const std::string & request_source,
    const std::string & pose_id,
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & frame_id,
    const bool direct_pose) const
  {
    std::ostringstream body;
    body << "{"
         << "\"schema\":\"njrh.pre_navigation_dock_check.v1\","
         << "\"request_source\":" << json_string(request_source) << ","
         << "\"pose_id\":" << json_string(pose_id) << ","
         << "\"building_id\":" << json_string(building_id) << ","
         << "\"floor_id\":" << json_string(floor_id) << ","
         << "\"frame_id\":" << json_string(frame_id) << ","
         << "\"direct_pose\":" << (direct_pose ? "true" : "false") << ","
         << "\"api_bms_charging_contact\":" << (check.bms.contact ? "true" : "false") << ","
         << "\"api_bms_charging_contact_stable\":"
         << (check.live_bms_charging_contact_stable ? "true" : "false") << ","
         << "\"api_bms_charging_contact_reason\":" << json_string(check.bms.reason) << ","
         << "\"bms\":{" << bms_charging_contact_snapshot_json(check.bms) << "},"
         << "\"dock_contact_snapshot\":" << dock_contact_latch_snapshot_json(check.dock_latch) << ","
         << "\"dock_contact_latch_present\":"
         << (check.dock_contact_latch_present ? "true" : "false") << ","
         << "\"dock_contact_latch_latched_docked\":"
         << (check.dock_contact_latch_latched_docked ? "true" : "false") << ","
         << "\"dock_contact_latch_source\":" << json_string(check.dock_contact_latch_source) << ","
         << "\"dock_contact_latch_reason\":" << json_string(check.dock_contact_latch_reason) << ","
         << "\"dock_contact_latch_age_sec\":"
         << json_nullable_number(check.dock_contact_latch_age_sec >= 0.0, check.dock_contact_latch_age_sec) << ","
         << "\"dock_contact_latch_stale\":"
         << (check.dock_contact_latch_stale ? "true" : "false") << ","
         << "\"dock_contact_latch_contradicted_by_live_state\":"
         << (check.dock_contact_latch_contradicted_by_live_state ? "true" : "false") << ","
         << "\"dock_contact_latch_auto_cleared\":"
         << (check.dock_contact_latch_auto_cleared ? "true" : "false") << ","
         << "\"dock_contact_latch_clear_reason\":"
         << json_string(check.dock_contact_latch_clear_reason) << ","
         << "\"docking\":{"
         << "\"state\":" << json_string(check.runtime.docking_state) << ","
         << "\"active\":" << (check.runtime.docking_active ? "true" : "false") << ","
         << "\"dock_id\":" << json_string(check.runtime.docking_dock_id) << ","
         << "\"last_status\":" << json_string(check.runtime.docking_status) << ","
         << "\"status_topic\":" << json_string(docking_status_topic_) << ","
         << "\"runtime_state_docked\":" << (check.runtime_state_docked ? "true" : "false") << ","
         << "\"runtime_state_charging\":" << (check.runtime_state_charging ? "true" : "false") << ","
         << "\"runtime_state_undocking\":" << (check.runtime_state_undocking ? "true" : "false") << ","
         << "\"status_indicates_docked\":" << (check.docking_status_indicates_docked ? "true" : "false") << ","
         << "\"status_indicates_charging\":"
         << (check.docking_status_indicates_charging ? "true" : "false") << ","
         << "\"live_docking_state_undocked\":"
         << (check.live_docking_state_undocked ? "true" : "false") << ","
         << "\"status_indicates_undocking\":"
         << (check.docking_status_indicates_undocking ? "true" : "false") << ","
         << "\"dock_latch_indicates_docked\":"
         << (check.dock_latch_indicates_docked ? "true" : "false") << "},"
         << "\"inferred_docked\":" << (check.inferred_docked ? "true" : "false") << ","
         << "\"latched_docked\":" << (check.dock_latch_indicates_docked ? "true" : "false") << ","
         << "\"latched_docked_source\":" << json_string(check.dock_latch.source) << ","
         << "\"latched_docked_age_sec\":"
         << json_nullable_number(check.dock_latch.age_sec >= 0.0, check.dock_latch.age_sec) << ","
         << "\"live_docking_state\":" << json_string(check.runtime.docking_state) << ","
         << "\"live_docking_status_indicates_docked\":"
         << (check.docking_status_indicates_docked ? "true" : "false") << ","
         << "\"live_docking_status_indicates_charging\":"
         << (check.docking_status_indicates_charging ? "true" : "false") << ","
         << "\"live_bms_charging_contact\":"
         << (check.bms.contact ? "true" : "false") << ","
         << "\"live_bms_charging_contact_stable\":"
         << (check.live_bms_charging_contact_stable ? "true" : "false") << ","
         << "\"strong_live_docked\":" << (check.strong_live_docked ? "true" : "false") << ","
         << "\"latch_valid_for_auto_undock\":"
         << (check.latch_valid_for_auto_undock ? "true" : "false") << ","
         << "\"docked_state_class\":" << json_string(check.docked_state_class) << ","
         << "\"docked_evidence\":" << json_string_array_fragment(check.docked_evidence) << ","
         << "\"docked_warnings\":" << json_string_array_fragment(check.docked_warnings) << ","
         << "\"final_is_docked_or_charging\":"
         << (check.final_is_docked_or_charging ? "true" : "false") << ","
         << "\"final_auto_undock_required\":"
         << (check.final_auto_undock_required ? "true" : "false") << ","
         << "\"can_auto_undock\":" << (check.can_auto_undock ? "true" : "false") << ","
         << "\"docking_active_not_docked_block\":"
         << (check.docking_active_not_docked_block ? "true" : "false") << ","
         << "\"auto_undock_reason\":" << json_string(check.auto_undock_reason) << ","
         << "\"final_auto_undock_reason\":" << json_string(check.auto_undock_reason)
         << "}";
    return body.str();
  }

  bool bms_charging_contact_active()
  {
    return bms_charging_contact_snapshot().contact;
  }

  bool teleop_charging_guard_active()
  {
    if (!teleop_stop_on_charging_) {
      return false;
    }
    return bms_charging_contact_active();
  }

  void mark_teleop_session_started()
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    ++teleop_session_count_;
  }

  void mark_teleop_session_stopped()
  {
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      if (teleop_session_count_ > 0) {
        --teleop_session_count_;
      }
      if (teleop_session_count_ > 0) {
        return;
      }
      teleop_command_active_ = false;
      teleop_zero_sent_ = true;
      latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
    }
    publish_teleop_zero();
    publish_teleop_reverse_enable(false);
  }

  void store_teleop_command(const geometry_msgs::msg::Twist & twist)
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    latest_teleop_cmd_ = twist;
    latest_teleop_cmd_at_ = std::chrono::steady_clock::now();
    teleop_command_active_ = true;
    teleop_zero_sent_ = false;
  }

  void clear_teleop_command()
  {
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
      teleop_command_active_ = false;
      teleop_zero_sent_ = true;
    }
    publish_teleop_zero();
    publish_teleop_reverse_enable(false);
  }

  bool teleop_session_active()
  {
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    return teleop_session_count_ > 0;
  }

  void on_teleop_repeat_timer()
  {
    if (!teleop_session_active()) {
      return;
    }
    if (teleop_charging_guard_active()) {
      clear_teleop_command();
      return;
    }

    geometry_msgs::msg::Twist twist;
    bool should_publish_cmd = false;
    bool should_publish_zero = false;
    {
      std::lock_guard<std::mutex> lock(teleop_mutex_);
      if (teleop_session_count_ == 0) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      const bool command_fresh = teleop_command_active_ &&
        latest_teleop_cmd_at_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - latest_teleop_cmd_at_).count() <= teleop_watchdog_timeout_sec_;
      if (command_fresh) {
        twist = latest_teleop_cmd_;
        should_publish_cmd = true;
      } else if (!teleop_zero_sent_) {
        teleop_command_active_ = false;
        teleop_zero_sent_ = true;
        latest_teleop_cmd_ = geometry_msgs::msg::Twist{};
        should_publish_zero = true;
      }
    }

    if (should_publish_cmd) {
      publish_teleop_reverse_enable(teleop_allow_reverse_);
      teleop_cmd_pub_->publish(twist);
    } else if (should_publish_zero) {
      publish_teleop_zero();
      publish_teleop_reverse_enable(false);
    }
  }

  bool mapping_2d_active_now()
  {
    std::lock_guard<std::mutex> process_lock(mapping_process_mutex_);
    return recover_mapping_2d_process_locked() || mapping_2d_active_;
  }

  bool teleop_session_allowed(std::string & reason)
  {
    if (!teleop_require_mapping_active_) {
      return true;
    }
    if (mapping_2d_active_now()) {
      return true;
    }
    reason = "WebSocket teleop is only allowed while 2D mapping is active";
    return false;
  }

  std::string teleop_state_json()
  {
    const bool mapping_active = mapping_2d_active_now();

    bool map_available = false;
    double map_age_sec = 0.0;
    double area_m2 = 0.0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    double resolution = 0.0;
    {
      std::lock_guard<std::mutex> lock(live_map_mutex_);
      if (have_live_map_) {
        map_available = true;
        const auto now = std::chrono::steady_clock::now();
        map_age_sec = std::chrono::duration<double>(now - latest_live_map_received_at_).count();
        width = latest_live_map_.info.width;
        height = latest_live_map_.info.height;
        resolution = latest_live_map_.info.resolution;
        const auto known_cells =
          std::count_if(latest_live_map_.data.begin(), latest_live_map_.data.end(), [](const int8_t value) {
            return value >= 0;
          });
        area_m2 = static_cast<double>(known_cells) * resolution * resolution;
      }
    }

    bool pose_available = false;
    std::string pose_frame;
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;
    double pose_age_sec = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_pose_) {
        pose_frame = latest_pose_frame_;
        pose_x = latest_pose_x_;
        pose_y = latest_pose_y_;
        pose_yaw = latest_pose_yaw_;
        pose_age_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_pose_received_at_).count();
        pose_available = pose_age_sec <= tf_pose_max_age_sec_;
      }
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"ok\":true,\"type\":\"mapping_state\","
        << "\"state\":" << json_string(mapping_active ? "running" : "stopped") << ","
        << "\"teleop_allowed\":" << (mapping_active || !teleop_require_mapping_active_ ? "true" : "false") << ","
        << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << ","
        << "\"map_available\":" << (map_available ? "true" : "false");
    if (map_available) {
      out << ",\"map_age_sec\":" << map_age_sec
          << ",\"area_m2\":" << area_m2
          << ",\"map\":{\"width\":" << width
          << ",\"height\":" << height
          << ",\"resolution\":" << resolution << "}";
    }
    if (pose_available) {
      out << ",\"pose\":{\"frame_id\":" << json_string(pose_frame)
          << ",\"x\":" << pose_x
          << ",\"y\":" << pose_y
          << ",\"yaw\":" << pose_yaw
          << ",\"age_sec\":" << pose_age_sec << "}";
    }
    out << "}";
    return out.str();
  }

  bool publish_teleop_command(const std::string & payload, std::string & ack_json)
  {
    const auto message_type = json_string_value(payload, "type").value_or("cmd_vel");
    if (message_type == "stop") {
      clear_teleop_command();
      ack_json = "{\"ok\":true,\"type\":\"teleop_stopped\"}";
      return true;
    }
    if (message_type != "cmd_vel") {
      ack_json = error_json("unsupported teleop message type: " + message_type);
      return false;
    }

    const double raw_linear_x = json_number_value(payload, "linear_x")
                                  .value_or(json_number_value(payload, "linearX")
                                              .value_or(json_number_value(payload, "vx")
                                                          .value_or(json_nested_number_value(payload, "linear", "x")
                                                                      .value_or(0.0))));
    const double raw_angular_z = json_number_value(payload, "angular_z")
                                  .value_or(json_number_value(payload, "angularZ")
                                              .value_or(json_number_value(payload, "wz")
                                                          .value_or(json_nested_number_value(payload, "angular", "z")
                                                                      .value_or(0.0))));
    if (!std::isfinite(raw_linear_x) || !std::isfinite(raw_angular_z)) {
      ack_json = error_json("linear_x/angular_z must be finite numbers");
      return false;
    }

    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      clear_teleop_command();
      ack_json = error_json(reject_reason);
      return false;
    }
    if (teleop_charging_guard_active()) {
      clear_teleop_command();
      ack_json = error_json("charging detected; teleop command stopped");
      return false;
    }

    const double min_linear_x = teleop_allow_reverse_ ? -teleop_max_linear_x_mps_ : 0.0;
    const double linear_x = std::clamp(raw_linear_x, min_linear_x, teleop_max_linear_x_mps_);
    const double angular_z =
      std::clamp(raw_angular_z, -teleop_max_angular_z_radps_, teleop_max_angular_z_radps_);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    store_teleop_command(twist);
    publish_teleop_reverse_enable(teleop_allow_reverse_);
    teleop_cmd_pub_->publish(twist);

    std::ostringstream out;
    out << "{\"ok\":true,\"type\":\"cmd_vel_ack\","
        << "\"linear_x\":" << linear_x << ","
        << "\"angular_z\":" << angular_z << ","
        << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << ","
        << "\"cmd_topic\":" << json_string(teleop_cmd_topic_) << "}";
    ack_json = out.str();
    return true;
  }

  void handle_teleop_websocket(const int client_fd, const HttpRequest & request)
  {
    if (!token_allowed(request)) {
      send_response(client_fd, {401, "application/json", error_json("missing or invalid X-Robot-Token")});
      return;
    }
    if (!websocket_headers_valid(request)) {
      send_response(client_fd, {400, "application/json", error_json("invalid websocket upgrade request")});
      return;
    }
    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      send_response(client_fd, {409, "application/json", error_json(reject_reason)});
      return;
    }

    send_websocket_handshake(client_fd, request);
    const std::string websocket_client_id = "websocket:" + std::to_string(client_fd);
    const int websocket_ttl_ms = std::clamp(
      static_cast<int>((teleop_socket_idle_timeout_sec_ + 1.0) * 1000.0),
      1000,
      subscription_max_ttl_ms_);
    if (subscription_manager_) {
      subscription_manager_->acquire(
        websocket_client_id,
        {"teleop", "tf"},
        std::chrono::milliseconds(websocket_ttl_ms));
    }
    mark_teleop_session_started();
    set_socket_receive_timeout(client_fd, teleop_socket_idle_timeout_sec_);
    publish_teleop_reverse_enable(teleop_allow_reverse_);
    {
      std::ostringstream ready;
      ready << "{\"ok\":true,\"type\":\"teleop_ready\","
            << "\"cmd_topic\":" << json_string(teleop_cmd_topic_) << ","
            << "\"reverse_enable_topic\":" << json_string(teleop_reverse_enable_topic_) << ","
            << "\"max_linear_x_mps\":" << teleop_max_linear_x_mps_ << ","
            << "\"max_angular_z_radps\":" << teleop_max_angular_z_radps_ << ","
            << "\"watchdog_timeout_sec\":" << teleop_watchdog_timeout_sec_ << ","
            << "\"socket_idle_timeout_sec\":" << teleop_socket_idle_timeout_sec_ << ","
            << "\"repeat_rate_hz\":" << teleop_repeat_rate_hz_ << ","
            << "\"require_mapping_active\":" << (teleop_require_mapping_active_ ? "true" : "false") << ","
            << "\"allow_reverse\":" << (teleop_allow_reverse_ ? "true" : "false") << "}";
      send_websocket_frame(client_fd, ready.str());
      send_websocket_frame(client_fd, teleop_state_json());
    }

    while (running_.load()) {
      const auto frame = read_websocket_frame(client_fd);
      if (!frame) {
        break;
      }
      if (frame->opcode == 0x8U) {
        send_websocket_frame(client_fd, "", 0x8U);
        break;
      }
      if (frame->opcode == 0x9U) {
        send_websocket_frame(client_fd, frame->payload, 0xAU);
        continue;
      }
      if (frame->opcode != 0x1U) {
        send_websocket_frame(client_fd, error_json("only text websocket frames are accepted"));
        continue;
      }

      std::string ack;
      if (subscription_manager_) {
        subscription_manager_->acquire(
          websocket_client_id,
          {"teleop", "tf"},
          std::chrono::milliseconds(websocket_ttl_ms));
      }
      publish_teleop_command(frame->payload, ack);
      if (!send_websocket_frame(client_fd, ack)) {
        break;
      }
      if (!send_websocket_frame(client_fd, teleop_state_json())) {
        break;
      }
    }

    mark_teleop_session_stopped();
    if (subscription_manager_) {
      subscription_manager_->release(websocket_client_id, {"teleop", "tf"});
    }
  }

  std::string host_;
  int port_{8080};
  std::string api_token_;
  int max_http_connections_{16};
  std::string maps_root_;
  std::string runtime_maps_dir_;
  std::string safety_estop_topic_;
  std::string safety_status_topic_;
  std::string safety_motion_allowed_topic_;
  std::string floor_status_topic_;
  std::string bms_state_topic_;
  double bms_state_max_age_sec_{3.0};
  std::string floor_switch_service_;
  std::string localization_trigger_service_;
  std::string localization_result_topic_;
  std::string localization_bridge_status_topic_;
  std::string navigate_to_pose_action_;
  std::string mapping_2d_start_command_;
  std::string mapping_2d_log_file_;
  std::string mapping_lidar_rps_xps_state_dir_;
  std::string navigation_resume_command_;
  std::string navigation_resume_log_file_;
  std::string runtime_map_context_file_;
  std::string amcl_runtime_status_file_;
  double amcl_runtime_status_ttl_sec_{5.0};
  std::string last_navigation_map_file_;
  std::string navigation_stop_command_;
  std::string navigation_stop_log_file_;
  std::string docking_manager_start_command_;
  std::string docking_manager_log_file_;
  std::string docking_start_service_;
  std::string docking_stop_service_;
  std::string docking_undock_service_;
  std::string docking_status_topic_;
  std::string docking_contact_latch_file_;
  double dock_contact_latch_bms_ttl_sec_{300.0};
  double dock_contact_latch_bms_require_contact_sec_{2.0};
  double dock_contact_latch_bms_clear_no_contact_sec_{3.0};
  bool dock_contact_latch_allow_bms_stale_auto_undock_{false};
  bool dock_contact_latch_clear_when_live_undocked_no_contact_{true};
  double dock_contact_latch_max_age_warn_sec_{600.0};
  bool have_last_dock_contact_latch_write_{false};
  bool last_dock_contact_latch_docked_{false};
  std::string last_dock_contact_latch_source_;
  std::string last_dock_contact_latch_reason_;
  std::string last_dock_contact_latch_dock_id_;
  std::string last_dock_contact_latch_note_;
  double docking_pre_dock_distance_m_{0.60};
  double docking_navigation_start_wait_sec_{45.0};
  double docking_predock_nav_timeout_sec_{180.0};
  bool docking_relocalize_before_predock_{true};
  bool docking_relocalize_after_predock_{true};
  bool docking_relocalize_after_predock_required_{true};
  bool docking_relocalize_after_fine_docking_{true};
  bool docking_relocalize_after_fine_docking_required_{false};
  bool docking_validate_predock_pose_after_relocalization_{true};
  double docking_predock_pose_max_distance_m_{0.35};
  double docking_predock_pose_max_yaw_rad_{0.35};
  bool docking_manual_predock_distance_check_enable_{false};
  double docking_manual_predock_min_distance_m_{0.50};
  double docking_manual_predock_max_distance_m_{1.20};
  double docking_manual_predock_max_yaw_error_rad_{0.80};
  double docking_relocalize_wait_sec_{8.0};
  double docking_relocalize_recent_result_max_age_sec_{5.0};
  bool undock_relocalize_after_success_{true};
  double undock_relocalize_wait_sec_{8.0};
  bool docking_cancel_active_goal_before_predock_{true};
  double navigation_auto_undock_timeout_sec_{28.0};
  double docking_undock_charging_retry_sec_{3.0};
  std::string localization_bridge_force_accept_service_;
  std::string mapping_2d_live_map_topic_;
  double mapping_2d_live_map_max_age_sec_{3.0};
  std::string scan_topic_;
  double scan_max_age_sec_{2.0};
  std::string tf_topic_;
  std::string tf_static_topic_;
  std::string tf_map_frame_{"map"};
  std::string tf_odom_frame_{"odom"};
  std::string tf_base_frame_{"base_link"};
  std::string post_relocalization_static_lidar_frame_{"lidar_level_link"};
  double tf_pose_max_age_sec_{2.0};
  double robot_pose_freshness_sec_{0.5};
  double tf_chain_freshness_sec_{0.30};
  double tf_chain_settle_timeout_sec_{2.0};
  std::string local_costmap_topic_{"/local_costmap/costmap"};
  bool post_relocalization_settle_enabled_{true};
  int post_relocalization_settle_min_ms_{800};
  int post_relocalization_settle_max_ms_{3000};
  int post_relocalization_stable_tf_samples_{5};
  int post_relocalization_tf_sample_period_ms_{100};
  bool post_relocalization_zero_cmd_{true};
  bool post_relocalization_require_local_costmap_update_{true};
  int post_relocalization_required_local_costmap_updates_{2};
  bool post_relocalization_reject_if_new_message_filter_drop_{true};
  double post_relocalization_large_correction_translation_m_{0.5};
  double post_relocalization_large_correction_yaw_rad_{0.3};
  int post_relocalization_large_correction_min_ms_{1500};
  std::string teleop_cmd_topic_;
  std::string teleop_reverse_enable_topic_;
  std::string teleop_pose_topic_;
  double teleop_max_linear_x_mps_{1.00};
  double teleop_max_angular_z_radps_{0.55};
  bool teleop_allow_reverse_{false};
  bool teleop_require_mapping_active_{true};
  bool teleop_stop_on_charging_{true};
  double teleop_charging_current_min_a_{0.10};
  double bms_charging_contact_voltage_min_v_{40.0};
  double bms_charging_contact_voltage_max_v_{1000.0};
  double bms_full_soc_threshold_pct_{99.0};
  bool bms_full_soc_voltage_contact_enable_{true};
  bool have_latest_bms_contact_started_at_{false};
  bool have_latest_bms_no_contact_started_at_{false};
  std::chrono::steady_clock::time_point latest_bms_contact_started_at_;
  std::chrono::steady_clock::time_point latest_bms_no_contact_started_at_;
  double latest_bms_contact_stable_duration_sec_{0.0};
  double latest_bms_no_contact_duration_sec_{0.0};
  double teleop_watchdog_timeout_sec_{0.5};
  double teleop_socket_idle_timeout_sec_{5.0};
  double teleop_repeat_rate_hz_{20.0};
  int subscription_default_ttl_ms_{10000};
  int subscription_max_ttl_ms_{60000};
  double service_timeout_sec_{8.0};
  double navigation_cancel_action_wait_sec_{0.75};
  double docking_stop_service_wait_sec_{3.0};
  double localization_trigger_service_timeout_sec_{15.0};
  double localization_bridge_acceptance_timeout_sec_{3.0};
  double localization_bridge_acceptance_max_distance_m_{1.0};
  double localization_bridge_acceptance_max_yaw_rad_{0.35};
  bool navigation_relocalize_before_goal_{true};
  bool navigation_relocalize_before_goal_always_{false};
  bool navigation_relocalize_before_goal_required_{true};
  double navigation_relocalize_wait_sec_{8.0};
  double navigation_goal_result_timeout_sec_{600.0};
  double navigation_goal_position_success_tolerance_m_{0.20};
  bool navigation_final_yaw_align_enable_{true};
  double navigation_final_yaw_tolerance_rad_{0.05};
  double navigation_final_yaw_align_trigger_rad_{0.08};
  double navigation_final_yaw_align_speed_radps_{0.25};
  double navigation_final_yaw_align_min_speed_radps_{0.06};
  double navigation_final_yaw_align_kp_{1.2};
  double navigation_final_yaw_align_max_speed_radps_{0.25};
  double navigation_final_yaw_align_timeout_sec_{8.0};
  double navigation_final_yaw_align_max_xy_drift_m_{0.08};
  bool navigation_final_yaw_align_require_fresh_pose_{true};
  std::string navigation_final_yaw_align_cmd_topic_{"/cmd_vel_collision_checked"};
  bool navigation_final_yaw_align_bypass_collision_monitor_{true};
  int navigation_final_yaw_align_zero_cmd_count_{3};
  double navigation_lifecycle_check_timeout_sec_{0.35};

  std::atomic<bool> running_{false};
  std::atomic<int> active_http_connections_{0};
  int server_fd_{-1};
  std::thread server_thread_;
  std::vector<std::thread> http_workers_;
  std::mutex http_queue_mutex_;
  std::condition_variable http_queue_cv_;
  std::deque<int> http_client_queue_;
  std::unique_ptr<MapCatalog> map_catalog_;
  std::unique_ptr<SubscriptionManager> subscription_manager_;
  rclcpp::TimerBase::SharedPtr subscription_ttl_timer_;
  std::mutex subscription_lifecycle_mutex_;

  std::mutex state_mutex_;
  std::string latest_safety_status_{"UNKNOWN"};
  std::string latest_floor_status_{"UNKNOWN"};
  bool latest_motion_allowed_{false};
  bool have_motion_allowed_{false};
  bool have_bms_state_{false};
  bool have_bms_soc_{false};
  double latest_bms_soc_{0.0};
  double latest_bms_voltage_{0.0};
  double latest_bms_current_{0.0};
  double latest_bms_temperature_{0.0};
  int latest_bms_power_supply_status_{sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
  int latest_bms_power_supply_health_{sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN};
  int latest_bms_power_supply_technology_{sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN};
  bool latest_bms_present_{false};
  bool latest_bms_charging_contact_{false};
  std::string latest_bms_charging_contact_reason_{"no_bms_state"};
  std::chrono::steady_clock::time_point latest_bms_received_at_{};
  std::string latest_pose_frame_;
  double latest_pose_x_{0.0};
  double latest_pose_y_{0.0};
  double latest_pose_yaw_{0.0};
  double latest_pose_stamp_sec_{0.0};
  bool have_pose_{false};
  std::chrono::steady_clock::time_point latest_pose_received_at_{};
  bool have_map_to_odom_{false};
  double latest_map_to_odom_x_{0.0};
  double latest_map_to_odom_y_{0.0};
  double latest_map_to_odom_yaw_{0.0};
  double latest_map_to_odom_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_map_to_odom_received_at_{};
  bool have_odom_to_base_{false};
  double latest_odom_to_base_x_{0.0};
  double latest_odom_to_base_y_{0.0};
  double latest_odom_to_base_yaw_{0.0};
  double latest_odom_to_base_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_odom_to_base_received_at_{};
  bool have_localization_result_{false};
  std::uint64_t latest_localization_result_seq_{0U};
  std::string latest_localization_result_frame_;
  double latest_localization_result_x_{0.0};
  double latest_localization_result_y_{0.0};
  double latest_localization_result_yaw_{0.0};
  double latest_localization_result_stamp_sec_{0.0};
  std::chrono::steady_clock::time_point latest_localization_result_received_at_{};
  bool have_scan_{false};
  std::string latest_scan_frame_;
  std::size_t latest_scan_range_count_{0U};
  double latest_scan_angle_min_{0.0};
  double latest_scan_angle_max_{0.0};
  std::chrono::steady_clock::time_point latest_scan_received_at_{};
  bool have_base_to_lidar_static_tf_{false};
  std::chrono::steady_clock::time_point base_to_lidar_static_tf_received_at_{};

  mutable std::mutex bridge_status_mutex_;
  BridgeStatusSnapshot latest_bridge_status_;
  mutable std::mutex local_costmap_mutex_;
  std::uint64_t local_costmap_update_count_{0U};
  std::chrono::steady_clock::time_point latest_local_costmap_received_at_{};
  mutable std::mutex rosout_mutex_;
  std::uint64_t message_filter_drop_count_{0U};
  std::uint64_t local_costmap_message_filter_drop_count_{0U};
  std::string last_message_filter_drop_text_;
  std::string last_local_costmap_message_filter_drop_text_;
  mutable std::mutex post_relocalization_settle_mutex_;
  PostRelocalizationSettleState post_relocalization_settle_state_;

  mutable std::mutex runtime_mode_mutex_;
  bool mapping_runtime_active_{false};
  bool navigation_runtime_active_{false};
  bool docking_runtime_active_{false};
  bool runtime_healthy_{true};
  std::string mapping_runtime_state_{"stopped"};
  std::string navigation_runtime_state_{"stopped"};
  std::string docking_runtime_state_{"stopped"};
  std::string docking_runtime_status_;
  std::string docking_runtime_dock_id_;
  std::string runtime_message_;

  std::mutex teleop_mutex_;
  int teleop_session_count_{0};
  geometry_msgs::msg::Twist latest_teleop_cmd_;
  bool teleop_command_active_{false};
  bool teleop_zero_sent_{true};
  std::chrono::steady_clock::time_point latest_teleop_cmd_at_{};

  std::mutex mapping_process_mutex_;
  pid_t mapping_2d_pid_{-1};
  bool mapping_2d_active_{false};
  std::chrono::steady_clock::time_point mapping_2d_started_at_{};

  std::mutex navigation_process_mutex_;
  pid_t navigation_resume_pid_{-1};
  std::mutex docking_manager_process_mutex_;
  pid_t docking_manager_pid_{-1};
  std::mutex navigation_cancel_start_mutex_;
  std::mutex navigation_goal_start_mutex_;
  std::mutex navigation_goal_job_mutex_;
  NavigationGoalJob navigation_goal_job_;
  std::uint64_t navigation_goal_job_seq_{0U};
  std::thread navigation_goal_worker_;
  std::mutex navigation_cancel_job_mutex_;
  NavigationCancelJob navigation_cancel_job_;
  std::uint64_t navigation_cancel_job_seq_{0U};
  std::thread navigation_cancel_worker_;
  std::mutex docking_start_mutex_;
  std::mutex docking_job_mutex_;
  std::mutex docking_relocalization_worker_mutex_;
  DockingJob docking_job_;
  std::uint64_t docking_job_seq_{0U};
  std::thread docking_worker_;
  std::thread docking_relocalization_worker_;

  std::mutex navigate_action_mutex_;
  std::mutex active_nav_goal_mutex_;
  NavigateGoalHandle::SharedPtr active_nav_goal_handle_;
  std::string active_nav_goal_pose_id_;
  std::string active_nav_goal_building_id_;
  std::string active_nav_goal_floor_id_;

  std::mutex live_map_mutex_;
  nav_msgs::msg::OccupancyGrid latest_live_map_;
  bool have_live_map_{false};
  std::chrono::steady_clock::time_point latest_live_map_received_at_{};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navigation_final_yaw_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_reverse_enable_pub_;
  rclcpp::TimerBase::SharedPtr teleop_repeat_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_allowed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bms_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_bridge_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr live_map_sub_;
  bool live_map_page_subscription_active_{false};
  bool live_map_mapping_cache_active_{false};
  rclcpp::Client<robot_interfaces::srv::SwitchFloor>::SharedPtr floor_switch_client_;
  rclcpp::Client<robot_interfaces::srv::TriggerLocalization>::SharedPtr localization_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localization_bridge_force_accept_client_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    navigation_lifecycle_clients_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr docking_undock_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<RobotApiServerNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok()) {
      try {
        executor.spin();
        break;
      } catch (const std::runtime_error & exc) {
        if (!is_transient_action_client_exception(exc)) {
          throw;
        }
        RCLCPP_ERROR(
          node->get_logger(),
          "continuing after transient action client executor exception: %s",
          exc.what());
        std::this_thread::sleep_for(100ms);
      }
    }
  } catch (const std::exception & exc) {
    std::cerr << "robot_api_server fatal exception: " << exc.what() << std::endl;
    exit_code = 1;
  } catch (...) {
    std::cerr << "robot_api_server unknown fatal exception" << std::endl;
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
