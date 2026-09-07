#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_global_localization/isaac_asset_reloader.hpp"
#include "robot_global_localization/post_reload_readiness_gate.hpp"
#include "robot_global_localization/ros_component_manager_port.hpp"
#include "robot_interfaces/msg/localizer_asset_state.hpp"
#include "robot_interfaces/srv/apply_floor_assets.hpp"
#include "robot_interfaces/srv/trigger_localization.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("robot_global_localization"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<bool>("mock_mode", false);
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<std::string>("pose_topic", "/global_localization/pose");
    declare_parameter<std::string>("health_topic", "/global_localization/health");
    declare_parameter<std::string>("default_floor_id", "floor_1");
    declare_parameter<std::string>(
      "grid_search_trigger_service", "/trigger_grid_search_localization");
    declare_parameter<double>("service_timeout_sec", 10.0);
    declare_parameter<double>("service_call_timeout_sec", 10.0);
    declare_parameter<double>("result_wait_timeout_sec", 20.0);
    declare_parameter<double>("bridge_accept_timeout_sec", 12.0);
    declare_parameter<double>("map_to_odom_wait_timeout_sec", 8.0);
    declare_parameter<double>("map_to_odom_max_age_ms", 1000.0);
    declare_parameter<bool>("localizer_input_freshness_enabled", true);
    declare_parameter<std::string>("localizer_input_topic", "/flatscan");
    declare_parameter<double>("localizer_input_wait_timeout_sec", 1.0);
    declare_parameter<double>("localizer_input_max_age_sec", 0.5);
    declare_parameter<double>("localizer_input_min_fov_deg", 115.0);
    declare_parameter<int>("localizer_input_required_consecutive_good", 2);
    declare_parameter<double>("post_reload_minimum_settle_sec", 1.0);
    declare_parameter<double>("post_reload_readiness_timeout_sec", 8.0);
    declare_parameter<int>("post_reload_required_service_ready_samples", 3);
    declare_parameter<double>("result_allowed_pretrigger_age_sec", 1.0);
    declare_parameter<bool>("require_grid_search_trigger", true);
    declare_parameter<bool>("require_bridge_acceptance", true);
    declare_parameter<std::string>("localization_result_topic", "/localization_result");
    declare_parameter<std::string>("bridge_status_topic", "/localization/bridge_status");
    declare_parameter<std::string>(
      "bridge_force_accept_service", "/robot_localization_bridge/force_accept_next_localization");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>(
      "asset_state_topic", "/global_localization/asset_state");
    declare_parameter<std::string>(
      "floor_asset_root", "/workspaces/njrh-v3/workspace1/maps_release");
    declare_parameter<std::string>(
      "runtime_map_context_file", "/tmp/njrh_runtime_map_context.json");
    declare_parameter<double>("runtime_context_bootstrap_retry_sec", 5.0);
    declare_parameter<std::string>(
      "localizer_component_container", "/occupancy_grid_localizer_container");
    declare_parameter<std::string>(
      "localizer_component_full_name", "/occupancy_grid_localizer");
    declare_parameter<std::string>(
      "localizer_component_package", "isaac_ros_occupancy_grid_localizer");
    declare_parameter<std::string>(
      "localizer_component_plugin",
      "nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode");
    declare_parameter<std::string>(
      "localizer_component_node_name", "occupancy_grid_localizer");
    declare_parameter<std::string>("localizer_component_node_namespace", "");
    declare_parameter<int>("localizer_component_log_level", 0);
    declare_parameter<std::vector<std::string>>(
      "localizer_component_remap_rules",
      std::vector<std::string>{
        "flatscan:=/flatscan",
        "localization_result:=/localization_result",
      });
    declare_parameter<double>("localizer_component_timeout_sec", 5.0);
    declare_parameter<int>("floor_asset_max_yaml_bytes", 2 * 1024 * 1024);
    declare_parameter<int>("floor_asset_max_png_bytes", 512 * 1024 * 1024);

    active_floor_id_ = get_parameter("default_floor_id").as_string();
    grid_search_trigger_service_ = get_parameter("grid_search_trigger_service").as_string();
    service_timeout_sec_ = get_parameter("service_timeout_sec").as_double();
    service_call_timeout_sec_ = get_parameter("service_call_timeout_sec").as_double();
    result_wait_timeout_sec_ = get_parameter("result_wait_timeout_sec").as_double();
    bridge_accept_timeout_sec_ = get_parameter("bridge_accept_timeout_sec").as_double();
    map_to_odom_wait_timeout_sec_ = get_parameter("map_to_odom_wait_timeout_sec").as_double();
    map_to_odom_max_age_ms_ = get_parameter("map_to_odom_max_age_ms").as_double();
    localizer_input_freshness_enabled_ =
      get_parameter("localizer_input_freshness_enabled").as_bool();
    localizer_input_topic_ = get_parameter("localizer_input_topic").as_string();
    localizer_input_wait_timeout_sec_ =
      get_parameter("localizer_input_wait_timeout_sec").as_double();
    localizer_input_max_age_sec_ = get_parameter("localizer_input_max_age_sec").as_double();
    localizer_input_min_fov_deg_ = get_parameter("localizer_input_min_fov_deg").as_double();
    localizer_input_required_consecutive_good_ = std::max(
      1, static_cast<int>(get_parameter("localizer_input_required_consecutive_good").as_int()));
    robot_global_localization::PostReloadReadinessConfig post_reload_readiness_config;
    post_reload_readiness_config.minimum_settle_sec =
      get_parameter("post_reload_minimum_settle_sec").as_double();
    post_reload_readiness_config.timeout_sec =
      get_parameter("post_reload_readiness_timeout_sec").as_double();
    post_reload_readiness_config.input_max_age_sec = localizer_input_max_age_sec_;
    post_reload_readiness_config.required_consecutive_service_ready_samples =
      static_cast<std::size_t>(std::max<std::int64_t>(
          1, get_parameter("post_reload_required_service_ready_samples").as_int()));
    post_reload_readiness_gate_ =
      std::make_unique<robot_global_localization::PostReloadReadinessGate>(
      post_reload_readiness_config);
    result_allowed_pretrigger_age_sec_ =
      get_parameter("result_allowed_pretrigger_age_sec").as_double();
    require_grid_search_trigger_ = get_parameter("require_grid_search_trigger").as_bool();
    require_bridge_acceptance_ = get_parameter("require_bridge_acceptance").as_bool();
    mock_mode_ = get_parameter("mock_mode").as_bool();
    localization_result_topic_ = get_parameter("localization_result_topic").as_string();
    bridge_status_topic_ = get_parameter("bridge_status_topic").as_string();
    bridge_force_accept_service_ = get_parameter("bridge_force_accept_service").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    runtime_map_context_file_ =
      get_parameter("runtime_map_context_file").as_string();
    runtime_context_bootstrap_retry_sec_ = positive_or_default(
      get_parameter("runtime_context_bootstrap_retry_sec").as_double(), 5.0);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), rclcpp::QoS(10));
    health_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("health_topic").as_string(), rclcpp::QoS(10));
    auto asset_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    asset_state_qos.reliable();
    asset_state_qos.transient_local();
    asset_state_pub_ = create_publisher<robot_interfaces::msg::LocalizerAssetState>(
      get_parameter("asset_state_topic").as_string(), asset_state_qos);

    robot_global_localization::IsaacAssetReloaderOptions reloader_options;
    reloader_options.allowed_asset_root =
      get_parameter("floor_asset_root").as_string();
    reloader_options.expected_full_node_name =
      get_parameter("localizer_component_full_name").as_string();
    reloader_options.component.package_name =
      get_parameter("localizer_component_package").as_string();
    reloader_options.component.plugin_name =
      get_parameter("localizer_component_plugin").as_string();
    reloader_options.component.node_name =
      get_parameter("localizer_component_node_name").as_string();
    reloader_options.component.node_namespace =
      get_parameter("localizer_component_node_namespace").as_string();
    const auto configured_log_level =
      get_parameter("localizer_component_log_level").as_int();
    reloader_options.component.log_level = static_cast<std::uint8_t>(
      std::clamp<std::int64_t>(configured_log_level, 0, 255));
    reloader_options.component.remap_rules =
      get_parameter("localizer_component_remap_rules").as_string_array();
    reloader_options.max_yaml_bytes = static_cast<std::uintmax_t>(
      std::max<std::int64_t>(
        1, get_parameter("floor_asset_max_yaml_bytes").as_int()));
    reloader_options.max_png_bytes = static_cast<std::uintmax_t>(
      std::max<std::int64_t>(
        1, get_parameter("floor_asset_max_png_bytes").as_int()));
    asset_reloader_ =
      std::make_unique<robot_global_localization::IsaacAssetReloader>(
      reloader_options);

    robot_global_localization::RosComponentManagerOptions component_options;
    component_options.container_name =
      get_parameter("localizer_component_container").as_string();
    component_options.expected_full_node_name =
      get_parameter("localizer_component_full_name").as_string();
    component_options.operation_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        positive_or_default(
          get_parameter("localizer_component_timeout_sec").as_double(), 5.0)));
    component_manager_ =
      std::make_unique<robot_global_localization::RosComponentManagerPort>(
      *this, callback_group_, component_options);

    grid_search_trigger_client_ = create_client<std_srvs::srv::Empty>(
      grid_search_trigger_service_, rmw_qos_profile_services_default, callback_group_);
    bridge_force_accept_client_ = create_client<std_srvs::srv::Trigger>(
      bridge_force_accept_service_, rmw_qos_profile_services_default, callback_group_);
    localization_result_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      localization_result_topic_,
      rclcpp::QoS(20),
      std::bind(&GlobalLocalizationNode::on_localization_result, this, std::placeholders::_1));
    bridge_status_sub_ = create_subscription<std_msgs::msg::String>(
      bridge_status_topic_,
      rclcpp::QoS(10),
      std::bind(&GlobalLocalizationNode::on_bridge_status, this, std::placeholders::_1));

    trigger_srv_ = create_service<robot_interfaces::srv::TriggerLocalization>(
      "/global_localization/trigger",
      std::bind(&GlobalLocalizationNode::on_trigger, this, std::placeholders::_1,
      std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);
    apply_floor_srv_ = create_service<robot_interfaces::srv::ApplyFloorAssets>(
      "/global_localization/apply_floor_assets",
      std::bind(&GlobalLocalizationNode::on_apply_floor, this, std::placeholders::_1,
      std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalLocalizationNode::on_timer, this),
      callback_group_);
    runtime_context_bootstrap_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalLocalizationNode::on_runtime_context_bootstrap, this),
      callback_group_);

    robot_global_localization::LocalizerAssetState initial_state;
    initial_state.detail = "no floor asset transaction has completed in this process";
    publish_asset_state(initial_state, false, "UNINITIALIZED");
  }

private:
  struct LocalizationResultSnapshot
  {
    bool available{false};
    std::uint64_t seq{0U};
    double received_sec{0.0};
    double header_stamp_sec{0.0};
    std::string frame_id;
  };

  struct LocalizerInputSnapshot
  {
    bool available{false};
    std::uint64_t seq{0U};
    double received_sec{0.0};
    double header_stamp_sec{0.0};
    double fov_deg{0.0};
    std::size_t point_count{0U};
    std::string topic_type;
  };

  struct BridgeStatusSnapshot
  {
    bool available{false};
    double received_sec{0.0};
    std::uint64_t accepted_result_count{0U};
    std::uint64_t rejected_result_count{0U};
    std::uint64_t last_explicit_relocalization_sequence{0U};
    std::uint64_t last_explicit_map_odom_target_sequence{0U};
    std::uint64_t current_sequence{0U};
    std::uint64_t target_sequence{0U};
    std::uint64_t amcl_post_isaac_refined_sequence{0U};
    bool has_map_to_odom{false};
    bool safe_for_goal_start{false};
    bool correction_active{false};
    double map_to_odom_age_ms{-1.0};
    double map_odom_publish_gap_ms{-1.0};
    double remaining_translation_error_m{-1.0};
    double remaining_yaw_error_rad{-1.0};
    std::uint64_t force_accept_ignored_pretrigger_result_count{0U};
    std::string owner;
    std::string gate_mode;
    std::string last_accept_reason;
    std::string last_reject_reason;
    std::string last_force_accept_ignored_reason;
    std::string last_explicit_relocalization_source;
    std::string current_source;
    std::string target_source;
    std::string raw;
  };

  void on_trigger(
    const std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Request> request,
    std::shared_ptr<robot_interfaces::srv::TriggerLocalization::Response> response)
  {
    std::unique_lock<std::mutex> localizer_operation_lock(
      localizer_operation_mutex_, std::try_to_lock);
    if (!localizer_operation_lock.owns_lock()) {
      response->accepted = false;
      response->message =
        "failure_code=LOCALIZER_OPERATION_BUSY dispatch_state=not_dispatched "
        "floor asset reload or another trigger is active";
      return;
    }

    std::string post_reload_readiness_detail;
    if (!wait_for_post_reload_readiness(post_reload_readiness_detail)) {
      response->accepted = false;
      response->message = post_reload_readiness_detail;
      last_trigger_status_ = response->message;
      return;
    }

    const double service_call_timeout_sec =
      positive_or_default(service_call_timeout_sec_, std::min(service_timeout_sec_, 5.0));
    const double result_wait_timeout_sec =
      positive_or_default(result_wait_timeout_sec_, std::max(service_timeout_sec_, 20.0));
    const double bridge_accept_timeout_sec =
      positive_or_default(bridge_accept_timeout_sec_, 12.0);
    const double map_to_odom_wait_timeout_sec =
      positive_or_default(map_to_odom_wait_timeout_sec_, 8.0);

    const auto service_timeout = std::chrono::duration<double>(service_call_timeout_sec);
    if (!grid_search_trigger_client_->wait_for_service(service_timeout)) {
      response->accepted = !require_grid_search_trigger_;
      response->message =
        "failure_code=ISAAC_SERVICE_UNAVAILABLE dispatch_state=not_dispatched "
        "service unavailable: " + grid_search_trigger_service_;
      return;
    }

    std::string input_detail;
    if (!wait_for_fresh_localizer_input(input_detail)) {
      response->accepted = false;
      response->message = input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    const auto pre_arm_input = localizer_input_snapshot();
    std::string force_accept_detail;
    const bool force_accept_armed =
      arm_bridge_force_accept(request->reason, service_call_timeout_sec, force_accept_detail);
    const double force_accept_ready_sec = now().seconds();

    if (!force_accept_armed) {
      response->accepted = false;
      response->message = force_accept_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string post_arm_input_detail;
    if (!wait_for_localizer_input_after_arm(
        pre_arm_input, force_accept_ready_sec, post_arm_input_detail))
    {
      input_detail += "; " + post_arm_input_detail + "; continuing with the latest fresh input";
    } else {
      input_detail += "; " + post_arm_input_detail;
    }

    const auto trigger_started_sec = now().seconds();
    const auto initial_result = localization_result_snapshot();
    const auto initial_bridge = bridge_status_snapshot();

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = grid_search_trigger_client_->async_send_request(empty_request);
    last_trigger_status_ = "grid_search_trigger_pending: " + request->reason;

    std::string direct_service_detail;
    const auto service_deadline = steady_deadline(service_call_timeout_sec);
    while (std::chrono::steady_clock::now() <= service_deadline) {
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          (void)future.get();
          direct_service_detail = "isaac direct service returned";
          break;
        } catch (const std::exception & exc) {
          response->accepted = false;
          response->message =
            std::string("failure_code=ISAAC_DISPATCH_FAILED dispatch_state=dispatched ") +
            "direct service failed: " + exc.what();
          last_trigger_status_ = response->message;
          return;
        }
      }
      if (
        localization_result_observed_after(initial_result, trigger_started_sec) ||
        bridge_processed_after(initial_bridge, trigger_started_sec))
      {
        direct_service_detail = "isaac direct service response pending; localization result already observed";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (direct_service_detail.empty()) {
      response->accepted = false;
      response->message =
        "failure_code=ISAAC_DISPATCH_TIMEOUT dispatch_state=dispatched "
        "direct service did not return or produce localization_result within " +
        std::to_string(service_call_timeout_sec) + "s";
      last_trigger_status_ = response->message;
      return;
    }

    if (!wait_for_localization_result_or_bridge_processing(
        initial_result, initial_bridge, trigger_started_sec, result_wait_timeout_sec))
    {
      response->accepted = false;
      response->message =
        "failure_code=LOCALIZATION_RESULT_TIMEOUT dispatch_state=dispatched "
        "no current-arm localization_result or explicit bridge acceptance within " +
        std::to_string(result_wait_timeout_sec) + "s; " + force_accept_detail + "; " +
        direct_service_detail + "; " + input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string bridge_detail;
    if (require_bridge_acceptance_ && !wait_for_bridge_acceptance(
        initial_bridge,
        trigger_started_sec,
        bridge_accept_timeout_sec,
        force_accept_armed,
        bridge_detail))
    {
      response->accepted = false;
      response->message =
        bridge_detail + "; " + force_accept_detail + "; " + direct_service_detail + "; " +
        input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    std::string map_to_odom_detail;
    if (!wait_for_map_to_odom(map_to_odom_wait_timeout_sec, map_to_odom_detail)) {
      response->accepted = false;
      response->message =
        map_to_odom_detail + "; " + force_accept_detail + "; " + direct_service_detail + "; " +
        input_detail;
      last_trigger_status_ = response->message;
      return;
    }

    response->accepted = true;
    std::ostringstream out;
    out << "triggered relocalization accepted"
        << " explicit_trigger=" << (force_accept_armed ? "true" : "false")
        << "; " << force_accept_detail
        << "; " << direct_service_detail
        << "; " << input_detail
        << "; " << bridge_detail
        << "; " << map_to_odom_detail
        << "; reason=" << request->reason;
    response->message = out.str();
    last_trigger_status_ = response->message;
    RCLCPP_INFO(get_logger(), "%s", last_trigger_status_.c_str());
  }

  void on_apply_floor(
    const std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Request> request,
    std::shared_ptr<robot_interfaces::srv::ApplyFloorAssets::Response> response)
  {
    response->transaction_id = request->transaction_id;
    response->building_id = request->building_id;
    response->floor_id = request->floor_id;
    response->map_id = request->map_id;
    response->asset_epoch = request->asset_epoch;
    response->asset_digest = request->asset_digest;

    std::unique_lock<std::mutex> localizer_operation_lock(
      localizer_operation_mutex_, std::try_to_lock);
    if (!localizer_operation_lock.owns_lock()) {
      response->success = false;
      response->code = "LOCALIZER_OPERATION_BUSY";
      response->message =
        "floor asset reload or explicit localization trigger is already active";
      response->localizer_generation = last_asset_generation();
      return;
    }

    robot_global_localization::FloorAssetRequest domain_request;
    domain_request.transaction_id = request->transaction_id;
    domain_request.identity.building_id = request->building_id;
    domain_request.identity.floor_id = request->floor_id;
    domain_request.identity.map_id = request->map_id;
    domain_request.identity.asset_epoch = request->asset_epoch;
    domain_request.identity.asset_digest = request->asset_digest;
    domain_request.nav_map_yaml = request->nav_map_yaml;
    domain_request.localizer_map_png = request->localizer_map_png;
    domain_request.localizer_params_yaml = request->localizer_params_yaml;

    auto applying_state = asset_reloader_->state();
    applying_state.transaction_id = domain_request.transaction_id;
    applying_state.requested_identity = domain_request.identity;
    applying_state.applying = true;
    applying_state.idempotent = false;
    applying_state.reloaded = false;
    applying_state.rollback_attempted = false;
    applying_state.rollback_succeeded = false;
    applying_state.failure_code.clear();
    applying_state.detail = "validating and replacing Isaac localizer floor assets";
    publish_asset_state(applying_state, false, "APPLYING");

    robot_global_localization::ApplyFloorAssetResult result;
    try {
      result = asset_reloader_->apply(domain_request, *component_manager_);
    } catch (const std::exception & exception) {
      result.success = false;
      result.idempotent = false;
      result.state = asset_reloader_->state();
      result.state.transaction_id = domain_request.transaction_id;
      result.state.requested_identity = domain_request.identity;
      result.state.applying = false;
      result.state.localizer_ready = false;
      result.state.failure_code = "LOCALIZER_RELOAD_EXCEPTION";
      result.state.detail = exception.what();
    }

    if (result.success && result.state.reloaded && !result.idempotent) {
      const auto input_baseline = localizer_input_snapshot();
      // Keep this client alive for the node lifetime. ROS 2 service clients
      // rediscover a same-named server after the Isaac component is replaced.
      // Recreating a waitable from this Reentrant callback mutates the callback
      // group while the MultiThreadedExecutor is spinning and can race inside
      // rclcpp::CallbackGroup::add_waitable().
      post_reload_readiness_gate_->arm(
        result.state.localizer_generation,
        input_baseline.seq,
        steady_now_seconds());
      result.state.detail +=
        "; post-reload trigger readiness gate armed generation=" +
        std::to_string(result.state.localizer_generation) +
        " input_baseline_seq=" + std::to_string(input_baseline.seq);
    }

    response->success = result.success;
    response->idempotent = result.idempotent;
    response->reloaded = result.state.reloaded;
    response->rollback_attempted = result.state.rollback_attempted;
    response->rollback_succeeded = result.state.rollback_succeeded;
    response->localizer_generation = result.state.localizer_generation;
    response->code = result.success ?
      (result.idempotent ? "IDEMPOTENT" : "OK") :
      (result.state.failure_code.empty() ?
      "LOCALIZER_RELOAD_FAILED" : result.state.failure_code);
    response->message = result.state.detail;

    if (result.success && result.state.active_identity_valid) {
      set_active_assets(result.state);
    }
    publish_asset_state(result.state, result.success, response->code);
  }

  std::uint64_t last_asset_generation()
  {
    std::lock_guard<std::mutex> lock(asset_state_mutex_);
    return last_asset_state_.localizer_generation;
  }

  void publish_asset_state(
    const robot_global_localization::LocalizerAssetState & state,
    const bool success,
    const std::string & code)
  {
    robot_interfaces::msg::LocalizerAssetState message;
    message.stamp = now();
    message.transaction_id = state.transaction_id;
    message.applying = state.applying;
    message.success = success;
    message.idempotent = state.idempotent;
    message.reloaded = state.reloaded;
    message.rollback_attempted = state.rollback_attempted;
    message.rollback_succeeded = state.rollback_succeeded;
    message.code = code;
    message.message = state.detail;
    message.requested_building_id = state.requested_identity.building_id;
    message.requested_floor_id = state.requested_identity.floor_id;
    message.requested_map_id = state.requested_identity.map_id;
    message.requested_asset_epoch = state.requested_identity.asset_epoch;
    message.requested_asset_digest = state.requested_identity.asset_digest;
    message.active_identity_valid = state.active_identity_valid;
    message.active_building_id = state.active_identity.building_id;
    message.active_floor_id = state.active_identity.floor_id;
    message.active_map_id = state.active_identity.map_id;
    message.active_asset_epoch = state.active_identity.asset_epoch;
    message.active_asset_digest = state.active_identity.asset_digest;
    message.active_nav_map_yaml = state.active_nav_map_yaml.string();
    message.active_localizer_map_png = state.active_localizer_map_png.string();
    message.active_localizer_params_yaml =
      state.active_localizer_params_yaml.string();
    message.localizer_generation = state.localizer_generation;
    message.localizer_ready = state.localizer_ready;
    {
      std::lock_guard<std::mutex> lock(asset_state_mutex_);
      last_asset_state_ = message;
    }
    asset_state_pub_->publish(message);
  }

  void set_active_assets(
    const robot_global_localization::LocalizerAssetState & state)
  {
    std::lock_guard<std::mutex> lock(active_asset_mutex_);
    active_floor_id_ = state.active_identity.floor_id;
    active_nav_map_yaml_ = state.active_nav_map_yaml.string();
    active_localizer_map_png_ = state.active_localizer_map_png.string();
    active_localizer_params_yaml_ =
      state.active_localizer_params_yaml.string();
  }

  std::string active_floor_id_snapshot()
  {
    std::lock_guard<std::mutex> lock(active_asset_mutex_);
    return active_floor_id_;
  }

  void on_runtime_context_bootstrap()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now < next_runtime_context_bootstrap_attempt_) {
      return;
    }

    std::unique_lock<std::mutex> localizer_operation_lock(
      localizer_operation_mutex_, std::try_to_lock);
    if (!localizer_operation_lock.owns_lock()) {
      return;
    }

    const auto current_state = asset_reloader_->state();
    if (
      current_state.active_identity_valid &&
      current_state.localizer_ready &&
      current_state.localizer_generation > 0U)
    {
      runtime_context_bootstrap_timer_->cancel();
      return;
    }

    auto checking_state = current_state;
    checking_state.applying = true;
    checking_state.idempotent = false;
    checking_state.reloaded = false;
    checking_state.rollback_attempted = false;
    checking_state.rollback_succeeded = false;
    checking_state.failure_code.clear();
    checking_state.detail =
      "verifying durable runtime context against exact current assets "
      "and the unique live Isaac component";
    publish_asset_state(checking_state, false, "BOOTSTRAP_CHECKING");

    robot_global_localization::ApplyFloorAssetResult result;
    try {
      result = asset_reloader_->bootstrap_from_runtime_context(
        std::filesystem::path(runtime_map_context_file_),
        *component_manager_);
    } catch (const std::exception & exception) {
      result.success = false;
      result.idempotent = false;
      result.state = asset_reloader_->state();
      result.state.applying = false;
      result.state.active_identity_valid = false;
      result.state.localizer_ready = false;
      result.state.failure_code = "EXCEPTION";
      result.state.detail = exception.what();
    }

    if (
      result.success &&
      result.state.active_identity_valid &&
      result.state.localizer_ready &&
      result.state.localizer_generation > 0U)
    {
      set_active_assets(result.state);
      publish_asset_state(result.state, true, "BOOTSTRAP_READY");
      runtime_context_bootstrap_timer_->cancel();
      RCLCPP_INFO(
        get_logger(),
        "Localizer identity bootstrap ready building=%s floor=%s map=%s "
        "epoch=%llu generation=%llu",
        result.state.active_identity.building_id.c_str(),
        result.state.active_identity.floor_id.c_str(),
        result.state.active_identity.map_id.c_str(),
        static_cast<unsigned long long>(
          result.state.active_identity.asset_epoch),
        static_cast<unsigned long long>(
          result.state.localizer_generation));
      return;
    }

    const std::string failure_code = result.state.failure_code.empty() ?
      "FAILED" : result.state.failure_code;
    publish_asset_state(
      result.state, false, "BOOTSTRAP_" + failure_code);
    RCLCPP_WARN(
      get_logger(), "Localizer identity bootstrap failed [%s]: %s",
      failure_code.c_str(), result.state.detail.c_str());
    next_runtime_context_bootstrap_attempt_ =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(runtime_context_bootstrap_retry_sec_));
  }

  void on_timer()
  {
    const bool trigger_ready = grid_search_trigger_client_->service_is_ready();
    const std::string active_floor_id = active_floor_id_snapshot();
    if (mock_mode_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      const int64_t now_ns = get_clock()->now().nanoseconds();
      pose.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      pose.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
      pose.header.frame_id = "map";
      pose_pub_->publish(pose);
      std_msgs::msg::String health;
      health.data = "mock_localizer_ready floor=" + active_floor_id;
      health_pub_->publish(health);
      return;
    }

    std_msgs::msg::String health;
    const std::string status = trigger_ready ? "localizer_ready" :
      "localizer_waiting_for_grid_search";
    health.data = status + " floor=" + active_floor_id +
      " trigger_status=" + last_trigger_status_;
    health_pub_->publish(health);
  }

  void on_localization_result(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    localization_result_.available = true;
    localization_result_.seq++;
    localization_result_.received_sec = now().seconds();
    localization_result_.header_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
    localization_result_.frame_id = msg->header.frame_id;
  }

  void on_bridge_status(const std_msgs::msg::String::SharedPtr msg)
  {
    BridgeStatusSnapshot snapshot;
    snapshot.available = true;
    snapshot.received_sec = now().seconds();
    snapshot.raw = msg->data;
    snapshot.accepted_result_count = json_uint_value(msg->data, "accepted_result_count", 0U);
    snapshot.rejected_result_count = json_uint_value(msg->data, "rejected_result_count", 0U);
    snapshot.force_accept_ignored_pretrigger_result_count =
      json_uint_value(msg->data, "force_accept_ignored_pretrigger_result_count", 0U);
    snapshot.last_explicit_relocalization_sequence =
      json_uint_value(msg->data, "last_explicit_relocalization_sequence", 0U);
    snapshot.last_explicit_map_odom_target_sequence =
      json_uint_value(msg->data, "last_explicit_map_odom_target_sequence", 0U);
    snapshot.current_sequence = json_uint_value(msg->data, "current_sequence", 0U);
    snapshot.target_sequence = json_uint_value(msg->data, "target_sequence", 0U);
    snapshot.amcl_post_isaac_refined_sequence =
      json_uint_value(msg->data, "amcl_post_isaac_refined_sequence", 0U);
    snapshot.has_map_to_odom = json_bool_value(msg->data, "has_map_to_odom", false);
    snapshot.safe_for_goal_start = json_bool_value(msg->data, "safe_for_goal_start", false);
    snapshot.correction_active = json_bool_value(msg->data, "correction_active", false);
    snapshot.map_to_odom_age_ms = json_double_value(msg->data, "map_to_odom_age_ms", -1.0);
    snapshot.map_odom_publish_gap_ms = json_double_value(msg->data, "map_odom_publish_gap_ms", -1.0);
    snapshot.remaining_translation_error_m =
      json_double_value(msg->data, "remaining_translation_error_m", -1.0);
    snapshot.remaining_yaw_error_rad = json_double_value(msg->data, "remaining_yaw_error_rad", -1.0);
    snapshot.owner = json_string_value(msg->data, "map_to_odom_publisher_owner");
    snapshot.gate_mode = json_string_value(msg->data, "gate_mode");
    snapshot.last_accept_reason = json_string_value(msg->data, "last_accept_reason");
    snapshot.last_reject_reason = json_string_value(msg->data, "last_reject_reason");
    snapshot.last_force_accept_ignored_reason =
      json_string_value(msg->data, "last_force_accept_ignored_reason");
    snapshot.last_explicit_relocalization_source =
      json_string_value(msg->data, "last_explicit_relocalization_source");
    snapshot.current_source = json_string_value(msg->data, "current_source");
    snapshot.target_source = json_string_value(msg->data, "target_source");

    std::lock_guard<std::mutex> lock(state_mutex_);
    bridge_status_ = snapshot;
  }

  static double localizer_input_fov_deg(
    const isaac_ros_pointcloud_interfaces::msg::FlatScan & msg)
  {
    double min_angle = std::numeric_limits<double>::infinity();
    double max_angle = -std::numeric_limits<double>::infinity();
    for (const float angle : msg.angles) {
      if (!std::isfinite(angle)) {
        continue;
      }
      min_angle = std::min(min_angle, static_cast<double>(angle));
      max_angle = std::max(max_angle, static_cast<double>(angle));
    }
    if (!std::isfinite(min_angle) || !std::isfinite(max_angle) || max_angle < min_angle) {
      return 0.0;
    }
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return (max_angle - min_angle) * kRadiansToDegrees;
  }

  void on_localizer_input(
    const isaac_ros_pointcloud_interfaces::msg::FlatScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    localizer_input_.available = true;
    localizer_input_.seq++;
    localizer_input_.received_sec = now().seconds();
    localizer_input_.header_stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
    localizer_input_.fov_deg = localizer_input_fov_deg(*msg);
    localizer_input_.point_count = msg->angles.size();
  }

  static double positive_or_default(const double value, const double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  static std::chrono::steady_clock::time_point steady_deadline(const double timeout_sec)
  {
    return std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(timeout_sec));
  }

  static std::uint64_t json_uint_value(
    const std::string & data,
    const std::string & key,
    const std::uint64_t fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    try {
      return static_cast<std::uint64_t>(std::stoull(data.substr(colon + 1)));
    } catch (...) {
      return fallback;
    }
  }

  static double json_double_value(
    const std::string & data,
    const std::string & key,
    const double fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    try {
      return std::stod(data.substr(colon + 1));
    } catch (...) {
      return fallback;
    }
  }

  static bool json_bool_value(
    const std::string & data,
    const std::string & key,
    const bool fallback)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return fallback;
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return fallback;
    }
    const auto value_pos = data.find_first_not_of(" \t\r\n", colon + 1);
    if (value_pos == std::string::npos) {
      return fallback;
    }
    if (data.compare(value_pos, 4, "true") == 0) {
      return true;
    }
    if (data.compare(value_pos, 5, "false") == 0) {
      return false;
    }
    return fallback;
  }

  static std::string json_string_value(const std::string & data, const std::string & key)
  {
    const auto pos = data.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return "";
    }
    const auto colon = data.find(':', pos);
    if (colon == std::string::npos) {
      return "";
    }
    auto quote = data.find('"', colon + 1);
    if (quote == std::string::npos) {
      return "";
    }
    std::string value;
    bool escaped = false;
    for (auto index = quote + 1; index < data.size(); ++index) {
      const char c = data[index];
      if (escaped) {
        value += c;
        escaped = false;
        continue;
      }
      if (c == '\\') {
        escaped = true;
        continue;
      }
      if (c == '"') {
        return value;
      }
      value += c;
    }
    return "";
  }

  LocalizationResultSnapshot localization_result_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return localization_result_;
  }

  BridgeStatusSnapshot bridge_status_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return bridge_status_;
  }

  LocalizerInputSnapshot localizer_input_snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return localizer_input_;
  }

  static double steady_now_seconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool wait_for_post_reload_readiness(std::string & detail)
  {
    if (!post_reload_readiness_gate_ || !post_reload_readiness_gate_->pending()) {
      detail = "no post-reload readiness wait is pending";
      return true;
    }

    std::string subscription_detail;
    while (rclcpp::ok()) {
      if (localizer_input_freshness_enabled_ && !localizer_input_topic_.empty()) {
        (void)ensure_localizer_input_subscription(subscription_detail);
      }

      const auto input = localizer_input_snapshot();
      const double input_age_sec = input.available ?
        now().seconds() - input.received_sec : -1.0;
      const std::uint64_t observed_input_sequence =
        (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) ?
        input.seq + 1U : input.seq;
      const double observed_input_age_sec =
        (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) ?
        0.0 : input_age_sec;
      const bool service_ready =
        grid_search_trigger_client_ && grid_search_trigger_client_->service_is_ready();
      const auto decision = post_reload_readiness_gate_->observe(
        {
          steady_now_seconds(),
          service_ready,
          observed_input_sequence,
          observed_input_age_sec,
        });

      if (decision == robot_global_localization::PostReloadReadinessDecision::kReady) {
        detail = "post-reload localizer ready generation=" +
          std::to_string(post_reload_readiness_gate_->localizer_generation()) +
          " service_ready=true input_seq=" + std::to_string(input.seq) +
          " input_age_sec=" + std::to_string(input_age_sec);
        RCLCPP_INFO(get_logger(), "%s", detail.c_str());
        return true;
      }
      if (decision == robot_global_localization::PostReloadReadinessDecision::kTimedOut) {
        detail = "failure_code=LOCALIZER_POST_RELOAD_NOT_READY dispatch_state=not_dispatched generation=" +
          std::to_string(post_reload_readiness_gate_->localizer_generation()) +
          " service_ready=" + bool_string(service_ready) +
          " input_seq=" + std::to_string(input.seq) +
          " input_age_sec=" + std::to_string(input_age_sec) +
          " subscription_detail=" + subscription_detail;
        RCLCPP_WARN(get_logger(), "%s", detail.c_str());
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    detail =
      "failure_code=LOCALIZER_POST_RELOAD_NOT_READY dispatch_state=not_dispatched "
      "ROS shutdown while waiting";
    return false;
  }

  bool ensure_localizer_input_subscription(std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "localizer input freshness gate disabled";
      return true;
    }
    if (localizer_input_sub_) {
      return true;
    }

    try {
      localizer_input_sub_ =
        create_subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>(
        localizer_input_topic_,
        rclcpp::QoS(10),
        std::bind(&GlobalLocalizationNode::on_localizer_input, this, std::placeholders::_1));
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        localizer_input_.topic_type = "isaac_ros_pointcloud_interfaces/msg/FlatScan";
      }
      detail = "localizer input subscription ready topic=" + localizer_input_topic_ +
               " type=isaac_ros_pointcloud_interfaces/msg/FlatScan";
      return true;
    } catch (const std::exception & exc) {
      detail = "localizer input subscription failed topic=" + localizer_input_topic_ +
               " type=isaac_ros_pointcloud_interfaces/msg/FlatScan: " + exc.what();
      return false;
    }
  }

  bool wait_for_fresh_localizer_input(std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "localizer input freshness gate disabled";
      return true;
    }

    const double timeout_sec = positive_or_default(localizer_input_wait_timeout_sec_, 1.0);
    const double max_age_sec = positive_or_default(localizer_input_max_age_sec_, 0.5);
    const auto deadline = steady_deadline(timeout_sec);
    std::string subscription_detail;

    while (std::chrono::steady_clock::now() <= deadline) {
      if (ensure_localizer_input_subscription(subscription_detail)) {
        const auto snapshot = localizer_input_snapshot();
        if (snapshot.available) {
          const double age_sec = now().seconds() - snapshot.received_sec;
          if (
            age_sec >= 0.0 && age_sec <= max_age_sec &&
            snapshot.fov_deg >= localizer_input_min_fov_deg_)
          {
            detail = "localizer input fresh topic=" + localizer_input_topic_ +
                     " type=" + snapshot.topic_type +
                     " age_sec=" + std::to_string(age_sec) +
                     " fov_deg=" + std::to_string(snapshot.fov_deg) +
                     " points=" + std::to_string(snapshot.point_count);
            return true;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto snapshot = localizer_input_snapshot();
    detail =
      "failure_code=LOCALIZER_INPUT_NOT_FRESH dispatch_state=not_dispatched topic=" +
      localizer_input_topic_ +
             " timeout_sec=" + std::to_string(timeout_sec) +
             " max_age_sec=" + std::to_string(max_age_sec) +
             " min_fov_deg=" + std::to_string(localizer_input_min_fov_deg_) +
             " available=" + bool_string(snapshot.available) +
             " seq=" + std::to_string(snapshot.seq) +
             " fov_deg=" + std::to_string(snapshot.fov_deg) +
             " last_age_sec=" +
             (snapshot.available ? std::to_string(now().seconds() - snapshot.received_sec) : "-1") +
             " subscription_detail=" + subscription_detail;
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    return false;
  }

  bool wait_for_localizer_input_after_arm(
    const LocalizerInputSnapshot & baseline,
    const double armed_sec,
    std::string & detail)
  {
    if (!localizer_input_freshness_enabled_ || localizer_input_topic_.empty()) {
      detail = "post-arm localizer input gate disabled";
      return true;
    }

    const double timeout_sec = positive_or_default(localizer_input_wait_timeout_sec_, 1.0);
    const double max_age_sec = positive_or_default(localizer_input_max_age_sec_, 0.5);
    const double min_header_stamp_sec =
      armed_sec - positive_or_default(result_allowed_pretrigger_age_sec_, 1.0);
    const auto deadline = steady_deadline(timeout_sec);
    std::string subscription_detail;
    const int required_consecutive_good = std::max(
      1, localizer_input_required_consecutive_good_);
    int consecutive_good = 0;
    std::uint64_t last_examined_seq = baseline.seq;

    while (std::chrono::steady_clock::now() <= deadline) {
      if (ensure_localizer_input_subscription(subscription_detail)) {
        const auto snapshot = localizer_input_snapshot();
        if (snapshot.seq == last_examined_seq) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }
        last_examined_seq = snapshot.seq;
        const double age_sec = now().seconds() - snapshot.received_sec;
        const bool sample_good =
          snapshot.available &&
          snapshot.seq > baseline.seq &&
          snapshot.received_sec >= armed_sec &&
          snapshot.header_stamp_sec >= min_header_stamp_sec &&
          snapshot.fov_deg >= localizer_input_min_fov_deg_ &&
          age_sec >= 0.0 && age_sec <= max_age_sec;
        if (sample_good) {
          ++consecutive_good;
          if (consecutive_good >= required_consecutive_good) {
            detail = "post-arm localizer input ready topic=" + localizer_input_topic_ +
                     " seq=" + std::to_string(snapshot.seq) +
                     " age_sec=" + std::to_string(age_sec) +
                     " fov_deg=" + std::to_string(snapshot.fov_deg) +
                     " points=" + std::to_string(snapshot.point_count) +
                     " consecutive_good=" + std::to_string(consecutive_good);
            return true;
          }
        } else {
          consecutive_good = 0;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto snapshot = localizer_input_snapshot();
    detail = "post-arm localizer input did not advance topic=" + localizer_input_topic_ +
             " baseline_seq=" + std::to_string(baseline.seq) +
             " current_seq=" + std::to_string(snapshot.seq) +
             " armed_sec=" + std::to_string(armed_sec) +
             " min_header_stamp_sec=" + std::to_string(min_header_stamp_sec) +
             " last_header_stamp_sec=" + std::to_string(snapshot.header_stamp_sec) +
             " last_received_sec=" + std::to_string(snapshot.received_sec) +
             " fov_deg=" + std::to_string(snapshot.fov_deg) +
             " min_fov_deg=" + std::to_string(localizer_input_min_fov_deg_) +
             " consecutive_good=" + std::to_string(consecutive_good) +
             " required_consecutive_good=" + std::to_string(required_consecutive_good);
    RCLCPP_WARN(get_logger(), "%s", detail.c_str());
    return false;
  }

  bool localization_result_is_fresh_for_trigger(
    const LocalizationResultSnapshot & snapshot,
    const double trigger_started_sec) const
  {
    const double allowed_pretrigger_age_sec =
      positive_or_default(result_allowed_pretrigger_age_sec_, 1.0);
    return snapshot.header_stamp_sec >= trigger_started_sec - allowed_pretrigger_age_sec;
  }

  bool localization_result_observed_after(
    const LocalizationResultSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = localization_result_snapshot();
    return snapshot.available &&
           snapshot.seq > initial.seq &&
           snapshot.received_sec >= trigger_started_sec &&
           localization_result_is_fresh_for_trigger(snapshot, trigger_started_sec);
  }

  bool bridge_processed_after(
    const BridgeStatusSnapshot & initial,
    const double trigger_started_sec)
  {
    const auto snapshot = bridge_status_snapshot();
    return snapshot.available &&
           snapshot.received_sec >= trigger_started_sec &&
           bridge_explicit_trigger_accept_observed(initial, snapshot);
  }

  bool arm_bridge_force_accept(
    const std::string & reason,
    const double timeout_sec,
    std::string & detail)
  {
    const auto timeout = std::chrono::duration<double>(std::min(timeout_sec, 2.0));
    if (!bridge_force_accept_client_->wait_for_service(timeout)) {
      detail =
        "failure_code=BRIDGE_FORCE_ACCEPT_UNAVAILABLE dispatch_state=not_dispatched "
        "bridge force-accept unavailable for explicit_trigger reason=" + reason;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = bridge_force_accept_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail =
        "failure_code=BRIDGE_FORCE_ACCEPT_TIMEOUT dispatch_state=not_dispatched "
        "bridge force-accept timed out for explicit_trigger reason=" + reason;
      return false;
    }
    try {
      const auto response = future.get();
      if (!response->success) {
        detail =
          "failure_code=BRIDGE_FORCE_ACCEPT_FAILED dispatch_state=not_dispatched "
          "bridge force-accept rejected explicit_trigger reason=" + reason +
                  ": " + response->message;
        return false;
      }
      detail = "bridge force-accept armed explicit_trigger=true reason=" + reason +
               ": " + response->message;
      return true;
    } catch (const std::exception & exc) {
      detail =
        std::string("failure_code=BRIDGE_FORCE_ACCEPT_FAILED dispatch_state=not_dispatched ") +
        "bridge force-accept failed explicit_trigger reason=" + reason +
                ": " + exc.what();
      return false;
    }
  }

  bool wait_for_localization_result_or_bridge_processing(
    const LocalizationResultSnapshot & initial_result,
    const BridgeStatusSnapshot & initial_bridge,
    const double trigger_started_sec,
    const double timeout_sec)
  {
    const auto deadline = steady_deadline(timeout_sec);
    while (std::chrono::steady_clock::now() <= deadline) {
      if (
        localization_result_observed_after(initial_result, trigger_started_sec) ||
        bridge_processed_after(initial_bridge, trigger_started_sec))
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  bool bridge_explicit_relocalization_target_settled(
    const BridgeStatusSnapshot & latest) const
  {
    if (
      latest.last_explicit_map_odom_target_sequence == 0U ||
      latest.current_sequence != latest.target_sequence ||
      latest.current_sequence < latest.last_explicit_map_odom_target_sequence)
    {
      return false;
    }
    const bool isaac_target_completed =
      latest.current_sequence == latest.last_explicit_map_odom_target_sequence &&
      latest.current_source == "isaac_triggered" &&
      latest.target_source == "isaac_triggered";
    const bool valid_amcl_refine_completed =
      latest.amcl_post_isaac_refined_sequence ==
      latest.last_explicit_relocalization_sequence &&
      latest.current_source == "amcl_gated" &&
      latest.target_source == "amcl_gated";
    return isaac_target_completed || valid_amcl_refine_completed;
  }

  bool wait_for_bridge_acceptance(
    const BridgeStatusSnapshot & initial,
    const double trigger_started_sec,
    const double timeout_sec,
    const bool explicit_accept_required,
    std::string & detail)
  {
    auto active_deadline = steady_deadline(timeout_sec);
    bool saw_nonfresh_bridge_accept = false;
    bool saw_prearm_result = false;
    bool saw_transient_triggered_stale_reject = false;
    std::uint64_t last_amcl_observe_only_reject_count = initial.rejected_result_count;
    std::uint64_t last_transient_triggered_stale_reject_count = initial.rejected_result_count;
    std::uint64_t last_pretrigger_ignored_count =
      initial.force_accept_ignored_pretrigger_result_count;
    BridgeStatusSnapshot latest;
    while (
      std::chrono::steady_clock::now() <= active_deadline)
    {
      latest = bridge_status_snapshot();
      if (latest.available && latest.received_sec >= trigger_started_sec) {
        if (
          bridge_explicit_trigger_accept_observed(initial, latest) &&
          latest.has_map_to_odom)
        {
          if (
            map_to_odom_ready(latest) &&
            latest.safe_for_goal_start &&
            !latest.correction_active &&
            latest.current_sequence == latest.target_sequence &&
            bridge_explicit_relocalization_target_settled(latest))
          {
            detail = "bridge accepted explicit triggered relocalization"
                     " gate_mode=" + latest.gate_mode +
                     " accept_reason=" + latest.last_accept_reason +
                     " explicit_sequence=" +
                     std::to_string(latest.last_explicit_relocalization_sequence) +
                     " explicit_source=" + latest.last_explicit_relocalization_source +
                     " has_map_to_odom=true age_ms=" +
                     std::to_string(latest.map_to_odom_age_ms) +
                     " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms) +
                     " safe_for_goal_start=" + bool_string(latest.safe_for_goal_start) +
                     " correction_active=" + bool_string(latest.correction_active) +
                     " remaining_translation_error_m=" +
                     std::to_string(latest.remaining_translation_error_m) +
                     " remaining_yaw_error_rad=" +
                     std::to_string(latest.remaining_yaw_error_rad) +
                     " current_sequence=" + std::to_string(latest.current_sequence) +
                     " target_sequence=" + std::to_string(latest.target_sequence) +
                     " current_source=" + latest.current_source +
                     " target_source=" + latest.target_source;
            return true;
          }
          saw_nonfresh_bridge_accept = true;
          detail = "bridge accepted explicit triggered relocalization but map->odom target is not settled yet"
                   " owner=" + latest.owner +
                   " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                   " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms) +
                   " explicit_sequence=" +
                   std::to_string(latest.last_explicit_relocalization_sequence) +
                   " safe_for_goal_start=" + bool_string(latest.safe_for_goal_start) +
                   " correction_active=" + bool_string(latest.correction_active) +
                   " current_sequence=" + std::to_string(latest.current_sequence) +
                   " target_sequence=" + std::to_string(latest.target_sequence) +
                   " current_source=" + latest.current_source +
                   " target_source=" + latest.target_source +
                   " last_reject_reason=" + latest.last_reject_reason;
        }
        if (
          latest.force_accept_ignored_pretrigger_result_count >
          last_pretrigger_ignored_count)
        {
          last_pretrigger_ignored_count =
            latest.force_accept_ignored_pretrigger_result_count;
          saw_prearm_result = true;
          detail =
            "draining pre-arm localization_result inside the same trigger transaction; "
            "force-accept arm time and deadline remain unchanged; ignored_reason=" +
            latest.last_force_accept_ignored_reason;
          RCLCPP_WARN(get_logger(), "%s", detail.c_str());
        }
        if (latest.rejected_result_count > initial.rejected_result_count) {
          if (bridge_reject_is_transient_triggered_stale(latest.last_reject_reason)) {
            saw_transient_triggered_stale_reject = true;
            if (latest.rejected_result_count > last_transient_triggered_stale_reject_count) {
              last_transient_triggered_stale_reject_count = latest.rejected_result_count;
              detail =
                "draining stale triggered localization_result inside the same trigger transaction; "
                "force-accept arm time and deadline remain unchanged; last_reject_reason=" +
                latest.last_reject_reason;
              RCLCPP_WARN(get_logger(), "%s", detail.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
          }
          if (bridge_reject_is_expected_amcl_observe_only(latest.last_reject_reason)) {
            if (latest.rejected_result_count > last_amcl_observe_only_reject_count) {
              last_amcl_observe_only_reject_count = latest.rejected_result_count;
              detail = "bridge ignoring AMCL observe-only reject while waiting for fresh "
                       "triggered localization_result: " + latest.last_reject_reason;
              RCLCPP_INFO(get_logger(), "%s", detail.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
          }
          detail = "failure_code=BRIDGE_REJECTED_RESULT dispatch_state=dispatched last_reject_reason=" +
                   latest.last_reject_reason;
          return false;
        }
        if (
          !explicit_accept_required &&
          latest.accepted_result_count > initial.accepted_result_count &&
          latest.has_map_to_odom)
        {
          if (
            map_to_odom_ready(latest) &&
            latest.safe_for_goal_start &&
            !latest.correction_active &&
            latest.current_sequence == latest.target_sequence)
          {
            detail = "bridge accepted result gate_mode=" + latest.gate_mode +
                     " accept_reason=" + latest.last_accept_reason +
                     " has_map_to_odom=true age_ms=" +
                     std::to_string(latest.map_to_odom_age_ms) +
                     " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
            return true;
          }
          saw_nonfresh_bridge_accept = true;
          detail = "bridge accepted count advanced but map->odom is not fresh yet owner=" +
                   latest.owner + " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                   " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail = saw_nonfresh_bridge_accept ?
      "failure_code=BRIDGE_ACCEPT_TIMEOUT dispatch_state=dispatched "
      "bridge accepted a result but map->odom target did not settle" :
      "failure_code=BRIDGE_ACCEPT_TIMEOUT dispatch_state=dispatched "
      "bridge did not accept a current-arm localization_result";
    if (saw_prearm_result) {
      detail += " saw_prearm_result=true";
    }
    if (saw_transient_triggered_stale_reject) {
      detail += " saw_transient_triggered_stale_reject=true";
    }
    if (latest.available) {
      detail += " last_reject_reason=" + latest.last_reject_reason;
      if (latest.last_reject_reason.find("tf_history_missing") != std::string::npos) {
        detail =
          "failure_code=TF_HISTORY_MISSING dispatch_state=dispatched " +
          latest.last_reject_reason;
      }
    }
    return false;
  }

  bool bridge_reject_is_transient_triggered_stale(const std::string & reason) const
  {
    return reason.find("isaac_triggered_pose_stale_ms") != std::string::npos &&
           reason.find("gate_mode=triggered") != std::string::npos;
  }

  bool bridge_reject_is_expected_amcl_observe_only(const std::string & reason) const
  {
    return (
      reason.find("AMCL_ROBOT_MOVING_OBSERVE_ONLY") != std::string::npos ||
      reason.find("amcl_suppressed_after_isaac_triggered") != std::string::npos ||
      reason.find("AMCL_POST_ISAAC_REFINE_") != std::string::npos ||
      reason.find("AMCL_CORRECTION_TOO_LARGE") != std::string::npos) &&
      reason.find("amcl") != std::string::npos;
  }

  bool bridge_explicit_trigger_accept_observed(
    const BridgeStatusSnapshot & initial,
    const BridgeStatusSnapshot & latest) const
  {
    return latest.last_explicit_relocalization_sequence >
           initial.last_explicit_relocalization_sequence ||
           (
      latest.accepted_result_count > initial.accepted_result_count &&
      latest.last_accept_reason == "EXPLICIT_TRIGGERED_RELOCALIZATION");
  }

  static std::string bool_string(const bool value)
  {
    return value ? "true" : "false";
  }

  bool map_to_odom_tf_available()
  {
    try {
      const rclcpp::Time latest_tf_time(0, 0, get_clock()->get_clock_type());
      const auto timeout = rclcpp::Duration::from_seconds(0.02);
      (void)tf_buffer_.lookupTransform(map_frame_, odom_frame_, latest_tf_time, timeout);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  bool map_to_odom_bridge_publish_healthy(const BridgeStatusSnapshot & latest) const
  {
    if (latest.map_to_odom_age_ms >= 0.0 && latest.map_to_odom_age_ms <= map_to_odom_max_age_ms_) {
      return true;
    }
    return latest.map_odom_publish_gap_ms >= 0.0 &&
           latest.map_odom_publish_gap_ms <= map_to_odom_max_age_ms_;
  }

  bool map_to_odom_ready(const BridgeStatusSnapshot & latest)
  {
    return latest.available &&
           latest.has_map_to_odom &&
           latest.owner == "robot_localization_bridge" &&
           map_to_odom_bridge_publish_healthy(latest) &&
           map_to_odom_tf_available();
  }

  bool wait_for_map_to_odom(const double timeout_sec, std::string & detail)
  {
    const auto deadline = steady_deadline(timeout_sec);
    BridgeStatusSnapshot latest;
    while (std::chrono::steady_clock::now() <= deadline) {
      latest = bridge_status_snapshot();
      if (
        latest.available &&
        latest.has_map_to_odom &&
        latest.owner != "robot_localization_bridge")
      {
        detail =
          "failure_code=MAP_TO_ODOM_WRONG_OWNER dispatch_state=dispatched owner=" +
          latest.owner;
        return false;
      }
      if (map_to_odom_ready(latest))
      {
        detail = "map->odom ready owner=robot_localization_bridge age_ms=" +
                 std::to_string(latest.map_to_odom_age_ms) +
                 " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    detail =
      "failure_code=MAP_TO_ODOM_TIMEOUT dispatch_state=dispatched "
      "map->odom unavailable or stale";
    if (latest.available) {
      detail += " has_map_to_odom=" + std::string(latest.has_map_to_odom ? "true" : "false") +
                " owner=" + latest.owner +
                " age_ms=" + std::to_string(latest.map_to_odom_age_ms) +
                " publish_gap_ms=" + std::to_string(latest.map_odom_publish_gap_ms);
    }
    return false;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizerAssetState>::SharedPtr
    asset_state_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr grid_search_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bridge_force_accept_client_;
  rclcpp::Subscription<isaac_ros_pointcloud_interfaces::msg::FlatScan>::SharedPtr
    localizer_input_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_status_sub_;
  rclcpp::Service<robot_interfaces::srv::TriggerLocalization>::SharedPtr trigger_srv_;
  rclcpp::Service<robot_interfaces::srv::ApplyFloorAssets>::SharedPtr apply_floor_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr runtime_context_bootstrap_timer_;
  std::mutex state_mutex_;
  std::mutex localizer_operation_mutex_;
  std::mutex asset_state_mutex_;
  std::mutex active_asset_mutex_;
  robot_interfaces::msg::LocalizerAssetState last_asset_state_;
  std::unique_ptr<robot_global_localization::IsaacAssetReloader> asset_reloader_;
  std::unique_ptr<robot_global_localization::PostReloadReadinessGate>
    post_reload_readiness_gate_;
  std::unique_ptr<robot_global_localization::RosComponentManagerPort> component_manager_;

  std::string active_floor_id_;
  std::string active_nav_map_yaml_;
  std::string active_localizer_map_png_;
  std::string active_localizer_params_yaml_;
  std::string grid_search_trigger_service_;
  std::string localization_result_topic_;
  std::string bridge_status_topic_;
  std::string bridge_force_accept_service_;
  std::string localizer_input_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string runtime_map_context_file_;
  double service_timeout_sec_{10.0};
  double service_call_timeout_sec_{10.0};
  double result_wait_timeout_sec_{20.0};
  double bridge_accept_timeout_sec_{12.0};
  double map_to_odom_wait_timeout_sec_{8.0};
  double map_to_odom_max_age_ms_{1000.0};
  double localizer_input_wait_timeout_sec_{1.0};
  double localizer_input_max_age_sec_{0.5};
  double localizer_input_min_fov_deg_{115.0};
  int localizer_input_required_consecutive_good_{2};
  double result_allowed_pretrigger_age_sec_{1.0};
  double runtime_context_bootstrap_retry_sec_{5.0};
  bool localizer_input_freshness_enabled_{true};
  bool require_grid_search_trigger_{true};
  bool require_bridge_acceptance_{true};
  bool mock_mode_{false};
  std::chrono::steady_clock::time_point
    next_runtime_context_bootstrap_attempt_{};
  std::string last_trigger_status_{"idle"};
  LocalizerInputSnapshot localizer_input_;
  LocalizationResultSnapshot localization_result_;
  BridgeStatusSnapshot bridge_status_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
