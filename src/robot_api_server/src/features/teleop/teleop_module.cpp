#include "robot_api_server/features/teleop/teleop_module.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "robot_api_server/features/teleop/teleop_session.hpp"
#include "robot_api_server/infrastructure/http/http_server.hpp"
#include "robot_api_server/infrastructure/http/websocket_transport.hpp"

namespace robot_api_server::features::teleop
{

using namespace std::chrono_literals;
using infrastructure::http::HttpServer;
using infrastructure::http::read_websocket_frame;
using infrastructure::http::send_websocket_frame;
using infrastructure::http::send_websocket_handshake;
using infrastructure::http::set_socket_receive_timeout;
using infrastructure::http::websocket_headers_valid;

namespace
{

void require_ports(const TeleopModulePorts & ports)
{
  if (!ports.token_allowed ||
    !ports.elevator_interlock ||
    !ports.capture_motion_admission_epoch ||
    !ports.acquire_motion_admission ||
    !ports.charging_contact_active ||
    !ports.mapping_snapshot ||
    !ports.pose_snapshot ||
    !ports.acquire_subscriptions ||
    !ports.release_subscriptions ||
    !ports.runtime_running)
  {
    throw std::invalid_argument("teleop module requires every cross-domain port");
  }
}

std::string motion_admission_failure_detail(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  if (admission.stale()) {
    return operation + " request predates the completed elevator admission barrier";
  }
  if (admission.interlock().delayed_side_effect_unknown()) {
    return operation + " is blocked because " +
           std::to_string(admission.interlock().delayed_side_effect_unknown_count) +
           " timed-out motion/runtime submission(s) have no proven outcome";
  }
  if (admission.interlock().recovery_required()) {
    return operation +
           " is blocked by retained elevator recovery state; transaction_id=" +
           admission.interlock().transaction_id;
  }
  return operation + " is blocked during elevator execution; transaction_id=" +
         admission.interlock().transaction_id;
}

HttpResponse motion_admission_failure_response(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  const std::string code = admission.stale() ?
    "ELEVATOR_MOTION_ADMISSION_STALE" :
    (admission.interlock().delayed_side_effect_unknown() ?
    "DELAYED_SIDE_EFFECT_UNKNOWN" :
    (admission.interlock().recovery_required() ?
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRED" :
    "ELEVATOR_EXECUTION_ACTIVE"));
  std::ostringstream body;
  body << "{\"ok\":false,\"code\":" << json_string(code)
       << ",\"transaction_id\":"
       << json_string(admission.interlock().transaction_id)
       << ",\"delayed_side_effect_unknown_count\":"
       << admission.interlock().delayed_side_effect_unknown_count
       << ",\"detail\":"
       << json_string(motion_admission_failure_detail(operation, admission))
       << "}";
  return {409, "application/json", body.str()};
}

}  // namespace

class TeleopModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    TeleopModuleConfig config,
    TeleopModulePorts ports)
  : node_(node), config_(std::move(config)), ports_(std::move(ports))
  {
    require_ports(ports_);
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    teleop_cmd_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
      config_.cmd_topic, command_qos);
    teleop_reverse_enable_pub_ = node_.create_publisher<std_msgs::msg::Bool>(
      config_.reverse_enable_topic, rclcpp::QoS(1));
    teleop_repeat_timer_ = node_.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / config_.repeat_rate_hz)),
      [this]() {on_repeat_timer();});
  }

  ~Impl()
  {
    shutdown();
  }

  bool handle_socket(const int client_fd, const HttpRequest & request)
  {
    if (request.method != "GET" || request.path != "/ws/v1/teleop") {
      return false;
    }
    handle_teleop_websocket(client_fd, request);
    return true;
  }

  void clear_command()
  {
    teleop_session_.clear_velocity();
    publish_zero();
    publish_reverse_enable(false);
  }

  void publish_zero()
  {
    teleop_cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publish_zero_burst()
  {
    for (int i = 0; i < 8; ++i) {
      publish_zero();
      std::this_thread::sleep_for(50ms);
    }
  }

  void on_charging_contact(const bool active)
  {
    if (active && config_.stop_on_charging && teleop_session_.session_active()) {
      clear_command();
    }
  }

  bool active() const
  {
    return teleop_session_.session_active();
  }

  bool idle() const
  {
    return teleop_session_.idle();
  }

  void shutdown()
  {
    if (shutdown_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    running_proxy_.store(false, std::memory_order_release);
    if (teleop_repeat_timer_) {
      teleop_repeat_timer_->cancel();
    }
  }

private:
  bool charging_guard_active() const
  {
    return config_.stop_on_charging && ports_.charging_contact_active();
  }

  bool teleop_session_allowed_for_mapping_state(
    const bool mapping_active,
    std::string & reason) const
  {
    const auto elevator_interlock = ports_.elevator_interlock();
    if (elevator_interlock.blocked()) {
      reason = elevator_interlock.recovery_required() ?
        "WebSocket teleop is blocked by elevator recovery state" :
        "WebSocket teleop is blocked during elevator execution";
      return false;
    }
    if (!config_.require_mapping_active || mapping_active) {
      return true;
    }
    reason = "WebSocket teleop is only allowed while 2D mapping is active";
    return false;
  }

  bool teleop_session_allowed(std::string & reason) const
  {
    const auto mapping = ports_.mapping_snapshot(true);
    return teleop_session_allowed_for_mapping_state(mapping.active, reason);
  }

  void publish_reverse_enable(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled && config_.allow_reverse;
    teleop_reverse_enable_pub_->publish(msg);
  }

  void mark_teleop_session_started()
  {
    teleop_session_.session_started();
  }

  void mark_teleop_session_stopped()
  {
    if (!teleop_session_.session_stopped()) {
      return;
    }
    publish_zero();
    publish_reverse_enable(false);
  }

  void store_teleop_command(const geometry_msgs::msg::Twist & twist)
  {
    teleop_session_.store_velocity(
      TeleopVelocity{twist.linear.x, twist.angular.z},
      TeleopSessionState::Clock::now());
  }

  void on_repeat_timer()
  {
    if (!teleop_session_.session_active()) {
      return;
    }
    if (ports_.elevator_interlock().blocked()) {
      clear_command();
      return;
    }
    if (charging_guard_active()) {
      clear_command();
      return;
    }

    const auto decision = teleop_session_.repeat_decision(
      TeleopSessionState::Clock::now(),
      std::chrono::duration<double>(config_.watchdog_timeout_sec));
    if (decision.action == TeleopRepeatAction::PublishVelocity) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = decision.velocity.linear_x;
      twist.angular.z = decision.velocity.angular_z;
      publish_reverse_enable(config_.allow_reverse);
      teleop_cmd_pub_->publish(twist);
    } else if (decision.action == TeleopRepeatAction::PublishZero) {
      publish_zero();
      publish_reverse_enable(false);
    }
  }

  std::string teleop_state_json() const
  {
    const auto mapping = ports_.mapping_snapshot(true);
    std::string teleop_block_reason;
    const bool teleop_allowed = teleop_session_allowed_for_mapping_state(
      mapping.active, teleop_block_reason);
    const auto pose = ports_.pose_snapshot();

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"ok\":true,\"type\":\"mapping_state\","
        << "\"state\":" << json_string(mapping.active ? "running" : "stopped") << ","
        << "\"teleop_allowed\":" << (teleop_allowed ? "true" : "false") << ","
        << "\"allow_reverse\":" << (config_.allow_reverse ? "true" : "false") << ","
        << "\"map_available\":" << (mapping.live_map_available ? "true" : "false");
    if (!teleop_allowed && !teleop_block_reason.empty()) {
      out << ",\"teleop_block_reason\":" << json_string(teleop_block_reason);
    }
    if (mapping.live_map_available) {
      out << ",\"map_age_sec\":" << mapping.live_map_age_sec
          << ",\"area_m2\":" << mapping.known_area_m2
          << ",\"map\":{\"width\":" << mapping.live_map_width
          << ",\"height\":" << mapping.live_map_height
          << ",\"resolution\":" << mapping.live_map_resolution << "}";
    }
    if (pose.available) {
      out << ",\"pose\":{\"frame_id\":" << json_string(pose.frame_id)
          << ",\"x\":" << pose.x
          << ",\"y\":" << pose.y
          << ",\"yaw\":" << pose.yaw
          << ",\"age_sec\":" << pose.age_sec << "}";
    }
    out << "}";
    return out.str();
  }

  bool publish_teleop_command(const std::string & payload, std::string & ack_json)
  {
    const auto evaluation = evaluate_teleop_payload(
      payload,
      TeleopCommandConfig{
        config_.max_linear_x_mps,
        config_.max_angular_z_radps,
        config_.allow_reverse,
        config_.cmd_topic,
      });
    if (evaluation.kind == TeleopMessageKind::Stop) {
      clear_command();
      ack_json = evaluation.response_json;
      return true;
    }
    if (evaluation.kind == TeleopMessageKind::Unsupported) {
      ack_json = evaluation.response_json;
      return false;
    }

    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      clear_command();
      ack_json = error_json(reject_reason);
      return false;
    }
    if (charging_guard_active()) {
      clear_command();
      ack_json = error_json("charging detected; teleop command stopped");
      return false;
    }
    if (!evaluation.valid) {
      ack_json = evaluation.response_json;
      return false;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = evaluation.velocity.linear_x;
    twist.angular.z = evaluation.velocity.angular_z;
    store_teleop_command(twist);
    publish_reverse_enable(config_.allow_reverse);
    teleop_cmd_pub_->publish(twist);
    ack_json = evaluation.response_json;
    return true;
  }

  void send_response(const int client_fd, const HttpResponse & response) const
  {
    if (!HttpServer::write_response(client_fd, response)) {
      RCLCPP_WARN(
        node_.get_logger(),
        "failed to send full HTTP response status=%d bytes=%zu",
        response.status,
        response.body.size());
    }
  }

  void handle_teleop_websocket(const int client_fd, const HttpRequest & request)
  {
    const auto motion_admission_epoch = ports_.capture_motion_admission_epoch();
    if (!ports_.token_allowed(request)) {
      send_response(
        client_fd,
        {401, "application/json", error_json("missing or invalid X-Robot-Token")});
      return;
    }
    if (!websocket_headers_valid(request)) {
      send_response(
        client_fd,
        {400, "application/json", error_json("invalid websocket upgrade request")});
      return;
    }
    std::string reject_reason;
    if (!teleop_session_allowed(reject_reason)) {
      send_response(client_fd, {409, "application/json", error_json(reject_reason)});
      return;
    }

    auto motion_admission = ports_.acquire_motion_admission(motion_admission_epoch);
    if (!motion_admission.admitted()) {
      send_response(
        client_fd,
        motion_admission_failure_response("websocket_teleop", motion_admission));
      return;
    }
    mark_teleop_session_started();
    motion_admission.unlock();
    send_websocket_handshake(client_fd, request);
    const std::string websocket_client_id = "websocket:" + std::to_string(client_fd);
    const int websocket_ttl_ms = std::clamp(
      static_cast<int>((config_.socket_idle_timeout_sec + 1.0) * 1000.0),
      1000,
      config_.subscription_max_ttl_ms);
    ports_.acquire_subscriptions(
      websocket_client_id,
      {"teleop", "tf"},
      std::chrono::milliseconds(websocket_ttl_ms));
    set_socket_receive_timeout(client_fd, config_.socket_idle_timeout_sec);
    publish_reverse_enable(config_.allow_reverse);
    {
      std::ostringstream ready;
      ready << "{\"ok\":true,\"type\":\"teleop_ready\","
            << "\"cmd_topic\":" << json_string(config_.cmd_topic) << ","
            << "\"reverse_enable_topic\":" << json_string(config_.reverse_enable_topic) << ","
            << "\"max_linear_x_mps\":" << config_.max_linear_x_mps << ","
            << "\"max_angular_z_radps\":" << config_.max_angular_z_radps << ","
            << "\"watchdog_timeout_sec\":" << config_.watchdog_timeout_sec << ","
            << "\"socket_idle_timeout_sec\":" << config_.socket_idle_timeout_sec << ","
            << "\"repeat_rate_hz\":" << config_.repeat_rate_hz << ","
            << "\"require_mapping_active\":"
            << (config_.require_mapping_active ? "true" : "false") << ","
            << "\"allow_reverse\":" << (config_.allow_reverse ? "true" : "false") << "}";
      send_websocket_frame(client_fd, ready.str());
      send_websocket_frame(client_fd, teleop_state_json());
    }

    while (ports_.runtime_running()) {
      const auto frame = read_websocket_frame(client_fd, running_proxy_);
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
      ports_.acquire_subscriptions(
        websocket_client_id,
        {"teleop", "tf"},
        std::chrono::milliseconds(websocket_ttl_ms));
      publish_teleop_command(frame->payload, ack);
      if (!send_websocket_frame(client_fd, ack)) {
        break;
      }
      if (!send_websocket_frame(client_fd, teleop_state_json())) {
        break;
      }
    }

    mark_teleop_session_stopped();
    ports_.release_subscriptions(websocket_client_id, {"teleop", "tf"});
  }

  rclcpp::Node & node_;
  TeleopModuleConfig config_;
  TeleopModulePorts ports_;
  TeleopSessionState teleop_session_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_reverse_enable_pub_;
  rclcpp::TimerBase::SharedPtr teleop_repeat_timer_;
  std::atomic_bool shutdown_{false};
  std::atomic_bool running_proxy_{true};
};

TeleopModule::TeleopModule(
  rclcpp::Node & node,
  TeleopModuleConfig config,
  TeleopModulePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

TeleopModule::~TeleopModule() = default;

bool TeleopModule::handle_socket(const int client_fd, const HttpRequest & request)
{
  return impl_->handle_socket(client_fd, request);
}

void TeleopModule::clear_command()
{
  impl_->clear_command();
}

void TeleopModule::publish_zero()
{
  impl_->publish_zero();
}

void TeleopModule::publish_zero_burst()
{
  impl_->publish_zero_burst();
}

void TeleopModule::on_charging_contact(const bool active)
{
  impl_->on_charging_contact(active);
}

bool TeleopModule::active() const
{
  return impl_->active();
}

bool TeleopModule::idle() const
{
  return impl_->idle();
}

void TeleopModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::teleop
