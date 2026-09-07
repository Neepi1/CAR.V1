#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace robot_api_server
{
namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u00";
          constexpr char digits[] = "0123456789abcdef";
          output << digits[(character >> 4U) & 0x0fU]
                 << digits[character & 0x0fU];
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

std::string trim(std::string value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(),
    [](const unsigned char c) {return std::isspace(c) != 0;});
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(),
    [](const unsigned char c) {return std::isspace(c) != 0;}).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::optional<int> parse_decimal(const std::string & value)
{
  if (value.empty()) {
    return std::nullopt;
  }
  int parsed = 0;
  for (const unsigned char character : value) {
    if (!std::isdigit(character)) {
      return std::nullopt;
    }
    const int digit = static_cast<int>(character - '0');
    if (parsed > (std::numeric_limits<int>::max() - digit) / 10) {
      return std::nullopt;
    }
    parsed = parsed * 10 + digit;
  }
  return parsed;
}

std::optional<int> parse_floor_level(const std::string & value)
{
  const bool negative = value.size() > 1U && value.front() == '-';
  const auto magnitude = parse_decimal(value.substr(negative ? 1U : 0U));
  if (!magnitude) {
    return std::nullopt;
  }
  return negative ? -*magnitude : *magnitude;
}

std::optional<YAML::Node> json_document(
  const ElevatorArmHttpResponse & response,
  std::string & error)
{
  if (!response.transport_error.empty()) {
    error = response.transport_error;
    return std::nullopt;
  }
  try {
    auto node = YAML::Load(response.body);
    if (!node || !node.IsMap()) {
      error = "response JSON is not an object";
      return std::nullopt;
    }
    return node;
  } catch (const std::exception & exception) {
    error = std::string("invalid response JSON: ") + exception.what();
    return std::nullopt;
  }
}

bool bool_value(
  const YAML::Node & node,
  const char * key,
  const bool fallback = false)
{
  try {
    const auto value = node[key];
    return value && value.IsScalar() ? value.as<bool>() : fallback;
  } catch (...) {
    return fallback;
  }
}

std::string string_value(
  const YAML::Node & node,
  const char * key,
  const std::string & fallback = {})
{
  try {
    const auto value = node[key];
    return value && value.IsScalar() ? value.as<std::string>() : fallback;
  } catch (...) {
    return fallback;
  }
}

ElevatorArmOutcome failed(
  std::string code,
  std::string detail,
  std::string task_id = {})
{
  return {
    ElevatorArmOutcomeKind::kFailed,
    std::move(code),
    std::move(detail),
    std::move(task_id),
  };
}

class LoopbackHttpTransport final : public ElevatorArmHttpTransport
{
public:
  explicit LoopbackHttpTransport(ElevatorArmClientOptions options)
  : options_(std::move(options))
  {
    if (options_.host != "127.0.0.1") {
      throw std::invalid_argument(
              "elevator arm service must remain bound to 127.0.0.1");
    }
    if (options_.port == 0U || options_.maximum_response_bytes == 0U) {
      throw std::invalid_argument("invalid elevator arm HTTP transport options");
    }
  }

  ElevatorArmHttpResponse request(
    const std::string & method,
    const std::string & path,
    const std::string & body,
    const std::chrono::milliseconds timeout) override
  {
#ifdef _WIN32
    (void)method;
    (void)path;
    (void)body;
    (void)timeout;
    return {599, {}, "loopback arm HTTP transport is only available on Linux"};
#else
    if ((method != "GET" && method != "POST") || path.empty() || path[0] != '/') {
      return {599, {}, "invalid loopback HTTP request"};
    }
    const int descriptor = ::socket(AF_INET, SOCK_STREAM, 0);
    if (descriptor < 0) {
      return {599, {}, std::string("socket failed: ") + std::strerror(errno)};
    }
    struct DescriptorGuard
    {
      int value;
      ~DescriptorGuard() {if (value >= 0) {(void)::close(value);}}
    } guard{descriptor};

    const auto bounded_timeout = std::max(timeout, std::chrono::milliseconds(1));
    timeval socket_timeout{};
    socket_timeout.tv_sec = static_cast<time_t>(bounded_timeout.count() / 1000);
    socket_timeout.tv_usec = static_cast<suseconds_t>(
      (bounded_timeout.count() % 1000) * 1000);
    (void)::setsockopt(
      descriptor, SOL_SOCKET, SO_RCVTIMEO,
      &socket_timeout, sizeof(socket_timeout));
    (void)::setsockopt(
      descriptor, SOL_SOCKET, SO_SNDTIMEO,
      &socket_timeout, sizeof(socket_timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.port);
    if (::inet_pton(AF_INET, options_.host.c_str(), &address.sin_addr) != 1) {
      return {599, {}, "invalid loopback IPv4 address"};
    }
    if (::connect(
        descriptor, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
    {
      return {599, {}, std::string("connect failed: ") + std::strerror(errno)};
    }

    std::ostringstream request_stream;
    request_stream << method << " " << path << " HTTP/1.0\r\n"
                   << "Host: " << options_.host << ":" << options_.port << "\r\n"
                   << "Accept: application/json\r\n"
                   << "Connection: close\r\n";
    if (method == "POST") {
      request_stream << "Content-Type: application/json\r\n"
                     << "Content-Length: " << body.size() << "\r\n";
    }
    request_stream << "\r\n";
    if (method == "POST") {
      request_stream << body;
    }
    const auto wire = request_stream.str();
    std::size_t sent = 0U;
    while (sent < wire.size()) {
      const auto count = ::send(
        descriptor, wire.data() + sent, wire.size() - sent, MSG_NOSIGNAL);
      if (count <= 0) {
        return {599, {}, std::string("send failed: ") + std::strerror(errno)};
      }
      sent += static_cast<std::size_t>(count);
    }

    std::string response;
    char buffer[4096];
    while (true) {
      const auto count = ::recv(descriptor, buffer, sizeof(buffer), 0);
      if (count == 0) {
        break;
      }
      if (count < 0) {
        return {599, {}, std::string("receive failed: ") + std::strerror(errno)};
      }
      response.append(buffer, static_cast<std::size_t>(count));
      if (response.size() > options_.maximum_response_bytes) {
        return {599, {}, "arm HTTP response exceeds configured size limit"};
      }
    }
    const auto header_end = response.find("\r\n\r\n");
    const auto first_line_end = response.find("\r\n");
    if (header_end == std::string::npos || first_line_end == std::string::npos) {
      return {599, {}, "malformed arm HTTP response"};
    }
    std::istringstream status_line(response.substr(0U, first_line_end));
    std::string protocol;
    int status = 0;
    status_line >> protocol >> status;
    if (protocol.rfind("HTTP/", 0U) != 0U || status < 100 || status > 599) {
      return {599, {}, "malformed arm HTTP status line"};
    }
    return {status, response.substr(header_end + 4U), {}};
#endif
  }

private:
  ElevatorArmClientOptions options_;
};

}  // namespace

ElevatorArmClient::ElevatorArmClient(ElevatorArmClientOptions options)
: ElevatorArmClient(
    options, std::make_shared<LoopbackHttpTransport>(options))
{
}

ElevatorArmClient::ElevatorArmClient(
  ElevatorArmClientOptions options,
  std::shared_ptr<ElevatorArmHttpTransport> transport)
: options_(std::move(options)), transport_(std::move(transport))
{
  if (!transport_) {
    throw std::invalid_argument("elevator arm HTTP transport is required");
  }
  if (
    options_.request_timeout.count() <= 0 ||
    options_.task_timeout.count() <= 0 ||
    options_.poll_interval.count() < 0)
  {
    throw std::invalid_argument("invalid elevator arm client timeout options");
  }
}

std::optional<std::string> ElevatorArmClient::floor_button_label(
  const std::string & floor_id)
{
  auto value = trim(floor_id);
  if (value.empty()) {
    return std::nullopt;
  }
  if (value == "-1" || value == "-2") {
    return value;
  }
  if (value.size() >= 2U && (value[0] == 'F' || value[0] == 'f')) {
    value.erase(value.begin());
  }
  const auto floor = parse_decimal(value);
  if (!floor || *floor < 1 || *floor > 20) {
    return std::nullopt;
  }
  return std::to_string(*floor);
}

std::optional<std::string> ElevatorArmClient::hall_call_direction(
  const std::string & source_floor_id,
  const std::string & target_floor_id)
{
  const auto source_label = floor_button_label(source_floor_id);
  const auto target_label = floor_button_label(target_floor_id);
  if (!source_label || !target_label) {
    return std::nullopt;
  }
  const auto source_floor = parse_floor_level(*source_label);
  const auto target_floor = parse_floor_level(*target_label);
  if (!source_floor || !target_floor || *source_floor == *target_floor) {
    return std::nullopt;
  }
  return *target_floor > *source_floor ? "up" : "down";
}

ElevatorArmOutcome ElevatorArmClient::run_task(
  const std::string & path,
  const std::string & request_body,
  const std::function<bool()> & cancellation_requested)
{
  const auto submitted = transport_->request(
    "POST", path, request_body, options_.request_timeout);
  return await_task(submitted, path, cancellation_requested);
}

ElevatorArmOutcome ElevatorArmClient::await_task(
  const ElevatorArmHttpResponse & submitted,
  const std::string & path,
  const std::function<bool()> & cancellation_requested)
{
  std::string parse_error;
  const auto accepted = json_document(submitted, parse_error);
  if (submitted.status != 202 || !accepted) {
    std::string code = accepted ? string_value(*accepted, "error_code") : std::string{};
    if (code.empty()) {
      code = "ELEVATOR_ARM_COMMAND_REJECTED";
    }
    return failed(
      code,
      "arm command was not accepted; path=" + path + ";http_status=" +
      std::to_string(submitted.status) + ";detail=" + parse_error);
  }
  const auto task_id = string_value(*accepted, "task_id");
  if (task_id.empty()) {
    return failed(
      "ELEVATOR_ARM_TASK_ID_INVALID",
      "accepted arm command did not return a task_id");
  }

  const auto deadline = std::chrono::steady_clock::now() + options_.task_timeout;
  while (std::chrono::steady_clock::now() <= deadline) {
    if (cancellation_requested && cancellation_requested()) {
      return failed(
        "ELEVATOR_ARM_TASK_CANCELLED",
        "elevator transaction was cancelled while waiting for arm task",
        task_id);
    }
    const auto polled = transport_->request(
      "GET", "/api/v1/tasks/" + task_id, {}, options_.request_timeout);
    parse_error.clear();
    const auto task = json_document(polled, parse_error);
    if (polled.status != 200 || !task) {
      return failed(
        "ELEVATOR_ARM_TASK_STATUS_UNAVAILABLE",
        "arm task status failed; task_id=" + task_id + ";http_status=" +
        std::to_string(polled.status) + ";detail=" + parse_error,
        task_id);
    }
    const auto state = string_value(*task, "state");
    if (state == "succeeded") {
      return {
        ElevatorArmOutcomeKind::kSucceeded,
        "ELEVATOR_ARM_TASK_SUCCEEDED",
        "arm task reported state=succeeded; task_id=" + task_id,
        task_id,
      };
    }
    if (state == "failed" || bool_value(*task, "terminal")) {
      auto code = string_value(*task, "error_code", "ELEVATOR_ARM_TASK_FAILED");
      return failed(
        code,
        "arm task failed; task_id=" + task_id + ";message=" +
        string_value(*task, "message"),
        task_id);
    }
    if (options_.poll_interval.count() > 0) {
      std::this_thread::sleep_for(options_.poll_interval);
    }
  }
  return failed(
    "ELEVATOR_ARM_TASK_TIMEOUT",
    "arm task did not reach a terminal state before timeout; task_id=" + task_id,
    task_id);
}

ElevatorArmOutcome ElevatorArmClient::run_button_sequence(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & action_path,
  const std::string & action_field,
  const std::string & action_value,
  const std::function<bool()> & cancellation_requested,
  const bool hall_call)
{
  if (transaction_id.empty() || effect_sequence == 0U) {
    return failed(
      "ELEVATOR_ARM_INVALID_TRANSACTION",
      "transaction_id and positive effect_sequence are required");
  }
  const auto mission_prefix = transaction_id + ":" + std::to_string(effect_sequence);
  const auto ready_body =
    "{\"mission_id\":\"" + json_escape(mission_prefix + ":ready") + "\"}";
  auto ready = run_task(
    "/api/v1/arm/ready", ready_body, cancellation_requested);

  ElevatorArmOutcome action = ready;
  if (ready.succeeded()) {
    const auto action_mission = hall_call ?
      mission_prefix + ":call:" + action_value :
      mission_prefix + ":press-floor:" + action_value;
    const auto action_body =
      "{\"mission_id\":\"" + json_escape(action_mission) + "\",\"" +
      action_field + "\":\"" + json_escape(action_value) + "\"}";
    if (hall_call) {
      const auto response = transport_->request(
        "POST", action_path, action_body, options_.request_timeout);
      std::string parse_error;
      const auto document = json_document(response, parse_error);
      if (
        response.status == 501 && document &&
        string_value(*document, "error_code") == "capability_unavailable")
      {
        action = {
          ElevatorArmOutcomeKind::kManualConfirmationRequired,
          "ELEVATOR_CALL_MANUAL_CONFIRMATION_REQUIRED",
          "8083 reports physical hall-call capability unavailable",
          {},
        };
      } else if (response.status == 202 && document) {
        action = await_task(response, action_path, cancellation_requested);
      } else {
        action = failed(
          "ELEVATOR_CALL_COMMAND_FAILED",
          "hall-call command failed; http_status=" +
          std::to_string(response.status) + ";detail=" + parse_error);
      }
    } else {
      action = run_task(action_path, action_body, cancellation_requested);
    }
  }

  const auto release_body =
    "{\"mission_id\":\"" + json_escape(mission_prefix + ":release") + "\"}";
  const auto release = run_task(
    "/api/v1/arm/release", release_body, []() {return false;});
  if (!release.succeeded()) {
    return failed(
      "ELEVATOR_ARM_RELEASE_FAILED",
      "arm release task failed; release=" + release.code,
      release.task_id);
  }
  if (action.kind == ElevatorArmOutcomeKind::kManualConfirmationRequired) {
    action.detail += "; arm release task completed";
    return action;
  }
  if (!action.succeeded()) {
    action.detail += "; arm release task completed";
    return action;
  }
  return {
    ElevatorArmOutcomeKind::kSucceeded,
    hall_call ? "ELEVATOR_HALL_CALL_PRESSED" : "ELEVATOR_TARGET_FLOOR_PRESSED",
    "button action and arm release tasks completed",
    action.task_id,
  };
}

ElevatorArmOutcome ElevatorArmClient::press_hall_call(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & source_floor_id,
  const std::string & target_floor_id,
  const std::function<bool()> & cancellation_requested)
{
  if (!floor_button_label(source_floor_id)) {
    return failed(
      "ELEVATOR_ARM_SOURCE_FLOOR_UNSUPPORTED",
      "source floor is not one of -2, -1, or 1 through 20");
  }
  if (!floor_button_label(target_floor_id)) {
    return failed(
      "ELEVATOR_ARM_TARGET_FLOOR_UNSUPPORTED",
      "target floor is not one of -2, -1, or 1 through 20");
  }
  const auto direction = hall_call_direction(source_floor_id, target_floor_id);
  if (!direction) {
    return failed(
      "ELEVATOR_ARM_HALL_CALL_DIRECTION_UNRESOLVED",
      "source and target floors must resolve to different numeric levels");
  }
  return run_button_sequence(
    transaction_id, effect_sequence,
    "/api/v1/elevator/call", "direction", *direction,
    cancellation_requested, true);
}

ElevatorArmOutcome ElevatorArmClient::press_floor(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & target_floor_id,
  const std::function<bool()> & cancellation_requested)
{
  const auto label = floor_button_label(target_floor_id);
  if (!label) {
    return failed(
      "ELEVATOR_ARM_TARGET_FLOOR_UNSUPPORTED",
      "target floor is not one of -2, -1, or 1 through 20");
  }
  return run_button_sequence(
    transaction_id, effect_sequence,
    "/api/v1/elevator/press-floor", "floor", *label,
    cancellation_requested, false);
}

}  // namespace robot_api_server
