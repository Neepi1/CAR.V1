#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
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

bool transient_query_failure(const ElevatorArmHttpResponse & response)
{
  if (response.status == 408 || response.status == 429 || response.status == 500 ||
    response.status == 502 || response.status == 503 || response.status == 504)
  {
    return true;
  }
  if (response.status != 599) {
    return false;
  }
  for (const auto * prefix : {"socket failed:", "connect failed:", "send failed:",
      "receive failed:", "request deadline exceeded"})
  {
    if (response.transport_error.rfind(prefix, 0U) == 0U) {
      return true;
    }
  }
  return false;
}

bool cancelled(const std::function<bool()> & probe)
{
  return probe && probe();
}

bool wait_until_or_cancel(
  const std::chrono::steady_clock::time_point deadline,
  const std::function<bool()> & probe)
{
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancelled(probe)) {
      return false;
    }
    std::this_thread::sleep_until(std::min(
      deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(20)));
  }
  return !cancelled(probe);
}

ElevatorArmOutcome cancelled_outcome()
{
  return failed("ELEVATOR_ARM_TASK_CANCELLED", "elevator transaction was cancelled");
}

bool retry_pause(
  const ElevatorArmOutcome & result, const std::string & path,
  const std::uint64_t attempt, const std::function<bool()> & probe)
{
  if (cancelled(probe)) {
    return false;
  }
  std::fprintf(stderr,
    "[elevator-arm] action_retry path=%s attempt=%llu task_id=%s code=%s detail=%s\n",
    path.c_str(), static_cast<unsigned long long>(attempt), result.task_id.c_str(),
    json_escape(result.code).c_str(), json_escape(result.detail).c_str());
  return wait_until_or_cancel(std::chrono::steady_clock::now() + std::chrono::seconds(1), probe);
}

std::string attempt_mission(const std::string & mission, const std::uint64_t attempt)
{
  return attempt == 0U ? mission : mission + ":retry:" + std::to_string(attempt);
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
    const auto deadline = std::chrono::steady_clock::now() +
      std::max(timeout, std::chrono::milliseconds(1));
    const int descriptor = ::socket(AF_INET, SOCK_STREAM, 0);
    if (descriptor < 0) {
      return {599, {}, std::string("socket failed: ") + std::strerror(errno)};
    }
    struct DescriptorGuard
    {
      int value;
      ~DescriptorGuard() {if (value >= 0) {(void)::close(value);}}
    } guard{descriptor};

    const int flags = ::fcntl(descriptor, F_GETFL, 0);
    if (flags < 0 || ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
      return {599, {}, std::string("socket configuration failed: ") + std::strerror(errno)};
    }
    // One monotonic deadline spans connect, every partial send and every recv.
    // Per-syscall SO_RCVTIMEO would renew the budget on a dripping response.
    const auto wait_for_io = [&](const short events) {
        while (true) {
          const auto now = std::chrono::steady_clock::now();
          if (now >= deadline) {return 0;}
          const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
          const int wait_ms = static_cast<int>(std::min<std::int64_t>(
              remaining.count(), std::numeric_limits<int>::max()));
          pollfd pending{descriptor, events, 0};
          const int ready = ::poll(&pending, 1, wait_ms);
          if (ready < 0) {
            if (errno == EINTR) {continue;}
            return -1;
          }
          if (std::chrono::steady_clock::now() >= deadline) {return 0;}
          if (ready == 0) {continue;}
          if ((pending.revents & POLLNVAL) != 0) {errno = EBADF; return -1;}
          // Let connect/recv/send report the original socket error on ERR/HUP.
          if ((pending.revents & (events | POLLERR | POLLHUP)) != 0) {return 1;}
        }
      };

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.port);
    if (::inet_pton(AF_INET, options_.host.c_str(), &address.sin_addr) != 1) {
      return {599, {}, "invalid loopback IPv4 address"};
    }
    if (::connect(
        descriptor, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
    {
      if (errno != EINPROGRESS && errno != EINTR) {
        return {599, {}, std::string("connect failed: ") + std::strerror(errno)};
      }
      const int ready = wait_for_io(POLLOUT);
      if (ready == 0) {return {599, {}, "request deadline exceeded"};}
      if (ready < 0) {
        return {599, {}, std::string("connect failed: ") + std::strerror(errno)};
      }
      int error = 0;
      socklen_t length = sizeof(error);
      if (::getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &error, &length) != 0) {
        error = errno;
      }
      if (error != 0) {
        return {599, {}, std::string("connect failed: ") + std::strerror(error)};
      }
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
      if (std::chrono::steady_clock::now() >= deadline) {
        return {599, {}, "request deadline exceeded"};
      }
      const auto count = ::send(
        descriptor, wire.data() + sent, wire.size() - sent, MSG_NOSIGNAL);
      if (count < 0 && errno == EINTR) {continue;}
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const int ready = wait_for_io(POLLOUT);
        if (ready == 0) {return {599, {}, "request deadline exceeded"};}
        if (ready > 0) {continue;}
      }
      if (count <= 0) {
        return {599, {}, std::string("send failed: ") + std::strerror(errno)};
      }
      sent += static_cast<std::size_t>(count);
    }

    std::string response;
    char buffer[4096];
    while (true) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return {599, {}, "request deadline exceeded"};
      }
      const auto count = ::recv(descriptor, buffer, sizeof(buffer), 0);
      if (count < 0 && errno == EINTR) {continue;}
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        const int ready = wait_for_io(POLLIN);
        if (ready == 0) {return {599, {}, "request deadline exceeded"};}
        if (ready > 0) {continue;}
      }
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
  const std::function<bool()> & cancellation_requested,
  bool * confirmed_task_failure)
{
  if (confirmed_task_failure) {
    *confirmed_task_failure = false;
  }
  if (cancelled(cancellation_requested)) {
    return cancelled_outcome();
  }
  const auto submitted = transport_->request(
    "POST", path, request_body, options_.request_timeout);
  return await_task(submitted, path, cancellation_requested, confirmed_task_failure);
}

ElevatorArmOutcome ElevatorArmClient::await_task(
  const ElevatorArmHttpResponse & submitted,
  const std::string & path,
  const std::function<bool()> & cancellation_requested,
  bool * confirmed_task_failure)
{
  if (confirmed_task_failure) {
    *confirmed_task_failure = false;
  }
  std::string parse_error;
  const auto accepted = json_document(submitted, parse_error);
  if (path == "/api/v1/elevator/call" && submitted.status == 501 && accepted &&
    string_value(*accepted, "error_code") == "capability_unavailable")
  {
    return {ElevatorArmOutcomeKind::kManualConfirmationRequired,
      "ELEVATOR_CALL_MANUAL_CONFIRMATION_REQUIRED",
      "8083 reports physical hall-call capability unavailable", {}};
  }
  if (submitted.status != 202 || !accepted) {
    std::string code = accepted ? string_value(*accepted, "error_code") : std::string{};
    if (path == "/api/v1/elevator/call") {
      code = "ELEVATOR_CALL_COMMAND_FAILED";
    } else if (code.empty()) {
      code = "ELEVATOR_ARM_COMMAND_REJECTED";
    }
    return failed(
      code,
      "arm command acceptance was not confirmed; path=" + path + ";http_status=" +
      std::to_string(submitted.status) + ";detail=" + parse_error + ";message=" +
      (accepted ? string_value(*accepted, "message", string_value(*accepted, "error")) : ""));
  }
  const auto task_id = string_value(*accepted, "task_id");
  if (task_id.empty()) {
    return failed(
      "ELEVATOR_ARM_TASK_ID_INVALID",
      "accepted arm command did not return a task_id");
  }

  const auto deadline = std::chrono::steady_clock::now() + options_.task_timeout;
  std::string last_query_error;
  auto next_query_log = std::chrono::steady_clock::time_point{};
  while (std::chrono::steady_clock::now() <= deadline) {
    if (cancellation_requested && cancellation_requested()) {
      return failed(
        "ELEVATOR_ARM_TASK_CANCELLED",
        "elevator transaction was cancelled while waiting for arm task",
        task_id);
    }
    const auto polled = transport_->request(
      "GET", "/api/v1/tasks/" + task_id, {}, std::min(options_.request_timeout,
      std::max(std::chrono::milliseconds(1), std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()))));
    if (cancelled(cancellation_requested)) {
      return failed("ELEVATOR_ARM_TASK_CANCELLED",
        "elevator transaction was cancelled while waiting for arm task", task_id);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      last_query_error = "task deadline elapsed after GET;http_status=" +
        std::to_string(polled.status) + ";detail=" + polled.transport_error;
      break;
    }
    parse_error.clear();
    const auto task = json_document(polled, parse_error);
    if (polled.status != 200 || !task) {
      last_query_error = "http_status=" + std::to_string(polled.status) +
        ";detail=" + parse_error;
      if (transient_query_failure(polled)) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_query_log) {
          std::fprintf(stderr, "[elevator-arm] query_retry task_id=%s %s\n",
            task_id.c_str(), json_escape(last_query_error).c_str());
          next_query_log = now + std::chrono::seconds(1);
        }
        (void)wait_until_or_cancel(std::min(deadline,
          now + std::max(options_.poll_interval, std::chrono::milliseconds(20))),
          cancellation_requested);
        continue;
      }
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
      if (confirmed_task_failure) {
        *confirmed_task_failure = state == "failed";
      }
      auto code = string_value(*task, "error_code", "ELEVATOR_ARM_TASK_FAILED");
      return failed(
        code,
        "arm task failed; task_id=" + task_id + ";message=" +
        string_value(*task, "message"),
        task_id);
    }
    (void)wait_until_or_cancel(std::min(deadline,
      std::chrono::steady_clock::now() + options_.poll_interval), cancellation_requested);
  }
  return failed(
    "ELEVATOR_ARM_TASK_TIMEOUT",
    "arm task did not reach a terminal state before timeout; task_id=" + task_id +
    ";last_query_error=" + last_query_error,
    task_id);
}

ElevatorArmOutcome ElevatorArmClient::run_button_sequence(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & action_path,
  const std::string & action_field,
  const std::string & action_value,
  const std::function<bool()> & cancellation_requested,
  const bool hall_call,
  const ElevatorArmRuntimeFailureProbe & runtime_failure)
{
  if (transaction_id.empty() || effect_sequence == 0U) {
    return failed(
      "ELEVATOR_ARM_INVALID_TRANSACTION",
      "transaction_id and positive effect_sequence are required");
  }
  const auto mission_prefix = transaction_id + ":" + std::to_string(effect_sequence);
  std::optional<ElevatorArmOutcome> runtime_fault;
  // Reuse the existing interruptible waits, but keep the local fault distinct
  // from user cancellation and from a remote arm task's terminal state.
  const std::function<bool()> interrupted = [&]() {
      if (!runtime_fault && runtime_failure) {
        try {
          runtime_fault = runtime_failure();
          if (runtime_fault) {runtime_fault->kind = ElevatorArmOutcomeKind::kFailed;}
        } catch (const std::exception & exception) {
          runtime_fault = failed("ELEVATOR_RUNTIME_HEALTH_EXCEPTION", exception.what());
        } catch (...) {
          runtime_fault = failed("ELEVATOR_RUNTIME_HEALTH_EXCEPTION",
            "unknown exception while observing the elevator runtime");
        }
      }
      return runtime_fault.has_value() || cancelled(cancellation_requested);
    };
  const auto interrupted_outcome = [&]() {
      return runtime_fault ? *runtime_fault : cancelled_outcome();
    };
  if (interrupted()) {
    return interrupted_outcome();
  }
  std::uint64_t ready_attempt = 0U;
  std::uint64_t action_attempt = 0U;
  std::uint64_t release_attempt = 0U;
  const auto posture_task = [&](const std::string & name, std::uint64_t & attempt,
      const std::function<bool()> & probe, const bool retry) {
      const auto path = "/api/v1/arm/" + name;
      while (true) {
        const auto body = "{\"mission_id\":\"" +
          json_escape(attempt_mission(mission_prefix + ":" + name, attempt++)) + "\"}";
        bool confirmed_failure = false;
        auto result = run_task(path, body, probe, &confirmed_failure);
        if (!retry || !confirmed_failure) {
          return result;
        }
        if (!retry_pause(result, path, attempt, interrupted)) {
          // Retain the last release result for fault cleanup diagnostics.
          return runtime_fault ? result : cancelled_outcome();
        }
      }
  };

  while (true) {
    auto action = posture_task("ready", ready_attempt, interrupted, true);
    bool action_failed = false;
    if (action.succeeded()) {
      const auto action_mission = mission_prefix +
        (hall_call ? ":call:" : ":press-floor:") + action_value;
      const auto action_body = "{\"mission_id\":\"" +
        json_escape(attempt_mission(action_mission, action_attempt++)) + "\",\"" +
        action_field + "\":\"" + json_escape(action_value) + "\"}";
      action = run_task(action_path, action_body, interrupted, &action_failed);
    }

    // Preserve the existing one-shot release on cancellation. Never turn that
    // uncancellable cleanup into an infinite retry or resubmit a pending release.
    const bool cancelling_cleanup = interrupted();
    // A cancellation arriving between this decision and submission must not
    // skip the existing release cleanup. Each release remains time-bounded;
    // retry_pause checks the real cancellation flag before any further attempt.
    const std::function<bool()> release_cancel = [] {return false;};
    const auto release = posture_task("release", release_attempt, release_cancel,
      !cancelling_cleanup);
    (void)interrupted();
    if (runtime_fault) {
      auto result = *runtime_fault;
      if (result.task_id.empty()) {result.task_id = action.task_id;}
      result.detail += "; arm release cleanup=" + release.code + ";detail=" + release.detail;
      return result;
    }
    if (!release.succeeded()) {
      return failed("ELEVATOR_ARM_RELEASE_FAILED",
        "arm release task failed; release=" + release.code + ";detail=" + release.detail +
        ";button_result=" + action.code + ";button_detail=" + action.detail,
        release.task_id);
    }
    if (cancelled(cancellation_requested)) {
      return cancelled_outcome();
    }
    if (action_failed) {
      if (!retry_pause(action, action_path, action_attempt, interrupted)) {
        return interrupted_outcome();
      }
      continue;
    }
    if (!action.succeeded()) {
      action.detail += "; arm release task completed";
      return action;
    }
    return {ElevatorArmOutcomeKind::kSucceeded,
      hall_call ? "ELEVATOR_HALL_CALL_PRESSED" : "ELEVATOR_TARGET_FLOOR_PRESSED",
      "button action and arm release tasks completed", action.task_id};
  }
}

ElevatorArmOutcome ElevatorArmClient::press_hall_call(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & source_floor_id,
  const std::string & target_floor_id,
  const std::function<bool()> & cancellation_requested,
  const ElevatorArmRuntimeFailureProbe & runtime_failure)
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
    cancellation_requested, true, runtime_failure);
}

ElevatorArmOutcome ElevatorArmClient::press_floor(
  const std::string & transaction_id,
  const std::uint64_t effect_sequence,
  const std::string & target_floor_id,
  const std::function<bool()> & cancellation_requested,
  const ElevatorArmRuntimeFailureProbe & runtime_failure)
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
    cancellation_requested, false, runtime_failure);
}

}  // namespace robot_api_server
