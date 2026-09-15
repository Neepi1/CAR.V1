// Final startup evidence observer: no service calls, TF changes or cached proof.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rapidjson/document.h>

namespace startup_context
{
bool source_is_current(std::int64_t source_ns, std::int64_t cutoff_ns, std::int64_t now_ns)
{
  return cutoff_ns > 0 && source_ns >= cutoff_ns && source_ns <= now_ns;
}

bool decimal_sequence(const std::string & value)
{
  return !value.empty() && (value.size() == 1 || value.front() != '0') &&
    value.find_first_not_of("0123456789") == std::string::npos;
}

const rapidjson::Value * field(const rapidjson::Value & value, const char * key)
{
  if (!value.IsObject()) {return nullptr;}
  // json.loads, used by the existing observer, retains the LAST duplicate key.
  const rapidjson::Value * found = nullptr;
  for (auto member = value.MemberBegin(); member != value.MemberEnd(); ++member) {
    if (std::string(member->name.GetString(), member->name.GetStringLength()) == key) {
      found = &member->value;
    }
  }
  return found;
}

bool equals_string(const rapidjson::Value * value, const std::string & expected)
{
  return value && value->IsString() &&
    std::string(value->GetString(), value->GetStringLength()) == expected;
}

std::optional<std::string> bridge_sequence(
  const std::string & payload, const std::string & minimum)
{
  rapidjson::Document status;
  // Preserve arbitrary-size integer/numeric-string sequences without rounding
  // them through double. Fractions, exponents, signs and booleans are not ints.
  status.Parse<rapidjson::kParseNumbersAsStringsFlag>(payload.data(), payload.size());
  if (status.HasParseError()) {return std::nullopt;}
  if (status.IsString()) {
    const std::string nested(status.GetString(), status.GetStringLength());
    status.Parse<rapidjson::kParseNumbersAsStringsFlag>(nested.data(), nested.size());
    if (status.HasParseError()) {return std::nullopt;}
  }
  const auto candidate = field(status, "last_explicit_relocalization_sequence");
  const auto has_map = field(status, "has_map_to_odom");
  if (!candidate || !candidate->IsString() || !has_map ||
    !((has_map->IsBool() && has_map->GetBool()) || equals_string(has_map, "true")) ||
    !equals_string(field(status, "map_to_odom_publisher_owner"), "robot_localization_bridge"))
  {
    return std::nullopt;
  }
  const std::string sequence(candidate->GetString(), candidate->GetStringLength());
  if (!decimal_sequence(sequence)) {return std::nullopt;}
  if (minimum != "-1" &&
    !(sequence.size() > minimum.size() ||
    (sequence.size() == minimum.size() && sequence > minimum)))
  {
    return std::nullopt;
  }
  return sequence;
}

std::int64_t positive_integer(const std::string & text)
{
  std::int64_t result = 0;
  if (text.empty() || text.front() == '0' || !decimal_sequence(text)) {return 0;}
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() && result > 0 ?
         result : 0;
}

std::optional<std::int64_t> commit_timestamp(const std::string & text, std::int64_t now_ns)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return std::nullopt;}
  const auto last = text.find_last_not_of(" \t\r\n");
  const auto stamp = positive_integer(text.substr(first, last - first + 1));
  if (stamp <= 0 || stamp > now_ns) {return std::nullopt;}
  return stamp;
}

enum class FileState {missing, present, invalid};
struct FileText {FileState state; std::string text;};

FileText read_text(const std::string & path, std::size_t maximum)
{
  std::error_code error;
  if (!std::filesystem::exists(path, error)) {
    return {error ? FileState::invalid : FileState::missing, {}};
  }
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return {FileState::invalid, {}};
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {return {FileState::invalid, {}};}
  std::string text(maximum + 1, '\0');
  stream.read(text.data(), static_cast<std::streamsize>(text.size()));
  text.resize(static_cast<std::size_t>(stream.gcount()));
  if (stream.bad() || text.size() > maximum) {return {FileState::invalid, {}};}
  return {FileState::present, std::move(text)};
}

bool handoff_record_pending(const std::string & text)
{
  // Match floor_startup_handoff.py:request_pending, not load_request: only a
  // correctly versioned terminal record is history. Malformed data fences us.
  rapidjson::Document record;
  record.Parse(text.data(), text.size());
  if (record.HasParseError() || !record.IsObject()) {return true;}
  const auto version = field(record, "version");
  const bool version_one = version &&
    ((version->IsNumber() && version->GetDouble() == 1.0) ||
    (version->IsBool() && version->GetBool()));  // Python True == 1.
  return !(equals_string(field(record, "schema"), "njrh.floor_startup_handoff.v1") &&
         version_one && (equals_string(field(record, "state"), "committed") ||
         equals_string(field(record, "state"), "failed")));
}

bool handoff_pending(const std::string & path)
{
  const auto record = read_text(path, 1024 * 1024);
  return record.state != FileState::missing &&
         (record.state == FileState::invalid || handoff_record_pending(record.text));
}

struct Options
{
  std::string commit_file;
  double warmup_wait_sec = 0.0;
  double service_wait_sec = 10.0;
  double bridge_wait_sec = 15.0;
  std::string minimum_sequence;
};

Options parse_options(const std::vector<std::string> & arguments)
{
  Options options;
  for (std::size_t index = 1; index < arguments.size(); index += 2) {
    if (index + 1 >= arguments.size()) {throw std::invalid_argument("missing option value");}
    const auto & key = arguments[index];
    const auto & value = arguments[index + 1];
    if (key == "--commit-file") {
      if (value.empty()) {throw std::invalid_argument("commit file must not be empty");}
      options.commit_file = value;
    } else if (key == "--minimum-sequence") {
      if (value != "-1" && !decimal_sequence(value)) {
        throw std::invalid_argument("minimum sequence must be an integer >= -1");
      }
      options.minimum_sequence = value;
    } else if (key == "--warmup-wait-sec" || key == "--service-wait-sec" ||
      key == "--bridge-wait-sec")
    {
      std::size_t used = 0;
      const auto seconds = std::stod(value, &used);
      if (used != value.size() || !std::isfinite(seconds) || seconds < 0.0) {
        throw std::invalid_argument("wait budgets must be finite and non-negative");
      }
      if (key == "--warmup-wait-sec") {options.warmup_wait_sec = seconds;}
      if (key == "--service-wait-sec") {options.service_wait_sec = seconds;}
      if (key == "--bridge-wait-sec") {options.bridge_wait_sec = seconds;}
    } else {throw std::invalid_argument("unknown option: " + key);}
  }
  if (options.minimum_sequence.empty()) {throw std::invalid_argument("minimum sequence required");}
  if (options.warmup_wait_sec > 0.0 && options.commit_file.empty()) {
    throw std::invalid_argument("warmup requires a commit file");
  }
  return options;
}

class Evidence
{
public:
  explicit Evidence(std::string minimum) : minimum_(std::move(minimum)) {}

  void begin_commit(std::int64_t commit_ns, std::int64_t service_ready_ns)
  {
    cutoff_ns_ = std::max(commit_ns, service_ready_ns);
    sequence_.reset();
  }

  void observe(const std::string & payload, std::int64_t source_ns, std::int64_t now_ns)
  {
    if (source_is_current(source_ns, cutoff_ns_, now_ns)) {
      const auto next = bridge_sequence(payload, minimum_);
      if (next) {sequence_ = next;}
    }
  }

  const std::optional<std::string> & sequence() const {return sequence_;}

private:
  std::string minimum_;
  std::int64_t cutoff_ns_ = 0;
  std::optional<std::string> sequence_;
};
}  // namespace startup_context

#ifndef STARTUP_CONTEXT_OBSERVER_CORE_ONLY
#include <cerrno>
#include <csignal>
#include <limits>
#include <thread>
#include <sys/types.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace startup_context
{
using Clock = std::chrono::steady_clock;

std::int64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void log_step(const char * phase, Clock::time_point started, const char * result)
{
  std::cerr << "[runtime-overlay] CONTEXT_STEP phase=" << phase << " elapsed_sec=" <<
    std::fixed << std::setprecision(3) <<
    std::chrono::duration<double>(Clock::now() - started).count() <<
    " result=" << result << std::endl;
}

class Cancellation
{
public:
  Cancellation()
  {
    const auto handoff = std::getenv("NJRH_FLOOR_STARTUP_HANDOFF_FILE");
    handoff_path_ = handoff ? handoff : "/tmp/njrh_floor_startup_handoff.json";
    const auto owner = std::getenv("NJRH_STARTUP_OWNER_PID");
    if (owner && *owner) {
      const auto parsed = positive_integer(owner);
      if (parsed <= 0 || parsed > std::numeric_limits<pid_t>::max()) {
        throw std::invalid_argument("invalid NJRH_STARTUP_OWNER_PID");
      }
      owner_pid_ = static_cast<pid_t>(parsed);
    }
  }

  int result() const
  {
    if (handoff_pending(handoff_path_)) {return 20;}
    if (owner_pid_ && ::kill(*owner_pid_, 0) != 0 && errno != EPERM) {return 1;}
    return 0;
  }

private:
  std::string handoff_path_;
  std::optional<pid_t> owner_pid_;
};

int observe(const Options & options, const Cancellation & cancellation)
{
  const auto started = Clock::now();
  auto node = std::make_shared<rclcpp::Node>(
    "startup_context_observer", rclcpp::NodeOptions().start_parameter_services(false)
    .start_parameter_event_publisher(false).enable_rosout(false));
  Evidence evidence(options.minimum_sequence);
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "/localization/bridge_status", rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile(),
    [&evidence](std_msgs::msg::String::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      evidence.observe(message->data, info.get_rmw_message_info().source_timestamp, system_now_ns());
    });
  // Resolve remapping through the same public node interface used by services;
  // private-domain integration can verify the real observer without production names.
  const auto service_name = node->get_node_base_interface()->resolve_topic_or_service_name(
    "/global_localization/trigger", true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  log_step("observer_start", started, "ready");  // The sole reader now exists.
  auto tick = [&executor]() {
      const auto next = Clock::now() + std::chrono::milliseconds(50);
      executor.spin_some(std::chrono::milliseconds(40));
      std::this_thread::sleep_until(next);
    };
  auto stopped = [&cancellation]() {
      const int cancelled = cancellation.result();
      return cancelled ? cancelled : (rclcpp::ok() ? 0 : 1);
    };
  std::int64_t commit_ns = 0;
  if (!options.commit_file.empty()) {
    const auto warm_started = Clock::now();
    while (true) {
      if (const int result = stopped()) {return result;}
      const auto request = read_text(options.commit_file, 128);
      if (request.state == FileState::invalid) {return 4;}
      if (request.state == FileState::present) {
        const auto parsed = commit_timestamp(request.text, system_now_ns());
        if (!parsed) {return 4;}
        commit_ns = *parsed;
        break;
      }
      if (std::chrono::duration<double>(Clock::now() - warm_started).count() >=
        options.warmup_wait_sec)
      {
        log_step("observer_warmup", warm_started, "expired");
        return 4;
      }
      tick();  // cutoff is zero: EVERY pre-commit sample is discarded.
    }
  }

  const auto service_started = Clock::now();
  bool service_ready = false;
  while (std::chrono::duration<double>(Clock::now() - service_started).count() <
    options.service_wait_sec)
  {
    if (const int result = stopped()) {return result;}
    const auto names = node->get_service_names_and_types();
    const auto found = names.find(service_name);
    if (found != names.end() && !found->second.empty()) {
      service_ready = true;
      break;
    }
    tick();  // Still unarmed; no pre-service evidence survives this phase.
  }
  if (const int result = stopped()) {return result;}
  log_step("wrapper_service", service_started, service_ready ? "ready" : "missing");
  std::cout << "service_ready: " << (service_ready ? "true" : "false") << std::endl;
  if (!service_ready) {return 2;}
  // Sample AFTER graph confirmation. Queued DDS data cannot prove this commit
  // even if it is delivered after we arm or after a suspended process resumes.
  evidence.begin_commit(commit_ns, system_now_ns());
  const auto bridge_started = Clock::now();
  while (!evidence.sequence() &&
    std::chrono::duration<double>(Clock::now() - bridge_started).count() < options.bridge_wait_sec)
  {
    if (const int result = stopped()) {return result;}
    tick();
  }
  if (const int result = stopped()) {return result;}
  log_step("bridge_sequence", bridge_started, evidence.sequence() ? "ready" : "missing");
  if (!evidence.sequence()) {return 3;}
  std::cout << "explicit_sequence: " << *evidence.sequence() << std::endl;
  return 0;
}
}  // namespace startup_context

int main(int argc, char ** argv)
{
  int result = 1;
  try {
    const auto options = startup_context::parse_options(rclcpp::remove_ros_arguments(argc, argv));
    const startup_context::Cancellation cancellation;
    if (const int cancelled = cancellation.result()) {return cancelled;}
    rclcpp::init(argc, argv);
    result = startup_context::observe(options, cancellation);
  } catch (const std::exception & error) {
    std::cerr << "[runtime-overlay] startup context observation failed: " << error.what() << std::endl;
  }
  // observe's executor, reader and node unwind before context shutdown on every
  // success, exception and cancellation path. No static participant survives.
  try {if (rclcpp::ok()) {rclcpp::shutdown();}} catch (...) {}
  return result;
}
#endif
