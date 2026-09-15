#include "robot_bringup/runtime_amcl_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <system_error>
#include <limits.h>
#include <poll.h>
#include <sys/epoll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>

namespace robot_bringup::runtime_amcl {
namespace {
struct Fd {
  int value{-1};
  explicit Fd(int f = -1) : value(f) {}
  ~Fd() { if (value >= 0) close(value); }
  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;
};
void require(bool good, const char *reason) { if (!good) throw std::runtime_error(reason); }
std::string read_link(const std::string &path) {
  char buf[PATH_MAX + 1]; const auto n = readlink(path.c_str(), buf, PATH_MAX);
  if (n < 0) throw std::system_error(errno, std::generic_category(), "process readlink");
  require(n > 0 && n < PATH_MAX, "invalid process executable link");
  return {buf, static_cast<size_t>(n)};
}
std::string canonical(const std::string &path) {
  return std::filesystem::canonical(path).string();
}
std::string read_proc(const std::string &path) {
  Fd fd(open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK));
  if (fd.value < 0) throw std::system_error(errno, std::generic_category(), "process open");
  std::string result;
  char buffer[4096];
  for (;;) {
    const auto n = read(fd.value, buffer, sizeof(buffer));
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) throw std::system_error(errno, std::generic_category(), "process read");
    if (n == 0) return result;
    result.append(buffer, static_cast<size_t>(n));
    require(result.size() <= 1024 * 1024, "process input too large");
  }
}
bool process_directory_gone(const std::string &base, const std::string &root) {
  struct stat st{};
  // A missing/mis-mounted proc root is an observation failure, not a death.
  if (stat(root.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) return false;
  return lstat(base.c_str(), &st) != 0 && errno == ENOENT;
}
std::string decimal(double n) {
  std::ostringstream out; out << std::fixed << std::setprecision(6) << n; return out.str();
}
void atomic_write(const std::string &path, const std::string &data) {
  std::string temp = path + ".XXXXXX";
  Fd fd(mkstemp(temp.data())); require(fd.value >= 0, "status temporary file unavailable");
  try {
    // Public status was historically readable by API/bridge users. Keep those
    // read bits, but never grant another UID write or execute permission.
    struct stat previous{};
    mode_t mode = 0644;
    if (lstat(path.c_str(), &previous) == 0 && S_ISREG(previous.st_mode))
      mode = 0600 | (previous.st_mode & 0044);
    require(fchmod(fd.value, mode) == 0, "status permissions failed");
    size_t offset = 0;
    while (offset < data.size()) {
      const auto n = write(fd.value, data.data() + offset, data.size() - offset);
      if (n < 0 && errno == EINTR) continue;
      require(n > 0, "status write failed"); offset += static_cast<size_t>(n);
    }
    require(rename(temp.c_str(), path.c_str()) == 0, "status commit failed");
  } catch (...) { unlink(temp.c_str()); throw; }
}
std::string env_document(const Fields &fields) {
  std::ostringstream out;
  for (const auto &[key, value] : fields) {
    out << key << "=\"";
    for (char c : value) {
      if (c == '\\' || c == '"' || c == '$' || c == '`') out << '\\';
      out << c;
    }
    out << "\"\n";
  }
  return out.str();
}
sockaddr_un address(const std::string &path) {
  sockaddr_un a{}; a.sun_family = AF_UNIX;
  require(path.size() < sizeof(a.sun_path), "AMCL status socket path too long");
  std::memcpy(a.sun_path, path.c_str(), path.size() + 1); return a;
}
void await_fd(int fd, short events, double deadline) {
  for (;;) {
    const auto left = deadline - json::monotonic_now();
    require(left > 0, "AMCL status request timed out");
    pollfd p{fd, events, 0};
    const int n = poll(&p, 1, std::max(1, static_cast<int>(std::ceil(left * 1000))));
    if (n < 0 && errno == EINTR) continue;
    require(n > 0, "AMCL status request timed out");
    if (p.revents & events) return;
    require(!(p.revents & (POLLERR | POLLHUP | POLLNVAL)), "AMCL status socket closed");
  }
}
std::string required_text(const json::Json &j, const char *key) {
  const auto &v = json::field(j, key);
  require(v.IsString(), "missing string field");
  const std::string s(v.GetString(), v.GetStringLength());
  require(text_safe(s), "invalid text field"); return s;
}
void validate_evidence(const Fields &f) {
  const auto mode = get(f, "AMCL_MODE"), state = get(f, "AMCL_START_RESULT");
  require(mode == "disabled" || mode == "shadow" || mode == "gated", "invalid AMCL mode");
  require(std::set<std::string>{"disabled", "starting", "waiting_seed", "ready", "degraded", "failed", "stopped"}.count(state), "invalid AMCL result");
  for (const auto &[key, value] : f) {
    if (key == "AMCL_READY" || key == "AMCL_DEGRADED" || key == "LIFECYCLE_VERIFIED" ||
        key == "SCAN_ADMISSION_ENABLED" || key.find("STALE_CLEARED") != std::string::npos ||
        key == "AMCL_SEED_SUCCEEDED" || key == "AMCL_SEED_RESPONSE_OK" ||
        key == "AMCL_NOMOTION_PROBE_USED" || key == "AMCL_NOMOTION_POSE_RECEIVED" ||
        key == "AMCL_STATIC_STANDBY_ACCEPTED")
      require(value == "true" || value == "false", "invalid boolean evidence");
  }
  require(get(f, "PROGRESS_KEY").empty() ||
    (get(f, "PROGRESS_KEY").size() == 64 &&
    get(f, "PROGRESS_KEY").find_first_not_of("0123456789abcdef") == std::string::npos), "invalid progress identity");
  for (const char *key : {"AMCL_STARTUP_EPOCH_SEC", "AMCL_NOMOTION_POSE_COUNT", "AMCL_NOMOTION_POSE_HEADER_AGE_MS"}) {
    const auto value = get(f, key);
    if (value.empty()) continue;
    size_t consumed = 0; const double n = std::stod(value, &consumed);
    require(consumed == value.size() && std::isfinite(n), "invalid numeric evidence");
    if (std::string(key) != "AMCL_NOMOTION_POSE_HEADER_AGE_MS")
      require(n >= 0, "negative count/startup time");
    if (std::string(key) == "AMCL_NOMOTION_POSE_COUNT")
      require(value.find_first_not_of("0123456789") == std::string::npos, "pose count must be an integer");
  }
}
}  // namespace

ProcessIdentity process_identity(int pid, const std::string &expected_executable,
  const std::string &argument, const std::string &proc_root) {
  if (pid <= 0) return {};
  const auto base = proc_root + "/" + std::to_string(pid);
  try {
    const auto stat = read_proc(base + "/stat");
    const auto close = stat.rfind(") "); require(close != std::string::npos, "invalid process stat");
    std::istringstream fields(stat.substr(close + 2));
    std::string field; uint64_t ticks = 0;
    for (int i = 0; i <= 19; ++i) {
      require(static_cast<bool>(fields >> field), "incomplete process stat");
      if (i == 0 && (field == "Z" || field == "X")) return {};
      if (i == 19) {
        require(!field.empty() && field.find_first_not_of("0123456789") == std::string::npos,
          "invalid process start ticks");
        ticks = std::stoull(field);
      }
    }
    require(ticks > 0, "missing process start ticks");
    const auto exe = read_link(base + "/exe");
    if (!expected_executable.empty() && exe != canonical(expected_executable)) return {};
    if (!argument.empty()) {
      const auto args = read_proc(base + "/cmdline");
      require(!args.empty() && args.back() == '\0', "incomplete process command line");
      std::istringstream input(args); bool found = false;
      while (std::getline(input, field, '\0')) if (field == argument) found = true;
      if (!found) return {};
    }
    return {pid, ticks, exe, argument};
  } catch (const std::system_error &e) {
    if ((e.code().value() == ENOENT || e.code().value() == ESRCH) &&
        process_directory_gone(base, proc_root)) return {};
    throw;
  }
}
bool same_process(const ProcessIdentity &a, const ProcessIdentity &b) {
  return a.pid > 0 && a.pid == b.pid && a.start_ticks == b.start_ticks &&
    a.executable == b.executable && a.argument == b.argument;
}
std::string socket_path(const std::string &path) { return path + ".control/socket"; }

std::string request(const std::string &path, const std::string &payload, int timeout_ms) {
  require(timeout_ms > 0 && timeout_ms <= 5000, "request timeout must be 1..5000 ms");
  require(!payload.empty() && payload.size() <= max_packet_bytes, "request size invalid");
  const double deadline = json::monotonic_now() + timeout_ms / 1000.0;
  Fd fd(socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
  require(fd.value >= 0, "cannot create AMCL status socket");
  const auto addr = address(socket_path(path));
  int rc = connect(fd.value, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
  if (rc != 0) {
    require(errno == EINPROGRESS || errno == EAGAIN, "AMCL status observer unavailable");
    await_fd(fd.value, POLLOUT, deadline);
    int error = 0; socklen_t len = sizeof(error);
    require(getsockopt(fd.value, SOL_SOCKET, SO_ERROR, &error, &len) == 0 && error == 0,
      "AMCL status connection failed");
  }
  ucred peer{}; socklen_t peer_length = sizeof(peer);
  require(getsockopt(fd.value, SOL_SOCKET, SO_PEERCRED, &peer, &peer_length) == 0 &&
    (peer.uid == 0 || peer.uid == geteuid()), "unauthorized AMCL status server");
  await_fd(fd.value, POLLOUT, deadline);
  require(send(fd.value, payload.data(), payload.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(payload.size()),
    "AMCL status request send failed");
  await_fd(fd.value, POLLIN, deadline);
  char buffer[max_packet_bytes + 1];
  const auto n = recv(fd.value, buffer, sizeof(buffer), MSG_TRUNC);
  require(n > 0 && n <= static_cast<ssize_t>(max_packet_bytes), "invalid AMCL status response");
  return {buffer, static_cast<size_t>(n)};
}

struct RuntimeState {
  Fields evidence, published;
  ProcessIdentity owner, amcl, relay;
  std::string owner_generation, map_generation, progress_key;
  uint64_t generation{0};
  bool lifecycle{false}, has_evidence{false}, invalidated{false};
  bool graph_seen{false}; GraphObservation graph;
  double graph_at{0}, pose_received{0}, pose_header{NAN}, pose_ros{NAN};
  double registered_at{0}, last_write{0};
  std::string map_owner;
};

struct StatusServer::Impl : RuntimeState {
  std::string path, socket, private_dir, boot;
  Options options;
  Fd lock, listener, poller;
  bool process_observation_unknown{false};
  std::string observer_error;
  double last_error_log{0};
  struct Client { int fd; double deadline; };
  std::vector<Client> pending;

  Impl(std::string p, Options o) : path(std::move(p)), socket(socket_path(path)),
    private_dir(path + ".control"), boot(boot_id(o.proc_root)), options(std::move(o)) {
    require(options.refresh_sec > 0 && options.ttl_sec > options.refresh_sec && options.startup_grace_sec > 0,
      "invalid AMCL observation periods");
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    require(mkdir(private_dir.c_str(), 0700) == 0 || errno == EEXIST, "cannot create private AMCL control directory");
    struct stat st{};
    require(lstat(private_dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode) &&
      st.st_uid == geteuid() && (st.st_mode & 0077) == 0, "unsafe AMCL control directory");
    lock.value = open((private_dir + "/writer.lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    require(lock.value >= 0 && flock(lock.value, LOCK_EX | LOCK_NB) == 0,
      "another AMCL status writer is already running");
    listener.value = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    require(listener.value >= 0, "cannot create AMCL status listener");
    const auto addr = address(socket);
    unlink(socket.c_str());
    require(bind(listener.value, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) == 0,
      "cannot bind AMCL status listener");
    require(chmod(socket.c_str(), 0600) == 0 && listen(listener.value, 16) == 0,
      "cannot enable AMCL status listener");
    poller.value = epoll_create1(EPOLL_CLOEXEC);
    epoll_event event{}; event.events = EPOLLIN; event.data.fd = listener.value;
    require(poller.value >= 0 && epoll_ctl(poller.value, EPOLL_CTL_ADD, listener.value, &event) == 0,
      "cannot create AMCL status poll set");
  }
  ~Impl() {
    for (const auto &c : pending) close(c.fd);
    if (listener.value >= 0) unlink(socket.c_str());
  }
  void tick() {
    try { serve(); commit(); }
    catch (const std::exception &e) {
      observer_error = e.what();
      const double now = json::monotonic_now();
      if (now - last_error_log >= 5.0) {
        std::cerr << "[runtime-amcl-status] periodic refresh deferred: " << observer_error << '\n';
        last_error_log = now;
      }
    }
  }
  enum class ProcessState { alive, gone, unknown };
  ProcessState process_state(const ProcessIdentity &p) const {
    if (p.pid <= 0) return ProcessState::gone;
    try {
      return same_process(p, process_identity(p.pid, p.executable, p.argument, options.proc_root)) ?
        ProcessState::alive : ProcessState::gone;
    } catch (...) { return ProcessState::unknown; }
  }
  bool live(const ProcessIdentity &p) const {
    const auto state = process_state(p);
    require(state != ProcessState::unknown, "AMCL_PROCESS_OBSERVATION_UNKNOWN");
    return state == ProcessState::alive;
  }
  void clear_observations() {
    lifecycle = false; graph_seen = false; graph = {}; pose_received = 0;
    pose_header = pose_ros = NAN; map_owner.clear(); invalidated = false;
    ++generation;
  }
  void commit(bool force = false) {
    RuntimeState before = *this;
    try { commit_status(force); }
    catch (...) { static_cast<RuntimeState &>(*this) = std::move(before); throw; }
  }
  void commit_status(bool force) {
    if (!has_evidence) return;
    const double m = json::monotonic_now(), wall = json::wall_now();
    if (!force && m - last_write < options.refresh_sec) return;
    const auto os = process_state(owner), as = process_state(amcl), rs = process_state(relay);
    process_observation_unknown = os == ProcessState::unknown || as == ProcessState::unknown ||
      (yes(evidence, "SCAN_ADMISSION_ENABLED") && rs == ProcessState::unknown);
    if (process_observation_unknown) {
      // Preserve receipts and their timestamp. Existing consumers observe TTL
      // expiry, not a fabricated process death or a fresh ready assertion.
      observer_error = "AMCL_PROCESS_OBSERVATION_UNKNOWN";
      require(!force, "AMCL_PROCESS_OBSERVATION_UNKNOWN");
      return;
    }
    const bool owner_live = os == ProcessState::alive, amcl_live = as == ProcessState::alive,
      relay_live = rs == ProcessState::alive;
    const bool dead_incarnation = (amcl.pid > 0 && !amcl_live) || !owner_live;
    if (dead_incarnation && !invalidated) {
      clear_observations(); invalidated = true;
      for (const char *key : {"AMCL_SEED_SUCCEEDED", "AMCL_SEED_RESPONSE_OK", "AMCL_NOMOTION_POSE_RECEIVED", "AMCL_STATIC_STANDBY_ACCEPTED"}) evidence[key] = "false";
    }
    auto result = get(evidence, "AMCL_START_RESULT"), reason = get(evidence, "AMCL_FAILURE_REASON");
    const auto mode = get(evidence, "AMCL_MODE", "disabled");
    const bool disabled = mode == "disabled" || result == "disabled";
    bool ready = yes(evidence, "AMCL_READY"), degraded = yes(evidence, "AMCL_DEGRADED");
    const bool active = amcl_live && lifecycle && owner_live;
    const bool seed = active && (yes(evidence, "AMCL_SEED_SUCCEEDED") || yes(evidence, "AMCL_SEED_RESPONSE_OK"));
    const bool static_allowed = yes(evidence, "AMCL_STATIC_STANDBY_ACCEPTED") || yes(evidence, "AMCL_NOMOTION_POSE_RECEIVED");
    const bool scan_ok = !yes(evidence, "SCAN_ADMISSION_ENABLED") || relay_live;
    if (!owner_live) { result = "stopped"; ready = false; degraded = false; reason = "AMCL_RUNTIME_OWNER_EXITED"; }
    else if (!disabled && result != "stopped" && result != "failed") {
      if (!amcl_live || !scan_ok) {
        ready = false;
        if (result == "starting" && m - registered_at <= options.startup_grace_sec) {
          reason = "resident AMCL startup is still in progress";
        } else { result = "failed"; degraded = true; reason = "AMCL_HEARTBEAT_PROCESS_NOT_ALIVE"; }
      } else if (!active) {
        ready = false; result = "starting"; reason = "AMCL lifecycle activation is not verified";
      } else if (!seed && result == "ready") {
        ready = false; result = "waiting_seed"; reason = "AMCL seed has not completed";
      } else if (seed && result == "waiting_seed") { result = "ready"; ready = true; reason.clear(); }
    }
    if (result == "failed" || result == "stopped") ready = false;
    if (disabled && owner_live && result != "stopped" && result != "failed") {
      ready = true; degraded = false; result = "disabled"; reason.clear();
    }
    const bool permitted = !disabled && result != "failed" && result != "stopped";
    const bool tracking = permitted && active && seed && (static_allowed || (ready && scan_ok));
    const double pose_age = pose_received > 0 ? (m - pose_received + pose_ros - pose_header) : NAN;
    // Preserve the existing public readiness contract. Candidate freshness and
    // mode-specific correction admission remain the bridge's responsibility.
    const bool correction = tracking && ready && !static_allowed;
    const bool standby = permitted && active && seed && (static_allowed || !ready);
    const bool graph_fresh = graph_seen && m - graph_at <= 10.0 && graph.available;
    Fields f;
    auto put = [&f](const std::string &k, bool v) { f[k] = v ? "true" : "false"; };
    for (const auto &key : input_fields()) if (key.rfind("AMCL_", 0) == 0 || key.rfind("SCAN_ADMISSION_", 0) == 0)
      f[key] = get(evidence, key);
    for (const char *key : {"AMCL_PID_STALE_CLEARED", "SCAN_ADMISSION_PID_STALE_CLEARED", "AMCL_SEED_SUCCEEDED", "AMCL_SEED_RESPONSE_OK", "AMCL_NOMOTION_PROBE_USED", "AMCL_NOMOTION_POSE_RECEIVED", "AMCL_STATIC_STANDBY_ACCEPTED"})
      if (f[key].empty()) f[key] = "false";
    f["AMCL_STATUS_STAMP_SEC"] = decimal(wall); f["AMCL_STATUS_AGE_MS"] = "0";
    put("AMCL_STATUS_STALE", false); f["AMCL_STATUS_TTL_SEC"] = decimal(options.ttl_sec);
    f["AMCL_MODE"] = mode; f["AMCL_START_RESULT"] = result;
    const std::map<std::string, std::string> states{{"disabled","AMCL_DISABLED"}, {"starting","AMCL_STARTING"},
      {"waiting_seed","AMCL_WAITING_SEED"}, {"ready","AMCL_READY"}, {"degraded","AMCL_DEGRADED"},
      {"stopped","AMCL_FAILED"}, {"failed","AMCL_FAILED"}};
    f["AMCL_STATE"] = states.at(result); put("AMCL_READY", ready); put("AMCL_DEGRADED", degraded);
    f["AMCL_FAILURE_REASON"] = reason; f["AMCL_DEGRADED_REASON"] = reason;
    put("AMCL_NODE_EXISTS", graph_fresh && graph.node_exists); put("AMCL_LIFECYCLE_ACTIVE", active);
    f["AMCL_PID"] = amcl.pid ? std::to_string(amcl.pid) : "";
    put("AMCL_PID_ALIVE", amcl_live); put("AMCL_PROCESS_ALIVE", amcl_live); put("AMCL_PROCESS_READY", active);
    f["SCAN_ADMISSION_PID"] = relay.pid ? std::to_string(relay.pid) : "";
    put("SCAN_ADMISSION_ALIVE", relay_live);
    f["SCAN_ADMISSION_STATUS_PUBLISHER_COUNT"] = std::to_string(graph_fresh ? graph.admission_status_publishers : 0);
    f["SCAN_AMCL_PUBLISHER_COUNT"] = std::to_string(graph_fresh ? graph.scan_publishers : 0);
    f["AMCL_POSE_PUBLISHER_COUNT"] = std::to_string(graph_fresh ? graph.pose_publishers : 0);
    f["AMCL_LAST_POSE_AGE_MS"] = std::isfinite(pose_age) ? decimal(pose_age * 1000) : "";
    f["AMCL_POSE_LAST_RECEIVE_AGE_MS"] = pose_received > 0 ? decimal((m - pose_received) * 1000) : "";
    if (f["AMCL_NOMOTION_POSE_COUNT"].empty()) f["AMCL_NOMOTION_POSE_COUNT"] = "0";
    put("AMCL_SEEDED", seed); put("AMCL_TRACKING_READY", tracking); put("AMCL_CORRECTION_READY", correction);
    put("AMCL_STATIC_STANDBY", standby); put("AMCL_NOT_MOVING_NO_UPDATE_OK", standby);
    f["MAP_TO_ODOM_OWNER"] = map_owner;
    std::time_t seconds = static_cast<std::time_t>(wall); std::tm tm{}; gmtime_r(&seconds, &tm);
    char timestamp[32]; std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &tm); f["TIMESTAMP"] = timestamp;
    f["AMCL_STATUS_WRITER"] = "runtime_health_guard"; f["AMCL_STATUS_BOOT_ID"] = boot;
    f["AMCL_STATUS_GENERATION"] = std::to_string(generation);
    f["AMCL_STATUS_OWNER_GENERATION"] = owner_generation;
    f["AMCL_STATUS_OWNER_PID"] = std::to_string(owner.pid);
    f["AMCL_STATUS_OWNER_START_TICKS"] = std::to_string(owner.start_ticks);
    f["AMCL_STATUS_AMCL_START_TICKS"] = std::to_string(amcl.start_ticks);
    f["AMCL_STATUS_MAP_GENERATION"] = map_generation;
    atomic_write(path, env_document(f)); published = std::move(f); last_write = m;
    observer_error.clear();
  }
  std::string dispatch(const std::string &payload) {
    rapidjson::Document d; d.Parse(payload.data(), payload.size());
    require(!d.HasParseError() && d.IsObject(), "invalid AMCL request JSON");
    std::set<std::string> names;
    for (auto i = d.MemberBegin(); i != d.MemberEnd(); ++i)
      require(names.emplace(i->name.GetString()).second, "duplicate request member");
    require(required_text(d, "schema") == "njrh.amcl.evidence.v1", "invalid AMCL request schema");
    const auto op = required_text(d, "operation");
    require(op == "ping" || op == "read" || op == "register" || op == "submit", "unknown AMCL operation");
    if (op == "register" || op == "submit") {
      require(required_text(d, "boot_id") == boot, "evidence from another boot");
      const auto new_owner = identity_from_json(json::field(d, "owner"));
      require(live(new_owner), "evidence owner identity no longer alive");
      const auto og = required_text(d, "owner_generation"), mg = required_text(d, "map_generation");
      require(!og.empty() && !mg.empty(), "owner/map generation required");
      const bool same_context = has_evidence && same_process(new_owner, owner) && og == owner_generation && mg == map_generation;
      if (!same_context && has_evidence && live(owner))
        require(op == "register" && same_process(new_owner, owner), "stale or competing AMCL owner generation");
      auto next = parse_fields(json::field(d, "fields")); validate_evidence(next);
      const auto new_amcl = identity_from_json(json::field(d, "amcl"));
      const auto new_relay = identity_from_json(json::field(d, "relay"));
      require(new_amcl.pid == 0 || live(new_amcl), "AMCL process identity changed before commit");
      require(new_relay.pid == 0 || live(new_relay), "relay process identity changed before commit");
      require(new_amcl.pid > 0 || (!yes(next, "LIFECYCLE_VERIFIED") && !yes(next, "AMCL_SEED_SUCCEEDED") &&
        !yes(next, "AMCL_SEED_RESPONSE_OK")), "lifecycle/seed receipt requires an identified live AMCL process");
      const auto pk = get(next, "PROGRESS_KEY");
      if (same_context && same_process(new_amcl, amcl) && !progress_key.empty() && !pk.empty())
        require(pk == progress_key, "startup progress belongs to a different map incarnation");
      const bool same_amcl = same_context && same_process(new_amcl, amcl) && !invalidated &&
        get(next, "AMCL_MODE") == get(evidence, "AMCL_MODE") &&
        (progress_key.empty() || pk.empty() || pk == progress_key);
      RuntimeState before = *this;
      try {
        if (op == "register" && same_context) { commit(true); }
        else {
        if (same_amcl && get(next, "AMCL_START_RESULT") != "stopped" && get(next, "AMCL_START_RESULT") != "failed") {
          for (const char *key : {"AMCL_SEED_SUCCEEDED", "AMCL_SEED_RESPONSE_OK", "AMCL_NOMOTION_PROBE_USED",
            "AMCL_NOMOTION_POSE_RECEIVED", "AMCL_STATIC_STANDBY_ACCEPTED"})
            if (yes(evidence, key)) next[key] = "true";
        } else { clear_observations(); }
        if (!same_context) registered_at = json::monotonic_now();
        owner = new_owner; owner_generation = og; map_generation = mg;
        amcl = new_amcl; relay = new_relay;
        if (!same_amcl || !pk.empty()) progress_key = pk;
        evidence = std::move(next); has_evidence = true;
        // Activation is a receipt, never inferred from PID or graph presence.
        if (amcl.pid > 0 && yes(evidence, "LIFECYCLE_VERIFIED")) lifecycle = true;
        if (get(evidence, "AMCL_START_RESULT") == "stopped" || get(evidence, "AMCL_START_RESULT") == "failed") lifecycle = false;
        commit(true);
        }
      } catch (...) {
        static_cast<RuntimeState &>(*this) = std::move(before);
        throw;
      }
    } else if (op == "read") { commit(); }
    rapidjson::Document out; out.SetObject(); auto &a = out.GetAllocator();
    json::put(out, "ok", true, a); json::put(out, "generation", generation, a);
    json::put(out, "registered", has_evidence, a);
    json::put(out, "process_observation_unknown", process_observation_unknown, a);
    json::put(out, "observer_error", observer_error, a);
    if (op == "read") json::put_json(out, "fields", fields_json(published, a), a);
    return json::serialize(out);
  }
  void serve() {
    const double deadline = json::monotonic_now() + 0.004;
    epoll_event events[32];
    epoll_wait(poller.value, events, 32, 0);
    // Slow clients never block the guard; at most 16 pending sockets for 250 ms.
    for (int i = 0; i < 16; ++i) {
      const int fd = accept4(listener.value, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0) break;
      ucred cred{}; socklen_t len = sizeof(cred);
      if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &len) != 0 ||
          (cred.uid != 0 && cred.uid != geteuid())) { close(fd); continue; }
      if (pending.size() >= 16) { close(fd); continue; }
      epoll_event event{}; event.events = EPOLLIN | EPOLLRDHUP; event.data.fd = fd;
      if (epoll_ctl(poller.value, EPOLL_CTL_ADD, fd, &event) != 0) { close(fd); continue; }
      pending.push_back({fd, json::monotonic_now() + 0.25});
    }
    for (auto i = pending.begin(); i != pending.end() && json::monotonic_now() < deadline;) {
      char buffer[max_packet_bytes + 1]; const auto n = recv(i->fd, buffer, sizeof(buffer), MSG_TRUNC);
      if (n < 0 && (errno == EAGAIN || errno == EINTR) && json::monotonic_now() < i->deadline) { ++i; continue; }
      if (n > 0) {
        std::string reply;
        try { require(n <= static_cast<ssize_t>(max_packet_bytes), "AMCL request too large");
          reply = dispatch({buffer, static_cast<size_t>(n)});
        } catch (const std::exception &e) {
          rapidjson::Document out; out.SetObject(); auto &a = out.GetAllocator();
          json::put(out, "ok", false, a); json::put(out, "error", e.what(), a); reply = json::serialize(out);
        }
        send(i->fd, reply.data(), reply.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
      }
      close(i->fd); i = pending.erase(i);
    }
  }
};

StatusServer::StatusServer(const std::string &path, Options options) : impl_(new Impl(path, std::move(options))) {}
StatusServer::~StatusServer() = default;
int StatusServer::fd() const { return impl_->poller.value; }
void StatusServer::handle_requests() { impl_->serve(); }
void StatusServer::tick() { impl_->tick(); }
uint64_t StatusServer::generation() const { return impl_->generation; }
ProcessIdentity StatusServer::amcl_identity() const { return impl_->amcl; }
std::string StatusServer::node_name() const { return get(impl_->evidence, "NODE_NAME", "amcl"); }
std::string StatusServer::pose_topic() const { return get(impl_->evidence, "POSE_TOPIC", "/amcl_pose"); }
std::string StatusServer::scan_topic() const { return get(impl_->evidence, "SCAN_TOPIC", "/scan_amcl"); }
std::string StatusServer::admission_status_topic() const { return get(impl_->evidence, "ADMISSION_STATUS_TOPIC", "/amcl_scan_admission/status"); }
bool StatusServer::registered() const { return impl_->has_evidence; }
void StatusServer::observe_graph(uint64_t gen, const GraphObservation &o) {
  if (gen != generation()) return;
  impl_->graph = o; impl_->graph_seen = true; impl_->graph_at = json::monotonic_now();
}
void StatusServer::observe_lifecycle(uint64_t gen, const ProcessIdentity &identity, bool active) {
  if (gen != generation() || !same_process(identity, impl_->amcl) ||
      impl_->process_state(identity) != Impl::ProcessState::alive) return;
  impl_->lifecycle = active;
}
void StatusServer::observe_pose(uint64_t gen, double header, double ros_now) {
  if (gen != generation() || impl_->process_state(impl_->amcl) != Impl::ProcessState::alive ||
      !std::isfinite(header) || !std::isfinite(ros_now) ||
      header <= 0 || ros_now - header < -0.25) return;
  impl_->pose_received = json::monotonic_now(); impl_->pose_header = header; impl_->pose_ros = ros_now;
}
void StatusServer::observe_map_owner(uint64_t gen, const std::string &owner) {
  if (gen == generation() && text_safe(owner)) impl_->map_owner = owner;
}
}  // namespace robot_bringup::runtime_amcl
