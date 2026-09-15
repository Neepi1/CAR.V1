#include "robot_bringup/runtime_amcl_protocol.hpp"

#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace a = robot_bringup::runtime_amcl;
namespace j = robot_bringup::health;
namespace fs = std::filesystem;
namespace {
void check(bool ok, const std::string &message) { if (!ok) throw std::runtime_error(message); }
void write_file(const fs::path &path, const std::string &s) {
  fs::create_directories(path.parent_path()); std::ofstream out(path, std::ios::binary); out << s;
}
struct Fixture {
  std::string root, proc, status;
  a::Options options;
  std::unique_ptr<a::StatusServer> server;
  std::string owner_generation{"owner-a"}, map_generation{"map-a"};
  bool observation_unknown{false};
  Fixture() {
    char pattern[] = "/tmp/amcl-status-test-XXXXXX";
    char *p = mkdtemp(pattern); check(p, "mkdtemp"); root = p; proc = root + "/proc"; status = root + "/status.env";
    write_file(proc + "/sys/kernel/random/boot_id", "test-boot\n");
    process(100, 1000); process(200, 2000); process(300, 3000);
    options.proc_root = proc; options.refresh_sec = 0.01; options.ttl_sec = 5.0;
    options.startup_grace_sec = 0.02;
    server = std::make_unique<a::StatusServer>(status, options);
  }
  ~Fixture() { server.reset(); fs::remove_all(root); }
  void process(int pid, uint64_t start, char state = 'S', const std::string &args = {}) {
    const auto path = proc + "/" + std::to_string(pid);
    std::string stat = std::to_string(pid) + " (name with ) spaces) " + state;
    for (int i = 1; i < 19; ++i) stat += " 0";
    stat += " " + std::to_string(start) + " 0 0\n";
    write_file(path + "/stat", stat); write_file(path + "/cmdline", args);
    std::error_code ec; fs::remove(path + "/exe", ec);
    fs::create_symlink(fs::canonical("/bin/sleep"), path + "/exe");
  }
  a::ProcessIdentity id(int pid) { return a::process_identity(pid, {}, {}, proc); }
  a::Fields fields(const std::string &result = "ready", bool lifecycle = true, bool seed = true) {
    return {{"AMCL_MODE", "gated"}, {"AMCL_START_RESULT", result}, {"AMCL_READY", result == "ready" ? "true" : "false"},
      {"AMCL_DEGRADED", "false"}, {"LIFECYCLE_VERIFIED", lifecycle ? "true" : "false"},
      {"AMCL_SEED_SUCCEEDED", seed ? "true" : "false"}, {"AMCL_SEED_RESPONSE_OK", "false"},
      {"SCAN_ADMISSION_ENABLED", "true"}, {"SCAN_ADMISSION_IMPL", "cpp"},
      {"AMCL_STATIC_STANDBY_ACCEPTED", "false"}, {"PROGRESS_KEY", std::string(64, 'a')}};
  }
  std::string payload(const a::Fields &f, const std::string &op = "submit", int amcl_pid = 200) {
    rapidjson::Document d; d.SetObject(); auto &alloc = d.GetAllocator();
    j::put(d, "schema", "njrh.amcl.evidence.v1", alloc); j::put(d, "operation", op, alloc);
    j::put(d, "boot_id", "test-boot", alloc); j::put(d, "owner_generation", owner_generation, alloc);
    j::put(d, "map_generation", map_generation, alloc);
    j::put_json(d, "owner", a::identity_json(id(100), alloc), alloc);
    j::put_json(d, "amcl", a::identity_json(id(amcl_pid), alloc), alloc);
    j::put_json(d, "relay", a::identity_json(id(300), alloc), alloc);
    j::put_json(d, "fields", a::fields_json(f, alloc), alloc);
    return j::serialize(d);
  }
  bool exchange(const std::string &s) {
    auto future = std::async(std::launch::async, [&] { return a::request(status, s, 500); });
    const double end = j::monotonic_now() + 1;
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      pollfd p{server->fd(), POLLIN, 0}; poll(&p, 1, 10); server->handle_requests();
      check(j::monotonic_now() < end, "exchange exceeded bounded deadline");
    }
    const auto reply = future.get(); rapidjson::Document d; d.Parse(reply.c_str());
    check(!d.HasParseError(), "malformed response");
    observation_unknown = j::boolean(j::field(d, "process_observation_unknown"));
    return j::boolean(j::field(d, "ok"));
  }
  std::string value(const std::string &key) {
    const auto text = j::read_text(status); const std::string prefix = key + "=\"";
    const auto start = text.find(prefix); check(start != std::string::npos, "missing public field " + key);
    const auto end = text.find("\"\n", start + prefix.size());
    return text.substr(start + prefix.size(), end - start - prefix.size());
  }
  void tick() { std::this_thread::sleep_for(std::chrono::milliseconds(12)); server->tick(); }
};
void identity_test() {
  Fixture f;
  check(f.id(200).start_ticks == 2000, "start tick parsing with spaces/parens");
  f.process(200, 2001, 'Z'); check(f.id(200).pid == 0, "zombie must not be alive");
  f.process(200, 2002, 'S', std::string("sleep\0__node:=amcl\0", 19));
  check(a::process_identity(200, "/bin/sleep", "__node:=amcl", f.proc).pid == 200, "exact argv match");
  check(a::process_identity(200, "/bin/sleep", "amcl", f.proc).pid == 0, "substring is not argv identity");
}
void receipts_test() {
  Fixture f;
  check(f.exchange(f.payload(f.fields("ready", false, false))), "submit without lifecycle receipt");
  check(f.value("AMCL_LIFECYCLE_ACTIVE") == "false", "alive must not imply lifecycle active");
  check(f.value("AMCL_SEEDED") == "false", "ready must not imply seed");
  check(f.value("AMCL_NODE_EXISTS") == "false", "alive must not imply graph presence");
  auto fields = f.fields("ready", true, false);
  fields["AMCL_STATIC_STANDBY_ACCEPTED"] = "true";
  check(f.exchange(f.payload(fields)), "submit static without seed");
  check(f.value("AMCL_READY") == "false" && f.value("AMCL_SEEDED") == "false", "standby is not seed proof");
  fields["AMCL_SEED_RESPONSE_OK"] = "true";
  check(f.exchange(f.payload(fields)), "actual seed reply accepted");
  check(f.value("AMCL_TRACKING_READY") == "true" && f.value("AMCL_CORRECTION_READY") == "false", "static standby contract");
  f.server->observe_pose(f.server->generation(), 100, 100); f.tick();
  check(f.value("AMCL_CORRECTION_READY") == "false", "pose metadata must not change static policy");
  f.server->observe_lifecycle(f.server->generation(), f.id(200), false); f.tick();
  check(f.value("AMCL_LIFECYCLE_ACTIVE") == "false" && f.value("AMCL_READY") == "false", "inactive lifecycle event");
}
void identity_and_generation_test() {
  Fixture f;
  check(f.exchange(f.payload(f.fields())), "initial ready receipt");
  const auto old = f.payload(f.fields());
  const auto old_gen = f.server->generation();
  f.process(200, 2001); f.tick();
  check(f.value("AMCL_SEEDED") == "false", "PID reuse clears seed");
  check(!f.exchange(old), "old PID-start receipt rejected");
  check(f.exchange(f.payload(f.fields("waiting_seed", true, false))), "new PID receipt");
  check(f.value("AMCL_READY") == "false", "new process cannot inherit seed");
  f.server->observe_lifecycle(old_gen, f.id(200), true);
  f.server->observe_graph(old_gen, {true, true, 9, 9, 9}); f.tick();
  check(f.value("AMCL_POSE_PUBLISHER_COUNT") == "0", "old async observation ignored");
  f.map_generation = "map-b";
  check(!f.exchange(f.payload(f.fields())), "different map needs explicit registration");
  check(f.exchange(f.payload(f.fields("starting", false, false), "register", 0)), "map registration");
  check(f.value("AMCL_SEEDED") == "false", "map registration clears seed");
  f.map_generation = "map-a";
  check(!f.exchange(f.payload(f.fields())), "old map worker cannot submit");
}
void status_contract_test() {
  Fixture f;
  check(f.exchange(f.payload(f.fields("waiting_seed", true, false))), "waiting seed receipt");
  f.tick(); check(f.value("AMCL_START_RESULT") == "waiting_seed", "heartbeat cannot manufacture seed");
  auto fields = f.fields(); fields["AMCL_MODE"] = "shadow";
  check(f.exchange(f.payload(fields)), "shadow receipt");
  check(f.value("AMCL_READY") == "true" && f.value("AMCL_MODE") == "shadow", "shadow readiness unchanged");
  f.server->observe_graph(f.server->generation(), {true, true, 1, 2, 3}); f.tick();
  check(f.value("SCAN_AMCL_PUBLISHER_COUNT") == "2", "real graph count");
  f.server->observe_graph(f.server->generation(), {}); f.tick();
  check(f.value("AMCL_POSE_PUBLISHER_COUNT") == "0", "unavailable graph is not synthesized");
  fields = f.fields("failed", false, false); fields["AMCL_FAILURE_REASON"] = "seed failed";
  check(f.exchange(f.payload(fields)), "failure receipt"); f.tick();
  check(f.value("AMCL_START_RESULT") == "failed", "heartbeat cannot erase failure");
  fields = f.fields("disabled", false, false); fields["AMCL_MODE"] = "disabled";
  check(f.exchange(f.payload(fields)), "disabled receipt");
  check(f.value("AMCL_READY") == "true" && f.value("AMCL_TRACKING_READY") == "false", "disabled contract");
  fields["AMCL_START_RESULT"] = "stopped";
  check(f.exchange(f.payload(fields)), "stop disabled instance");
  check(f.value("AMCL_READY") == "false", "explicit stop must not become disabled-ready");
  f.process(100, 1000, 'Z'); f.tick();
  check(f.value("AMCL_READY") == "false" && f.value("AMCL_START_RESULT") == "stopped", "owner exit is a normal stopped state");
  f.process(100, 1001); f.owner_generation = "owner-b";
  check(f.exchange(f.payload(f.fields("waiting_seed", true, false))), "new live owner accepted");
  check(f.value("AMCL_SEEDED") == "false", "new owner cannot inherit old seed");
}
void isolation_test() {
  Fixture f;
  check(f.exchange(f.payload(f.fields())), "initial receipt");
  struct stat permissions{};
  check(stat(f.status.c_str(), &permissions) == 0 && (permissions.st_mode & 0777) == 0644,
    "public status must retain read-only consumer access");
  check(stat((f.status + ".control").c_str(), &permissions) == 0 && (permissions.st_mode & 0777) == 0700,
    "control directory must be private");
  check(chmod(f.status.c_str(), 0640) == 0, "fixture public permissions");
  check(f.exchange(f.payload(f.fields())), "permission-preserving commit");
  check(stat(f.status.c_str(), &permissions) == 0 && (permissions.st_mode & 0777) == 0640,
    "existing read permissions must be preserved");
  const auto before = j::read_text(f.status);
  check(!f.exchange("{not_json}"), "malformed request rejected");
  auto bad = f.fields(); bad["LIFECYCLE_VERIFIED"] = "maybe";
  check(!f.exchange(f.payload(bad)), "invalid boolean rejected");
  bad = f.fields(); bad["AMCL_TRACKING_READY"] = "true";
  check(!f.exchange(f.payload(bad)), "derived tracking field cannot be injected");
  bad = f.fields(); bad["AMCL_FAILURE_REASON"] = "bad\nEVIL=1";
  check(!f.exchange(f.payload(bad)), "newline shell payload rejected");
  bad = f.fields(); bad["PROGRESS_KEY"] = std::string(64, 'b');
  check(!f.exchange(f.payload(bad)), "different startup/map proof cannot replace current incarnation");
  auto old_boot = f.payload(f.fields());
  old_boot.replace(old_boot.find("test-boot"), 9, "last-boot");
  check(!f.exchange(old_boot), "old boot receipt rejected");
  check(j::read_text(f.status) == before, "rejected requests cannot modify public status");
  bool locked = false;
  try { a::StatusServer second(f.status, f.options); } catch (...) { locked = true; }
  check(locked, "single public writer lock");
  check(f.exchange("{\"schema\":\"njrh.amcl.evidence.v1\",\"operation\":\"ping\"}"), "second-writer rejection preserves socket");
  f.server.reset();
  f.server = std::make_unique<a::StatusServer>(f.status, f.options);
  f.tick(); check(!f.server->registered(), "guard restart must not source old ready/seed from env");
  check(j::read_text(f.status) == before, "old public status must age instead of replaying seed");
}
void timeout_test() {
  Fixture f;
  const double start = j::monotonic_now(); bool timed_out = false;
  try { a::request(f.status, "{}", 40); } catch (...) { timed_out = true; }
  check(timed_out && j::monotonic_now() - start < 0.25, "unserviced socket must time out");
  f.server->handle_requests();
  check(f.exchange(f.payload(f.fields())), "timed out peer must not poison subsequent requests");
}
void process_observation_recovery_test() {
  Fixture f;
  check(f.exchange(f.payload(f.fields())), "ready before proc read failures");
  const auto identity = f.id(200);
  const auto generation = f.server->generation();
  const auto request = f.payload(f.fields());
  for (int pid : {100, 200, 300}) {
    const auto stat = f.proc + "/" + std::to_string(pid) + "/stat";
    const auto original = j::read_text(stat);
    for (int fault = 0; fault < 4; ++fault) {
      const auto before = j::read_text(f.status);
      fs::remove(stat);
      if (fault == 0) write_file(stat, "truncated process stat");
      if (fault == 1) fs::create_directory(stat);  // read fails even under root
      if (fault == 2) fs::create_symlink("/proc/self/mem", stat); // EIO/EACCES, never gone
      // fault 3: missing stat while the process directory still exists
      bool unknown = false;
      try { f.id(pid); } catch (...) { unknown = true; }
      check(unknown, "proc observation failure must not return a gone identity");
      f.tick();
      f.server->observe_pose(generation, 100, 100);
      f.server->observe_lifecycle(generation, identity, false);
      // Only an AMCL identity failure suppresses its lifecycle event.
      if (pid != 200) f.server->observe_lifecycle(generation, identity, true);
      check(f.server->generation() == generation, "unknown must not invalidate generation");
      check(j::read_text(f.status) == before, "unknown must not refresh ready or publish a death");
      check(f.exchange("{\"schema\":\"njrh.amcl.evidence.v1\",\"operation\":\"read\"}"), "unknown diagnostic read");
      check(f.observation_unknown, "unknown process observation must be explicit in IPC");
      check(!f.exchange(request), "unknown identity cannot acknowledge new evidence");
      fs::remove(stat); write_file(stat, original);
      f.tick();
      check(f.server->generation() == generation, "recovery preserves incarnation");
      check(f.value("AMCL_SEEDED") == "true" && f.value("AMCL_READY") == "true",
        "read recovery restores refresh without replay or reseed");
    }
  }
  fs::remove_all(f.proc + "/200"); f.tick();
  check(f.value("AMCL_SEEDED") == "false" && f.value("AMCL_READY") == "false",
    "confirmed process disappearance still invalidates evidence");
}
void writer_transaction_test() {
  Fixture f;
  fs::create_directory(f.status);
  check(!f.exchange(f.payload(f.fields())), "failed initial public write must reject receipt");
  check(!f.server->registered() && f.server->generation() == 0, "failed initial registration rolled back");
  fs::remove(f.status); f.tick();
  check(!fs::exists(f.status), "tick must not publish rejected initial evidence");
  check(f.exchange(f.payload(f.fields())), "registration after writer recovery");
  const auto generation = f.server->generation();
  const auto identity = f.server->amcl_identity();
  auto rejected = f.fields("starting", false, false);
  rejected["POSE_TOPIC"] = "/rejected_pose";
  f.map_generation = "map-b";
  const auto payload = f.payload(rejected, "register", 0);
  fs::rename(f.status, f.status + ".saved"); fs::create_directory(f.status);
  f.tick(); // A periodic writer error must not escape into the guard main loop.
  check(f.server->generation() == generation && a::same_process(f.server->amcl_identity(), identity),
    "periodic writer failure keeps accepted identity and generation");
  check(!f.exchange(payload), "failed replacement write must reject registration");
  check(f.server->generation() == generation && a::same_process(f.server->amcl_identity(), identity),
    "rejected registration must roll back generation and process identities");
  check(f.server->pose_topic() == "/amcl_pose", "rejected subscription metadata cannot leak");
  fs::remove(f.status); fs::rename(f.status + ".saved", f.status); f.tick();
  check(f.value("AMCL_STATUS_MAP_GENERATION") == "map-a" && f.value("AMCL_SEEDED") == "true" &&
    f.value("AMCL_READY") == "true", "tick must publish only previously acknowledged evidence");
  check(f.exchange(payload), "same registration can be retried after writer recovery");
  check(f.value("AMCL_STATUS_MAP_GENERATION") == "map-b" && f.value("AMCL_SEEDED") == "false",
    "successful retry commits new generation without inheriting seed");
}
}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--serve") {
      a::StatusServer server(argv[2]);
      const double deadline = j::monotonic_now() + 30.0;
      while (j::monotonic_now() < deadline) {
        pollfd p{server.fd(), POLLIN, 0}; poll(&p, 1, 50);
        server.handle_requests(); server.tick();
      }
      return 0;
    }
    identity_test(); receipts_test(); identity_and_generation_test(); status_contract_test(); isolation_test(); timeout_test();
    process_observation_recovery_test(); writer_transaction_test();
    std::cout << "runtime_amcl_status: 8 native fixture suites passed\n"; return 0;
  } catch (const std::exception &e) { std::cerr << "runtime_amcl_status: " << e.what() << '\n'; return 1; }
}
