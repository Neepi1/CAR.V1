#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace robot_bringup::runtime_amcl {

struct ProcessIdentity {
  int pid{0};
  uint64_t start_ticks{0};
  std::string executable;
  std::string argument;
};

// Exact /proc identity, including zombie rejection and an optional exact argv item.
// Empty means confirmed absent/mismatched; unreadable or malformed /proc throws.
ProcessIdentity process_identity(int pid, const std::string &expected_executable = {},
  const std::string &argument = {}, const std::string &proc_root = "/proc");
bool same_process(const ProcessIdentity &a, const ProcessIdentity &b);

struct Options {
  std::string proc_root{"/proc"};
  double refresh_sec{2.0};
  double ttl_sec{5.0};
  double startup_grace_sec{45.0};
  double correction_pose_max_age_sec{1.0};
};

struct GraphObservation {
  bool available{false};
  bool node_exists{false};
  int pose_publishers{0};
  int scan_publishers{0};
  int admission_status_publishers{0};
};

// Embed in the EXISTING guard. This class does not create a thread, ROS node,
// executor, participant, or child process. No evidence is read from old env files.
class StatusServer {
public:
  explicit StatusServer(const std::string &status_path, Options options = {});
  ~StatusServer();
  StatusServer(const StatusServer &) = delete;
  StatusServer &operator=(const StatusServer &) = delete;

  int fd() const;  // poll(POLLIN) between the guard's fixed sampling deadlines
  void handle_requests();  // bounded nonblocking batch; ACK follows atomic commit
  void tick();  // call on the guard's 1 Hz sample; public refresh defaults to 2 s
  // Unknown process observations preserve evidence and suspend public refresh
  // until recovery (TTL expires normally). IPC exposes process_observation_unknown.
  uint64_t generation() const;  // capture when initiating any asynchronous query
  ProcessIdentity amcl_identity() const;
  std::string node_name() const;
  std::string pose_topic() const;
  std::string scan_topic() const;
  std::string admission_status_topic() const;
  bool registered() const;

  // Hooks use the guard's existing graph/client/subscription infrastructure.
  // Stale asynchronous results from an earlier registration are ignored.
  void observe_graph(uint64_t generation, const GraphObservation &observation);
  void observe_lifecycle(uint64_t generation, const ProcessIdentity &identity, bool active);
  void observe_pose(uint64_t generation, double header_stamp_sec, double ros_now_sec);
  void observe_map_owner(uint64_t generation, const std::string &owner);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::string socket_path(const std::string &status_path);
// One SOCK_SEQPACKET exchange; timeout covers connect, send, and ACK together.
std::string request(const std::string &status_path, const std::string &payload,
  int timeout_ms = 1000);

}  // namespace robot_bringup::runtime_amcl
