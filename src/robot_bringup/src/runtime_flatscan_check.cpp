#include "robot_bringup/runtime_flatscan_snapshot.hpp"
#include <iostream>

int main(int argc, char **argv) {
  using namespace robot_bringup::flatscan;
  Decision result;
  try {
    if (argc != 4) throw std::invalid_argument("usage: runtime_flatscan_check FILE MAX_AGE STREAM_TIMEOUT");
    const double max_age = std::stod(argv[2]), stream_timeout = std::stod(argv[3]);
    if (!std::isfinite(max_age) || max_age <= 0 || !std::isfinite(stream_timeout) || stream_timeout <= 0)
      throw std::invalid_argument("timeouts must be finite and positive");
    std::ifstream in(argv[1], std::ios::binary);
    if (!in) throw std::runtime_error("snapshot unavailable");
    std::string text(128 * 1024 + 1, '\0');
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    text.resize(static_cast<size_t>(in.gcount()));
    if (text.size() > 128 * 1024) throw std::runtime_error("snapshot too large");
    rapidjson::Document root;
    root.Parse(text.c_str());
    if (!root.HasParseError()) result = evaluate(root, monotonic_now(), wall_now(),
      read_text("/proc/sys/kernel/random/boot_id"), max_age, stream_timeout);
  } catch (const std::exception &e) {
    std::cerr << "runtime_flatscan_check: " << e.what() << '\n';
  }
  std::cout << "status=" << result.status << " source_id=" << result.source_id
            << " evidence_kind=" << result.evidence_kind
            << " evidence_sequence=" << result.evidence_sequence << '\n';
  return result.code;
}
