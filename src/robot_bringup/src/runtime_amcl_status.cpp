#include "robot_bringup/runtime_amcl_protocol.hpp"
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <unistd.h>

namespace amcl = robot_bringup::runtime_amcl;
namespace json = robot_bringup::health;
namespace {
std::string env(const char *key, const std::string &fallback = {}) {
  const char *s = std::getenv(key); return s && *s ? std::string(s) : fallback;
}
int integer(const std::string &s) {
  if (s.empty() || s.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("positive integer required");
  size_t end = 0; int value = std::stoi(s, &end);
  if (end != s.size() || value <= 0) throw std::runtime_error("positive integer required");
  return value;
}
amcl::ProcessIdentity from_pid_file(const std::string &file, const std::string &exe, const std::string &arg) {
  if (file.empty()) return {};
  if (!std::filesystem::exists(file)) return {};
  std::ifstream in(file); std::string s, extra;
  if (!(in >> s) || (in >> extra)) throw std::runtime_error("unreadable or invalid PID file");
  if (exe.empty()) throw std::runtime_error("expected executable is required with a PID file");
  return amcl::process_identity(integer(s), exe, arg);
}
}  // namespace

int main(int argc, char **argv) {
  try {
    std::string path = env("NJRH_AMCL_RUNTIME_STATUS_FILE", "/tmp/njrh_amcl_runtime_status.env"), op;
    int timeout_ms = integer(env("NJRH_AMCL_STATUS_REQUEST_TIMEOUT_MS", "1000"));
    std::string owner_pid = env("NJRH_AMCL_STATUS_OWNER_PID", env("NJRH_STARTUP_OWNER_PID"));
    std::string owner_generation = env("NJRH_AMCL_STATUS_OWNER_GENERATION");
    std::string map_generation, amcl_pid_file, relay_pid_file, amcl_exe, relay_exe, amcl_arg, relay_arg;
    amcl::Fields fields;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "ping" || key == "read" || key == "register" || key == "submit") {
        if (!op.empty()) throw std::runtime_error("only one operation is allowed"); op = key; continue;
      }
      if (key == "--help") {
        std::cout << "runtime_amcl_status [--status-file PATH] [--timeout-ms 1..5000] ping|read|register|submit\n"
          "submit/register: --owner-pid PID --owner-generation TOKEN --map-generation TOKEN\n"
          " --amcl-pid-file FILE --amcl-exe PATH [--amcl-argument EXACT_ARG]\n"
          " --relay-pid-file FILE --relay-exe PATH [--relay-argument EXACT_ARG] --set KEY=VALUE ...\n";
        return 0;
      }
      if (i + 1 >= argc) throw std::runtime_error("missing option value");
      const std::string value = argv[++i];
      if (key == "--status-file") path = value;
      else if (key == "--timeout-ms") timeout_ms = integer(value);
      else if (key == "--owner-pid") owner_pid = value;
      else if (key == "--owner-generation") owner_generation = value;
      else if (key == "--map-generation") map_generation = value;
      else if (key == "--amcl-pid-file") amcl_pid_file = value;
      else if (key == "--relay-pid-file") relay_pid_file = value;
      else if (key == "--amcl-exe") amcl_exe = value;
      else if (key == "--relay-exe") relay_exe = value;
      else if (key == "--amcl-argument") amcl_arg = value;
      else if (key == "--relay-argument") relay_arg = value;
      else if (key == "--set") {
        const auto at = value.find('=');
        if (at == std::string::npos || !amcl::input_fields().count(value.substr(0, at)) ||
            !amcl::text_safe(value.substr(at + 1)) || !fields.emplace(value.substr(0, at), value.substr(at + 1)).second)
          throw std::runtime_error("invalid or duplicate evidence field");
      } else throw std::runtime_error("unknown option: " + key);
    }
    if (op.empty()) throw std::runtime_error("operation is required");
    rapidjson::Document doc; doc.SetObject(); auto &a = doc.GetAllocator();
    json::put(doc, "schema", "njrh.amcl.evidence.v1", a); json::put(doc, "operation", op, a);
    if (op == "submit" || op == "register") {
      const auto owner = amcl::process_identity(integer(owner_pid));
      if (!owner.pid || owner_generation.empty() || map_generation.empty())
        throw std::runtime_error("live owner PID and explicit owner/map generations are required");
      json::put(doc, "boot_id", amcl::boot_id(), a);
      json::put_json(doc, "owner", amcl::identity_json(owner, a), a);
      json::put(doc, "owner_generation", owner_generation, a); json::put(doc, "map_generation", map_generation, a);
      json::put_json(doc, "amcl", amcl::identity_json(from_pid_file(amcl_pid_file, amcl_exe, amcl_arg), a), a);
      json::put_json(doc, "relay", amcl::identity_json(from_pid_file(relay_pid_file, relay_exe, relay_arg), a), a);
      json::put_json(doc, "fields", amcl::fields_json(fields, a), a);
    }
    const auto reply = amcl::request(path, json::serialize(doc), timeout_ms);
    rapidjson::Document response; response.Parse(reply.data(), reply.size());
    if (response.HasParseError() || !response.IsObject() || !json::boolean(json::field(response, "ok")))
      throw std::runtime_error("observer rejected request: " + json::string(json::field(response, "error")));
    if (op == "read") {
      if (json::boolean(json::field(response, "process_observation_unknown")))
        throw std::runtime_error("AMCL_PROCESS_OBSERVATION_UNKNOWN");
      // Output is data, not shell syntax. Callers use read/printf -v, never eval/source.
      for (const auto &[key, value] : amcl::parse_fields(json::field(response, "fields"), false))
        std::cout << key << '=' << value << '\n';
    } else std::cout << reply << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[runtime-amcl-status] " << e.what() << '\n';
    return 1;
  }
}
