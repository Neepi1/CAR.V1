#pragma once

#include "robot_bringup/runtime_amcl_status.hpp"
#include "robot_bringup/runtime_health_json.hpp"
#include <map>
#include <set>
#include <cctype>

namespace robot_bringup::runtime_amcl {
namespace json = robot_bringup::health;
using Fields = std::map<std::string, std::string>;
constexpr size_t max_packet_bytes = 16384;
inline const std::set<std::string> &input_fields() {
  static const std::set<std::string> keys{
    "AMCL_MODE", "AMCL_START_RESULT", "AMCL_READY", "AMCL_DEGRADED", "AMCL_FAILURE_REASON",
    "AMCL_STARTUP_EPOCH_SEC", "AMCL_PID_STALE_CLEARED", "SCAN_ADMISSION_PID_STALE_CLEARED",
    "SCAN_ADMISSION_IMPL", "AMCL_SEED_SUCCEEDED", "AMCL_SEED_RESPONSE_OK", "AMCL_NOMOTION_PROBE_USED",
    "AMCL_NOMOTION_POSE_RECEIVED", "AMCL_NOMOTION_POSE_COUNT", "AMCL_NOMOTION_POSE_HEADER_AGE_MS",
    "AMCL_STATIC_STANDBY_ACCEPTED", "LIFECYCLE_VERIFIED", "SCAN_ADMISSION_ENABLED",
    "NODE_NAME", "POSE_TOPIC", "SCAN_TOPIC", "ADMISSION_STATUS_TOPIC", "PROGRESS_KEY"};
  return keys;
}
inline bool text_safe(const std::string &s) {
  if (s.size() > 2048) return false;
  for (unsigned char c : s) if (c < 32 || c == 127) return false;
  return true;
}
inline json::Json identity_json(const ProcessIdentity &p, json::Allocator &a) {
  json::Json j(rapidjson::kObjectType);
  json::put(j, "pid", p.pid, a); json::put(j, "start_ticks", p.start_ticks, a);
  json::put(j, "executable", p.executable, a); json::put(j, "argument", p.argument, a);
  return j;
}
inline ProcessIdentity identity_from_json(const json::Json &j) {
  const auto &pid = json::field(j, "pid"), &ticks = json::field(j, "start_ticks");
  const auto &exe = json::field(j, "executable"), &arg = json::field(j, "argument");
  if (!j.IsObject() || !pid.IsInt() || pid.GetInt() < 0 || !ticks.IsUint64() ||
      !exe.IsString() || !arg.IsString()) throw std::runtime_error("invalid process identity");
  ProcessIdentity p{pid.GetInt(), ticks.GetUint64(), exe.GetString(), arg.GetString()};
  if (!text_safe(p.executable) || !text_safe(p.argument)) throw std::runtime_error("invalid identity text");
  if ((p.pid > 0 && (!p.start_ticks || p.executable.empty())) ||
      (p.pid == 0 && (p.start_ticks || !p.executable.empty())))
    throw std::runtime_error("incomplete process identity");
  return p;
}
inline std::string get(const Fields &f, const std::string &key, const std::string &fallback = {}) {
  const auto it = f.find(key); return it == f.end() ? fallback : it->second;
}
inline bool yes(const Fields &f, const std::string &key) { return get(f, key) == "true"; }
inline Fields parse_fields(const json::Json &j, bool enforce_input = true) {
  if (!j.IsObject()) throw std::runtime_error("fields must be an object");
  Fields result;
  for (auto it = j.MemberBegin(); it != j.MemberEnd(); ++it) {
    const std::string key(it->name.GetString(), it->name.GetStringLength());
    if (!it->value.IsString() || (enforce_input && !input_fields().count(key)))
      throw std::runtime_error("invalid evidence field: " + key);
    const std::string value(it->value.GetString(), it->value.GetStringLength());
    if (!text_safe(value) || !result.emplace(key, value).second)
      throw std::runtime_error("duplicate or invalid evidence field");
  }
  return result;
}
inline json::Json fields_json(const Fields &fields, json::Allocator &a) {
  json::Json j(rapidjson::kObjectType);
  for (const auto &[key, value] : fields) json::put(j, key.c_str(), value, a);
  return j;
}
inline std::string boot_id(const std::string &root = "/proc") {
  auto s = json::read_text(root + "/sys/kernel/random/boot_id");
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}
}  // namespace robot_bringup::runtime_amcl
