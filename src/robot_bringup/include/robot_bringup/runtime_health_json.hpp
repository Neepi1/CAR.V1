#pragma once

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace robot_bringup::health {
using Json = rapidjson::Value;
using Allocator = rapidjson::Document::AllocatorType;
inline double monotonic_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double wall_now() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
inline std::string read_text(const std::string &path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot read " + path);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}
inline const Json &field(const Json &v, const char *name) {
  static const Json missing;
  if (!v.IsObject()) return missing;
  const auto it = v.FindMember(name);
  return it == v.MemberEnd() ? missing : it->value;
}
inline double number(const Json &v, double fallback = NAN) {
  return v.IsNumber() && std::isfinite(v.GetDouble()) ? v.GetDouble() : fallback;
}
inline bool boolean(const Json &v) { return v.IsBool() && v.GetBool(); }
inline std::string string(const Json &v) { return v.IsString() ? v.GetString() : ""; }
inline Json value(const std::string &s, Allocator &a) { return Json(s.c_str(), s.size(), a); }
inline Json value(const char *s, Allocator &a) { return Json(s, a); }
inline Json value(bool b, Allocator &) { return Json(b); }
inline Json value(double n, Allocator &) { return std::isfinite(n) ? Json(n) : Json(); }
inline Json value(int n, Allocator &) { return Json(n); }
inline Json value(uint64_t n, Allocator &) { return Json(n); }
template<class T>
void put(Json &v, const char *key, T data, Allocator &a) {
  v.AddMember(Json(key, a), value(data, a), a);
}
inline void put_json(Json &v, const char *key, Json data, Allocator &a) {
  v.AddMember(Json(key, a), data, a);
}
inline std::string serialize(const Json &v) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  v.Accept(writer);
  return {buffer.GetString(), buffer.GetSize()};
}
}  // namespace robot_bringup::health
