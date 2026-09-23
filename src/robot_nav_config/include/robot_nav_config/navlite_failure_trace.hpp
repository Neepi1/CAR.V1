#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace robot_nav_config
{
// Diagnostic counters only. Never caches commands or authorizes control.
class NavliteFailureTrace
{
public:
  using Clock = std::chrono::steady_clock;
  const char * failure(const std::string & raw, Clock::time_point now = Clock::now())
  {
    const bool first = !failed_;
    const bool changed = raw != reason_;
    if (first) {
      ++episode_;
      count_ = 0;
      started_ = now;
    }
    failed_ = true;
    awaiting_nonzero_ = false;
    ++count_;
    reason_ = raw;
    elapsed_ = std::chrono::duration<double>(now - started_).count();
    if (!first && !changed && now - last_log_ < std::chrono::seconds(1)) {return nullptr;}
    last_log_ = now;
    return first ? "failure_start" : (changed ? "failure_reason_changed" : "failure_continues");
  }

  const char * success(bool nonzero, Clock::time_point now = Clock::now())
  {
    if (failed_) {
      failed_ = false;
      elapsed_ = std::chrono::duration<double>(now - started_).count();
      awaiting_nonzero_ = !nonzero;
      return nonzero ? "calculation_recovered_nonzero" : "calculation_recovered_zero";
    }
    if (awaiting_nonzero_ && nonzero) {
      awaiting_nonzero_ = false;
      elapsed_ = std::chrono::duration<double>(now - started_).count();
      return "first_nonzero_after_calculation_recovery";
    }
    return nullptr;
  }

  bool pending() const {return failed_ || awaiting_nonzero_;}
  void interrupt() {failed_ = false; awaiting_nonzero_ = false;}
  std::uint64_t episode() const {return episode_;}
  std::uint64_t failures() const {return count_;}
  double elapsed() const {return elapsed_;}
  const std::string & reason() const {return reason_;}
  std::int64_t started_ns() const
  {return std::chrono::duration_cast<std::chrono::nanoseconds>(started_.time_since_epoch()).count();}

private:
  bool failed_{false}, awaiting_nonzero_{false};
  std::uint64_t episode_{0}, count_{0};
  double elapsed_{0.0};
  std::string reason_;
  Clock::time_point started_{}, last_log_{};
};

// Keep the original bytes recoverable while preserving a single log line.
inline std::string navlite_quote(const std::string & value)
{
  std::string out = "\"";
  const char * hex = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {out += '\\'; out += c;}
    else if (c < 32) {
      out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15];
    } else {out += c;}
  }
  return out + "\"";
}
}  // namespace robot_nav_config
