#include "robot_floor_manager/runtime_map_context_writer.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>
#include <system_error>
#include <yaml-cpp/yaml.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace robot_floor_manager
{
namespace fs = std::filesystem;
namespace
{

std::atomic<std::uint64_t> temporary_sequence{0U};

bool safe_identifier(const std::string & value)
{
  if (
    value.empty() || value == "." || value.size() > 128U ||
    value.find("..") != std::string::npos ||
    value.find('/') != std::string::npos ||
    value.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(
    value.cbegin(), value.cend(),
    [](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == static_cast<unsigned char>('-') ||
             character == static_cast<unsigned char>('_') ||
             character == static_cast<unsigned char>('.');
    });
}

bool canonical_sha256(const std::string & value)
{
  constexpr char prefix[] = "sha256:";
  constexpr std::size_t prefix_size = sizeof(prefix) - 1U;
  if (value.size() != prefix_size + 64U ||
    value.compare(0U, prefix_size, prefix) != 0)
  {
    return false;
  }
  return std::all_of(
    value.cbegin() + static_cast<std::ptrdiff_t>(prefix_size),
    value.cend(),
    [](const unsigned char character) {
      return std::isdigit(character) != 0 ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
    });
}

std::string json_string(const std::string & value)
{
  std::ostringstream output;
  output << '"';
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
          output << "\\u"
                 << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character)
                 << std::dec << std::setfill(' ');
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

bool validate(
  const RuntimeMapContextRecord & record,
  std::string & error)
{
  if (
    record.state.empty() ||
    !safe_identifier(record.transaction_id) ||
    !safe_identifier(record.building_id) ||
    !safe_identifier(record.floor_id) ||
    !safe_identifier(record.map_id) ||
    record.asset_epoch == 0U ||
    !canonical_sha256(record.asset_digest) ||
    !std::isfinite(record.updated_at_sec) ||
    record.updated_at_sec < 0.0)
  {
    error =
      "runtime context requires state, exact safe identity, nonzero epoch, "
      "canonical sha256 digest, and finite timestamp";
    return false;
  }
  if (
    record.confirmed &&
    (record.localizer_generation == 0U ||
    record.explicit_relocalization_sequence == 0U))
  {
    error =
      "confirmed runtime context requires localizer generation and explicit "
      "relocalization sequence";
    return false;
  }
  if (record.startup_handoff) {
    if (record.confirmed || !safe_identifier(record.request_nonce) ||
      (record.state != "requested" && record.state != "committed" && record.state != "failed"))
    {
      error = "startup handoff requires a unique nonce and an unconfirmed request state";
      return false;
    }
    const fs::path root(record.asset_root);
    if (!root.is_absolute() || root.lexically_normal() != root) {
      error = "startup handoff requires an absolute immutable asset root";
      return false;
    }
    for (const auto & role : {record.nav_map_yaml, record.localizer_map_png,
        record.localizer_params_yaml, record.keepout_mask_yaml, record.speed_mask_yaml})
    {
      const fs::path path(role);
      const auto relative = path.lexically_relative(root);
      if (!path.is_absolute() || path.lexically_normal() != path || relative.empty() ||
        *relative.begin() == "..")
      {
        error = "startup asset paths must stay inside the verified immutable bundle";
        return false;
      }
    }
  }
  return true;
}

std::string serialize(const RuntimeMapContextRecord & record)
{
  std::ostringstream body;
  body << std::fixed << std::setprecision(6)
       << "{"
       << "\"schema\":" << json_string(record.startup_handoff ?
    "njrh.floor_startup_handoff.v1" : "njrh.runtime_map_context.v1") << ","
       << "\"state\":" << json_string(record.state) << ","
       << "\"startup_stage\":\"\","
       << "\"confirmed\":" << (record.confirmed ? "true" : "false") << ","
       << "\"message\":" << json_string(record.message) << ","
       << "\"transaction_id\":" << json_string(record.transaction_id) << ","
       << "\"map_id\":" << json_string(record.map_id) << ","
       << "\"display_name\":" << json_string(record.map_id) << ","
       << "\"building_id\":" << json_string(record.building_id) << ","
       << "\"floor_id\":" << json_string(record.floor_id) << ","
       << "\"asset_epoch\":" << record.asset_epoch << ","
       << "\"asset_digest\":" << json_string(record.asset_digest) << ","
       << "\"localizer_generation\":" << record.localizer_generation << ","
       << "\"explicit_relocalization_sequence\":"
       << record.explicit_relocalization_sequence << ","
       << "\"updated_at\":" << record.updated_at_sec;
  if (record.startup_handoff) {
    body << ",\"version\":1,\"request_nonce\":" << json_string(record.request_nonce)
         << ",\"explicit_sequence_baseline\":" << record.explicit_sequence_baseline
         << ",\"speed_filter_enabled\":" << (record.speed_filter_enabled ? "true" : "false")
         << ",\"asset_root\":" << json_string(record.asset_root)
         << ",\"nav_map_yaml\":" << json_string(record.nav_map_yaml)
         << ",\"localizer_map_png\":" << json_string(record.localizer_map_png)
         << ",\"localizer_params_yaml\":" << json_string(record.localizer_params_yaml)
         << ",\"keepout_mask_yaml\":" << json_string(record.keepout_mask_yaml)
         << ",\"speed_mask_yaml\":" << json_string(record.speed_mask_yaml);
  }
  body << "}\n";
  return body.str();
}

#ifndef _WIN32
bool write_all(const int descriptor, const std::string & value)
{
  std::size_t offset = 0U;
  while (offset < value.size()) {
    const auto count = ::write(
      descriptor, value.data() + offset, value.size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}
#endif

}  // namespace

std::optional<FloorStartupHandoffAck> read_floor_startup_handoff_ack(
  const fs::path & path, const RuntimeMapContextRecord & expected)
{
  try {
    if (!fs::is_regular_file(fs::symlink_status(path)) || fs::file_size(path) > 65536U) {
      return std::nullopt;
    }
    const auto root = YAML::LoadFile(path.string());
    if (!root.IsMap()) {return std::nullopt;}
    std::set<std::string> keys;
    for (const auto & field : root) {
      if (!field.first.IsScalar() || !field.second.IsScalar() ||
        !keys.insert(field.first.as<std::string>()).second)
      {
        return std::nullopt;
      }
    }
    if (root["schema"].as<std::string>() != "njrh.floor_startup_handoff_ack.v1" ||
      root["version"].as<unsigned int>() != 1U ||
      root["transaction_id"].as<std::string>() != expected.transaction_id ||
      root["request_nonce"].as<std::string>() != expected.request_nonce ||
      root["building_id"].as<std::string>() != expected.building_id ||
      root["floor_id"].as<std::string>() != expected.floor_id ||
      root["map_id"].as<std::string>() != expected.map_id ||
      root["asset_epoch"].as<std::uint64_t>() != expected.asset_epoch ||
      root["asset_digest"].as<std::string>() != expected.asset_digest)
    {
      return std::nullopt;
    }
    FloorStartupHandoffAck ack;
    ack.state = root["state"].as<std::string>();
    ack.failure = root["failure"] ? root["failure"].as<std::string>() : "";
    ack.detail = root["detail"] ? root["detail"].as<std::string>() : "";
    if (ack.state != "adopted" && ack.state != "runtime_ready" && ack.state != "failed") {
      return std::nullopt;
    }
    if (ack.state == "runtime_ready") {
      ack.explicit_relocalization_sequence = root["explicit_relocalization_sequence"].as<std::uint64_t>();
      ack.localizer_generation = root["localizer_generation"].as<std::uint64_t>();
      if (ack.explicit_relocalization_sequence <= expected.explicit_sequence_baseline ||
        ack.localizer_generation == 0U) {return std::nullopt;}
    }
    if (root["cleanup_completed"] || root["effects_settled"] || root["owner_available"]) {
      const auto strict_bool = [](const YAML::Node & value) -> std::optional<bool> {
          if (!value.IsScalar() || value.Tag() != "?") {return std::nullopt;}
          if (value.Scalar() == "true") {return true;}
          if (value.Scalar() == "false") {return false;}
          return std::nullopt;
        };
      ack.cleanup_completed = strict_bool(root["cleanup_completed"]);
      ack.effects_settled = strict_bool(root["effects_settled"]);
      ack.owner_available = strict_bool(root["owner_available"]);
      if (ack.state != "failed" || !ack.cleanup_completed.has_value() ||
        !ack.effects_settled.has_value() || !ack.owner_available.has_value())
      {return std::nullopt;}
    }
    return ack;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool AtomicRuntimeMapContextWriter::write(
  const fs::path & path,
  const RuntimeMapContextRecord & record,
  std::string & error) const noexcept
{
  error.clear();
  try {
    if (!validate(record, error)) {
      return false;
    }
    if (path.empty() || path.filename().empty() || path.filename() == ".") {
      error = "runtime context path requires a file name";
      return false;
    }
    const auto parent =
      path.parent_path().empty() ? fs::path{"."} : path.parent_path();
    fs::create_directories(parent);
    const auto leaf = path.filename().string();
    const auto sequence = temporary_sequence.fetch_add(1U);

#ifdef _WIN32
    const auto temporary =
      parent / ("." + leaf + ".tmp." + std::to_string(sequence));
    {
      std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
      if (!output) {
        error = "failed to open runtime context temporary file";
        return false;
      }
      output << serialize(record);
      output.flush();
      if (!output) {
        error = "failed to flush runtime context temporary file";
        std::error_code ignored;
        fs::remove(temporary, ignored);
        return false;
      }
    }
    if (!MoveFileExW(
        temporary.c_str(), path.c_str(),
        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    {
      error = "failed to atomically replace runtime context";
      std::error_code ignored;
      fs::remove(temporary, ignored);
      return false;
    }
#else
    const int directory_descriptor = ::open(
      parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory_descriptor < 0) {
      error = "failed to open runtime context directory: " +
        std::string(std::strerror(errno));
      return false;
    }
    const auto temporary_leaf =
      "." + leaf + ".tmp." + std::to_string(::getpid()) + "." +
      std::to_string(sequence);
    const int temporary_descriptor = ::openat(
      directory_descriptor, temporary_leaf.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
      S_IRUSR | S_IWUSR);
    if (temporary_descriptor < 0) {
      error = "failed to create runtime context temporary file: " +
        std::string(std::strerror(errno));
      ::close(directory_descriptor);
      return false;
    }

    const auto body = serialize(record);
    bool success = write_all(temporary_descriptor, body);
    if (success) {
      success = ::fsync(temporary_descriptor) == 0;
    }
    const int write_error = errno;
    ::close(temporary_descriptor);
    if (!success) {
      error = "failed to durably write runtime context: " +
        std::string(std::strerror(write_error));
      ::unlinkat(directory_descriptor, temporary_leaf.c_str(), 0);
      ::close(directory_descriptor);
      return false;
    }

    struct stat target_status {};
    errno = 0;
    const int target_result = ::fstatat(
      directory_descriptor, leaf.c_str(), &target_status,
      AT_SYMLINK_NOFOLLOW);
    if (target_result == 0 && !S_ISREG(target_status.st_mode)) {
        error = "runtime context target exists but is not a regular file";
        ::unlinkat(directory_descriptor, temporary_leaf.c_str(), 0);
        ::close(directory_descriptor);
        return false;
    }
    if (target_result != 0 && errno != ENOENT) {
      error = "failed to inspect runtime context target: " +
        std::string(std::strerror(errno));
      ::unlinkat(directory_descriptor, temporary_leaf.c_str(), 0);
      ::close(directory_descriptor);
      return false;
    }
    if (
      ::renameat(
        directory_descriptor, temporary_leaf.c_str(),
        directory_descriptor, leaf.c_str()) != 0)
    {
      error = "failed to atomically replace runtime context: " +
        std::string(std::strerror(errno));
      ::unlinkat(directory_descriptor, temporary_leaf.c_str(), 0);
      ::close(directory_descriptor);
      return false;
    }
    if (::fsync(directory_descriptor) != 0) {
      error = "runtime context renamed but directory sync failed: " +
        std::string(std::strerror(errno));
      ::close(directory_descriptor);
      return false;
    }
    ::close(directory_descriptor);
#endif
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  } catch (...) {
    error = "unknown runtime context write failure";
    return false;
  }
}

}  // namespace robot_floor_manager
