#include "robot_api_server/elevator_configuration_module.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include <yaml-cpp/yaml.h>

#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/http_common.hpp"
#include "robot_api_server/map_asset_digest.hpp"
#include "robot_elevator_manager/elevator_topology.hpp"
#include "robot_elevator_manager/elevator_topology_loader.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;
using robot_elevator_manager::DoorThreshold;
using robot_elevator_manager::ElevatorTopology;
using robot_elevator_manager::FloorElevatorTopology;
using robot_elevator_manager::Point2;
using robot_elevator_manager::PoseBinding;
using robot_elevator_manager::PoseRole;

constexpr std::size_t kMaximumDocumentBytes = 2U * 1024U * 1024U;
constexpr std::size_t kMaximumElevators = 16U;
constexpr std::size_t kMaximumFloorsPerElevator = 64U;
constexpr std::size_t kMaximumFloorBindings = 128U;
constexpr std::size_t kMaximumJsonDepth = 64U;
constexpr double kPi = 3.14159265358979323846;

struct ValidationIssue
{
  std::string code;
  std::string field;
  std::string message;
};

struct PoseRecord
{
  PoseRole role{PoseRole::kHallCall};
  std::string pose_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct FloorRecord
{
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  std::vector<PoseRecord> poses;
  DoorThreshold threshold;
};

struct ElevatorRecord
{
  std::string elevator_id;
  std::string display_name;
  std::vector<FloorRecord> floors;
};

struct MapBinding
{
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
};

struct ValidationResult
{
  std::vector<ValidationIssue> issues;
  std::vector<ElevatorRecord> elevators;
  std::vector<MapBinding> bindings;
  std::string configuration_yaml;
  std::string topology_yaml;
  std::string internal_poses_yaml;

  bool ok() const noexcept
  {
    return issues.empty();
  }
};

struct DraftPointer
{
  bool exists{false};
  std::string revision;
};

struct ReleasePointer
{
  bool exists{false};
  std::string release_id;
  std::uint64_t generation{0U};
};

class StrictJsonParser
{
public:
  StrictJsonParser(
    const std::string_view document,
    std::string label)
  : document_(document), label_(std::move(label))
  {
  }

  void parse()
  {
    if (document_.empty()) {
      fail("document is empty");
    }
    if (document_.size() > kMaximumDocumentBytes) {
      fail("document exceeds the 2 MiB limit");
    }
    skip_whitespace();
    parse_value(0U);
    skip_whitespace();
    if (position_ != document_.size()) {
      fail("trailing content is not allowed");
    }
  }

private:
  [[noreturn]] void fail(const std::string & message) const
  {
    throw std::runtime_error(
            label_ + " is not strict JSON: " + message +
            " at byte " + std::to_string(position_));
  }

  static bool is_json_whitespace(const unsigned char character)
  {
    return character == ' ' || character == '\t' ||
           character == '\n' || character == '\r';
  }

  static int hex_value(const unsigned char character)
  {
    if (character >= '0' && character <= '9') {
      return character - '0';
    }
    if (character >= 'a' && character <= 'f') {
      return character - 'a' + 10;
    }
    if (character >= 'A' && character <= 'F') {
      return character - 'A' + 10;
    }
    return -1;
  }

  void skip_whitespace()
  {
    while (position_ < document_.size() &&
      is_json_whitespace(
        static_cast<unsigned char>(document_[position_])))
    {
      ++position_;
    }
  }

  void expect(const char expected)
  {
    if (position_ >= document_.size() ||
      document_[position_] != expected)
    {
      fail(std::string("expected '") + expected + "'");
    }
    ++position_;
  }

  void parse_value(const std::size_t depth)
  {
    if (position_ >= document_.size()) {
      fail("expected a value");
    }
    switch (document_[position_]) {
      case '{':
        parse_object(depth + 1U);
        return;
      case '[':
        parse_array(depth + 1U);
        return;
      case '"':
        (void)parse_string(false);
        return;
      case 't':
        parse_literal("true");
        return;
      case 'f':
        parse_literal("false");
        return;
      case 'n':
        parse_literal("null");
        return;
      default:
        if (document_[position_] == '-' ||
          (document_[position_] >= '0' && document_[position_] <= '9'))
        {
          parse_number();
          return;
        }
        fail("expected an object, array, string, number, boolean, or null");
    }
  }

  void parse_object(const std::size_t depth)
  {
    if (depth > kMaximumJsonDepth) {
      fail("nesting exceeds the maximum depth of 64");
    }
    expect('{');
    skip_whitespace();
    if (position_ < document_.size() && document_[position_] == '}') {
      ++position_;
      return;
    }

    std::set<std::string> keys;
    while (true) {
      if (position_ >= document_.size() || document_[position_] != '"') {
        fail("object keys must be JSON strings");
      }
      auto key = parse_string(true);
      if (!keys.insert(std::move(key)).second) {
        fail("duplicate decoded object key");
      }
      skip_whitespace();
      expect(':');
      skip_whitespace();
      parse_value(depth);
      skip_whitespace();
      if (position_ >= document_.size()) {
        fail("unterminated object");
      }
      if (document_[position_] == '}') {
        ++position_;
        return;
      }
      expect(',');
      skip_whitespace();
    }
  }

  void parse_array(const std::size_t depth)
  {
    if (depth > kMaximumJsonDepth) {
      fail("nesting exceeds the maximum depth of 64");
    }
    expect('[');
    skip_whitespace();
    if (position_ < document_.size() && document_[position_] == ']') {
      ++position_;
      return;
    }
    while (true) {
      parse_value(depth);
      skip_whitespace();
      if (position_ >= document_.size()) {
        fail("unterminated array");
      }
      if (document_[position_] == ']') {
        ++position_;
        return;
      }
      expect(',');
      skip_whitespace();
    }
  }

  std::uint16_t parse_hex_quad()
  {
    if (document_.size() - position_ < 4U) {
      fail("incomplete Unicode escape");
    }
    std::uint16_t value = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const auto digit =
        hex_value(static_cast<unsigned char>(document_[position_++]));
      if (digit < 0) {
        fail("invalid Unicode escape");
      }
      value = static_cast<std::uint16_t>(
        (value << 4U) | static_cast<std::uint16_t>(digit));
    }
    return value;
  }

  static void append_utf8(
    std::string & output,
    const std::uint32_t code_point)
  {
    if (code_point <= 0x7fU) {
      output.push_back(static_cast<char>(code_point));
    } else if (code_point <= 0x7ffU) {
      output.push_back(static_cast<char>(0xc0U | (code_point >> 6U)));
      output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
    } else if (code_point <= 0xffffU) {
      output.push_back(static_cast<char>(0xe0U | (code_point >> 12U)));
      output.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3fU)));
      output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
    } else {
      output.push_back(static_cast<char>(0xf0U | (code_point >> 18U)));
      output.push_back(static_cast<char>(0x80U | ((code_point >> 12U) & 0x3fU)));
      output.push_back(static_cast<char>(0x80U | ((code_point >> 6U) & 0x3fU)));
      output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
    }
  }

  std::size_t validate_utf8_sequence()
  {
    const auto first =
      static_cast<unsigned char>(document_[position_]);
    std::size_t length = 0U;
    std::uint32_t code_point = 0U;
    std::uint32_t minimum = 0U;
    if (first >= 0xc2U && first <= 0xdfU) {
      length = 2U;
      code_point = first & 0x1fU;
      minimum = 0x80U;
    } else if (first >= 0xe0U && first <= 0xefU) {
      length = 3U;
      code_point = first & 0x0fU;
      minimum = 0x800U;
    } else if (first >= 0xf0U && first <= 0xf4U) {
      length = 4U;
      code_point = first & 0x07U;
      minimum = 0x10000U;
    } else {
      fail("invalid UTF-8 leading byte in string");
    }
    if (document_.size() - position_ < length) {
      fail("incomplete UTF-8 sequence in string");
    }
    for (std::size_t index = 1U; index < length; ++index) {
      const auto continuation =
        static_cast<unsigned char>(document_[position_ + index]);
      if ((continuation & 0xc0U) != 0x80U) {
        fail("invalid UTF-8 continuation byte in string");
      }
      code_point = (code_point << 6U) | (continuation & 0x3fU);
    }
    if (code_point < minimum || code_point > 0x10ffffU ||
      (code_point >= 0xd800U && code_point <= 0xdfffU))
    {
      fail("non-canonical UTF-8 code point in string");
    }
    return length;
  }

  std::string parse_string(const bool materialize)
  {
    expect('"');
    std::string decoded;
    while (position_ < document_.size()) {
      const auto character =
        static_cast<unsigned char>(document_[position_]);
      if (character == '"') {
        ++position_;
        return decoded;
      }
      if (character < 0x20U) {
        fail("unescaped control character in string");
      }
      if (character == '\\') {
        ++position_;
        if (position_ >= document_.size()) {
          fail("unterminated escape sequence");
        }
        const char escape = document_[position_++];
        switch (escape) {
          case '"':
          case '\\':
          case '/':
            if (materialize) {
              decoded.push_back(escape);
            }
            break;
          case 'b':
            if (materialize) {
              decoded.push_back('\b');
            }
            break;
          case 'f':
            if (materialize) {
              decoded.push_back('\f');
            }
            break;
          case 'n':
            if (materialize) {
              decoded.push_back('\n');
            }
            break;
          case 'r':
            if (materialize) {
              decoded.push_back('\r');
            }
            break;
          case 't':
            if (materialize) {
              decoded.push_back('\t');
            }
            break;
          case 'u':
          {
            const auto high = parse_hex_quad();
            std::uint32_t code_point = high;
            if (high >= 0xd800U && high <= 0xdbffU) {
              if (document_.size() - position_ < 6U ||
                document_[position_] != '\\' ||
                document_[position_ + 1U] != 'u')
              {
                fail("high surrogate is not followed by a low surrogate");
              }
              position_ += 2U;
              const auto low = parse_hex_quad();
              if (low < 0xdc00U || low > 0xdfffU) {
                fail("high surrogate is not followed by a low surrogate");
              }
              code_point =
                0x10000U +
                ((static_cast<std::uint32_t>(high) - 0xd800U) << 10U) +
                (static_cast<std::uint32_t>(low) - 0xdc00U);
            } else if (high >= 0xdc00U && high <= 0xdfffU) {
              fail("unpaired low surrogate in Unicode escape");
            }
            if (materialize) {
              append_utf8(decoded, code_point);
            }
            break;
          }
          default:
            fail("invalid string escape");
        }
        continue;
      }
      if (character < 0x80U) {
        if (materialize) {
          decoded.push_back(static_cast<char>(character));
        }
        ++position_;
        continue;
      }
      const auto length = validate_utf8_sequence();
      if (materialize) {
        decoded.append(document_.substr(position_, length));
      }
      position_ += length;
    }
    fail("unterminated string");
  }

  void parse_literal(const std::string_view literal)
  {
    if (document_.substr(position_, literal.size()) != literal) {
      fail("invalid literal");
    }
    position_ += literal.size();
  }

  void parse_number()
  {
    if (document_[position_] == '-') {
      ++position_;
      if (position_ >= document_.size()) {
        fail("incomplete number");
      }
    }
    if (document_[position_] == '0') {
      ++position_;
      if (position_ < document_.size() &&
        std::isdigit(
          static_cast<unsigned char>(document_[position_])) != 0)
      {
        fail("leading zero in number");
      }
    } else if (document_[position_] >= '1' &&
      document_[position_] <= '9')
    {
      do {
        ++position_;
      } while (
        position_ < document_.size() &&
        std::isdigit(
          static_cast<unsigned char>(document_[position_])) != 0);
    } else {
      fail("invalid integer component");
    }
    if (position_ < document_.size() && document_[position_] == '.') {
      ++position_;
      const auto fraction_begin = position_;
      while (position_ < document_.size() &&
        std::isdigit(
          static_cast<unsigned char>(document_[position_])) != 0)
      {
        ++position_;
      }
      if (position_ == fraction_begin) {
        fail("fraction requires at least one digit");
      }
    }
    if (position_ < document_.size() &&
      (document_[position_] == 'e' || document_[position_] == 'E'))
    {
      ++position_;
      if (position_ < document_.size() &&
        (document_[position_] == '+' || document_[position_] == '-'))
      {
        ++position_;
      }
      const auto exponent_begin = position_;
      while (position_ < document_.size() &&
        std::isdigit(
          static_cast<unsigned char>(document_[position_])) != 0)
      {
        ++position_;
      }
      if (position_ == exponent_begin) {
        fail("exponent requires at least one digit");
      }
    }
  }

  std::string_view document_;
  std::string label_;
  std::size_t position_{0U};
};

YAML::Node load_strict_json(
  const std::string & document,
  const std::string & label)
{
  StrictJsonParser(document, label).parse();
  try {
    return YAML::Load(document);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            label + " passed JSON preflight but could not be decoded: " +
            error.what());
  }
}

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &value);
#else
  gmtime_r(&value, &utc);
#endif
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

std::string unique_suffix()
{
  static std::atomic<std::uint64_t> sequence{0U};
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  return fixed_hex(
    fnv1a64(std::to_string(ticks) + ":" + std::to_string(sequence.fetch_add(1U))),
    16);
}

void sync_file(const fs::path & path)
{
#ifndef _WIN32
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    throw std::runtime_error("failed to open staged file for fsync: " + path.string());
  }
  const int result = ::fsync(fd);
  const int saved_errno = errno;
  ::close(fd);
  if (result != 0) {
    throw std::runtime_error(
            "failed to fsync staged file: " + path.string() +
            " errno=" + std::to_string(saved_errno));
  }
#else
  (void)path;
#endif
}

void sync_directory(const fs::path & path)
{
#ifndef _WIN32
  const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY);
  if (fd < 0) {
    throw std::runtime_error("failed to open directory for fsync: " + path.string());
  }
  const int result = ::fsync(fd);
  const int saved_errno = errno;
  ::close(fd);
  if (result != 0) {
    throw std::runtime_error(
            "failed to fsync directory: " + path.string() +
            " errno=" + std::to_string(saved_errno));
  }
#else
  (void)path;
#endif
}

void durable_write(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open file for writing: " + path.string());
  }
  output.write(text.data(), static_cast<std::streamsize>(text.size()));
  output.flush();
  if (!output) {
    throw std::runtime_error("failed to write file: " + path.string());
  }
  output.close();
  sync_file(path);
}

void atomic_replace(const fs::path & target, const std::string & text)
{
  fs::create_directories(target.parent_path());
  const auto temporary =
    target.parent_path() / ("." + target.filename().string() + ".tmp-" + unique_suffix());
  try {
    durable_write(temporary, text);
#ifdef _WIN32
    const auto backup =
      target.parent_path() / ("." + target.filename().string() + ".bak-" + unique_suffix());
    const bool had_target = fs::exists(target);
    if (had_target) {
      fs::rename(target, backup);
    }
    try {
      fs::rename(temporary, target);
      if (had_target) {
        fs::remove(backup);
      }
    } catch (...) {
      std::error_code ignored;
      if (had_target && !fs::exists(target) && fs::exists(backup)) {
        fs::rename(backup, target, ignored);
      }
      throw;
    }
#else
    fs::rename(temporary, target);
#endif
    sync_directory(target.parent_path());
  } catch (...) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    throw;
  }
}

std::string read_bounded_managed_text(
  const fs::path & path,
  const std::string & label)
{
  std::error_code error;
  const auto size = fs::file_size(path, error);
  if (error) {
    throw std::runtime_error(
            "failed to inspect " + label + ": " + error.message());
  }
  if (size > kMaximumDocumentBytes) {
    throw std::runtime_error(label + " exceeds the 2 MiB limit");
  }
  auto text = read_text_file(path);
  if (text.size() > kMaximumDocumentBytes) {
    throw std::runtime_error(label + " exceeds the 2 MiB limit");
  }
  return text;
}

bool path_entry_exists(const fs::path & path)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  return !error && status.type() != fs::file_type::not_found;
}

#ifndef _WIN32
void install_relative_symlink(
  const fs::path & link_path,
  const fs::path & relative_target,
  const bool directory_target)
{
  fs::create_directories(link_path.parent_path());
  if (fs::is_symlink(link_path)) {
    std::error_code error;
    const auto existing = fs::read_symlink(link_path, error);
    if (!error && existing == relative_target) {
      sync_directory(link_path.parent_path());
      return;
    }
  }
  if (path_entry_exists(link_path) && !fs::is_symlink(link_path)) {
    throw std::runtime_error(
            "managed elevator projection path is not a symbolic link: " +
            link_path.string());
  }
  const auto temporary =
    link_path.parent_path() /
    ("." + link_path.filename().string() + ".link-" + unique_suffix());
  try {
    if (directory_target) {
      fs::create_directory_symlink(relative_target, temporary);
    } else {
      fs::create_symlink(relative_target, temporary);
    }
    fs::rename(temporary, link_path);
    sync_directory(link_path.parent_path());
  } catch (...) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    throw;
  }
}

void switch_current_release_link(
  const fs::path & config_root,
  const std::string & release_id)
{
  const auto current = config_root / "current";
  if (path_entry_exists(current) && !fs::is_symlink(current)) {
    throw std::runtime_error(
            "managed elevator current path is not a symbolic link");
  }
  const auto temporary =
    config_root / (".current-link-" + unique_suffix());
  try {
    fs::create_directory_symlink(
      fs::path("releases") / release_id, temporary);
    sync_directory(config_root);
    fs::rename(temporary, current);
    sync_directory(config_root);
  } catch (...) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    throw;
  }
}
#endif

bool path_is_within(const fs::path & child, const fs::path & parent)
{
  const auto normalized_child = fs::weakly_canonical(child);
  const auto normalized_parent = fs::weakly_canonical(parent);
  auto child_iterator = normalized_child.begin();
  for (auto parent_iterator = normalized_parent.begin();
    parent_iterator != normalized_parent.end(); ++parent_iterator, ++child_iterator)
  {
    if (child_iterator == normalized_child.end() || *child_iterator != *parent_iterator) {
      return false;
    }
  }
  return true;
}

bool safe_regular_file_under(
  const fs::path & path,
  const fs::path & managed_root)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  return !error &&
         status.type() == fs::file_type::regular &&
         !fs::is_symlink(path) &&
         path_is_within(path, managed_root);
}

void require_safe_regular_file_under(
  const fs::path & path,
  const fs::path & managed_root,
  const std::string & label)
{
  if (!safe_regular_file_under(path, managed_root)) {
    throw std::runtime_error(
            label + " is missing, is a symbolic link, or escapes managed storage: " +
            path.string());
  }
}

std::string emit_yaml(const YAML::Node & node)
{
  YAML::Emitter output;
  output.SetIndent(2);
  output << node;
  if (!output.good()) {
    throw std::runtime_error("failed to serialize elevator configuration YAML");
  }
  return std::string(output.c_str()) + "\n";
}

std::string scalar_json(const std::string & value)
{
  if (value == "null" || value == "Null" || value == "NULL" || value == "~") {
    return "null";
  }
  if (value == "true" || value == "True" || value == "TRUE") {
    return "true";
  }
  if (value == "false" || value == "False" || value == "FALSE") {
    return "false";
  }
  const auto canonical_unsigned_integer =
    !value.empty() &&
    (value == "0" ||
    (value.front() >= '1' && value.front() <= '9' &&
    std::all_of(
      value.begin() + 1, value.end(),
      [](const unsigned char character) {
        return std::isdigit(character) != 0;
      })));
  const auto canonical_negative_integer =
    value.size() > 1U && value.front() == '-' &&
    value[1] >= '1' && value[1] <= '9' &&
    std::all_of(
    value.begin() + 2, value.end(),
    [](const unsigned char character) {
      return std::isdigit(character) != 0;
    });
  if (canonical_unsigned_integer || canonical_negative_integer) {
    // Preserve integral identity exactly. Converting an asset epoch through a
    // double would silently round valid uint64_t values above 2^53.
    return value;
  }
  if (!value.empty()) {
    char * end = nullptr;
    errno = 0;
    const double number = std::strtod(value.c_str(), &end);
    if (errno == 0 && end == value.c_str() + value.size() && std::isfinite(number)) {
      std::ostringstream normalized;
      normalized << std::setprecision(std::numeric_limits<double>::max_digits10)
                 << number;
      return normalized.str();
    }
  }
  return json_string(value);
}

std::string yaml_node_json(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return "null";
  }
  if (node.IsScalar()) {
    const auto tag = node.Tag();
    if (tag == "!" || tag == "tag:yaml.org,2002:str") {
      return json_string(node.Scalar());
    }
    return scalar_json(node.Scalar());
  }
  if (node.IsSequence()) {
    std::ostringstream out;
    out << "[";
    for (std::size_t index = 0; index < node.size(); ++index) {
      if (index > 0U) {
        out << ",";
      }
      out << yaml_node_json(node[index]);
    }
    out << "]";
    return out.str();
  }
  if (node.IsMap()) {
    // yaml-cpp Node assignment mutates the referenced node instead of merely
    // rebinding a handle. Sorting pairs that contain Node values can therefore
    // corrupt the source tree. Materialize JSON values before sorting keys.
    std::vector<std::pair<std::string, std::string>> entries;
    for (const auto & item : node) {
      entries.emplace_back(
        item.first.as<std::string>(),
        yaml_node_json(item.second));
    }
    std::sort(
      entries.begin(), entries.end(),
      [](const auto & left, const auto & right) {return left.first < right.first;});
    std::ostringstream out;
    out << "{";
    for (std::size_t index = 0; index < entries.size(); ++index) {
      if (index > 0U) {
        out << ",";
      }
      out << json_string(entries[index].first) << ":"
          << entries[index].second;
    }
    out << "}";
    return out.str();
  }
  return "null";
}

std::string canonical_configuration_json(const YAML::Node & node)
{
  // JSON is valid YAML and retains quoted scalar types across persistence.
  // A YAML emitter can discard that distinction for ambiguous values such as
  // "01", "+1", and "true", causing a later GET to change their JSON types.
  return yaml_node_json(node) + "\n";
}

std::string issues_json(const std::vector<ValidationIssue> & issues)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0; index < issues.size(); ++index) {
    if (index > 0U) {
      out << ",";
    }
    out << "{\"severity\":\"error\","
        << "\"code\":" << json_string(issues[index].code) << ","
        << "\"field\":" << json_string(issues[index].field) << ","
        << "\"message\":" << json_string(issues[index].message) << "}";
  }
  out << "]";
  return out.str();
}

ElevatorConfigurationReply make_reply(
  const int status,
  const std::string & code,
  const std::string & fields = "")
{
  std::ostringstream body;
  body << "{\"ok\":" << ((status >= 200 && status < 300) ? "true" : "false")
       << ",\"code\":" << json_string(code);
  if (!fields.empty()) {
    body << "," << fields;
  }
  body << "}";
  return ElevatorConfigurationReply{status, code, body.str()};
}

void append_issue(
  ValidationResult & result,
  std::string code,
  std::string field,
  std::string message)
{
  result.issues.push_back(
    ValidationIssue{std::move(code), std::move(field), std::move(message)});
}

std::string topology_issue_code(
  const robot_elevator_manager::TopologyIssueCode code)
{
  using Code = robot_elevator_manager::TopologyIssueCode;
  switch (code) {
    case Code::kUnsafeElevatorId:
      return "UNSAFE_ELEVATOR_ID";
    case Code::kUnsafeBuildingId:
      return "UNSAFE_BUILDING_ID";
    case Code::kUnsafeFloorId:
      return "UNSAFE_FLOOR_ID";
    case Code::kUnsafeMapId:
      return "UNSAFE_MAP_ID";
    case Code::kUnsafePoseId:
      return "UNSAFE_POSE_ID";
    case Code::kTooFewFloors:
      return "TOO_FEW_FLOORS";
    case Code::kDuplicateFloor:
      return "DUPLICATE_FLOOR";
    case Code::kMissingPoseRole:
      return "MISSING_POSE_ROLE";
    case Code::kDuplicatePoseRole:
      return "DUPLICATE_POSE_ROLE";
    case Code::kUnknownPoseRole:
      return "UNKNOWN_POSE_ROLE";
    case Code::kDuplicatePoseId:
      return "DUPLICATE_POSE_ID";
    case Code::kInvalidThreshold:
      return "INVALID_THRESHOLD";
  }
  return "INVALID_TOPOLOGY";
}

std::optional<std::string> node_string(const YAML::Node & node)
{
  try {
    if (!node || !node.IsScalar()) {
      return std::nullopt;
    }
    return node.as<std::string>();
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

std::optional<double> node_number(const YAML::Node & node)
{
  try {
    if (!node || !node.IsScalar()) {
      return std::nullopt;
    }
    const double value = node.as<double>();
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    return value;
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

std::optional<std::uint64_t> node_positive_uint(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const auto tag = node.Tag();
  const auto value = node.Scalar();
  if (tag == "!" || tag == "tag:yaml.org,2002:str" ||
    value.empty() || value == "0" ||
    value.front() == '0' ||
    !std::all_of(
      value.begin(), value.end(),
      [](const unsigned char character) {
        return std::isdigit(character) != 0;
      }))
  {
    return std::nullopt;
  }
  try {
    std::size_t parsed = 0U;
    const auto result = std::stoull(value, &parsed, 10);
    if (parsed != value.size() || result == 0U) {
      return std::nullopt;
    }
    return static_cast<std::uint64_t>(result);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::string> json_node_string(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const auto tag = node.Tag();
  if (tag != "!" && tag != "tag:yaml.org,2002:str") {
    return std::nullopt;
  }
  return node.Scalar();
}

std::optional<bool> json_node_bool(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const auto tag = node.Tag();
  if (tag == "!" || tag == "tag:yaml.org,2002:str") {
    return std::nullopt;
  }
  if (node.Scalar() == "true") {
    return true;
  }
  if (node.Scalar() == "false") {
    return false;
  }
  return std::nullopt;
}

std::optional<std::uint64_t> json_node_uint(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return std::nullopt;
  }
  const auto tag = node.Tag();
  const auto value = node.Scalar();
  if (tag == "!" || tag == "tag:yaml.org,2002:str" ||
    value.empty() ||
    !std::all_of(
      value.begin(), value.end(),
      [](const unsigned char character) {
        return std::isdigit(character) != 0;
      }))
  {
    return std::nullopt;
  }
  try {
    std::size_t parsed = 0U;
    const auto result = std::stoull(value, &parsed, 10);
    if (parsed != value.size()) {
      return std::nullopt;
    }
    return static_cast<std::uint64_t>(result);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool read_point(
  const YAML::Node & node,
  Point2 & point)
{
  if (!node || !node.IsSequence() || node.size() != 2U) {
    return false;
  }
  const auto x = node_number(node[0]);
  const auto y = node_number(node[1]);
  if (!x || !y) {
    return false;
  }
  point = Point2{*x, *y};
  return true;
}

bool point_inside_map(
  const double x,
  const double y,
  const MapYamlInfo & map)
{
  if (map.width == 0U || map.height == 0U || !std::isfinite(map.resolution) ||
    map.resolution <= 0.0 || !std::isfinite(map.origin[0]) ||
    !std::isfinite(map.origin[1]) || !std::isfinite(map.origin[2]))
  {
    return false;
  }
  const double dx = x - map.origin[0];
  const double dy = y - map.origin[1];
  const double cosine = std::cos(map.origin[2]);
  const double sine = std::sin(map.origin[2]);
  const double map_x = cosine * dx + sine * dy;
  const double map_y = -sine * dx + cosine * dy;
  const double width_m = static_cast<double>(map.width) * map.resolution;
  const double height_m = static_cast<double>(map.height) * map.resolution;
  return map_x >= 0.0 && map_y >= 0.0 && map_x < width_m && map_y < height_m;
}

double normalized_yaw(double yaw)
{
  yaw = std::remainder(yaw, 2.0 * kPi);
  if (yaw <= -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

std::string internal_pose_id(
  const std::string & building_id,
  const std::string & elevator_id,
  const std::string & floor_id,
  const std::string & map_id,
  const PoseRole role)
{
  const auto role_name = robot_elevator_manager::to_string(role);
  const std::string key =
    building_id + "\n" + elevator_id + "\n" + floor_id + "\n" + map_id + "\n" + role_name;
  return "eip_" + fixed_hex(fnv1a64(key), 16) + "_" + role_name;
}

YAML::Node pose_node(const PoseRecord & pose)
{
  YAML::Node node;
  node["x"] = pose.x;
  node["y"] = pose.y;
  node["yaw"] = pose.yaw;
  return node;
}

std::string bindings_json(const std::vector<MapBinding> & bindings)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0; index < bindings.size(); ++index) {
    if (index > 0U) {
      out << ",";
    }
    out << "{\"floor_id\":" << json_string(bindings[index].floor_id) << ","
        << "\"map_id\":" << json_string(bindings[index].map_id) << ","
        << "\"asset_epoch\":" << bindings[index].asset_epoch << ","
        << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
        << "\"asset_digest_algorithm\":\"sha256\","
        << "\"asset_digest\":" << json_string(bindings[index].asset_digest) << "}";
  }
  out << "]";
  return out.str();
}

ValidationResult validate_document(
  const std::string & expected_building_id,
  const YAML::Node & root,
  const ElevatorFloorAssetResolver & resolver)
{
  ValidationResult result;
  if (!root || !root.IsMap()) {
    append_issue(result, "INVALID_ROOT", "", "configuration root must be an object");
    return result;
  }

  const auto schema_version = node_number(root["schema_version"]);
  if (!schema_version || *schema_version != 1.0) {
    append_issue(
      result, "UNSUPPORTED_SCHEMA_VERSION", "schema_version",
      "schema_version must be 1");
  }
  const auto building_id = node_string(root["building_id"]);
  if (!building_id) {
    append_issue(result, "MISSING_BUILDING_ID", "building_id", "building_id is required");
  } else if (*building_id != expected_building_id) {
    append_issue(
      result, "BUILDING_ID_MISMATCH", "building_id",
      "document building_id must match the requested building_id");
  } else if (!robot_elevator_manager::safe_asset_id(*building_id)) {
    append_issue(
      result, "UNSAFE_BUILDING_ID", "building_id",
      "building_id must be a bounded path-safe identifier");
  }

  try {
    if (root["mock_ports_enabled"] && root["mock_ports_enabled"].as<bool>()) {
      append_issue(
        result, "MOCK_PORTS_FORBIDDEN", "mock_ports_enabled",
        "published elevator configuration always disables mock ports");
    }
  } catch (const YAML::Exception &) {
    append_issue(
      result, "INVALID_MOCK_PORTS", "mock_ports_enabled",
      "mock_ports_enabled must be false when supplied");
  }

  const auto elevators = root["elevators"];
  if (!elevators || !elevators.IsSequence() || elevators.size() == 0U) {
    append_issue(
      result, "MISSING_ELEVATORS", "elevators",
      "at least one elevator is required");
    return result;
  }
  bool configuration_limit_exceeded = false;
  if (elevators.size() > kMaximumElevators) {
    append_issue(
      result, "CONFIGURATION_LIMIT_EXCEEDED", "elevators",
      "configuration exceeds the maximum of 16 elevators");
    configuration_limit_exceeded = true;
  }
  std::size_t total_floor_bindings = 0U;
  for (std::size_t elevator_index = 0U;
    elevator_index < elevators.size() && !configuration_limit_exceeded;
    ++elevator_index)
  {
    const auto elevator = elevators[elevator_index];
    if (!elevator.IsMap()) {
      continue;
    }
    const auto floors = elevator["floors"];
    if (!floors || !floors.IsSequence()) {
      continue;
    }
    if (floors.size() > kMaximumFloorsPerElevator) {
      append_issue(
        result, "CONFIGURATION_LIMIT_EXCEEDED",
        "elevators[" + std::to_string(elevator_index) + "].floors",
        "an elevator exceeds the maximum of 64 served floors");
      configuration_limit_exceeded = true;
      break;
    }
    if (floors.size() > kMaximumFloorBindings - total_floor_bindings) {
      append_issue(
        result, "CONFIGURATION_LIMIT_EXCEEDED", "elevators",
        "configuration exceeds the maximum of 128 floor bindings");
      configuration_limit_exceeded = true;
      break;
    }
    total_floor_bindings += floors.size();
  }
  if (configuration_limit_exceeded) {
    return result;
  }

  std::set<std::string> elevator_ids;
  std::set<std::string> global_pose_ids;
  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>> map_binding_identities;
  std::map<
    std::pair<std::string, std::string>,
    std::optional<ElevatorFloorAssetSnapshot>> snapshot_cache;

  for (std::size_t elevator_index = 0; elevator_index < elevators.size(); ++elevator_index) {
    const auto elevator_node = elevators[elevator_index];
    const std::string elevator_path =
      "elevators[" + std::to_string(elevator_index) + "]";
    if (!elevator_node.IsMap()) {
      append_issue(
        result, "INVALID_ELEVATOR", elevator_path,
        "elevator entry must be an object");
      continue;
    }

    ElevatorRecord elevator;
    elevator.elevator_id = node_string(elevator_node["elevator_id"]).value_or("");
    elevator.display_name =
      node_string(elevator_node["display_name"]).value_or(elevator.elevator_id);
    if (!robot_elevator_manager::safe_asset_id(elevator.elevator_id)) {
      append_issue(
        result, "UNSAFE_ELEVATOR_ID", elevator_path + ".elevator_id",
        "elevator_id must be a bounded path-safe identifier");
    } else if (!elevator_ids.insert(elevator.elevator_id).second) {
      append_issue(
        result, "DUPLICATE_ELEVATOR_ID", elevator_path + ".elevator_id",
        "elevator_id must be unique within the building");
    }

    ElevatorTopology topology;
    topology.elevator_id = elevator.elevator_id;
    topology.building_id = expected_building_id;

    const auto floors = elevator_node["floors"];
    if (!floors || !floors.IsSequence()) {
      append_issue(
        result, "INVALID_FLOORS", elevator_path + ".floors",
        "floors must be a sequence");
    } else {
      for (std::size_t floor_index = 0; floor_index < floors.size(); ++floor_index) {
        const auto floor_node = floors[floor_index];
        const std::string floor_path =
          elevator_path + ".floors[" + std::to_string(floor_index) + "]";
        if (!floor_node.IsMap()) {
          append_issue(
            result, "INVALID_FLOOR", floor_path,
            "floor entry must be an object");
          continue;
        }

        FloorRecord floor;
        floor.floor_id = node_string(floor_node["floor_id"]).value_or("");
        floor.map_id = node_string(floor_node["map_id"]).value_or("");
        const auto bound_epoch =
          node_positive_uint(floor_node["map_asset_epoch"]);
        if (floor_node["map_asset_epoch"] && !bound_epoch) {
          append_issue(
            result, "INVALID_MAP_ASSET_EPOCH",
            floor_path + ".map_asset_epoch",
            "map_asset_epoch must be an unquoted positive uint64 integer when supplied");
        }
        const auto bound_digest = node_string(floor_node["map_asset_digest"]);
        if (floor_node["map_asset_digest"] && !bound_digest) {
          append_issue(
            result, "INVALID_MAP_ASSET_DIGEST",
            floor_path + ".map_asset_digest",
            "map_asset_digest must be a scalar digest when supplied");
        } else if (
          bound_digest &&
          !is_canonical_sha256_digest(*bound_digest))
        {
          append_issue(
            result, "NON_CANONICAL_MAP_ASSET_DIGEST",
            floor_path + ".map_asset_digest",
            "map_asset_digest must use sha256:<64 lowercase hex>");
        }
        FloorElevatorTopology topology_floor;
        topology_floor.floor_id = floor.floor_id;
        topology_floor.map_id = floor.map_id;

        const auto poses = floor_node["poses"];
        if (!poses || !poses.IsMap()) {
          append_issue(
            result, "INVALID_POSES", floor_path + ".poses",
            "poses must be an object containing all five roles");
        } else {
          const std::set<std::string> known_roles{
            "hall_call", "hall_wait", "doorway", "cabin", "exit"};
          for (const auto & entry : poses) {
            const auto name = node_string(entry.first).value_or("");
            if (known_roles.count(name) == 0U) {
              append_issue(
                result, "UNKNOWN_POSE_ROLE", floor_path + ".poses." + name,
                "pose role is not part of the elevator configuration contract");
            }
          }
          for (const auto role : robot_elevator_manager::required_pose_roles()) {
            const auto role_name = robot_elevator_manager::to_string(role);
            const auto pose = poses[role_name];
            const std::string pose_path = floor_path + ".poses." + role_name;
            if (!pose || !pose.IsMap()) {
              append_issue(
                result, "MISSING_POSE_ROLE", pose_path,
                "required elevator pose role is missing");
              continue;
            }
            const auto x = node_number(pose["x"]);
            const auto y = node_number(pose["y"]);
            const auto yaw = node_number(pose["yaw"]);
            if (!x || !y || !yaw) {
              append_issue(
                result, "INVALID_POSE", pose_path,
                "pose x, y, and yaw must be finite numbers");
              continue;
            }
            PoseRecord record;
            record.role = role;
            record.x = *x;
            record.y = *y;
            record.yaw = normalized_yaw(*yaw);
            record.pose_id = internal_pose_id(
              expected_building_id, elevator.elevator_id, floor.floor_id,
              floor.map_id, role);
            if (!robot_elevator_manager::safe_pose_id(record.pose_id) ||
              !global_pose_ids.insert(record.pose_id).second)
            {
              append_issue(
                result, "DUPLICATE_INTERNAL_POSE_ID", pose_path,
                "generated internal pose ID is unsafe or not globally unique");
            }
            topology_floor.poses.push_back(PoseBinding{role, record.pose_id});
            floor.poses.push_back(std::move(record));
          }
        }

        const auto threshold = floor_node["threshold"];
        bool threshold_ok = threshold && threshold.IsMap();
        if (threshold_ok) {
          threshold_ok =
            read_point(threshold["left"], floor.threshold.left) &&
            read_point(threshold["right"], floor.threshold.right) &&
            read_point(threshold["cabin_reference"], floor.threshold.cabin_reference);
          const auto clearance = node_number(threshold["clearance_m"]);
          const auto jamb_clearance = node_number(threshold["jamb_clearance_m"]);
          if (threshold["clearance_m"] && !clearance) {
            append_issue(
              result, "INVALID_THRESHOLD_CLEARANCE",
              floor_path + ".threshold.clearance_m",
              "clearance_m must be a finite non-negative number");
          }
          if (threshold["jamb_clearance_m"] && !jamb_clearance) {
            append_issue(
              result, "INVALID_THRESHOLD_JAMB_CLEARANCE",
              floor_path + ".threshold.jamb_clearance_m",
              "jamb_clearance_m must be a finite non-negative number");
          }
          floor.threshold.clearance_m = clearance.value_or(0.05);
          floor.threshold.jamb_clearance_m = jamb_clearance.value_or(0.05);
        }
        if (!threshold_ok) {
          append_issue(
            result, "INVALID_THRESHOLD", floor_path + ".threshold",
            "threshold requires finite left, right, and cabin_reference points");
        }
        topology_floor.threshold = floor.threshold;

        std::optional<ElevatorFloorAssetSnapshot> snapshot;
        if (!robot_elevator_manager::safe_asset_id(floor.floor_id)) {
          append_issue(
            result, "UNSAFE_FLOOR_ID", floor_path + ".floor_id",
            "floor_id must be a bounded path-safe identifier");
        }
        if (!robot_elevator_manager::safe_asset_id(floor.map_id)) {
          append_issue(
            result, "UNSAFE_MAP_ID", floor_path + ".map_id",
            "map_id must be a bounded path-safe identifier");
        }
        if (robot_elevator_manager::safe_asset_id(floor.floor_id) &&
          robot_elevator_manager::safe_asset_id(floor.map_id))
        {
          const auto cache_key =
            std::make_pair(floor.floor_id, floor.map_id);
          auto cached = snapshot_cache.find(cache_key);
          if (cached == snapshot_cache.end()) {
            cached = snapshot_cache.emplace(
              cache_key,
              resolver(
                expected_building_id, floor.floor_id, floor.map_id)).first;
          }
          snapshot = cached->second;
          if (!snapshot) {
            append_issue(
              result, "MAP_NOT_FOUND", floor_path + ".map_id",
              "map_id does not exist for the requested building and floor");
          } else if (
            snapshot->manifest.building_id != expected_building_id ||
            snapshot->manifest.floor_id != floor.floor_id ||
            snapshot->manifest.map_id != floor.map_id)
          {
            append_issue(
              result, "MAP_BINDING_MISMATCH", floor_path + ".map_id",
              "resolved map asset does not exactly match building, floor, and map IDs");
          } else {
            if (!snapshot->required_assets_complete || !snapshot->map_info ||
              snapshot->asset_digest.empty())
            {
              append_issue(
                result, "MAP_ASSETS_INCOMPLETE", floor_path + ".map_id",
                "map bundle must contain readable navigation and localization assets");
            } else {
              const bool epoch_is_authoritative = snapshot->asset_epoch > 0U;
              if (!epoch_is_authoritative) {
                append_issue(
                  result, "MAP_ASSET_EPOCH_UNAVAILABLE",
                  floor_path + ".map_asset_epoch",
                  "resolved map bundle has no authoritative positive asset epoch");
              }
              const bool digest_is_canonical =
                is_canonical_sha256_digest(snapshot->asset_digest);
              if (!digest_is_canonical) {
                append_issue(
                  result, "NON_CANONICAL_MAP_ASSET_DIGEST",
                  floor_path + ".map_asset_digest",
                  "resolved map bundle identity must use sha256:<64 lowercase hex>");
              }
              if (epoch_is_authoritative && digest_is_canonical) {
                floor.asset_epoch = snapshot->asset_epoch;
                floor.asset_digest = snapshot->asset_digest;
                if (bound_epoch && *bound_epoch != snapshot->asset_epoch) {
                  append_issue(
                    result, "MAP_ASSET_EPOCH_CHANGED",
                    floor_path + ".map_asset_epoch",
                    "map asset epoch changed after this elevator configuration was reviewed");
                }
                if (bound_digest && *bound_digest != snapshot->asset_digest) {
                  append_issue(
                    result, "MAP_ASSET_DIGEST_CHANGED",
                    floor_path + ".map_asset_digest",
                    "map assets changed after this elevator configuration was published");
                }
                const auto binding_key =
                  std::make_pair(floor.floor_id, floor.map_id);
                const auto inserted =
                  map_binding_identities.emplace(
                  binding_key,
                  std::make_pair(
                    snapshot->asset_epoch, snapshot->asset_digest));
                if (!inserted.second &&
                  inserted.first->second !=
                  std::make_pair(
                    snapshot->asset_epoch, snapshot->asset_digest))
                {
                  append_issue(
                    result, "MAP_ASSET_IDENTITY_CONFLICT", floor_path + ".map_id",
                    "the same map binding resolved to different asset identities");
                }
              }
              for (const auto & pose : floor.poses) {
                if (!point_inside_map(pose.x, pose.y, *snapshot->map_info)) {
                  append_issue(
                    result, "POSE_OUT_OF_MAP",
                    floor_path + ".poses." +
                    robot_elevator_manager::to_string(pose.role),
                    "pose lies outside the bound map extent");
                }
              }
              const std::vector<std::pair<std::string, Point2>> threshold_points{
                {"left", floor.threshold.left},
                {"right", floor.threshold.right},
                {"cabin_reference", floor.threshold.cabin_reference},
              };
              for (const auto & point : threshold_points) {
                if (!point_inside_map(point.second.x, point.second.y, *snapshot->map_info)) {
                  append_issue(
                    result, "THRESHOLD_OUT_OF_MAP",
                    floor_path + ".threshold." + point.first,
                    "threshold point lies outside the bound map extent");
                }
              }
            }
          }
        }

        topology.floors.push_back(std::move(topology_floor));
        elevator.floors.push_back(std::move(floor));
      }
    }

    const auto topology_validation =
      robot_elevator_manager::validate_topology(topology);
    for (const auto & issue : topology_validation.issues) {
      append_issue(
        result, topology_issue_code(issue.code),
        elevator_path + "." + issue.field, issue.message);
    }
    result.elevators.push_back(std::move(elevator));
  }

  for (const auto & binding : map_binding_identities) {
    result.bindings.push_back(
      MapBinding{
        binding.first.first,
        binding.first.second,
        binding.second.first,
        binding.second.second});
  }

  YAML::Node configuration;
  configuration["schema_version"] = 1;
  configuration["building_id"] = expected_building_id;
  YAML::Node topology;
  topology["schema_version"] = 1;
  topology["mock_ports_enabled"] = false;
  topology["building_id"] = expected_building_id;
  YAML::Node internal;
  internal["schema_version"] = 1;
  internal["building_id"] = expected_building_id;
  YAML::Node internal_poses(YAML::NodeType::Sequence);

  for (const auto & elevator : result.elevators) {
    YAML::Node configuration_elevator;
    configuration_elevator["elevator_id"] = elevator.elevator_id;
    if (!elevator.display_name.empty()) {
      configuration_elevator["display_name"] = elevator.display_name;
    }
    YAML::Node topology_elevator;
    topology_elevator["elevator_id"] = elevator.elevator_id;

    for (const auto & floor : elevator.floors) {
      YAML::Node configuration_floor;
      configuration_floor["floor_id"] = floor.floor_id;
      configuration_floor["map_id"] = floor.map_id;
      if (floor.asset_epoch > 0U) {
        configuration_floor["map_asset_epoch"] = floor.asset_epoch;
      }
      if (!floor.asset_digest.empty()) {
        configuration_floor["map_asset_digest"] = floor.asset_digest;
      }
      YAML::Node topology_floor;
      topology_floor["floor_id"] = floor.floor_id;
      topology_floor["map_id"] = floor.map_id;

      for (const auto & pose : floor.poses) {
        const auto role_name = robot_elevator_manager::to_string(pose.role);
        configuration_floor["poses"][role_name] = pose_node(pose);
        topology_floor["poses"][role_name] = pose.pose_id;

        YAML::Node internal_pose;
        internal_pose["pose_id"] = pose.pose_id;
        internal_pose["type"] = "elevator_internal";
        internal_pose["elevator_id"] = elevator.elevator_id;
        internal_pose["floor_id"] = floor.floor_id;
        internal_pose["map_id"] = floor.map_id;
        internal_pose["role"] = role_name;
        internal_pose["x"] = pose.x;
        internal_pose["y"] = pose.y;
        internal_pose["yaw"] = pose.yaw;
        internal_poses.push_back(internal_pose);
      }

      const auto write_threshold = [&floor](YAML::Node target) {
          target["left"].push_back(floor.threshold.left.x);
          target["left"].push_back(floor.threshold.left.y);
          target["right"].push_back(floor.threshold.right.x);
          target["right"].push_back(floor.threshold.right.y);
          target["cabin_reference"].push_back(floor.threshold.cabin_reference.x);
          target["cabin_reference"].push_back(floor.threshold.cabin_reference.y);
          target["clearance_m"] = floor.threshold.clearance_m;
          target["jamb_clearance_m"] = floor.threshold.jamb_clearance_m;
        };
      write_threshold(configuration_floor["threshold"]);
      write_threshold(topology_floor["threshold"]);
      configuration_elevator["floors"].push_back(configuration_floor);
      topology_elevator["floors"].push_back(topology_floor);
    }
    configuration["elevators"].push_back(configuration_elevator);
    topology["elevators"].push_back(topology_elevator);
  }
  internal["poses"] = internal_poses;

  result.configuration_yaml = emit_yaml(configuration);
  result.topology_yaml = emit_yaml(topology);
  result.internal_poses_yaml = emit_yaml(internal);
  if (result.ok()) {
    const auto round_trip =
      robot_elevator_manager::parse_topology_yaml(result.topology_yaml);
    if (!round_trip.ok()) {
      append_issue(
        result, "TOPOLOGY_ROUND_TRIP_FAILED", "elevators",
        round_trip.errors.empty() ?
        "generated topology could not be parsed" : round_trip.errors.front());
    }
  }
  return result;
}

std::string draft_configuration_with_map_bindings(
  const YAML::Node & root,
  const std::vector<MapBinding> & bindings)
{
  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>> identities;
  for (const auto & binding : bindings) {
    identities.emplace(
      std::make_pair(binding.floor_id, binding.map_id),
      std::make_pair(binding.asset_epoch, binding.asset_digest));
  }
  auto stamped = load_strict_json(
    canonical_configuration_json(root),
    "server-generated elevator draft");
  YAML::Node elevators = stamped["elevators"];
  if (!stamped.IsMap() || !elevators || !elevators.IsSequence()) {
    throw std::runtime_error(
            "validated elevator draft cannot be cloned for binding");
  }
  for (std::size_t elevator_index = 0U;
    elevator_index < elevators.size(); ++elevator_index)
  {
    YAML::Node elevator = elevators[elevator_index];
    YAML::Node floors = elevator["floors"];
    for (std::size_t floor_index = 0U;
      floor_index < floors.size(); ++floor_index)
    {
      YAML::Node floor = floors[floor_index];
      const auto floor_id = node_string(floor["floor_id"]);
      const auto map_id = node_string(floor["map_id"]);
      if (!floor_id || !map_id) {
        throw std::runtime_error(
                "validated elevator draft lost a floor map binding");
      }
      const auto identity = identities.find(
        std::make_pair(*floor_id, *map_id));
      if (identity == identities.end() ||
        identity->second.first == 0U ||
        identity->second.second.empty())
      {
        throw std::runtime_error(
                "validated elevator draft has no resolved map asset identity");
      }
      floor["map_asset_epoch"] = identity->second.first;
      floor["map_asset_digest"] = identity->second.second;
    }
  }
  if (!stamped["elevators"].IsSequence()) {
    throw std::runtime_error(
            "elevator draft binding stamp corrupted the elevator sequence");
  }
  return canonical_configuration_json(stamped);
}

std::string validation_json(const ValidationResult & validation)
{
  return std::string("{\"valid_for_publish\":") +
         (validation.ok() ? "true" : "false") +
         ",\"issues\":" + issues_json(validation.issues) + "}\n";
}

bool valid_release_id(const std::string & value)
{
  constexpr std::size_t kPrefixLength = 16U;
  constexpr std::size_t kGenerationLength = 6U;
  constexpr std::size_t kDigestLength = 12U;
  if (value.size() !=
    kPrefixLength + kGenerationLength + 1U + kDigestLength ||
    value.rfind("elevator-config-", 0U) != 0U ||
    value[kPrefixLength + kGenerationLength] != '-')
  {
    return false;
  }
  const auto generation_begin = value.begin() +
    static_cast<std::ptrdiff_t>(kPrefixLength);
  const auto generation_end = generation_begin +
    static_cast<std::ptrdiff_t>(kGenerationLength);
  if (!std::all_of(
      generation_begin, generation_end,
      [](const unsigned char character) {return std::isdigit(character) != 0;}))
  {
    return false;
  }
  return std::all_of(
    generation_end + 1, value.end(),
    [](const unsigned char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

bool valid_draft_revision(const std::string & value)
{
  constexpr std::size_t kPrefixLength = 9U;
  constexpr std::size_t kDigestLength = 16U;
  if (value.size() != kPrefixLength + kDigestLength ||
    value.rfind("draft-v1-", 0U) != 0U)
  {
    return false;
  }
  return std::all_of(
    value.begin() + static_cast<std::ptrdiff_t>(kPrefixLength),
    value.end(),
    [](const unsigned char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

std::string draft_revision_for(
  const std::string & building_id,
  const std::string & configuration,
  const std::string & validation)
{
  return "draft-v1-" + fixed_hex(
    fnv1a64(
      building_id + "\n" + configuration + "\n" + validation),
    16);
}

struct VerifiedDraftBundle
{
  std::string configuration;
  std::string validation;
};

VerifiedDraftBundle load_verified_draft_bundle(
  const fs::path & config_root,
  const std::string & building_id,
  const std::string & revision)
{
  if (!valid_draft_revision(revision)) {
    throw std::runtime_error("elevator draft revision identity is invalid");
  }
  const auto draft_root = config_root / "drafts" / revision;
  if (fs::is_symlink(draft_root) ||
    !fs::is_directory(draft_root) ||
    !path_is_within(draft_root, config_root / "drafts"))
  {
    throw std::runtime_error("elevator draft directory is unsafe or missing");
  }
  require_safe_regular_file_under(
    draft_root / "configuration.yaml", draft_root,
    "elevator draft configuration");
  require_safe_regular_file_under(
    draft_root / "validation.json", draft_root,
    "elevator draft validation");
  VerifiedDraftBundle bundle;
  bundle.configuration =
    read_bounded_managed_text(
    draft_root / "configuration.yaml",
    "elevator draft configuration");
  bundle.validation =
    read_bounded_managed_text(
    draft_root / "validation.json",
    "elevator draft validation");
  const auto canonical_configuration =
    canonical_configuration_json(
    load_strict_json(
      bundle.configuration, "elevator draft configuration"));
  if (canonical_configuration != bundle.configuration ||
    draft_revision_for(
      building_id, bundle.configuration, bundle.validation) != revision)
  {
    throw std::runtime_error(
            "elevator draft content no longer matches its immutable revision");
  }
  return bundle;
}

DraftPointer read_draft_pointer(const fs::path & config_root)
{
  const auto path = config_root / "draft.json";
  if (!path_entry_exists(path)) {
    return {};
  }
  require_safe_regular_file_under(
    path, config_root, "elevator draft pointer");
  const auto text =
    read_bounded_managed_text(path, "elevator draft pointer");
  const auto root = load_strict_json(text, "elevator draft pointer");
  if (!root.IsMap()) {
    throw std::runtime_error("elevator draft pointer is corrupt");
  }
  const auto revision = json_node_string(root["draft_revision"]);
  const auto schema = json_node_uint(root["schema_version"]);
  if (!schema || *schema != 1U ||
    !revision || !valid_draft_revision(*revision))
  {
    throw std::runtime_error("elevator draft pointer is corrupt");
  }
  return DraftPointer{true, *revision};
}

ReleasePointer read_release_pointer(const fs::path & config_root)
{
  fs::path path;
#ifndef _WIN32
  const auto current_link = config_root / "current";
  if (path_entry_exists(current_link)) {
    if (!fs::is_symlink(current_link)) {
      throw std::runtime_error("elevator current selector is not a symbolic link");
    }
    std::error_code target_error;
    const auto relative_target = fs::read_symlink(current_link, target_error);
    if (target_error || relative_target.is_absolute() ||
      relative_target.parent_path() != fs::path("releases") ||
      !robot_elevator_manager::safe_asset_id(
        relative_target.filename().string()))
    {
      throw std::runtime_error(
              "elevator current selector target is not a direct managed release");
    }
    const auto selected_release = config_root / relative_target;
    if (!path_is_within(selected_release, config_root / "releases") ||
      fs::is_symlink(selected_release) ||
      !fs::is_directory(selected_release))
    {
      throw std::runtime_error("elevator current selector escapes the release store");
    }
    path = selected_release / "current.json";
    require_safe_regular_file_under(
      path, selected_release, "elevator release metadata");
  } else {
    path = config_root / "current.json";
    if (!path_entry_exists(path)) {
      return {};
    }
    if (fs::is_symlink(path)) {
      std::error_code target_error;
      const auto target = fs::read_symlink(path, target_error);
      if (!target_error && target == fs::path("current") / "current.json" &&
        !fs::exists(path))
      {
        // A first publication may have installed the stable view before the
        // single authoritative selector was switched. Treat that state as
        // unpublished so an exact retry can finish the transaction.
        return {};
      }
      throw std::runtime_error("legacy elevator release pointer is an unsafe symbolic link");
    }
    require_safe_regular_file_under(
      path, config_root, "legacy elevator release pointer");
  }
#else
  path = config_root / "current.json";
  if (!path_entry_exists(path)) {
    return {};
  }
  require_safe_regular_file_under(
    path, config_root, "elevator release pointer");
#endif
  const auto text =
    read_bounded_managed_text(path, "elevator release pointer");
  const auto root = load_strict_json(text, "elevator release pointer");
  if (!root.IsMap()) {
    throw std::runtime_error("elevator release pointer is corrupt");
  }
  const auto release_id = json_node_string(root["release_id"]);
  const auto generation = json_node_uint(root["generation"]);
  const auto schema = json_node_uint(root["schema_version"]);
  if (!schema || *schema != 1U ||
    !release_id || !valid_release_id(*release_id) ||
    !generation || *generation < 1U)
  {
    throw std::runtime_error("elevator release pointer is corrupt");
  }
  if (path.parent_path().filename().string() != ".elevator_config" &&
    path.parent_path().filename().string() != *release_id)
  {
    throw std::runtime_error(
            "elevator current selector and release metadata disagree");
  }
  return ReleasePointer{
    true, *release_id, *generation};
}

std::string draft_pointer_json(
  const std::string & revision,
  const std::string & actor_id)
{
  return std::string("{\"schema_version\":1,\"draft_revision\":") +
         json_string(revision) + ",\"updated_at\":" + json_string(utc_now()) +
         ",\"actor_id\":" + json_string(actor_id) + "}\n";
}

std::string current_pointer_json(
  const std::string & release_id,
  const std::string & parent_release_id,
  const std::uint64_t generation,
  const std::string & actor_id,
  const std::string & rollback_of)
{
  std::ostringstream out;
  out << "{\"schema_version\":1,"
      << "\"release_id\":" << json_string(release_id) << ","
      << "\"parent_release_id\":"
      << (parent_release_id.empty() ? "null" : json_string(parent_release_id)) << ","
      << "\"generation\":" << generation << ","
      << "\"published_at\":" << json_string(utc_now()) << ","
      << "\"actor_id\":" << json_string(actor_id) << ","
      << "\"rollback_of\":"
      << (rollback_of.empty() ? "null" : json_string(rollback_of)) << "}\n";
  return out.str();
}

std::string release_manifest_json(
  const std::string & release_id,
  const std::string & parent_release_id,
  const std::string & source_draft_revision,
  const std::uint64_t generation,
  const std::string & actor_id,
  const std::string & rollback_of,
  const ValidationResult & validation)
{
  const auto content_digest = fixed_hex(
    fnv1a64(
      validation.configuration_yaml + validation.topology_yaml +
      validation.internal_poses_yaml),
    16);
  std::ostringstream out;
  out << "{\"schema_version\":1,"
      << "\"release_id\":" << json_string(release_id) << ","
      << "\"parent_release_id\":"
      << (parent_release_id.empty() ? "null" : json_string(parent_release_id)) << ","
      << "\"source_draft_revision\":"
      << (source_draft_revision.empty() ? "null" : json_string(source_draft_revision)) << ","
      << "\"generation\":" << generation << ","
      << "\"created_at\":" << json_string(utc_now()) << ","
      << "\"actor_id\":" << json_string(actor_id) << ","
      << "\"rollback_of\":"
      << (rollback_of.empty() ? "null" : json_string(rollback_of)) << ","
      << "\"configuration_digest_algorithm\":\"fnv1a64\","
      << "\"configuration_digest\":" << json_string(content_digest) << ","
      << "\"map_bindings\":" << bindings_json(validation.bindings) << ","
      << "\"asset_published\":true,\"runtime_applied\":false}\n";
  return out.str();
}

std::string release_id_for(
  const std::uint64_t generation,
  const ValidationResult & validation)
{
  std::ostringstream prefix;
  prefix << "elevator-config-" << std::setw(6) << std::setfill('0') << generation << "-";
  const auto digest = fixed_hex(
    fnv1a64(
      std::to_string(generation) + "\n" + validation.configuration_yaml +
      validation.topology_yaml + validation.internal_poses_yaml),
    16);
  return prefix.str() + digest.substr(0U, 12U);
}

struct VerifiedReleaseBundle
{
  std::string configuration;
  std::string topology;
  std::string internal_poses;
  std::string validation;
  std::string manifest;
  std::string current_metadata;
  std::uint64_t generation{0U};
};

VerifiedReleaseBundle load_verified_release_bundle(
  const fs::path & release_root,
  const std::string & expected_release_id,
  const std::optional<std::uint64_t> expected_generation = std::nullopt)
{
  if (!valid_release_id(expected_release_id) ||
    fs::is_symlink(release_root) ||
    !fs::is_directory(release_root))
  {
    throw std::runtime_error("elevator release directory identity is invalid");
  }

  const std::vector<std::string> required_files{
    "configuration.yaml",
    "elevators.yaml",
    "elevator_internal_poses.yaml",
    "validation.json",
    "manifest.json",
    "current.json",
  };
  for (const auto & filename : required_files) {
    require_safe_regular_file_under(
      release_root / filename, release_root,
      "elevator release file " + filename);
  }

  VerifiedReleaseBundle bundle;
  bundle.configuration =
    read_bounded_managed_text(
    release_root / "configuration.yaml",
    "elevator release configuration");
  bundle.topology = read_bounded_managed_text(
    release_root / "elevators.yaml",
    "elevator release topology");
  bundle.internal_poses =
    read_bounded_managed_text(
    release_root / "elevator_internal_poses.yaml",
    "elevator release internal poses");
  bundle.validation = read_bounded_managed_text(
    release_root / "validation.json",
    "elevator release validation");
  bundle.manifest = read_bounded_managed_text(
    release_root / "manifest.json",
    "elevator release manifest");
  bundle.current_metadata =
    read_bounded_managed_text(
    release_root / "current.json",
    "elevator release current metadata");

  const auto manifest_root = load_strict_json(
    bundle.manifest, "elevator release manifest");
  const auto current_root = load_strict_json(
    bundle.current_metadata, "elevator release current metadata");
  const auto configuration_root = load_strict_json(
    bundle.configuration, "elevator release configuration");
  if (!manifest_root.IsMap() || !current_root.IsMap() ||
    !configuration_root.IsMap())
  {
    throw std::runtime_error("elevator release metadata root is malformed");
  }
  const auto manifest_release_id =
    json_node_string(manifest_root["release_id"]);
  const auto current_release_id =
    json_node_string(current_root["release_id"]);
  const auto manifest_generation =
    json_node_uint(manifest_root["generation"]);
  const auto current_generation =
    json_node_uint(current_root["generation"]);
  const auto digest_algorithm =
    json_node_string(manifest_root["configuration_digest_algorithm"]);
  const auto recorded_digest =
    json_node_string(manifest_root["configuration_digest"]);
  const auto manifest_parent =
    json_node_string(manifest_root["parent_release_id"]);
  const auto current_parent =
    json_node_string(current_root["parent_release_id"]);
  const auto manifest_actor =
    json_node_string(manifest_root["actor_id"]);
  const auto current_actor =
    json_node_string(current_root["actor_id"]);
  const auto manifest_rollback =
    json_node_string(manifest_root["rollback_of"]);
  const auto current_rollback =
    json_node_string(current_root["rollback_of"]);
  const auto source_draft =
    json_node_string(manifest_root["source_draft_revision"]);
  const auto manifest_schema =
    json_node_uint(manifest_root["schema_version"]);
  const auto current_schema =
    json_node_uint(current_root["schema_version"]);
  const auto asset_published =
    json_node_bool(manifest_root["asset_published"]);
  const auto runtime_applied =
    json_node_bool(manifest_root["runtime_applied"]);
  const auto created_at =
    json_node_string(manifest_root["created_at"]);
  const auto published_at =
    json_node_string(current_root["published_at"]);
  if (!manifest_release_id || !current_release_id ||
    *manifest_release_id != expected_release_id ||
    *current_release_id != expected_release_id ||
    !manifest_generation || !current_generation ||
    *manifest_generation < 1U ||
    *current_generation != *manifest_generation ||
    !manifest_schema || *manifest_schema != 1U ||
    !current_schema || *current_schema != 1U ||
    !asset_published || !*asset_published ||
    !runtime_applied || *runtime_applied ||
    !created_at || created_at->empty() ||
    !published_at || published_at->empty())
  {
    throw std::runtime_error("elevator release metadata identity is inconsistent");
  }
  bundle.generation = *manifest_generation;
  if (manifest_parent != current_parent ||
    manifest_actor != current_actor ||
    manifest_rollback != current_rollback ||
    !manifest_actor ||
    (manifest_parent && !valid_release_id(*manifest_parent)) ||
    (manifest_rollback && !valid_release_id(*manifest_rollback)) ||
    (source_draft && !valid_draft_revision(*source_draft)) ||
    (source_draft.has_value() == manifest_rollback.has_value()) ||
    (bundle.generation == 1U && manifest_parent) ||
    (bundle.generation > 1U && !manifest_parent) ||
    bundle.validation !=
    "{\"valid_for_publish\":true,\"issues\":[]}\n")
  {
    throw std::runtime_error(
            "elevator release audit metadata or validation record is inconsistent");
  }
  if (expected_generation && bundle.generation != *expected_generation) {
    throw std::runtime_error(
            "elevator release generation disagrees with the current selector");
  }

  const auto calculated_digest = fixed_hex(
    fnv1a64(
      bundle.configuration + bundle.topology +
      bundle.internal_poses),
    16);
  ValidationResult identity_material;
  identity_material.configuration_yaml = bundle.configuration;
  identity_material.topology_yaml = bundle.topology;
  identity_material.internal_poses_yaml = bundle.internal_poses;
  if (!digest_algorithm || *digest_algorithm != "fnv1a64" ||
    !recorded_digest || *recorded_digest != calculated_digest ||
    release_id_for(bundle.generation, identity_material) != expected_release_id)
  {
    throw std::runtime_error(
            "elevator release content digest or release identity is inconsistent");
  }

  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>>
  configuration_bindings;
  const auto configured_elevators = configuration_root["elevators"];
  if (!configured_elevators || !configured_elevators.IsSequence()) {
    throw std::runtime_error("elevator release configuration has no binding catalog");
  }
  for (const auto & elevator : configured_elevators) {
    const auto floors = elevator["floors"];
    if (!floors || !floors.IsSequence()) {
      throw std::runtime_error("elevator release floor bindings are malformed");
    }
    for (const auto & floor : floors) {
      const auto floor_id = node_string(floor["floor_id"]);
      const auto map_id = node_string(floor["map_id"]);
      const auto asset_epoch =
        node_positive_uint(floor["map_asset_epoch"]);
      const auto asset_digest = node_string(floor["map_asset_digest"]);
      if (!floor_id || !map_id || !asset_epoch || !asset_digest ||
        !is_canonical_sha256_digest(*asset_digest))
      {
        throw std::runtime_error("elevator release map binding is incomplete");
      }
      const auto key = std::make_pair(*floor_id, *map_id);
      const auto inserted =
        configuration_bindings.emplace(
        key, std::make_pair(*asset_epoch, *asset_digest));
      if (!inserted.second &&
        inserted.first->second !=
        std::make_pair(*asset_epoch, *asset_digest))
      {
        throw std::runtime_error("elevator release map binding identities conflict");
      }
    }
  }

  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>>
  manifest_bindings;
  const auto recorded_bindings = manifest_root["map_bindings"];
  if (!recorded_bindings || !recorded_bindings.IsSequence()) {
    throw std::runtime_error("elevator release manifest map bindings are malformed");
  }
  for (const auto & binding : recorded_bindings) {
    const auto floor_id = node_string(binding["floor_id"]);
    const auto map_id = node_string(binding["map_id"]);
    const auto asset_epoch =
      node_positive_uint(binding["asset_epoch"]);
    const auto contract =
      node_string(binding["asset_digest_contract"]);
    const auto algorithm =
      node_string(binding["asset_digest_algorithm"]);
    const auto asset_digest = node_string(binding["asset_digest"]);
    if (!floor_id || !map_id || !asset_epoch ||
      !contract || *contract != "njrh-map-asset-bundle-v1" ||
      !algorithm || *algorithm != "sha256" ||
      !asset_digest || !is_canonical_sha256_digest(*asset_digest) ||
      !manifest_bindings.emplace(
        std::make_pair(*floor_id, *map_id),
        std::make_pair(*asset_epoch, *asset_digest)).second)
    {
      throw std::runtime_error(
              "elevator release manifest map binding is invalid or duplicated");
    }
  }
  if (manifest_bindings != configuration_bindings) {
    throw std::runtime_error(
            "elevator release manifest map bindings disagree with configuration");
  }

  const auto topology =
    robot_elevator_manager::parse_topology_yaml(bundle.topology);
  if (!topology.ok()) {
    throw std::runtime_error(
            "elevator release topology no longer passes validation");
  }
  return bundle;
}

bool mask_json_string_field(
  std::string & text,
  const std::string & field)
{
  const auto marker = "\"" + field + "\":";
  auto position = text.find(marker);
  if (position == std::string::npos) {
    return false;
  }
  position += marker.size();
  while (position < text.size() &&
    std::isspace(static_cast<unsigned char>(text[position])))
  {
    ++position;
  }
  if (position >= text.size() || text[position] != '"') {
    return false;
  }
  const auto value_begin = position;
  ++position;
  bool escaped = false;
  for (; position < text.size(); ++position) {
    const char character = text[position];
    if (!escaped && character == '"') {
      text.replace(
        value_begin, position - value_begin + 1U,
        "\"<publication-time>\"");
      return true;
    }
    if (!escaped && character == '\\') {
      escaped = true;
    } else {
      escaped = false;
    }
  }
  return false;
}

bool immutable_file_matches(
  const std::string & filename,
  const std::string & existing,
  const std::string & expected)
{
  if (existing == expected) {
    return true;
  }
  std::string normalized_existing = existing;
  std::string normalized_expected = expected;
  if (filename == "manifest.json") {
    return mask_json_string_field(normalized_existing, "created_at") &&
           mask_json_string_field(normalized_expected, "created_at") &&
           normalized_existing == normalized_expected;
  }
  if (filename == "current.json") {
    return mask_json_string_field(normalized_existing, "published_at") &&
           mask_json_string_field(normalized_expected, "published_at") &&
           normalized_existing == normalized_expected;
  }
  return false;
}

void write_immutable_directory(
  const fs::path & final_path,
  const std::map<std::string, std::string> & files)
{
  if (path_entry_exists(final_path.parent_path()) &&
    (fs::is_symlink(final_path.parent_path()) ||
    !fs::is_directory(final_path.parent_path())))
  {
    throw std::runtime_error(
            "immutable elevator asset store has an unexpected type: " +
            final_path.parent_path().string());
  }
  fs::create_directories(final_path.parent_path());
  if (fs::is_symlink(final_path.parent_path()) ||
    !fs::is_directory(final_path.parent_path()))
  {
    throw std::runtime_error(
            "immutable elevator asset store became unsafe: " +
            final_path.parent_path().string());
  }
  if (path_entry_exists(final_path)) {
    if (fs::is_symlink(final_path) || !fs::is_directory(final_path)) {
      throw std::runtime_error(
              "immutable elevator asset path has an unexpected type: " +
              final_path.string());
    }
    for (const auto & file : files) {
      const auto existing = final_path / file.first;
      if (fs::is_symlink(existing) || !fs::is_regular_file(existing) ||
        !immutable_file_matches(
          file.first,
          read_bounded_managed_text(
            existing, "immutable elevator asset " + file.first),
          file.second))
      {
        throw std::runtime_error(
                "immutable elevator asset already exists with different content: " +
                final_path.string());
      }
      sync_file(existing);
    }
    // A previous attempt can observe rename success but parent-fsync failure.
    // Re-sync the complete immutable bundle and both directory entries before
    // allowing a selector or draft pointer to reference it.
    sync_directory(final_path);
    sync_directory(final_path.parent_path());
    return;
  }
  const auto staging =
    final_path.parent_path() / (".stage-" + final_path.filename().string() + "-" + unique_suffix());
  fs::create_directories(staging);
  try {
    for (const auto & file : files) {
      durable_write(staging / file.first, file.second);
    }
    sync_directory(staging);
    fs::rename(staging, final_path);
    sync_directory(final_path.parent_path());
  } catch (...) {
    std::error_code ignored;
    fs::remove_all(staging, ignored);
    throw;
  }
}

struct ReleaseInventory
{
  std::vector<std::string> valid;
  std::vector<std::string> integrity_errors;
};

ReleaseInventory release_inventory(const fs::path & config_root)
{
  ReleaseInventory result;
  const auto root = config_root / "releases";
  if (path_entry_exists(root) &&
    (fs::is_symlink(root) || !path_is_within(root, config_root)))
  {
    throw std::runtime_error("elevator release store is a symbolic link or escapes its managed root");
  }
  if (!fs::exists(root) || !fs::is_directory(root)) {
    return result;
  }
  for (const auto & entry : fs::directory_iterator(root)) {
    const auto release_root = entry.path();
    const auto release_id = release_root.filename().string();
    if (!entry.is_symlink() && entry.is_directory() &&
      valid_release_id(release_id) &&
      path_is_within(release_root, root))
    {
      try {
        (void)load_verified_release_bundle(release_root, release_id);
        result.valid.push_back(release_id);
      } catch (const std::exception &) {
        result.integrity_errors.push_back(release_id);
      }
    }
  }
  std::sort(result.valid.begin(), result.valid.end());
  std::sort(result.integrity_errors.begin(), result.integrity_errors.end());
  return result;
}

std::string string_array_json(const std::vector<std::string> & values)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      out << ",";
    }
    out << json_string(values[index]);
  }
  out << "]";
  return out.str();
}

bool configuration_references_map(
  const YAML::Node & configuration,
  const std::string & floor_id,
  const std::string & map_id)
{
  const auto elevators = configuration["elevators"];
  if (!elevators || !elevators.IsSequence()) {
    return false;
  }
  for (const auto & elevator : elevators) {
    const auto floors = elevator["floors"];
    if (!floors || !floors.IsSequence()) {
      continue;
    }
    for (const auto & floor : floors) {
      if (!floor.IsMap()) {
        continue;
      }
      const auto configured_floor = floor["floor_id"];
      const auto configured_map = floor["map_id"];
      if (configured_floor && configured_floor.IsScalar() &&
        configured_map && configured_map.IsScalar() &&
        configured_floor.as<std::string>() == floor_id &&
        configured_map.as<std::string>() == map_id)
      {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

class ElevatorConfigurationModule::Implementation
{
public:
  Implementation(
    fs::path maps_root,
    ElevatorFloorAssetResolver floor_asset_resolver)
  : maps_root_(std::move(maps_root)),
    resolver_(std::move(floor_asset_resolver))
  {
    if (maps_root_.empty()) {
      throw std::invalid_argument("elevator configuration maps_root is empty");
    }
    if (!resolver_) {
      throw std::invalid_argument("elevator configuration floor asset resolver is empty");
    }
  }

  ElevatorConfigurationReply execute(const ElevatorConfigurationCommand & command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_elevator_manager::safe_asset_id(command.building_id)) {
      return make_reply(
        400, "INVALID_BUILDING_ID",
        "\"error\":\"building_id must be a bounded path-safe identifier\"");
    }
    if (command.actor_id.size() > 256U) {
      return make_reply(
        400, "INVALID_ACTOR_ID",
        "\"error\":\"actor_id is too long\"");
    }
    try {
      const auto config_root = checked_config_root(command.building_id, true);
      switch (command.type) {
        case ElevatorConfigurationCommandType::kSaveDraft:
          return save_draft(command, config_root);
        case ElevatorConfigurationCommandType::kPublish:
          return publish(command, config_root);
        case ElevatorConfigurationCommandType::kRollback:
          return rollback(command, config_root);
      }
    } catch (const YAML::Exception & error) {
      return make_reply(
        400, "INVALID_DOCUMENT",
        "\"error\":" + json_string(error.what()));
    } catch (const std::exception & error) {
      return make_reply(
        500, "ELEVATOR_CONFIG_IO_ERROR",
        "\"error\":" + json_string(error.what()));
    }
    return make_reply(500, "UNKNOWN_COMMAND");
  }

  ElevatorConfigurationReply query(const ElevatorConfigurationQuery & request) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_elevator_manager::safe_asset_id(request.building_id)) {
      return make_reply(
        400, "INVALID_BUILDING_ID",
        "\"error\":\"building_id must be a bounded path-safe identifier\"");
    }
    if (!request.release_id.empty() &&
      !valid_release_id(request.release_id))
    {
      return make_reply(
        400, "INVALID_RELEASE_ID",
        "\"error\":\"release_id must be a bounded path-safe identifier\"");
    }
    const bool map_reference_requested =
      !request.floor_id.empty() || !request.map_id.empty();
    if (map_reference_requested &&
      (request.floor_id.empty() || request.map_id.empty() ||
      !robot_elevator_manager::safe_asset_id(request.floor_id) ||
      !robot_elevator_manager::safe_asset_id(request.map_id)))
    {
      return make_reply(
        400, "INVALID_MAP_REFERENCE_QUERY",
        "\"error\":\"floor_id and map_id must both be bounded path-safe identifiers\"");
    }
    try {
      const auto config_root = checked_config_root(request.building_id, false);
      if (!fs::exists(config_root)) {
        if (map_reference_requested) {
          return make_reply(
            200, "MAP_REFERENCE_STATUS",
            "\"building_id\":" + json_string(request.building_id) +
            ",\"floor_id\":" + json_string(request.floor_id) +
            ",\"map_id\":" + json_string(request.map_id) +
            ",\"referenced\":false,\"release_id\":null");
        }
        return make_reply(
          200, "CONFIGURATION_EMPTY",
          "\"building_id\":" + json_string(request.building_id) +
          ",\"draft_revision\":null,\"current_release_id\":null,"
          "\"configuration_source\":null,\"configuration\":null,"
          "\"validation\":null,\"releases\":[],\"release_integrity_errors\":[]");
      }
      const auto draft = read_draft_pointer(config_root);
      const auto current = read_release_pointer(config_root);

      if (map_reference_requested) {
        bool referenced = false;
        if (current.exists) {
          const auto release_root =
            config_root / "releases" / current.release_id;
          if (!path_is_within(release_root, config_root / "releases"))
          {
            throw std::runtime_error(
                    "elevator current release directory is unsafe");
          }
          const auto bundle = load_verified_release_bundle(
            release_root, current.release_id, current.generation);
          referenced = configuration_references_map(
            load_strict_json(
              bundle.configuration, "elevator release configuration"),
            request.floor_id, request.map_id);
        }
        std::ostringstream fields;
        fields << "\"building_id\":" << json_string(request.building_id) << ","
               << "\"floor_id\":" << json_string(request.floor_id) << ","
               << "\"map_id\":" << json_string(request.map_id) << ","
               << "\"referenced\":" << (referenced ? "true" : "false") << ","
               << "\"release_id\":"
               << (current.exists ? json_string(current.release_id) : "null");
        return make_reply(200, "MAP_REFERENCE_STATUS", fields.str());
      }

      if (!request.release_id.empty()) {
        const auto release_root = config_root / "releases" / request.release_id;
        if (!managed_path(release_root, config_root) ||
          fs::is_symlink(release_root) ||
          !fs::is_directory(release_root))
        {
          return make_reply(
            404, "RELEASE_NOT_FOUND",
            "\"error\":\"elevator configuration release was not found\"");
        }
        const auto bundle =
          load_verified_release_bundle(release_root, request.release_id);
        const auto configuration = load_strict_json(
          bundle.configuration, "elevator release configuration");
        const auto validation = validate_document(
          request.building_id, configuration, resolver_);
        std::ostringstream fields;
        fields << "\"building_id\":" << json_string(request.building_id) << ","
               << "\"release_id\":" << json_string(request.release_id) << ","
               << "\"current\":"
               << ((current.exists && current.release_id == request.release_id) ? "true" : "false")
               << ",\"configuration\":" << yaml_node_json(configuration) << ","
               << "\"validation\":" << validation_json(validation)
               << ",\"manifest\":"
               << bundle.manifest;
        return make_reply(200, "RELEASE_FOUND", fields.str());
      }

      const auto inventory = release_inventory(config_root);
      YAML::Node configuration;
      ValidationResult validation;
      if (draft.exists) {
        const auto bundle = load_verified_draft_bundle(
          config_root, request.building_id, draft.revision);
        configuration = load_strict_json(
          bundle.configuration, "elevator draft configuration");
        validation = validate_document(
          request.building_id, configuration, resolver_);
      } else if (current.exists) {
        const auto release_root =
          config_root / "releases" / current.release_id;
        if (!path_is_within(release_root, config_root / "releases"))
        {
          throw std::runtime_error("elevator current release directory is unsafe");
        }
        const auto bundle = load_verified_release_bundle(
          release_root, current.release_id, current.generation);
        configuration = load_strict_json(
          bundle.configuration, "elevator release configuration");
        validation = validate_document(
          request.building_id, configuration, resolver_);
      }

      std::ostringstream fields;
      fields << "\"building_id\":" << json_string(request.building_id) << ","
             << "\"draft_revision\":"
             << (draft.exists ? json_string(draft.revision) : "null") << ","
             << "\"current_release_id\":"
             << (current.exists ? json_string(current.release_id) : "null") << ","
             << "\"generation\":" << (current.exists ? std::to_string(current.generation) : "0")
             << ",\"runtime_applied\":false,"
             << "\"configuration_source\":"
             << (draft.exists ? "\"draft\"" :
        (current.exists ? "\"current\"" : "null")) << ","
             << "\"configuration\":"
             << (configuration ? yaml_node_json(configuration) : "null") << ","
             << "\"validation\":"
             << (configuration ? validation_json(validation) : "null") << ","
             << "\"releases\":" << string_array_json(inventory.valid) << ","
             << "\"release_integrity_errors\":"
             << string_array_json(inventory.integrity_errors);
      return make_reply(200, "CONFIGURATION_FOUND", fields.str());
    } catch (const std::exception & error) {
      return make_reply(
        500, "ELEVATOR_CONFIG_IO_ERROR",
        "\"error\":" + json_string(error.what()));
    }
  }

private:
  static bool managed_path(
    const fs::path & path,
    const fs::path & config_root)
  {
    return path_is_within(path, config_root);
  }

  static void require_managed_path(
    const fs::path & path,
    const fs::path & config_root)
  {
    if (!managed_path(path, config_root)) {
      throw std::runtime_error(
              "elevator configuration storage path escapes its managed root");
    }
  }

  fs::path checked_config_root(
    const std::string & building_id,
    const bool create) const
  {
    if (create) {
      fs::create_directories(maps_root_);
    }
    const auto building_root = maps_root_ / building_id;
    const auto config_root = building_root / ".elevator_config";
    if (path_entry_exists(building_root) && fs::is_symlink(building_root)) {
      throw std::runtime_error("building asset root cannot be a symbolic link");
    }
    if (path_entry_exists(config_root) && fs::is_symlink(config_root)) {
      throw std::runtime_error("elevator configuration root cannot be a symbolic link");
    }
    if (create) {
      fs::create_directories(config_root);
    }
    for (const auto & store_name : {"drafts", "releases"}) {
      const auto store_root = config_root / store_name;
      if (path_entry_exists(store_root) &&
        (fs::is_symlink(store_root) || !fs::is_directory(store_root)))
      {
        throw std::runtime_error(
                std::string("elevator configuration ") + store_name +
                " store must be a real managed directory");
      }
    }
    if (!path_is_within(config_root, maps_root_)) {
      throw std::runtime_error("elevator configuration path escapes maps_root");
    }
    return config_root;
  }

  ElevatorConfigurationReply save_draft(
    const ElevatorConfigurationCommand & command,
    const fs::path & config_root)
  {
    if (command.document.empty() || command.document.size() > kMaximumDocumentBytes) {
      return make_reply(
        400, "INVALID_DOCUMENT",
        "\"error\":\"configuration document is empty or too large\"");
    }
    YAML::Node root;
    try {
      root = load_strict_json(
        command.document, "elevator configuration document");
    } catch (const std::exception & error) {
      return make_reply(
        400, "INVALID_DOCUMENT",
        "\"error\":" + json_string(error.what()));
    }
    if (!root || !root.IsMap()) {
      return make_reply(
        400, "INVALID_DOCUMENT",
        "\"error\":\"configuration document root must be an object\"");
    }
    const auto existing = read_draft_pointer(config_root);
    if (existing.exists &&
      (command.expected_draft_revision.empty() ||
      command.expected_draft_revision != existing.revision))
    {
      return make_reply(
        409, "DRAFT_REVISION_CONFLICT",
        "\"expected_revision\":" +
        (command.expected_draft_revision.empty() ?
        std::string("null") : json_string(command.expected_draft_revision)) +
        ",\"actual_revision\":" + json_string(existing.revision));
    }
    if (!existing.exists && !command.expected_draft_revision.empty()) {
      return make_reply(
        409, "DRAFT_REVISION_CONFLICT",
        "\"expected_revision\":" + json_string(command.expected_draft_revision) +
        ",\"actual_revision\":null");
    }

    const auto canonical_document = canonical_configuration_json(root);
    const auto validation = validate_document(
      command.building_id, root, resolver_);
    const auto persisted_document = validation.ok() ?
      draft_configuration_with_map_bindings(root, validation.bindings) :
      canonical_document;
    const auto validation_record = validation_json(validation);
    const auto revision = draft_revision_for(
      command.building_id, persisted_document, validation_record);
    const auto draft_root = config_root / "drafts" / revision;
    require_managed_path(draft_root, config_root);
    write_immutable_directory(
      draft_root,
      {
        {"configuration.yaml", persisted_document},
        {"validation.json", validation_record},
      });
    atomic_replace(
      config_root / "draft.json",
      draft_pointer_json(revision, command.actor_id));

    std::ostringstream fields;
    fields << "\"building_id\":" << json_string(command.building_id) << ","
           << "\"draft_revision\":" << json_string(revision) << ","
           << "\"valid_for_publish\":" << (validation.ok() ? "true" : "false") << ","
           << "\"issues\":" << issues_json(validation.issues) << ","
           << "\"runtime_applied\":false";
    return make_reply(200, "DRAFT_SAVED", fields.str());
  }

  ElevatorConfigurationReply publish(
    const ElevatorConfigurationCommand & command,
    const fs::path & config_root)
  {
    const auto draft = read_draft_pointer(config_root);
    if (!draft.exists) {
      return make_reply(
        404, "DRAFT_NOT_FOUND",
        "\"error\":\"save an elevator configuration draft before publishing\"");
    }
    if (command.expected_draft_revision.empty() ||
      command.expected_draft_revision != draft.revision)
    {
      return make_reply(
        409, "DRAFT_REVISION_CONFLICT",
        "\"expected_revision\":" +
        (command.expected_draft_revision.empty() ?
        std::string("null") : json_string(command.expected_draft_revision)) +
        ",\"actual_revision\":" + json_string(draft.revision));
    }
    const auto current = read_release_pointer(config_root);
    if ((current.exists && command.expected_release_id != current.release_id) ||
      (!current.exists && !command.expected_release_id.empty()))
    {
      return make_reply(
        409, "RELEASE_REVISION_CONFLICT",
        "\"expected_release_id\":" +
        (command.expected_release_id.empty() ?
        std::string("null") : json_string(command.expected_release_id)) +
        ",\"actual_release_id\":" +
        (current.exists ? json_string(current.release_id) : "null"));
    }
    if (current.exists) {
      const auto current_root =
        config_root / "releases" / current.release_id;
      (void)load_verified_release_bundle(
        current_root, current.release_id, current.generation);
    }
    const auto draft_bundle = load_verified_draft_bundle(
      config_root, command.building_id, draft.revision);
    if (draft_bundle.validation !=
      "{\"valid_for_publish\":true,\"issues\":[]}\n")
    {
      return make_reply(
        422, "DRAFT_REVIEW_REQUIRED",
        "\"valid_for_publish\":false,\"review_required\":true,"
        "\"saved_validation\":" + draft_bundle.validation +
        ",\"runtime_applied\":false");
    }
    const auto document = load_strict_json(
      draft_bundle.configuration, "elevator draft configuration");
    auto validation = validate_document(
      command.building_id, document, resolver_);
    if (!validation.ok()) {
      return make_reply(
        422, "CONFIGURATION_INVALID",
        "\"valid_for_publish\":false,\"issues\":" +
        issues_json(validation.issues) + ",\"runtime_applied\":false");
    }
    // configuration.yaml is the reviewed commissioning document. Runtime
    // topology and internal poses are separately generated from validated
    // records, so retaining unknown extension fields here is both auditable
    // and forward-compatible.
    validation.configuration_yaml = draft_bundle.configuration;
    return commit_release(
      command, config_root, current, validation, draft.revision, "");
  }

  ElevatorConfigurationReply rollback(
    const ElevatorConfigurationCommand & command,
    const fs::path & config_root)
  {
    if (!valid_release_id(command.release_id)) {
      return make_reply(
        400, "INVALID_RELEASE_ID",
        "\"error\":\"release_id must be a bounded path-safe identifier\"");
    }
    const auto current = read_release_pointer(config_root);
    if (!current.exists) {
      return make_reply(
        409, "NO_CURRENT_RELEASE",
        "\"error\":\"cannot rollback before the first published release\"");
    }
    if (command.expected_release_id.empty() ||
      command.expected_release_id != current.release_id)
    {
      return make_reply(
        409, "RELEASE_REVISION_CONFLICT",
        "\"expected_release_id\":" +
        (command.expected_release_id.empty() ?
        std::string("null") : json_string(command.expected_release_id)) +
        ",\"actual_release_id\":" + json_string(current.release_id));
    }
    const auto current_root =
      config_root / "releases" / current.release_id;
    (void)load_verified_release_bundle(
      current_root, current.release_id, current.generation);
    const auto target_root = config_root / "releases" / command.release_id;
    if (!managed_path(target_root, config_root) ||
      fs::is_symlink(target_root) ||
      !fs::is_directory(target_root))
    {
      return make_reply(
        404, "RELEASE_NOT_FOUND",
        "\"error\":\"rollback target release was not found\"");
    }
    const auto target_bundle =
      load_verified_release_bundle(target_root, command.release_id);
    const auto document = load_strict_json(
      target_bundle.configuration, "elevator release configuration");
    auto validation = validate_document(
      command.building_id, document, resolver_);
    if (!validation.ok()) {
      return make_reply(
        422, "ROLLBACK_TARGET_INVALID",
        "\"valid_for_publish\":false,\"issues\":" +
        issues_json(validation.issues) + ",\"runtime_applied\":false");
    }
    validation.configuration_yaml = target_bundle.configuration;
    return commit_release(
      command, config_root, current, validation, "", command.release_id);
  }

  ElevatorConfigurationReply commit_release(
    const ElevatorConfigurationCommand & command,
    const fs::path & config_root,
    const ReleasePointer & current,
    const ValidationResult & validation,
    const std::string & source_draft_revision,
    const std::string & rollback_of)
  {
    const auto generation = current.generation + 1U;
    const auto release_id = release_id_for(generation, validation);
    const auto release_root = config_root / "releases" / release_id;
    require_managed_path(release_root, config_root);
    const bool release_existed_before = path_entry_exists(release_root);
    const auto parent_release_id = current.exists ? current.release_id : "";
    const auto manifest = release_manifest_json(
      release_id, parent_release_id, source_draft_revision, generation,
      command.actor_id, rollback_of, validation);
    const auto current_metadata = current_pointer_json(
      release_id, parent_release_id, generation, command.actor_id, rollback_of);
    write_immutable_directory(
      release_root,
      {
        {"configuration.yaml", validation.configuration_yaml},
        {"elevators.yaml", validation.topology_yaml},
        {"elevator_internal_poses.yaml", validation.internal_poses_yaml},
        {"validation.json", validation_json(validation)},
        {"manifest.json", manifest},
        {"current.json", current_metadata},
      });

    try {
      (void)load_verified_release_bundle(
        release_root, release_id, generation);
    } catch (...) {
      if (!release_existed_before) {
        std::error_code ignored;
        fs::remove_all(release_root, ignored);
      }
      throw;
    }

    const auto building_root = config_root.parent_path();
    const auto topology_projection = building_root / "elevators.yaml";
    const auto internal_projection = building_root / "elevator_internal_poses.yaml";
#ifndef _WIN32
    install_relative_symlink(
      topology_projection,
      fs::path(".elevator_config") / "current" / "elevators.yaml",
      false);
    install_relative_symlink(
      internal_projection,
      fs::path(".elevator_config") / "current" /
      "elevator_internal_poses.yaml",
      false);
    install_relative_symlink(
      config_root / "current.json",
      fs::path("current") / "current.json",
      false);
    try {
      switch_current_release_link(config_root, release_id);
    } catch (const std::exception & error) {
      bool pointer_committed = false;
      try {
        const auto observed = read_release_pointer(config_root);
        pointer_committed =
          observed.exists && observed.release_id == release_id &&
          observed.generation == generation;
      } catch (...) {
        pointer_committed = false;
      }
      // An unselected immutable release is intentionally retained. A retry
      // with the same generation/content can verify and reuse it.
      if (pointer_committed) {
        throw std::runtime_error(
                "elevator release selector is visible but its durability is "
                "uncertain; reconcile with GET before retrying: " +
                std::string(error.what()));
      }
      throw;
    }
#else
    const auto old_topology = read_optional_text_file(topology_projection);
    const auto old_internal = read_optional_text_file(internal_projection);
    const bool had_topology = fs::exists(topology_projection);
    const bool had_internal = fs::exists(internal_projection);
    try {
      atomic_replace(topology_projection, validation.topology_yaml);
      atomic_replace(internal_projection, validation.internal_poses_yaml);
      atomic_replace(config_root / "current.json", current_metadata);
    } catch (...) {
      bool pointer_committed = false;
      try {
        const auto observed = read_release_pointer(config_root);
        pointer_committed =
          observed.exists && observed.release_id == release_id &&
          observed.generation == generation;
      } catch (...) {
        pointer_committed = false;
      }
      try {
        if (pointer_committed) {
          atomic_replace(topology_projection, validation.topology_yaml);
          atomic_replace(internal_projection, validation.internal_poses_yaml);
        } else {
          if (had_topology) {
            atomic_replace(topology_projection, old_topology);
          } else {
            std::error_code ignored;
            fs::remove(topology_projection, ignored);
          }
          if (had_internal) {
            atomic_replace(internal_projection, old_internal);
          } else {
            std::error_code ignored;
            fs::remove(internal_projection, ignored);
          }
        }
      } catch (...) {
        // Keep the original durability error; query() will expose the authoritative pointer.
      }
      if (!pointer_committed) {
        std::error_code ignored;
        fs::remove_all(release_root, ignored);
      }
      throw;
    }
#endif

    std::ostringstream fields;
    fields << "\"building_id\":" << json_string(command.building_id) << ","
           << "\"release_id\":" << json_string(release_id) << ","
           << "\"parent_release_id\":"
           << (parent_release_id.empty() ? "null" : json_string(parent_release_id)) << ","
           << "\"generation\":" << generation << ","
           << "\"rollback_of\":"
           << (rollback_of.empty() ? "null" : json_string(rollback_of)) << ","
           << "\"asset_published\":true,\"runtime_applied\":false";
    return make_reply(
      201,
      rollback_of.empty() ? "CONFIGURATION_PUBLISHED" : "CONFIGURATION_ROLLED_BACK",
      fields.str());
  }

  fs::path maps_root_;
  ElevatorFloorAssetResolver resolver_;
  mutable std::mutex mutex_;
};

ElevatorConfigurationModule::ElevatorConfigurationModule(
  fs::path maps_root,
  ElevatorFloorAssetResolver floor_asset_resolver)
: implementation_(
    std::make_unique<Implementation>(
      std::move(maps_root), std::move(floor_asset_resolver)))
{
}

ElevatorConfigurationModule::~ElevatorConfigurationModule() = default;

ElevatorConfigurationModule::ElevatorConfigurationModule(
  ElevatorConfigurationModule &&) noexcept = default;

ElevatorConfigurationModule & ElevatorConfigurationModule::operator=(
  ElevatorConfigurationModule &&) noexcept = default;

ElevatorConfigurationReply ElevatorConfigurationModule::execute(
  const ElevatorConfigurationCommand & command)
{
  return implementation_->execute(command);
}

ElevatorConfigurationReply ElevatorConfigurationModule::query(
  const ElevatorConfigurationQuery & request) const
{
  return implementation_->query(request);
}

bool is_elevator_internal_pose_type(const std::string & type) noexcept
{
  std::string normalized;
  normalized.reserve(type.size());
  std::transform(
    type.begin(), type.end(), std::back_inserter(normalized),
    [](const unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return normalized == "elevator_internal";
}

bool is_reserved_elevator_pose_id(const std::string & pose_id) noexcept
{
  return pose_id.rfind("eip_", 0U) == 0U;
}

}  // namespace robot_api_server
