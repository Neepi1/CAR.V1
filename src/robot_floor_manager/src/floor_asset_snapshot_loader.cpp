#include "robot_floor_manager/floor_asset_snapshot_loader.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace robot_floor_manager
{
namespace
{
namespace fs = std::filesystem;

constexpr char kManifestSchema[] = "njrh.map_manifest.v2";
constexpr char kDigestAlgorithm[] = "sha256";
constexpr char kDigestContract[] = "njrh-map-asset-bundle-v1";
constexpr std::uintmax_t kMaximumManifestBytes = 1024U * 1024U;
constexpr std::uintmax_t kMaximumPosesBytes = 64U * 1024U * 1024U;
constexpr std::uintmax_t kMaximumRoleAssetBytes =
  512ULL * 1024ULL * 1024ULL;
constexpr std::uintmax_t kMaximumBundleBytes =
  1024ULL * 1024ULL * 1024ULL;

class LoadFailure : public std::runtime_error
{
public:
  LoadFailure(
    const FloorAssetSnapshotError error,
    const std::string & message)
  : std::runtime_error(message), error_(error)
  {
  }

  FloorAssetSnapshotError error() const noexcept
  {
    return error_;
  }

private:
  FloorAssetSnapshotError error_;
};

[[noreturn]] void fail(
  const FloorAssetSnapshotError error,
  const std::string & message)
{
  throw LoadFailure(error, message);
}

bool safe_asset_id(const std::string_view value)
{
  if (value.empty() || value == "." || value.size() > 128U ||
    value.find("..") != std::string_view::npos ||
    value.find('/') != std::string_view::npos ||
    value.find('\\') != std::string_view::npos)
  {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(), [](const char character) {
      return (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') ||
      character == '-' || character == '_' || character == '.';
    });
}

enum class JsonKind
{
  kNull,
  kBoolean,
  kNumber,
  kString,
  kObject,
  kArray,
};

struct JsonValue
{
  JsonKind kind{JsonKind::kNull};
  std::string scalar;
  std::map<std::string, JsonValue> object;
  std::vector<JsonValue> array;
};

class StrictJsonParser
{
public:
  explicit StrictJsonParser(const std::string_view input)
  : input_(input)
  {
  }

  JsonValue parse()
  {
    skip_whitespace();
    auto value = parse_value(0U);
    skip_whitespace();
    if (position_ != input_.size()) {
      parse_error("trailing content");
    }
    return value;
  }

private:
  static bool decimal_digit(const char value)
  {
    return value >= '0' && value <= '9';
  }

  static int hex_digit(const char value)
  {
    if (value >= '0' && value <= '9') {
      return value - '0';
    }
    if (value >= 'a' && value <= 'f') {
      return value - 'a' + 10;
    }
    if (value >= 'A' && value <= 'F') {
      return value - 'A' + 10;
    }
    return -1;
  }

  [[noreturn]] void parse_error(const std::string & reason) const
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest is not strict JSON: " + reason);
  }

  void skip_whitespace()
  {
    while (position_ < input_.size()) {
      const auto value = input_[position_];
      if (value != ' ' && value != '\t' && value != '\r' && value != '\n') {
        break;
      }
      ++position_;
    }
  }

  bool consume(const char expected)
  {
    if (position_ < input_.size() && input_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  void expect(const char expected)
  {
    if (!consume(expected)) {
      parse_error(std::string{"expected '"} + expected + "'");
    }
  }

  static void append_utf8(std::string & output, const std::uint32_t codepoint)
  {
    if (codepoint <= 0x7fU) {
      output.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7ffU) {
      output.push_back(static_cast<char>(0xc0U | (codepoint >> 6U)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    } else if (codepoint <= 0xffffU) {
      output.push_back(static_cast<char>(0xe0U | (codepoint >> 12U)));
      output.push_back(
        static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    } else {
      output.push_back(static_cast<char>(0xf0U | (codepoint >> 18U)));
      output.push_back(
        static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3fU)));
      output.push_back(
        static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
      output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
    }
  }

  std::uint32_t parse_hex_quad()
  {
    if (input_.size() - position_ < 4U) {
      parse_error("incomplete unicode escape");
    }
    std::uint32_t value = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const auto digit = hex_digit(input_[position_++]);
      if (digit < 0) {
        parse_error("invalid unicode escape");
      }
      value = (value << 4U) | static_cast<std::uint32_t>(digit);
    }
    return value;
  }

  std::string parse_string()
  {
    expect('"');
    std::string result;
    while (position_ < input_.size()) {
      const unsigned char value =
        static_cast<unsigned char>(input_[position_++]);
      if (value == '"') {
        return result;
      }
      if (value < 0x20U) {
        parse_error("unescaped control character");
      }
      if (value != '\\') {
        result.push_back(static_cast<char>(value));
        continue;
      }
      if (position_ >= input_.size()) {
        parse_error("incomplete string escape");
      }
      const auto escaped = input_[position_++];
      switch (escaped) {
        case '"':
        case '\\':
        case '/':
          result.push_back(escaped);
          break;
        case 'b':
          result.push_back('\b');
          break;
        case 'f':
          result.push_back('\f');
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        case 'u':
          {
            auto codepoint = parse_hex_quad();
            if (codepoint >= 0xd800U && codepoint <= 0xdbffU) {
              if (input_.size() - position_ < 6U ||
                input_[position_] != '\\' || input_[position_ + 1U] != 'u')
              {
                parse_error("unpaired high surrogate");
              }
              position_ += 2U;
              const auto low = parse_hex_quad();
              if (low < 0xdc00U || low > 0xdfffU) {
                parse_error("invalid low surrogate");
              }
              codepoint =
                0x10000U + ((codepoint - 0xd800U) << 10U) +
                (low - 0xdc00U);
            } else if (codepoint >= 0xdc00U && codepoint <= 0xdfffU) {
              parse_error("unpaired low surrogate");
            }
            append_utf8(result, codepoint);
            break;
          }
        default:
          parse_error("invalid string escape");
      }
    }
    parse_error("unterminated string");
  }

  std::string parse_number()
  {
    const auto start = position_;
    consume('-');
    if (position_ >= input_.size()) {
      parse_error("incomplete number");
    }
    if (consume('0')) {
      if (position_ < input_.size() && decimal_digit(input_[position_])) {
        parse_error("leading zero in number");
      }
    } else {
      if (input_[position_] < '1' || input_[position_] > '9') {
        parse_error("invalid number");
      }
      while (position_ < input_.size() && decimal_digit(input_[position_])) {
        ++position_;
      }
    }
    if (consume('.')) {
      if (position_ >= input_.size() || !decimal_digit(input_[position_])) {
        parse_error("incomplete fraction");
      }
      while (position_ < input_.size() && decimal_digit(input_[position_])) {
        ++position_;
      }
    }
    if (position_ < input_.size() &&
      (input_[position_] == 'e' || input_[position_] == 'E'))
    {
      ++position_;
      if (position_ < input_.size() &&
        (input_[position_] == '+' || input_[position_] == '-'))
      {
        ++position_;
      }
      if (position_ >= input_.size() || !decimal_digit(input_[position_])) {
        parse_error("incomplete exponent");
      }
      while (position_ < input_.size() && decimal_digit(input_[position_])) {
        ++position_;
      }
    }
    return std::string(input_.substr(start, position_ - start));
  }

  void parse_literal(const std::string_view literal)
  {
    if (input_.substr(position_, literal.size()) != literal) {
      parse_error("invalid literal");
    }
    position_ += literal.size();
  }

  JsonValue parse_object(const std::size_t depth)
  {
    expect('{');
    skip_whitespace();
    JsonValue value;
    value.kind = JsonKind::kObject;
    if (consume('}')) {
      return value;
    }
    while (true) {
      skip_whitespace();
      if (position_ >= input_.size() || input_[position_] != '"') {
        parse_error("object key is not a string");
      }
      auto key = parse_string();
      if (value.object.find(key) != value.object.end()) {
        parse_error("duplicate object key '" + key + "'");
      }
      skip_whitespace();
      expect(':');
      skip_whitespace();
      value.object.emplace(std::move(key), parse_value(depth + 1U));
      skip_whitespace();
      if (consume('}')) {
        return value;
      }
      expect(',');
      skip_whitespace();
    }
  }

  JsonValue parse_array(const std::size_t depth)
  {
    expect('[');
    skip_whitespace();
    JsonValue value;
    value.kind = JsonKind::kArray;
    if (consume(']')) {
      return value;
    }
    while (true) {
      value.array.push_back(parse_value(depth + 1U));
      skip_whitespace();
      if (consume(']')) {
        return value;
      }
      expect(',');
      skip_whitespace();
    }
  }

  JsonValue parse_value(const std::size_t depth)
  {
    constexpr std::size_t kMaximumDepth = 64U;
    if (depth > kMaximumDepth || position_ >= input_.size()) {
      parse_error("invalid nesting or missing value");
    }
    switch (input_[position_]) {
      case '{':
        return parse_object(depth);
      case '[':
        return parse_array(depth);
      case '"':
        return {JsonKind::kString, parse_string(), {}, {}};
      case 't':
        parse_literal("true");
        return {JsonKind::kBoolean, "true", {}, {}};
      case 'f':
        parse_literal("false");
        return {JsonKind::kBoolean, "false", {}, {}};
      case 'n':
        parse_literal("null");
        return {};
      default:
        if (input_[position_] == '-' || decimal_digit(input_[position_])) {
          return {JsonKind::kNumber, parse_number(), {}, {}};
        }
        parse_error("unknown value");
    }
  }

  std::string_view input_;
  std::size_t position_{0U};
};

std::size_t count_key_recursively(
  const JsonValue & value,
  const std::string & key)
{
  std::size_t count = 0U;
  if (value.kind == JsonKind::kObject) {
    for (const auto & [candidate, child] : value.object) {
      if (candidate == key) {
        ++count;
      }
      count += count_key_recursively(child, key);
    }
  } else if (value.kind == JsonKind::kArray) {
    for (const auto & child : value.array) {
      count += count_key_recursively(child, key);
    }
  }
  return count;
}

const JsonValue & required_root_field(
  const JsonValue & root,
  const std::string & name,
  const JsonKind kind)
{
  if (root.kind != JsonKind::kObject) {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest root must be an object");
  }
  if (count_key_recursively(root, name) != 1U) {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest identity field '" + name +
      "' must occur exactly once");
  }
  const auto field = root.object.find(name);
  if (field == root.object.end() || field->second.kind != kind) {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest root field '" + name + "' has the wrong type");
  }
  return field->second;
}

std::uint64_t canonical_positive_epoch(const JsonValue & value)
{
  const auto & text = value.scalar;
  if (text.empty() || text.front() == '-' ||
    (text.size() > 1U && text.front() == '0') ||
    !std::all_of(
      text.begin(), text.end(), [](const char character) {
        return character >= '0' && character <= '9';
      }))
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest asset_epoch is not a canonical positive uint64");
  }
  std::uint64_t epoch = 0U;
  const auto parsed =
    std::from_chars(text.data(), text.data() + text.size(), epoch);
  if (parsed.ec != std::errc{} ||
    parsed.ptr != text.data() + text.size() || epoch == 0U)
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest asset_epoch is not a canonical positive uint64");
  }
  return epoch;
}

void validate_manifest(
  const std::string & content,
  const FloorAssetSnapshotRequest & request,
  const std::string & discovered_safe_map_name)
{
  const auto root = StrictJsonParser(content).parse();
  const auto & schema =
    required_root_field(root, "schema", JsonKind::kString).scalar;
  const auto epoch = canonical_positive_epoch(
    required_root_field(root, "asset_epoch", JsonKind::kNumber));
  const auto & algorithm =
    required_root_field(
    root, "asset_digest_algorithm", JsonKind::kString).scalar;
  const auto & contract =
    required_root_field(
    root, "asset_digest_contract", JsonKind::kString).scalar;
  const auto & digest =
    required_root_field(root, "asset_digest", JsonKind::kString).scalar;
  const auto & building_id =
    required_root_field(root, "building_id", JsonKind::kString).scalar;
  const auto & floor_id =
    required_root_field(root, "floor_id", JsonKind::kString).scalar;
  const auto & map_id =
    required_root_field(root, "map_id", JsonKind::kString).scalar;
  const auto & safe_map_name =
    required_root_field(root, "safe_map_name", JsonKind::kString).scalar;

  if (schema != kManifestSchema || algorithm != kDigestAlgorithm ||
    contract != kDigestContract ||
    !robot_map_asset_identity::is_canonical_sha256_digest(digest))
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest v2 digest contract is invalid");
  }
  if (epoch != request.expected_asset_epoch ||
    digest != request.expected_asset_digest ||
    building_id != request.building_id ||
    floor_id != request.floor_id || map_id != request.map_id)
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest identity does not match the requested authoritative tuple");
  }
  if (!safe_asset_id(safe_map_name) ||
    safe_map_name != discovered_safe_map_name)
  {
    fail(
      FloorAssetSnapshotError::kInvalidManifest,
      "map manifest safe_map_name does not match the unique map assets");
  }
}

struct RoleSpec
{
  const char * logical_name;
  fs::path relative_path;
};

std::vector<RoleSpec> digest_roles(const std::string & safe_map_name)
{
  return {
    {"nav_map_yaml", fs::path("nav") / (safe_map_name + ".yaml")},
    {"nav_map_pgm", fs::path("nav") / (safe_map_name + ".pgm")},
    {"localizer_map_png",
      fs::path("localizer") / (safe_map_name + ".png")},
    {"localizer_params_yaml",
      fs::path("localizer") / (safe_map_name + ".yaml")},
    {"keepout_mask_yaml", fs::path("filters") / "keepout_mask.yaml"},
    {"keepout_mask_pgm", fs::path("filters") / "keepout_mask.pgm"},
    {"speed_mask_yaml", fs::path("filters") / "speed_mask.yaml"},
    {"speed_mask_pgm", fs::path("filters") / "speed_mask.pgm"},
    {"binary_mask_yaml", fs::path("filters") / "binary_mask.yaml"},
    {"binary_mask_pgm", fs::path("filters") / "binary_mask.pgm"},
    {"asset_report_json", fs::path("reports") / "asset_report.json"},
  };
}

std::vector<std::size_t> canonical_role_order(
  const std::vector<RoleSpec> & roles)
{
  std::vector<std::size_t> order;
  order.reserve(roles.size());
  for (std::size_t index = 0U; index < roles.size(); ++index) {
    order.push_back(index);
  }
  std::sort(
    order.begin(), order.end(),
    [&roles](const std::size_t left, const std::size_t right) {
      return std::string_view(roles[left].logical_name) <
             std::string_view(roles[right].logical_name);
    });
  return order;
}

void require_registry_identity(
  const FloorAssetSnapshotRequest & request)
{
  try {
    robot_map_asset_identity::PersistentAssetEpochRegistry registry(
      request.maps_root);
    const auto identity = registry.lookup(
        {
          request.building_id,
          request.floor_id,
          request.map_id,
        });
    if (!identity) {
      fail(
        FloorAssetSnapshotError::kIdentityNotFound,
        "map asset identity is not present in the authoritative registry");
    }
    if (identity->asset_epoch != request.expected_asset_epoch ||
      identity->asset_digest != request.expected_asset_digest)
    {
      fail(
        FloorAssetSnapshotError::kIdentityMismatch,
        "requested map asset epoch/digest differs from the authoritative registry");
    }
  } catch (const LoadFailure &) {
    throw;
  } catch (const std::exception & error) {
    fail(
      FloorAssetSnapshotError::kRegistryUnavailable,
      "authoritative map asset registry lookup failed: " +
      std::string(error.what()));
  }
}

std::string unique_stem(
  const std::vector<std::string> & names,
  const std::string & extension,
  const std::string & description)
{
  std::vector<std::string> matches;
  for (const auto & name : names) {
    if (name.size() > extension.size() &&
      name.compare(
        name.size() - extension.size(), extension.size(), extension) == 0)
    {
      matches.push_back(name.substr(0U, name.size() - extension.size()));
    }
  }
  if (matches.size() != 1U || !safe_asset_id(matches.front())) {
    fail(
      FloorAssetSnapshotError::kInvalidLayout,
      description + " must contain exactly one safe " + extension + " file");
  }
  return matches.front();
}

#ifndef _WIN32

class UniqueFd
{
public:
  explicit UniqueFd(const int descriptor = -1) noexcept
  : descriptor_(descriptor)
  {
  }

  ~UniqueFd()
  {
    if (descriptor_ >= 0) {
      (void)::close(descriptor_);
    }
  }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : descriptor_(other.descriptor_)
  {
    other.descriptor_ = -1;
  }

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      if (descriptor_ >= 0) {
        (void)::close(descriptor_);
      }
      descriptor_ = other.descriptor_;
      other.descriptor_ = -1;
    }
    return *this;
  }

  int get() const noexcept
  {
    return descriptor_;
  }

private:
  int descriptor_;
};

bool same_timestamp(const timespec & left, const timespec & right)
{
  return left.tv_sec == right.tv_sec && left.tv_nsec == right.tv_nsec;
}

bool same_file_identity(const struct stat & left, const struct stat & right)
{
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino &&
         left.st_mode == right.st_mode && left.st_nlink == right.st_nlink &&
         left.st_size == right.st_size &&
         same_timestamp(left.st_mtim, right.st_mtim) &&
         same_timestamp(left.st_ctim, right.st_ctim);
}

std::string errno_message(
  const std::string & operation,
  const fs::path & path)
{
  return operation + " failed for " + path.string() + ": " +
         std::strerror(errno);
}

struct OpenDirectory
{
  UniqueFd descriptor;
  struct stat initial {};
  int parent_descriptor{-1};
  std::string name;
  fs::path path;
  bool is_root{false};
};

OpenDirectory open_root_directory(const fs::path & path)
{
  struct stat path_status {};
  if (::lstat(path.string().c_str(), &path_status) != 0) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      errno_message("inspect maps_root", path));
  }
  if (!S_ISDIR(path_status.st_mode) || S_ISLNK(path_status.st_mode)) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      "maps_root must be a real directory: " + path.string());
  }
  UniqueFd descriptor(
    ::open(
      path.string().c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC |
      O_NOFOLLOW));
  if (descriptor.get() < 0) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      errno_message("open maps_root", path));
  }
  struct stat opened {};
  if (::fstat(descriptor.get(), &opened) != 0 ||
    !same_file_identity(opened, path_status))
  {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "maps_root changed while it was opened");
  }
  return {
    std::move(descriptor),
    opened,
    -1,
    {},
    path,
    true,
  };
}

OpenDirectory open_child_directory(
  const OpenDirectory & parent,
  const std::string & name,
  const fs::path & path)
{
  struct stat path_status {};
  if (::fstatat(
      parent.descriptor.get(), name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    const auto code =
      errno == ENOENT ? FloorAssetSnapshotError::kInvalidLayout :
      FloorAssetSnapshotError::kUnsafePath;
    fail(code, errno_message("inspect map bundle directory", path));
  }
  if (!S_ISDIR(path_status.st_mode) || S_ISLNK(path_status.st_mode)) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      "map bundle path component must be a real directory: " + path.string());
  }
  UniqueFd descriptor(
    ::openat(
      parent.descriptor.get(), name.c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (descriptor.get() < 0) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      errno_message("open map bundle directory", path));
  }
  struct stat opened {};
  if (::fstat(descriptor.get(), &opened) != 0 ||
    !same_file_identity(opened, path_status))
  {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "map bundle directory changed while it was opened: " + path.string());
  }
  return {
    std::move(descriptor),
    opened,
    parent.descriptor.get(),
    name,
    path,
    false,
  };
}

void revalidate_directory(const OpenDirectory & directory)
{
  struct stat opened {};
  struct stat path_status {};
  const auto path_result = directory.is_root ?
    ::lstat(directory.path.string().c_str(), &path_status) :
    ::fstatat(
    directory.parent_descriptor, directory.name.c_str(), &path_status,
    AT_SYMLINK_NOFOLLOW);
  if (::fstat(directory.descriptor.get(), &opened) != 0 ||
    path_result != 0 ||
    !S_ISDIR(path_status.st_mode) ||
    !same_file_identity(directory.initial, opened) ||
    !same_file_identity(opened, path_status))
  {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "map bundle directory changed during snapshot: " +
      directory.path.string());
  }
}

std::vector<std::string> list_directory(
  const OpenDirectory & directory,
  const std::string & description)
{
  const auto duplicate = ::dup(directory.descriptor.get());
  if (duplicate < 0) {
    fail(
      FloorAssetSnapshotError::kIoError,
      errno_message("duplicate directory descriptor", directory.path));
  }
  DIR * stream = ::fdopendir(duplicate);
  if (stream == nullptr) {
    (void)::close(duplicate);
    fail(
      FloorAssetSnapshotError::kIoError,
      errno_message("open directory stream", directory.path));
  }

  std::vector<std::string> names;
  errno = 0;
  while (const auto * entry = ::readdir(stream)) {
    const std::string name(entry->d_name);
    if (name == "." || name == "..") {
      continue;
    }
    struct stat status {};
    if (::fstatat(
        directory.descriptor.get(), name.c_str(), &status,
        AT_SYMLINK_NOFOLLOW) != 0)
    {
      const auto message =
        errno_message("inspect " + description + " entry", directory.path / name);
      (void)::closedir(stream);
      fail(FloorAssetSnapshotError::kAssetChanged, message);
    }
    if (S_ISLNK(status.st_mode)) {
      (void)::closedir(stream);
      fail(
        FloorAssetSnapshotError::kUnsafeAsset,
        description + " contains a symlink: " +
        (directory.path / name).string());
    }
    names.push_back(name);
    errno = 0;
  }
  if (errno != 0) {
    const auto message =
      errno_message("read " + description + " directory", directory.path);
    (void)::closedir(stream);
    fail(FloorAssetSnapshotError::kIoError, message);
  }
  if (::closedir(stream) != 0) {
    fail(
      FloorAssetSnapshotError::kIoError,
      errno_message("close directory stream", directory.path));
  }
  return names;
}

struct OpenFile
{
  UniqueFd descriptor;
  struct stat initial {};
  int parent_descriptor{-1};
  std::string name;
  fs::path path;
};

OpenFile open_file(
  const OpenDirectory & parent,
  const std::string & name,
  const fs::path & path,
  const std::uintmax_t maximum_bytes)
{
  struct stat path_status {};
  if (::fstatat(
      parent.descriptor.get(), name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    fail(
      FloorAssetSnapshotError::kInvalidLayout,
      errno_message("inspect required map asset", path));
  }
  if (S_ISLNK(path_status.st_mode) || !S_ISREG(path_status.st_mode) ||
    path_status.st_nlink != 1 || path_status.st_size < 0 ||
    static_cast<std::uintmax_t>(path_status.st_size) > maximum_bytes)
  {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset must be a single-link bounded regular file: " +
      path.string());
  }

  UniqueFd descriptor(
    ::openat(
      parent.descriptor.get(), name.c_str(),
      O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
  if (descriptor.get() < 0) {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      errno_message("open required map asset", path));
  }
  struct stat opened {};
  if (::fstat(descriptor.get(), &opened) != 0 ||
    !same_file_identity(path_status, opened))
  {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "required map asset changed while it was opened: " + path.string());
  }
  return {
    std::move(descriptor),
    opened,
    parent.descriptor.get(),
    name,
    path,
  };
}

std::string read_bounded_file(const OpenFile & file)
{
  if (static_cast<std::uintmax_t>(file.initial.st_size) >
    static_cast<std::uintmax_t>(
      std::numeric_limits<std::size_t>::max()))
  {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset is too large for this process: " +
      file.path.string());
  }
  if (::lseek(file.descriptor.get(), 0, SEEK_SET) < 0) {
    fail(
      FloorAssetSnapshotError::kIoError,
      errno_message("rewind required map asset", file.path));
  }

  const auto expected_size = static_cast<std::size_t>(file.initial.st_size);
  std::string content(expected_size, '\0');
  std::size_t offset = 0U;
  while (offset < expected_size) {
    const auto count = ::read(
      file.descriptor.get(), content.data() + offset,
      expected_size - offset);
    if (count == 0) {
      fail(
        FloorAssetSnapshotError::kAssetChanged,
        "required map asset became shorter while it was read: " +
        file.path.string());
    }
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      fail(
        FloorAssetSnapshotError::kIoError,
        errno_message("read required map asset", file.path));
    }
    offset += static_cast<std::size_t>(count);
  }
  return content;
}

std::string digest_open_role_files(
  const std::vector<RoleSpec> & roles,
  const std::vector<OpenFile> & files)
{
  if (roles.size() != files.size()) {
    fail(
      FloorAssetSnapshotError::kInvalidLayout,
      "internal map asset role/file count differs");
  }

  robot_map_asset_identity::CanonicalMapAssetDigestStream digest;
  std::array<char, 64U * 1024U> buffer{};
  for (const auto index : canonical_role_order(roles)) {
    const auto & file = files[index];
    if (::lseek(file.descriptor.get(), 0, SEEK_SET) < 0) {
      fail(
        FloorAssetSnapshotError::kIoError,
        errno_message("rewind required map asset", file.path));
    }

    const auto expected_size =
      static_cast<std::uint64_t>(file.initial.st_size);
    digest.begin_entry(roles[index].logical_name, expected_size);
    std::uint64_t bytes_read = 0U;
    while (bytes_read < expected_size) {
      const auto remaining = expected_size - bytes_read;
      const auto requested = static_cast<std::size_t>(
        std::min<std::uint64_t>(remaining, buffer.size()));
      const auto count =
        ::read(file.descriptor.get(), buffer.data(), requested);
      if (count == 0) {
        fail(
          FloorAssetSnapshotError::kAssetChanged,
          "required map asset became shorter while it was hashed: " +
          file.path.string());
      }
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        fail(
          FloorAssetSnapshotError::kIoError,
          errno_message("read required map asset", file.path));
      }
      digest.update(
        std::string_view(buffer.data(), static_cast<std::size_t>(count)));
      bytes_read += static_cast<std::uint64_t>(count);
    }
    digest.end_entry();
  }
  return digest.finish();
}

void revalidate_file(const OpenFile & file)
{
  struct stat opened {};
  struct stat path_status {};
  if (::fstat(file.descriptor.get(), &opened) != 0 ||
    ::fstatat(
      file.parent_descriptor, file.name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    !S_ISREG(path_status.st_mode) || path_status.st_nlink != 1 ||
    !same_file_identity(file.initial, opened) ||
    !same_file_identity(opened, path_status))
  {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "required map asset changed during snapshot: " + file.path.string());
  }
}

FloorAssetFileFingerprint fingerprint(const OpenFile & file)
{
  return {
    static_cast<std::uint64_t>(file.initial.st_dev),
    static_cast<std::uint64_t>(file.initial.st_ino),
    static_cast<std::uint64_t>(file.initial.st_size),
    static_cast<std::int64_t>(file.initial.st_mtim.tv_sec),
    static_cast<std::int64_t>(file.initial.st_mtim.tv_nsec),
  };
}

FloorAssetSnapshot load_platform_snapshot(
  const FloorAssetSnapshotRequest & request)
{
  std::error_code absolute_error;
  const auto maps_root =
    fs::absolute(request.maps_root, absolute_error).lexically_normal();
  if (absolute_error) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      "cannot resolve maps_root: " + absolute_error.message());
  }
  const auto root =
    maps_root / request.building_id / request.floor_id / "maps" /
    request.map_id;

  auto maps_root_directory = open_root_directory(maps_root);
  auto building_directory = open_child_directory(
    maps_root_directory, request.building_id,
    maps_root / request.building_id);
  auto floor_directory = open_child_directory(
    building_directory, request.floor_id,
    maps_root / request.building_id / request.floor_id);
  auto maps_directory = open_child_directory(
    floor_directory, "maps",
    maps_root / request.building_id / request.floor_id / "maps");
  auto map_directory = open_child_directory(
    maps_directory, request.map_id, root);
  auto nav_directory =
    open_child_directory(map_directory, "nav", root / "nav");
  auto localizer_directory =
    open_child_directory(map_directory, "localizer", root / "localizer");
  auto filters_directory =
    open_child_directory(map_directory, "filters", root / "filters");
  auto reports_directory =
    open_child_directory(map_directory, "reports", root / "reports");

  const auto nav_names = list_directory(nav_directory, "nav asset directory");
  const auto localizer_names =
    list_directory(localizer_directory, "localizer asset directory");
  const auto nav_yaml_stem =
    unique_stem(nav_names, ".yaml", "nav asset directory");
  const auto nav_pgm_stem =
    unique_stem(nav_names, ".pgm", "nav asset directory");
  const auto localizer_yaml_stem =
    unique_stem(localizer_names, ".yaml", "localizer asset directory");
  const auto localizer_png_stem =
    unique_stem(localizer_names, ".png", "localizer asset directory");
  if (nav_yaml_stem != nav_pgm_stem ||
    nav_yaml_stem != localizer_yaml_stem ||
    nav_yaml_stem != localizer_png_stem)
  {
    fail(
      FloorAssetSnapshotError::kInvalidLayout,
      "nav and localizer assets do not have one common safe map name");
  }
  const auto safe_map_name = nav_yaml_stem;

  const auto roles = digest_roles(safe_map_name);
  std::vector<OpenFile> role_files;
  role_files.reserve(roles.size());
  std::uintmax_t aggregate_bytes = 0U;
  for (const auto & role : roles) {
    const auto parent_name = role.relative_path.parent_path().string();
    const OpenDirectory * parent = nullptr;
    if (parent_name == "nav") {
      parent = &nav_directory;
    } else if (parent_name == "localizer") {
      parent = &localizer_directory;
    } else if (parent_name == "filters") {
      parent = &filters_directory;
    } else if (parent_name == "reports") {
      parent = &reports_directory;
    }
    if (parent == nullptr) {
      fail(
        FloorAssetSnapshotError::kInvalidLayout,
        "internal map asset role has no trusted parent");
    }
    auto file = open_file(
      *parent, role.relative_path.filename().string(),
      root / role.relative_path, kMaximumRoleAssetBytes);
    const auto bytes = static_cast<std::uintmax_t>(file.initial.st_size);
    if (bytes > kMaximumBundleBytes - aggregate_bytes) {
      fail(
        FloorAssetSnapshotError::kUnsafeAsset,
        "map asset bundle exceeds the aggregate size limit");
    }
    aggregate_bytes += bytes;
    role_files.push_back(std::move(file));
  }

  auto manifest = open_file(
    map_directory, "manifest.json", root / "manifest.json",
    kMaximumManifestBytes);
  auto poses = open_file(
    map_directory, "poses.yaml", root / "poses.yaml",
    kMaximumPosesBytes);
  validate_manifest(read_bounded_file(manifest), request, safe_map_name);

  for (const auto & file : role_files) {
    revalidate_file(file);
  }
  revalidate_file(manifest);
  revalidate_file(poses);

  const auto digest = digest_open_role_files(roles, role_files);
  if (digest != request.expected_asset_digest) {
    fail(
      FloorAssetSnapshotError::kDigestMismatch,
      "map bundle content digest differs from the authoritative identity");
  }

  for (const auto & file : role_files) {
    revalidate_file(file);
  }
  revalidate_file(manifest);
  revalidate_file(poses);
  revalidate_directory(reports_directory);
  revalidate_directory(filters_directory);
  revalidate_directory(localizer_directory);
  revalidate_directory(nav_directory);
  revalidate_directory(map_directory);
  revalidate_directory(maps_directory);
  revalidate_directory(floor_directory);
  revalidate_directory(building_directory);
  revalidate_directory(maps_root_directory);

  // Close the TOCTOU window around registry lookup as much as the read-only
  // registry API permits. A changed authoritative binding invalidates this
  // result even when all source files stayed stable.
  require_registry_identity(request);

  const auto & files = role_files;
  FloorAssetSnapshot snapshot;
  snapshot.building_id = request.building_id;
  snapshot.floor_id = request.floor_id;
  snapshot.map_id = request.map_id;
  snapshot.asset_epoch = request.expected_asset_epoch;
  snapshot.asset_digest = request.expected_asset_digest;
  snapshot.paths = {
    root,
    root / "manifest.json",
    root / "nav" / (safe_map_name + ".yaml"),
    root / "nav" / (safe_map_name + ".pgm"),
    root / "localizer" / (safe_map_name + ".png"),
    root / "localizer" / (safe_map_name + ".yaml"),
    root / "filters" / "keepout_mask.yaml",
    root / "filters" / "keepout_mask.pgm",
    root / "filters" / "speed_mask.yaml",
    root / "filters" / "speed_mask.pgm",
    root / "filters" / "binary_mask.yaml",
    root / "filters" / "binary_mask.pgm",
    root / "reports" / "asset_report.json",
    root / "poses.yaml",
  };
  snapshot.fingerprints = {
    fingerprint(manifest),
    fingerprint(files[0U]),
    fingerprint(files[1U]),
    fingerprint(files[2U]),
    fingerprint(files[3U]),
    fingerprint(files[4U]),
    fingerprint(files[5U]),
    fingerprint(files[6U]),
    fingerprint(files[7U]),
    fingerprint(files[8U]),
    fingerprint(files[9U]),
    fingerprint(files[10U]),
    fingerprint(poses),
  };
  return snapshot;
}

#else

FloorAssetFileFingerprint fingerprint_path(const fs::path & path)
{
  std::error_code error;
  const auto size = fs::file_size(path, error);
  if (error) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot size required map asset " + path.string() + ": " +
      error.message());
  }
  const auto modified = fs::last_write_time(path, error);
  if (error) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot timestamp required map asset " + path.string() + ": " +
      error.message());
  }
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    modified.time_since_epoch()).count();
  const auto seconds = nanoseconds / 1000000000LL;
  return {
    0U,
    0U,
    static_cast<std::uint64_t>(size),
    seconds,
    nanoseconds - seconds * 1000000000LL,
  };
}

FloorAssetFileFingerprint inspect_file_windows(
  const fs::path & path,
  const std::uintmax_t maximum_bytes)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error || status.type() != fs::file_type::regular)
  {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset must be a single-link bounded regular file: " +
      path.string());
  }
  const auto link_count = fs::hard_link_count(path, error);
  if (error || link_count != 1U) {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset must be a single-link bounded regular file: " +
      path.string());
  }
  const auto size = fs::file_size(path, error);
  if (error || size > maximum_bytes) {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset must be a single-link bounded regular file: " +
      path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot read required map asset: " + path.string());
  }
  return fingerprint_path(path);
}

void revalidate_file_windows(
  const fs::path & path,
  const std::uintmax_t maximum_bytes,
  const FloorAssetFileFingerprint & expected)
{
  if (!(inspect_file_windows(path, maximum_bytes) == expected)) {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "required map asset changed during snapshot: " + path.string());
  }
}

std::string read_bounded_file_windows(
  const fs::path & path,
  const std::uintmax_t maximum_bytes,
  FloorAssetFileFingerprint & result_fingerprint)
{
  const auto before = inspect_file_windows(path, maximum_bytes);
  if (before.size >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
  {
    fail(
      FloorAssetSnapshotError::kUnsafeAsset,
      "required map asset is too large for this process: " + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot read required map asset: " + path.string());
  }
  std::string content(static_cast<std::size_t>(before.size), '\0');
  if (!content.empty()) {
    input.read(content.data(), static_cast<std::streamsize>(content.size()));
    if (input.gcount() != static_cast<std::streamsize>(content.size())) {
      fail(
        FloorAssetSnapshotError::kAssetChanged,
        "required map asset became shorter while it was read: " +
        path.string());
    }
  }
  const auto after = inspect_file_windows(path, maximum_bytes);
  if (!(before == after)) {
    fail(
      FloorAssetSnapshotError::kAssetChanged,
      "required map asset changed while it was read: " + path.string());
  }
  result_fingerprint = after;
  return content;
}

void stream_file_windows(
  const fs::path & path,
  const std::string_view logical_name,
  const FloorAssetFileFingerprint & expected,
  robot_map_asset_identity::CanonicalMapAssetDigestStream & digest)
{
  revalidate_file_windows(path, kMaximumRoleAssetBytes, expected);
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot read required map asset: " + path.string());
  }

  digest.begin_entry(logical_name, expected.size);
  std::array<char, 64U * 1024U> buffer{};
  std::uint64_t bytes_read = 0U;
  while (bytes_read < expected.size) {
    const auto remaining = expected.size - bytes_read;
    const auto requested = static_cast<std::streamsize>(
      std::min<std::uint64_t>(remaining, buffer.size()));
    input.read(buffer.data(), requested);
    const auto count = input.gcount();
    if (count <= 0) {
      fail(
        FloorAssetSnapshotError::kAssetChanged,
        "required map asset became shorter while it was hashed: " +
        path.string());
    }
    digest.update(
      std::string_view(buffer.data(), static_cast<std::size_t>(count)));
    bytes_read += static_cast<std::uint64_t>(count);
  }
  digest.end_entry();
  revalidate_file_windows(path, kMaximumRoleAssetBytes, expected);
}

void require_real_directory_windows(const fs::path & path)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error || status.type() != fs::file_type::directory) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      "map bundle path component must be a real directory: " + path.string());
  }
}

std::vector<std::string> list_directory_windows(
  const fs::path & path,
  const std::string & description)
{
  std::vector<std::string> names;
  std::error_code error;
  for (fs::directory_iterator iterator(path, error), end;
    !error && iterator != end; iterator.increment(error))
  {
    const auto status = iterator->symlink_status(error);
    if (error || status.type() == fs::file_type::symlink) {
      fail(
        FloorAssetSnapshotError::kUnsafeAsset,
        description + " contains an unsafe entry");
    }
    names.push_back(iterator->path().filename().string());
  }
  if (error) {
    fail(
      FloorAssetSnapshotError::kIoError,
      "cannot enumerate " + description + ": " + error.message());
  }
  return names;
}

FloorAssetSnapshot load_platform_snapshot(
  const FloorAssetSnapshotRequest & request)
{
  std::error_code error;
  const auto maps_root =
    fs::absolute(request.maps_root, error).lexically_normal();
  if (error) {
    fail(
      FloorAssetSnapshotError::kUnsafePath,
      "cannot resolve maps_root: " + error.message());
  }
  const auto root =
    maps_root / request.building_id / request.floor_id / "maps" /
    request.map_id;
  require_real_directory_windows(maps_root);
  require_real_directory_windows(maps_root / request.building_id);
  require_real_directory_windows(
    maps_root / request.building_id / request.floor_id);
  require_real_directory_windows(
    maps_root / request.building_id / request.floor_id / "maps");
  require_real_directory_windows(root);
  for (const auto & name : {"nav", "localizer", "filters", "reports"}) {
    require_real_directory_windows(root / name);
  }

  const auto nav_names =
    list_directory_windows(root / "nav", "nav asset directory");
  const auto localizer_names =
    list_directory_windows(root / "localizer", "localizer asset directory");
  const auto nav_yaml_stem =
    unique_stem(nav_names, ".yaml", "nav asset directory");
  const auto nav_pgm_stem =
    unique_stem(nav_names, ".pgm", "nav asset directory");
  const auto localizer_yaml_stem =
    unique_stem(localizer_names, ".yaml", "localizer asset directory");
  const auto localizer_png_stem =
    unique_stem(localizer_names, ".png", "localizer asset directory");
  if (nav_yaml_stem != nav_pgm_stem ||
    nav_yaml_stem != localizer_yaml_stem ||
    nav_yaml_stem != localizer_png_stem)
  {
    fail(
      FloorAssetSnapshotError::kInvalidLayout,
      "nav and localizer assets do not have one common safe map name");
  }
  const auto safe_map_name = nav_yaml_stem;
  const auto roles = digest_roles(safe_map_name);
  std::vector<FloorAssetFileFingerprint> fingerprints;
  fingerprints.reserve(roles.size());
  std::uintmax_t aggregate_bytes = 0U;
  for (const auto & role : roles) {
    const auto fingerprint = inspect_file_windows(
      root / role.relative_path, kMaximumRoleAssetBytes);
    const auto bytes = static_cast<std::uintmax_t>(fingerprint.size);
    if (bytes > kMaximumBundleBytes - aggregate_bytes) {
      fail(
        FloorAssetSnapshotError::kUnsafeAsset,
        "map asset bundle exceeds the aggregate size limit");
    }
    aggregate_bytes += bytes;
    fingerprints.push_back(fingerprint);
  }
  FloorAssetFileFingerprint manifest_fingerprint;
  const auto manifest_content = read_bounded_file_windows(
    root / "manifest.json", kMaximumManifestBytes, manifest_fingerprint);
  const auto poses_fingerprint =
    inspect_file_windows(root / "poses.yaml", kMaximumPosesBytes);
  validate_manifest(manifest_content, request, safe_map_name);

  robot_map_asset_identity::CanonicalMapAssetDigestStream digest_stream;
  for (const auto index : canonical_role_order(roles)) {
    stream_file_windows(
      root / roles[index].relative_path,
      roles[index].logical_name,
      fingerprints[index],
      digest_stream);
  }
  if (digest_stream.finish() != request.expected_asset_digest)
  {
    fail(
      FloorAssetSnapshotError::kDigestMismatch,
      "map bundle content digest differs from the authoritative identity");
  }

  for (std::size_t index = 0U; index < roles.size(); ++index) {
    revalidate_file_windows(
      root / roles[index].relative_path,
      kMaximumRoleAssetBytes,
      fingerprints[index]);
  }
  revalidate_file_windows(
    root / "manifest.json", kMaximumManifestBytes, manifest_fingerprint);
  revalidate_file_windows(
    root / "poses.yaml", kMaximumPosesBytes, poses_fingerprint);
  require_registry_identity(request);

  FloorAssetSnapshot snapshot;
  snapshot.building_id = request.building_id;
  snapshot.floor_id = request.floor_id;
  snapshot.map_id = request.map_id;
  snapshot.asset_epoch = request.expected_asset_epoch;
  snapshot.asset_digest = request.expected_asset_digest;
  snapshot.paths = {
    root,
    root / "manifest.json",
    root / roles[0U].relative_path,
    root / roles[1U].relative_path,
    root / roles[2U].relative_path,
    root / roles[3U].relative_path,
    root / roles[4U].relative_path,
    root / roles[5U].relative_path,
    root / roles[6U].relative_path,
    root / roles[7U].relative_path,
    root / roles[8U].relative_path,
    root / roles[9U].relative_path,
    root / roles[10U].relative_path,
    root / "poses.yaml",
  };
  snapshot.fingerprints = {
    manifest_fingerprint,
    fingerprints[0U],
    fingerprints[1U],
    fingerprints[2U],
    fingerprints[3U],
    fingerprints[4U],
    fingerprints[5U],
    fingerprints[6U],
    fingerprints[7U],
    fingerprints[8U],
    fingerprints[9U],
    fingerprints[10U],
    poses_fingerprint,
  };
  return snapshot;
}

#endif

}  // namespace

const char * floor_asset_snapshot_error_name(
  const FloorAssetSnapshotError error) noexcept
{
  switch (error) {
    case FloorAssetSnapshotError::kNone:
      return "NONE";
    case FloorAssetSnapshotError::kInvalidRequest:
      return "INVALID_REQUEST";
    case FloorAssetSnapshotError::kRegistryUnavailable:
      return "REGISTRY_UNAVAILABLE";
    case FloorAssetSnapshotError::kIdentityNotFound:
      return "IDENTITY_NOT_FOUND";
    case FloorAssetSnapshotError::kIdentityMismatch:
      return "IDENTITY_MISMATCH";
    case FloorAssetSnapshotError::kUnsafePath:
      return "UNSAFE_PATH";
    case FloorAssetSnapshotError::kInvalidManifest:
      return "INVALID_MANIFEST";
    case FloorAssetSnapshotError::kInvalidLayout:
      return "INVALID_LAYOUT";
    case FloorAssetSnapshotError::kUnsafeAsset:
      return "UNSAFE_ASSET";
    case FloorAssetSnapshotError::kAssetChanged:
      return "ASSET_CHANGED";
    case FloorAssetSnapshotError::kDigestMismatch:
      return "DIGEST_MISMATCH";
    case FloorAssetSnapshotError::kIoError:
      return "IO_ERROR";
  }
  return "UNKNOWN";
}

bool FloorAssetFileFingerprint::operator==(
  const FloorAssetFileFingerprint & other) const noexcept
{
  return device == other.device && inode == other.inode &&
         size == other.size &&
         modified_seconds == other.modified_seconds &&
         modified_nanoseconds == other.modified_nanoseconds;
}

bool FloorAssetSnapshotResult::ok() const noexcept
{
  return error == FloorAssetSnapshotError::kNone && snapshot.has_value();
}

FloorAssetSnapshotResult FloorAssetSnapshotLoader::load(
  const FloorAssetSnapshotRequest & request) const noexcept
{
  try {
    if (request.maps_root.empty() ||
      !safe_asset_id(request.building_id) ||
      !safe_asset_id(request.floor_id) ||
      !safe_asset_id(request.map_id) ||
      request.expected_asset_epoch == 0U ||
      !robot_map_asset_identity::is_canonical_sha256_digest(
        request.expected_asset_digest))
    {
      fail(
        FloorAssetSnapshotError::kInvalidRequest,
        "snapshot request requires a safe building/floor/map and a nonzero "
        "epoch with canonical lowercase sha256 digest");
    }

    require_registry_identity(request);
    auto snapshot = load_platform_snapshot(request);
    return {
      FloorAssetSnapshotError::kNone,
      "exact source map asset snapshot verified",
      std::move(snapshot),
    };
  } catch (const LoadFailure & error) {
    return {error.error(), error.what(), std::nullopt};
  } catch (const std::exception & error) {
    return {
      FloorAssetSnapshotError::kIoError,
      std::string("unexpected floor asset snapshot failure: ") + error.what(),
      std::nullopt,
    };
  } catch (...) {
    return {
      FloorAssetSnapshotError::kIoError,
      "unexpected non-standard floor asset snapshot failure",
      std::nullopt,
    };
  }
}

}  // namespace robot_floor_manager
