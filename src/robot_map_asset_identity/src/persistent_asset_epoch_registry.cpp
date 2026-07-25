#include "robot_map_asset_identity/map_asset_identity.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace robot_map_asset_identity
{
namespace
{

constexpr char kRegistryDirectory[] = ".map_asset_registry";
constexpr char kLockFilename[] = "registry.lock";
constexpr char kStateFilename[] = "epoch_state.json";
constexpr char kBindingsFilename[] = "bindings.json";
constexpr std::size_t kMaximumRegistryFileSize = 32U * 1024U * 1024U;
constexpr std::size_t kMaximumBindingCount = 100000U;
constexpr std::string_view kStatePrefix{
  "{\"schema\":\"njrh.map_asset_epoch_state.v1\","
  "\"high_watermark\":\""};
constexpr std::string_view kStateSuffix{"\"}\n"};
constexpr std::string_view kBindingsHeader{
  "{\"schema\":\"njrh.map_asset_bindings.v1\",\"bindings\":["};
constexpr std::string_view kBindingsFooter{"]}"};

struct AssetIdentityKeyLess
{
  bool operator()(
    const AssetIdentityKey & left,
    const AssetIdentityKey & right) const noexcept
  {
    return std::tie(left.building_id, left.floor_id, left.map_id) <
           std::tie(right.building_id, right.floor_id, right.map_id);
  }
};

using BindingMap =
  std::map<AssetIdentityKey, AssetIdentity, AssetIdentityKeyLess>;

struct RegistrySnapshot
{
  std::uint64_t high_watermark{0U};
  bool state_present{false};
  BindingMap bindings;
};

std::runtime_error filesystem_error(
  const std::string & operation,
  const std::filesystem::path & path,
  const std::error_code & error)
{
  return std::runtime_error(
    operation + " failed for " + path.string() + ": " + error.message());
}

std::runtime_error errno_error(
  const std::string & operation,
  const std::filesystem::path & path)
{
  return std::runtime_error(
    operation + " failed for " + path.string() + ": " +
    std::error_code(errno, std::generic_category()).message());
}

bool valid_key_component(const std::string_view value)
{
  if (value.empty() || value.size() > 128U ||
    value.find("..") != std::string_view::npos)
  {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
    return (character >= 'a' && character <= 'z') ||
           (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') ||
           character == '-' || character == '_' || character == '.';
  });
}

void validate_key(const AssetIdentityKey & key)
{
  if (!valid_key_component(key.building_id)) {
    throw std::invalid_argument("building_id is not a safe asset identifier");
  }
  if (!valid_key_component(key.floor_id)) {
    throw std::invalid_argument("floor_id is not a safe asset identifier");
  }
  if (!valid_key_component(key.map_id)) {
    throw std::invalid_argument("map_id is not a safe asset identifier");
  }
}

std::filesystem::file_status checked_symlink_status(
  const std::filesystem::path & path)
{
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (error == std::errc::no_such_file_or_directory) {
    return std::filesystem::file_status{
      std::filesystem::file_type::not_found};
  }
  if (error) {
    throw filesystem_error("symlink_status", path, error);
  }
  return status;
}

bool path_exists(const std::filesystem::file_status & status)
{
  return status.type() != std::filesystem::file_type::not_found;
}

#ifndef _WIN32
void fsync_directory(const std::filesystem::path & directory)
{
  int flags = O_RDONLY;
#ifdef O_DIRECTORY
  flags |= O_DIRECTORY;
#endif
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif
  const int descriptor = ::open(directory.c_str(), flags);
  if (descriptor < 0) {
    throw errno_error("open directory for fsync", directory);
  }
  if (::fsync(descriptor) != 0) {
    const auto error = errno_error("fsync directory", directory);
    ::close(descriptor);
    throw error;
  }
  if (::close(descriptor) != 0) {
    throw errno_error("close directory after fsync", directory);
  }
}
#endif

void require_existing_directory(
  const std::filesystem::path & path,
  const std::string & description)
{
  const auto status = checked_symlink_status(path);
  if (std::filesystem::is_symlink(status) ||
    !std::filesystem::is_directory(status))
  {
    throw std::runtime_error(description + " must be a real directory: " +
            path.string());
  }
}

void ensure_registry_directory(
  const std::filesystem::path & maps_root,
  const std::filesystem::path & registry_root)
{
  require_existing_directory(maps_root, "maps_root");

  auto status = checked_symlink_status(registry_root);
  if (!path_exists(status)) {
    std::error_code error;
    if (!std::filesystem::create_directory(registry_root, error) && error) {
      throw filesystem_error("create registry directory", registry_root, error);
    }
#ifndef _WIN32
    fsync_directory(maps_root);
#endif
    status = checked_symlink_status(registry_root);
  }
  if (std::filesystem::is_symlink(status) ||
    !std::filesystem::is_directory(status))
  {
    throw std::runtime_error(
            "map asset registry root must be a real directory: " +
            registry_root.string());
  }
}

class RegistryFileLock
{
public:
  explicit RegistryFileLock(
    const std::filesystem::path & path,
    const bool create_if_missing = true,
    const bool shared_read_only = false)
  {
#ifdef _WIN32
    handle_ = ::CreateFileW(
      path.c_str(),
      shared_read_only ? GENERIC_READ : (GENERIC_READ | GENERIC_WRITE),
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      create_if_missing ? OPEN_ALWAYS : OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
      nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error(
              "open registry lock failed for " + path.string() +
              " (win32 error " + std::to_string(::GetLastError()) + ")");
    }
    BY_HANDLE_FILE_INFORMATION information{};
    if (!::GetFileInformationByHandle(handle_, &information) ||
      (information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0U ||
      (information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0U)
    {
      ::CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
      throw std::runtime_error(
              "registry lock must be a non-reparse regular file: " +
              path.string());
    }
    OVERLAPPED overlapped{};
    if (!::LockFileEx(
        handle_, shared_read_only ? 0U : LOCKFILE_EXCLUSIVE_LOCK, 0U,
        std::numeric_limits<DWORD>::max(),
        std::numeric_limits<DWORD>::max(), &overlapped))
    {
      const auto error = ::GetLastError();
      ::CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
      throw std::runtime_error(
              "lock registry failed for " + path.string() +
              " (win32 error " + std::to_string(error) + ")");
    }
#else
    int flags = shared_read_only ? O_RDONLY : O_RDWR;
    if (create_if_missing) {
      flags |= O_CREAT;
    }
#ifdef O_CLOEXEC
    flags |= O_CLOEXEC;
#endif
#ifdef O_NOFOLLOW
    flags |= O_NOFOLLOW;
#endif
    descriptor_ = ::open(path.c_str(), flags, 0600);
    if (descriptor_ < 0) {
      throw errno_error("open registry lock", path);
    }
    struct stat information {};
    if (::fstat(descriptor_, &information) != 0 ||
      !S_ISREG(information.st_mode))
    {
      ::close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error(
              "registry lock must be a regular file: " + path.string());
    }
    if (::flock(descriptor_, shared_read_only ? LOCK_SH : LOCK_EX) != 0) {
      const auto error = errno_error("flock registry", path);
      ::close(descriptor_);
      descriptor_ = -1;
      throw error;
    }
#endif
  }

  RegistryFileLock(const RegistryFileLock &) = delete;
  RegistryFileLock & operator=(const RegistryFileLock &) = delete;

  ~RegistryFileLock()
  {
#ifdef _WIN32
    if (handle_ != INVALID_HANDLE_VALUE) {
      OVERLAPPED overlapped{};
      (void)::UnlockFileEx(
        handle_, 0U,
        std::numeric_limits<DWORD>::max(),
        std::numeric_limits<DWORD>::max(), &overlapped);
      (void)::CloseHandle(handle_);
    }
#else
    if (descriptor_ >= 0) {
      (void)::flock(descriptor_, LOCK_UN);
      (void)::close(descriptor_);
    }
#endif
  }

private:
#ifdef _WIN32
  HANDLE handle_{INVALID_HANDLE_VALUE};
#else
  int descriptor_{-1};
#endif
};

void validate_registry_contents(const std::filesystem::path & registry_root)
{
  std::error_code iterator_error;
  std::filesystem::directory_iterator iterator(
    registry_root,
    std::filesystem::directory_options::none,
    iterator_error);
  if (iterator_error) {
    throw filesystem_error(
            "enumerate map asset registry", registry_root, iterator_error);
  }

  for (const auto & entry : iterator) {
    const auto filename = entry.path().filename().string();
    if (filename != kLockFilename &&
      filename != kStateFilename &&
      filename != kBindingsFilename)
    {
      throw std::runtime_error(
              "unexpected path in map asset registry: " +
              entry.path().string());
    }
    const auto status = checked_symlink_status(entry.path());
    if (std::filesystem::is_symlink(status) ||
      !std::filesystem::is_regular_file(status))
    {
      throw std::runtime_error(
              "registry paths must be non-symlink regular files: " +
              entry.path().string());
    }
  }
}

std::optional<std::string> read_optional_regular_file(
  const std::filesystem::path & path)
{
  const auto status = checked_symlink_status(path);
  if (!path_exists(status)) {
    return std::nullopt;
  }
  if (std::filesystem::is_symlink(status) ||
    !std::filesystem::is_regular_file(status))
  {
    throw std::runtime_error(
            "registry path must be a non-symlink regular file: " +
            path.string());
  }

#ifdef _WIN32
  std::error_code size_error;
  const auto size = std::filesystem::file_size(path, size_error);
  if (size_error || size > kMaximumRegistryFileSize) {
    throw std::runtime_error(
            "registry file is unreadable or too large: " + path.string());
  }
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("open registry file failed: " + path.string());
  }
  std::string content(
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>());
  if (input.bad() || content.size() != size) {
    throw std::runtime_error(
            "registry file changed or failed while reading: " + path.string());
  }
  return content;
#else
  int flags = O_RDONLY;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif
#ifdef O_NOFOLLOW
  flags |= O_NOFOLLOW;
#endif
  const int descriptor = ::open(path.c_str(), flags);
  if (descriptor < 0) {
    throw errno_error("open registry file", path);
  }
  struct stat information {};
  if (::fstat(descriptor, &information) != 0 ||
    !S_ISREG(information.st_mode) || information.st_size < 0 ||
    static_cast<std::uintmax_t>(information.st_size) >
    kMaximumRegistryFileSize)
  {
    ::close(descriptor);
    throw std::runtime_error(
            "registry file is not regular or exceeds the size limit: " +
            path.string());
  }

  std::string content;
  content.reserve(static_cast<std::size_t>(information.st_size));
  char buffer[8192];
  while (true) {
    const auto count = ::read(descriptor, buffer, sizeof(buffer));
    if (count == 0) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      const auto error = errno_error("read registry file", path);
      ::close(descriptor);
      throw error;
    }
    if (content.size() + static_cast<std::size_t>(count) >
      kMaximumRegistryFileSize)
    {
      ::close(descriptor);
      throw std::runtime_error(
              "registry file exceeds the size limit: " + path.string());
    }
    content.append(buffer, static_cast<std::size_t>(count));
  }
  if (::close(descriptor) != 0) {
    throw errno_error("close registry file", path);
  }
  return content;
#endif
}

std::uint64_t parse_canonical_u64(
  const std::string_view text,
  const std::string & context)
{
  if (text.empty() ||
    (text.size() > 1U && text.front() == '0') ||
    !std::all_of(text.begin(), text.end(), [](const char character) {
      return character >= '0' && character <= '9';
    }))
  {
    throw std::runtime_error(context + " is not a canonical uint64");
  }
  std::uint64_t value{0U};
  const auto result = std::from_chars(
    text.data(), text.data() + text.size(), value);
  if (result.ec == std::errc::result_out_of_range ||
    result.ec != std::errc{} ||
    result.ptr != text.data() + text.size())
  {
    throw std::runtime_error(context + " is outside uint64 range");
  }
  return value;
}

enum class ManifestJsonScalarKind
{
  kString,
  kNumber,
  kBoolean,
  kOther,
};

struct ManifestJsonScalar
{
  ManifestJsonScalarKind kind{ManifestJsonScalarKind::kOther};
  std::string value;
};

using ManifestJsonObject = std::map<std::string, ManifestJsonScalar>;

class StrictManifestJsonParser
{
public:
  explicit StrictManifestJsonParser(const std::string & input)
  : input_(input)
  {
  }

  ManifestJsonObject parse()
  {
    skip_whitespace();
    if (position_ >= input_.size() || input_[position_] != '{') {
      fail("root must be an object");
    }
    auto root = parse_object(0U);
    skip_whitespace();
    if (position_ != input_.size()) {
      fail("contains trailing content");
    }
    return root;
  }

private:
  static bool decimal_digit(const char value)
  {
    return value >= '0' && value <= '9';
  }

  [[noreturn]] void fail(const std::string & reason) const
  {
    throw std::runtime_error("map manifest strict JSON " + reason);
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
      fail(std::string{"expected '"} + expected + "'");
    }
  }

  ManifestJsonScalar parse_value(const std::size_t depth)
  {
    constexpr std::size_t kMaximumJsonDepth = 64U;
    if (depth > kMaximumJsonDepth) {
      fail("exceeds maximum nesting depth");
    }
    skip_whitespace();
    if (position_ >= input_.size()) {
      fail("ends before a value");
    }

    switch (input_[position_]) {
      case '{':
        static_cast<void>(parse_object(depth));
        return {};
      case '[':
        parse_array(depth);
        return {};
      case '"':
        return {ManifestJsonScalarKind::kString, parse_string()};
      case 't':
        parse_literal("true");
        return {ManifestJsonScalarKind::kBoolean, "true"};
      case 'f':
        parse_literal("false");
        return {ManifestJsonScalarKind::kBoolean, "false"};
      case 'n':
        parse_literal("null");
        return {};
      default:
        if (input_[position_] == '-' || decimal_digit(input_[position_])) {
          return {ManifestJsonScalarKind::kNumber, parse_number()};
        }
        fail("contains a token outside strict JSON");
    }
  }

  ManifestJsonObject parse_object(const std::size_t depth)
  {
    expect('{');
    skip_whitespace();
    ManifestJsonObject values;
    if (consume('}')) {
      return values;
    }

    while (true) {
      skip_whitespace();
      if (position_ >= input_.size() || input_[position_] != '"') {
        fail("requires quoted object keys");
      }
      auto key = parse_string();
      if (values.find(key) != values.end()) {
        fail("contains a duplicate object key");
      }
      skip_whitespace();
      expect(':');
      auto value = parse_value(depth + 1U);
      values.emplace(std::move(key), std::move(value));
      skip_whitespace();
      if (consume('}')) {
        return values;
      }
      expect(',');
    }
  }

  void parse_array(const std::size_t depth)
  {
    expect('[');
    skip_whitespace();
    if (consume(']')) {
      return;
    }
    while (true) {
      static_cast<void>(parse_value(depth + 1U));
      skip_whitespace();
      if (consume(']')) {
        return;
      }
      expect(',');
    }
  }

  void parse_literal(const std::string_view literal)
  {
    if (input_.compare(
        position_, literal.size(), literal.data(), literal.size()) != 0)
    {
      fail("contains an invalid literal");
    }
    position_ += literal.size();
  }

  std::string parse_number()
  {
    const auto start = position_;
    consume('-');
    if (position_ >= input_.size()) {
      fail("contains an incomplete number");
    }
    if (consume('0')) {
      if (position_ < input_.size() && decimal_digit(input_[position_])) {
        fail("contains a number with a leading zero");
      }
    } else {
      if (input_[position_] < '1' || input_[position_] > '9') {
        fail("contains an invalid number");
      }
      while (position_ < input_.size() && decimal_digit(input_[position_])) {
        ++position_;
      }
    }
    if (consume('.')) {
      if (position_ >= input_.size() || !decimal_digit(input_[position_])) {
        fail("contains an incomplete fractional number");
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
        fail("contains an incomplete exponent");
      }
      while (position_ < input_.size() && decimal_digit(input_[position_])) {
        ++position_;
      }
    }
    return input_.substr(start, position_ - start);
  }

  std::uint32_t parse_hex_quad()
  {
    std::uint32_t value = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      if (position_ >= input_.size()) {
        fail("contains an incomplete unicode escape");
      }
      const auto character =
        static_cast<unsigned char>(input_[position_++]);
      std::uint32_t digit = 0U;
      if (character >= '0' && character <= '9') {
        digit = character - '0';
      } else if (character >= 'a' && character <= 'f') {
        digit = 10U + character - 'a';
      } else if (character >= 'A' && character <= 'F') {
        digit = 10U + character - 'A';
      } else {
        fail("contains a non-hex unicode escape");
      }
      value = value * 16U + digit;
    }
    return value;
  }

  static void append_utf8(
    const std::uint32_t codepoint,
    std::string & output)
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

  std::uint32_t parse_raw_utf8()
  {
    const auto lead = static_cast<unsigned char>(input_[position_]);
    std::size_t length = 0U;
    std::uint32_t codepoint = 0U;
    std::uint32_t minimum = 0U;
    if (lead >= 0xc2U && lead <= 0xdfU) {
      length = 2U;
      codepoint = lead & 0x1fU;
      minimum = 0x80U;
    } else if (lead >= 0xe0U && lead <= 0xefU) {
      length = 3U;
      codepoint = lead & 0x0fU;
      minimum = 0x800U;
    } else if (lead >= 0xf0U && lead <= 0xf4U) {
      length = 4U;
      codepoint = lead & 0x07U;
      minimum = 0x10000U;
    } else {
      fail("contains invalid UTF-8");
    }
    if (position_ + length > input_.size()) {
      fail("contains truncated UTF-8");
    }
    for (std::size_t index = 1U; index < length; ++index) {
      const auto continuation =
        static_cast<unsigned char>(input_[position_ + index]);
      if ((continuation & 0xc0U) != 0x80U) {
        fail("contains invalid UTF-8 continuation bytes");
      }
      codepoint = (codepoint << 6U) | (continuation & 0x3fU);
    }
    position_ += length;
    if (codepoint < minimum || codepoint > 0x10ffffU ||
      (codepoint >= 0xd800U && codepoint <= 0xdfffU))
    {
      fail("contains a non-canonical UTF-8 codepoint");
    }
    return codepoint;
  }

  std::string parse_string()
  {
    expect('"');
    std::string result;
    while (position_ < input_.size()) {
      const auto character =
        static_cast<unsigned char>(input_[position_++]);
      if (character == '"') {
        return result;
      }
      if (character < 0x20U) {
        fail("contains an unescaped control character");
      }
      if (character == '\\') {
        if (position_ >= input_.size()) {
          fail("contains an incomplete string escape");
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
              if (position_ + 2U > input_.size() ||
                input_[position_] != '\\' ||
                input_[position_ + 1U] != 'u')
              {
                fail("contains an unpaired high surrogate");
              }
              position_ += 2U;
              const auto low = parse_hex_quad();
              if (low < 0xdc00U || low > 0xdfffU) {
                fail("contains an invalid low surrogate");
              }
              codepoint = 0x10000U +
                ((codepoint - 0xd800U) << 10U) + (low - 0xdc00U);
            } else if (codepoint >= 0xdc00U && codepoint <= 0xdfffU) {
              fail("contains an unpaired low surrogate");
            }
            append_utf8(codepoint, result);
            break;
          }
          default:
            fail("contains an unsupported string escape");
        }
        continue;
      }
      if (character < 0x80U) {
        result.push_back(static_cast<char>(character));
        continue;
      }
      --position_;
      append_utf8(parse_raw_utf8(), result);
    }
    fail("contains an unterminated string");
  }

  const std::string & input_;
  std::size_t position_{0U};
};

const ManifestJsonScalar * find_manifest_field(
  const ManifestJsonObject & object,
  const std::string & key)
{
  const auto found = object.find(key);
  return found == object.end() ? nullptr : &found->second;
}

std::uint64_t manifest_v2_epoch(const std::string & content)
{
  const auto root = StrictManifestJsonParser(content).parse();
  const auto schema = find_manifest_field(root, "schema");
  if (schema == nullptr) {
    return 0U;
  }
  if (schema->kind != ManifestJsonScalarKind::kString) {
    throw std::runtime_error("map manifest schema must be a JSON string");
  }
  if (schema->value != "njrh.map_manifest.v2") {
    return 0U;
  }

  const auto epoch = find_manifest_field(root, "asset_epoch");
  if (epoch == nullptr || epoch->kind != ManifestJsonScalarKind::kNumber) {
    throw std::runtime_error(
            "map manifest v2 asset_epoch must be a JSON number");
  }
  const auto value = parse_canonical_u64(
    epoch->value, "map manifest v2 asset_epoch");
  if (value == 0U) {
    throw std::runtime_error(
            "map manifest v2 asset_epoch must be positive");
  }
  return value;
}

std::uint64_t maximum_committed_v2_manifest_epoch(
  const std::filesystem::path & maps_root)
{
  constexpr std::uintmax_t kMaximumManifestSize = 1024U * 1024U;
  std::uint64_t maximum_epoch = 0U;
  std::error_code iterator_error;
  std::filesystem::recursive_directory_iterator iterator(
    maps_root,
    std::filesystem::directory_options::none,
    iterator_error);
  if (iterator_error) {
    throw filesystem_error(
            "scan maps_root for committed asset epochs", maps_root,
            iterator_error);
  }

  const std::filesystem::recursive_directory_iterator end;
  while (iterator != end) {
    const auto path = iterator->path();
    const auto relative = path.lexically_relative(maps_root);
    std::vector<std::string> components;
    for (const auto & component : relative) {
      components.push_back(component.string());
    }
    const auto source_prefix = [&components]() {
        if (components.empty() || components.size() > 5U ||
          !valid_key_component(components[0]) ||
          components[0] == kRegistryDirectory)
        {
          return false;
        }
        if (components.size() >= 2U &&
          (!valid_key_component(components[1]) ||
          components[1] == ".elevator_config" ||
          components[1] == "elevators.yaml" ||
          components[1] == "elevator_internal_poses.yaml"))
        {
          return false;
        }
        if (components.size() >= 3U && components[2] != "maps") {
          return false;
        }
        if (components.size() >= 4U &&
          !valid_key_component(components[3]))
        {
          return false;
        }
        if (components.size() == 5U &&
          components[4] != "manifest.json")
        {
          return false;
        }
        return true;
      }();
    const bool source_manifest =
      source_prefix && components.size() == 5U;
    const auto status = checked_symlink_status(path);
    if (std::filesystem::is_symlink(status)) {
      iterator.disable_recursion_pending();
      if (source_prefix) {
        throw std::runtime_error(
                "source map path symlink is not auditable: " + path.string());
      }
    } else if (std::filesystem::is_directory(status)) {
      if (!source_prefix || source_manifest) {
        iterator.disable_recursion_pending();
      }
    } else if (source_manifest) {
      if (!std::filesystem::is_regular_file(status)) {
        throw std::runtime_error(
                "map manifest is not a regular file while auditing epochs: " +
                path.string());
      }
      std::error_code size_error;
      const auto size = std::filesystem::file_size(path, size_error);
      if (size_error || size > kMaximumManifestSize) {
        throw std::runtime_error(
                "map manifest is unreadable or too large while auditing "
                "epochs: " + path.string());
      }
      std::ifstream input(path, std::ios::binary);
      if (!input) {
        throw std::runtime_error(
                "map manifest cannot be read while auditing epochs: " +
                path.string());
      }
      const std::string content{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
      if (input.bad() || content.size() != size) {
        throw std::runtime_error(
                "map manifest changed or failed while auditing epochs: " +
                path.string());
      }
      try {
        maximum_epoch = std::max(
          maximum_epoch, manifest_v2_epoch(content));
      } catch (const std::exception & error) {
        throw std::runtime_error(
            "map manifest parse failed while auditing epochs: " +
            path.string() + ": " + error.what());
      }
    } else if (
      source_prefix && components.size() >= 3U &&
      components.size() < 5U)
    {
      throw std::runtime_error(
              "source map directory component is not a directory: " +
              path.string());
    }

    iterator.increment(iterator_error);
    if (iterator_error) {
      throw filesystem_error(
              "scan maps_root for committed asset epochs", maps_root,
              iterator_error);
    }
  }
  return maximum_epoch;
}

std::uint64_t parse_state(const std::string & content)
{
  if (content.size() <= kStatePrefix.size() + kStateSuffix.size() ||
    content.compare(0U, kStatePrefix.size(), kStatePrefix) != 0 ||
    content.compare(
      content.size() - kStateSuffix.size(),
      kStateSuffix.size(), kStateSuffix) != 0)
  {
    throw std::runtime_error("asset epoch state has invalid strict JSON");
  }
  const auto value_text = std::string_view(content).substr(
    kStatePrefix.size(),
    content.size() - kStatePrefix.size() - kStateSuffix.size());
  const auto value = parse_canonical_u64(
    value_text, "asset epoch high watermark");
  if (value == 0U) {
    throw std::runtime_error(
            "persisted asset epoch high watermark must not be zero");
  }
  return value;
}

std::string serialize_state(const std::uint64_t high_watermark)
{
  if (high_watermark == 0U) {
    throw std::logic_error("asset epoch zero must never be persisted");
  }
  return std::string(kStatePrefix) + std::to_string(high_watermark) +
         std::string(kStateSuffix);
}

class LineCursor
{
public:
  explicit LineCursor(const std::string_view line)
  : line_(line)
  {
  }

  void consume(const std::string_view expected)
  {
    if (line_.substr(position_, expected.size()) != expected) {
      throw std::runtime_error(
              "asset binding has invalid strict JSON structure");
    }
    position_ += expected.size();
  }

  std::string consume_string_value()
  {
    const auto end = line_.find('"', position_);
    if (end == std::string_view::npos) {
      throw std::runtime_error(
              "asset binding has unterminated JSON string");
    }
    std::string value(line_.substr(position_, end - position_));
    position_ = end;
    return value;
  }

  bool finished() const noexcept
  {
    return position_ == line_.size();
  }

private:
  std::string_view line_;
  std::size_t position_{0U};
};

std::pair<AssetIdentityKey, AssetIdentity> parse_binding_line(
  std::string_view line)
{
  LineCursor cursor(line);
  cursor.consume("{\"building_id\":\"");
  AssetIdentityKey key;
  key.building_id = cursor.consume_string_value();
  cursor.consume("\",\"floor_id\":\"");
  key.floor_id = cursor.consume_string_value();
  cursor.consume("\",\"map_id\":\"");
  key.map_id = cursor.consume_string_value();
  cursor.consume("\",\"asset_epoch\":\"");
  const auto epoch_text = cursor.consume_string_value();
  cursor.consume("\",\"asset_digest\":\"");
  std::string digest = cursor.consume_string_value();
  cursor.consume("\"}");
  if (!cursor.finished()) {
    throw std::runtime_error(
            "asset binding has trailing JSON content");
  }

  try {
    validate_key(key);
  } catch (const std::invalid_argument & error) {
    throw std::runtime_error(
            std::string("persisted asset binding key is invalid: ") +
            error.what());
  }
  const auto epoch = parse_canonical_u64(
    epoch_text, "persisted asset epoch");
  if (epoch == 0U) {
    throw std::runtime_error("persisted asset epoch must not be zero");
  }
  if (!is_canonical_sha256_digest(digest)) {
    throw std::runtime_error(
            "persisted asset digest is not canonical sha256");
  }
  return {std::move(key), AssetIdentity{epoch, std::move(digest)}};
}

BindingMap parse_bindings(const std::string & content)
{
  if (content.empty() || content.back() != '\n') {
    throw std::runtime_error(
            "asset bindings must be canonical newline-terminated JSON");
  }

  std::vector<std::string_view> lines;
  std::size_t begin = 0U;
  while (begin < content.size()) {
    const auto end = content.find('\n', begin);
    if (end == std::string::npos) {
      throw std::runtime_error("asset bindings contain an incomplete line");
    }
    lines.emplace_back(content.data() + begin, end - begin);
    begin = end + 1U;
  }
  if (lines.size() < 2U ||
    lines.front() != kBindingsHeader ||
    lines.back() != kBindingsFooter)
  {
    throw std::runtime_error("asset bindings have invalid strict JSON");
  }
  if (lines.size() - 2U > kMaximumBindingCount) {
    throw std::runtime_error("asset binding count exceeds the safety limit");
  }

  BindingMap bindings;
  std::set<std::uint64_t> epochs;
  for (std::size_t index = 1U; index + 1U < lines.size(); ++index) {
    auto line = lines[index];
    const bool should_have_comma = index + 2U < lines.size();
    if (should_have_comma) {
      if (line.empty() || line.back() != ',') {
        throw std::runtime_error(
                "asset bindings have an invalid JSON separator");
      }
      line.remove_suffix(1U);
    } else if (!line.empty() && line.back() == ',') {
      throw std::runtime_error(
              "asset bindings contain a trailing JSON comma");
    }

    auto parsed = parse_binding_line(line);
    const auto epoch = parsed.second.asset_epoch;
    if (!epochs.insert(epoch).second) {
      throw std::runtime_error(
              "asset bindings contain a duplicate asset_epoch");
    }
    if (!bindings.emplace(
        std::move(parsed.first), std::move(parsed.second)).second)
    {
      throw std::runtime_error(
              "asset bindings contain a duplicate identity key");
    }
  }
  return bindings;
}

std::string serialize_bindings(const BindingMap & bindings)
{
  if (bindings.size() > kMaximumBindingCount) {
    throw std::runtime_error("asset binding count exceeds the safety limit");
  }

  std::string content(kBindingsHeader);
  const auto append_checked =
    [&content](const std::string_view fragment) {
      if (fragment.size() > kMaximumRegistryFileSize - content.size()) {
        throw std::runtime_error(
                "serialized asset bindings exceed the safety size limit");
      }
      content.append(fragment.data(), fragment.size());
    };
  append_checked("\n");
  std::size_t index = 0U;
  for (const auto & [key, identity] : bindings) {
    const std::string record =
      "{\"building_id\":\"" + key.building_id +
      "\",\"floor_id\":\"" + key.floor_id +
      "\",\"map_id\":\"" + key.map_id +
      "\",\"asset_epoch\":\"" + std::to_string(identity.asset_epoch) +
      "\",\"asset_digest\":\"" + identity.asset_digest + "\"}";
    append_checked(record);
    ++index;
    if (index < bindings.size()) {
      append_checked(",");
    }
    append_checked("\n");
  }
  append_checked(kBindingsFooter);
  append_checked("\n");
  return content;
}

std::atomic<std::uint64_t> temporary_counter{0U};

std::filesystem::path temporary_path_for(
  const std::filesystem::path & target)
{
#ifdef _WIN32
  const auto process_id = static_cast<std::uint64_t>(::GetCurrentProcessId());
#else
  const auto process_id = static_cast<std::uint64_t>(::getpid());
#endif
  const auto sequence =
    temporary_counter.fetch_add(1U, std::memory_order_relaxed);
  return target.parent_path() /
         ("." + target.filename().string() + ".tmp." +
         std::to_string(process_id) + "." + std::to_string(sequence));
}

void validate_atomic_target(const std::filesystem::path & target)
{
  const auto status = checked_symlink_status(target);
  if (path_exists(status) &&
    (std::filesystem::is_symlink(status) ||
    !std::filesystem::is_regular_file(status)))
  {
    throw std::runtime_error(
            "atomic registry target must be a non-symlink regular file: " +
            target.string());
  }
}

void atomic_write_file(
  const std::filesystem::path & target,
  const std::string & content)
{
  validate_atomic_target(target);
  const auto temporary = temporary_path_for(target);

#ifdef _WIN32
  HANDLE handle = ::CreateFileW(
    temporary.c_str(), GENERIC_WRITE, 0U, nullptr, CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    throw std::runtime_error(
            "create temporary registry file failed for " +
            temporary.string() + " (win32 error " +
            std::to_string(::GetLastError()) + ")");
  }
  bool success = false;
  try {
    std::size_t offset = 0U;
    while (offset < content.size()) {
      const auto remaining = content.size() - offset;
      const auto chunk = static_cast<DWORD>(std::min<std::size_t>(
          remaining, std::numeric_limits<DWORD>::max()));
      DWORD written = 0U;
      if (!::WriteFile(
          handle, content.data() + offset, chunk, &written, nullptr) ||
        written == 0U)
      {
        throw std::runtime_error(
                "write temporary registry file failed for " +
                temporary.string());
      }
      offset += written;
    }
    if (!::FlushFileBuffers(handle)) {
      throw std::runtime_error(
              "flush temporary registry file failed for " +
              temporary.string());
    }
    if (!::CloseHandle(handle)) {
      handle = INVALID_HANDLE_VALUE;
      throw std::runtime_error(
              "close temporary registry file failed for " +
              temporary.string());
    }
    handle = INVALID_HANDLE_VALUE;
    if (!::MoveFileExW(
        temporary.c_str(), target.c_str(),
        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    {
      throw std::runtime_error(
              "replace registry file failed for " + target.string() +
              " (win32 error " + std::to_string(::GetLastError()) + ")");
    }
    success = true;
  } catch (...) {
    if (handle != INVALID_HANDLE_VALUE) {
      (void)::CloseHandle(handle);
    }
    (void)::DeleteFileW(temporary.c_str());
    throw;
  }
  (void)success;
#else
  int flags = O_WRONLY | O_CREAT | O_EXCL;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif
#ifdef O_NOFOLLOW
  flags |= O_NOFOLLOW;
#endif
  int descriptor = ::open(temporary.c_str(), flags, 0600);
  if (descriptor < 0) {
    throw errno_error("create temporary registry file", temporary);
  }
  try {
    std::size_t offset = 0U;
    while (offset < content.size()) {
      const auto count = ::write(
        descriptor, content.data() + offset, content.size() - offset);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw errno_error("write temporary registry file", temporary);
      }
      if (count == 0) {
        throw std::runtime_error(
                "write temporary registry file made no progress: " +
                temporary.string());
      }
      offset += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) {
      throw errno_error("fsync temporary registry file", temporary);
    }
    if (::close(descriptor) != 0) {
      descriptor = -1;
      throw errno_error("close temporary registry file", temporary);
    }
    descriptor = -1;
    if (::rename(temporary.c_str(), target.c_str()) != 0) {
      throw errno_error("rename registry file", target);
    }
    fsync_directory(target.parent_path());
  } catch (...) {
    if (descriptor >= 0) {
      (void)::close(descriptor);
    }
    (void)::unlink(temporary.c_str());
    throw;
  }
#endif
}

RegistrySnapshot load_snapshot(const std::filesystem::path & registry_root)
{
  RegistrySnapshot snapshot;
  if (const auto bindings_content =
    read_optional_regular_file(registry_root / kBindingsFilename))
  {
    snapshot.bindings = parse_bindings(*bindings_content);
  }

  std::uint64_t maximum_binding_epoch = 0U;
  for (const auto & [key, identity] : snapshot.bindings) {
    (void)key;
    maximum_binding_epoch =
      std::max(maximum_binding_epoch, identity.asset_epoch);
  }

  if (const auto state_content =
    read_optional_regular_file(registry_root / kStateFilename))
  {
    snapshot.high_watermark = parse_state(*state_content);
    snapshot.state_present = true;
    if (snapshot.high_watermark < maximum_binding_epoch) {
      throw std::runtime_error(
              "asset epoch high watermark regressed below persisted bindings");
    }
  } else {
    snapshot.high_watermark = maximum_binding_epoch;
  }
  return snapshot;
}

template<typename Operation>
auto with_locked_registry(
  const std::filesystem::path & maps_root,
  Operation && operation)
{
  const auto registry_root = maps_root / kRegistryDirectory;
  ensure_registry_directory(maps_root, registry_root);
  RegistryFileLock lock(registry_root / kLockFilename);
  validate_registry_contents(registry_root);
  return operation(registry_root);
}

template<typename Operation>
auto with_read_only_locked_registry(
  const std::filesystem::path & maps_root,
  Operation && operation)
{
  require_existing_directory(maps_root, "maps_root");
  const auto registry_root = maps_root / kRegistryDirectory;
  const auto registry_status = checked_symlink_status(registry_root);
  if (!path_exists(registry_status)) {
    throw std::runtime_error(
            "map asset registry is missing; read-only lookup cannot prove "
            "asset identity");
  }
  if (std::filesystem::is_symlink(registry_status) ||
    !std::filesystem::is_directory(registry_status))
  {
    throw std::runtime_error(
            "map asset registry root must be a real directory: " +
            registry_root.string());
  }

  const auto lock_path = registry_root / kLockFilename;
  const auto lock_status = checked_symlink_status(lock_path);
  if (!path_exists(lock_status)) {
    validate_registry_contents(registry_root);
    throw std::runtime_error(
            "map asset registry coordination lock is missing; read-only "
            "lookup cannot prove asset identity");
  }
  if (std::filesystem::is_symlink(lock_status) ||
    !std::filesystem::is_regular_file(lock_status))
  {
    throw std::runtime_error(
            "registry lock must be a non-symlink regular file: " +
            lock_path.string());
  }

  RegistryFileLock lock(lock_path, false, true);
  validate_registry_contents(registry_root);
  return operation(registry_root);
}

}  // namespace

bool AssetIdentityKey::operator==(const AssetIdentityKey & other) const noexcept
{
  return building_id == other.building_id &&
         floor_id == other.floor_id &&
         map_id == other.map_id;
}

bool AssetIdentity::operator==(const AssetIdentity & other) const noexcept
{
  return asset_epoch == other.asset_epoch &&
         asset_digest == other.asset_digest;
}

PersistentAssetEpochRegistry::PersistentAssetEpochRegistry(
  std::filesystem::path maps_root)
{
  if (maps_root.empty()) {
    throw std::invalid_argument("maps_root must not be empty");
  }
  std::error_code error;
  maps_root_ = std::filesystem::absolute(std::move(maps_root), error);
  if (error) {
    throw filesystem_error("resolve maps_root", maps_root, error);
  }
  maps_root_ = maps_root_.lexically_normal();
}

AssetIdentity PersistentAssetEpochRegistry::bind(
  const AssetIdentityKey & key,
  const std::string_view canonical_asset_digest)
{
  validate_key(key);
  if (!is_canonical_sha256_digest(canonical_asset_digest)) {
    throw std::invalid_argument(
            "asset_digest must be canonical lowercase sha256");
  }
  const std::string digest(canonical_asset_digest);

  return with_locked_registry(
    maps_root_,
    [&](const std::filesystem::path & registry_root) {
      auto snapshot = load_snapshot(registry_root);
      const auto maximum_manifest_epoch =
        maximum_committed_v2_manifest_epoch(maps_root_);
      if (snapshot.high_watermark < maximum_manifest_epoch) {
        throw std::runtime_error(
                "asset epoch registry high watermark regressed below a "
                "committed map manifest; refusing epoch reuse");
      }
      const auto existing = snapshot.bindings.find(key);
      if (existing != snapshot.bindings.end() &&
        existing->second.asset_digest == digest)
      {
        if (!snapshot.state_present) {
          atomic_write_file(
            registry_root / kStateFilename,
            serialize_state(snapshot.high_watermark));
        }
        return existing->second;
      }

      if (snapshot.high_watermark ==
        std::numeric_limits<std::uint64_t>::max())
      {
        throw std::overflow_error("asset_epoch uint64 space is exhausted");
      }
      const std::uint64_t next_epoch = snapshot.high_watermark + 1U;
      const AssetIdentity identity{next_epoch, digest};
      if (existing == snapshot.bindings.end() &&
        snapshot.bindings.size() >= kMaximumBindingCount)
      {
        throw std::runtime_error(
                "asset binding count would exceed the safety limit");
      }
      snapshot.bindings[key] = identity;
      const auto serialized_bindings =
        serialize_bindings(snapshot.bindings);

      // Persisting the high watermark first guarantees that a crash can only
      // leave a gap. The consumed epoch can never be silently reused.
      atomic_write_file(
        registry_root / kStateFilename,
        serialize_state(next_epoch));
      atomic_write_file(
        registry_root / kBindingsFilename,
        serialized_bindings);
      return identity;
    });
}

std::optional<AssetIdentity> PersistentAssetEpochRegistry::lookup(
  const AssetIdentityKey & key) const
{
  validate_key(key);
  return with_read_only_locked_registry(
    maps_root_,
    [&](const std::filesystem::path & registry_root)
    -> std::optional<AssetIdentity>
    {
      const auto snapshot = load_snapshot(registry_root);
      const auto maximum_manifest_epoch =
        maximum_committed_v2_manifest_epoch(maps_root_);
      if (snapshot.high_watermark < maximum_manifest_epoch) {
        throw std::runtime_error(
                "asset epoch registry high watermark regressed below a "
                "committed map manifest; read-only lookup refuses proof");
      }
      const auto found = snapshot.bindings.find(key);
      if (found == snapshot.bindings.end()) {
        return std::nullopt;
      }
      return found->second;
    });
}

}  // namespace robot_map_asset_identity
