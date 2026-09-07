#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;
namespace
{

constexpr char kManifestSchema[] = "njrh.map_manifest.v2";
constexpr char kDigestAlgorithm[] = "sha256";
constexpr char kDigestContract[] = "njrh-map-asset-bundle-v1";

bool canonical_sha256_digest(const std::string & digest)
{
  constexpr std::size_t kPrefixLength = 7U;
  constexpr std::size_t kHexLength = 64U;
  if (digest.size() != kPrefixLength + kHexLength ||
    digest.compare(0U, kPrefixLength, "sha256:") != 0)
  {
    return false;
  }
  for (auto index = kPrefixLength; index < digest.size(); ++index) {
    const auto value = digest[index];
    if (!((value >= '0' && value <= '9') ||
      (value >= 'a' && value <= 'f')))
    {
      return false;
    }
  }
  return true;
}

void validate_v2_identity(const MapManifest & manifest)
{
  if (manifest.schema != kManifestSchema ||
    manifest.asset_epoch == 0U ||
    manifest.asset_digest_algorithm != kDigestAlgorithm ||
    manifest.asset_digest_contract != kDigestContract ||
    !canonical_sha256_digest(manifest.asset_digest))
  {
    throw std::invalid_argument(
            "map manifest v2 requires a complete canonical asset identity");
  }
}

enum class JsonScalarKind
{
  kString,
  kNumber,
  kBoolean,
  kOther,
};

struct JsonScalar
{
  JsonScalarKind kind{JsonScalarKind::kOther};
  std::string value;
};

using JsonObject = std::map<std::string, JsonScalar>;

class StrictJsonParseError : public std::runtime_error
{
public:
  explicit StrictJsonParseError(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

class StrictJsonObjectParser
{
public:
  explicit StrictJsonObjectParser(const std::string & input)
  : input_(input)
  {
  }

  JsonObject parse()
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
    throw StrictJsonParseError(reason);
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

  JsonScalar parse_value(const std::size_t depth)
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
        return {JsonScalarKind::kString, parse_string()};
      case 't':
        parse_literal("true");
        return {JsonScalarKind::kBoolean, "true"};
      case 'f':
        parse_literal("false");
        return {JsonScalarKind::kBoolean, "false"};
      case 'n':
        parse_literal("null");
        return {};
      default:
        if (input_[position_] == '-' || decimal_digit(input_[position_])) {
          return {JsonScalarKind::kNumber, parse_number()};
        }
        fail("contains a token outside strict JSON");
    }
  }

  JsonObject parse_object(const std::size_t depth)
  {
    expect('{');
    skip_whitespace();
    JsonObject values;
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

  void parse_literal(const std::string & literal)
  {
    if (input_.compare(position_, literal.size(), literal) != 0) {
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

const JsonScalar * find_field(
  const JsonObject & object,
  const std::string & key)
{
  const auto found = object.find(key);
  return found == object.end() ? nullptr : &found->second;
}

std::optional<std::string> string_field(
  const JsonObject & object,
  const std::string & key)
{
  const auto value = find_field(object, key);
  if (value == nullptr || value->kind != JsonScalarKind::kString) {
    return std::nullopt;
  }
  return value->value;
}

std::optional<std::uint64_t> strict_positive_uint64_value(
  const JsonScalar * scalar)
{
  if (scalar == nullptr || scalar->kind != JsonScalarKind::kNumber ||
    scalar->value.empty() ||
    scalar->value.front() < '1' || scalar->value.front() > '9')
  {
    return std::nullopt;
  }
  std::uint64_t value = 0U;
  for (const auto character : scalar->value) {
    if (character < '0' || character > '9') {
      return std::nullopt;
    }
    const auto digit = static_cast<std::uint64_t>(character - '0');
    if (value > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return std::nullopt;
    }
    value = value * 10U + digit;
  }
  return value;
}

std::string read_text_file(const fs::path & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open file for reading: " + path.string());
  }
  std::ostringstream data;
  data << file.rdbuf();
  return data.str();
}

std::atomic<std::uint64_t> temporary_file_counter{0U};

void refuse_non_regular_destination(const fs::path & path)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error) {
    if (error == std::errc::no_such_file_or_directory) {
      return;
    }
    throw std::runtime_error(
            "failed to inspect map manifest destination: " +
            path.string() + ": " + error.message());
  }
  if (fs::exists(status) && status.type() != fs::file_type::regular) {
    throw std::runtime_error(
            "map manifest destination must be a regular file or absent: " +
            path.string());
  }
}

fs::path make_temporary_path(const fs::path & path)
{
#ifdef _WIN32
  const auto process_id = static_cast<std::uint64_t>(::GetCurrentProcessId());
#else
  const auto process_id = static_cast<std::uint64_t>(::getpid());
#endif
  const auto sequence =
    temporary_file_counter.fetch_add(1U, std::memory_order_relaxed);
  return path.parent_path() /
         ("." + path.filename().string() + ".tmp." +
         std::to_string(process_id) + "." + std::to_string(sequence));
}

#ifdef _WIN32
void write_text_file_atomic(const fs::path & path, const std::string & text)
{
  const auto parent =
    path.parent_path().empty() ? fs::path{"."} : path.parent_path();
  fs::create_directories(parent);
  refuse_non_regular_destination(path);

  fs::path temporary_path;
  HANDLE handle = INVALID_HANDLE_VALUE;
  for (std::size_t attempt = 0U; attempt < 128U; ++attempt) {
    temporary_path = make_temporary_path(path);
    handle = ::CreateFileW(
      temporary_path.c_str(),
      GENERIC_WRITE,
      0,
      nullptr,
      CREATE_NEW,
      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH,
      nullptr);
    if (handle != INVALID_HANDLE_VALUE) {
      break;
    }
    if (::GetLastError() != ERROR_FILE_EXISTS &&
      ::GetLastError() != ERROR_ALREADY_EXISTS)
    {
      throw std::runtime_error(
              "failed to create temporary map manifest: " +
              std::to_string(::GetLastError()));
    }
  }
  if (handle == INVALID_HANDLE_VALUE) {
    throw std::runtime_error(
            "failed to allocate a unique temporary map manifest");
  }

  try {
    std::size_t offset = 0U;
    while (offset < text.size()) {
      const auto remaining = text.size() - offset;
      const auto chunk = static_cast<DWORD>(
        (std::min)(
          remaining,
          static_cast<std::size_t>(
            std::numeric_limits<DWORD>::max())));
      DWORD written = 0U;
      if (!::WriteFile(
          handle,
          text.data() + offset,
          chunk,
          &written,
          nullptr) ||
        written == 0U)
      {
        throw std::runtime_error(
                "failed to write temporary map manifest: " +
                std::to_string(::GetLastError()));
      }
      offset += written;
    }
    if (!::FlushFileBuffers(handle)) {
      throw std::runtime_error(
              "failed to flush temporary map manifest: " +
              std::to_string(::GetLastError()));
    }
    if (!::CloseHandle(handle)) {
      handle = INVALID_HANDLE_VALUE;
      throw std::runtime_error(
              "failed to close temporary map manifest: " +
              std::to_string(::GetLastError()));
    }
    handle = INVALID_HANDLE_VALUE;

    if (!::MoveFileExW(
        temporary_path.c_str(),
        path.c_str(),
        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    {
      throw std::runtime_error(
              "failed to atomically replace map manifest: " +
              std::to_string(::GetLastError()));
    }
  } catch (...) {
    if (handle != INVALID_HANDLE_VALUE) {
      ::CloseHandle(handle);
    }
    std::error_code ignored;
    fs::remove(temporary_path, ignored);
    throw;
  }
}
#else
void write_text_file_atomic(const fs::path & path, const std::string & text)
{
  const auto parent =
    path.parent_path().empty() ? fs::path{"."} : path.parent_path();
  fs::create_directories(parent);
  refuse_non_regular_destination(path);

  fs::path temporary_path;
  int descriptor = -1;
  for (std::size_t attempt = 0U; attempt < 128U; ++attempt) {
    temporary_path = make_temporary_path(path);
    descriptor = ::open(
      temporary_path.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (descriptor >= 0) {
      break;
    }
    if (errno != EEXIST) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to create temporary map manifest");
    }
  }
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to allocate a unique temporary map manifest");
  }

  try {
    struct stat status {};
    if (::fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode)) {
      throw std::runtime_error(
              "temporary map manifest is not a regular file");
    }

    std::size_t offset = 0U;
    while (offset < text.size()) {
      const auto written = ::write(
        descriptor,
        text.data() + offset,
        text.size() - offset);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::system_error(
                errno,
                std::generic_category(),
                "failed to write temporary map manifest");
      }
      if (written == 0) {
        throw std::runtime_error(
                "temporary map manifest write made no progress");
      }
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor) != 0) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to flush temporary map manifest");
    }
    if (::close(descriptor) != 0) {
      descriptor = -1;
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to close temporary map manifest");
    }
    descriptor = -1;

    if (::rename(temporary_path.c_str(), path.c_str()) != 0) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to atomically replace map manifest");
    }

    const auto directory_descriptor = ::open(
      parent.c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory_descriptor < 0) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to open map manifest parent directory");
    }
    if (::fsync(directory_descriptor) != 0) {
      const auto saved_errno = errno;
      ::close(directory_descriptor);
      throw std::system_error(
              saved_errno,
              std::generic_category(),
              "failed to flush map manifest parent directory");
    }
    if (::close(directory_descriptor) != 0) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "failed to close map manifest parent directory");
    }
  } catch (...) {
    if (descriptor >= 0) {
      ::close(descriptor);
    }
    std::error_code ignored;
    fs::remove(temporary_path, ignored);
    throw;
  }
}
#endif

}  // namespace

void fill_manifest_paths(MapManifest & manifest)
{
  manifest.manifest_json = manifest.root / "manifest.json";
  manifest.nav_map_yaml = manifest.root / "nav" / (manifest.safe_map_name + ".yaml");
  manifest.nav_map_pgm = manifest.root / "nav" / (manifest.safe_map_name + ".pgm");
  manifest.localizer_map_png = manifest.root / "localizer" / (manifest.safe_map_name + ".png");
  manifest.localizer_params_yaml = manifest.root / "localizer" / (manifest.safe_map_name + ".yaml");
  manifest.keepout_mask_yaml = manifest.root / "filters" / "keepout_mask.yaml";
  manifest.keepout_mask_pgm = manifest.root / "filters" / "keepout_mask.pgm";
  manifest.speed_mask_yaml = manifest.root / "filters" / "speed_mask.yaml";
  manifest.speed_mask_pgm = manifest.root / "filters" / "speed_mask.pgm";
  manifest.binary_mask_yaml = manifest.root / "filters" / "binary_mask.yaml";
  manifest.binary_mask_pgm = manifest.root / "filters" / "binary_mask.pgm";
  manifest.asset_report_json = manifest.root / "reports" / "asset_report.json";
  manifest.poses_yaml = manifest.root / "poses.yaml";
}

std::string map_manifest_json(const MapManifest & manifest)
{
  validate_v2_identity(manifest);
  std::ostringstream out;
  out << "{\n"
      << "  \"schema\": " << json_string(kManifestSchema) << ",\n"
      << "  \"asset_epoch\": " << manifest.asset_epoch << ",\n"
      << "  \"asset_digest_algorithm\": " << json_string(kDigestAlgorithm) << ",\n"
      << "  \"asset_digest_contract\": " << json_string(kDigestContract) << ",\n"
      << "  \"asset_digest\": " << json_string(manifest.asset_digest) << ",\n"
      << "  \"map_id\": " << json_string(manifest.map_id) << ",\n"
      << "  \"display_name\": " << json_string(manifest.display_name) << ",\n"
      << "  \"map_name\": " << json_string(manifest.display_name) << ",\n"
      << "  \"safe_map_name\": " << json_string(manifest.safe_map_name) << ",\n"
      << "  \"building_id\": " << json_string(manifest.building_id) << ",\n"
      << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n"
      << "  \"created_at\": " << json_string(manifest.created_at) << ",\n"
      << "  \"active\": " << (manifest.active ? "true" : "false") << ",\n"
      << "  \"assets\": {\n"
      << "    \"nav_map_yaml\": " << json_string(manifest.nav_map_yaml.string()) << ",\n"
      << "    \"nav_map_pgm\": " << json_string(manifest.nav_map_pgm.string()) << ",\n"
      << "    \"localizer_map_png\": " << json_string(manifest.localizer_map_png.string()) << ",\n"
      << "    \"localizer_params_yaml\": " << json_string(manifest.localizer_params_yaml.string()) << ",\n"
      << "    \"keepout_mask_yaml\": " << json_string(manifest.keepout_mask_yaml.string()) << ",\n"
      << "    \"keepout_mask_pgm\": " << json_string(manifest.keepout_mask_pgm.string()) << ",\n"
      << "    \"speed_mask_yaml\": " << json_string(manifest.speed_mask_yaml.string()) << ",\n"
      << "    \"speed_mask_pgm\": " << json_string(manifest.speed_mask_pgm.string()) << ",\n"
      << "    \"binary_mask_yaml\": " << json_string(manifest.binary_mask_yaml.string()) << ",\n"
      << "    \"binary_mask_pgm\": " << json_string(manifest.binary_mask_pgm.string()) << ",\n"
      << "    \"asset_report_json\": " << json_string(manifest.asset_report_json.string()) << ",\n"
      << "    \"poses_yaml\": " << json_string(manifest.poses_yaml.string()) << "\n"
      << "  }\n"
      << "}\n";
  return out.str();
}

std::optional<MapManifest> parse_map_manifest_json(
  const std::string & text,
  const fs::path & manifest_path)
{
  JsonObject root;
  try {
    root = StrictJsonObjectParser(text).parse();
  } catch (const StrictJsonParseError &) {
    return std::nullopt;
  }

  MapManifest manifest;
  const auto schema = string_field(root, "schema");
  if (schema.has_value()) {
    if (*schema != kManifestSchema) {
      return std::nullopt;
    }
    const auto asset_epoch =
      strict_positive_uint64_value(find_field(root, "asset_epoch"));
    const auto asset_digest_algorithm =
      string_field(root, "asset_digest_algorithm");
    const auto asset_digest_contract =
      string_field(root, "asset_digest_contract");
    const auto asset_digest = string_field(root, "asset_digest");
    if (!asset_epoch ||
      !asset_digest_algorithm ||
      *asset_digest_algorithm != kDigestAlgorithm ||
      !asset_digest_contract ||
      *asset_digest_contract != kDigestContract ||
      !asset_digest ||
      !canonical_sha256_digest(*asset_digest))
    {
      return std::nullopt;
    }
    manifest.schema = *schema;
    manifest.asset_epoch = *asset_epoch;
    manifest.asset_digest_algorithm = *asset_digest_algorithm;
    manifest.asset_digest_contract = *asset_digest_contract;
    manifest.asset_digest = *asset_digest;
  } else if (
    find_field(root, "schema") != nullptr ||
    find_field(root, "asset_epoch") != nullptr ||
    find_field(root, "asset_digest_algorithm") != nullptr ||
    find_field(root, "asset_digest_contract") != nullptr ||
    find_field(root, "asset_digest") != nullptr)
  {
    return std::nullopt;
  }
  const auto map_id = string_field(root, "map_id");
  const auto building_id = string_field(root, "building_id");
  const auto floor_id = string_field(root, "floor_id");
  if (!map_id || !building_id || !floor_id || !safe_asset_id(*map_id) ||
    !safe_asset_id(*building_id) || !safe_asset_id(*floor_id))
  {
    return std::nullopt;
  }
  manifest.map_id = *map_id;
  manifest.display_name = string_field(root, "display_name").value_or(
    string_field(root, "map_name").value_or(*map_id));
  manifest.safe_map_name = string_field(root, "safe_map_name").value_or(
    safe_file_stem_from_display_name(manifest.display_name));
  if (!safe_asset_id(manifest.safe_map_name)) {
    return std::nullopt;
  }
  manifest.building_id = *building_id;
  manifest.floor_id = *floor_id;
  manifest.created_at = string_field(root, "created_at").value_or("");
  const auto active = find_field(root, "active");
  if (active != nullptr) {
    if (active->kind != JsonScalarKind::kBoolean) {
      return std::nullopt;
    }
    manifest.active = active->value == "true";
  }
  manifest.root = manifest_path.parent_path();
  fill_manifest_paths(manifest);
  return manifest;
}

std::optional<MapManifest> read_map_manifest(const fs::path & manifest_path)
{
  constexpr std::uintmax_t kMaximumManifestBytes = 1024U * 1024U;
  std::error_code error;
  const auto status = fs::symlink_status(manifest_path, error);
  if (error || status.type() != fs::file_type::regular) {
    return std::nullopt;
  }
  const auto size = fs::file_size(manifest_path, error);
  if (error || size > kMaximumManifestBytes) {
    return std::nullopt;
  }
  return parse_map_manifest_json(read_text_file(manifest_path), manifest_path);
}

void write_map_manifest(const MapManifest & manifest)
{
  write_text_file_atomic(
    manifest.manifest_json,
    map_manifest_json(manifest));
}

}  // namespace robot_api_server
