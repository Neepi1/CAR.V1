#include "robot_elevator_manager/elevator_release_loader.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "robot_elevator_manager/elevator_topology_loader.hpp"

#ifndef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace robot_elevator_manager
{
namespace
{

namespace fs = std::filesystem;

constexpr std::uintmax_t kMaximumReleaseFileBytes = 2U * 1024U * 1024U;
constexpr std::size_t kMaximumElevators = 16U;
constexpr std::size_t kMaximumFloorsPerElevator = 64U;
constexpr std::size_t kMaximumFloorBindings = 128U;
constexpr std::size_t kRoleCount = 5U;

class LoadFailure : public std::runtime_error
{
public:
  LoadFailure(
    const ElevatorReleaseLoadError error,
    const std::string & message)
  : std::runtime_error(message), error(error)
  {
  }

  ElevatorReleaseLoadError error;
};

struct ReleaseFiles
{
  std::string configuration;
  std::string topology;
  std::string internal_poses;
  std::string validation;
  std::string manifest;
  std::string current;
};

#ifndef _WIN32
class UniqueFd
{
public:
  UniqueFd() = default;

  explicit UniqueFd(const int fd)
  : fd_(fd)
  {
  }

  ~UniqueFd()
  {
    reset();
  }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : fd_(other.release())
  {
  }

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  int get() const noexcept
  {
    return fd_;
  }

  explicit operator bool() const noexcept
  {
    return fd_ >= 0;
  }

  void reset(const int fd = -1) noexcept
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }

  int release() noexcept
  {
    const auto fd = fd_;
    fd_ = -1;
    return fd;
  }

private:
  int fd_{-1};
};
#endif

struct PinnedRelease
{
  std::string release_id;
#ifdef _WIN32
  fs::path release_root;
#else
  UniqueFd release_directory;
#endif
};

struct ConfigPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ConfigFloor
{
  std::string floor_id;
  std::string map_id;
  std::uint64_t map_asset_epoch{0U};
  std::string map_asset_digest;
  DoorThreshold threshold;
  std::map<PoseRole, ConfigPose> poses;
};

struct ConfigElevator
{
  std::string elevator_id;
  std::map<std::string, ConfigFloor> floors;
};

struct ParsedConfiguration
{
  std::string building_id;
  std::map<std::string, ConfigElevator> elevators;
  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>> bindings;
};

struct InternalPose
{
  std::string pose_id;
  std::string elevator_id;
  std::string floor_id;
  std::string map_id;
  PoseRole role{PoseRole::kHallCall};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

using InternalPoseKey =
  std::tuple<std::string, std::string, std::string, PoseRole>;

struct VerifiedMetadata
{
  std::uint64_t generation{0U};
  std::string configuration_digest;
};

ElevatorReleaseLoadResult failed(
  const ElevatorReleaseLoadError error,
  std::string message)
{
  return ElevatorReleaseLoadResult{
    error,
    std::move(message),
    std::nullopt,
  };
}

bool lower_hex(const std::string & value, const std::size_t size)
{
  return value.size() == size &&
         std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return (character >= '0' && character <= '9') ||
             (character >= 'a' && character <= 'f');
    });
}

bool canonical_sha256(const std::string & value)
{
  constexpr char prefix[] = "sha256:";
  constexpr std::size_t prefix_length = sizeof(prefix) - 1U;
  return value.size() == prefix_length + 64U &&
         value.compare(0U, prefix_length, prefix) == 0 &&
         lower_hex(value.substr(prefix_length), 64U);
}

bool valid_release_id(const std::string & value)
{
  constexpr std::size_t prefix_length = 16U;
  constexpr std::size_t generation_length = 6U;
  constexpr std::size_t digest_length = 12U;
  if (value.size() != prefix_length + generation_length + 1U + digest_length ||
    value.rfind("elevator-config-", 0U) != 0U ||
    value[prefix_length + generation_length] != '-')
  {
    return false;
  }
  const auto generation_begin =
    value.begin() + static_cast<std::ptrdiff_t>(prefix_length);
  const auto generation_end =
    generation_begin + static_cast<std::ptrdiff_t>(generation_length);
  return std::all_of(
    generation_begin, generation_end,
    [](const unsigned char character) {
      return character >= '0' && character <= '9';
    }) &&
         std::any_of(
    generation_begin, generation_end,
    [](const unsigned char character) {
      return character != '0';
    }) &&
         lower_hex(
    value.substr(prefix_length + generation_length + 1U),
    digest_length);
}

std::uint64_t release_generation(const std::string & value)
{
  if (!valid_release_id(value)) {
    return 0U;
  }
  constexpr std::size_t prefix_length = 16U;
  constexpr std::size_t generation_length = 6U;
  std::uint64_t generation = 0U;
  const auto begin = value.data() + prefix_length;
  const auto converted =
    std::from_chars(begin, begin + generation_length, generation);
  return converted.ec == std::errc{} ? generation : 0U;
}

bool valid_draft_revision(const std::string & value)
{
  return value.rfind("draft-v1-", 0U) == 0U &&
         lower_hex(value.substr(9U), 16U);
}

std::uint64_t fnv1a64(const std::string & value)
{
  // This intentionally preserves the historical robot_api_server release
  // identity.  The publisher shipped this non-standard offset basis on the
  // wire; "correcting" it to the canonical FNV-1a constant would invalidate
  // every published elevator release.
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= static_cast<std::uint64_t>(character);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string fixed_hex(const std::uint64_t value, const std::size_t width)
{
  std::ostringstream out;
  out << std::hex << std::nouppercase << std::setw(static_cast<int>(width))
      << std::setfill('0') << value;
  return out.str();
}

std::string release_id_for(
  const std::uint64_t generation,
  const ReleaseFiles & files)
{
  std::ostringstream prefix;
  prefix << "elevator-config-" << std::setw(6) << std::setfill('0')
         << generation << "-";
  const auto digest = fixed_hex(
    fnv1a64(
      std::to_string(generation) + "\n" + files.configuration +
      files.topology + files.internal_poses),
    16U);
  return prefix.str() + digest.substr(0U, 12U);
}

std::string node_string(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node || !node.IsScalar()) {
    throw LoadFailure(error, field + " must be a scalar string");
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception &) {
    throw LoadFailure(error, field + " must be a scalar string");
  }
}

std::optional<std::string> nullable_string(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node) {
    throw LoadFailure(error, field + " is required");
  }
  if (node.IsNull()) {
    return std::nullopt;
  }
  return node_string(node, field, error);
}

std::uint64_t node_uint(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  const auto text = node_string(node, field, error);
  if (text.empty() ||
    (text.size() > 1U && text.front() == '0'))
  {
    throw LoadFailure(error, field + " must be a canonical unsigned integer");
  }
  std::uint64_t result = 0U;
  const auto converted =
    std::from_chars(text.data(), text.data() + text.size(), result);
  if (converted.ec != std::errc{} ||
    converted.ptr != text.data() + text.size())
  {
    throw LoadFailure(error, field + " must be a canonical unsigned integer");
  }
  return result;
}

std::uint64_t node_positive_uint(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node || !node.IsScalar()) {
    throw LoadFailure(error, field + " must be a positive uint64 integer");
  }
  const auto tag = node.Tag();
  if (tag == "!" || tag == "tag:yaml.org,2002:str") {
    throw LoadFailure(
            error, field + " must be an unquoted positive uint64 integer");
  }
  const auto value = node_uint(node, field, error);
  if (value == 0U) {
    throw LoadFailure(error, field + " must be greater than zero");
  }
  return value;
}

bool node_bool(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  const auto text = node_string(node, field, error);
  if (text == "true") {
    return true;
  }
  if (text == "false") {
    return false;
  }
  throw LoadFailure(error, field + " must be a boolean");
}

double node_finite_double(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node || !node.IsScalar()) {
    throw LoadFailure(error, field + " must be a finite number");
  }
  try {
    const auto value = node.as<double>();
    if (!std::isfinite(value)) {
      throw LoadFailure(error, field + " must be a finite number");
    }
    return value;
  } catch (const YAML::Exception &) {
    throw LoadFailure(error, field + " must be a finite number");
  }
}

int schema_version(
  const YAML::Node & root,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  const auto value = node_uint(root["schema_version"], field, error);
  if (value != 1U) {
    throw LoadFailure(error, field + " must be 1");
  }
  return 1;
}

std::optional<PoseRole> parse_role(const std::string & role)
{
  if (role == "hall_call") {
    return PoseRole::kHallCall;
  }
  if (role == "hall_wait") {
    return PoseRole::kHallWait;
  }
  if (role == "doorway") {
    return PoseRole::kDoorway;
  }
  if (role == "cabin") {
    return PoseRole::kCabin;
  }
  if (role == "exit") {
    return PoseRole::kExit;
  }
  return std::nullopt;
}

Point2 parse_point(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node || !node.IsSequence() || node.size() != 2U) {
    throw LoadFailure(error, field + " must contain two finite numbers");
  }
  return Point2{
    node_finite_double(node[0], field + "[0]", error),
    node_finite_double(node[1], field + "[1]", error),
  };
}

DoorThreshold parse_threshold(
  const YAML::Node & node,
  const std::string & field,
  const ElevatorReleaseLoadError error)
{
  if (!node || !node.IsMap()) {
    throw LoadFailure(error, field + " must be a map");
  }
  return DoorThreshold{
    parse_point(node["left"], field + ".left", error),
    parse_point(node["right"], field + ".right", error),
    parse_point(node["cabin_reference"], field + ".cabin_reference", error),
    node_finite_double(node["clearance_m"], field + ".clearance_m", error),
    node_finite_double(
      node["jamb_clearance_m"], field + ".jamb_clearance_m", error),
  };
}

bool same_threshold(const DoorThreshold & left, const DoorThreshold & right)
{
  return left.left.x == right.left.x && left.left.y == right.left.y &&
         left.right.x == right.right.x && left.right.y == right.right.y &&
         left.cabin_reference.x == right.cabin_reference.x &&
         left.cabin_reference.y == right.cabin_reference.y &&
         left.clearance_m == right.clearance_m &&
         left.jamb_clearance_m == right.jamb_clearance_m;
}

double normalized_publisher_yaw(double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  yaw = std::remainder(yaw, 2.0 * kPi);
  if (yaw <= -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

bool same_publisher_yaw(const double configured, const double generated)
{
  // The publisher intentionally retains the reviewed configuration bytes but
  // emits normalized runtime poses. Compare their represented heading, not
  // their different-yet-equivalent numeric spellings.
  return std::abs(
    normalized_publisher_yaw(configured) -
    normalized_publisher_yaw(generated)) <= 1.0e-12;
}

#ifdef _WIN32
fs::path canonical_directory(
  const fs::path & path,
  const std::string & label)
{
  std::error_code status_error;
  const auto status = fs::symlink_status(path, status_error);
  if (status_error || status.type() != fs::file_type::directory) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            label + " must be a real directory");
  }
  std::error_code canonical_error;
  const auto canonical = fs::canonical(path, canonical_error);
  if (canonical_error) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            label + " cannot be canonicalized");
  }
  return canonical;
}

void require_real_path_chain(
  const fs::path & path,
  const std::string & label)
{
  if (!path.is_absolute()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            label + " must be absolute");
  }
  for (const auto & component : path.relative_path()) {
    if (component == "." || component == "..") {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              label + " contains a relative path component");
    }
  }

  fs::path current = path.root_path();
  for (const auto & component : path.relative_path()) {
    current /= component;
    std::error_code status_error;
    const auto status = fs::symlink_status(current, status_error);
    if (status_error) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              label + " contains an unreadable path component");
    }
    if (status.type() == fs::file_type::symlink) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              label + " traverses a symbolic link");
    }
  }
}

std::string read_release_file(
  const fs::path & release_root,
  const fs::path & canonical_release_root,
  const std::string & filename)
{
  const auto path = release_root / filename;
  std::error_code status_error;
  const auto status = fs::symlink_status(path, status_error);
  if (status_error || status.type() != fs::file_type::regular) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " must be a regular non-symlink file");
  }
  std::error_code size_error;
  const auto size = fs::file_size(path, size_error);
  if (size_error || size > kMaximumReleaseFileBytes) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " is unreadable or too large");
  }
  std::error_code canonical_error;
  const auto canonical_path = fs::canonical(path, canonical_error);
  if (canonical_error ||
    canonical_path.parent_path() != canonical_release_root)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " escapes the pinned release");
  }

  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " cannot be opened");
  }
  std::string value;
  value.resize(static_cast<std::size_t>(size));
  stream.read(value.data(), static_cast<std::streamsize>(size));
  if (!stream || stream.gcount() != static_cast<std::streamsize>(size)) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " could not be read completely");
  }
  return value;
}
#else
UniqueFd open_real_directory_chain(
  const fs::path & path,
  const std::string & label)
{
  if (!path.is_absolute()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            label + " must be absolute");
  }

  const auto root = path.root_path();
  UniqueFd current(
    ::open(
      root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (!current) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            label + " root cannot be opened safely");
  }

  for (const auto & component : path.relative_path()) {
    if (component.empty() || component == "." || component == "..") {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              label + " contains an unsafe path component");
    }
    const auto name = component.string();
    UniqueFd next(
      ::openat(
        current.get(), name.c_str(),
        O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
    if (!next) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              label + " contains a missing, non-directory, or symbolic link");
    }
    current = std::move(next);
  }
  return current;
}

UniqueFd open_managed_directory(
  const int parent_fd,
  const std::string & name,
  const std::string & label,
  const bool missing_is_not_found)
{
  UniqueFd directory(
    ::openat(
      parent_fd, name.c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (directory) {
    return directory;
  }
  if (missing_is_not_found && errno == ENOENT) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kReleaseNotFound,
            label + " does not exist");
  }
  throw LoadFailure(
          ElevatorReleaseLoadError::kUnsafePath,
          label + " must be a real directory");
}

std::string read_pinned_selector(const int config_root_fd)
{
  UniqueFd selector(
    ::openat(
      config_root_fd, "current", O_PATH | O_NOFOLLOW | O_CLOEXEC));
  if (!selector) {
    if (errno == ENOENT) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kReleaseNotFound,
              "current selector does not exist");
    }
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "current selector cannot be opened safely");
  }

  struct stat status {};
  if (::fstat(selector.get(), &status) != 0 || !S_ISLNK(status.st_mode)) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "current selector must be a symbolic link");
  }

  std::array<char, 4096> target_buffer {};
  const auto length = ::readlinkat(
    selector.get(), "", target_buffer.data(), target_buffer.size());
  if (length < 0 ||
    static_cast<std::size_t>(length) >= target_buffer.size())
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "current selector target cannot be read safely");
  }
  return std::string(
    target_buffer.data(), static_cast<std::size_t>(length));
}

bool same_file_snapshot(
  const struct stat & before,
  const struct stat & after)
{
  return before.st_dev == after.st_dev &&
         before.st_ino == after.st_ino &&
         before.st_mode == after.st_mode &&
         before.st_nlink == after.st_nlink &&
         before.st_size == after.st_size &&
         before.st_mtim.tv_sec == after.st_mtim.tv_sec &&
         before.st_mtim.tv_nsec == after.st_mtim.tv_nsec &&
         before.st_ctim.tv_sec == after.st_ctim.tv_sec &&
         before.st_ctim.tv_nsec == after.st_ctim.tv_nsec;
}

std::string read_release_file(
  const int release_directory_fd,
  const std::string & filename)
{
  UniqueFd file(
    ::openat(
      release_directory_fd, filename.c_str(),
      O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
  if (!file) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " cannot be opened safely");
  }

  struct stat before {};
  if (::fstat(file.get(), &before) != 0 ||
    !S_ISREG(before.st_mode) ||
    before.st_nlink != 1 ||
    before.st_size < 0 ||
    static_cast<std::uintmax_t>(before.st_size) >
    kMaximumReleaseFileBytes)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename +
            " must be one bounded, regular, single-link file");
  }

  std::string value;
  value.reserve(static_cast<std::size_t>(before.st_size));
  std::array<char, 64U * 1024U> buffer {};
  while (true) {
    const auto count = ::read(file.get(), buffer.data(), buffer.size());
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              "release file " + filename + " could not be read");
    }
    if (count == 0) {
      break;
    }
    if (value.size() + static_cast<std::size_t>(count) >
      kMaximumReleaseFileBytes)
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              "release file " + filename + " grew beyond its size limit");
    }
    value.append(buffer.data(), static_cast<std::size_t>(count));
  }

  struct stat after {};
  if (::fstat(file.get(), &after) != 0 ||
    !same_file_snapshot(before, after) ||
    value.size() != static_cast<std::size_t>(before.st_size))
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "release file " + filename + " changed while being read");
  }
  return value;
}
#endif

PinnedRelease pin_release(
  const ElevatorReleaseLoadRequest & request)
{
  if (request.config_root.empty()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidRequest,
            "config_root is required");
  }
  const fs::path requested_root(request.config_root);
  if (!requested_root.is_absolute()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidRequest,
            "config_root must be absolute");
  }
#ifdef _WIN32
  require_real_path_chain(
    requested_root, "elevator configuration root");
  const auto config_root =
    canonical_directory(requested_root, "elevator configuration root");
  const auto releases_root =
    canonical_directory(config_root / "releases", "elevator release store");
  if (releases_root.parent_path() != config_root) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "elevator release store escapes config_root");
  }

  std::string release_id = request.expected_release_id;
  const bool selected_from_current = release_id.empty();
  if (selected_from_current) {
    const auto selector = config_root / "current";
    std::error_code status_error;
    const auto selector_status = fs::symlink_status(selector, status_error);
    if (status_error || selector_status.type() != fs::file_type::symlink) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kReleaseNotFound,
              "current selector must be a symbolic link");
    }
    std::error_code target_error;
    const auto target = fs::read_symlink(selector, target_error);
    if (target_error || target.is_absolute() ||
      target.parent_path() != fs::path("releases"))
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              "current selector must target one direct managed release");
    }
    release_id = target.filename().string();
  }
  if (!valid_release_id(release_id)) {
    throw LoadFailure(
            selected_from_current ?
            ElevatorReleaseLoadError::kUnsafePath :
            ElevatorReleaseLoadError::kInvalidRequest,
            "release_id is not a valid immutable release identifier");
  }

  const auto release_root = releases_root / release_id;
  std::error_code release_status_error;
  const auto release_status =
    fs::symlink_status(release_root, release_status_error);
  if ((!release_status_error && !fs::exists(release_status)) ||
    release_status_error ==
    std::make_error_code(std::errc::no_such_file_or_directory))
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kReleaseNotFound,
            "pinned elevator release does not exist");
  }
  if (release_status_error) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "pinned elevator release cannot be inspected");
  }
  const auto canonical_release =
    canonical_directory(release_root, "pinned elevator release");
  if (canonical_release.parent_path() != releases_root ||
    canonical_release.filename() != fs::path(release_id))
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kUnsafePath,
            "pinned elevator release escapes the release store");
  }
  return PinnedRelease{release_id, canonical_release};
#else
  auto config_root = open_real_directory_chain(
    requested_root, "elevator configuration root");
  auto releases_root = open_managed_directory(
    config_root.get(), "releases", "elevator release store", false);

  std::string release_id = request.expected_release_id;
  const bool selected_from_current = release_id.empty();
  if (selected_from_current) {
    const auto target = read_pinned_selector(config_root.get());
    constexpr char kReleasePrefix[] = "releases/";
    if (target.rfind(kReleasePrefix, 0U) != 0U) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              "current selector must target one direct managed release");
    }
    release_id = target.substr(sizeof(kReleasePrefix) - 1U);
    if (target != std::string(kReleasePrefix) + release_id) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kUnsafePath,
              "current selector target is not canonical");
    }
  }
  if (!valid_release_id(release_id)) {
    throw LoadFailure(
            selected_from_current ?
            ElevatorReleaseLoadError::kUnsafePath :
            ElevatorReleaseLoadError::kInvalidRequest,
            "release_id is not a valid immutable release identifier");
  }

  auto release_directory = open_managed_directory(
    releases_root.get(), release_id, "pinned elevator release", true);
  return PinnedRelease{release_id, std::move(release_directory)};
#endif
}

ReleaseFiles read_release_files(const PinnedRelease & pinned)
{
#ifdef _WIN32
  const auto & release_root = pinned.release_root;
  return ReleaseFiles{
    read_release_file(release_root, release_root, "configuration.yaml"),
    read_release_file(release_root, release_root, "elevators.yaml"),
    read_release_file(
      release_root, release_root, "elevator_internal_poses.yaml"),
    read_release_file(release_root, release_root, "validation.json"),
    read_release_file(release_root, release_root, "manifest.json"),
    read_release_file(release_root, release_root, "current.json"),
  };
#else
  const auto release_directory = pinned.release_directory.get();
  return ReleaseFiles{
    read_release_file(release_directory, "configuration.yaml"),
    read_release_file(release_directory, "elevators.yaml"),
    read_release_file(release_directory, "elevator_internal_poses.yaml"),
    read_release_file(release_directory, "validation.json"),
    read_release_file(release_directory, "manifest.json"),
    read_release_file(release_directory, "current.json"),
  };
#endif
}

class StrictJsonValidator
{
public:
  StrictJsonValidator(
    const std::string & input,
    std::string label,
    const ElevatorReleaseLoadError error)
  : input_(input), label_(std::move(label)), error_(error)
  {
  }

  void validate()
  {
    skip_whitespace();
    parse_value(0U);
    skip_whitespace();
    if (position_ != input_.size()) {
      fail("contains trailing non-JSON content");
    }
  }

private:
  static bool decimal_digit(const char value)
  {
    return value >= '0' && value <= '9';
  }

  [[noreturn]] void fail(const std::string & reason) const
  {
    throw LoadFailure(error_, label_ + " " + reason);
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
      fail(std::string("expected '") + expected + "'");
    }
  }

  void parse_value(const std::size_t depth)
  {
    constexpr std::size_t kMaximumJsonDepth = 64U;
    if (depth > kMaximumJsonDepth) {
      fail("exceeds the maximum nesting depth");
    }
    skip_whitespace();
    if (position_ >= input_.size()) {
      fail("ends before a value");
    }
    switch (input_[position_]) {
      case '{':
        parse_object(depth);
        return;
      case '[':
        parse_array(depth);
        return;
      case '"':
        static_cast<void>(parse_string());
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
        if (input_[position_] == '-' || decimal_digit(input_[position_])) {
          parse_number();
          return;
        }
        fail("contains a token outside strict JSON");
    }
  }

  void parse_object(const std::size_t depth)
  {
    expect('{');
    skip_whitespace();
    if (consume('}')) {
      return;
    }

    std::set<std::string> keys;
    while (true) {
      skip_whitespace();
      if (position_ >= input_.size() || input_[position_] != '"') {
        fail("requires quoted object keys");
      }
      auto key = parse_string();
      if (!keys.insert(std::move(key)).second) {
        fail("contains a duplicate object key");
      }
      skip_whitespace();
      expect(':');
      parse_value(depth + 1U);
      skip_whitespace();
      if (consume('}')) {
        return;
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
      parse_value(depth + 1U);
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

  void parse_number()
  {
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
      while (position_ < input_.size() &&
        decimal_digit(input_[position_]))
      {
        ++position_;
      }
    }
    if (consume('.')) {
      if (position_ >= input_.size() ||
        !decimal_digit(input_[position_]))
      {
        fail("contains an incomplete fractional number");
      }
      while (position_ < input_.size() &&
        decimal_digit(input_[position_]))
      {
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
      if (position_ >= input_.size() ||
        !decimal_digit(input_[position_]))
      {
        fail("contains an incomplete exponent");
      }
      while (position_ < input_.size() &&
        decimal_digit(input_[position_]))
      {
        ++position_;
      }
    }
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
  std::string label_;
  ElevatorReleaseLoadError error_;
  std::size_t position_{0U};
};

YAML::Node parse_document(
  const std::string & text,
  const std::string & label,
  const ElevatorReleaseLoadError error)
{
  try {
    const auto root = YAML::Load(text);
    if (!root || !root.IsMap()) {
      throw LoadFailure(error, label + " root must be a map");
    }
    return root;
  } catch (const LoadFailure &) {
    throw;
  } catch (const YAML::Exception & exception) {
    throw LoadFailure(error, label + " is malformed: " + exception.what());
  }
}

YAML::Node parse_json_document(
  const std::string & text,
  const std::string & label,
  const ElevatorReleaseLoadError error)
{
  StrictJsonValidator(text, label, error).validate();
  return parse_document(text, label, error);
}

VerifiedMetadata verify_metadata(
  const std::string & expected_release_id,
  const ReleaseFiles & files)
{
  if (files.validation !=
    "{\"valid_for_publish\":true,\"issues\":[]}\n")
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "validation.json is not the fixed valid publication record");
  }

  const auto manifest = parse_json_document(
    files.manifest, "manifest.json",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current = parse_json_document(
    files.current, "current.json",
    ElevatorReleaseLoadError::kInvalidMetadata);
  schema_version(
    manifest, "manifest.schema_version",
    ElevatorReleaseLoadError::kInvalidMetadata);
  schema_version(
    current, "current.schema_version",
    ElevatorReleaseLoadError::kInvalidMetadata);

  const auto manifest_release = node_string(
    manifest["release_id"], "manifest.release_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current_release = node_string(
    current["release_id"], "current.release_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto manifest_generation = node_uint(
    manifest["generation"], "manifest.generation",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current_generation = node_uint(
    current["generation"], "current.generation",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto manifest_parent = nullable_string(
    manifest["parent_release_id"], "manifest.parent_release_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current_parent = nullable_string(
    current["parent_release_id"], "current.parent_release_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto manifest_actor = node_string(
    manifest["actor_id"], "manifest.actor_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current_actor = node_string(
    current["actor_id"], "current.actor_id",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto manifest_rollback = nullable_string(
    manifest["rollback_of"], "manifest.rollback_of",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto current_rollback = nullable_string(
    current["rollback_of"], "current.rollback_of",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto source_draft = nullable_string(
    manifest["source_draft_revision"], "manifest.source_draft_revision",
    ElevatorReleaseLoadError::kInvalidMetadata);

  if (manifest_release != expected_release_id ||
    current_release != expected_release_id ||
    manifest_generation == 0U ||
    manifest_generation != current_generation ||
    manifest_parent != current_parent ||
    manifest_actor != current_actor ||
    manifest_actor.size() > 256U ||
    manifest_rollback != current_rollback)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "manifest and current metadata identity disagree");
  }
  if ((manifest_generation == 1U && manifest_parent) ||
    (manifest_generation > 1U && !manifest_parent) ||
    (manifest_parent && !valid_release_id(*manifest_parent)) ||
    (manifest_parent && *manifest_parent == expected_release_id) ||
    (manifest_parent &&
    release_generation(*manifest_parent) + 1U != manifest_generation) ||
    (manifest_rollback && !valid_release_id(*manifest_rollback)) ||
    (manifest_rollback && *manifest_rollback == expected_release_id) ||
    (manifest_rollback &&
    release_generation(*manifest_rollback) >= manifest_generation) ||
    (source_draft && !valid_draft_revision(*source_draft)) ||
    (source_draft.has_value() == manifest_rollback.has_value()))
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "release lineage metadata is invalid");
  }
  if (!node_bool(
      manifest["asset_published"], "manifest.asset_published",
      ElevatorReleaseLoadError::kInvalidMetadata) ||
    node_bool(
      manifest["runtime_applied"], "manifest.runtime_applied",
      ElevatorReleaseLoadError::kInvalidMetadata))
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "release must be published but not runtime-applied");
  }
  if (node_string(
      manifest["created_at"], "manifest.created_at",
      ElevatorReleaseLoadError::kInvalidMetadata).empty() ||
    node_string(
      current["published_at"], "current.published_at",
      ElevatorReleaseLoadError::kInvalidMetadata).empty())
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "release timestamps are required");
  }

  const auto algorithm = node_string(
    manifest["configuration_digest_algorithm"],
    "manifest.configuration_digest_algorithm",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto recorded_digest = node_string(
    manifest["configuration_digest"],
    "manifest.configuration_digest",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto calculated_digest = fixed_hex(
    fnv1a64(
      files.configuration + files.topology + files.internal_poses),
    16U);
  if (algorithm != "fnv1a64" ||
    !lower_hex(recorded_digest, 16U) ||
    recorded_digest != calculated_digest ||
    release_id_for(manifest_generation, files) != expected_release_id)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidContent,
            "release content FNV identity is invalid");
  }
  return VerifiedMetadata{manifest_generation, recorded_digest};
}

ConfigPose parse_config_pose(
  const YAML::Node & node,
  const std::string & field)
{
  if (!node || !node.IsMap()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidConfiguration,
            field + " must be a map");
  }
  return ConfigPose{
    node_finite_double(
      node["x"], field + ".x",
      ElevatorReleaseLoadError::kInvalidConfiguration),
    node_finite_double(
      node["y"], field + ".y",
      ElevatorReleaseLoadError::kInvalidConfiguration),
    node_finite_double(
      node["yaw"], field + ".yaw",
      ElevatorReleaseLoadError::kInvalidConfiguration),
  };
}

ParsedConfiguration parse_configuration(
  const std::string & text,
  const std::string & expected_building)
{
  const auto root = parse_document(
    text, "configuration.yaml",
    ElevatorReleaseLoadError::kInvalidConfiguration);
  schema_version(
    root, "configuration.schema_version",
    ElevatorReleaseLoadError::kInvalidConfiguration);
  ParsedConfiguration result;
  result.building_id = node_string(
    root["building_id"], "configuration.building_id",
    ElevatorReleaseLoadError::kInvalidConfiguration);
  if (result.building_id != expected_building) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidConfiguration,
            "configuration building_id disagrees with the request");
  }
  const auto elevators = root["elevators"];
  if (!elevators || !elevators.IsSequence() || elevators.size() == 0U ||
    elevators.size() > kMaximumElevators)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidConfiguration,
            "configuration elevators count is invalid");
  }

  std::size_t total_bindings = 0U;
  for (std::size_t elevator_index = 0U;
    elevator_index < elevators.size(); ++elevator_index)
  {
    const auto elevator_node = elevators[elevator_index];
    if (!elevator_node.IsMap()) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidConfiguration,
              "configuration elevator must be a map");
    }
    ConfigElevator elevator;
    elevator.elevator_id = node_string(
      elevator_node["elevator_id"], "configuration.elevator_id",
      ElevatorReleaseLoadError::kInvalidConfiguration);
    if (!safe_asset_id(elevator.elevator_id)) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidConfiguration,
              "configuration elevator_id is unsafe");
    }
    const auto floors = elevator_node["floors"];
    if (!floors || !floors.IsSequence() || floors.size() == 0U ||
      floors.size() > kMaximumFloorsPerElevator)
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidConfiguration,
              "configuration floor count is invalid");
    }
    total_bindings += floors.size();
    if (total_bindings > kMaximumFloorBindings) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidConfiguration,
              "configuration has too many floor bindings");
    }

    for (std::size_t floor_index = 0U;
      floor_index < floors.size(); ++floor_index)
    {
      const auto floor_node = floors[floor_index];
      if (!floor_node.IsMap()) {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration floor must be a map");
      }
      ConfigFloor floor;
      floor.floor_id = node_string(
        floor_node["floor_id"], "configuration.floor_id",
        ElevatorReleaseLoadError::kInvalidConfiguration);
      floor.map_id = node_string(
        floor_node["map_id"], "configuration.map_id",
        ElevatorReleaseLoadError::kInvalidConfiguration);
      floor.map_asset_epoch = node_positive_uint(
        floor_node["map_asset_epoch"],
        "configuration.map_asset_epoch",
        ElevatorReleaseLoadError::kInvalidConfiguration);
      floor.map_asset_digest = node_string(
        floor_node["map_asset_digest"],
        "configuration.map_asset_digest",
        ElevatorReleaseLoadError::kInvalidConfiguration);
      if (!safe_asset_id(floor.floor_id) ||
        !safe_asset_id(floor.map_id) ||
        !canonical_sha256(floor.map_asset_digest))
      {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration floor identity or SHA-256 binding is invalid");
      }
      floor.threshold = parse_threshold(
        floor_node["threshold"], "configuration.threshold",
        ElevatorReleaseLoadError::kInvalidConfiguration);
      const auto poses = floor_node["poses"];
      if (!poses || !poses.IsMap()) {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration poses must be a map");
      }
      for (const auto & entry : poses) {
        const auto role_name = node_string(
          entry.first, "configuration.pose.role",
          ElevatorReleaseLoadError::kInvalidConfiguration);
        const auto role = parse_role(role_name);
        if (!role ||
          !floor.poses.emplace(
            *role,
            parse_config_pose(
              entry.second, "configuration.poses." + role_name)).second)
        {
          throw LoadFailure(
                  ElevatorReleaseLoadError::kInvalidConfiguration,
                  "configuration pose role is unknown or duplicated");
        }
      }
      if (floor.poses.size() != kRoleCount) {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration must contain all five pose roles");
      }

      const auto binding_key =
        std::make_pair(floor.floor_id, floor.map_id);
      const auto binding =
        result.bindings.emplace(
        binding_key,
        std::make_pair(
          floor.map_asset_epoch, floor.map_asset_digest));
      if (!binding.second &&
        binding.first->second !=
        std::make_pair(
          floor.map_asset_epoch, floor.map_asset_digest))
      {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration map binding identities conflict");
      }
      if (!elevator.floors.emplace(floor.floor_id, std::move(floor)).second) {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidConfiguration,
                "configuration floor_id is duplicated");
      }
    }
    if (!result.elevators.emplace(
        elevator.elevator_id, std::move(elevator)).second)
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidConfiguration,
              "configuration elevator_id is duplicated");
    }
  }
  return result;
}

std::map<
  std::pair<std::string, std::string>,
  std::pair<std::uint64_t, std::string>>
parse_manifest_bindings(const std::string & manifest_text)
{
  const auto manifest = parse_json_document(
    manifest_text, "manifest.json",
    ElevatorReleaseLoadError::kInvalidMetadata);
  const auto bindings = manifest["map_bindings"];
  if (!bindings || !bindings.IsSequence() ||
    bindings.size() == 0U ||
    bindings.size() > kMaximumFloorBindings)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidMetadata,
            "manifest map_bindings count is invalid");
  }
  std::map<
    std::pair<std::string, std::string>,
    std::pair<std::uint64_t, std::string>> result;
  for (const auto & binding : bindings) {
    if (!binding.IsMap()) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidMetadata,
              "manifest map binding must be a map");
    }
    const auto floor_id = node_string(
      binding["floor_id"], "manifest.map_bindings.floor_id",
      ElevatorReleaseLoadError::kInvalidMetadata);
    const auto map_id = node_string(
      binding["map_id"], "manifest.map_bindings.map_id",
      ElevatorReleaseLoadError::kInvalidMetadata);
    const auto asset_epoch = node_positive_uint(
      binding["asset_epoch"], "manifest.map_bindings.asset_epoch",
      ElevatorReleaseLoadError::kInvalidMetadata);
    const auto contract = node_string(
      binding["asset_digest_contract"],
      "manifest.map_bindings.asset_digest_contract",
      ElevatorReleaseLoadError::kInvalidMetadata);
    const auto algorithm = node_string(
      binding["asset_digest_algorithm"],
      "manifest.map_bindings.asset_digest_algorithm",
      ElevatorReleaseLoadError::kInvalidMetadata);
    const auto digest = node_string(
      binding["asset_digest"], "manifest.map_bindings.asset_digest",
      ElevatorReleaseLoadError::kInvalidMetadata);
    if (!safe_asset_id(floor_id) || !safe_asset_id(map_id) ||
      contract != "njrh-map-asset-bundle-v1" ||
      algorithm != "sha256" || !canonical_sha256(digest) ||
      !result.emplace(
        std::make_pair(floor_id, map_id),
        std::make_pair(asset_epoch, digest)).second)
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidMetadata,
              "manifest map binding is invalid or duplicated");
    }
  }
  return result;
}

ElevatorTopologyCatalog parse_topology(
  const std::string & text,
  const std::string & expected_building)
{
  const auto result = parse_topology_yaml(text);
  if (!result.ok() || !result.catalog) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidTopology,
            result.errors.empty() ?
            "elevators.yaml is invalid" : result.errors.front());
  }
  if (result.catalog->building_id != expected_building ||
    result.catalog->mock_ports_enabled ||
    result.catalog->elevators.empty() ||
    result.catalog->elevators.size() > kMaximumElevators)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidTopology,
            "topology building or runtime policy is invalid");
  }
  return *result.catalog;
}

void cross_check_configuration_and_topology(
  const ParsedConfiguration & configuration,
  const ElevatorTopologyCatalog & topology)
{
  if (configuration.elevators.size() != topology.elevators.size()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidTopology,
            "configuration and topology elevator sets disagree");
  }
  for (const auto & topology_elevator : topology.elevators) {
    const auto configured =
      configuration.elevators.find(topology_elevator.elevator_id);
    if (configured == configuration.elevators.end() ||
      configured->second.floors.size() != topology_elevator.floors.size())
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidTopology,
              "configuration and topology elevator floors disagree");
    }
    for (const auto & topology_floor : topology_elevator.floors) {
      const auto floor = configured->second.floors.find(topology_floor.floor_id);
      if (floor == configured->second.floors.end() ||
        floor->second.map_id != topology_floor.map_id ||
        !same_threshold(floor->second.threshold, topology_floor.threshold))
      {
        throw LoadFailure(
                ElevatorReleaseLoadError::kInvalidTopology,
                "configuration and topology floor content disagree");
      }
    }
  }
}

std::map<InternalPoseKey, InternalPose> parse_internal_poses(
  const std::string & text,
  const std::string & expected_building)
{
  const auto root = parse_document(
    text, "elevator_internal_poses.yaml",
    ElevatorReleaseLoadError::kInvalidInternalPoses);
  schema_version(
    root, "internal_poses.schema_version",
    ElevatorReleaseLoadError::kInvalidInternalPoses);
  const auto building = node_string(
    root["building_id"], "internal_poses.building_id",
    ElevatorReleaseLoadError::kInvalidInternalPoses);
  if (building != expected_building) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidInternalPoses,
            "internal pose building_id disagrees with the request");
  }
  const auto poses = root["poses"];
  if (!poses || !poses.IsSequence() ||
    poses.size() == 0U ||
    poses.size() > kMaximumFloorBindings * kRoleCount)
  {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidInternalPoses,
            "internal pose count is invalid");
  }

  std::map<InternalPoseKey, InternalPose> result;
  std::set<std::string> pose_ids;
  for (const auto & node : poses) {
    if (!node.IsMap()) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidInternalPoses,
              "internal pose must be a map");
    }
    InternalPose pose;
    pose.pose_id = node_string(
      node["pose_id"], "internal_pose.pose_id",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    pose.elevator_id = node_string(
      node["elevator_id"], "internal_pose.elevator_id",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    pose.floor_id = node_string(
      node["floor_id"], "internal_pose.floor_id",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    pose.map_id = node_string(
      node["map_id"], "internal_pose.map_id",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    const auto role_name = node_string(
      node["role"], "internal_pose.role",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    const auto role = parse_role(role_name);
    if (!role ||
      node_string(
        node["type"], "internal_pose.type",
        ElevatorReleaseLoadError::kInvalidInternalPoses) != "elevator_internal" ||
      !safe_pose_id(pose.pose_id) ||
      !safe_asset_id(pose.elevator_id) ||
      !safe_asset_id(pose.floor_id) ||
      !safe_asset_id(pose.map_id))
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidInternalPoses,
              "internal pose identity is invalid");
    }
    pose.role = *role;
    pose.x = node_finite_double(
      node["x"], "internal_pose.x",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    pose.y = node_finite_double(
      node["y"], "internal_pose.y",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    pose.yaw = node_finite_double(
      node["yaw"], "internal_pose.yaw",
      ElevatorReleaseLoadError::kInvalidInternalPoses);
    const InternalPoseKey key{
      pose.elevator_id, pose.floor_id, pose.map_id, pose.role};
    if (!pose_ids.insert(pose.pose_id).second ||
      !result.emplace(key, pose).second)
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidInternalPoses,
              "internal pose ID or role binding is duplicated");
    }
  }
  return result;
}

void cross_check_internal_poses(
  const ParsedConfiguration & configuration,
  const ElevatorTopologyCatalog & topology,
  const std::map<InternalPoseKey, InternalPose> & internal_poses)
{
  std::size_t expected_count = 0U;
  for (const auto & elevator : topology.elevators) {
    const auto configured_elevator =
      configuration.elevators.find(elevator.elevator_id);
    for (const auto & floor : elevator.floors) {
      const auto configured_floor =
        configured_elevator->second.floors.find(floor.floor_id);
      for (const auto role : required_pose_roles()) {
        ++expected_count;
        const auto topology_pose_id = find_pose_id(floor, role);
        const auto configured_pose = configured_floor->second.poses.find(role);
        const InternalPoseKey key{
          elevator.elevator_id, floor.floor_id, floor.map_id, role};
        const auto internal = internal_poses.find(key);
        if (!topology_pose_id ||
          configured_pose == configured_floor->second.poses.end() ||
          internal == internal_poses.end() ||
          internal->second.pose_id != *topology_pose_id ||
          internal->second.x != configured_pose->second.x ||
          internal->second.y != configured_pose->second.y ||
          !same_publisher_yaw(
            configured_pose->second.yaw, internal->second.yaw))
        {
          throw LoadFailure(
                  ElevatorReleaseLoadError::kInvalidInternalPoses,
                  "topology, configuration, and internal pose binding disagree");
        }
      }
    }
  }
  if (internal_poses.size() != expected_count) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kInvalidInternalPoses,
            "internal pose catalog contains missing or extra records");
  }
}

const ElevatorTopology & select_elevator(
  const ElevatorReleaseLoadRequest & request,
  const ElevatorTopologyCatalog & topology)
{
  std::vector<const ElevatorTopology *> candidates;
  for (const auto & elevator : topology.elevators) {
    if (resolve_route(
        elevator,
        request.source_floor_id,
        request.source_map_id,
        request.target_floor_id,
        request.target_map_id).ok())
    {
      candidates.push_back(&elevator);
    }
  }
  if (!request.preferred_elevator_id.empty()) {
    const auto selected = std::find_if(
      candidates.begin(), candidates.end(),
      [&request](const ElevatorTopology * elevator) {
        return elevator->elevator_id == request.preferred_elevator_id;
      });
    if (selected == candidates.end()) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kNoRoute,
              "preferred elevator does not serve the exact requested route");
    }
    return **selected;
  }
  if (candidates.empty()) {
    throw LoadFailure(
            ElevatorReleaseLoadError::kNoRoute,
            "no elevator serves the exact requested route");
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const ElevatorTopology * left, const ElevatorTopology * right) {
      return left->elevator_id < right->elevator_id;
    });
  return *candidates.front();
}

ElevatorRuntimeFloor make_runtime_floor(
  const std::string & elevator_id,
  const FloorElevatorTopology & topology_floor,
  const ConfigFloor & configured_floor,
  const std::map<InternalPoseKey, InternalPose> & internal_poses)
{
  ElevatorRuntimeFloor floor;
  floor.floor_id = topology_floor.floor_id;
  floor.map_id = topology_floor.map_id;
  floor.map_asset_epoch = configured_floor.map_asset_epoch;
  floor.map_asset_digest = configured_floor.map_asset_digest;
  floor.threshold = topology_floor.threshold;
  const auto & roles = required_pose_roles();
  for (std::size_t index = 0U; index < roles.size(); ++index) {
    const InternalPoseKey key{
      elevator_id, floor.floor_id, floor.map_id, roles[index]};
    const auto pose = internal_poses.find(key);
    if (pose == internal_poses.end()) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidInternalPoses,
              "selected runtime route has no internal pose");
    }
    floor.poses[index] = ElevatorRuntimePose{
      pose->second.role,
      pose->second.pose_id,
      pose->second.x,
      pose->second.y,
      pose->second.yaw,
    };
  }
  return floor;
}

}  // namespace

bool ElevatorReleaseLoadResult::ok() const noexcept
{
  return error == ElevatorReleaseLoadError::kNone && release.has_value();
}

ElevatorReleaseLoadResult load_elevator_release(
  const ElevatorReleaseLoadRequest & request)
{
  try {
    if (!safe_asset_id(request.building_id) ||
      !safe_asset_id(request.source_floor_id) ||
      !safe_asset_id(request.source_map_id) ||
      !safe_asset_id(request.target_floor_id) ||
      !safe_asset_id(request.target_map_id) ||
      (!request.preferred_elevator_id.empty() &&
      !safe_asset_id(request.preferred_elevator_id)))
    {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidRequest,
              "request identifiers must be bounded and path-safe");
    }

    const auto pinned = pin_release(request);
    const auto files = read_release_files(pinned);
    const auto metadata = verify_metadata(pinned.release_id, files);
    const auto configuration =
      parse_configuration(files.configuration, request.building_id);
    const auto manifest_bindings =
      parse_manifest_bindings(files.manifest);
    if (manifest_bindings != configuration.bindings) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kInvalidContent,
              "manifest and configuration map bindings disagree");
    }
    const auto topology =
      parse_topology(files.topology, request.building_id);
    cross_check_configuration_and_topology(configuration, topology);
    const auto internal_poses =
      parse_internal_poses(files.internal_poses, request.building_id);
    cross_check_internal_poses(configuration, topology, internal_poses);

    const auto & selected = select_elevator(request, topology);
    const auto route = resolve_route(
      selected,
      request.source_floor_id,
      request.source_map_id,
      request.target_floor_id,
      request.target_map_id);
    if (!route.ok() || !route.route) {
      throw LoadFailure(
              ElevatorReleaseLoadError::kNoRoute,
              "selected elevator route could not be frozen");
    }
    const auto configured_elevator =
      configuration.elevators.find(selected.elevator_id);
    const auto configured_source =
      configured_elevator->second.floors.find(route.route->source.floor_id);
    const auto configured_target =
      configured_elevator->second.floors.find(route.route->target.floor_id);

    FrozenElevatorRelease release;
    release.release_id = pinned.release_id;
    release.generation = metadata.generation;
    release.configuration_digest = metadata.configuration_digest;
    release.building_id = request.building_id;
    release.elevator_id = selected.elevator_id;
    release.source = make_runtime_floor(
      selected.elevator_id, route.route->source,
      configured_source->second, internal_poses);
    release.target = make_runtime_floor(
      selected.elevator_id, route.route->target,
      configured_target->second, internal_poses);
    return ElevatorReleaseLoadResult{
      ElevatorReleaseLoadError::kNone,
      "",
      std::move(release),
    };
  } catch (const LoadFailure & failure) {
    return failed(failure.error, failure.what());
  } catch (const std::exception & failure) {
    return failed(
      ElevatorReleaseLoadError::kInvalidContent,
      std::string("unexpected elevator release error: ") + failure.what());
  }
}

const char * to_string(const ElevatorReleaseLoadError error) noexcept
{
  switch (error) {
    case ElevatorReleaseLoadError::kNone:
      return "NONE";
    case ElevatorReleaseLoadError::kInvalidRequest:
      return "INVALID_REQUEST";
    case ElevatorReleaseLoadError::kUnsafePath:
      return "UNSAFE_PATH";
    case ElevatorReleaseLoadError::kReleaseNotFound:
      return "RELEASE_NOT_FOUND";
    case ElevatorReleaseLoadError::kInvalidMetadata:
      return "INVALID_METADATA";
    case ElevatorReleaseLoadError::kInvalidContent:
      return "INVALID_CONTENT";
    case ElevatorReleaseLoadError::kInvalidConfiguration:
      return "INVALID_CONFIGURATION";
    case ElevatorReleaseLoadError::kInvalidTopology:
      return "INVALID_TOPOLOGY";
    case ElevatorReleaseLoadError::kInvalidInternalPoses:
      return "INVALID_INTERNAL_POSES";
    case ElevatorReleaseLoadError::kNoRoute:
      return "NO_ROUTE";
  }
  return "INVALID_CONTENT";
}

}  // namespace robot_elevator_manager
