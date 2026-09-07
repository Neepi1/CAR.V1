#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace robot_api_server::features::maps
{

namespace fs = std::filesystem;

bool path_lexically_within(
  const fs::path & child,
  const fs::path & parent)
{
  try {
    const auto normalized_child = fs::absolute(child).lexically_normal();
    const auto normalized_parent = fs::absolute(parent).lexically_normal();
    auto child_iterator = normalized_child.begin();
    for (auto parent_iterator = normalized_parent.begin();
      parent_iterator != normalized_parent.end();
      ++parent_iterator, ++child_iterator)
    {
      if (child_iterator == normalized_child.end() ||
        *child_iterator != *parent_iterator)
      {
        return false;
      }
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool same_normalized_path(
  const fs::path & left,
  const fs::path & right)
{
  try {
    return fs::absolute(left).lexically_normal() ==
           fs::absolute(right).lexically_normal();
  } catch (const std::exception &) {
    return false;
  }
}

bool same_exact_map_asset_source(
  const MapManifest & left,
  const MapManifest & right)
{
  // active is selection state changed by this transaction, not source identity.
  const bool metadata_same =
    left.schema == right.schema &&
    left.building_id == right.building_id &&
    left.floor_id == right.floor_id &&
    left.map_id == right.map_id &&
    left.display_name == right.display_name &&
    left.safe_map_name == right.safe_map_name &&
    left.created_at == right.created_at &&
    left.asset_epoch == right.asset_epoch &&
    left.asset_digest_algorithm == right.asset_digest_algorithm &&
    left.asset_digest_contract == right.asset_digest_contract &&
    left.asset_digest == right.asset_digest;
  const bool paths_same =
    same_normalized_path(left.root, right.root) &&
    same_normalized_path(left.manifest_json, right.manifest_json) &&
    same_normalized_path(left.nav_map_yaml, right.nav_map_yaml) &&
    same_normalized_path(left.nav_map_pgm, right.nav_map_pgm) &&
    same_normalized_path(left.localizer_map_png, right.localizer_map_png) &&
    same_normalized_path(
      left.localizer_params_yaml, right.localizer_params_yaml) &&
    same_normalized_path(left.keepout_mask_yaml, right.keepout_mask_yaml) &&
    same_normalized_path(left.keepout_mask_pgm, right.keepout_mask_pgm) &&
    same_normalized_path(left.speed_mask_yaml, right.speed_mask_yaml) &&
    same_normalized_path(left.speed_mask_pgm, right.speed_mask_pgm) &&
    same_normalized_path(left.binary_mask_yaml, right.binary_mask_yaml) &&
    same_normalized_path(left.binary_mask_pgm, right.binary_mask_pgm) &&
    same_normalized_path(left.asset_report_json, right.asset_report_json) &&
    same_normalized_path(left.poses_yaml, right.poses_yaml);
  return metadata_same && paths_same;
}

ExactMapAssetSourceDrift::ExactMapAssetSourceDrift(
  const std::string & message)
: std::runtime_error(message)
{
}

bool safe_bundle_directory(
  const fs::path & directory,
  const fs::path & managed_root)
{
  try {
    if (!path_lexically_within(directory, managed_root)) {
      return false;
    }
    const auto normalized_root =
      fs::absolute(managed_root).lexically_normal();
    const auto normalized_directory =
      fs::absolute(directory).lexically_normal();
    std::error_code error;
    auto cursor = normalized_root;
    auto status = fs::symlink_status(cursor, error);
    if (error || status.type() == fs::file_type::symlink ||
      status.type() != fs::file_type::directory)
    {
      return false;
    }
    const auto relative =
      normalized_directory.lexically_relative(normalized_root);
    for (const auto & component : relative) {
      if (component == "..") {
        return false;
      }
      cursor /= component;
      status = fs::symlink_status(cursor, error);
      if (error || status.type() == fs::file_type::symlink ||
        status.type() != fs::file_type::directory)
      {
        return false;
      }
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void durable_sync_directory(const fs::path & directory)
{
  const int descriptor =
    ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to open directory for fsync: " + directory.string() +
            " errno=" + std::to_string(errno));
  }
  const int result = ::fsync(descriptor);
  const int saved_errno = errno;
  ::close(descriptor);
  if (result != 0) {
    throw std::runtime_error(
            "failed to fsync directory: " + directory.string() +
            " errno=" + std::to_string(saved_errno));
  }
}

void durable_sync_regular_file(const fs::path & path)
{
  const int descriptor =
    ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to open file for fsync: " + path.string() +
            " errno=" + std::to_string(errno));
  }
  struct stat status {};
  if (::fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode)) {
    const int saved_errno = errno;
    ::close(descriptor);
    throw std::runtime_error(
            "refusing to fsync a non-regular projection file: " +
            path.string() + " errno=" + std::to_string(saved_errno));
  }
  const int result = ::fsync(descriptor);
  const int saved_errno = errno;
  const int close_result = ::close(descriptor);
  const int close_errno = errno;
  if (result != 0) {
    throw std::runtime_error(
            "failed to fsync projection file: " + path.string() +
            " errno=" + std::to_string(saved_errno));
  }
  if (close_result != 0) {
    throw std::runtime_error(
            "failed to close projection file after fsync: " +
            path.string() + " errno=" + std::to_string(close_errno));
  }
}

std::string read_regular_text_file_checked(
  const fs::path & path,
  const std::size_t maximum_size)
{
  int descriptor = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to open projection source: " + path.string() +
            " errno=" + std::to_string(errno));
  }
  try {
    struct stat before {};
    if (::fstat(descriptor, &before) != 0 || !S_ISREG(before.st_mode) ||
      before.st_size < 0 ||
      static_cast<std::uint64_t>(before.st_size) >
      static_cast<std::uint64_t>(maximum_size))
    {
      throw std::runtime_error(
              "projection source is not a bounded regular file: " +
              path.string());
    }
    std::string content(static_cast<std::size_t>(before.st_size), '\0');
    std::size_t offset = 0U;
    while (offset < content.size()) {
      const auto count =
        ::read(descriptor, content.data() + offset, content.size() - offset);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(
                "failed to read projection source: " + path.string() +
                " errno=" + std::to_string(errno));
      }
      if (count == 0) {
        throw std::runtime_error(
                "projection source was truncated while reading: " +
                path.string());
      }
      offset += static_cast<std::size_t>(count);
    }
    char extra = '\0';
    ssize_t extra_count = -1;
    do {
      extra_count = ::read(descriptor, &extra, 1U);
    } while (extra_count < 0 && errno == EINTR);
    if (extra_count < 0) {
      throw std::runtime_error(
              "failed to finish reading projection source: " +
              path.string() + " errno=" + std::to_string(errno));
    }
    struct stat after {};
    if (extra_count != 0 || ::fstat(descriptor, &after) != 0 ||
      before.st_dev != after.st_dev || before.st_ino != after.st_ino ||
      before.st_size != after.st_size ||
      before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
      before.st_mtim.tv_nsec != after.st_mtim.tv_nsec)
    {
      throw std::runtime_error(
              "projection source changed while reading: " + path.string());
    }
    const int close_result = ::close(descriptor);
    descriptor = -1;
    if (close_result != 0) {
      throw std::runtime_error(
              "failed to close projection source: " + path.string() +
              " errno=" + std::to_string(errno));
    }
    return content;
  } catch (...) {
    if (descriptor >= 0) {
      (void)::close(descriptor);
    }
    throw;
  }
}

bool regular_file_contents_equal_checked(
  const fs::path & left_path,
  const fs::path & right_path,
  const std::uintmax_t maximum_size,
  std::string & error)
{
  int left = -1;
  int right = -1;
  const auto close_descriptors = [&]() {
      if (left >= 0) {
        (void)::close(left);
        left = -1;
      }
      if (right >= 0) {
        (void)::close(right);
        right = -1;
      }
    };
  const auto fail = [&](const std::string & detail) {
      error = detail;
      close_descriptors();
      return false;
    };

  left = ::open(left_path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (left < 0) {
    return fail(
      "failed to open source projection file: " + left_path.string() +
      " errno=" + std::to_string(errno));
  }
  right = ::open(right_path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (right < 0) {
    return fail(
      "failed to open current projection file: " + right_path.string() +
      " errno=" + std::to_string(errno));
  }

  struct stat left_before {};
  struct stat right_before {};
  if (::fstat(left, &left_before) != 0 ||
    ::fstat(right, &right_before) != 0 ||
    !S_ISREG(left_before.st_mode) ||
    !S_ISREG(right_before.st_mode) ||
    left_before.st_nlink != 1 ||
    right_before.st_nlink != 1 ||
    left_before.st_size < 0 ||
    right_before.st_size < 0)
  {
    return fail("map projection comparison requires single-link regular files");
  }
  if (left_before.st_size != right_before.st_size) {
    return fail(
      "current projection size differs from source: " + right_path.string());
  }
  const auto file_size = static_cast<std::uintmax_t>(left_before.st_size);
  if (file_size > maximum_size) {
    return fail(
      "map projection exceeds comparison size limit: " + left_path.string());
  }

  const auto read_exact =
    [](const int descriptor, char * buffer, const std::size_t size) {
      std::size_t offset = 0U;
      while (offset < size) {
        const auto count = ::read(descriptor, buffer + offset, size - offset);
        if (count < 0 && errno == EINTR) {
          continue;
        }
        if (count <= 0) {
          return false;
        }
        offset += static_cast<std::size_t>(count);
      }
      return true;
    };
  std::array<char, 64U * 1024U> left_buffer {};
  std::array<char, 64U * 1024U> right_buffer {};
  std::uintmax_t remaining = file_size;
  while (remaining > 0U) {
    const auto requested = static_cast<std::size_t>(
      std::min<std::uintmax_t>(remaining, left_buffer.size()));
    if (!read_exact(left, left_buffer.data(), requested) ||
      !read_exact(right, right_buffer.data(), requested))
    {
      return fail("map projection changed or became unreadable during comparison");
    }
    if (std::memcmp(left_buffer.data(), right_buffer.data(), requested) != 0) {
      return fail(
        "current projection content differs from source: " + right_path.string());
    }
    remaining -= requested;
  }

  char left_extra = '\0';
  char right_extra = '\0';
  const auto read_extra = [](const int descriptor, char & value) {
      ssize_t count = -1;
      do {
        count = ::read(descriptor, &value, 1U);
      } while (count < 0 && errno == EINTR);
      return count;
    };
  if (read_extra(left, left_extra) != 0 ||
    read_extra(right, right_extra) != 0)
  {
    return fail("map projection size changed during comparison");
  }

  struct stat left_after {};
  struct stat right_after {};
  const auto unchanged = [](const struct stat & before, const struct stat & after) {
      return before.st_dev == after.st_dev &&
             before.st_ino == after.st_ino &&
             before.st_size == after.st_size &&
             before.st_mtim.tv_sec == after.st_mtim.tv_sec &&
             before.st_mtim.tv_nsec == after.st_mtim.tv_nsec;
    };
  if (::fstat(left, &left_after) != 0 ||
    ::fstat(right, &right_after) != 0 ||
    !unchanged(left_before, left_after) ||
    !unchanged(right_before, right_after))
  {
    return fail("map projection changed during comparison");
  }
  if (::close(left) != 0) {
    left = -1;
    return fail(
      "failed to close source projection file: " + left_path.string());
  }
  left = -1;
  if (::close(right) != 0) {
    right = -1;
    return fail(
      "failed to close current projection file: " + right_path.string());
  }
  right = -1;
  return true;
}

void durable_write_text_file_atomic(
  const fs::path & path,
  const std::string & content,
  const mode_t mode)
{
  static std::atomic<std::uint64_t> sequence{0U};
  const auto parent = path.parent_path();
  if (parent.empty() || !fs::is_directory(parent)) {
    throw std::runtime_error(
            "durable file parent is not an existing directory: " +
            parent.string());
  }
  const auto temporary =
    parent /
    ("." + path.filename().string() + ".tmp." +
    std::to_string(::getpid()) + "." +
    std::to_string(sequence.fetch_add(1U)));
  int descriptor = ::open(
    temporary.c_str(),
    O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
    mode);
  if (descriptor < 0) {
    throw std::runtime_error(
            "failed to create durable temporary file: " +
            temporary.string() + " errno=" + std::to_string(errno));
  }
  bool renamed = false;
  try {
    std::size_t offset = 0U;
    while (offset < content.size()) {
      const auto count = ::write(
        descriptor, content.data() + offset, content.size() - offset);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(
                "failed to write durable temporary file: " +
                temporary.string() + " errno=" + std::to_string(errno));
      }
      if (count == 0) {
        throw std::runtime_error(
                "durable temporary file write made no progress: " +
                temporary.string());
      }
      offset += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) {
      throw std::runtime_error(
              "failed to fsync durable temporary file: " +
              temporary.string() + " errno=" + std::to_string(errno));
    }
    if (::close(descriptor) != 0) {
      descriptor = -1;
      throw std::runtime_error(
              "failed to close durable temporary file: " +
              temporary.string() + " errno=" + std::to_string(errno));
    }
    descriptor = -1;
    if (::rename(temporary.c_str(), path.c_str()) != 0) {
      throw std::runtime_error(
              "failed to commit durable file: " + path.string() +
              " errno=" + std::to_string(errno));
    }
    renamed = true;
    durable_sync_directory(parent);
  } catch (...) {
    if (descriptor >= 0) {
      (void)::close(descriptor);
    }
    if (!renamed) {
      (void)::unlink(temporary.c_str());
    }
    throw;
  }
}

void durable_remove_file(const fs::path & path)
{
  if (::unlink(path.c_str()) != 0 && errno != ENOENT) {
    throw std::runtime_error(
            "failed to remove durable marker: " + path.string() +
            " errno=" + std::to_string(errno));
  }
  durable_sync_directory(path.parent_path());
}

fs::path map_delete_tombstone_path(const fs::path & map_root)
{
  static std::atomic<std::uint64_t> sequence{0U};
  const auto ticks =
    std::chrono::steady_clock::now().time_since_epoch().count();
  return map_root.parent_path() /
         ("." + map_root.filename().string() + ".delete-" +
         std::to_string(::getpid()) + "-" + std::to_string(ticks) + "-" +
         std::to_string(sequence.fetch_add(1U)));
}

bool safe_bundle_regular_file(
  const fs::path & path,
  const fs::path & bundle_root)
{
  try {
    std::error_code error;
    if (fs::is_symlink(bundle_root) || !fs::is_directory(bundle_root) ||
      !path_lexically_within(path, bundle_root))
    {
      return false;
    }
    const auto normalized_root =
      fs::absolute(bundle_root).lexically_normal();
    const auto normalized_path =
      fs::absolute(path).lexically_normal();
    auto relative = normalized_path.lexically_relative(normalized_root);
    if (relative.empty() || relative.is_absolute()) {
      return false;
    }
    fs::path cursor = normalized_root;
    for (const auto & component : relative) {
      if (component == "..") {
        return false;
      }
      cursor /= component;
      const auto status = fs::symlink_status(cursor, error);
      if (error || status.type() == fs::file_type::symlink ||
        status.type() == fs::file_type::not_found)
      {
        return false;
      }
    }
    return fs::is_regular_file(normalized_path) &&
           !fs::is_symlink(normalized_path);
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace robot_api_server::features::maps
