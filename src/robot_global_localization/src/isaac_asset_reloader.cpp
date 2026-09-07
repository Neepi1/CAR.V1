#include "robot_global_localization/isaac_asset_reloader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#include <unordered_set>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace robot_global_localization
{

namespace
{

namespace fs = std::filesystem;

struct ParsedMapParameters
{
  fs::path nav_map_yaml;
  fs::path localizer_map_png;
  fs::path localizer_params_yaml;
  std::string image;
  double resolution{0.0};
  std::vector<double> origin;
  double occupied_thresh{0.0};
};

struct RuntimeBootstrapContext
{
  std::string transaction_id;
  FloorAssetIdentity identity;
  std::uint64_t localizer_generation{0U};
  std::uint64_t explicit_relocalization_sequence{0U};
  double updated_at_sec{0.0};
};

bool identities_equal(
  const FloorAssetIdentity & lhs,
  const FloorAssetIdentity & rhs)
{
  return lhs.building_id == rhs.building_id &&
         lhs.floor_id == rhs.floor_id &&
         lhs.map_id == rhs.map_id &&
         lhs.asset_epoch == rhs.asset_epoch &&
         lhs.asset_digest == rhs.asset_digest;
}

bool is_safe_identifier(const std::string & value)
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }
  const auto is_first = [](const unsigned char character) {
      return std::isalnum(character) != 0;
    };
  const auto is_rest = [](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == '_' || character == '-' || character == '.';
    };
  return is_first(static_cast<unsigned char>(value.front())) &&
         std::all_of(
    value.begin() + 1, value.end(),
    [&](const char character) {
      return is_rest(static_cast<unsigned char>(character));
    });
}

bool is_sha256_digest(const std::string & digest)
{
  static constexpr char kPrefix[] = "sha256:";
  constexpr std::size_t kPrefixLength = sizeof(kPrefix) - 1U;
  if (digest.size() != kPrefixLength + 64U ||
    digest.compare(0U, kPrefixLength, kPrefix) != 0)
  {
    return false;
  }
  return std::all_of(
    digest.begin() + static_cast<std::ptrdiff_t>(kPrefixLength), digest.end(),
    [](const char character) {
      return (character >= '0' && character <= '9') ||
      (character >= 'a' && character <= 'f');
    });
}

bool has_whitespace_or_control(const std::string & value)
{
  return std::any_of(
    value.begin(), value.end(),
    [](const char character) {
      const auto unsigned_character = static_cast<unsigned char>(character);
      return std::isspace(unsigned_character) != 0 ||
      std::iscntrl(unsigned_character) != 0;
    });
}

bool is_ros_node_name(const std::string & value)
{
  if (value.empty() || value.size() > 255U) {
    return false;
  }
  const auto first = static_cast<unsigned char>(value.front());
  if (!(std::isalpha(first) != 0 || value.front() == '_')) {
    return false;
  }
  return std::all_of(
    value.begin() + 1, value.end(),
    [](const char character) {
      const auto unsigned_character = static_cast<unsigned char>(character);
      return std::isalnum(unsigned_character) != 0 || character == '_';
    });
}

bool valid_component_options(
  const IsaacAssetReloaderOptions & options,
  std::string & detail)
{
  const auto & component = options.component;
  if (component.package_name.empty() || component.plugin_name.empty() ||
    has_whitespace_or_control(component.package_name) ||
    has_whitespace_or_control(component.plugin_name) ||
    !is_ros_node_name(component.node_name))
  {
    detail = "component package, plugin, or ROS node name is invalid";
    return false;
  }
  if (!component.node_namespace.empty() &&
    component.node_namespace != "/")
  {
    if (component.node_namespace.front() != '/' ||
      component.node_namespace.back() == '/' ||
      component.node_namespace.find("//") != std::string::npos ||
      has_whitespace_or_control(component.node_namespace))
    {
      detail = "component node namespace is invalid";
      return false;
    }
  }
  const std::string expected_from_description =
    component.node_namespace.empty() || component.node_namespace == "/" ?
    "/" + component.node_name :
    component.node_namespace + "/" + component.node_name;
  if (options.expected_full_node_name != expected_from_description) {
    detail =
      "expected full node name does not match component namespace and node name";
    return false;
  }
  if (options.max_yaml_bytes == 0U || options.max_png_bytes == 0U) {
    detail = "asset size limits must be greater than zero";
    return false;
  }
  for (const auto & remap_rule : component.remap_rules) {
    if (remap_rule.empty() || has_whitespace_or_control(remap_rule) ||
      remap_rule.find("__node:=") != std::string::npos ||
      remap_rule.find("__name:=") != std::string::npos ||
      remap_rule.find("__ns:=") != std::string::npos)
    {
      detail = "component remap rule is empty, malformed, or changes node identity";
      return false;
    }
  }
  return true;
}

bool path_has_prefix(const fs::path & path, const fs::path & prefix)
{
  auto path_iterator = path.begin();
  for (auto prefix_iterator = prefix.begin();
    prefix_iterator != prefix.end(); ++prefix_iterator, ++path_iterator)
  {
    if (path_iterator == path.end() || *path_iterator != *prefix_iterator) {
      return false;
    }
  }
  return true;
}

bool has_symlink_between(
  const fs::path & canonical_root,
  const fs::path & requested_path,
  std::string & detail)
{
  std::error_code error;
  fs::path current = canonical_root;
  const fs::path lexical_path = requested_path.lexically_normal();
  if (!path_has_prefix(lexical_path, canonical_root)) {
    detail = "asset path is not lexically contained by the allowed asset root";
    return true;
  }
  const fs::path relative = lexical_path.lexically_relative(canonical_root);
  if (relative.empty()) {
    detail = "failed to derive lexical path relative to allowed asset root";
    return true;
  }
  for (const auto & component : relative) {
    if (component == ".") {
      continue;
    }
    if (component == "..") {
      detail = "parent traversal in asset paths is forbidden";
      return true;
    }
    current /= component;
    const fs::file_status status = fs::symlink_status(current, error);
    if (error) {
      detail = "failed to inspect asset path component: " + current.string();
      return true;
    }
    if (fs::is_symlink(status)) {
      detail = "symlink asset paths are forbidden: " + current.string();
      return true;
    }
  }
  return false;
}

bool canonical_regular_file(
  const fs::path & requested,
  const fs::path & canonical_root,
  const std::uintmax_t max_bytes,
  fs::path & canonical,
  std::string & failure_code,
  std::string & detail)
{
  if (!requested.is_absolute()) {
    failure_code = "ASSET_PATH_NOT_ABSOLUTE";
    detail = "asset path must be absolute: " + requested.string();
    return false;
  }

  std::error_code error;
  canonical = fs::canonical(requested, error);
  if (error) {
    failure_code = "ASSET_PATH_INVALID";
    detail = "asset path cannot be resolved: " + requested.string();
    return false;
  }
  if (!path_has_prefix(canonical, canonical_root)) {
    failure_code = "ASSET_PATH_ESCAPE";
    detail = "asset path escapes allowed root: " + requested.string();
    return false;
  }
  if (has_symlink_between(canonical_root, requested.lexically_normal(), detail)) {
    failure_code = "ASSET_PATH_SYMLINK";
    return false;
  }
  if (!fs::is_regular_file(canonical, error) || error) {
    failure_code = "ASSET_NOT_REGULAR_FILE";
    detail = "asset is not a regular file: " + canonical.string();
    return false;
  }
  const std::uintmax_t bytes = fs::file_size(canonical, error);
  if (error || bytes == 0U || bytes > max_bytes) {
    failure_code = "ASSET_SIZE_INVALID";
    detail = "asset is empty, unreadable, or exceeds its size limit: " + canonical.string();
    return false;
  }
  return true;
}

bool read_png_signature(const fs::path & path)
{
  static constexpr std::array<unsigned char, 8U> kExpected{
    0x89U, 0x50U, 0x4eU, 0x47U, 0x0dU, 0x0aU, 0x1aU, 0x0aU,
  };
  std::array<unsigned char, kExpected.size()> actual{};
  std::ifstream stream(path, std::ios::binary);
  stream.read(
    reinterpret_cast<char *>(actual.data()),
    static_cast<std::streamsize>(actual.size()));
  return stream.gcount() == static_cast<std::streamsize>(actual.size()) &&
         actual == kExpected;
}

bool finite_sequence(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](const double value) {return std::isfinite(value);});
}

bool unique_mapping_keys(const YAML::Node & root, std::string & detail)
{
  if (!root.IsMap()) {
    detail = "document root must be a mapping";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & entry : root) {
    if (!entry.first.IsScalar()) {
      detail = "document contains a non-scalar field name";
      return false;
    }
    const std::string name = entry.first.Scalar();
    if (name.empty() || !names.insert(name).second) {
      detail = "document contains an empty or duplicate field name";
      return false;
    }
  }
  return true;
}

bool required_scalar(
  const YAML::Node & root,
  const std::string & name,
  std::string & value,
  std::string & detail)
{
  const YAML::Node node = root[name];
  if (!node || !node.IsScalar()) {
    detail = "required scalar field is missing: " + name;
    return false;
  }
  value = node.Scalar();
  return true;
}

bool canonical_positive_uint64(
  const YAML::Node & root,
  const std::string & name,
  std::uint64_t & value,
  std::string & detail)
{
  std::string text;
  if (!required_scalar(root, name, text, detail) || text.empty() ||
    (text.size() > 1U && text.front() == '0') ||
    !std::all_of(
      text.begin(), text.end(),
      [](const char character) {
        return character >= '0' && character <= '9';
      }))
  {
    detail = "field must be a canonical positive uint64: " + name;
    return false;
  }
  const auto conversion =
    std::from_chars(text.data(), text.data() + text.size(), value);
  if (conversion.ec != std::errc{} ||
    conversion.ptr != text.data() + text.size() || value == 0U)
  {
    detail = "field must be a canonical positive uint64: " + name;
    return false;
  }
  return true;
}

bool read_strict_document(
  const fs::path & requested_path,
  const std::uintmax_t max_bytes,
  fs::path & canonical_path,
  YAML::Node & root,
  std::string & failure_code,
  std::string & detail)
{
  if (!requested_path.is_absolute()) {
    failure_code = "RUNTIME_CONTEXT_PATH_INVALID";
    detail = "runtime context path must be absolute";
    return false;
  }
  const fs::path lexical_path = requested_path.lexically_normal();
  std::error_code error;
  canonical_path = fs::canonical(requested_path, error);
  if (error || canonical_path != lexical_path) {
    failure_code = "RUNTIME_CONTEXT_PATH_INVALID";
    detail = "runtime context path is missing, unresolved, or contains a symlink";
    return false;
  }
  const fs::file_status status = fs::symlink_status(lexical_path, error);
  if (error || fs::is_symlink(status) || !fs::is_regular_file(status)) {
    failure_code = "RUNTIME_CONTEXT_PATH_INVALID";
    detail = "runtime context must be a regular non-symlink file";
    return false;
  }
  const std::uintmax_t size = fs::file_size(canonical_path, error);
  if (error || size == 0U || size > max_bytes) {
    failure_code = "RUNTIME_CONTEXT_SIZE_INVALID";
    detail = "runtime context is empty, unreadable, or exceeds its size limit";
    return false;
  }
  try {
    root = YAML::LoadFile(canonical_path.string());
  } catch (const YAML::Exception & exception) {
    failure_code = "RUNTIME_CONTEXT_PARSE_FAILED";
    detail = exception.what();
    return false;
  }
  if (!unique_mapping_keys(root, detail)) {
    failure_code = "RUNTIME_CONTEXT_INVALID";
    return false;
  }
  return true;
}

bool parse_runtime_bootstrap_context(
  const YAML::Node & root,
  RuntimeBootstrapContext & context,
  std::string & failure_code,
  std::string & detail)
{
  std::string schema;
  std::string state;
  std::string confirmed;
  std::string updated_at;
  if (!required_scalar(root, "schema", schema, detail) ||
    !required_scalar(root, "state", state, detail) ||
    !required_scalar(root, "confirmed", confirmed, detail))
  {
    failure_code = "RUNTIME_CONTEXT_INVALID";
    return false;
  }
  if (schema != "njrh.runtime_map_context.v1") {
    failure_code = "RUNTIME_CONTEXT_SCHEMA_INVALID";
    detail = "runtime context schema is not njrh.runtime_map_context.v1";
    return false;
  }
  if (state != "ready" || confirmed != "true") {
    failure_code = "RUNTIME_CONTEXT_NOT_READY";
    detail = "runtime context must be state=ready and confirmed=true";
    return false;
  }
  if (!required_scalar(
      root, "transaction_id", context.transaction_id, detail) ||
    !required_scalar(
      root, "building_id", context.identity.building_id, detail) ||
    !required_scalar(
      root, "floor_id", context.identity.floor_id, detail) ||
    !required_scalar(
      root, "map_id", context.identity.map_id, detail) ||
    !required_scalar(
      root, "asset_digest", context.identity.asset_digest, detail) ||
    !canonical_positive_uint64(
      root, "asset_epoch", context.identity.asset_epoch, detail) ||
    !canonical_positive_uint64(
      root, "localizer_generation", context.localizer_generation, detail) ||
    !canonical_positive_uint64(
      root, "explicit_relocalization_sequence",
      context.explicit_relocalization_sequence, detail) ||
    !required_scalar(root, "updated_at", updated_at, detail))
  {
    failure_code = "RUNTIME_CONTEXT_IDENTITY_INVALID";
    return false;
  }
  if (!is_safe_identifier(context.transaction_id) ||
    !is_safe_identifier(context.identity.building_id) ||
    !is_safe_identifier(context.identity.floor_id) ||
    !is_safe_identifier(context.identity.map_id) ||
    !is_sha256_digest(context.identity.asset_digest))
  {
    failure_code = "RUNTIME_CONTEXT_IDENTITY_INVALID";
    detail = "runtime context exact identity is unsafe or incomplete";
    return false;
  }
  try {
    context.updated_at_sec = root["updated_at"].as<double>();
  } catch (const YAML::Exception & exception) {
    failure_code = "RUNTIME_CONTEXT_IDENTITY_INVALID";
    detail = exception.what();
    return false;
  }
  if (!std::isfinite(context.updated_at_sec) ||
    context.updated_at_sec <= 0.0)
  {
    failure_code = "RUNTIME_CONTEXT_IDENTITY_INVALID";
    detail = "runtime context updated_at must be finite and positive";
    return false;
  }
  return true;
}

bool parse_exact_current_manifest(
  const YAML::Node & root,
  const FloorAssetIdentity & expected,
  std::string & failure_code,
  std::string & detail)
{
  if (!unique_mapping_keys(root, detail)) {
    failure_code = "RUNTIME_ASSET_MANIFEST_INVALID";
    return false;
  }
  std::string schema;
  std::string active;
  std::string digest_algorithm;
  std::string digest_contract;
  FloorAssetIdentity actual;
  if (!required_scalar(root, "schema", schema, detail) ||
    !required_scalar(root, "active", active, detail) ||
    !required_scalar(
      root, "asset_digest_algorithm", digest_algorithm, detail) ||
    !required_scalar(
      root, "asset_digest_contract", digest_contract, detail) ||
    !required_scalar(root, "building_id", actual.building_id, detail) ||
    !required_scalar(root, "floor_id", actual.floor_id, detail) ||
    !required_scalar(root, "map_id", actual.map_id, detail) ||
    !required_scalar(root, "asset_digest", actual.asset_digest, detail) ||
    !canonical_positive_uint64(
      root, "asset_epoch", actual.asset_epoch, detail))
  {
    failure_code = "RUNTIME_ASSET_MANIFEST_INVALID";
    return false;
  }
  if (
    schema != "njrh.map_manifest.v2" ||
    active != "true" ||
    digest_algorithm != "sha256" ||
    digest_contract != "njrh-map-asset-bundle-v1")
  {
    failure_code = "RUNTIME_ASSET_MANIFEST_INVALID";
    detail =
      "current manifest must be active njrh.map_manifest.v2 with the "
      "canonical sha256 bundle contract";
    return false;
  }
  if (!identities_equal(actual, expected)) {
    failure_code = "RUNTIME_ASSET_IDENTITY_MISMATCH";
    detail = "current manifest identity differs from durable runtime context";
    return false;
  }
  return true;
}

const LocalizerParameter * find_parameter(
  const std::vector<LocalizerParameter> & parameters,
  const std::string & name)
{
  const auto found = std::find_if(
    parameters.begin(), parameters.end(),
    [&](const LocalizerParameter & parameter) {
      return parameter.name == name;
    });
  return found == parameters.end() ? nullptr : &(*found);
}

bool nearly_equal(const double lhs, const double rhs)
{
  const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
  return std::abs(lhs - rhs) <=
         std::numeric_limits<double>::epsilon() * 8.0 * scale;
}

bool live_parameters_match(
  const ComponentSnapshot & snapshot,
  const ParsedMapParameters & expected,
  std::string & detail)
{
  const auto * map_yaml_path =
    find_parameter(snapshot.parameters, "map_yaml_path");
  const auto * image = find_parameter(snapshot.parameters, "image");
  const auto * resolution = find_parameter(snapshot.parameters, "resolution");
  const auto * origin = find_parameter(snapshot.parameters, "origin");
  const auto * occupied_thresh =
    find_parameter(snapshot.parameters, "occupied_thresh");
  if (map_yaml_path == nullptr || image == nullptr || resolution == nullptr ||
    origin == nullptr || occupied_thresh == nullptr ||
    map_yaml_path->type != LocalizerParameterType::kString ||
    image->type != LocalizerParameterType::kString ||
    resolution->type != LocalizerParameterType::kDouble ||
    origin->type != LocalizerParameterType::kDoubleArray ||
    occupied_thresh->type != LocalizerParameterType::kDouble)
  {
    detail = "live localizer is missing a required typed map parameter";
    return false;
  }
  std::error_code error;
  const fs::path live_map_yaml =
    fs::canonical(fs::path(map_yaml_path->string_value), error);
  if (error || live_map_yaml != expected.localizer_params_yaml ||
    image->string_value != expected.image ||
    !nearly_equal(resolution->double_value, expected.resolution) ||
    origin->double_array_value.size() != expected.origin.size() ||
    !std::equal(
      origin->double_array_value.begin(),
      origin->double_array_value.end(),
      expected.origin.begin(), nearly_equal) ||
    !nearly_equal(
      occupied_thresh->double_value, expected.occupied_thresh))
  {
    detail =
      "live map_yaml_path/image/resolution/origin/occupied_thresh "
      "do not match the exact current asset";
    return false;
  }
  return true;
}

bool validate_request(
  const FloorAssetRequest & request,
  const IsaacAssetReloaderOptions & options,
  ParsedMapParameters & parsed,
  std::string & failure_code,
  std::string & detail)
{
  if (!valid_component_options(options, detail)) {
    failure_code = "COMPONENT_CONFIG_INVALID";
    return false;
  }
  if (!is_safe_identifier(request.transaction_id)) {
    failure_code = "TRANSACTION_ID_INVALID";
    detail = "transaction_id must be 1..128 safe identifier characters";
    return false;
  }
  if (!is_safe_identifier(request.identity.building_id) ||
    !is_safe_identifier(request.identity.floor_id) ||
    !is_safe_identifier(request.identity.map_id))
  {
    failure_code = "ASSET_IDENTITY_INVALID";
    detail = "building_id, floor_id, and map_id must be safe identifiers";
    return false;
  }
  if (request.identity.asset_epoch == 0U) {
    failure_code = "ASSET_EPOCH_INVALID";
    detail = "asset_epoch must be greater than zero";
    return false;
  }
  if (!is_sha256_digest(request.identity.asset_digest)) {
    failure_code = "ASSET_DIGEST_INVALID";
    detail = "asset_digest must be lowercase sha256:<64 hex>";
    return false;
  }

  std::error_code error;
  if (!options.allowed_asset_root.is_absolute()) {
    failure_code = "ASSET_ROOT_INVALID";
    detail = "allowed asset root must be absolute";
    return false;
  }
  const fs::file_status root_status = fs::symlink_status(options.allowed_asset_root, error);
  if (error || fs::is_symlink(root_status)) {
    failure_code = "ASSET_ROOT_INVALID";
    detail = "allowed asset root is unavailable or is a symlink";
    return false;
  }
  const fs::path canonical_root = fs::canonical(options.allowed_asset_root, error);
  if (error || !fs::is_directory(canonical_root)) {
    failure_code = "ASSET_ROOT_INVALID";
    detail = "allowed asset root is not a readable directory";
    return false;
  }

  if (!canonical_regular_file(
      request.nav_map_yaml, canonical_root, options.max_yaml_bytes,
      parsed.nav_map_yaml, failure_code, detail) ||
    !canonical_regular_file(
      request.localizer_map_png, canonical_root, options.max_png_bytes,
      parsed.localizer_map_png, failure_code, detail) ||
    !canonical_regular_file(
      request.localizer_params_yaml, canonical_root, options.max_yaml_bytes,
      parsed.localizer_params_yaml, failure_code, detail))
  {
    return false;
  }
  if (!read_png_signature(parsed.localizer_map_png)) {
    failure_code = "LOCALIZER_PNG_INVALID";
    detail = "localizer map does not have a PNG signature";
    return false;
  }

  try {
    const YAML::Node nav = YAML::LoadFile(parsed.nav_map_yaml.string());
    if (!nav.IsMap() || !nav["image"] || !nav["resolution"]) {
      failure_code = "NAV_MAP_YAML_INVALID";
      detail = "nav map YAML must contain image and resolution";
      return false;
    }

    const YAML::Node localizer = YAML::LoadFile(parsed.localizer_params_yaml.string());
    if (!localizer.IsMap() || !localizer["image"] || !localizer["resolution"] ||
      !localizer["origin"] || !localizer["occupied_thresh"])
    {
      failure_code = "LOCALIZER_YAML_INVALID";
      detail = "localizer YAML is missing a required map field";
      return false;
    }
    parsed.image = localizer["image"].as<std::string>();
    parsed.resolution = localizer["resolution"].as<double>();
    parsed.origin = localizer["origin"].as<std::vector<double>>();
    parsed.occupied_thresh = localizer["occupied_thresh"].as<double>();

    if (parsed.image.empty() || parsed.resolution <= 0.0 ||
      !std::isfinite(parsed.resolution) || parsed.origin.size() != 3U ||
      !finite_sequence(parsed.origin) || parsed.occupied_thresh <= 0.0 ||
      parsed.occupied_thresh >= 1.0 || !std::isfinite(parsed.occupied_thresh))
    {
      failure_code = "LOCALIZER_YAML_INVALID";
      detail = "localizer YAML map values are outside the supported range";
      return false;
    }
    if (localizer["free_thresh"]) {
      const double free_thresh = localizer["free_thresh"].as<double>();
      if (!std::isfinite(free_thresh) || free_thresh < 0.0 ||
        free_thresh >= parsed.occupied_thresh)
      {
        failure_code = "LOCALIZER_YAML_INVALID";
        detail = "free_thresh must be finite and lower than occupied_thresh";
        return false;
      }
    }

    fs::path image_path(parsed.image);
    if (!image_path.is_absolute()) {
      image_path = parsed.localizer_params_yaml.parent_path() / image_path;
    }
    const fs::path canonical_image = fs::canonical(image_path, error);
    if (error || canonical_image != parsed.localizer_map_png) {
      failure_code = "LOCALIZER_IMAGE_MISMATCH";
      detail = "localizer YAML image does not resolve to localizer_map_png";
      return false;
    }
  } catch (const YAML::Exception & exception) {
    failure_code = "ASSET_YAML_PARSE_FAILED";
    detail = exception.what();
    return false;
  }
  return true;
}

LocalizerParameter string_parameter(std::string name, std::string value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kString;
  parameter.string_value = std::move(value);
  return parameter;
}

LocalizerParameter double_parameter(std::string name, const double value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kDouble;
  parameter.double_value = value;
  return parameter;
}

LocalizerParameter double_array_parameter(
  std::string name,
  std::vector<double> value)
{
  LocalizerParameter parameter;
  parameter.name = std::move(name);
  parameter.type = LocalizerParameterType::kDoubleArray;
  parameter.double_array_value = std::move(value);
  return parameter;
}

void replace_parameter(
  std::vector<LocalizerParameter> & parameters,
  LocalizerParameter replacement)
{
  const auto found = std::find_if(
    parameters.begin(), parameters.end(),
    [&](const LocalizerParameter & parameter) {
      return parameter.name == replacement.name;
    });
  if (found == parameters.end()) {
    parameters.push_back(std::move(replacement));
  } else {
    *found = std::move(replacement);
  }
}

bool validate_capture(
  const ComponentCaptureResult & capture,
  const std::string & expected_full_node_name,
  std::string & failure_code,
  std::string & detail)
{
  if (!capture.success) {
    failure_code =
      capture.failure_code.empty() ? "COMPONENT_CAPTURE_FAILED" : capture.failure_code;
    detail = capture.message;
    return false;
  }
  if (capture.snapshot.unique_id == 0U ||
    capture.snapshot.full_node_name != expected_full_node_name)
  {
    failure_code = "LOCALIZER_COMPONENT_AMBIGUOUS";
    detail = "captured component does not exactly match the configured localizer";
    return false;
  }
  std::vector<std::string> names;
  names.reserve(capture.snapshot.parameters.size());
  for (const auto & parameter : capture.snapshot.parameters) {
    if (parameter.name.empty() || parameter.type == LocalizerParameterType::kNotSet ||
      std::find(names.begin(), names.end(), parameter.name) != names.end())
    {
      failure_code = "LOCALIZER_PARAMETER_CAPTURE_INVALID";
      detail = "captured localizer parameters contain an unset, unnamed, or duplicate value";
      return false;
    }
    names.push_back(parameter.name);
  }
  return true;
}

}  // namespace

IsaacAssetReloader::IsaacAssetReloader(IsaacAssetReloaderOptions options)
: options_(std::move(options))
{
}

ApplyFloorAssetResult IsaacAssetReloader::apply(
  const FloorAssetRequest & request,
  ComponentManagerPort & component_manager)
{
  state_.transaction_id = request.transaction_id;
  state_.requested_identity = request.identity;
  state_.applying = true;
  state_.idempotent = false;
  state_.reloaded = false;
  state_.rollback_attempted = false;
  state_.rollback_succeeded = false;
  state_.failure_code.clear();
  state_.detail.clear();

  ParsedMapParameters parsed;
  if (!validate_request(
      request, options_, parsed, state_.failure_code, state_.detail))
  {
    state_.applying = false;
    return ApplyFloorAssetResult{false, false, state_};
  }

  if (state_.active_identity_valid &&
    identities_equal(state_.active_identity, request.identity))
  {
    if (state_.active_nav_map_yaml != parsed.nav_map_yaml ||
      state_.active_localizer_map_png != parsed.localizer_map_png ||
      state_.active_localizer_params_yaml != parsed.localizer_params_yaml)
    {
      state_.failure_code = "ASSET_IDENTITY_PATH_CONFLICT";
      state_.detail = "exact active identity was requested with different canonical paths";
      state_.applying = false;
      return ApplyFloorAssetResult{false, false, state_};
    }
    state_.idempotent = true;
    state_.failure_code.clear();
    state_.detail = "requested exact asset identity is already active";
    state_.applying = false;
    return ApplyFloorAssetResult{true, true, state_};
  }

  const ComponentOperationResult preflight = component_manager.preflight();
  if (!preflight.success) {
    state_.failure_code =
      preflight.failure_code.empty() ? "COMPONENT_MANAGER_UNAVAILABLE" :
      preflight.failure_code;
    state_.detail = preflight.message;
    state_.applying = false;
    return ApplyFloorAssetResult{false, false, state_};
  }

  const ComponentCaptureResult capture = component_manager.capture();
  if (!validate_capture(
      capture, options_.expected_full_node_name,
      state_.failure_code, state_.detail))
  {
    state_.applying = false;
    return ApplyFloorAssetResult{false, false, state_};
  }
  state_.localizer_ready = true;

  ComponentLoadRequest target_load;
  target_load.description = options_.component;
  target_load.parameters = capture.snapshot.parameters;
  replace_parameter(
    target_load.parameters,
    string_parameter("map_yaml_path", parsed.localizer_params_yaml.string()));
  replace_parameter(
    target_load.parameters,
    string_parameter("image", parsed.image));
  replace_parameter(
    target_load.parameters,
    double_parameter("resolution", parsed.resolution));
  replace_parameter(
    target_load.parameters,
    double_array_parameter("origin", parsed.origin));
  replace_parameter(
    target_load.parameters,
    double_parameter("occupied_thresh", parsed.occupied_thresh));

  const ComponentOperationResult unload =
    component_manager.unload(capture.snapshot.unique_id);
  if (!unload.success) {
    state_.failure_code =
      unload.failure_code.empty() ? "LOCALIZER_UNLOAD_FAILED" : unload.failure_code;
    state_.detail = unload.message;
    state_.localizer_ready =
      unload.resulting_presence == ComponentPresence::kPresent;
    if (unload.resulting_presence == ComponentPresence::kAbsent) {
      state_.rollback_attempted = true;
      ComponentLoadRequest rollback_load;
      rollback_load.description = options_.component;
      rollback_load.parameters = capture.snapshot.parameters;
      const ComponentOperationResult rollback = component_manager.load(rollback_load);
      state_.rollback_succeeded = rollback.success;
      if (rollback.success) {
        ++state_.localizer_generation;
        state_.localizer_ready = true;
      }
      state_.detail += rollback.success ?
        "; previous component restored" : "; previous component restore failed";
    }
    state_.applying = false;
    return ApplyFloorAssetResult{false, false, state_};
  }
  state_.localizer_ready = false;

  const ComponentOperationResult target = component_manager.load(target_load);
  if (target.success) {
    ++state_.localizer_generation;
    state_.active_identity = request.identity;
    state_.active_identity_valid = true;
    state_.active_nav_map_yaml = parsed.nav_map_yaml;
    state_.active_localizer_map_png = parsed.localizer_map_png;
    state_.active_localizer_params_yaml = parsed.localizer_params_yaml;
    state_.localizer_ready = true;
    state_.reloaded = true;
    state_.failure_code.clear();
    state_.detail = "Isaac occupancy localizer component replaced with exact floor assets";
    state_.applying = false;
    return ApplyFloorAssetResult{true, false, state_};
  }

  const std::string target_failure_code =
    target.failure_code.empty() ? "LOCALIZER_TARGET_LOAD_FAILED" : target.failure_code;
  const std::string target_failure_detail = target.message;
  const bool safe_to_restore =
    target.resulting_presence == ComponentPresence::kAbsent;
  ComponentOperationResult rollback;
  if (safe_to_restore) {
    state_.rollback_attempted = true;
    ComponentLoadRequest rollback_load;
    rollback_load.description = options_.component;
    rollback_load.parameters = capture.snapshot.parameters;
    rollback = component_manager.load(rollback_load);
    state_.rollback_succeeded = rollback.success;
    if (rollback.success) {
      ++state_.localizer_generation;
      state_.localizer_ready = true;
    }
  }
  if (!state_.rollback_succeeded) {
    state_.localizer_ready = false;
  }
  state_.failure_code = target_failure_code;
  std::ostringstream failure_detail;
  failure_detail << target_failure_detail << "; rollback ";
  if (!safe_to_restore) {
    failure_detail << "skipped because target component presence is not safely absent";
  } else if (rollback.success) {
    failure_detail << "succeeded";
  } else {
    failure_detail << "failed";
    if (!rollback.failure_code.empty()) {
      failure_detail << " [" << rollback.failure_code << "]";
    }
    if (!rollback.message.empty()) {
      failure_detail << ": " << rollback.message;
    }
  }
  state_.detail = failure_detail.str();
  state_.applying = false;
  return ApplyFloorAssetResult{false, false, state_};
}

ApplyFloorAssetResult IsaacAssetReloader::bootstrap_from_runtime_context(
  const std::filesystem::path & runtime_context_path,
  ComponentManagerPort & component_manager)
{
  if (
    state_.active_identity_valid &&
    state_.localizer_ready &&
    state_.localizer_generation > 0U)
  {
    state_.idempotent = true;
    state_.reloaded = false;
    state_.applying = false;
    state_.failure_code.clear();
    state_.detail = "active localizer identity is already established";
    return ApplyFloorAssetResult{true, true, state_};
  }

  state_.transaction_id.clear();
  state_.requested_identity = {};
  state_.active_identity = {};
  state_.active_nav_map_yaml.clear();
  state_.active_localizer_map_png.clear();
  state_.active_localizer_params_yaml.clear();
  state_.localizer_generation = 0U;
  state_.active_identity_valid = false;
  state_.applying = true;
  state_.localizer_ready = false;
  state_.idempotent = false;
  state_.reloaded = false;
  state_.rollback_attempted = false;
  state_.rollback_succeeded = false;
  state_.failure_code.clear();
  state_.detail.clear();

  const auto fail =
    [this](std::string failure_code, std::string detail) {
      state_.active_identity_valid = false;
      state_.localizer_ready = false;
      state_.applying = false;
      state_.failure_code = std::move(failure_code);
      state_.detail = std::move(detail);
      return ApplyFloorAssetResult{false, false, state_};
    };

  fs::path canonical_context;
  YAML::Node context_document;
  std::string failure_code;
  std::string detail;
  if (!read_strict_document(
      runtime_context_path, options_.max_yaml_bytes, canonical_context,
      context_document, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }

  RuntimeBootstrapContext context;
  if (!parse_runtime_bootstrap_context(
      context_document, context, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }
  state_.transaction_id = context.transaction_id;
  state_.requested_identity = context.identity;

  const fs::path current_root =
    options_.allowed_asset_root / context.identity.building_id /
    context.identity.floor_id / "current";
  FloorAssetRequest asset_request;
  asset_request.transaction_id = context.transaction_id;
  asset_request.identity = context.identity;
  asset_request.nav_map_yaml = current_root / "nav" / "nav_map.yaml";
  asset_request.localizer_map_png =
    current_root / "localizer" / "localizer_map.png";
  asset_request.localizer_params_yaml =
    current_root / "localizer" / "localizer_params.yaml";
  ParsedMapParameters parsed;
  if (!validate_request(
      asset_request, options_, parsed, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }

  std::error_code error;
  const fs::path canonical_asset_root =
    fs::canonical(options_.allowed_asset_root, error);
  if (error) {
    return fail(
      "ASSET_ROOT_INVALID",
      "allowed asset root cannot be resolved during bootstrap");
  }
  fs::path canonical_manifest;
  if (!canonical_regular_file(
      current_root / "manifest.json", canonical_asset_root,
      options_.max_yaml_bytes, canonical_manifest, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }

  YAML::Node manifest;
  try {
    manifest = YAML::LoadFile(canonical_manifest.string());
  } catch (const YAML::Exception & exception) {
    return fail("RUNTIME_ASSET_MANIFEST_INVALID", exception.what());
  }
  if (!parse_exact_current_manifest(
      manifest, context.identity, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }

  const ComponentOperationResult preflight = component_manager.preflight();
  if (!preflight.success) {
    return fail(
      preflight.failure_code.empty() ?
      "COMPONENT_MANAGER_UNAVAILABLE" : preflight.failure_code,
      preflight.message);
  }
  const ComponentCaptureResult capture = component_manager.capture();
  if (!validate_capture(
      capture, options_.expected_full_node_name, failure_code, detail))
  {
    return fail(std::move(failure_code), std::move(detail));
  }
  if (!live_parameters_match(capture.snapshot, parsed, detail)) {
    return fail("LIVE_LOCALIZER_PARAMETER_MISMATCH", std::move(detail));
  }

  state_.active_identity = context.identity;
  state_.active_identity_valid = true;
  state_.active_nav_map_yaml = parsed.nav_map_yaml;
  state_.active_localizer_map_png = parsed.localizer_map_png;
  state_.active_localizer_params_yaml = parsed.localizer_params_yaml;
  state_.localizer_generation = context.localizer_generation;
  state_.localizer_ready = true;
  state_.applying = false;
  state_.failure_code.clear();
  state_.detail =
    "bootstrap verified durable ready context, exact current manifest, "
    "unique live Isaac component, and matching map parameters";
  return ApplyFloorAssetResult{true, false, state_};
}

const LocalizerAssetState & IsaacAssetReloader::state() const noexcept
{
  return state_;
}

}  // namespace robot_global_localization
