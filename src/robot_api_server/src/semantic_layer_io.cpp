#include "robot_api_server/semantic_layer_io.hpp"

#include <sstream>

#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

namespace fs = std::filesystem;

fs::path keepout_semantic_json_path(const MapManifest & manifest)
{
  return manifest.root / "filters" / "keepout_semantic_layer.json";
}

std::string json_raw_or_null(const std::string & text)
{
  const auto stripped = trim(text);
  if (stripped.empty()) {
    return "null";
  }
  if (stripped.front() == '{' || stripped.front() == '[') {
    return stripped;
  }
  return "null";
}

std::string keepout_semantic_payload_json(const std::string & semantic_json)
{
  const auto keepout = json_object_value(semantic_json, "keepout");
  if (keepout) {
    return *keepout;
  }
  return json_raw_or_null(semantic_json);
}

std::string keepout_filter_json(const MapManifest & manifest)
{
  const auto semantic_path = keepout_semantic_json_path(manifest);
  const auto mask_yaml = read_optional_text_file(manifest.keepout_mask_yaml);
  const auto semantic_json = read_optional_text_file(semantic_path);
  const auto keepout_payload = keepout_semantic_payload_json(semantic_json);
  std::ostringstream response;
  response << "{"
           << "\"filter\":\"keepout\","
           << "\"keepout_mask_yaml\":" << json_string(manifest.keepout_mask_yaml.string()) << ","
           << "\"keepout_mask_pgm\":" << json_string(manifest.keepout_mask_pgm.string()) << ","
           << "\"mask_yaml_exists\":" << (fs::exists(manifest.keepout_mask_yaml) ? "true" : "false") << ","
           << "\"mask_pgm_exists\":" << (fs::exists(manifest.keepout_mask_pgm) ? "true" : "false") << ","
           << "\"yaml\":" << json_string(mask_yaml) << ","
           << "\"mask_yaml\":" << json_string(mask_yaml) << ","
           << "\"semantic_json_path\":" << json_string(semantic_path.string()) << ","
           << "\"semantic_json_exists\":" << (fs::exists(semantic_path) ? "true" : "false") << ","
           << "\"semantic_json\":" << json_string(semantic_json) << ","
           << "\"raw_json\":" << json_string(semantic_json) << ","
           << "\"payload\":" << json_raw_or_null(semantic_json) << ","
           << "\"keepout_payload\":" << keepout_payload << ","
           << "\"keepout\":" << keepout_payload
           << "}";
  return response.str();
}

}  // namespace robot_api_server
