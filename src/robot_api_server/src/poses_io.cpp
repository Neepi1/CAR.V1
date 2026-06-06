#include "robot_api_server/poses_io.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>

#include "robot_api_server/http_common.hpp"
#include "robot_api_server/tf_pose_utils.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;
namespace
{

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

void write_text_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open file for writing: " + path.string());
  }
  file << text;
}

std::string yaml_scalar_unquote(std::string value)
{
  value = trim(value);
  const auto comment = value.find(" #");
  if (comment != std::string::npos) {
    value = trim(value.substr(0, comment));
  }
  if (value.size() >= 2U &&
    ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
  {
    value = value.substr(1, value.size() - 2U);
  }
  return value;
}

std::optional<double> parse_yaml_double(std::string value)
{
  value = yaml_scalar_unquote(std::move(value));
  if (value.empty()) {
    return std::nullopt;
  }
  try {
    std::size_t idx = 0;
    const double parsed = std::stod(value, &idx);
    if (idx == 0U || !std::isfinite(parsed)) {
      return std::nullopt;
    }
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

bool parse_yaml_key_value(const std::string & line, std::string & key, std::string & value)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {
    return false;
  }
  key = trim(line.substr(0, colon));
  value = trim(line.substr(colon + 1));
  return !key.empty();
}

void apply_pose_yaml_field(
  StoredPose & pose,
  bool & have_x,
  bool & have_y,
  bool & have_yaw,
  const std::string & key,
  const std::string & value)
{
  if (key == "id" || key == "pose_id") {
    pose.id = yaml_scalar_unquote(value);
  } else if (key == "name") {
    pose.name = yaml_scalar_unquote(value);
  } else if (key == "type") {
    pose.type = yaml_scalar_unquote(value);
  } else if (key == "x") {
    const auto parsed = parse_yaml_double(value);
    if (parsed) {
      pose.x = *parsed;
      have_x = true;
    }
  } else if (key == "y") {
    const auto parsed = parse_yaml_double(value);
    if (parsed) {
      pose.y = *parsed;
      have_y = true;
    }
  } else if (key == "yaw" || key == "theta" || key == "heading") {
    const auto parsed = parse_yaml_double(value);
    if (parsed) {
      pose.yaw = *parsed;
      have_yaw = true;
    }
  }
}

}  // namespace

std::string poses_json_array(const std::vector<StoredPose> & poses)
{
  std::ostringstream response;
  response << std::fixed << std::setprecision(6);
  response << "[";
  bool first = true;
  for (const auto & pose : poses) {
    if (!first) {
      response << ",";
    }
    first = false;
    response << "{\"pose_id\":" << json_string(pose.id)
             << ",\"id\":" << json_string(pose.id)
             << ",\"type\":" << json_string(pose.type.empty() ? "delivery_point" : pose.type)
             << ",\"name\":" << json_string(pose.name.empty() ? pose.id : pose.name)
             << ",\"x\":" << pose.x
             << ",\"y\":" << pose.y
             << ",\"yaw\":" << normalize_angle(pose.yaw) << "}";
  }
  response << "]";
  return response.str();
}

std::vector<StoredPose> read_floor_poses(const fs::path & path)
{
  std::vector<StoredPose> poses;
  if (!fs::exists(path)) {
    return poses;
  }
  const auto text = read_text_file(path);
  std::istringstream input(text);
  std::string line;
  StoredPose current;
  bool in_pose = false;
  bool have_x = false;
  bool have_y = false;
  bool have_yaw = false;

  const auto flush_pose = [&]() {
    if (in_pose && !current.id.empty() && have_x && have_y && have_yaw) {
      if (current.name.empty()) {
        current.name = current.id;
      }
      if (current.type.empty()) {
        current.type = "delivery_point";
      }
      poses.push_back(current);
    }
    current = StoredPose{};
    in_pose = false;
    have_x = false;
    have_y = false;
    have_yaw = false;
  };

  while (std::getline(input, line)) {
    auto stripped = trim(line);
    if (stripped.empty() || stripped.front() == '#') {
      continue;
    }
    if (stripped.rfind("- ", 0) == 0) {
      flush_pose();
      in_pose = true;
      stripped = trim(stripped.substr(2));
      if (!stripped.empty()) {
        std::string key;
        std::string value;
        if (parse_yaml_key_value(stripped, key, value)) {
          apply_pose_yaml_field(current, have_x, have_y, have_yaw, key, value);
        }
      }
      continue;
    }
    if (!in_pose) {
      continue;
    }
    std::string key;
    std::string value;
    if (parse_yaml_key_value(stripped, key, value)) {
      apply_pose_yaml_field(current, have_x, have_y, have_yaw, key, value);
    }
  }
  flush_pose();
  return poses;
}

void write_floor_poses(const fs::path & path, const std::vector<StoredPose> & poses)
{
  std::ostringstream yaml;
  yaml << std::fixed << std::setprecision(6);
  yaml << "poses:\n";
  for (const auto & pose : poses) {
    yaml << "  - id: " << json_string(pose.id) << "\n";
    yaml << "    type: " << json_string(pose.type.empty() ? "delivery_point" : pose.type) << "\n";
    yaml << "    name: " << json_string(pose.name.empty() ? pose.id : pose.name) << "\n";
    yaml << "    x: " << pose.x << "\n";
    yaml << "    y: " << pose.y << "\n";
    yaml << "    yaw: " << normalize_angle(pose.yaw) << "\n";
  }
  write_text_file(path, yaml.str());
}

std::optional<StoredPose> find_floor_pose(const fs::path & path, const std::string & pose_id)
{
  if (!fs::exists(path)) {
    return std::nullopt;
  }
  for (const auto & pose : read_floor_poses(path)) {
    if (pose.id == pose_id) {
      return pose;
    }
  }
  return std::nullopt;
}

}  // namespace robot_api_server
