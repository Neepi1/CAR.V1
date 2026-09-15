// Offline CAN replay against the SAME ResponseModel used by the MPPI plugin.
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "robot_nav_config/chassis_dynamics/response_model.hpp"

std::vector<std::string> fields(const std::string & line)
{
  std::vector<std::string> out(1);
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {out.back() += ch; ++i;}
      else {quoted = !quoted;}
    } else if (ch == ',' && !quoted) {out.emplace_back();}
    else if (ch != '\r') {out.back() += ch;}
  }
  return out;
}

int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "usage: replay_can samples.csv\n"; return 2;}
  std::ifstream stream(argv[1]);
  if (!stream) {return 2;}
  std::string line;
  std::getline(stream, line);
  const auto header = fields(line);
  std::map<std::string, std::size_t> keys;
  for (std::size_t i = 0; i < header.size(); ++i) {keys[header[i]] = i;}
  struct Row {double t, v, angle, u, w;};
  std::map<std::string, std::vector<Row>> segments;
  while (std::getline(stream, line)) {
    const auto f = fields(line);
    if (f.size() != header.size()) {std::cerr << "Invalid CSV row\n"; return 2;}
    const auto value = [&](const std::string & key) {return std::stod(f.at(keys.at(key)));};
    const auto kind = f.at(keys.at("segment_kind"));
    if (kind != "linear_accel" && kind != "linear_decel" && kind != "steering_step") {continue;}
    segments[f.at(keys.at("segment"))].push_back({value("segment_elapsed_sec"),
      value("can221_linear"), value("can221_steering"), value("cmd_vx"), value("cmd_wz")});
  }
  bool passed = !segments.empty();
  for (const auto & [name, rows] : segments) {
    robot_nav_config::chassis_dynamics::ResponseModel model;
    model.reset(rows.front().v, rows.front().angle, rows.front().v,
      model.yaw_rate(rows.front().v, rows.front().angle));
    double v_error = 0, angle_error = 0, predicted_distance = 0, actual_distance = 0;
    for (std::size_t j = 1; j < rows.size(); ++j) {
      const double dt = rows[j].t - rows[j - 1].t;
      const auto state = model.advance(rows[j - 1].u, rows[j - 1].w, dt);
      v_error += std::pow(state.linear - rows[j].v, 2);
      angle_error += std::pow(state.steering - rows[j].angle, 2);
      predicted_distance += state.linear * dt;
      actual_distance += rows[j].v * dt;
    }
    const double vrmse = std::sqrt(v_error / (rows.size() - 1));
    const double armse = std::sqrt(angle_error / (rows.size() - 1));
    const bool steering = name.find("steer_") == 0;
    const bool ok = steering ? armse < 0.012 : vrmse < 0.04;
    passed = passed && ok;
    std::cout << (ok ? "PASS " : "FAIL ") << name << " velocity_rmse=" << vrmse
      << " angle_rmse=" << armse << " predicted_signed_distance=" << predicted_distance
      << " actual_signed_distance=" << actual_distance << '\n';
  }
  return passed ? 0 : 1;
}
