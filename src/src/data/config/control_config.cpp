#include "data/config/control_config.hpp"

#include <vector>

#include <yaml-cpp/yaml.h>

ControlConfig ControlConfig::load(const std::string &path) {
  ControlConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node controller = file["controller"];
  if (!controller) {
    return config;
  }

  const auto kp_diag = controller["Kp"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.Kp = Eigen::Vector2d(kp_diag[0], kp_diag[1]).asDiagonal();

  const auto kd_diag = controller["Kd"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.Kd = Eigen::Vector2d(kd_diag[0], kd_diag[1]).asDiagonal();

  config.hz = controller["frequency"].as<double>(config.hz);
  config.time_delay = controller["time_delay"].as<double>(config.time_delay);
  config.max_time_gap = controller["max_time_gap"].as<double>(config.max_time_gap);
  config.err_p_track = controller["err_p_track"].as<double>(config.err_p_track);
  config.err_v_track = controller["err_v_track"].as<double>(config.err_v_track);
  config.err_p_fire = controller["err_p_fire"].as<double>(config.err_p_fire);

  return config;
}
