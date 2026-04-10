#include "data/estimator_config.hpp"

#include <yaml-cpp/yaml.h>

ControlConfig ControlConfig::load(const std::string &path) {
  ControlConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node controller = file["controller"];

  auto c = controller["image_center"].as<std::vector<double>>();
  config.img_center = Eigen::Vector2d(c[0], c[1]);
  
  auto Kp_diag = controller["Kp"].as<std::vector<double>>();
  auto Kd_diag = controller["Kd"].as<std::vector<double>>();
  config.Kp = Eigen::Vector2d(Kp_diag[0],  Kp_diag[1]).asDiagonal();
  config.Kd = Eigen::Vector2d(Kd_diag[0],  Kd_diag[1]).asDiagonal();

  config.hz = file["frequency"].as<double>();
  return config;
}
