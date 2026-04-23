#include "data/config/estimator_config.hpp"

#include <yaml-cpp/yaml.h>

EstimatorConfig EstimatorConfig::load(const std::string &path) {
  EstimatorConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node kalman = file["estimator"]["kalman"];
  
  config.hz = file["controller"]["frequency"].as<double>();
  config.q_acc = kalman["q_acc"].as<double>();
  config.r_detected = kalman["r_detected"].as<double>();
  config.r_tracked = kalman["r_tracked"].as<double>();
  config.r_temp_vel = kalman["r_temp_vel"].as<double>();
  config.r_temp_acc = kalman["r_temp_acc"].as<double>();

  // config.r_angle = kalman["r_angle"].as<double>();
  // config.r_omega = kalman["r_omega"].as<double>();

  config.p0_pos = kalman["p0_pos"].as<double>();
  config.p0_vel = kalman["p0_vel"].as<double>();
  config.p0_acc = kalman["p0_acc"].as<double>();
  config.max_time_gap = kalman["max_time_gap"].as<double>();

  return config;
}
