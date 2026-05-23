#include "data/config/estimator_config.hpp"

#include <yaml-cpp/yaml.h>

EstimatorConfig EstimatorConfig::load(const std::string &path) {
  EstimatorConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node kalman = file["estimator"]["kalman"];
  
  config.prediction_time = file["estimator"]["prediction_time"].as<double>();

  config.q_acc = kalman["q_acc"].as<double>();
  config.r_pos = kalman["r_pos"].as<double>();
  config.r_vel = kalman["r_vel"].as<double>();
  config.r_acc = kalman["r_acc"].as<double>();

  config.p0_pos = kalman["p0_pos"].as<double>();
  config.p0_vel = kalman["p0_vel"].as<double>();
  config.p0_acc = kalman["p0_acc"].as<double>();
  config.max_time_gap = kalman["max_time_gap"].as<double>();

  return config;
}
