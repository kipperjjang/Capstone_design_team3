#include "data/estimator_config.hpp"

#include <yaml-cpp/yaml.h>

EstimatorConfig EstimatorConfig::load(const std::string &path) {
  EstimatorConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node estimator = file["bridge"]["serial"];

  config.q_acc = estimator["q_acc"].as<double>();
  config.r_detected = estimator["r_detected"].as<double>();
  config.r_tracked = estimator["r_tracked"].as<double>();
  config.p0_pos = estimator["p0_pos"].as<double>();
  config.p0_vel = estimator["p0_vel"].as<double>();
  config.max_gap = estimator["max_gap"].as<double>();

  return config;
}
