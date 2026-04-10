#include "data/config/estimator_config.hpp"

#include <yaml-cpp/yaml.h>

EstimatorConfig EstimatorConfig::load(const std::string &path) {
  EstimatorConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node estimator = file["estimator"];
  if (!estimator) {
    return config;
  }

  if (estimator["q_acc"]) config.q_acc = estimator["q_acc"].as<double>();
  if (estimator["r_detected"]) config.r_detected = estimator["r_detected"].as<double>();
  if (estimator["r_tracked"]) config.r_tracked = estimator["r_tracked"].as<double>();
  if (estimator["r_temp_vel"]) config.r_temp_vel = estimator["r_temp_vel"].as<double>();
  if (estimator["r_temp_acc"]) config.r_temp_acc = estimator["r_temp_acc"].as<double>();
  if (estimator["p0_pos"]) config.p0_pos = estimator["p0_pos"].as<double>();
  if (estimator["p0_vel"]) config.p0_vel = estimator["p0_vel"].as<double>();
  if (estimator["p0_acc"]) config.p0_acc = estimator["p0_acc"].as<double>();
  if (estimator["max_time_gap"]) config.max_time_gap = estimator["max_time_gap"].as<double>();
  if (estimator["hz"]) {
    config.hz = estimator["hz"].as<double>();
  } else if (file["controller"] && file["controller"]["frequency"]) {
    config.hz = file["controller"]["frequency"].as<double>();
  }

  return config;
}
