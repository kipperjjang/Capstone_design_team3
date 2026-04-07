#pragma once

#include "data/config/estimator_config.hpp"
#include "data/state/robot_state.hpp"
#include "estimator/kalman_filter.hpp"

class Estimator {
public:
  Estimator(const EstimatorConfig &config)
    : config_(config), kf_(config_) {}

  void init(const RobotState &state);

  void update(const RobotState &state);
  void update(const double t);
  void update(const Eigen::Vector2d &omega);

  // Utils
  const RobotState& getState();
  bool isInitialized() { return initialized_; }

private:
  // Configuration
  EstimatorConfig config_;

  // State
  RobotState state_;
  bool initialized_{false};
  double last_filter_time_{0.0};

  // Kalman Filter
  KalmanFilter kf_;
};
