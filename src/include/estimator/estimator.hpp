#pragma once

#include "data/config/estimator_config.hpp"
#include "data/state/robot_state.hpp"
#include "estimator/kalman_filter.hpp"

#include "utils/lpf.hpp"

class Estimator {
public:
  Estimator(const EstimatorConfig &config)
    : config_(config), kf_(config_) {}

  void init(const RobotState &state);

  void update(const RobotState &state);       // Vision input
  void update(const double t);                // Process prediction
  void update(const Eigen::Vector2d &joint, const Eigen::Vector2d &joint_vel, const double dt); // Joint update

  // Utils
  const RobotState& getState(bool isProcess);
  bool isInitialized() const { return initialized_; }

private: 
  // Configuration
  EstimatorConfig config_;

  // State
  RobotState state_;
  bool initialized_{false};
  double last_measurement_time_{0.0};

  // Kalman Filter
  KalmanFilter kf_;
};
