#pragma once

#include "data/config/estimator_config.hpp"
#include "data/state/robot_state.hpp"
#include "estimator/kalman_filter.hpp"

class Estimator {
public:
  Estimator(const EstimatorConfig &config)
    : config_(config), kf_(config_) {}

  void init(const RobotState &state);

  void update(const RobotState &state);       // Vision input
  RobotState predict(double t);                       // KF predict with internal state change
  void updateJoint(const Eigen::Vector2d &joint, const Eigen::Vector2d &joint_vel, double t); // Joint update

  // Utils
  RobotState updatedState() const;                    // KF updated state with last vision measurement
  RobotState predictedState(double t) const;          // KF predicted state with absolute time without internal change; data for debug
  RobotState predictedStateFromDt(double dt) const;   // KF predicted state with dt; data for debug
  const RobotState& getState(bool isProcess);         // Backward-compatible state getter
  const RobotState& getState(double dt);
  bool isInitialized() const { return initialized_; }

  // Configuration
  EstimatorConfig config_;
  
private:
  RobotState attachJointMetadata(RobotState state) const;
  Eigen::Vector2d initialVelocityForMeasurement(const RobotState &state) const;
  RobotState stateFromFilter(const Eigen::VectorXd &x, RobotState metadata, bool process, double t, double dt) const;
  void reset();

  // State
  RobotState updated_state_;
  RobotState predicted_state_;
  RobotState last_measurement_state_;
  bool initialized_{false};
  bool has_last_measurement_state_{false};
  bool has_predicted_state_{false};
  double last_measurement_time_{0.0};
  double last_prediction_time_{0.0};

  // Joint
  Eigen::Vector2d latest_joint_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d latest_joint_vel_{Eigen::Vector2d::Zero()};
  double latest_joint_time_{0.0};
  bool has_latest_joint_{false};

  // Kalman Filter
  KalmanFilter kf_;
};
