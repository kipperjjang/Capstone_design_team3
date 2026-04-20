#include "estimator/estimator.hpp"

const RobotState& Estimator::getState(bool isProcess) {
  if (kf_.isInitialized()) {
    const double f = config_.focal_length;
    const Eigen::VectorXd &x = isProcess ? kf_.x_pred() : kf_.x();
    state_.angle = x.segment<2>(0);
    state_.omega = x.segment<2>(2);
  }
  return state_;
}

void Estimator::init(const RobotState &state) {
  // Initialize estimator
  if (!state.detected && !state.tracked) return;

  kf_.init(state);

  state_ = state;
  state_.process = false;
  state_.dt = 0.0;

  last_measurement_time_ = state.t;
  initialized_ = true;
}

void Estimator::update(const RobotState &state) {
  // Update estimator state
  const double t = state.t;
  const double dt = t - last_measurement_time_;
  const bool has_measurement = state.detected || state.tracked;

  const Eigen::Vector4d x_obs = computeBellAngle(state);
  const Eigen::Matrix2d A = state.Jp;
  const Eigen::Matrix2d B = state.Jj;

  state_.process = false;
  state_.dt = dt;

  // Update State with the vision input
  if (dt < 0.0) return;

  if (dt > config_.max_time_gap) {
    // Reset Filter
    kf_.reset();
    initialized_ = false;
    if (!has_measurement) return;

    // Re-initialize with the new vision input
    kf_.init(state);
    initialized_ = true;
  } else {
    if (!has_measurement) return;

    // Compute Measurement noise
    const auto r_pixel = (2 * config_.r_detected / (dt*dt)) * Eigen::Matrix2d::Identity();
    const auto r_omega = config_.r_omega * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d R_pos = Eigen::Matrix2d::Identity() * config_.r_detected / config_.focal_length;
    const Eigen::Matrix2d R_vel = A * r_pixel * A.transpose() + B * r_omega * B.transpose();
    
    // Filter
    kf_.predict(dt, false);
    kf_.updatePosition(x_obs.segment<2>(0), R_pos);
    if (state.has_velocity) {
      kf_.updateVelocity(x_obs.segment<2>(2), R_vel);
    }
  }
  
  // Store data
  last_measurement_time_ = t;
}

void Estimator::update(const double t) {
  const double dt = t - last_measurement_time_;
  if (dt < 0.0) return;
  
  state_.process = true;
  state_.dt = dt;
  state_.t = t;
  
  // Predict state via process
  if (dt > config_.max_time_gap) {
    kf_.reset();
    initialized_ = false;
    return;
    // No input to update; timer callback
  }
  kf_.predict(dt, true);
}

void Estimator::update(const Eigen::Vector2d &joint, const Eigen::Vector2d &joint_vel, const double dt) {
  if (dt <= 0.0) return;

  const double sample_hz = 1/dt;
  state_.joint = lpf(joint, state_.joint, sample_hz/10, dt);
  state_.joint_vel = lpf(joint_vel, state_.joint_vel, sample_hz/10, dt);
}
