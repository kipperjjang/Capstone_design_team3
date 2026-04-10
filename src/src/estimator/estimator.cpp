#include "estimator/estimator.hpp"

const RobotState& Estimator::getState(bool isProcess) {
  if (kf_.isInitialized()) {
    const Eigen::VectorXd &x = isProcess ? kf_.x_pred() : kf_.x();
    state_.p = x.head<2>();
    state_.v = x.segment<2>(2);
  }
  return state_;
}

void Estimator::init(const RobotState &state) {
  // Initialize estimator
  if (!state.detected && !state.tracked) return;

  kf_.init(state.p, state.v);

  state_ = state;
  state_.process = false;
  state_.dt = 0.0;

  last_measurement_time_ = state.t;
  initialized_ = true;
}

void Estimator::update(const RobotState &state) {
  // Update estimator state
  state_ = state;
  const double t = state.t;
  const double dt = t - last_measurement_time_;
  const bool has_measurement = state.detected || state.tracked;
  const Eigen::Vector2d& p = state.p;
  const Eigen::Vector2d& v = state.v;
  const Eigen::Vector2d& a = state.a;

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
    kf_.init(p, v);
    initialized_ = true;
  } else {
    if (!has_measurement) return;

    const double r = state.detected ? config_.r_detected : config_.r_tracked;
    const Eigen::Matrix2d R_pos = Eigen::Matrix2d::Identity() * r;
    const Eigen::Matrix2d R_vel = Eigen::Matrix2d::Identity() * config_.r_temp_vel;
    const Eigen::Matrix2d R_acc = Eigen::Matrix2d::Identity() * config_.r_temp_acc;
    kf_.predict(dt, false);
    kf_.updatePosition(p, R_pos);
    if (state.has_velocity) {
      kf_.updateVelocity(v, R_vel);
    }
    if (state.has_acceleration) {
      kf_.updateAcceleration(a, R_acc);
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

void Estimator::update(const Eigen::Vector2d &omega, const double dt) {
  if (dt <= 0.0) return;

  const double sample_hz = 1/dt;
  state_.w = lpf(omega, state_.w, sample_hz/10, dt);
}
