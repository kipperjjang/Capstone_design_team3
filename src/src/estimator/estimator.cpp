#include "estimator/estimator.hpp"

const RobotState &Estimator::getState(bool isProcess) {
  if (kf_.isInitialized()) {
    const Eigen::VectorXd &x = isProcess ? kf_.x_pred() : kf_.x();
    state_.p = x.segment<2>(0);
    state_.v = x.segment<2>(2);
    state_.a = x.segment<2>(4);
    state_.has_velocity = true;
    state_.has_acceleration = true;
  }
  return state_;
}

void Estimator::init(const RobotState &state) {
  // Initialize estimator
  if (!state.detected && !state.tracked) return;

  kf_.init(state.p, state.has_velocity ? state.v : Eigen::Vector2d::Zero());

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

  state_ = state;
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
    kf_.init(state.p, state.has_velocity ? state.v : Eigen::Vector2d::Zero());
    initialized_ = true;
  } else {
    if (!has_measurement) return;

    kf_.predict(dt, false);
    const double r_position = state.detected ? config_.r_detected : config_.r_tracked;
    kf_.updatePosition(state.p, Eigen::Matrix2d::Identity() * r_position);
    if (state.has_velocity) {
      kf_.updateVelocity(state.v, Eigen::Matrix2d::Identity() * config_.r_temp_vel);
    }
    if (state.has_acceleration) {
      kf_.updateAcceleration(state.a, Eigen::Matrix2d::Identity() * config_.r_temp_acc);
    }
  }

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

  const double sample_hz = 1.0 / dt;
  state_.joint = lpf(joint, state_.joint, sample_hz / 10.0, dt);
  state_.joint_vel = lpf(joint_vel, state_.joint_vel, sample_hz / 10.0, dt);
}
