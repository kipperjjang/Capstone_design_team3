#include "estimator/estimator.hpp"

const ControlState& Estimator::getState() {
  auto state = state_;
  auto x = kf_->x();
  state.p = x.head<2>();
  state.v = x.tail<2>();
  return state;
}

void Estimator::init(const ControlState &state) {
  // Initialize estimator
  if (!state.detected) return;
  kf_.init(state.p)

  state_ = state;
  state_.process = false;

  last_filter_time_ = state.t;
  initialized_ = true;
  return;
}

void Estimator::update(const ControlState &state) {
  // Update estimator state
  state_ = state;
  const double t = state.t;
  const double dt = t - last_filter_time_;
  const bool detected = state.detected;
  const bool tracked = state.tracked;
  const Eigen::Vector2d p = state.p;

  state_.process = false;
  state_.dt = dt;

  // Update State with the vision input
  if (dt < 0.0) return;

  if (dt > config_.max_time_gap) {
    // Reset Filter
    kf_.reset();
    initialized_ = false;
    if (!detected) return;

    // Re-initialize with the new vision input
    kf_.init(p);
    initialized_ = true;
  } else {
    if (!tracked) return;
    kf_.predict(dt);
    kf_.update(p);
  }
  
  // Store data
  last_filter_time_ = t;
}

void Estimator::update(const double t) {
  const double dt = t - last_filter_time_;
  
  state_.process = true;
  state_.dt = dt;
  
  // Predict state via process
  if (dt > config_.max_time_gap) {
    kf_.reset();
    initialized_ = false;
    return;
    // No input to update; timer callback
  }
  kf_.predict(dt);
}

void Estimator::update(const Eigen::Vector2d &omega) {
  state_.w = lpf(omega);
}

