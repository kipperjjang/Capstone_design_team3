#include "estimator/estimator.hpp"

#include <algorithm>

RobotState Estimator::attachJointMetadata(RobotState state) const {
  if (has_latest_joint_) {
    state.joint = latest_joint_;
    state.joint_vel = latest_joint_vel_;
  }
  return state;
}

Eigen::Vector2d Estimator::initialVelocityForMeasurement(const RobotState &state) const {
  if (state.has_velocity) {
    return state.v;
  }

  if (has_last_measurement_state_) {
    const double dt = state.t - last_measurement_state_.t;
    if (dt > 0.0 && dt <= config_.max_time_gap && last_measurement_state_.detected) {
      return (state.p - last_measurement_state_.p) / dt;
    }
  }

  return Eigen::Vector2d::Zero();
}

RobotState Estimator::stateFromFilter(
    const Eigen::VectorXd &x,
    RobotState metadata,
    bool process,
    double t,
    double dt) const {
  RobotState state = attachJointMetadata(metadata);
  if (x.size() >= 6) {
    state.p = x.segment<2>(0);
    state.v = x.segment<2>(2);
    state.a = x.segment<2>(4);
    state.has_velocity = true;
    state.has_acceleration = true;
  }
  state.t = t;
  state.dt = dt;
  state.process = process;
  if (process) {
    state.detected = false;
  }
  return state;
}

void Estimator::init(const RobotState &state) {
  // Initialize estimator
  if (!state.detected) return;

  const RobotState measurement = attachJointMetadata(state);
  const Eigen::Vector2d initial_velocity = initialVelocityForMeasurement(measurement);
  kf_.init(measurement.p, initial_velocity);

  updated_state_ = stateFromFilter(kf_.x(), measurement, false, measurement.t, 0.0);
  predicted_state_ = updated_state_;
  has_predicted_state_ = false;

  last_measurement_state_ = measurement;
  has_last_measurement_state_ = true;
  last_measurement_time_ = measurement.t;
  last_prediction_time_ = measurement.t;
  initialized_ = kf_.isInitialized();
}

void Estimator::reset() {
  kf_.reset();
  initialized_ = false;
  has_predicted_state_ = false;
}

void Estimator::update(const RobotState &state) {
  if (!state.detected) return;

  // Get current measurement
  const RobotState measurement = attachJointMetadata(state);
  if (!initialized_) {
    init(state);
    return;
  }
  
  // Time check
  const double dt = measurement.t - last_measurement_time_;
  if (dt < 0.0) return;
  if (dt > config_.max_time_gap) {
    reset();
    init(measurement);
    return;
  }

  // Compute Kalman prior
  kf_.predict(dt);

  // Update state
  kf_.updatePosition(measurement.p, Eigen::Matrix2d::Identity() * config_.r_pos);
  // if (measurement.has_velocity) {
  //   kf_.updateVelocity(measurement.v, Eigen::Matrix2d::Identity() * config_.r_vel);
  // }
  // if (measurement.has_acceleration) {
  //   kf_.updateAcceleration(measurement.a, Eigen::Matrix2d::Identity() * config_.r_acc);
  // }

  updated_state_ = stateFromFilter(kf_.x(), measurement, false, measurement.t, dt);
  predicted_state_ = updated_state_;
  has_predicted_state_ = false;

  last_measurement_state_ = measurement;
  has_last_measurement_state_ = true;
  last_measurement_time_ = measurement.t;
  initialized_ = kf_.isInitialized();
}

RobotState Estimator::predict(double t) {
  const RobotState last_state = has_predicted_state_ ? predicted_state_ : updated_state_;
  if (!initialized_) {
    return attachJointMetadata(last_state);
  }

  // Time check
  const double dt = t - last_measurement_time_;
  if (dt < 0.0) return last_state;
  if (dt > config_.max_time_gap) {
    reset();
    RobotState state = attachJointMetadata(last_state);
    state.t = t;
    state.dt = dt;
    state.process = true;
    state.detected = false;
    predicted_state_ = state;
    has_predicted_state_ = true;
    last_prediction_time_ = t;
    return predicted_state_;
  }

  predicted_state_ = stateFromFilter(kf_.predictedState(dt), updated_state_, true, t, dt);
  has_predicted_state_ = true;
  last_prediction_time_ = t;
  return predicted_state_;
}

void Estimator::updateJoint(const Eigen::Vector2d &joint, const Eigen::Vector2d &joint_vel, double t) {
  latest_joint_ = joint;
  latest_joint_vel_ = joint_vel;
  latest_joint_time_ = t;
  has_latest_joint_ = true;
  
  updated_state_ = attachJointMetadata(updated_state_);
  if (has_predicted_state_) {
    predicted_state_ = attachJointMetadata(predicted_state_);
  }
}

RobotState Estimator::updatedState() const {
  return attachJointMetadata(updated_state_);
}

RobotState Estimator::predictedState(double t) const {
  const RobotState last_state = has_predicted_state_ ? predicted_state_ : updated_state_;
  if (!initialized_) {
    return attachJointMetadata(last_state);
  }

  const double dt = t - last_measurement_time_;
  if (dt < 0.0) {
    return attachJointMetadata(last_state);
  }

  return predictedStateFromDt(dt);
}

RobotState Estimator::predictedStateFromDt(double dt) const {
  const RobotState last_state = has_predicted_state_ ? predicted_state_ : updated_state_;
  if (!initialized_) {
    return attachJointMetadata(last_state);
  }

  const double clamped_dt = std::max(0.0, dt);
  return stateFromFilter(kf_.predictedState(clamped_dt), updated_state_, true, last_measurement_time_ + clamped_dt, clamped_dt);
}
