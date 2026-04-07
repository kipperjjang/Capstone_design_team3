#include "estimator/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const EstimatorConfig &config) : config_(config) {
  x_ = Eigen::VectorXd::Zero(4);
  P_ = Eigen::MatrixXd::Identity(4, 4);

  H_ = Eigen::MatrixXd::Zero(2, 4);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
}

void KalmanFilter::init(const Eigen::Vector2d &p) {
  // Initialize the state vector
  x_.setZero();
  x_.head<2>(0) = p;

  P_.setZero();
  P_(0, 0) = config_.p0_pos;
  P_(1, 1) = config_.p0_pos;
  P_(2, 2) = config_.p0_vel;
  P_(3, 3) = config_.p0_vel;

  initialized_ = true;
}

void KalmanFilter::reset() {
  x_.setZero();
  P_.setIdentity();
  initialized_ = false;
}

Eigen::MatrixXd KalmanFilter::getF(double dt) const {
  // Process function
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

Eigen::MatrixXd KalmanFilter::getQ(double dt) const {
  // Process noise covariance
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double q = config_.q_acc;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  Q(0, 0) = 0.25 * dt4 * q;
  Q(1, 1) = 0.25 * dt4 * q;
  Q(0, 2) = 0.5 * dt3 * q;
  Q(1, 3) = 0.5 * dt3 * q;
  Q(2, 0) = 0.5 * dt3 * q;
  Q(3, 1) = 0.5 * dt3 * q;
  Q(2, 2) = dt2 * q;
  Q(3, 3) = dt2 * q;
  return Q;
}

void KalmanFilter::predict(double dt) {
  // One forward step
  if (!initialized_ || dt <= 0.0) return;

  // Predict state and covariance vector
  const Eigen::MatrixXd F = getF(dt);
  const Eigen::MatrixXd Q = getQ(dt);

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

Eigen::VectorXd KalmanFilter::getPredictedState(double dt) const {
  if (!initialized_ || dt <= 0.0) return x_;
  return getF(dt) * x_;
}

void KalmanFilter::update(const Eigen::Vector2d &p, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  
  // Observed state
  const Eigen::Vector2d y = p - H_ * x_;

  // Compute Kalman Gain
  const Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R;
  const Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  
  // Estimate state
  x_ = x_ + K * y;
  P_ = (I - K*H_) * P_;
}
