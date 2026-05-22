#include "estimator/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const EstimatorConfig &config) : config_(config) {
  x_        = Eigen::VectorXd::Zero(6);
  x_prior_  = Eigen::VectorXd::Zero(6);
  P_        = Eigen::MatrixXd::Identity(6, 6);
  P_prior_  = Eigen::MatrixXd::Identity(6, 6);
  H_        = Eigen::MatrixXd::Identity(6, 6);
}

void KalmanFilter::init(const Eigen::Vector2d &p, const Eigen::Vector2d &v) {
  x_.setZero();
  x_.segment<2>(0) = p;
  x_.segment<2>(2) = v;
  x_prior_ = x_;

  P_.setZero();
  P_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * config_.p0_pos;
  P_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * config_.p0_vel;
  P_.block<2, 2>(4, 4) = Eigen::Matrix2d::Identity() * config_.p0_acc;
  P_prior_ = P_;

  initialized_ = true;
}

void KalmanFilter::reset() {
  x_.setZero();
  x_prior_.setZero();
  P_.setIdentity();
  P_prior_.setIdentity();
  initialized_ = false;
}

Eigen::MatrixXd KalmanFilter::getF(double dt) const {
  const double dt2 = dt * dt;
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 2) = dt;
  F(1, 3) = dt;
  F(0, 4) = 0.5 * dt2;
  F(1, 5) = 0.5 * dt2;
  F(2, 4) = dt;
  F(3, 5) = dt;

  return F;
}

Eigen::MatrixXd KalmanFilter::getQ(double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double dt5 = dt4 * dt;
  const double q = config_.q_acc;

  Eigen::Matrix3d q_axis = Eigen::Matrix3d::Zero();
  q_axis(0, 0) = dt5 / 20.0;
  q_axis(0, 1) = dt4 / 8.0;
  q_axis(0, 2) = dt3 / 6.0;
  q_axis(1, 0) = dt4 / 8.0;
  q_axis(1, 1) = dt3 / 3.0;
  q_axis(1, 2) = dt2 / 2.0;
  q_axis(2, 0) = dt3 / 6.0;
  q_axis(2, 1) = dt2 / 2.0;
  q_axis(2, 2) = dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
  const Eigen::Matrix3d q_scaled = q * q_axis;
  Q(0, 0) = q_scaled(0, 0);
  Q(0, 2) = q_scaled(0, 1);
  Q(0, 4) = q_scaled(0, 2);
  Q(2, 0) = q_scaled(1, 0);
  Q(2, 2) = q_scaled(1, 1);
  Q(2, 4) = q_scaled(1, 2);
  Q(4, 0) = q_scaled(2, 0);
  Q(4, 2) = q_scaled(2, 1);
  Q(4, 4) = q_scaled(2, 2);

  Q(1, 1) = q_scaled(0, 0);
  Q(1, 3) = q_scaled(0, 1);
  Q(1, 5) = q_scaled(0, 2);
  Q(3, 1) = q_scaled(1, 0);
  Q(3, 3) = q_scaled(1, 1);
  Q(3, 5) = q_scaled(1, 2);
  Q(5, 1) = q_scaled(2, 0);
  Q(5, 3) = q_scaled(2, 1);
  Q(5, 5) = q_scaled(2, 2);
  return Q;
}

void KalmanFilter::predict(double dt) {
  if (!initialized_) return;
  if (dt <= 0.0) {
    x_prior_ = x_;
    P_prior_ = P_;
    return;
  }

  const Eigen::MatrixXd F = getF(dt);
  const Eigen::MatrixXd Q = getQ(dt);

  x_prior_ = F * x_;
  P_prior_ = F * P_ * F.transpose() + Q;
}

Eigen::VectorXd KalmanFilter::predictedState(double dt) const {
  if (!initialized_ || dt <= 0.0) return x_;
  return getF(dt) * x_;
}

void KalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
  if (!initialized_) return;

  const Eigen::VectorXd y = z - H * x_prior_;
  const Eigen::MatrixXd S = H * P_prior_ * H.transpose() + R;
  const Eigen::MatrixXd K = P_prior_ * H.transpose() * S.inverse();
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

  x_ = x_prior_ + K * y;
  P_ = (I - K * H) * P_prior_;
  x_prior_ = x_;
  P_prior_ = P_;
}

void KalmanFilter::updatePosition(const Eigen::Vector2d &p, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(p, H_.block(0, 0, 2, 6), R);
}

void KalmanFilter::updateVelocity(const Eigen::Vector2d &v, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(v, H_.block(2, 0, 2, 6), R);
}

void KalmanFilter::updateAcceleration(const Eigen::Vector2d &a, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(a, H_.block(4, 0, 2, 6), R);
}
