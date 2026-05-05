#include "estimator/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const EstimatorConfig &config) : config_(config) {
  x_pred_ = Eigen::VectorXd::Zero(4);
  x_ = Eigen::VectorXd::Zero(4);
  P_ = Eigen::MatrixXd::Identity(4, 4);
  H_ = Eigen::MatrixXd::Identity(4, 4);
}

void KalmanFilter::init(const Eigen::Vector2d &p, const Eigen::Vector2d &v) {
  x_.setZero();
  x_.segment<2>(0) = p;
  x_.segment<2>(2) = v;
  x_pred_ = x_;

  P_.setZero();
  P_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * config_.p0_pos;
  P_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * config_.p0_vel;

  initialized_ = true;
}

void KalmanFilter::reset() {
  x_.setZero();
  x_pred_.setZero();
  P_.setIdentity();
  initialized_ = false;
}

Eigen::MatrixXd KalmanFilter::getF(double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
  F(0, 2) = dt;
  F(1, 3) = dt;

  return F;
}

Eigen::MatrixXd KalmanFilter::getQ(double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double q = config_.q_acc;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  Q(0, 0) = dt4 / 4.0;
  Q(0, 1) = dt3 / 2.0;
  Q(1, 0) = dt3 / 2.0;
  Q(1, 1) = dt2;
  Q(2, 2) = dt4 / 4.0;
  Q(2, 3) = dt3 / 2.0;
  Q(3, 2) = dt3 / 2.0;
  Q(3, 3) = dt2;
  return Q;
}

void KalmanFilter::predict(double dt, bool isProcess) {
  if (!initialized_ || dt <= 0.0) return;

  const Eigen::MatrixXd F = getF(dt);
  const Eigen::MatrixXd Q = getQ(dt);

  if (isProcess) {
    x_pred_ = F * x_;
  } else {
    x_ = F * x_;
    x_pred_ = x_;
    P_ = F * P_ * F.transpose() + Q;
  }
}

Eigen::VectorXd KalmanFilter::getPredictedState(double dt) const {
  if (!initialized_ || dt <= 0.0) return x_;
  return getF(dt) * x_;
}

void KalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &prior) {
  const Eigen::VectorXd y = z - H * prior;
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

  x_ = prior + K * y;
  x_pred_ = x_;
  P_ = (I - K * H) * P_;
}

void KalmanFilter::updatePosition(const Eigen::Vector2d &p, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(p, H_.block(0, 0, 2, 4), R, x_pred_);
}

void KalmanFilter::updateVelocity(const Eigen::Vector2d &v, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(v, H_.block(2, 0, 2, 4), R, x_pred_);
}

// void KalmanFilter::updateAcceleration(const Eigen::Vector2d &a, const Eigen::Matrix2d &R) {
//   if (!initialized_) return;
//   update(a, H_.block(4, 0, 2, 6), R, x_pred_);
// }
