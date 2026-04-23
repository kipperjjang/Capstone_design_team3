#include "estimator/kalman_filter.hpp"

KalmanFilter::KalmanFilter(const EstimatorConfig &config) : config_(config) {
  x_pred_ = Eigen::VectorXd::Zero(6);
  x_ = Eigen::VectorXd::Zero(6);
  P_ = Eigen::MatrixXd::Identity(6, 6);
  H_ = Eigen::MatrixXd::Identity(4, 6);
}

Eigen::Vector4d KalmanFilter::computeBellAngle(const RobotState &state) {
  const double p = state.p(0);
  const double q = state.p(1);
  const double c1 = std::cos(state.joint(0));
  const double s1 = std::sin(state.joint(0));
  const double c2 = std::cos(state.joint(1));
  const double s2 = std::sin(state.joint(1));
  const double f = config_.focal_length;
  
  // Compute yaw and pitch angle of the bell with respect to the base frame
  const double alpha = std::atan2(-p*c1 + q*s1*s2 + f*s1*c2, p*s1+q*c1*s2+f*c1*c2);
  const double beta = std::atan2(-q*c2 + f*c2, std::sqrt(p*p + std::pow(q*s2 + f*c2, 2)))

  // Compute angular velocity of the bell
  const double R = std::sqrt(p*p + std::pow(q*s2+f*c2, 2));
  const double L = std::sqrt(p*p + q*q + f*f);
  state.Jp <<     -(q*s2 + f*c2)/(R*R),             p*s2/(R*R),
              -p*(f*s2 - q*c2)/(L*L*R), -(q*s2 + f*c2)/(L*L*R);
  state.Jj << 1, p*(q*c2 - f*s2),
              0, (q*s2 + f*c2)/R;
  Eigen::Vector2d omega = state.Jp * state.v + state.Jj * state.joint_vel;

  Eigen::Vector4d out;
  out << alpha, beta, omega;
  return out;
}

void KalmanFilter::init(const RobotState &state) {
  // Initialize
  x_.setZero();
  x_ = computeBellAngle(state);
  x_pred_ = x_;

  P_.setZero();
  P_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * config_.p0_pos;
  P_.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * config_.p0_vel;
  P_.block<2, 2>(4, 4) = Eigen::Matrix2d::Identity() * config_.p0_acc;
  initialized_ = true;
}

void KalmanFilter::reset() {
  x_.setZero();
  x_pred_.setZero();
  P_.setIdentity();
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

void KalmanFilter::predict(double dt, bool isProcess) {
  if (!initialized_ || dt <= 0.0) return;

  const Eigen::MatrixXd F = getF(dt);
  const Eigen::MatrixXd Q = getQ(dt);

  if (isProcess) {
    x_pred_ = F * x_;
  } else {
    x_ = F * x_;
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
  update(p, H_.block(0, 0, 2, 6), R, x_pred_);
}

void KalmanFilter::updateVelocity(const Eigen::Vector2d &v, const Eigen::Matrix2d &R) {
  if (!initialized_) return;
  update(v, H_.block(2, 0, 2, 6), R, x_pred_);
}

// void KalmanFilter::updateAcceleration(const Eigen::Vector2d &a, const Eigen::Matrix2d &R) {
//   if (!initialized_) return;
//   update(a, H_.block(4, 0, 2, 6), R, x_);
// }
