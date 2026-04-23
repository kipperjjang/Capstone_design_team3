#pragma once

#include <Eigen/Dense>

#include "data/config/estimator_config.hpp"

class KalmanFilter {
public:
  KalmanFilter(const EstimatorConfig &config);

  void init(const Eigen::Vector2d &p, const Eigen::Vector2d &v = Eigen::Vector2d::Zero());
  void reset();

  void predict(double dt, bool isProcess);
  void updatePosition(const Eigen::Vector2d &p, const Eigen::Matrix2d &R);
  void updateVelocity(const Eigen::Vector2d &v, const Eigen::Matrix2d &R);
  void updateAcceleration(const Eigen::Vector2d &a, const Eigen::Matrix2d &R);
  Eigen::VectorXd getPredictedState(double dt) const;

  const Eigen::VectorXd& x() const { return x_; };
  const Eigen::VectorXd& x_pred() const { return x_pred_; };
  const Eigen::MatrixXd& P() const { return P_; };
  bool isInitialized() const { return initialized_; }

private:
  // Utils
  void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, const Eigen::VectorXd &prior);

  Eigen::MatrixXd getF(double dt) const;
  Eigen::MatrixXd getQ(double dt) const;
  EstimatorConfig config_;

  Eigen::VectorXd x_pred_, x_;        // State
  Eigen::MatrixXd P_;                 // Covariance
  Eigen::MatrixXd H_;                 // Observation model
  bool initialized_{false};
};
