#pragma once

#include <Eigen/Dense>
#include "data/estimator_config.hpp"

class KalmanFilter {
public:
  KalmanFilter(const EstimatorConfig &config);

  void init(const Eigen::Vector2d &p);
  void reset();

  void predict(double dt);
  void update(const Eigen::Vector2d &p);

  const Eigen::VectorXd& x() const { return x_; };
  const Eigen::VectorXd& P() const { return P_; };
  bool isInitialized() const { return initialized_; }

private:
  Eigen::MatrixXd getF(double dt) const;
  Eigen::MatrixXd getQ(double dt) const;
  EstimatorConfig config_;

  Eigen::VectorXd x_;   // State
  Eigen::MatrixXd P_;   // Covariance
  Eigen::MatrixXd H_;   // Observation model
  bool initialized_{false};
};
