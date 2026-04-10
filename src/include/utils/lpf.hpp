#pragma once

#include <Eigen/Dense>
#include <cmath>

inline Eigen::VectorXd lpf(const Eigen::VectorXd &input, const Eigen::VectorXd &output, double cutoff_frequency, double dt) {
  if (cutoff_frequency <= 0.0 || dt <= 0.0) return output;

  const double tau = 1.0 / (2.0 * M_PI * cutoff_frequency);
  const double beta = dt / (tau + dt);
  return output + beta * (input - output);
}
