#pragma once

#include <string>

struct EstimatorConfig {
  double q_acc = 50.0;           // Process noise
  double r_detected = 4.0;       // Measurement covariance for detected input
  double r_tracked = 16.0;       // Measurement covariance for tracked input
  double p0_pos = 10.0;          // Initial position covariance
  double p0_vel = 100.0;         // Initial velocity covariance
  double max_gap = 0.5;          // Max valid gap without measurement

  static EstimatorConfig load(const std::string &path);
};
