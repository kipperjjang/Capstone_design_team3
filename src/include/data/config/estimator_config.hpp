#pragma once

#include <string>

struct EstimatorConfig {
  double hz = 100.0;             // Node frequency

  double q_acc = 50.0;           // Process noise
  double r_detected = 4.0;       // Measurement covariance for detected input
  double r_tracked = 16.0;       // Measurement covariance for tracked input
  double r_temp_vel = 16.0;      // Measurement covariance for external velocity hint
  double r_temp_acc = 64.0;      // Measurement covariance for external acceleration hint
  double p0_pos = 10.0;          // Initial position covariance
  double p0_vel = 100.0;         // Initial velocity covariance
  double p0_acc = 100.0;         // Initial acceleration covariance
  double max_time_gap = 0.5;     // Max valid gap without measurement

  static EstimatorConfig load(const std::string &path);
};
