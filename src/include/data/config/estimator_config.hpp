#pragma once

#include <string>

struct EstimatorConfig {
  double hz;                // Node frequency

  double q_acc;             // Process noise
  double r_pos;             // Measurement covariance for detected pose
  double r_vel;             // Measurement covariance for external velocity hint
  double r_acc;             // Measurement covariance for external acceleration hint
  
  double p0_pos;            // Initial position covariance
  double p0_vel;            // Initial velocity covariance
  double p0_acc;            // Initial acceleration covariance
  double max_time_gap;      // Max valid gap without measurement

  double prediction_time;   // Prediction time to compensate lag

  static EstimatorConfig load(const std::string &path);
};
