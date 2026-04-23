#pragma once

#include <string>

struct EstimatorConfig {
  double hz;                // Node frequency

  double focal_length;      // Focal length of the camera
  double q_acc;             // Process noise
  double r_detected;        // Measurement covariance for detected input
  double r_tracked;         // Measurement covariance for tracked input
  double r_temp_vel;        // Measurement covariance for external velocity hint
  double r_temp_acc;        // Measurement covariance for external acceleration hint

  double r_angle;           // Measurement covarinace for encoder angle
  double r_omega;           // Measurement covarinace for encoder velocity
  
  double p0_pos;            // Initial position covariance
  double p0_vel;            // Initial velocity covariance
  double p0_acc;            // Initial acceleration covariance
  double max_time_gap;      // Max valid gap without measurement

  static EstimatorConfig load(const std::string &path);
};
