#pragma once

#include <Eigen/Dense>

#include <string>

struct ControlConfig {
  Eigen::Matrix2d Kp, Kd;     // PD coefficient

  double hz;                  // Node frequency
  double time_delay;          // Expected time delay during fire
  double max_time_gap;        // Maximum time gap between vision message

  double err_p_track;         // Error threshold for tracking, position
  double err_v_track;         // Error threshold for tracking, velocity
  double err_p_fire;          // Error threshold for aim, position

  static ControlConfig load(const std::string &path);
};
