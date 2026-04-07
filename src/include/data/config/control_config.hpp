#pragma once
#include <string>
#include <Eigen/Dense>

struct ControlConfig {
  Eigen::Vector2d img_center; // Pixel of the center of the image
  Eigen::Matrix2d K, D;       // PD coefficient  

  double controller_hz;       // Controller node frequency
  double time_delay;          // Expected time delay during fire
  double max_time_gap;        // Maximum time gap between vision message

  double err_p_track;         // Error threshold for tracking, position
  double err_v_track;         // Error threshold for tracking, velocity
  double err_p_fire;          // Error threshold for aim, position

  static ControlConfig load(const std::string &path);
};
 