#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

struct CameraCalibration {
  Eigen::Matrix3d camera_matrix{Eigen::Matrix3d::Identity()};
  std::vector<double> distortion_coefficients;
  double fx{1360.0};
  double fy{1360.0};
  double cx{0.0};
  double cy{0.0};
  int image_width{0};
  int image_height{0};
  bool valid{false};
};

struct ControlConfig {
  Eigen::Vector2d img_offset{Eigen::Vector2d::Zero()}; 
  Eigen::Vector2d ang_offset{Eigen::Vector2d::Zero()}; 
  Eigen::Matrix2d Kp{Eigen::Matrix2d::Zero()};
  Eigen::Matrix2d Kd{Eigen::Matrix2d::Zero()};     // PD coefficient

  CameraCalibration webcam_calibration;
  CameraCalibration picam_calibration;

  double hz{50.0};            // Node frequency
  double time_delay{0.0};     // Expected time delay during fire
  double max_time_gap{0.15};  // Maximum time gap between vision message

  double ctrl_max_time_gap{1.0};  // Maximum time gap between ctrl
  double picam_prediction_max_sec{0.25};
  double picam_track_hold_sec{1.0};
  double webcam_measurement_max_age{0.3};

  double err_p_track{0.0};    // Error threshold for tracking, position
  double err_v_track{0.0};    // Error threshold for tracking, velocity
  double err_p_fire{0.0};     // Error threshold for aim, position

  const CameraCalibration& calibration(const std::string &camera_name) const;
  static ControlConfig load(const std::string &path);
};
