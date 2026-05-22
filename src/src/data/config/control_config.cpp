#include "data/config/control_config.hpp"

#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {
CameraCalibration loadCameraCalibration(const YAML::Node &node) {
  CameraCalibration calibration;
  if (!node) {
    return calibration;
  }

  calibration.image_width = node["image_width"].as<int>(calibration.image_width);
  calibration.image_height = node["image_height"].as<int>(calibration.image_height);

  const auto matrix = node["camera_matrix"].as<std::vector<double>>(std::vector<double>{});
  if (matrix.size() == 9) {
    calibration.camera_matrix <<
      matrix[0], matrix[1], matrix[2],
      matrix[3], matrix[4], matrix[5],
      matrix[6], matrix[7], matrix[8];
    calibration.fx = calibration.camera_matrix(0, 0);
    calibration.fy = calibration.camera_matrix(1, 1);
    calibration.cx = calibration.camera_matrix(0, 2);
    calibration.cy = calibration.camera_matrix(1, 2);
    calibration.valid = true;
  }

  calibration.distortion_coefficients =
    node["distortion_coefficients"].as<std::vector<double>>(std::vector<double>{});

  return calibration;
}
}  // namespace

const CameraCalibration& ControlConfig::calibration(const std::string &camera_name) const {
  if (camera_name == "webcam") {
    if (!webcam_calibration.valid) {
      throw std::runtime_error("webcam calibration is not valid.");
    }
    return webcam_calibration;
  }

  if (camera_name == "picam") {
    if (!picam_calibration.valid) {
      throw std::runtime_error("picam calibration is not valid.");
    }
    return picam_calibration;
  }

  throw std::runtime_error("Unknown camera name: " + camera_name + ". Expected 'webcam' or 'picam'.");
}

ControlConfig ControlConfig::load(const std::string &path) {
  ControlConfig config;
  if (path.empty()) {
    return config;
  }

  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node controller = file["controller"];
  if (!controller) {
    return config;
  }

  const YAML::Node calibration = file["calibration"];
  if (calibration) {
    config.webcam_calibration = loadCameraCalibration(calibration["webcam"]);
    config.picam_calibration = loadCameraCalibration(calibration["picam"]);
  }

  const auto img_offset = controller["image_offset"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.img_offset = Eigen::Vector2d(img_offset[0], img_offset[1]);

  const auto ang_offset = controller["angle_offset"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.ang_offset = Eigen::Vector2d(ang_offset[0], ang_offset[1]);

  const auto kp_diag = controller["Kp"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.Kp = Eigen::Vector2d(kp_diag[0], kp_diag[1]).asDiagonal();

  const auto kd_diag = controller["Kd"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
  config.Kd = Eigen::Vector2d(kd_diag[0], kd_diag[1]).asDiagonal();

  config.hz = controller["frequency"].as<double>(config.hz);
  config.time_delay = controller["time_delay"].as<double>(config.time_delay);
  config.max_time_gap = controller["max_time_gap"].as<double>(config.max_time_gap);
  config.ctrl_max_time_gap = controller["ctrl_max_time_gap"].as<double>(config.ctrl_max_time_gap);
  config.picam_prediction_max_sec = controller["picam_prediction_max_sec"].as<double>(config.picam_prediction_max_sec);
  config.picam_track_hold_sec = controller["picam_track_hold_sec"].as<double>(config.picam_track_hold_sec);
  config.webcam_measurement_max_age = controller["webcam_measurement_max_age"].as<double>(config.webcam_measurement_max_age);
  config.err_p_track = controller["err_p_track"].as<double>(config.err_p_track);
  config.err_v_track = controller["err_v_track"].as<double>(config.err_v_track);
  config.err_p_fire = controller["err_p_fire"].as<double>(config.err_p_fire);

  return config;
}
