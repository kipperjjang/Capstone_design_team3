#include "controller/controller.hpp"

#include <cmath>

namespace {
double distortionCoeff(const CameraCalibration &calibration, size_t index) {
  return index < calibration.distortion_coefficients.size() ? calibration.distortion_coefficients[index] : 0.0;
}

Eigen::Vector2d pixelToUndistortedRay(const Eigen::Vector2d &pixel, const CameraCalibration &calibration) {
  const double xd = (pixel.x() - calibration.cx) / calibration.fx;
  const double yd = (pixel.y() - calibration.cy) / calibration.fy;

  if (calibration.distortion_coefficients.empty()) {
    return Eigen::Vector2d(xd, yd);
  }

  const double k1 = distortionCoeff(calibration, 0);
  const double k2 = distortionCoeff(calibration, 1);
  const double p1 = distortionCoeff(calibration, 2);
  const double p2 = distortionCoeff(calibration, 3);
  const double k3 = distortionCoeff(calibration, 4);
  const double k4 = distortionCoeff(calibration, 5);
  const double k5 = distortionCoeff(calibration, 6);
  const double k6 = distortionCoeff(calibration, 7);
  const double s1 = distortionCoeff(calibration, 8);
  const double s2 = distortionCoeff(calibration, 9);
  const double s3 = distortionCoeff(calibration, 10);
  const double s4 = distortionCoeff(calibration, 11);

  double x = xd;
  double y = yd;
  for (int i = 0; i < 8; ++i) {
    const double x2 = x*x;
    const double y2 = y*y;
    const double xy = x*y;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;
    const double r6 = r2 * r4;

    const double radial_num = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double radial_den = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
    const double radial = radial_num / radial_den;
    if (std::abs(radial) < 1e-12) {
      break;
    }

    const double delta_x = 2.0*p1*xy + p2*(r2 + 2.0*x2) + s1*r2 + s2*r4;
    const double delta_y = p1*(r2 + 2.0*y2) + 2.0*p2*xy + s3*r2 + s4*r4;

    x = (xd - delta_x) / radial;
    y = (yd - delta_y) / radial;
  }

  return Eigen::Vector2d(x, y);
}

Eigen::Vector2d computeBellAngle(const RobotState &state, const ControlConfig &config) {
  const CameraCalibration &calibration = config.calibration(state.camera);
  // const double center_x = calibration.cx;
  // const double center_y = calibration.cy;
  // const double p = (state.p(0) - center_x) / calibration.fx;
  // const double q = (state.p(1) - center_y) / calibration.fy;
  
  Eigen::Vector2d joint = state.camera == "picam" ? state.joint : Eigen::Vector2d(0.0, 0.0);
  const Eigen::Vector2d ray = pixelToUndistortedRay(state.p, calibration);
  const double p = ray.x();
  const double q = ray.y();
  const double c1 = std::cos(joint(0));
  const double s1 = std::sin(joint(0));
  const double c2 = std::cos(joint(1));
  const double s2 = std::sin(joint(1));
  const double f = 1.0;
  
  // std::cout << "joint:\t" << joint(0) << "\t" << joint(1) << std::endl;
  // Compute yaw and pitch angle of the bell with respect to the base frame
  const double alpha = std::atan2(-p*c1 - q*s1*s2 + f*s1*c2, p*s1 - q*c1*s2+f*c1*c2);
  const double n = q*s2 + f*c2;
  const double beta = std::atan2(q*c2 + f*s2, std::sqrt(p*p + n*n));

  // Compute angular velocity of the bell
  // const double R = std::sqrt(p*p + std::pow(q*s2+f*c2, 2));
  // const double L = std::sqrt(p*p + q*q + f*f);
  // Eigen::Matrix2d Jp, Jj;
  // Jp <<     -(q*s2 + f*c2)/(R*R),             p*s2/(R*R),
  //             -p*(f*s2 - q*c2)/(L*L*R), -(q*s2 + f*c2)/(L*L*R);
  // Jj << 1.0, p*(q*c2 - f*s2),
  //             0, (q*s2 + f*c2)/R;
  // Eigen::Vector2d omega = Jp * state.v + Jj * state.joint_vel;

  Eigen::Vector2d out;
  out << alpha, beta;
  return out;
}
} // namespace

ControlState Controller::computeWebcamAngleSearch(const RobotState &webcam_state) const {
  return ControlState::angle(computeBellAngle(webcam_state, config_));
}

ControlState Controller::computePicamPixelTrack(const RobotState &picam_state) const {
  const Eigen::Vector2d img_center = picam_state.img_center + config_.img_offset;
  Eigen::Vector2d u = config_.Kp * (img_center - picam_state.p);
  u = (M_PI / 180.0) * u;
  return ControlState::pixel(u);
}

ControlState Controller::computeJointHold(const Eigen::Vector2d &joint) const {
  return ControlState::angle(joint);
}
