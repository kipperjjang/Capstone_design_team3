#pragma once

#include <Eigen/Dense>

struct ControlState {
  double u_yaw{0.0};
  double u_pitch{0.0};
  bool isPixel{false};
  bool fire{false};

  void update(bool _fire) {
    fire = _fire;
  }

  void update(const Eigen::Vector2d &u, bool _isPixel) {
    u_yaw = u(0);
    u_pitch = u(1);
    isPixel = _isPixel;
    fire = false;
  }

  static ControlState pixel(const Eigen::Vector2d &u) {
    ControlState out;
    out.update(u, true);
    return out;
  }

  static ControlState angle(const Eigen::Vector2d &u) {
    ControlState out;
    out.update(u, false);
    return out;
  }
};
