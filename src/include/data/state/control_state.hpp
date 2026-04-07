#pragma once

#include <Eigen/Dense>

/* State structure */
struct ControlState {
  double u_yaw{0.0};
  double u_pitch{0.0};
  bool fire{false};
  bool reload{false};
  
  static void update(const Eigen::Vector2d &u, bool _fire, bool _reload) {
    u_yaw = u(0);
    u_pitch = u(1);
    fire = _fire;
    reload = _reload;
  }
};

