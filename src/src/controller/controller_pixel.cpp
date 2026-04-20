#include "controller/controller.hpp"

// Compute rotation matrix from base to the camera (R_bc; camera relative to the base)
static Eigen::Matrix3d& getRotationMatrix(const Eigen::Vector2d &j) {
  const double c1 = std::cos(j(0));
  const double s1 = std::sin(j(0));
  const double c2 = std::cos(j(1));
  const double s2 = std::sin(j(1));

  Eigen::Matrix3d out;
  out <<  s1, c1*s2,  c1*c2,
         -c1, s1*s2,  s1*c2,
           0,   -c2,     s2;
  return out;
}

// Compute ray vector from camera to the bell in the camera frame
static Eigen::Vector3d getRayVector(const Eigen::Vector2d &p, double f) {
  Eigen::Vector3d out;
  out << p(0), p(1), f;
  return out.normalized();
}

// Compute pixel velocity of the camera in the image made by joint motions
static Eigen::Vector2d getCameraVelocity(const Eigen::Vector2d &pixel, const Eigen::Vector2d &j, const Eigen::Vector2d &j_vel, double f) {
  const double p = pixel(0);
  const double q = pixel(1);

  // Compute jacobian
  const double v00 = ((f*f + p*p) / f) * std::cos(j(1)) + q * std::sin(j(1));
  const double v01 = p * q / f;
  const double v10 = -p * std::sin(j(1)) + v01 * std::cos(j(1));
  const double v11 = (f*f + q*q) / f;
  
  Eigen::Matrix2d J;
  J << v00, v01,
       v10, v11;
  return J * j_vel;
}

void Controller::run(const RobotState &state) {
  state_ = state;
  const double t = state.t;
  const double dt = state.dt;
  const bool detected = state.detected;
  const bool tracked = state.tracked;
  const bool process = state.process;

  // Update FSM State
  fsm_->update(state);

  // Handle based on the current state
  switch (fsm_->getState()) {
    case FSMState::SEARCH:
      // Wait; zero control input
      u_.update(Eigen::Vector2d::Zero(), false, false);
      break;
    case FSMState::TRACK:
      // Compute desired angle
      Eigen::Vector2d u = computeIK(state.p, state.joint);

      u_.update(u, false, false);
      break;
    case FSMState::AIM:
      // Predict bell position
      Eigen::Vector2d v_bell = state.v;
      Eigen::Vector2d p_pred = state.p + v_bell * config_.time_delay;

      // Add offset for projectile motion and disturbance compensation
      Eigen::Vector2d p_aim = p_pred + offset;

      // Compute desired angle
      Eigen::Vector2d u = computeIK(p_aim, state.joint);
      u_.update(u, false, false);

      // Shooting condition satisfied
      u_.update(u, true, false);
      break;
    case FSMState::RELOAD:
      /* code */
      u_.update(u, false, true);
      break;
    default:
      break;
  }
}

Eigen::Vector2d Controller::computeIK(const Eigen::Vector2d &p, const Eigen::Vector2d &joint) {
  // Get rotation matrix
  Eigen::Matrix3d& R_bc = getRotationMatrix(joint);
  
  // Get ray vector
  Eigen::Vector3d u_c = getRayVector(p, config_.focal_length);

  // Ray vector in base frame coordinate
  Eigen::Vector3d u_b = R_bc * u_c;

  Eigen::Vector2d out;
  out << std::atan2(u_b(1), u_b(0)),
         std::atan2(u_b(2), std::sqrt(u_b(0)*u_b(0) + u_b(1)*u_b(1)));
  return out;
}