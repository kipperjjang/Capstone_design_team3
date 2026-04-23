#include "controller/controller.hpp"

// Angle state KF based controller
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
      // Control 1. Give desired joint angle
      Eigen::Vector2d u = state.angle;

      // Control 2. Give error dynamics; desired joint velocity
      state.e = state.angle - state.joint;
      Eigen::Vector2d u = state.omega + config_.Kp * state.e; 

      u_.update(u, false, false);
      break;
    case FSMState::AIM:
      // Predict bell position
      Eigen::Vector2d v_bell = state.omega;
      Eigen::Vector2d p_pred = state.angle + v_bell * config_.time_delay;

      // Add offset for projectile motion and disturbance compensation
      Eigen::Vector2d p_aim = p_pred + offset;


      // Control 1. Give desired joint angle
      Eigen::Vector2d u = p_aim;

      // Control 2. Give error dynamics; desired joint velocity
      state.e = p_aim - state.joint;
      Eigen::Vector2d u = state.omega + config_.Kp * state.e;

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
