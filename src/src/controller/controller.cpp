#include "controller/controller.hpp"

void Controller::run(const ControlState &state) {
  state_ = state;
  const double t = state.t;
  const double dt = state.dt;
  const bool detected = state.detected;
  const bool tracked = state.tracked;
  const bool process = state.process;
  const Eigen::Vector2d p = state.p;
  const Eigen::Vector2d v = state.v;

  // Update FSM State
  fsm_->update(state);

  // Handle based on the current state
  switch (fsm_->getState()) {
    case FSMState::SEARCH:
      // Wait; zero control input
      u_.update(Eigen::Vector2d::Zero(), false, false);
      break;
    case FSMState::TRACK:
      // Compute error
      Eigen::Vector2d err_p = p - config_.img_center;
      Eigen::Vector2d err_v = lpf(err_p, error_.p, config_.hz/10, dt);
      error_.p = err_p;
      error_.v = err_v;

      Eigen::Vector2d u = config_.K * err_p + config_.D * err_v;
      u_.update(u, false, false);
      break;
    case FSMState::AIM:
      Eigen::Vector2d v_bell = error_.v + v_cam_;
      Eigen::Vector2d p_aim = p + v_bell * config_.time_delay;
      Eigen::Vector2d err_aim_p = p_aim - config_.img_center;
      Eigen::Vector2d err_aim_v = lpf(err_aim_p, error_aim_.p, config_.hz/10, dt);

      Eigen::Vector2d u = config_.K * err_aim_p + config_.D * err_aim_v;
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
