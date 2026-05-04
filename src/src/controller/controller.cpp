#include "controller/controller.hpp"

void Controller::run(const RobotState &state) {
  state_ = state;

  // Update FSM State
  fsm_.update(state_);

  // Handle based on the current state
  switch (fsm_.getFSMState()) {
    case FSMState::SEARCH: {
      // Wait; zero control input
      u_.update(Eigen::Vector2d::Zero(), false, false);
      break;
    }

    case FSMState::TRACK: {
      // Compute desired angle
      Eigen::Vector2d u = -config_.Kp * state.p - config_.Kd * state.v;

      u_.update(u, false, false);
      break;
    }

    case FSMState::AIM: {
      // Predict bell position
      Eigen::Vector2d v_bell = state.v;
      Eigen::Vector2d p_pred = state.p + v_bell * config_.time_delay;

      // Add offset for projectile motion and disturbance compensation
      Eigen::Vector2d p_aim = p_pred;

      // Compute desired angle
      Eigen::Vector2d u = -config_.Kp * p_aim - config_.Kd * state.v;
      // Shooting condition
      const bool fire = (p_pred).norm() < config_.err_p_fire && state_.v.norm() < config_.err_v_track;
      u_.update(u, fire, false);
      break;
    }

    case FSMState::RELOAD: {
      u_.update(Eigen::Vector2d::Zero(), false, true);
      break;
    }

    default: {
      u_.update(Eigen::Vector2d::Zero(), false, false);
      break;
    }
  }
}
