#include "controller/fsm.hpp"

void FSM::update(const RobotState &state) {
  const double dt = state.dt;
  const bool has_target = state.detected || state.tracked;
  const double err_p = state.p.norm();
  const double err_v = state.v.norm();

  switch (fsm_state_) {
    case FSMState::SEARCH:
      if (has_target) {
        fsm_state_ = FSMState::TRACK;
      }
      break;

    case FSMState::TRACK:
      if (!has_target || dt > config_.max_time_gap) {
        fsm_state_ = FSMState::SEARCH;
      } else if (err_p < config_.err_p_track && err_v < config_.err_v_track) {
        fsm_state_ = FSMState::AIM;
      }
      break;

    case FSMState::AIM:
      if (!has_target || dt > config_.max_time_gap) {
        fsm_state_ = FSMState::SEARCH;
      } else if (err_p < config_.err_p_fire && err_v < config_.err_v_track) {
        fsm_state_ = FSMState::RELOAD;
      } else if (err_p >= config_.err_p_track) {
        fsm_state_ = FSMState::TRACK;
      }
      break;

    case FSMState::RELOAD:
      fsm_state_ = has_target ? FSMState::TRACK : FSMState::SEARCH;
      break;

    default:
      fsm_state_ = FSMState::SEARCH;
      break;
  }
}
