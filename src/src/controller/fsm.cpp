#include "controller/fsm.hpp"

void FSM::update(const RobotState &state) {
  const double t = state.t;
  const double dt = state.dt;
  const bool detected = state.detected;
  const bool tracked = state.tracked;
  const bool process = state.process;
  
  // Handle based on the current state
  switch (fsm_state_) {
    case FSMState::SEARCH:
      // Target detected
      if (detected) fsm_state_ = FSMState::TRACK;
      break;

    case FSMState::TRACK:
      // Aim the target
      const double err_p = state.e.norm();
      if (err_p < config_.err_p_track) fsm_state_ = FSMState::AIM;
      


      // Failed to track or time is too old
      if (!detected || !tracked || dt > config_.max_time_gap) fsm_state_ = FSMState::SEARCH;
      break;

    case FSMState::AIM:
      // Failed to track
      if (!detected || !tracked) fsm_state_ = FSMState::SEARCH;
      
      // Failed to aim
      // if (!is_stable) state_ = FSMState::SEARCH;
      
      // AIM->RELOAD when shot is fired
      const double err_p = state.e.norm();
      if (err_p < config_.err_p_fire) fsm_state_ = FSMState::RELOAD;
      break;
      
    case FSMState::RELOAD:
      // Re-search or track when reload is done
      break;
    default:
      break;
  }
}
